// SPDX-License-Identifier: MIT
// Aurora speakerguardd v2 differential rig: offline host for macOS protection AudioUnits.
//
// Our own code. It instantiates the installed AudioDSP component through public AudioToolbox
// APIs, applies the machine's shipped preset, feeds deterministic audio (and V/I telemetry, from
// a file or from our own speaker plant model) and records, every quantum, every read-only meter
// parameter the unit publishes plus the rendered output. No Apple code is contained here.
//
// Usage:
//   rig_host --unit spp3|lsm2|atsp --strip X.austrip [--sidecar X.plist] [--spts X.bin]
//            [--prop ID=U32 ...] --in in.f32 --ch N [--q 512] [--sr 48000]
//            [--tele tele.f32 --tele-ch 12] [--plant plant.json] [--no-tele]
//            [--lsm2-tele]  # planar telemetry bus; requires explicit --tele-ch
//            [--param SCOPE.ELEMENT.ID=VALUE ...]
//            [--event FRAME:KIND[=ARG] ...] [--every K] --out PREFIX
//            [--preset-param ID=VALUE ...] [--param-at FRAME:ID=VALUE ...]
//            [--file-ch N] [--limit-frames N]
// --preset-param rewrites a stored record of the strip's preset before ClassInfo (the record
// must exist); --param-at sets a global parameter before the first render call that starts
// at FRAME; --file-ch: the input file has N channels and stream channel c reads file channel
// c mod N; --limit-frames renders at most N input frames.
// Event kinds, applied before the first render call that starts at or after FRAME:
//   reset          AudioUnitReset (global scope)
//   sleep=S        idle S wall-clock seconds
//   tsjump=N       offset later render timestamps by N frames
//   uninit[=P]     AudioUnitUninitialize then AudioUnitInitialize, same instance; P (if
//                  given) is set as property 64007 in between
//   classinfo      read kAudioUnitProperty_ClassInfo and set it back (SaveState round trip)
//   reinit         read ClassInfo, uninitialize, set it back, initialize
//   newinst[=P]    dispose the instance and build a new one in this process with the same
//                  configuration; P (if given) replaces property 64007 on the new
//                  instance (-1: do not set 64007 at all)
//   newinst_ci[=P] the same, and the old instance's ClassInfo is set on the new one
//                  before it is initialized
// in.f32 is interleaved float32 with N channels. Outputs: PREFIX.out.f32 (interleaved),
// PREFIX.state.csv, PREFIX.tele.f32 (telemetry actually fed), PREFIX.meta.json.
// Exit status: 0 only when every frame read was rendered. A refused --cfprop or a CF
// property file that does not parse is fatal (1); a render error or a short render
// exits 3 after writing the meta (render_error, frames, frames_read).
#import <AudioToolbox/AudioToolbox.h>
#import <Foundation/Foundation.h>
#include <dlfcn.h>
#include <math.h>

#define MAXCH 8
#define MAXQ 4096
#define MAXTELE 16
#define MAXEV 256
#define MAXPROP 32
#define MAXCF 16
#define MAXPL 8
#define MAXMETERS 4096
#define MAXPAR 32
#define MAXPRESET 128
#define MAXPARAT 32

static UInt32 fcc(const char *s) { return ((UInt32)s[0] << 24) | ((UInt32)s[1] << 16) | ((UInt32)s[2] << 8) | (UInt32)s[3]; }
static void ck(OSStatus s, const char *w) { if (s) { fprintf(stderr, "%s: %d\n", w, (int)s); } }
static void die(OSStatus s, const char *w) { if (s) { fprintf(stderr, "FATAL %s: %d\n", w, (int)s); exit(1); } }

static AudioUnit makeUnit(const char *st, UInt32 *version) {
    AudioComponentDescription d = {kAudioUnitType_Effect, fcc(st), kAudioUnitManufacturer_Apple, 0, 0};
    AudioComponent c = AudioComponentFindNext(NULL, &d);
    if (!c) {
        const char *fn = !strcmp(st, "spp3") ? "AUSpeakerProtectionV3Factory" :
                         !strcmp(st, "atsp") ? "AUThermalSpeakerProtectionFactory" :
                         !strcmp(st, "lsm2") ? "AULoudspeakerManagerV2Factory" : NULL;
        void *h = dlopen("/System/Library/Components/AudioDSP.component/Contents/MacOS/AudioDSP", RTLD_NOW | RTLD_LOCAL);
        AudioComponentFactoryFunction f = (h && fn) ? (AudioComponentFactoryFunction)dlsym(h, fn) : NULL;
        if (!f) { fprintf(stderr, "factory %s not found\n", fn ? fn : "?"); exit(1); }
        c = AudioComponentRegister(&d, CFSTR("aurora rig"), 0x10001, f);
    }
    AudioComponentGetVersion(c, version);
    AudioUnit au = NULL;
    die(AudioComponentInstanceNew(c, &au), "InstanceNew");
    return au;
}

// ---- our speaker plant model (used only to synthesise V/I telemetry) ----
typedef struct Plant {
    int enabled, delay;          // delay in frames between AU output and sense sample
    int order_iv;                // 1: [I,V] per speaker, 0: [V,I]
    int nspk;
    double sr;
    double volts_fs[MAXCH], vmon_scale[MAXCH], imon_scale[MAXCH];
    double re20[MAXCH], alpha[MAXCH], rshunt[MAXCH];
    double rth_vc[MAXCH], rth_mg[MAXCH], tau_vc[MAXCH], tau_mg[MAXCH];
    double t_amb, t_vc[MAXCH], t_mg[MAXCH];
    double fixed_temp[MAXCH];    // NAN: dynamic thermal plant; else hold coil at this °C
    double i_gain[MAXCH], v_gain[MAXCH]; // fault injection multipliers (0 = dead sense)
    float *ring; int ringlen, ringpos;
    // time schedule: from frame ev_frame[k], field ev_field[k] takes ev_val[k][c] (NAN = dynamic)
    int nev, nextev; long long nframe;
    long long ev_frame[MAXEV]; int ev_field[MAXEV]; double ev_val[MAXEV][MAXCH];
    double v_sat[MAXCH], i_sat[MAXCH];  // raw sense full scale (clips there; INFINITY = none)
} Plant;

// schedulable plant fields
static const char *kSchedField[] = {"i_gain", "v_gain", "fixed_temp"};
static double *plantField(struct Plant *p, int f);

static double jnum(NSDictionary *d, NSString *k, int c, double def) {
    id v = d[k];
    if ([v isKindOfClass:NSArray.class]) return c < (int)[v count] ? [v[c] doubleValue] : def;
    if ([v isKindOfClass:NSNumber.class]) return [v doubleValue];
    return def;
}

static void plantLoad(Plant *p, const char *path, int nspk, double sr) {
    NSDictionary *d = [NSJSONSerialization JSONObjectWithData:[NSData dataWithContentsOfFile:@(path)] options:0 error:nil];
    if (!d) { fprintf(stderr, "bad plant json\n"); exit(1); }
    // An unknown key is an error, never ignored: a misspelled or retired field
    // (the old "faults" list) would otherwise silently drop what it asked for.
    static const char *known[] = {"comment", "delay_frames", "order", "t_ambient", "volts_fs", "vmon_scale",
                                  "imon_scale", "re20", "alpha", "rshunt", "rth_vc", "rth_mg", "tau_vc",
                                  "tau_mg", "fixed_temp", "i_gain", "v_gain", "t_start", "schedule",
                                  "v_sat", "i_sat"};
    for (NSString *k in d) {
        int ok = 0;
        for (unsigned j = 0; j < sizeof known / sizeof *known; ++j) ok |= [k isEqual:@(known[j])];
        if (!ok) { fprintf(stderr, "plant: unknown key %s\n", k.UTF8String); exit(1); }
    }
    p->enabled = 1; p->nspk = nspk; p->sr = sr;
    p->delay = (int)jnum(d, @"delay_frames", 0, 512);
    p->order_iv = [d[@"order"] isEqual:@"VI"] ? 0 : 1;
    p->t_amb = jnum(d, @"t_ambient", 0, NAN);
    if (isnan(p->t_amb)) { fprintf(stderr, "plant: t_ambient missing\n"); exit(1); }
    // Every plant constant is required: the rig has no built-in speaker values.
    for (int c = 0; c < nspk; ++c) {
        static const char *req[] = {"volts_fs", "vmon_scale", "imon_scale", "re20", "alpha", "rth_vc",
                                    "rth_mg", "tau_vc", "tau_mg"};
        for (unsigned k = 0; k < sizeof req / sizeof *req; ++k)
            if (isnan(jnum(d, @(req[k]), c, NAN))) { fprintf(stderr, "plant: %s missing\n", req[k]); exit(1); }
        p->volts_fs[c] = jnum(d, @"volts_fs", c, NAN);
        p->vmon_scale[c] = jnum(d, @"vmon_scale", c, NAN);
        p->imon_scale[c] = jnum(d, @"imon_scale", c, NAN);
        p->re20[c] = jnum(d, @"re20", c, NAN);
        p->alpha[c] = jnum(d, @"alpha", c, NAN);
        p->rshunt[c] = jnum(d, @"rshunt", c, 0.0);
        p->rth_vc[c] = jnum(d, @"rth_vc", c, NAN); p->rth_mg[c] = jnum(d, @"rth_mg", c, NAN);
        p->tau_vc[c] = jnum(d, @"tau_vc", c, NAN); p->tau_mg[c] = jnum(d, @"tau_mg", c, NAN);
        p->fixed_temp[c] = jnum(d, @"fixed_temp", c, NAN);
        p->i_gain[c] = jnum(d, @"i_gain", c, 1); p->v_gain[c] = jnum(d, @"v_gain", c, 1);
        p->t_vc[c] = p->t_mg[c] = jnum(d, @"t_start", c, p->t_amb);
        // the sense converters' own full scale: the raw sample clips there
        p->v_sat[c] = jnum(d, @"v_sat", c, INFINITY);
        p->i_sat[c] = jnum(d, @"i_sat", c, INFINITY);
    }
    // "schedule": [{"t": seconds, "i_gain": x | [x per speaker], "v_gain": ..., "fixed_temp": T | null}, ...]
    // A listed field takes the new value from that time on (fault injection and sense steps
    // during a run); null for fixed_temp returns the coil to the dynamic plant.
    NSArray *sch = d[@"schedule"];
    if (sch && ![sch isKindOfClass:NSArray.class]) { fprintf(stderr, "plant: schedule must be a list\n"); exit(1); }
    for (NSDictionary *e in sch) {
        if (![e isKindOfClass:NSDictionary.class] || ![e[@"t"] isKindOfClass:NSNumber.class]) {
            fprintf(stderr, "plant: schedule entry needs t\n"); exit(1);
        }
        long long fr = llround([e[@"t"] doubleValue] * sr);
        for (NSString *k in e) {
            if ([k isEqual:@"t"]) continue;
            int f = -1;
            for (int j = 0; j < (int)(sizeof kSchedField / sizeof *kSchedField); ++j)
                if ([k isEqual:@(kSchedField[j])]) f = j;
            if (f < 0) { fprintf(stderr, "plant: schedule field %s not schedulable\n", k.UTF8String); exit(1); }
            if (p->nev >= MAXEV) { fprintf(stderr, "plant: too many schedule events\n"); exit(1); }
            int k2 = p->nev++;
            p->ev_frame[k2] = fr; p->ev_field[k2] = f;
            for (int c = 0; c < nspk; ++c)
                p->ev_val[k2][c] = [e[k] isKindOfClass:NSNull.class] ? NAN : jnum(e, k, c, NAN);
        }
    }
    // stable sort by frame (events at the same time apply in file order)
    for (int a = 1; a < p->nev; ++a)
        for (int b = a; b > 0 && p->ev_frame[b - 1] > p->ev_frame[b]; --b) {
            long long tf = p->ev_frame[b]; p->ev_frame[b] = p->ev_frame[b - 1]; p->ev_frame[b - 1] = tf;
            int ti = p->ev_field[b]; p->ev_field[b] = p->ev_field[b - 1]; p->ev_field[b - 1] = ti;
            double tv[MAXCH]; memcpy(tv, p->ev_val[b], sizeof tv);
            memcpy(p->ev_val[b], p->ev_val[b - 1], sizeof tv); memcpy(p->ev_val[b - 1], tv, sizeof tv);
        }
    p->ringlen = p->delay + MAXQ;
    p->ring = calloc((size_t)p->ringlen * nspk, sizeof(float));
}

static double *plantField(Plant *p, int f) {
    return f == 0 ? p->i_gain : f == 1 ? p->v_gain : p->fixed_temp;
}

// Produce telemetry for n frames from what the AU output `delay` frames ago; then push this
// quantum's output into the ring.
static void plantStep(Plant *p, float *tele, int telech, float out[MAXCH][MAXQ], int n) {
    for (int i = 0; i < n; ++i, ++p->nframe) {
        while (p->nextev < p->nev && p->ev_frame[p->nextev] <= p->nframe) {
            int k = p->nextev++;
            double *f = plantField(p, p->ev_field[k]);
            for (int c = 0; c < p->nspk; ++c) f[c] = p->ev_val[k][c];
        }
        int rp = (p->ringpos + i) % p->ringlen;
        for (int c = 0; c < p->nspk; ++c) {
            double s = p->ring[rp * p->nspk + c];
            double tc = isnan(p->fixed_temp[c]) ? p->t_vc[c] : p->fixed_temp[c];
            double re = p->re20[c] * (1 + p->alpha[c] * (tc - 20)) + p->rshunt[c];
            double v = s * p->volts_fs[c], cur = v / re;
            // two-node thermal plant (forward Euler at the sample rate)
            if (isnan(p->fixed_temp[c])) {
                double pw = v * v / re, dt = 1.0 / p->sr;
                double tvc = p->t_vc[c], tmg = p->t_mg[c];
                p->t_vc[c] += dt * ((pw * p->rth_vc[c] - (tvc - tmg)) / p->tau_vc[c]);
                p->t_mg[c] += dt * ((pw * p->rth_mg[c] - (tmg - p->t_amb)) / p->tau_mg[c]);
            }
            double ir = p->i_gain[c] * cur / p->imon_scale[c], vr = p->v_gain[c] * v / p->vmon_scale[c];
            float iv = (float)fmax(-p->i_sat[c], fmin(p->i_sat[c], ir));
            float vv = (float)fmax(-p->v_sat[c], fmin(p->v_sat[c], vr));
            tele[i * telech + 2 * c + (p->order_iv ? 0 : 1)] = iv;
            tele[i * telech + 2 * c + (p->order_iv ? 1 : 0)] = vv;
        }
    }
    // write current output so it is read back `delay` frames later
    for (int i = 0; i < n; ++i) {
        int wp = (p->ringpos + p->delay + i) % p->ringlen;
        for (int c = 0; c < p->nspk; ++c) p->ring[wp * p->nspk + c] = out[c][i];
    }
    p->ringpos = (p->ringpos + n) % p->ringlen;
}

// ---- host buffers ----
typedef struct {
    float audio[MAXCH][MAXQ], tele[MAXQ * MAXTELE], telePlanar[MAXTELE][MAXQ];
    int nch, telech, planarTele;
    UInt32 teleAvailable, telePulls; int teleInvalid; Float64 teleSampleTime;
    UInt64 teleCallbacks, teleFrames, teleRejected;
} Bufs;

// ---- unit configuration (also used to rebuild the unit for newinst events) ----
typedef struct Config {
    const char *unit, *strip, *sidecar, *spts; int nch, telech, notele, lsm2tele, q, maxframes; double sr;
    UInt32 propIds[MAXPROP], propVals[MAXPROP]; int nprops;
    UInt32 cfIds[MAXCF]; const char *cfPaths[MAXCF]; int ncf;
    UInt32 plIds[MAXPL]; const char *plPaths[MAXPL]; int npl;
    UInt32 parScope[MAXPAR], parEl[MAXPAR], parId[MAXPAR]; float parVal[MAXPAR]; int npar;
    UInt32 presetId[MAXPRESET]; float presetVal[MAXPRESET]; int npreset;
    long long prop64007; // >= -1: replaces any --prop 64007 (-1 omits it); < -1: as given
    int haveTeleBus;
} Config;

static Bufs B;

static OSStatus cbAudio(void *r, AudioUnitRenderActionFlags *f, const AudioTimeStamp *t, UInt32 bus, UInt32 n, AudioBufferList *io) {
    for (UInt32 c = 0; c < io->mNumberBuffers; ++c) {
        io->mBuffers[c].mData = B.audio[c]; io->mBuffers[c].mDataByteSize = n * sizeof(float);
    }
    return noErr;
}
static OSStatus cbTele(void *r, AudioUnitRenderActionFlags *f, const AudioTimeStamp *t, UInt32 bus, UInt32 n, AudioBufferList *io) {
    if (B.planarTele) {
        // The LSM2 telemetry bus is planar. Keep the recorded/file format
        // interleaved, and expose exactly this render's samples to the AU.
        // A second or partial pull is not silently served from offset zero.
        if (!io || !t || !(t->mFlags & kAudioTimeStampSampleTimeValid) || t->mSampleTime != B.teleSampleTime ||
            n != B.teleAvailable || n > MAXQ || B.telePulls || io->mNumberBuffers != (UInt32)B.telech) {
            B.teleInvalid = 1; B.teleRejected++;
            return kAudio_ParamError;
        }
        for (int c = 0; c < B.telech; ++c) {
            for (UInt32 i = 0; i < n; ++i) B.telePlanar[c][i] = B.tele[i * B.telech + c];
            io->mBuffers[c].mData = B.telePlanar[c];
            io->mBuffers[c].mDataByteSize = n * sizeof(float);
            io->mBuffers[c].mNumberChannels = 1;
        }
    } else {
        io->mBuffers[0].mData = B.tele; io->mBuffers[0].mDataByteSize = n * B.telech * sizeof(float);
        io->mBuffers[0].mNumberChannels = B.telech;
    }
    B.telePulls++; B.teleCallbacks++; B.teleFrames += n;
    return noErr;
}

static AudioStreamBasicDescription fmt(double sr, UInt32 ch, bool ni) {
    AudioStreamBasicDescription f = {0};
    f.mSampleRate = sr; f.mFormatID = kAudioFormatLinearPCM;
    f.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked | (ni ? kAudioFormatFlagIsNonInterleaved : 0);
    f.mBytesPerPacket = f.mBytesPerFrame = sizeof(float) * (ni ? 1 : ch);
    f.mFramesPerPacket = 1; f.mChannelsPerFrame = ch; f.mBitsPerChannel = 32;
    return f;
}

// Strict telemetry must remain configured across ClassInfo and lifecycle changes.
static OSStatus checkLsm2Tele(AudioUnit au, const Config *cf) {
    UInt32 inputs = 0, size = sizeof inputs;
    OSStatus r = AudioUnitGetProperty(au, kAudioUnitProperty_ElementCount, kAudioUnitScope_Input, 0, &inputs, &size);
    if (r) return r;
    if (size != sizeof inputs || inputs != 2) return kAudio_ParamError;
    AudioStreamBasicDescription expected = fmt(cf->sr, cf->telech, true), got = {0}; size = sizeof got;
    r = AudioUnitGetProperty(au, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 1, &got, &size);
    if (r) return r;
    if (size != sizeof got || got.mSampleRate != expected.mSampleRate ||
        got.mChannelsPerFrame != expected.mChannelsPerFrame || got.mFormatID != expected.mFormatID ||
        got.mFormatFlags != expected.mFormatFlags || got.mBitsPerChannel != expected.mBitsPerChannel ||
        got.mBytesPerFrame != expected.mBytesPerFrame || got.mBytesPerPacket != expected.mBytesPerPacket ||
        got.mFramesPerPacket != expected.mFramesPerPacket) return kAudio_ParamError;
    return noErr;
}

// Apply the whole configuration and initialize. classInfo, when given, is set after the
// configuration and before AudioUnitInitialize (a restored SaveState).
static void configureUnit(AudioUnit au, Config *cf, CFPropertyListRef classInfo) {
    const char *unit = cf->unit, *strip = cf->strip, *sidecar = cf->sidecar, *spts = cf->spts;
    int q = cf->q, maxframes = cf->maxframes, nch = cf->nch, telech = cf->telech, notele = cf->notele;
    double sr = cf->sr;
        // preset from the strip
        NSDictionary *sd = [NSDictionary dictionaryWithContentsOfFile:@(strip)];
        NSDictionary *preset = nil;
        for (NSDictionary *s in sd[@"strips"]) for (NSDictionary *e in s[@"effects"])
            if ([e[@"unit"][@"subtype"] unsignedIntValue] == fcc(unit)) preset = e[@"aupreset"];
        if (!preset) { fprintf(stderr, "no %s preset in strip\n", unit); exit(1); }
        if (cf->npreset) {
            NSMutableDictionary *mp = [preset mutableCopy];
            NSMutableData *pd = [mp[@"data"] mutableCopy];
            if (!pd || pd.length < 12) { fprintf(stderr, "preset has no parameter data\n"); exit(1); }
            uint8_t *bytes = pd.mutableBytes;
            UInt32 count = CFSwapInt32BigToHost(*(UInt32 *)(bytes + 8));
            if (pd.length != 12 + 8 * (NSUInteger)count) { fprintf(stderr, "unsupported preset parameter data\n"); exit(1); }
            for (int k = 0; k < cf->npreset; ++k) {
                bool found = false;
                for (UInt32 i = 0; i < count && !found; ++i) {
                    if (CFSwapInt32BigToHost(*(UInt32 *)(bytes + 12 + 8 * i)) != cf->presetId[k]) continue;
                    union { float f; UInt32 u; } val = {.f = cf->presetVal[k]};
                    *(UInt32 *)(bytes + 16 + 8 * i) = CFSwapInt32HostToBig(val.u);
                    found = true;
                }
                if (!found) { fprintf(stderr, "FATAL preset parameter %u not found\n", cf->presetId[k]); exit(1); }
            }
            mp[@"data"] = pd;
            preset = mp;
        }
        CFPropertyListRef ci = (__bridge CFPropertyListRef)preset;
        die(AudioUnitSetProperty(au, kAudioUnitProperty_ClassInfo, 0, 0, &ci, sizeof ci), "ClassInfo");
        for (int k = 0; k < cf->npreset; ++k) {
            AudioUnitParameterValue rb = NAN;
            OSStatus r = AudioUnitGetParameter(au, cf->presetId[k], kAudioUnitScope_Global, 0, &rb);
            fprintf(stderr, "preset-param %u=%.9g readback %.9g : %d\n", cf->presetId[k], cf->presetVal[k], rb, (int)r);
        }
        if (sidecar) {
            CFDictionaryRef sc = (__bridge CFDictionaryRef)[NSDictionary dictionaryWithContentsOfFile:@(sidecar)];
            die(AudioUnitSetProperty(au, 64003, 0, 0, &sc, sizeof sc), "sidecar 64003");
        }
        for (int i = 0; i < cf->nprops; ++i) {
            if (cf->propIds[i] == 64007 && cf->prop64007 >= -1) continue;
            ck(AudioUnitSetProperty(au, cf->propIds[i], 0, 0, &cf->propVals[i], 4), "prop");
        }
        if (cf->prop64007 >= 0) { UInt32 v = (UInt32)cf->prop64007; ck(AudioUnitSetProperty(au, 64007, 0, 0, &v, 4), "prop 64007"); }
        for (int i = 0; i < cf->npl; ++i) { // a propstrip "Path" property: the plist's contents as a CF dictionary
            CFDictionaryRef pd = (__bridge CFDictionaryRef)[NSDictionary dictionaryWithContentsOfFile:@(cf->plPaths[i])];
            OSStatus r = AudioUnitSetProperty(au, cf->plIds[i], 0, 0, &pd, sizeof pd);
            fprintf(stderr, "plistprop %u <- %s : %d\n", cf->plIds[i], cf->plPaths[i], (int)r);
        }
        for (int i = 0; i < cf->ncf; ++i) {
            // JSON -> CF property list; strings of the form "fcc:XXXX" become 4CC UInt32 numbers
            NSData *raw = [NSData dataWithContentsOfFile:@(cf->cfPaths[i])];
            id obj = raw ? [NSJSONSerialization JSONObjectWithData:raw options:0 error:nil] : nil;
            if (!obj) { fprintf(stderr, "FATAL cfprop %u: %s is missing or not JSON\n", cf->cfIds[i], cf->cfPaths[i]); exit(1); }
            id (^__block conv)(id) = nil;
            id (^fix)(id) = ^id(id o) {
                if ([o isKindOfClass:NSString.class] && [o hasPrefix:@"fcc:"] && [o length] == 8) return @(fcc([[o substringFromIndex:4] UTF8String]));
                if ([o isKindOfClass:NSArray.class]) { NSMutableArray *r = [NSMutableArray array]; for (id x in o) [r addObject:conv(x)]; return r; }
                if ([o isKindOfClass:NSDictionary.class]) { NSMutableDictionary *r = [NSMutableDictionary dictionary]; for (id k in o) r[k] = conv(o[k]); return r; }
                return o;
            };
            conv = fix;
            obj = fix(obj);
            CFTypeRef cfo = (__bridge CFTypeRef)obj;
            OSStatus r = AudioUnitSetProperty(au, cf->cfIds[i], 0, 0, &cfo, sizeof cfo);
            fprintf(stderr, "cfprop %u <- %s : %d\n", cf->cfIds[i], cf->cfPaths[i], (int)r);
            die(r, "cfprop"); // a refused run-time property would give a reference that only looks valid
        }
        if (spts) {
            CFDataRef d = (__bridge CFDataRef)[NSData dataWithContentsOfFile:@(spts)];
            ck(AudioUnitSetProperty(au, 64009, 0, 0, &d, sizeof d), "SpTS 64009");
        }
        UInt32 mx = maxframes > q ? (UInt32)maxframes : (UInt32)q;
        die(AudioUnitSetProperty(au, kAudioUnitProperty_MaximumFramesPerSlice, 0, 0, &mx, 4), "maxframes");
        if (cf->lsm2tele) {
            UInt32 bytes = 0, inputs = 0, size = sizeof inputs; Boolean writable = false;
            die(AudioUnitGetPropertyInfo(au, kAudioUnitProperty_ElementCount, kAudioUnitScope_Input, 0,
                                        &bytes, &writable), "LSM2 input count info");
            die(AudioUnitGetProperty(au, kAudioUnitProperty_ElementCount, kAudioUnitScope_Input, 0,
                                    &inputs, &size), "LSM2 input count");
            if (bytes != sizeof inputs || size != sizeof inputs || (inputs != 2 && !writable)) {
                fprintf(stderr, "FATAL LSM2 input bus count is not configurable\n"); exit(1);
            }
            if (inputs != 2) {
                inputs = 2;
                die(AudioUnitSetProperty(au, kAudioUnitProperty_ElementCount, kAudioUnitScope_Input, 0,
                                        &inputs, sizeof inputs), "LSM2 input count set");
            }
            inputs = 0; size = sizeof inputs;
            die(AudioUnitGetProperty(au, kAudioUnitProperty_ElementCount, kAudioUnitScope_Input, 0,
                                    &inputs, &size), "LSM2 input count readback");
            if (inputs != 2 || size != sizeof inputs) {
                fprintf(stderr, "FATAL LSM2 input bus count readback is not 2\n"); exit(1);
            }
        }
        AudioStreamBasicDescription af = fmt(sr, nch, true), tf = fmt(sr, telech, cf->lsm2tele);
        die(AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &af, sizeof af), "in fmt");
        die(AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output, 0, &af, sizeof af), "out fmt");
        UInt32 nin = 0, sz = 4;
        OSStatus countStatus = AudioUnitGetProperty(au, kAudioUnitProperty_ElementCount, kAudioUnitScope_Input, 0, &nin, &sz);
        if (cf->lsm2tele) die(countStatus, "LSM2 configured input count");
        int haveTeleBus = nin > 1 && !notele;
        cf->haveTeleBus = haveTeleBus;
        AURenderCallbackStruct ca = {cbAudio, NULL}, ct = {cbTele, NULL};
        die(AudioUnitSetProperty(au, kAudioUnitProperty_SetRenderCallback, kAudioUnitScope_Input, 0, &ca, sizeof ca), "cb0");
        if (haveTeleBus) {
            die(AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 1, &tf, sizeof tf), "tele fmt");
            if (cf->lsm2tele) {
                AudioStreamBasicDescription got = {0}; UInt32 size = sizeof got;
                die(AudioUnitGetProperty(au, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 1,
                                        &got, &size), "LSM2 tele format readback");
                if (size != sizeof got || got.mSampleRate != tf.mSampleRate ||
                    got.mChannelsPerFrame != tf.mChannelsPerFrame || got.mFormatID != tf.mFormatID ||
                    got.mFormatFlags != tf.mFormatFlags || got.mBitsPerChannel != tf.mBitsPerChannel ||
                    got.mBytesPerFrame != tf.mBytesPerFrame || got.mBytesPerPacket != tf.mBytesPerPacket ||
                    got.mFramesPerPacket != tf.mFramesPerPacket) {
                    fprintf(stderr, "FATAL LSM2 tele format readback differs\n"); exit(1);
                }
            }
            die(AudioUnitSetProperty(au, kAudioUnitProperty_SetRenderCallback, kAudioUnitScope_Input, 1, &ct, sizeof ct), "cb1");
        }
        for (int i = 0; i < cf->npar; ++i)
            die(AudioUnitSetParameter(au, cf->parId[i], cf->parScope[i], cf->parEl[i], cf->parVal[i], 0), "param");
        if (classInfo) {
            OSStatus r = AudioUnitSetProperty(au, kAudioUnitProperty_ClassInfo, 0, 0, &classInfo, sizeof classInfo);
            if (cf->lsm2tele) die(r, "LSM2 restore ClassInfo"); else ck(r, "restore ClassInfo");
        }
        die(AudioUnitInitialize(au), "Initialize");
        for (int i = 0; i < cf->npar; ++i) // again after Initialize, in case setup rebuilt them
            die(AudioUnitSetParameter(au, cf->parId[i], cf->parScope[i], cf->parEl[i], cf->parVal[i], 0), "param");
        if (cf->lsm2tele) die(checkLsm2Tele(au, cf), "LSM2 initialized telemetry readback");
}

typedef struct { AudioUnitScope sc; AudioUnitElement el; AudioUnitParameterID id; char name[128]; } Meter;

// Every parameter (writable or not) of every element in global and part scope, as
// "params SCOPE.ELEMENT.ID name = value" lines on f (--dump-params; again after events).
static void dumpParams(AudioUnit au, FILE *f, const char *tag) {
    for (AudioUnitScope sc = 0; sc <= 4; sc += 4) {
        UInt32 lsz = 0;
        if (AudioUnitGetPropertyInfo(au, kAudioUnitProperty_ParameterList, sc, 0, &lsz, NULL) || !lsz) continue;
        AudioUnitParameterID *ids = malloc(lsz);
        AudioUnitGetProperty(au, kAudioUnitProperty_ParameterList, sc, 0, ids, &lsz);
        UInt32 nel = 1, esz = 4;
        if (sc != 0) AudioUnitGetProperty(au, kAudioUnitProperty_ElementCount, sc, 0, &nel, &esz);
        for (UInt32 e = 0; e < nel; ++e)
            for (UInt32 k = 0; k < lsz / sizeof *ids; ++k) {
                AudioUnitParameterInfo pi = {0}; UInt32 ps = sizeof pi;
                if (AudioUnitGetProperty(au, kAudioUnitProperty_ParameterInfo, sc, ids[k], &pi, &ps)) continue;
                char nm0[100] = {0};
                if (pi.cfNameString) CFStringGetCString(pi.cfNameString, nm0, sizeof nm0, kCFStringEncodingUTF8);
                AudioUnitParameterValue v = NAN;
                AudioUnitGetParameter(au, ids[k], sc, e, &v);
                fprintf(f, "params[%s] %u.%u.%u %s = %.9g\n", tag, sc, e, ids[k], nm0, v);
            }
        free(ids);
    }
}

int main(int argc, const char **argv) {
    int rc = 0;
    if (geteuid() == 0) { fprintf(stderr, "refusing to run as root: a missed log redirect would write system directories\n"); return 2; }
    @autoreleasepool {
        const char *unit = "spp3", *strip = NULL, *sidecar = NULL, *spts = NULL, *in = NULL, *tele = NULL,
                   *plant = NULL, *out = NULL;
        int nch = 0, q = 512, telech = 12, telechExplicit = 0, every = 1, notele = 0, lsm2tele = 0, filech = 0;
        long long limitframes = 0;
        UInt64 patFrame[MAXPARAT]; UInt32 patId[MAXPARAT]; float patVal[MAXPARAT]; int npat = 0;
        long long failAt = -1; // --test-fail-at FRAME: the rig's own tests; simulates a render error there
        double pace = 0; int maxframes = 0; // >0: render at most pace x real time (lets the units' async log writers keep up)
        double sr = 48000;
        static Config C; C.prop64007 = -2;
        UInt32 *propIds = C.propIds, *propVals = C.propVals; int nprops = 0;
        UInt32 *cfIds = C.cfIds; const char **cfPaths = C.cfPaths; int ncf = 0;
        UInt32 *plIds = C.plIds; const char **plPaths = C.plPaths; int npl = 0;
        int npar = 0, dumpp = 0;
        // --event FRAME:KIND[=ARG] (see the usage above), applied before the first render call
        // that starts at or after FRAME
        long long evFrame[16]; char evKind[16][16]; double evArg[16]; int nevt = 0, nextevt = 0;
        Float64 tsOffset = 0;
        for (int i = 1; i < argc; ++i) {
            const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
#define ARG(n) (!strcmp(a, n) && v && ++i)
            if (ARG("--unit")) unit = v; else if (ARG("--strip")) strip = v; else if (ARG("--sidecar")) sidecar = v;
            else if (ARG("--spts")) spts = v; else if (ARG("--in")) in = v; else if (ARG("--ch")) nch = atoi(v);
            else if (ARG("--q")) q = atoi(v); else if (ARG("--sr")) sr = atof(v); else if (ARG("--tele")) tele = v;
            else if (ARG("--tele-ch")) { telech = atoi(v); telechExplicit = 1; } else if (ARG("--plant")) plant = v;
            else if (ARG("--every")) every = atoi(v); else if (ARG("--pace")) pace = atof(v); else if (ARG("--maxframes")) maxframes = atoi(v); else if (ARG("--out")) out = v;
            else if (ARG("--prop")) {
                if (nprops >= MAXPROP) { fprintf(stderr, "too many --prop (max %d)\n", MAXPROP); return 2; }
                if (sscanf(v, "%u=%u", &propIds[nprops], &propVals[nprops]) != 2) { fprintf(stderr, "bad --prop %s\n", v); return 2; }
                nprops++;
            }
            else if (ARG("--plistprop")) {
                const char *eq = strchr(v, '=');
                if (npl >= MAXPL) { fprintf(stderr, "too many --plistprop (max %d)\n", MAXPL); return 2; }
                if (!eq || eq == v || !eq[1]) { fprintf(stderr, "bad --plistprop %s\n", v); return 2; }
                plIds[npl] = (UInt32)atoi(v); plPaths[npl++] = eq + 1;
            }
            else if (ARG("--cfprop")) {
                const char *eq = strchr(v, '=');
                if (ncf >= MAXCF) { fprintf(stderr, "too many --cfprop (max %d)\n", MAXCF); return 2; }
                if (!eq || eq == v || !eq[1]) { fprintf(stderr, "bad --cfprop %s\n", v); return 2; }
                cfIds[ncf] = (UInt32)atoi(v); cfPaths[ncf++] = eq + 1;
            }
            else if (ARG("--param")) {
                // SCOPE.ELEMENT.ID=VALUE, set with AudioUnitSetParameter before and after Initialize
                if (npar >= MAXPAR) { fprintf(stderr, "too many --param (max %d)\n", MAXPAR); return 2; }
                if (sscanf(v, "%u.%u.%u=%f", &C.parScope[npar], &C.parEl[npar], &C.parId[npar], &C.parVal[npar]) != 4) {
                    fprintf(stderr, "bad --param %s\n", v); return 2;
                }
                npar++;
            }
            else if (ARG("--preset-param")) {
                if (C.npreset >= MAXPRESET || sscanf(v, "%u=%f", &C.presetId[C.npreset], &C.presetVal[C.npreset]) != 2) {
                    fprintf(stderr, "bad --preset-param %s\n", v); return 2;
                }
                C.npreset++;
            }
            else if (ARG("--param-at")) {
                unsigned long long fr = 0;
                if (npat >= MAXPARAT || sscanf(v, "%llu:%u=%f", &fr, &patId[npat], &patVal[npat]) != 3) {
                    fprintf(stderr, "bad --param-at %s\n", v); return 2;
                }
                if (npat && fr < patFrame[npat - 1]) { fprintf(stderr, "--param-at frames must ascend\n"); return 2; }
                patFrame[npat++] = fr;
            }
            else if (ARG("--file-ch")) filech = atoi(v);
            else if (ARG("--limit-frames")) limitframes = atoll(v);
            else if (ARG("--event")) {
                if (nevt >= 16) { fprintf(stderr, "too many --event\n"); return 2; }
                char kind[16] = {0}; double arg = NAN; long long fr = 0;
                if (sscanf(v, "%lld:%15[a-z_]=%lf", &fr, kind, &arg) < 2) { fprintf(stderr, "bad --event %s\n", v); return 2; }
                if (strcmp(kind, "reset") && strcmp(kind, "sleep") && strcmp(kind, "tsjump") && strcmp(kind, "uninit") &&
                    strcmp(kind, "classinfo") && strcmp(kind, "reinit") && strcmp(kind, "newinst") && strcmp(kind, "newinst_ci")) {
                    fprintf(stderr, "bad --event kind %s\n", kind); return 2;
                }
                if ((!strcmp(kind, "sleep") || !strcmp(kind, "tsjump")) && isnan(arg)) { fprintf(stderr, "--event %s needs =ARG\n", kind); return 2; }
                if (nevt && fr < evFrame[nevt - 1]) { fprintf(stderr, "--event frames must ascend\n"); return 2; }
                evFrame[nevt] = fr; strcpy(evKind[nevt], kind); evArg[nevt] = arg; nevt++;
            }
            else if (ARG("--test-fail-at")) failAt = atoll(v);
            else if (!strcmp(a, "--no-tele")) notele = 1;
            else if (!strcmp(a, "--lsm2-tele")) lsm2tele = 1;
            else if (!strcmp(a, "--dump-params")) dumpp = 1;
            else { fprintf(stderr, "bad arg %s\n", a); return 2; }
        }
        if (!strip || !in || !nch || !out || q > MAXQ || nch > MAXCH) { fprintf(stderr, "missing args\n"); return 2; }
        if (!filech) filech = nch;
        if (filech < 1 || filech > MAXCH || limitframes < 0) { fprintf(stderr, "bad --file-ch or --limit-frames\n"); return 2; }
        if (q < 1 || nch < 1 || every < 1) { fprintf(stderr, "--q, --ch and --every must be positive\n"); return 2; }
        if (telech < 1 || telech > MAXTELE) { fprintf(stderr, "--tele-ch must be 1..%d\n", MAXTELE); return 2; }
        if (lsm2tele && (strcmp(unit, "lsm2") || notele || plant || !telechExplicit || !isfinite(sr) || sr <= 0)) {
            fprintf(stderr, "--lsm2-tele needs lsm2, positive finite --sr, explicit --tele-ch, and no --no-tele or --plant\n"); return 2;
        }
        B.nch = nch; B.telech = telech; B.planarTele = lsm2tele;
        C.unit = unit; C.strip = strip; C.sidecar = sidecar; C.spts = spts; C.nch = nch; C.telech = telech;
        C.notele = notele; C.lsm2tele = lsm2tele; C.q = q; C.maxframes = maxframes; C.sr = sr;
        C.nprops = nprops; C.ncf = ncf; C.npl = npl; C.npar = npar;
        UInt32 compVersion = 0;
        AudioUnit au = makeUnit(unit, &compVersion);

        configureUnit(au, &C, NULL);
        int haveTeleBus = C.haveTeleBus;
        if (dumpp) dumpParams(au, stderr, "init");
        Float64 lat = 0; UInt32 sz = sizeof lat;
        AudioUnitGetProperty(au, kAudioUnitProperty_Latency, 0, 0, &lat, &sz);

        // meters: every non-writable parameter in global scope and in each Part element
        Meter *m = calloc(MAXMETERS, sizeof *m); int nm = 0;
        for (AudioUnitScope sc = 0; sc <= 4; ++sc) {
            UInt32 lsz = 0;
            if (AudioUnitGetPropertyInfo(au, kAudioUnitProperty_ParameterList, sc, 0, &lsz, NULL) || !lsz) continue;
            AudioUnitParameterID *ids = malloc(lsz);
            AudioUnitGetProperty(au, kAudioUnitProperty_ParameterList, sc, 0, ids, &lsz);
            UInt32 nel = 1, esz = 4;
            if (sc != 0) AudioUnitGetProperty(au, kAudioUnitProperty_ElementCount, sc, 0, &nel, &esz);
            for (UInt32 k = 0; k < lsz / sizeof *ids; ++k) {
                AudioUnitParameterInfo pi = {0}; UInt32 ps = sizeof pi;
                if (AudioUnitGetProperty(au, kAudioUnitProperty_ParameterInfo, sc, ids[k], &pi, &ps)) continue;
                if (pi.flags & kAudioUnitParameterFlag_IsWritable) continue;
                char nm0[100] = {0};
                if (pi.cfNameString) CFStringGetCString(pi.cfNameString, nm0, sizeof nm0, kCFStringEncodingUTF8);
                for (UInt32 e = 0; e < nel; ++e) {
                    if (nm >= MAXMETERS) { fprintf(stderr, "FATAL more than %d meters\n", MAXMETERS); exit(1); }
                    m[nm].sc = sc; m[nm].el = e; m[nm].id = ids[k];
                    snprintf(m[nm].name, sizeof m[nm].name, "s%u.e%u.p%u:%s", sc, e, ids[k], nm0);
                    for (char *c = m[nm].name; *c; ++c) if (*c == ',' || *c == ' ') *c = '_';
                    nm++;
                }
            }
            free(ids);
        }

        Plant P = {0};
        if (plant && haveTeleBus) plantLoad(&P, plant, nch, sr);
        FILE *fin = fopen(in, "rb"), *ftele = tele ? fopen(tele, "rb") : NULL;
        if (!fin || (tele && !ftele)) { fprintf(stderr, "cannot open input\n"); return 1; }
        char path[1024];
        snprintf(path, sizeof path, "%s.out.f32", out); FILE *fo = fopen(path, "wb");
        snprintf(path, sizeof path, "%s.state.csv", out); FILE *fs = fopen(path, "w");
        if (!fo || !fs) { fprintf(stderr, "cannot open outputs at %s\n", out); return 1; }
        snprintf(path, sizeof path, "%s.tele.f32", out); FILE *ft = haveTeleBus ? fopen(path, "wb") : NULL;
        fprintf(fs, "frame");
        for (int i = 0; i < nm; ++i) fprintf(fs, ",%s", m[i].name);
        fprintf(fs, "\n");

        AudioBufferList *abl = calloc(1, offsetof(AudioBufferList, mBuffers) + MAXCH * sizeof(AudioBuffer));
        static float outb[MAXCH][MAXQ], inter[MAXQ * MAXCH];
        int nextpat = 0;
        abl->mNumberBuffers = nch;
        UInt64 done = 0, nread = 0; long quanta = 0;
        int failed = 0;
        for (;;) {
            while (nextpat < npat && done >= patFrame[nextpat]) {
                int k = nextpat++;
                OSStatus r = AudioUnitSetParameter(au, patId[k], kAudioUnitScope_Global, 0, patVal[k], 0);
                fprintf(stderr, "param-at %llu %u=%.9g : %d\n", done, patId[k], patVal[k], (int)r);
                die(r, "param-at");
            }
            size_t want = (size_t)q;
            if (limitframes > 0) want = (size_t)MIN((long long)q, MAX(0LL, limitframes - (long long)nread));
            if (!want) break;
            size_t got = fread(inter, sizeof(float) * filech, want, fin);
            if (!got) break;
            nread += got;
            for (size_t i = 0; i < got; ++i) for (int c = 0; c < nch; ++c) B.audio[c][i] = inter[i * filech + c % filech];
            if (haveTeleBus) {
                if (ftele) {
                    size_t g2 = fread(B.tele, sizeof(float) * telech, got, ftele);
                    if (g2 < got && lsm2tele) {
                        fprintf(stderr, "short LSM2 telemetry at %llu: got %zu of %zu frames\n", done, g2, got);
                        failed = -1; break;
                    }
                    if (g2 < got) memset(B.tele + g2 * telech, 0, (got - g2) * telech * sizeof(float));
                } else if (P.enabled) {
                    plantStep(&P, B.tele, telech, outb, (int)got); // uses the previous quantum's output
                } else memset(B.tele, 0, got * telech * sizeof(float));
                fwrite(B.tele, sizeof(float) * telech, got, ft);
            }
            for (int c = 0; c < nch; ++c) {
                abl->mBuffers[c].mNumberChannels = 1; abl->mBuffers[c].mData = outb[c];
                abl->mBuffers[c].mDataByteSize = (UInt32)got * sizeof(float);
            }
            while (nextevt < nevt && (long long)done >= evFrame[nextevt]) {
                int k = nextevt++;
                if (!strcmp(evKind[k], "reset")) {
                    OSStatus r = AudioUnitReset(au, kAudioUnitScope_Global, 0);
                    fprintf(stderr, "event reset at %llu: %d\n", done, (int)r);
                    if (lsm2tele) die(r, "LSM2 event reset");
                } else if (!strcmp(evKind[k], "sleep")) {
                    fprintf(stderr, "event sleep %g s at %llu\n", evArg[k], done);
                    usleep((useconds_t)(evArg[k] * 1e6));
                } else if (!strcmp(evKind[k], "uninit")) {
                    OSStatus r1 = AudioUnitUninitialize(au), rp = 0;
                    if (!isnan(evArg[k])) { UInt32 pv = (UInt32)evArg[k]; rp = AudioUnitSetProperty(au, 64007, 0, 0, &pv, 4); }
                    OSStatus r2 = AudioUnitInitialize(au);
                    fprintf(stderr, "event uninit/init at %llu: %d %d (64007 %g: %d)\n", done, (int)r1, (int)r2, evArg[k], (int)rp);
                    if (lsm2tele) { die(r1, "LSM2 event uninitialize"); die(rp, "LSM2 event property"); die(r2, "LSM2 event initialize"); }
                } else if (!strcmp(evKind[k], "classinfo") || !strcmp(evKind[k], "reinit")) {
                    CFPropertyListRef ci = NULL; UInt32 cs = sizeof ci;
                    OSStatus r0 = AudioUnitGetProperty(au, kAudioUnitProperty_ClassInfo, 0, 0, &ci, &cs), r1 = 0, r3 = 0;
                    int re = !strcmp(evKind[k], "reinit");
                    if (re) r1 = AudioUnitUninitialize(au);
                    OSStatus r2 = ci ? AudioUnitSetProperty(au, kAudioUnitProperty_ClassInfo, 0, 0, &ci, sizeof ci) : -1;
                    if (re) r3 = AudioUnitInitialize(au);
                    fprintf(stderr, "event %s at %llu: get %d uninit %d set %d init %d\n", evKind[k], done, (int)r0, (int)r1, (int)r2, (int)r3);
                    if (ci) CFRelease(ci);
                    if (lsm2tele) { die(r0, "LSM2 event ClassInfo get"); die(r1, "LSM2 event uninitialize"); die(r2, "LSM2 event ClassInfo set"); die(r3, "LSM2 event initialize"); }
                } else if (!strcmp(evKind[k], "newinst") || !strcmp(evKind[k], "newinst_ci")) {
                    CFPropertyListRef ci = NULL; UInt32 cs = sizeof ci;
                    int withci = !strcmp(evKind[k], "newinst_ci");
                    OSStatus r0 = withci ? AudioUnitGetProperty(au, kAudioUnitProperty_ClassInfo, 0, 0, &ci, &cs) : 0;
                    if (lsm2tele) { die(r0, "LSM2 newinst ClassInfo get"); if (withci && !ci) die(kAudio_ParamError, "LSM2 missing ClassInfo"); }
                    OSStatus r1 = AudioUnitUninitialize(au);
                    if (lsm2tele) die(r1, "LSM2 newinst uninitialize");
                    OSStatus r2 = AudioComponentInstanceDispose(au);
                    if (lsm2tele) die(r2, "LSM2 newinst dispose");
                    if (!isnan(evArg[k])) C.prop64007 = (long long)evArg[k];
                    au = makeUnit(unit, &compVersion);
                    configureUnit(au, &C, ci);
                    if (C.haveTeleBus != haveTeleBus) { fprintf(stderr, "FATAL newinst: telemetry bus changed\n"); exit(1); }
                    fprintf(stderr, "event %s at %llu: 64007 %lld, ClassInfo %s\n", evKind[k], done, C.prop64007, ci ? "restored" : "no");
                    if (ci) CFRelease(ci);
                } else {
                    tsOffset += evArg[k];
                    fprintf(stderr, "event tsjump %g frames at %llu\n", evArg[k], done);
                }
                if (lsm2tele) die(checkLsm2Tele(au, &C), "LSM2 event telemetry readback");
                if (dumpp && strcmp(evKind[k], "sleep") && strcmp(evKind[k], "tsjump")) {
                    char tag[48]; snprintf(tag, sizeof tag, "%s@%llu", evKind[k], done);
                    dumpParams(au, stderr, tag);
                }
            }
            AudioTimeStamp ts = {.mSampleTime = (Float64)done + tsOffset, .mFlags = kAudioTimeStampSampleTimeValid};
            AudioUnitRenderActionFlags fl = 0;
            B.teleAvailable = (UInt32)got; B.telePulls = 0; B.teleInvalid = 0; B.teleSampleTime = ts.mSampleTime;
            OSStatus s = failAt >= 0 && (long long)done >= failAt ? -1 : AudioUnitRender(au, &fl, &ts, 0, (UInt32)got, abl);
            if (!s && lsm2tele && (B.telePulls != 1 || B.teleInvalid)) {
                fprintf(stderr, "LSM2 telemetry was not consumed exactly once at %llu\n", done);
                s = kAudio_ParamError;
            }
            if (s) { fprintf(stderr, "render %d at %llu\n", (int)s, done); failed = s; break; }
            for (size_t i = 0; i < got; ++i) for (int c = 0; c < nch; ++c) inter[i * nch + c] = outb[c][i];
            fwrite(inter, sizeof(float) * nch, got, fo);
            done += got;
            if (++quanta % every == 0) {
                fprintf(fs, "%llu", done);
                for (int i = 0; i < nm; ++i) {
                    AudioUnitParameterValue v = NAN;
                    AudioUnitGetParameter(au, m[i].id, m[i].sc, m[i].el, &v);
                    fprintf(fs, ",%.9g", v);
                }
                fprintf(fs, "\n");
            }
            if (pace > 0) usleep((useconds_t)(1e6 * got / sr / pace));
            if (got < (size_t)q) break;
        }
        if (ferror(fin)) { fprintf(stderr, "input read error after %llu frames\n", nread); if (!failed) failed = -1; }
        if (pace > 0) usleep(1500000); // final drain of the units' log writers
        fclose(fo); fclose(fs); if (ft) fclose(ft);
        snprintf(path, sizeof path, "%s.meta.json", out); FILE *fm = fopen(path, "w");
        fprintf(fm, "{\"unit\":\"%s\",\"component_version\":\"0x%x\",\"strip\":\"%s\",\"sidecar\":\"%s\","
                "\"spts\":%s,\"quantum\":%d,\"sr\":%g,\"channels\":%d,\"file_channels\":%d,\"tele_bus\":%d,\"tele_source\":\"%s\","
                "\"pace\":%g,\"latency_s\":%.12g,\"frames\":%llu,\"frames_read\":%llu,\"meters\":%d,\"render_error\":%d,"
                "\"lsm2_tele\":%s,\"tele_layout\":\"%s\",\"tele_channels_supplied\":%d,\"tele_callbacks\":%llu,\"tele_frames_delivered\":%llu,\"tele_rejected_callbacks\":%llu}\n",
                unit, compVersion, strip, sidecar ? sidecar : "", spts ? "true" : "false", q, sr, nch, filech, haveTeleBus,
                tele ? tele : plant ? plant : "zeros", pace, lat, done, nread, nm, failed,
                lsm2tele ? "true" : "false", lsm2tele ? "planar" : "interleaved", haveTeleBus ? telech : 0, B.teleCallbacks, B.teleFrames, B.teleRejected);
        fclose(fm);
        AudioUnitUninitialize(au); AudioComponentInstanceDispose(au);
        fprintf(stderr, "done %llu frames, %d meters, latency %.6f s\n", done, nm, lat);
        if (failed || done != nread) {
            fprintf(stderr, "FAILED: render error %d, rendered %llu of %llu frames read\n", failed, done, nread);
            rc = 3;
        }
    }
    return rc;
}
