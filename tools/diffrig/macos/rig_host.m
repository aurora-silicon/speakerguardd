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
//            [--every K] --out PREFIX
// in.f32 is interleaved float32 with N channels. Outputs: PREFIX.out.f32 (interleaved),
// PREFIX.state.csv, PREFIX.tele.f32 (telemetry actually fed), PREFIX.meta.json.
#import <AudioToolbox/AudioToolbox.h>
#import <Foundation/Foundation.h>
#include <dlfcn.h>
#include <math.h>

#define MAXCH 8
#define MAXQ 4096
#define MAXTELE 16

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
typedef struct {
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
} Plant;

static double jnum(NSDictionary *d, NSString *k, int c, double def) {
    id v = d[k];
    if ([v isKindOfClass:NSArray.class]) return c < (int)[v count] ? [v[c] doubleValue] : def;
    if ([v isKindOfClass:NSNumber.class]) return [v doubleValue];
    return def;
}

static void plantLoad(Plant *p, const char *path, int nspk, double sr) {
    NSDictionary *d = [NSJSONSerialization JSONObjectWithData:[NSData dataWithContentsOfFile:@(path)] options:0 error:nil];
    if (!d) { fprintf(stderr, "bad plant json\n"); exit(1); }
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
    }
    p->ringlen = p->delay + MAXQ;
    p->ring = calloc((size_t)p->ringlen * nspk, sizeof(float));
}

// Produce telemetry for n frames from what the AU output `delay` frames ago; then push this
// quantum's output into the ring.
static void plantStep(Plant *p, float *tele, int telech, float out[MAXCH][MAXQ], int n) {
    for (int i = 0; i < n; ++i) {
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
            float iv = (float)(p->i_gain[c] * cur / p->imon_scale[c]);
            float vv = (float)(p->v_gain[c] * v / p->vmon_scale[c]);
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
typedef struct { float audio[MAXCH][MAXQ]; float tele[MAXQ * MAXTELE]; int nch, telech; } Bufs;
static Bufs B;

static OSStatus cbAudio(void *r, AudioUnitRenderActionFlags *f, const AudioTimeStamp *t, UInt32 bus, UInt32 n, AudioBufferList *io) {
    for (UInt32 c = 0; c < io->mNumberBuffers; ++c) {
        io->mBuffers[c].mData = B.audio[c]; io->mBuffers[c].mDataByteSize = n * sizeof(float);
    }
    return noErr;
}
static OSStatus cbTele(void *r, AudioUnitRenderActionFlags *f, const AudioTimeStamp *t, UInt32 bus, UInt32 n, AudioBufferList *io) {
    io->mBuffers[0].mData = B.tele; io->mBuffers[0].mDataByteSize = n * B.telech * sizeof(float);
    io->mBuffers[0].mNumberChannels = B.telech;
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

typedef struct { AudioUnitScope sc; AudioUnitElement el; AudioUnitParameterID id; char name[128]; } Meter;

int main(int argc, const char **argv) {
    @autoreleasepool {
        const char *unit = "spp3", *strip = NULL, *sidecar = NULL, *spts = NULL, *in = NULL, *tele = NULL,
                   *plant = NULL, *out = NULL;
        int nch = 0, q = 512, telech = 12, every = 1, notele = 0;
        double pace = 0; int maxframes = 0; // >0: render at most pace x real time (lets the units' async log writers keep up)
        double sr = 48000;
        UInt32 propIds[32], propVals[32]; int nprops = 0;
        UInt32 cfIds[16]; const char *cfPaths[16]; int ncf = 0;
        UInt32 plIds[8]; const char *plPaths[8]; int npl = 0;
        for (int i = 1; i < argc; ++i) {
            const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
#define ARG(n) (!strcmp(a, n) && v && ++i)
            if (ARG("--unit")) unit = v; else if (ARG("--strip")) strip = v; else if (ARG("--sidecar")) sidecar = v;
            else if (ARG("--spts")) spts = v; else if (ARG("--in")) in = v; else if (ARG("--ch")) nch = atoi(v);
            else if (ARG("--q")) q = atoi(v); else if (ARG("--sr")) sr = atof(v); else if (ARG("--tele")) tele = v;
            else if (ARG("--tele-ch")) telech = atoi(v); else if (ARG("--plant")) plant = v;
            else if (ARG("--every")) every = atoi(v); else if (ARG("--pace")) pace = atof(v); else if (ARG("--maxframes")) maxframes = atoi(v); else if (ARG("--out")) out = v;
            else if (ARG("--prop")) { sscanf(v, "%u=%u", &propIds[nprops], &propVals[nprops]); nprops++; }
            else if (ARG("--plistprop")) { char *eq = strchr(v, '='); plIds[npl] = (UInt32)atoi(v); plPaths[npl++] = eq + 1; }
            else if (ARG("--cfprop")) { char *eq = strchr(v, '='); cfIds[ncf] = (UInt32)atoi(v); cfPaths[ncf++] = eq + 1; }
            else if (!strcmp(a, "--no-tele")) notele = 1;
            else { fprintf(stderr, "bad arg %s\n", a); return 2; }
        }
        if (!strip || !in || !nch || !out || q > MAXQ || nch > MAXCH) { fprintf(stderr, "missing args\n"); return 2; }
        B.nch = nch; B.telech = telech;
        UInt32 compVersion = 0;
        AudioUnit au = makeUnit(unit, &compVersion);

        // preset from the strip
        NSDictionary *sd = [NSDictionary dictionaryWithContentsOfFile:@(strip)];
        NSDictionary *preset = nil;
        for (NSDictionary *s in sd[@"strips"]) for (NSDictionary *e in s[@"effects"])
            if ([e[@"unit"][@"subtype"] unsignedIntValue] == fcc(unit)) preset = e[@"aupreset"];
        if (!preset) { fprintf(stderr, "no %s preset in strip\n", unit); return 1; }
        CFPropertyListRef ci = (__bridge CFPropertyListRef)preset;
        die(AudioUnitSetProperty(au, kAudioUnitProperty_ClassInfo, 0, 0, &ci, sizeof ci), "ClassInfo");
        if (sidecar) {
            CFDictionaryRef sc = (__bridge CFDictionaryRef)[NSDictionary dictionaryWithContentsOfFile:@(sidecar)];
            die(AudioUnitSetProperty(au, 64003, 0, 0, &sc, sizeof sc), "sidecar 64003");
        }
        for (int i = 0; i < nprops; ++i) ck(AudioUnitSetProperty(au, propIds[i], 0, 0, &propVals[i], 4), "prop");
        for (int i = 0; i < npl; ++i) { // a propstrip "Path" property: the plist's contents as a CF dictionary
            CFDictionaryRef pd = (__bridge CFDictionaryRef)[NSDictionary dictionaryWithContentsOfFile:@(plPaths[i])];
            OSStatus r = AudioUnitSetProperty(au, plIds[i], 0, 0, &pd, sizeof pd);
            fprintf(stderr, "plistprop %u <- %s : %d\n", plIds[i], plPaths[i], (int)r);
        }
        for (int i = 0; i < ncf; ++i) {
            // JSON -> CF property list; strings of the form "fcc:XXXX" become 4CC UInt32 numbers
            id obj = [NSJSONSerialization JSONObjectWithData:[NSData dataWithContentsOfFile:@(cfPaths[i])] options:0 error:nil];
            id (^__block conv)(id) = nil;
            id (^fix)(id) = ^id(id o) {
                if ([o isKindOfClass:NSString.class] && [o hasPrefix:@"fcc:"] && [o length] == 8) return @(fcc([[o substringFromIndex:4] UTF8String]));
                if ([o isKindOfClass:NSArray.class]) { NSMutableArray *r = [NSMutableArray array]; for (id x in o) [r addObject:conv(x)]; return r; }
                if ([o isKindOfClass:NSDictionary.class]) { NSMutableDictionary *r = [NSMutableDictionary dictionary]; for (id k in o) r[k] = conv(o[k]); return r; }
                return o;
            };
            conv = fix;
            obj = fix(obj);
            CFTypeRef cf = (__bridge CFTypeRef)obj;
            OSStatus r = AudioUnitSetProperty(au, cfIds[i], 0, 0, &cf, sizeof cf);
            fprintf(stderr, "cfprop %u <- %s : %d\n", cfIds[i], cfPaths[i], (int)r);
        }
        if (spts) {
            CFDataRef d = (__bridge CFDataRef)[NSData dataWithContentsOfFile:@(spts)];
            ck(AudioUnitSetProperty(au, 64009, 0, 0, &d, sizeof d), "SpTS 64009");
        }
        UInt32 mx = maxframes > q ? (UInt32)maxframes : (UInt32)q;
        die(AudioUnitSetProperty(au, kAudioUnitProperty_MaximumFramesPerSlice, 0, 0, &mx, 4), "maxframes");
        AudioStreamBasicDescription af = fmt(sr, nch, true), tf = fmt(sr, telech, false);
        die(AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &af, sizeof af), "in fmt");
        die(AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output, 0, &af, sizeof af), "out fmt");
        UInt32 nin = 0, sz = 4;
        AudioUnitGetProperty(au, kAudioUnitProperty_ElementCount, kAudioUnitScope_Input, 0, &nin, &sz);
        int haveTeleBus = nin > 1 && !notele;
        AURenderCallbackStruct ca = {cbAudio, NULL}, ct = {cbTele, NULL};
        die(AudioUnitSetProperty(au, kAudioUnitProperty_SetRenderCallback, kAudioUnitScope_Input, 0, &ca, sizeof ca), "cb0");
        if (haveTeleBus) {
            die(AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 1, &tf, sizeof tf), "tele fmt");
            die(AudioUnitSetProperty(au, kAudioUnitProperty_SetRenderCallback, kAudioUnitScope_Input, 1, &ct, sizeof ct), "cb1");
        }
        die(AudioUnitInitialize(au), "Initialize");
        Float64 lat = 0; sz = sizeof lat;
        AudioUnitGetProperty(au, kAudioUnitProperty_Latency, 0, 0, &lat, &sz);

        // meters: every non-writable parameter in global scope and in each Part element
        Meter *m = calloc(4096, sizeof *m); int nm = 0;
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
        abl->mNumberBuffers = nch;
        UInt64 done = 0; long quanta = 0;
        int failed = 0;
        for (;;) {
            size_t got = fread(inter, sizeof(float) * nch, q, fin);
            if (!got) break;
            for (size_t i = 0; i < got; ++i) for (int c = 0; c < nch; ++c) B.audio[c][i] = inter[i * nch + c];
            if (haveTeleBus) {
                if (ftele) {
                    size_t g2 = fread(B.tele, sizeof(float) * telech, got, ftele);
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
            AudioTimeStamp ts = {.mSampleTime = (Float64)done, .mFlags = kAudioTimeStampSampleTimeValid};
            AudioUnitRenderActionFlags fl = 0;
            OSStatus s = AudioUnitRender(au, &fl, &ts, 0, (UInt32)got, abl);
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
        if (pace > 0) usleep(500000); // final drain of the units' log writers
        fclose(fo); fclose(fs); if (ft) fclose(ft);
        snprintf(path, sizeof path, "%s.meta.json", out); FILE *fm = fopen(path, "w");
        fprintf(fm, "{\"unit\":\"%s\",\"component_version\":\"0x%x\",\"strip\":\"%s\",\"sidecar\":\"%s\","
                "\"spts\":%s,\"quantum\":%d,\"sr\":%g,\"channels\":%d,\"tele_bus\":%d,\"tele_source\":\"%s\","
                "\"pace\":%g,\"latency_s\":%.12g,\"frames\":%llu,\"meters\":%d,\"render_error\":%d}\n",
                unit, compVersion, strip, sidecar ? sidecar : "", spts ? "true" : "false", q, sr, nch, haveTeleBus,
                tele ? tele : plant ? plant : "zeros", pace, lat, done, nm, failed);
        fclose(fm);
        AudioUnitUninitialize(au); AudioComponentInstanceDispose(au);
        fprintf(stderr, "done %llu frames, %d meters, latency %.6f s\n", done, nm, lat);
    }
    return 0;
}
