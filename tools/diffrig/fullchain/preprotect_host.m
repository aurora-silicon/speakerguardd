// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT
//
// Offline renderer for the finite-signal portion of Apple's shipped
// builtin_speaker_out graph, stopping immediately before SpeakerProtect.
// Uses public AudioToolbox APIs and the installed AudioDSP component.  Graph
// properties are mandatory: omitting AULowFlow's property 6620, for example,
// renders a different chain from the one installed by builtin_speaker_out.
#import <AudioToolbox/AudioToolbox.h>
#import <Foundation/Foundation.h>
#include <dlfcn.h>

#define MAXCH 6
#define MAXQ 4096

static UInt32 fcc(const char *s) {
    return ((UInt32)s[0] << 24) | ((UInt32)s[1] << 16) |
           ((UInt32)s[2] << 8) | (UInt32)s[3];
}

static void die(OSStatus s, const char *what) {
    if (s) { fprintf(stderr, "FATAL %s: %d\n", what, (int)s); exit(1); }
}

typedef struct { float (*samples)[MAXQ]; int channels; } Feed;
static Feed feed;
static NSDictionary *property_strip;

static OSStatus input_cb(void *ref, AudioUnitRenderActionFlags *flags,
                         const AudioTimeStamp *ts, UInt32 bus, UInt32 frames,
                         AudioBufferList *io) {
    (void)ref; (void)flags; (void)ts; (void)bus;
    if ((int)io->mNumberBuffers != feed.channels) return -1;
    for (int c = 0; c < feed.channels; ++c) {
        io->mBuffers[c].mNumberChannels = 1;
        io->mBuffers[c].mData = feed.samples[c];
        io->mBuffers[c].mDataByteSize = frames * sizeof(float);
    }
    return noErr;
}

static AudioStreamBasicDescription format(double sr, UInt32 channels) {
    AudioStreamBasicDescription f = {0};
    f.mSampleRate = sr;
    f.mFormatID = kAudioFormatLinearPCM;
    f.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked |
                     kAudioFormatFlagIsNonInterleaved;
    f.mBytesPerPacket = f.mBytesPerFrame = sizeof(float);
    f.mFramesPerPacket = 1;
    f.mChannelsPerFrame = channels;
    f.mBitsPerChannel = 32;
    return f;
}

typedef struct {
    const char *display;
    const char *subtype;
    const char *factory;
    int channels;
    AudioUnit au;
    Float64 latency;
} Stage;

static NSDictionary *effect_named(NSDictionary *strip, const char *name) {
    NSString *wanted = @(name);
    for (NSDictionary *s in strip[@"strips"])
        for (NSDictionary *e in s[@"effects"])
            if ([e[@"displayname"] isEqualToString:wanted]) return e;
    return nil;
}

static void apply_properties(AudioUnit au, const char *display) {
    if (!property_strip) return;
    NSString *wanted = @(display);
    for (NSDictionary *box in property_strip[@"Boxes"]) {
        if (![box[@"Name"] isEqualToString:wanted]) continue;
        for (NSDictionary *p in box[@"Properties"]) {
            UInt32 number = [p[@"Number"] unsignedIntValue];
            OSStatus status = -1;
            if (p[@"UInt32"]) {
                UInt32 value = [p[@"UInt32"] unsignedIntValue];
                status = AudioUnitSetProperty(au, number, kAudioUnitScope_Global, 0, &value, sizeof(value));
            } else if (p[@"Float32"]) {
                Float32 value = [p[@"Float32"] floatValue];
                status = AudioUnitSetProperty(au, number, kAudioUnitScope_Global, 0, &value, sizeof(value));
            } else if (p[@"Path"]) {
                CFURLRef url = (__bridge CFURLRef)[NSURL fileURLWithPath:p[@"Path"]];
                status = AudioUnitSetProperty(au, number, kAudioUnitScope_Global, 0, &url, sizeof(url));
                if (status) {
                    CFStringRef path = (__bridge CFStringRef)p[@"Path"];
                    status = AudioUnitSetProperty(au, number, kAudioUnitScope_Global, 0, &path, sizeof(path));
                }
                if (status) {
                    NSDictionary *contents = [NSDictionary dictionaryWithContentsOfFile:p[@"Path"]];
                    if (contents) {
                        CFDictionaryRef dict = (__bridge CFDictionaryRef)contents;
                        status = AudioUnitSetProperty(au, number, kAudioUnitScope_Global, 0, &dict, sizeof(dict));
                    }
                }
            }
            if (status) {
                fprintf(stderr, "FATAL %s property %u: %d\n", display, number, (int)status);
                exit(1);
            }
        }
    }
}

static AudioUnit make_stage(Stage *s, NSDictionary *strip, double sr, int q) {
    NSDictionary *effect = effect_named(strip, s->display);
    if (!effect) { fprintf(stderr, "missing preset: %s\n", s->display); exit(1); }

    AudioComponentDescription d = {
        kAudioUnitType_Effect, fcc(s->subtype), kAudioUnitManufacturer_Apple, 0, 0
    };
    AudioComponent component = AudioComponentFindNext(NULL, &d);
    if (!component) {
        void *image = dlopen("/System/Library/Components/AudioDSP.component/Contents/MacOS/AudioDSP",
                            RTLD_NOW | RTLD_LOCAL);
        AudioComponentFactoryFunction factory = image ?
            (AudioComponentFactoryFunction)dlsym(image, s->factory) : NULL;
        if (!factory) { fprintf(stderr, "missing factory: %s\n", s->factory); exit(1); }
        component = AudioComponentRegister(&d, (__bridge CFStringRef)@(s->display),
                                           0x10001, factory);
    }

    AudioUnit au = NULL;
    die(AudioComponentInstanceNew(component, &au), s->display);
    CFPropertyListRef preset = (__bridge CFPropertyListRef)effect[@"aupreset"];
    die(AudioUnitSetProperty(au, kAudioUnitProperty_ClassInfo,
                             kAudioUnitScope_Global, 0, &preset, sizeof(preset)),
        s->display);
    apply_properties(au, s->display);

    UInt32 max_frames = q;
    die(AudioUnitSetProperty(au, kAudioUnitProperty_MaximumFramesPerSlice,
                             kAudioUnitScope_Global, 0, &max_frames, sizeof(max_frames)),
        s->display);
    AudioStreamBasicDescription f = format(sr, s->channels);
    die(AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat,
                             kAudioUnitScope_Input, 0, &f, sizeof(f)), s->display);
    die(AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat,
                             kAudioUnitScope_Output, 0, &f, sizeof(f)), s->display);
    AURenderCallbackStruct cb = { input_cb, NULL };
    die(AudioUnitSetProperty(au, kAudioUnitProperty_SetRenderCallback,
                             kAudioUnitScope_Input, 0, &cb, sizeof(cb)), s->display);

    // Graph-level parameters wired by builtin_speaker_out.dspg.
    if (!strcmp(s->display, "VarEQ"))
        die(AudioUnitSetParameter(au, 5, kAudioUnitScope_Global, 0, 1.0f, 0), "VarEQ currentGain");
    if (!strcmp(s->display, "AULowFlow"))
        die(AudioUnitSetParameter(au, 4, kAudioUnitScope_Global, 0, 0.0f, 0), "AULowFlow mvol");

    die(AudioUnitInitialize(au), s->display);
    UInt32 size = sizeof(s->latency);
    AudioUnitGetProperty(au, kAudioUnitProperty_Latency, kAudioUnitScope_Global,
                         0, &s->latency, &size);
    s->au = au;
    return au;
}

static void render(Stage *s, float in[MAXCH][MAXQ], float out[MAXCH][MAXQ],
                   UInt32 frames, Float64 sample_time) {
    feed.samples = in;
    feed.channels = s->channels;
    AudioBufferList *abl = calloc(1, offsetof(AudioBufferList, mBuffers) +
                                    s->channels * sizeof(AudioBuffer));
    abl->mNumberBuffers = s->channels;
    for (int c = 0; c < s->channels; ++c) {
        abl->mBuffers[c].mNumberChannels = 1;
        abl->mBuffers[c].mData = out[c];
        abl->mBuffers[c].mDataByteSize = frames * sizeof(float);
    }
    AudioTimeStamp ts = {.mSampleTime = sample_time,
                         .mFlags = kAudioTimeStampSampleTimeValid};
    AudioUnitRenderActionFlags flags = 0;
    die(AudioUnitRender(s->au, &flags, &ts, 0, frames, abl), s->display);
    free(abl);
}

int main(int argc, const char **argv) {
    @autoreleasepool {
        const char *strip_path = NULL, *propstrip_path = NULL, *input_path = NULL, *output_path = NULL,
                   *meta_path = NULL;
        int drivers = 0, q = 512;
        double sr = 48000;
        for (int i = 1; i < argc; ++i) {
            const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
#define ARG(x) (!strcmp(a, x) && v && ++i)
            if (ARG("--strip")) strip_path = v;
            else if (ARG("--propstrip")) propstrip_path = v;
            else if (ARG("--in")) input_path = v;
            else if (ARG("--out")) output_path = v;
            else if (ARG("--meta")) meta_path = v;
            else if (ARG("--drivers")) drivers = atoi(v);
            else if (ARG("--q")) q = atoi(v);
            else if (ARG("--sr")) sr = atof(v);
            else { fprintf(stderr, "bad arg: %s\n", a); return 2; }
        }
        if (!strip_path || !propstrip_path || !input_path || !output_path || !meta_path ||
            (drivers != 4 && drivers != 6) || q < 1 || q > MAXQ) {
            fprintf(stderr, "usage: preprotect_host --strip FILE --propstrip FILE --in stereo.f32 "
                            "--out drivers.f32 --meta FILE --drivers 4|6 [--q 512]\n");
            return 2;
        }

        NSDictionary *strip = [NSDictionary dictionaryWithContentsOfFile:@(strip_path)];
        if (!strip) { fprintf(stderr, "bad strip\n"); return 1; }
        property_strip = [NSDictionary dictionaryWithContentsOfFile:@(propstrip_path)];
        if (!property_strip) { fprintf(stderr, "bad property strip\n"); return 1; }

        Stage pre[] = {
            {"Norm", "ldnm", "AULoudnessNormalizerFactory", 2},
            {"VirtBass", "vben", "AUVirtualBassFactory", 2},
            {"Tone1", "tmst", "AUToneMeisterFactory", 2},
            {"XTC", "axtc", "AUXTCFactory", 2},
            {"Tone2", "tmst", "AUToneMeisterFactory", 2},
            {"VarEQ", "vrq2", "AUVariableEQV2Factory", 2},
            {"Comp", "mstk", "AUMeisterStueckFactory", 2},
        };
        Stage tone3 = {drivers == 4 ? "Tone3" : "analytical", "tmst",
                       "AUToneMeisterFactory", drivers};
        Stage lf = {"ControlFreakLF", "cfrk", "AUControlFreakFactory",
                    drivers == 4 ? 2 : 4};
        Stage hf = {"ControlFreakHF", "cfrk", "AUControlFreakFactory", 2};
        Stage lowflow = {"AULowFlow", "flow", "AULowFlowFactory", drivers};

        for (unsigned i = 0; i < sizeof(pre) / sizeof(pre[0]); ++i)
            make_stage(&pre[i], strip, sr, q);
        make_stage(&tone3, strip, sr, q);
        make_stage(&lf, strip, sr, q);
        make_stage(&hf, strip, sr, q);
        bool have_lowflow = effect_named(strip, "AULowFlow") != nil;
        if (have_lowflow) make_stage(&lowflow, strip, sr, q);

        FILE *fi = fopen(input_path, "rb"), *fo = fopen(output_path, "wb");
        if (!fi || !fo) { fprintf(stderr, "cannot open input/output\n"); return 1; }
        static float inter_in[MAXQ * 2], inter_out[MAXQ * MAXCH];
        static float a[MAXCH][MAXQ], b[MAXCH][MAXQ], branch_in[MAXCH][MAXQ],
                     branch_out[MAXCH][MAXQ];
        uint64_t done = 0;
        for (;;) {
            size_t got = fread(inter_in, sizeof(float) * 2, q, fi);
            if (!got) break;
            memset(a, 0, sizeof(a));
            for (size_t i = 0; i < got; ++i) {
                a[0][i] = inter_in[2 * i];
                a[1][i] = inter_in[2 * i + 1];
            }
            float (*src)[MAXQ] = a, (*dst)[MAXQ] = b;
            for (unsigned k = 0; k < sizeof(pre) / sizeof(pre[0]); ++k) {
                render(&pre[k], src, dst, (UInt32)got, (Float64)done);
                float (*tmp)[MAXQ] = src; src = dst; dst = tmp;
            }

            memset(dst, 0, sizeof(a));
            if (drivers == 4) {
                for (size_t i = 0; i < got; ++i) {
                    dst[0][i] = src[0][i]; dst[1][i] = src[1][i];
                    dst[2][i] = src[0][i]; dst[3][i] = src[1][i];
                }
            } else {
                for (size_t i = 0; i < got; ++i) {
                    dst[0][i] = src[0][i]; dst[1][i] = src[0][i];
                    dst[2][i] = src[0][i]; dst[3][i] = src[1][i];
                    dst[4][i] = src[1][i]; dst[5][i] = src[1][i];
                }
            }
            float (*tmp)[MAXQ] = src; src = dst; dst = tmp;
            render(&tone3, src, dst, (UInt32)got, (Float64)done);
            src = dst;

            memset(branch_in, 0, sizeof(branch_in));
            memset(branch_out, 0, sizeof(branch_out));
            if (drivers == 4) {
                for (int c = 0; c < 2; ++c) memcpy(branch_in[c], src[c], got * sizeof(float));
            } else {
                int map[] = {0, 1, 3, 4};
                for (int c = 0; c < 4; ++c) memcpy(branch_in[c], src[map[c]], got * sizeof(float));
            }
            render(&lf, branch_in, branch_out, (UInt32)got, (Float64)done);
            memset(a, 0, sizeof(a));
            if (drivers == 4) {
                memcpy(a[0], branch_out[0], got * sizeof(float));
                memcpy(a[1], branch_out[1], got * sizeof(float));
            } else {
                // FreakJoin order is L-W1, R-W1, L-W2, R-W2, L-T, R-T.
                int map[] = {0, 2, 1, 3};
                for (int c = 0; c < 4; ++c)
                    memcpy(a[map[c]], branch_out[c], got * sizeof(float));
            }

            memset(branch_in, 0, sizeof(branch_in));
            memset(branch_out, 0, sizeof(branch_out));
            int source_h0 = 2, source_h1 = drivers == 4 ? 3 : 5;
            memcpy(branch_in[0], src[source_h0], got * sizeof(float));
            memcpy(branch_in[1], src[source_h1], got * sizeof(float));
            render(&hf, branch_in, branch_out, (UInt32)got, (Float64)done);
            int out_h0 = drivers == 4 ? 2 : 4, out_h1 = drivers == 4 ? 3 : 5;
            memcpy(a[out_h0], branch_out[0], got * sizeof(float));
            memcpy(a[out_h1], branch_out[1], got * sizeof(float));

            src = a;
            if (have_lowflow) {
                render(&lowflow, src, b, (UInt32)got, (Float64)done);
                src = b;
            }
            for (size_t i = 0; i < got; ++i)
                for (int c = 0; c < drivers; ++c)
                    inter_out[i * drivers + c] = src[c][i];
            fwrite(inter_out, sizeof(float) * drivers, got, fo);
            done += got;
            if (got < (size_t)q) break;
        }
        fclose(fi); fclose(fo);

        FILE *fm = fopen(meta_path, "w");
        fprintf(fm, "{\"frames\":%llu,\"sample_rate\":%.9g,\"quantum\":%d,"
                    "\"drivers\":%d,\"finite_nfp_identity\":true,\"propstrip\":true,\"lowflow\":%s,"
                    "\"stages\":[", done, sr, q, drivers, have_lowflow ? "true" : "false");
        bool first = true;
#define META_STAGE(s) do { if (!first) fprintf(fm, ","); first = false; \
    fprintf(fm, "{\"name\":\"%s\",\"latency_s\":%.12g}", (s).display, (s).latency); } while (0)
        for (unsigned i = 0; i < sizeof(pre) / sizeof(pre[0]); ++i) META_STAGE(pre[i]);
        META_STAGE(tone3); META_STAGE(lf); META_STAGE(hf);
        if (have_lowflow) META_STAGE(lowflow);
        fprintf(fm, "]}\n"); fclose(fm);

        for (unsigned i = 0; i < sizeof(pre) / sizeof(pre[0]); ++i) {
            AudioUnitUninitialize(pre[i].au); AudioComponentInstanceDispose(pre[i].au);
        }
        AudioUnitUninitialize(tone3.au); AudioComponentInstanceDispose(tone3.au);
        AudioUnitUninitialize(lf.au); AudioComponentInstanceDispose(lf.au);
        AudioUnitUninitialize(hf.au); AudioComponentInstanceDispose(hf.au);
        if (have_lowflow) { AudioUnitUninitialize(lowflow.au); AudioComponentInstanceDispose(lowflow.au); }
    }
    return 0;
}
