// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT
//
// Offline reference host for the Norm rig: renders one installed Apple
// Audio Unit stage through the public AudioToolbox API (no audio device),
// with the stage's saved state taken from the Mac's strip at run time, and
// optionally records the unit's read-only momentary gain (parameter 100)
// and maximum gain (101) after every render call (--trace).
#import <AudioToolbox/AudioToolbox.h>
#import <Foundation/Foundation.h>
#include <dlfcn.h>

#define MAXCH 8
#define MAXQ 4096
#define MAXSET 64

static UInt32 fcc(const char *s) {
    return ((UInt32)s[0] << 24) | ((UInt32)s[1] << 16) |
           ((UInt32)s[2] << 8) | (UInt32)s[3];
}

static void fail_status(OSStatus status, const char *what) {
    if (status) {
        fprintf(stderr, "FATAL %s: %d\n", what, (int)status);
        exit(1);
    }
}

typedef struct {
    float (*samples)[MAXQ];
    int channels;
} Feed;
static Feed feed;

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

static AudioStreamBasicDescription pcm_format(double sample_rate, int channels) {
    AudioStreamBasicDescription f = {0};
    f.mSampleRate = sample_rate;
    f.mFormatID = kAudioFormatLinearPCM;
    f.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked |
                     kAudioFormatFlagIsNonInterleaved;
    f.mBytesPerPacket = f.mBytesPerFrame = sizeof(float);
    f.mFramesPerPacket = 1;
    f.mChannelsPerFrame = channels;
    f.mBitsPerChannel = 32;
    return f;
}

static NSDictionary *effect_named(NSDictionary *strip, NSString *wanted) {
    for (NSDictionary *s in strip[@"strips"])
        for (NSDictionary *e in s[@"effects"])
            if ([e[@"displayname"] isEqualToString:wanted]) return e;
    return nil;
}

static const char *factory_for(NSString *stage) {
    if ([stage isEqualToString:@"Comp"]) return "AUMeisterStueckFactory";
    if ([stage isEqualToString:@"AULowFlow"]) return "AULowFlowFactory";
    if ([stage isEqualToString:@"Norm"]) return "AULoudnessNormalizerFactory";
    if ([stage isEqualToString:@"VirtBass"]) return "AUVirtualBassFactory";
    if ([stage isEqualToString:@"Tone1"] || [stage isEqualToString:@"Tone2"] ||
        [stage isEqualToString:@"Tone3"] || [stage isEqualToString:@"analytical"])
        return "AUToneMeisterFactory";
    if ([stage isEqualToString:@"XTC"]) return "AUXTCFactory";
    if ([stage isEqualToString:@"VarEQ"]) return "AUVariableEQV2Factory";
    if ([stage isEqualToString:@"ControlFreakLF"] ||
        [stage isEqualToString:@"ControlFreakHF"])
        return "AUControlFreakFactory";
    return NULL;
}

static const char *subtype_for(NSString *stage) {
    if ([stage isEqualToString:@"Comp"]) return "mstk";
    if ([stage isEqualToString:@"AULowFlow"]) return "flow";
    if ([stage isEqualToString:@"Norm"]) return "ldnm";
    if ([stage isEqualToString:@"VirtBass"]) return "vben";
    if ([stage isEqualToString:@"Tone1"] || [stage isEqualToString:@"Tone2"] ||
        [stage isEqualToString:@"Tone3"] || [stage isEqualToString:@"analytical"])
        return "tmst";
    if ([stage isEqualToString:@"XTC"]) return "axtc";
    if ([stage isEqualToString:@"VarEQ"]) return "vrq2";
    if ([stage isEqualToString:@"ControlFreakLF"] ||
        [stage isEqualToString:@"ControlFreakHF"])
        return "cfrk";
    return NULL;
}

static AudioUnit instantiate(NSString *stage, NSDictionary *effect,
                             double sample_rate, int channels, int quantum) {
    const char *subtype = subtype_for(stage);
    const char *factory_name = factory_for(stage);
    if (!subtype || !factory_name) {
        fprintf(stderr, "unsupported stage: %s\n", stage.UTF8String);
        exit(2);
    }
    AudioComponentDescription d = {
        kAudioUnitType_Effect, fcc(subtype), kAudioUnitManufacturer_Apple, 0, 0
    };
    AudioComponent component = AudioComponentFindNext(NULL, &d);
    if (!component) {
        void *image = dlopen("/System/Library/Components/AudioDSP.component/Contents/MacOS/AudioDSP",
                            RTLD_NOW | RTLD_LOCAL);
        AudioComponentFactoryFunction factory = image ?
            (AudioComponentFactoryFunction)dlsym(image, factory_name) : NULL;
        if (!factory) {
            fprintf(stderr, "missing factory: %s\n", factory_name);
            exit(1);
        }
        component = AudioComponentRegister(&d, (__bridge CFStringRef)stage,
                                           0x10001, factory);
    }

    AudioUnit au = NULL;
    fail_status(AudioComponentInstanceNew(component, &au), "AudioComponentInstanceNew");
    CFPropertyListRef preset = (__bridge CFPropertyListRef)effect[@"aupreset"];
    fail_status(AudioUnitSetProperty(au, kAudioUnitProperty_ClassInfo,
                                    kAudioUnitScope_Global, 0,
                                    &preset, sizeof(preset)), "set class info");
    UInt32 max_frames = quantum;
    fail_status(AudioUnitSetProperty(au, kAudioUnitProperty_MaximumFramesPerSlice,
                                    kAudioUnitScope_Global, 0,
                                    &max_frames, sizeof(max_frames)), "set max frames");
    AudioStreamBasicDescription f = pcm_format(sample_rate, channels);
    fail_status(AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat,
                                    kAudioUnitScope_Input, 0,
                                    &f, sizeof(f)), "set input format");
    fail_status(AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat,
                                    kAudioUnitScope_Output, 0,
                                    &f, sizeof(f)), "set output format");
    AURenderCallbackStruct cb = { input_cb, NULL };
    fail_status(AudioUnitSetProperty(au, kAudioUnitProperty_SetRenderCallback,
                                    kAudioUnitScope_Input, 0,
                                    &cb, sizeof(cb)), "set render callback");
    return au;
}

static id json_number(double x) {
    if (!isfinite(x)) return [NSNull null];
    return @(x);
}

static NSArray *parameter_inventory(AudioUnit au) {
    UInt32 size = 0;
    Boolean writable = false;
    OSStatus status = AudioUnitGetPropertyInfo(au, kAudioUnitProperty_ParameterList,
                                               kAudioUnitScope_Global, 0,
                                               &size, &writable);
    if (status || !size) return @[];
    AudioUnitParameterID *ids = malloc(size);
    fail_status(AudioUnitGetProperty(au, kAudioUnitProperty_ParameterList,
                                    kAudioUnitScope_Global, 0,
                                    ids, &size), "get parameter list");
    NSMutableArray *out = [NSMutableArray array];
    for (UInt32 i = 0; i < size / sizeof(*ids); ++i) {
        AudioUnitParameterInfo info = {0};
        UInt32 info_size = sizeof(info);
        status = AudioUnitGetProperty(au, kAudioUnitProperty_ParameterInfo,
                                      kAudioUnitScope_Global, ids[i],
                                      &info, &info_size);
        AudioUnitParameterValue value = NAN;
        AudioUnitGetParameter(au, ids[i], kAudioUnitScope_Global, 0, &value);
        NSString *name = info.cfNameString ? (__bridge NSString *)info.cfNameString : @"";
        [out addObject:@{
            @"id": @(ids[i]), @"name": name,
            @"unit": @(info.unit), @"flags": @(info.flags),
            @"min": json_number(info.minValue),
            @"max": json_number(info.maxValue),
            @"default": json_number(info.defaultValue),
            @"value": json_number(value),
        }];
    }
    free(ids);
    return out;
}

int main(int argc, const char **argv) {
    @autoreleasepool {
        const char *strip_path = NULL, *input_path = NULL, *output_path = NULL,
                   *meta_path = NULL, *stage_c = NULL, *trace_path = NULL;
        const char *sets[MAXSET] = {0};
        int set_count = 0, channels = 0, quantum = 512;
        double sample_rate = 48000.0;
        bool inventory_only = false;
        for (int i = 1; i < argc; ++i) {
            const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
#define ARG(x) (!strcmp(a, x) && v && ++i)
            if (ARG("--strip")) strip_path = v;
            else if (ARG("--stage")) stage_c = v;
            else if (ARG("--in")) input_path = v;
            else if (ARG("--out")) output_path = v;
            else if (ARG("--meta")) meta_path = v;
            else if (ARG("--trace")) trace_path = v;
            else if (ARG("--channels")) channels = atoi(v);
            else if (ARG("--q")) quantum = atoi(v);
            else if (ARG("--sr")) sample_rate = atof(v);
            else if (ARG("--set") && set_count < MAXSET) sets[set_count++] = v;
            else if (!strcmp(a, "--inventory")) inventory_only = true;
            else { fprintf(stderr, "bad arg: %s\n", a); return 2; }
        }
        if (!strip_path || !stage_c || !meta_path || channels < 1 ||
            channels > MAXCH || quantum < 1 || quantum > MAXQ ||
            (!inventory_only && (!input_path || !output_path))) {
            fprintf(stderr, "usage: au_stage_host --strip FILE --stage NAME "
                    "--channels N --meta FILE [--inventory] [--in FILE --out FILE] "
                    "[--set ID=VALUE] [--q N] [--sr HZ]\n");
            return 2;
        }

        NSDictionary *strip = [NSDictionary dictionaryWithContentsOfFile:@(strip_path)];
        NSString *stage = @(stage_c);
        NSDictionary *effect = strip ? effect_named(strip, stage) : nil;
        if (!effect) { fprintf(stderr, "stage not present in strip\n"); return 1; }
        AudioUnit au = instantiate(stage, effect, sample_rate, channels, quantum);
        for (int i = 0; i < set_count; ++i) {
            char *end = NULL;
            unsigned long pid = strtoul(sets[i], &end, 0);
            if (!end || *end != '=') { fprintf(stderr, "bad --set: %s\n", sets[i]); return 2; }
            float value = strtof(end + 1, NULL);
            fail_status(AudioUnitSetParameter(au, (AudioUnitParameterID)pid,
                                              kAudioUnitScope_Global, 0,
                                              value, 0), "set parameter");
        }
        fail_status(AudioUnitInitialize(au), "AudioUnitInitialize");

        Float64 latency = 0, tail = 0;
        UInt32 property_size = sizeof(latency);
        AudioUnitGetProperty(au, kAudioUnitProperty_Latency,
                             kAudioUnitScope_Global, 0, &latency, &property_size);
        property_size = sizeof(tail);
        AudioUnitGetProperty(au, kAudioUnitProperty_TailTime,
                             kAudioUnitScope_Global, 0, &tail, &property_size);

        uint64_t frame_count = 0;
        if (!inventory_only) {
            FILE *input = fopen(input_path, "rb"), *output = fopen(output_path, "wb");
            if (!input || !output) { fprintf(stderr, "cannot open input/output\n"); return 1; }
            FILE *trace = trace_path ? fopen(trace_path, "wb") : NULL;
            static float interleaved_in[MAXQ * MAXCH], interleaved_out[MAXQ * MAXCH];
            static float planar_in[MAXCH][MAXQ], planar_out[MAXCH][MAXQ];
            for (;;) {
                size_t got = fread(interleaved_in, sizeof(float) * channels,
                                   quantum, input);
                if (!got) break;
                memset(planar_in, 0, sizeof(planar_in));
                memset(planar_out, 0, sizeof(planar_out));
                for (size_t f = 0; f < got; ++f)
                    for (int c = 0; c < channels; ++c)
                        planar_in[c][f] = interleaved_in[f * channels + c];
                feed.samples = planar_in;
                feed.channels = channels;
                AudioBufferList *abl = calloc(1, offsetof(AudioBufferList, mBuffers) +
                                                 channels * sizeof(AudioBuffer));
                abl->mNumberBuffers = channels;
                for (int c = 0; c < channels; ++c) {
                    abl->mBuffers[c].mNumberChannels = 1;
                    abl->mBuffers[c].mData = planar_out[c];
                    abl->mBuffers[c].mDataByteSize = (UInt32)got * sizeof(float);
                }
                AudioTimeStamp ts = {.mSampleTime = (Float64)frame_count,
                                     .mFlags = kAudioTimeStampSampleTimeValid};
                AudioUnitRenderActionFlags flags = 0;
                fail_status(AudioUnitRender(au, &flags, &ts, 0, (UInt32)got, abl),
                            "AudioUnitRender");
                free(abl);
                for (size_t f = 0; f < got; ++f)
                    for (int c = 0; c < channels; ++c)
                        interleaved_out[f * channels + c] = planar_out[c][f];
                fwrite(interleaved_out, sizeof(float) * channels, got, output);
                frame_count += got;
                if (trace) { AudioUnitParameterValue tv[2] = {0, 0};
                    AudioUnitGetParameter(au, 100, kAudioUnitScope_Global, 0, &tv[0]);
                    AudioUnitGetParameter(au, 101, kAudioUnitScope_Global, 0, &tv[1]);
                    fwrite(tv, sizeof(tv), 1, trace); }
                if (got < (size_t)quantum) break;
            }
            fclose(input); fclose(output); if (trace) fclose(trace);
        }

        NSDictionary *meta = @{
            @"stage": stage,
            @"preset": effect[@"aupreset"][@"name"] ?: @"",
            @"sample_rate": @(sample_rate), @"channels": @(channels),
            @"quantum": @(quantum), @"frames": @(frame_count),
            @"latency_s": @(latency), @"tail_s": @(tail),
            @"parameters": parameter_inventory(au),
        };
        NSData *json = [NSJSONSerialization dataWithJSONObject:meta
                                                       options:NSJSONWritingPrettyPrinted
                                                         error:nil];
        [json writeToFile:@(meta_path) atomically:YES];
        AudioUnitUninitialize(au);
        AudioComponentInstanceDispose(au);
    }
    return 0;
}
