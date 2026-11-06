// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT
//
// Offline render of ONE box of a Mac's speaker graph, through the Mac's own
// installed Audio Unit, for the voicing rig (chain.py). Public AudioToolbox
// API only; the unit's state comes from the tuning's strip on disk (class
// info), its properties from the property strip, and graph parameters the
// caller sets with --set. No audio device is opened.
//
// Input and output are interleaved float32 at the given rate and width.
//
//   box_host --strip AUSTRIP --box NAME --channels N --in F --out F --meta F.json
//            [--propstrip PROPSTRIP] [--set ID[/SCOPE/ELEMENT]=VALUE ...]
//            [--bypass] [--q FRAMES] [--sr HZ]
//
// --bypass sets kAudioUnitProperty_BypassEffect (the strip's own "bypass").
// A propstrip "Path" property is tried as a CFURL, a CFString, then the
// file's contents as a CFDictionary; the meta file records
// each property's status so a refused one is visible.
#import <AudioToolbox/AudioToolbox.h>
#import <Foundation/Foundation.h>
#include <dlfcn.h>

#define MAXCH 8
#define MAXQ 4096
#define MAXSET 64

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

static OSStatus input_cb(void *ref, AudioUnitRenderActionFlags *flags, const AudioTimeStamp *ts, UInt32 bus,
                         UInt32 frames, AudioBufferList *io) {
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
    f.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked | kAudioFormatFlagIsNonInterleaved;
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

static void fourcc_str(UInt32 v, char out[5]) {
    out[0] = (char)(v >> 24); out[1] = (char)(v >> 16); out[2] = (char)(v >> 8); out[3] = (char)v; out[4] = 0;
}

// The AudioDSP factory symbol for a subtype, used only when the component
// is not registered in this process already.
static const char *factory_for(const char *subtype) {
    static const char *map[][2] = {
        {"mstk", "AUMeisterStueckFactory"}, {"flow", "AULowFlowFactory"},
        {"ldnm", "AULoudnessNormalizerFactory"}, {"vben", "AUVirtualBassFactory"},
        {"tmst", "AUToneMeisterFactory"}, {"axtc", "AUXTCFactory"},
        {"vrq2", "AUVariableEQV2Factory"}, {"cfrk", "AUControlFreakFactory"},
        {"buzk", "AUBuzzKillFactory"}, {"cdly", "AUChannelSpecificDelayFactory"},
        {"volt", "AUVolumeTaperFactory"},
    };
    for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); ++i)
        if (!strcmp(map[i][0], subtype)) return map[i][1];
    return NULL;
}

static id json_number(double x) { return isfinite(x) ? @(x) : (id)[NSNull null]; }

int main(int argc, const char **argv) {
    @autoreleasepool {
        const char *strip_path = NULL, *prop_path = NULL, *input_path = NULL, *output_path = NULL,
                   *meta_path = NULL, *box_c = NULL;
        const char *sets[MAXSET] = {0};
        int set_count = 0, channels = 0, quantum = 512;
        double sample_rate = 48000.0;
        bool bypass = false;
        for (int i = 1; i < argc; ++i) {
            const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
#define ARG(x) (!strcmp(a, x) && v && ++i)
            if (ARG("--strip")) strip_path = v;
            else if (ARG("--propstrip")) prop_path = v;
            else if (ARG("--box")) box_c = v;
            else if (ARG("--in")) input_path = v;
            else if (ARG("--out")) output_path = v;
            else if (ARG("--meta")) meta_path = v;
            else if (ARG("--channels")) channels = atoi(v);
            else if (ARG("--q")) quantum = atoi(v);
            else if (ARG("--sr")) sample_rate = atof(v);
            else if (ARG("--set") && set_count < MAXSET) sets[set_count++] = v;
            else if (!strcmp(a, "--bypass")) bypass = true;
            else { fprintf(stderr, "bad arg: %s\n", a); return 2; }
#undef ARG
        }
        if (!strip_path || !box_c || !meta_path || !input_path || !output_path || channels < 1 ||
            channels > MAXCH || quantum < 1 || quantum > MAXQ) {
            fprintf(stderr, "usage: box_host --strip F --box NAME --channels N --in F --out F --meta F "
                            "[--propstrip F] [--set ID[/SCOPE/ELEMENT]=V] [--bypass] [--q N] [--sr HZ]\n");
            return 2;
        }
        NSDictionary *strip = [NSDictionary dictionaryWithContentsOfFile:@(strip_path)];
        NSString *box = @(box_c);
        NSDictionary *effect = strip ? effect_named(strip, box) : nil;
        if (!effect) { fprintf(stderr, "box %s not in strip\n", box_c); return 1; }
        NSDictionary *preset = effect[@"aupreset"];
        UInt32 type = [preset[@"type"] unsignedIntValue], subtype = [preset[@"subtype"] unsignedIntValue],
               manu = [preset[@"manufacturer"] unsignedIntValue];
        char st[5];
        fourcc_str(subtype, st);

        AudioComponentDescription d = {type, subtype, manu, 0, 0};
        AudioComponent component = AudioComponentFindNext(NULL, &d);
        if (!component) {
            const char *fname = factory_for(st);
            void *image = dlopen("/System/Library/Components/AudioDSP.component/Contents/MacOS/AudioDSP",
                                 RTLD_NOW | RTLD_LOCAL);
            AudioComponentFactoryFunction factory =
                image && fname ? (AudioComponentFactoryFunction)dlsym(image, fname) : NULL;
            if (!factory) { fprintf(stderr, "no factory for %s\n", st); return 1; }
            component = AudioComponentRegister(&d, (__bridge CFStringRef)box, 0x10001, factory);
        }
        AudioUnit au = NULL;
        fail_status(AudioComponentInstanceNew(component, &au), "AudioComponentInstanceNew");
        CFPropertyListRef cls = (__bridge CFPropertyListRef)preset;
        fail_status(AudioUnitSetProperty(au, kAudioUnitProperty_ClassInfo, kAudioUnitScope_Global, 0, &cls,
                                         sizeof(cls)), "set class info");
        UInt32 max_frames = quantum;
        fail_status(AudioUnitSetProperty(au, kAudioUnitProperty_MaximumFramesPerSlice, kAudioUnitScope_Global, 0,
                                         &max_frames, sizeof(max_frames)), "set max frames");
        AudioStreamBasicDescription f = pcm_format(sample_rate, channels);
        fail_status(AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &f,
                                         sizeof(f)), "set input format");
        fail_status(AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output, 0, &f,
                                         sizeof(f)), "set output format");
        AURenderCallbackStruct cb = {input_cb, NULL};
        fail_status(AudioUnitSetProperty(au, kAudioUnitProperty_SetRenderCallback, kAudioUnitScope_Input, 0, &cb,
                                         sizeof(cb)), "set render callback");

        // Property strip: the box's properties, as the graph sets them.
        NSMutableArray *props = [NSMutableArray array];
        if (prop_path) {
            NSDictionary *ps = [NSDictionary dictionaryWithContentsOfFile:@(prop_path)];
            for (NSDictionary *b in ps[@"Boxes"]) {
                if (![b[@"Name"] isEqualToString:box]) continue;
                for (NSDictionary *p in b[@"Properties"]) {
                    UInt32 num = [p[@"Number"] unsignedIntValue];
                    OSStatus s = -1;
                    NSString *how = @"unsupported";
                    if (p[@"UInt32"]) {
                        UInt32 v = [p[@"UInt32"] unsignedIntValue];
                        s = AudioUnitSetProperty(au, num, kAudioUnitScope_Global, 0, &v, sizeof(v));
                        how = @"UInt32";
                    } else if (p[@"Float32"]) {
                        Float32 v = [p[@"Float32"] floatValue];
                        s = AudioUnitSetProperty(au, num, kAudioUnitScope_Global, 0, &v, sizeof(v));
                        how = @"Float32";
                    } else if (p[@"Path"]) {
                        // The graph's form for a path property is not public:
                        // try a URL, the path string, then the file's contents.
                        CFURLRef url = (__bridge CFURLRef)[NSURL fileURLWithPath:p[@"Path"]];
                        s = AudioUnitSetProperty(au, num, kAudioUnitScope_Global, 0, &url, sizeof(url));
                        how = @"Path(CFURL)";
                        if (s) {
                            CFStringRef str = (__bridge CFStringRef)p[@"Path"];
                            s = AudioUnitSetProperty(au, num, kAudioUnitScope_Global, 0, &str, sizeof(str));
                            how = @"Path(CFString)";
                        }
                        if (s) {
                            NSDictionary *contents = [NSDictionary dictionaryWithContentsOfFile:p[@"Path"]];
                            CFDictionaryRef dict = (__bridge CFDictionaryRef)contents;
                            if (contents)
                                s = AudioUnitSetProperty(au, num, kAudioUnitScope_Global, 0, &dict, sizeof(dict));
                            how = @"Path(contents)";
                        }
                    }
                    [props addObject:@{@"number": @(num), @"as": how, @"status": @(s)}];
                }
            }
        }
        if (bypass) {
            UInt32 one = 1;
            fail_status(AudioUnitSetProperty(au, kAudioUnitProperty_BypassEffect, kAudioUnitScope_Global, 0, &one,
                                             sizeof(one)), "set bypass");
        }
        NSMutableArray *applied = [NSMutableArray array];
        for (int i = 0; i < set_count; ++i) {
            char *end = NULL;
            unsigned long pid = strtoul(sets[i], &end, 0), scope = 0, elem = 0;
            if (end && *end == '/') scope = strtoul(end + 1, &end, 0);
            if (end && *end == '/') elem = strtoul(end + 1, &end, 0);
            if (!end || *end != '=') { fprintf(stderr, "bad --set: %s\n", sets[i]); return 2; }
            float value = strtof(end + 1, NULL);
            fail_status(AudioUnitSetParameter(au, (AudioUnitParameterID)pid, (AudioUnitScope)scope,
                                              (AudioUnitElement)elem, value, 0), "set parameter");
            [applied addObject:@{@"id": @(pid), @"scope": @(scope), @"element": @(elem), @"value": @(value)}];
        }
        fail_status(AudioUnitInitialize(au), "AudioUnitInitialize");
        // Graph parameters set before initialisation are applied again after
        // it, the way a running graph keeps them.
        for (NSDictionary *a in applied)
            AudioUnitSetParameter(au, [a[@"id"] unsignedIntValue], [a[@"scope"] unsignedIntValue],
                                  [a[@"element"] unsignedIntValue], [a[@"value"] floatValue], 0);

        Float64 latency = 0, tail = 0;
        UInt32 sz = sizeof(latency);
        AudioUnitGetProperty(au, kAudioUnitProperty_Latency, kAudioUnitScope_Global, 0, &latency, &sz);
        sz = sizeof(tail);
        AudioUnitGetProperty(au, kAudioUnitProperty_TailTime, kAudioUnitScope_Global, 0, &tail, &sz);

        FILE *input = fopen(input_path, "rb"), *output = fopen(output_path, "wb");
        if (!input || !output) { fprintf(stderr, "cannot open input/output\n"); return 1; }
        static float iin[MAXQ * MAXCH], iout[MAXQ * MAXCH];
        static float pin[MAXCH][MAXQ], pout[MAXCH][MAXQ];
        uint64_t frames = 0;
        size_t abl_size = offsetof(AudioBufferList, mBuffers) + channels * sizeof(AudioBuffer);
        AudioBufferList *abl = calloc(1, abl_size);
        for (;;) {
            size_t got = fread(iin, sizeof(float) * channels, quantum, input);
            if (!got) break;
            memset(pin, 0, sizeof(pin));
            memset(pout, 0, sizeof(pout));
            for (size_t i = 0; i < got; ++i)
                for (int c = 0; c < channels; ++c) pin[c][i] = iin[i * channels + c];
            feed.samples = pin;
            feed.channels = channels;
            abl->mNumberBuffers = channels;
            for (int c = 0; c < channels; ++c) {
                abl->mBuffers[c].mNumberChannels = 1;
                abl->mBuffers[c].mData = pout[c];
                abl->mBuffers[c].mDataByteSize = (UInt32)got * sizeof(float);
            }
            AudioTimeStamp ts = {.mSampleTime = (Float64)frames, .mFlags = kAudioTimeStampSampleTimeValid};
            AudioUnitRenderActionFlags fl = 0;
            fail_status(AudioUnitRender(au, &fl, &ts, 0, (UInt32)got, abl), "AudioUnitRender");
            for (size_t i = 0; i < got; ++i)
                for (int c = 0; c < channels; ++c) iout[i * channels + c] = pout[c][i];
            fwrite(iout, sizeof(float) * channels, got, output);
            frames += got;
            if (got < (size_t)quantum) break;
        }
        free(abl);
        fclose(input);
        fclose(output);

        NSDictionary *meta = @{
            @"box": box, @"subtype": @(st), @"preset": preset[@"name"] ?: @"",
            @"bypass": @(bypass), @"sample_rate": @(sample_rate), @"channels": @(channels),
            @"quantum": @(quantum), @"frames": @(frames), @"latency_s": json_number(latency),
            @"tail_s": json_number(tail), @"properties": props, @"set": applied,
        };
        NSData *json = [NSJSONSerialization dataWithJSONObject:meta options:NSJSONWritingPrettyPrinted error:nil];
        [json writeToFile:@(meta_path) atomically:YES];
        AudioUnitUninitialize(au);
        AudioComponentInstanceDispose(au);
    }
    return 0;
}
