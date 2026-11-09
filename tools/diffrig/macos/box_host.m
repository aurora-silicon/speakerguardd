// Offline host for one box of a Mac's speaker graph (any aufx subtype).
// It uses only public AudioToolbox APIs and an on-disk class-info preset.
#import <AudioToolbox/AudioToolbox.h>
#import <Foundation/Foundation.h>
#include <dlfcn.h>

#define MAXCH 8
#define MAXQ 4096
#define MAXSET 512

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
// The property strip (`--propstrip`): the box's graph properties, set as
// macOS sets them after the class info (AULowFlow's 6620 sidecar, say).
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

static void apply_properties(AudioUnit au, NSString *wanted) {
    if (!property_strip) return;
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
                fprintf(stderr, "FATAL %s property %u: %d\n", wanted.UTF8String, number, (int)status);
                exit(1);
            }
        }
    }
}

static AudioUnit instantiate(NSString *stage, NSDictionary *effect,
                             const char *subtype, const char *factory_name,
                             double sample_rate, int channels, int quantum) {
    if (!subtype || strlen(subtype) != 4 || !factory_name) {
        fprintf(stderr, "need --subtype XXXX and --factory NAME\n");
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
    if (effect) {
        CFPropertyListRef preset = (__bridge CFPropertyListRef)effect[@"aupreset"];
        fail_status(AudioUnitSetProperty(au, kAudioUnitProperty_ClassInfo,
                                        kAudioUnitScope_Global, 0,
                                        &preset, sizeof(preset)), "set class info");
    }
    apply_properties(au, stage);
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
                   *meta_path = NULL, *stage_c = NULL,
                   *subtype_c = NULL, *factory_c = NULL;
        const char *sets[MAXSET] = {0};
        int set_count = 0, channels = 0, quantum = 512;
        double sample_rate = 48000.0;
        bool inventory_only = false, no_preset = false;
        for (int i = 1; i < argc; ++i) {
            const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
#define ARG(x) (!strcmp(a, x) && v && ++i)
            if (ARG("--strip")) strip_path = v;
            else if (ARG("--stage")) stage_c = v;
            else if (ARG("--propstrip")) {
                property_strip = [NSDictionary dictionaryWithContentsOfFile:@(v)];
                if (!property_strip) { fprintf(stderr, "bad property strip: %s\n", v); return 1; }
            }
            else if (ARG("--subtype")) subtype_c = v;
            else if (ARG("--factory")) factory_c = v;
            else if (ARG("--in")) input_path = v;
            else if (ARG("--out")) output_path = v;
            else if (ARG("--meta")) meta_path = v;
            else if (ARG("--channels")) channels = atoi(v);
            else if (ARG("--q")) quantum = atoi(v);
            else if (ARG("--sr")) sample_rate = atof(v);
            else if (ARG("--set") && set_count < MAXSET) sets[set_count++] = v;
            else if (!strcmp(a, "--inventory")) inventory_only = true;
            else if (!strcmp(a, "--no-preset")) no_preset = true;
            else { fprintf(stderr, "bad arg: %s\n", a); return 2; }
        }
        if (!strip_path || !stage_c || !meta_path || channels < 1 ||
            channels > MAXCH || quantum < 1 || quantum > MAXQ ||
            (!inventory_only && (!input_path || !output_path))) {
            fprintf(stderr, "usage: box_host --strip FILE --stage NAME --subtype XXXX --factory NAME "
                    "--channels N --meta FILE [--inventory] [--in FILE --out FILE] "
                    "[--set ID=VALUE] [--propstrip FILE] [--q N] [--sr HZ]\n");
            return 2;
        }

        NSDictionary *strip = [NSDictionary dictionaryWithContentsOfFile:@(strip_path)];
        NSString *stage = @(stage_c);
        NSDictionary *effect = strip ? effect_named(strip, stage) : nil;
        if (!effect) { fprintf(stderr, "stage not present in strip\n"); return 1; }
        if (no_preset) effect = nil; // the unit's own defaults (black-box probing)
        AudioUnit au = instantiate(stage, effect, subtype_c, factory_c, sample_rate, channels, quantum);
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
                if (got < (size_t)quantum) break;
            }
            fclose(input); fclose(output);
        }

        NSDictionary *meta = @{
            @"stage": stage,
            @"preset": (effect ? effect[@"aupreset"][@"name"] : nil) ?: @"",
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
