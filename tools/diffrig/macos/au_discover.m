// SPDX-License-Identifier: MIT
// Aurora speakerguardd v2 rig: discover the observable state of a protection AudioUnit.
// Our own host code; it only uses public AudioToolbox APIs on the installed component.
#import <AudioToolbox/AudioToolbox.h>
#import <Foundation/Foundation.h>
#include <dlfcn.h>

static const char *factoryFor(const char *st) {
    if (!strcmp(st, "spp3")) return "AUSpeakerProtectionV3Factory";
    if (!strcmp(st, "atsp")) return "AUThermalSpeakerProtectionFactory";
    if (!strcmp(st, "lsm2")) return "AULoudspeakerManagerV2Factory";
    return NULL;
}

static AudioUnit makeUnit(const char *st) {
    UInt32 sub = ((UInt32)st[0] << 24) | ((UInt32)st[1] << 16) | ((UInt32)st[2] << 8) | (UInt32)st[3];
    AudioComponentDescription d = {kAudioUnitType_Effect, sub, kAudioUnitManufacturer_Apple, 0, 0};
    AudioComponent c = AudioComponentFindNext(NULL, &d);
    printf("registered-component=%s\n", c ? "yes" : "no");
    if (!c) {
        void *h = dlopen("/System/Library/Components/AudioDSP.component/Contents/MacOS/AudioDSP", RTLD_NOW | RTLD_LOCAL);
        AudioComponentFactoryFunction f = h ? dlsym(h, factoryFor(st)) : NULL;
        if (!f) { fprintf(stderr, "no factory\n"); return NULL; }
        c = AudioComponentRegister(&d, CFSTR("aurora rig"), 0x10001, f);
    }
    AudioUnit au = NULL;
    OSStatus s = AudioComponentInstanceNew(c, &au);
    if (s) { fprintf(stderr, "InstanceNew %d\n", (int)s); return NULL; }
    return au;
}

int main(int argc, const char **argv) {
    @autoreleasepool {
        const char *st = argc > 1 ? argv[1] : "lsm2";
        NSString *strip = argc > 2 ? @(argv[2]) : nil;
        AudioUnit au = makeUnit(st);
        if (!au) return 1;
        if (strip) {
            NSDictionary *d = [NSDictionary dictionaryWithContentsOfFile:strip];
            UInt32 sub = ((UInt32)st[0] << 24) | ((UInt32)st[1] << 16) | ((UInt32)st[2] << 8) | (UInt32)st[3];
            for (NSDictionary *s in d[@"strips"]) for (NSDictionary *e in s[@"effects"])
                if ([e[@"unit"][@"subtype"] unsignedIntValue] == sub) {
                    CFPropertyListRef ci = (__bridge CFPropertyListRef)e[@"aupreset"];
                    OSStatus r = AudioUnitSetProperty(au, kAudioUnitProperty_ClassInfo, 0, 0, &ci, sizeof(ci));
                    printf("ClassInfo set=%d\n", (int)r);
                }
        }
        // element counts
        for (AudioUnitScope sc = 0; sc <= 4; ++sc) {
            UInt32 n = 0, sz = sizeof(n);
            if (!AudioUnitGetProperty(au, kAudioUnitProperty_ElementCount, sc, 0, &n, &sz))
                printf("scope %u elements %u\n", sc, n);
        }
        // property scan
        UInt32 ranges[][2] = {{0, 200}, {1000, 1100}, {3000, 3200}, {4000, 4100}, {64000, 64100}, {100000, 100100}};
        for (int r = 0; r < 6; ++r) for (UInt32 p = ranges[r][0]; p < ranges[r][1]; ++p)
            for (AudioUnitScope sc = 0; sc <= 4; ++sc) {
                UInt32 sz = 0; Boolean wr = 0;
                if (AudioUnitGetPropertyInfo(au, p, sc, 0, &sz, &wr)) continue;
                printf("prop %u scope %u size %u writable %u", p, sc, sz, wr);
                if (p >= 64000 && sz == 8) {
                    CFTypeRef v = NULL; UInt32 s2 = sizeof(v);
                    if (!AudioUnitGetProperty(au, p, sc, 0, &v, &s2) && v) {
                        NSString *desc = [(__bridge id)v description];
                        printf(" cf=%s", [[desc substringToIndex:MIN(desc.length, 300)] UTF8String]);
                    }
                } else if (p >= 64000 && sz == 4) {
                    UInt32 v = 0, s2 = 4;
                    if (!AudioUnitGetProperty(au, p, sc, 0, &v, &s2)) printf(" u32=%u", v);
                }
                printf("\n");
            }
        // parameter lists per scope; mark non-writable (meter) ones
        for (AudioUnitScope sc = 0; sc <= 4; ++sc) {
            UInt32 sz = 0;
            if (AudioUnitGetPropertyInfo(au, kAudioUnitProperty_ParameterList, sc, 0, &sz, NULL) || !sz) continue;
            AudioUnitParameterID *ids = malloc(sz);
            AudioUnitGetProperty(au, kAudioUnitProperty_ParameterList, sc, 0, ids, &sz);
            UInt32 n = sz / sizeof(*ids), meters = 0;
            for (UInt32 i = 0; i < n; ++i) {
                AudioUnitParameterInfo pi = {0}; UInt32 ps = sizeof(pi);
                if (AudioUnitGetProperty(au, kAudioUnitProperty_ParameterInfo, sc, ids[i], &pi, &ps)) continue;
                bool wr = pi.flags & kAudioUnitParameterFlag_IsWritable;
                if (wr && !getenv("ALL")) continue;
                char name[256] = {0};
                if (pi.cfNameString) CFStringGetCString(pi.cfNameString, name, sizeof name, kCFStringEncodingUTF8);
                printf("meter scope %u id %u name '%s' unit %u flags 0x%x min %g max %g\n", sc, ids[i], name,
                       pi.unit, (unsigned)pi.flags, pi.minValue, pi.maxValue);
                meters++;
            }
            printf("scope %u params %u meters %u\n", sc, n, meters);
            free(ids);
        }
        AudioComponentInstanceDispose(au);
    }
    return 0;
}
