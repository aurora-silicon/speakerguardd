// SPDX-License-Identifier: MIT
// lsidprobe: silent read-back of the speaker path's observable state on macOS,
// for the Aurora speakerguardd v2 LSID tester kit.
//
// Our own reader. It uses only public, documented interfaces:
//   CoreAudio  (AudioObject* properties of the built-in output device, one IOProc)
//   AudioToolbox (AudioComponentFindNext / AudioComponentCopyName / GetVersion,
//                 AudioComponentInstanceNew + kAudioUnitProperty_ParameterList)
//   IOKit      (IORegistry iteration and property reads, as `ioreg` does)
//   CommonCrypto (SHA-256)
// No private calls, no dlopen/dlsym, no binary inspection, nothing recorded.
//
// Safety: it plays only digital silence (every output sample is 0.0) on the
// built-in speaker device, and only after it has set that device's output
// volume to 0 and mute on and read both back. A watchdog re-reads them every
// 200 ms while the stream runs and stops the stream at once if either moves,
// or if the output stops being the internal speaker (headphones plugged).
//
// usage: lsidprobe [--seconds N] [--no-stream] [--leave-muted]
//   --seconds N    length of the muted zero stream (default 30, 5..120)
//   --no-stream    read-only: inventory and registry only, no stream, no volume change
//   --leave-muted  do not restore the previous volume/mute afterwards
// Output: "RESULT <step> PASS|FAIL|SKIP|INFO <detail>" and "DATA <key>=<value>" lines.
// Exit: 0 every safety step held, 1 a safety step failed, 2 usage.

#include <AudioToolbox/AudioToolbox.h>
#include <CommonCrypto/CommonDigest.h>
#include <CoreAudio/CoreAudio.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <ctype.h>
#include <stdarg.h>
#include <mach/mach_time.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int g_safety_fail = 0;

static void result(const char *step, const char *verdict, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
static void result(const char *step, const char *verdict, const char *fmt, ...) {
  printf("RESULT %s %s ", step, verdict);
  va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
  printf("\n"); fflush(stdout);
}

static void fourcc(UInt32 v, char out[5]) {
  for (int i = 0; i < 4; i++) { char c = (char)((v >> (24 - 8 * i)) & 0xff); out[i] = isprint((unsigned char)c) ? c : '?'; }
  out[4] = 0;
}

static void cfstr(CFStringRef s, char *buf, size_t n) {
  buf[0] = 0;
  if (s && CFGetTypeID(s) == CFStringGetTypeID()) CFStringGetCString(s, buf, (CFIndex)n, kCFStringEncodingUTF8);
}

// ---------------------------------------------------------------- CoreAudio
static int has(AudioObjectID o, AudioObjectPropertySelector s, AudioObjectPropertyScope sc, UInt32 el) {
  AudioObjectPropertyAddress a = {s, sc, el};
  return AudioObjectHasProperty(o, &a);
}
static int settable(AudioObjectID o, AudioObjectPropertySelector s, AudioObjectPropertyScope sc, UInt32 el) {
  AudioObjectPropertyAddress a = {s, sc, el};
  Boolean w = 0;
  return AudioObjectHasProperty(o, &a) && !AudioObjectIsPropertySettable(o, &a, &w) && w;
}
static int get_u32(AudioObjectID o, AudioObjectPropertySelector s, AudioObjectPropertyScope sc, UInt32 el, UInt32 *v) {
  AudioObjectPropertyAddress a = {s, sc, el};
  UInt32 sz = sizeof *v;
  return AudioObjectGetPropertyData(o, &a, 0, NULL, &sz, v) == noErr;
}
static int get_f32(AudioObjectID o, AudioObjectPropertySelector s, AudioObjectPropertyScope sc, UInt32 el, Float32 *v) {
  AudioObjectPropertyAddress a = {s, sc, el};
  UInt32 sz = sizeof *v;
  return AudioObjectGetPropertyData(o, &a, 0, NULL, &sz, v) == noErr;
}
static int get_f64(AudioObjectID o, AudioObjectPropertySelector s, AudioObjectPropertyScope sc, UInt32 el, Float64 *v) {
  AudioObjectPropertyAddress a = {s, sc, el};
  UInt32 sz = sizeof *v;
  return AudioObjectGetPropertyData(o, &a, 0, NULL, &sz, v) == noErr;
}
static int set_u32(AudioObjectID o, AudioObjectPropertySelector s, AudioObjectPropertyScope sc, UInt32 el, UInt32 v) {
  AudioObjectPropertyAddress a = {s, sc, el};
  return AudioObjectSetPropertyData(o, &a, 0, NULL, sizeof v, &v) == noErr;
}
static int set_f32(AudioObjectID o, AudioObjectPropertySelector s, AudioObjectPropertyScope sc, UInt32 el, Float32 v) {
  AudioObjectPropertyAddress a = {s, sc, el};
  return AudioObjectSetPropertyData(o, &a, 0, NULL, sizeof v, &v) == noErr;
}

static UInt32 out_channels(AudioObjectID d) {
  AudioObjectPropertyAddress a = {kAudioDevicePropertyStreamConfiguration, kAudioObjectPropertyScopeOutput, kAudioObjectPropertyElementMain};
  UInt32 sz = 0;
  if (AudioObjectGetPropertyDataSize(d, &a, 0, NULL, &sz) || !sz) return 0;
  AudioBufferList *bl = malloc(sz);
  UInt32 n = 0;
  if (!AudioObjectGetPropertyData(d, &a, 0, NULL, &sz, bl))
    for (UInt32 i = 0; i < bl->mNumberBuffers; i++) n += bl->mBuffers[i].mNumberChannels;
  free(bl);
  return n;
}

// The built-in output device whose current data source is the internal
// speaker ('ispk'), or, when the device has no data source property, whose
// name says Speaker(s).
static AudioObjectID find_speaker(char *name, size_t n, char *why, size_t wn) {
  AudioObjectPropertyAddress a = {kAudioHardwarePropertyDevices, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
  UInt32 sz = 0;
  snprintf(why, wn, "no built-in output device");
  if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &a, 0, NULL, &sz) || !sz) return 0;
  AudioObjectID *devs = malloc(sz), found = 0;
  AudioObjectGetPropertyData(kAudioObjectSystemObject, &a, 0, NULL, &sz, devs);
  for (UInt32 i = 0; i < sz / sizeof *devs && !found; i++) {
    UInt32 t = 0;
    if (!get_u32(devs[i], kAudioDevicePropertyTransportType, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain, &t)) continue;
    if (t != kAudioDeviceTransportTypeBuiltIn || out_channels(devs[i]) == 0) continue;
    CFStringRef s = NULL; UInt32 ssz = sizeof s; char nm[256];
    AudioObjectPropertyAddress na = {kAudioObjectPropertyName, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    AudioObjectGetPropertyData(devs[i], &na, 0, NULL, &ssz, &s);
    cfstr(s, nm, sizeof nm); if (s) CFRelease(s);
    UInt32 src = 0;
    if (get_u32(devs[i], kAudioDevicePropertyDataSource, kAudioObjectPropertyScopeOutput, kAudioObjectPropertyElementMain, &src)) {
      char cc[5]; fourcc(src, cc);
      if (src == 'ispk') { found = devs[i]; snprintf(name, n, "%s", nm); }
      else snprintf(why, wn, "built-in output data source is '%s', not the internal speaker ('ispk'): unplug headphones", cc);
    } else if (strstr(nm, "Speaker")) { found = devs[i]; snprintf(name, n, "%s", nm); }
  }
  free(devs);
  return found;
}

#define MAXEL 17
typedef struct { int vol_has[MAXEL], vol_set[MAXEL], mute_has[MAXEL], mute_set[MAXEL]; Float32 vol[MAXEL]; UInt32 mute[MAXEL]; UInt32 nel; } Levels;

static void read_levels(AudioObjectID d, Levels *L) {
  for (UInt32 e = 0; e < L->nel; e++) {
    L->vol_has[e] = has(d, kAudioDevicePropertyVolumeScalar, kAudioObjectPropertyScopeOutput, e) &&
                    get_f32(d, kAudioDevicePropertyVolumeScalar, kAudioObjectPropertyScopeOutput, e, &L->vol[e]);
    L->mute_has[e] = has(d, kAudioDevicePropertyMute, kAudioObjectPropertyScopeOutput, e) &&
                     get_u32(d, kAudioDevicePropertyMute, kAudioObjectPropertyScopeOutput, e, &L->mute[e]);
    L->vol_set[e] = L->vol_has[e] && settable(d, kAudioDevicePropertyVolumeScalar, kAudioObjectPropertyScopeOutput, e);
    L->mute_set[e] = L->mute_has[e] && settable(d, kAudioDevicePropertyMute, kAudioObjectPropertyScopeOutput, e);
  }
}

// 1 when every readable volume element reads 0 and every readable mute
// element reads 1, with at least one of each readable.
static int levels_silent(AudioObjectID d, char *why, size_t wn) {
  Levels L = {.nel = out_channels(d) + 1};
  if (L.nel > MAXEL) L.nel = MAXEL;
  read_levels(d, &L);
  int nv = 0, nm = 0;
  for (UInt32 e = 0; e < L.nel; e++) {
    if (L.vol_has[e]) { nv++; if (L.vol[e] > 1e-6f) { snprintf(why, wn, "volume element %u reads %.4f", e, L.vol[e]); return 0; } }
    if (L.mute_has[e]) { nm++; if (L.mute[e] != 1) { snprintf(why, wn, "mute element %u reads %u", e, L.mute[e]); return 0; } }
  }
  if (!nv) { snprintf(why, wn, "no readable volume element"); return 0; }
  if (!nm) { snprintf(why, wn, "no readable mute element"); return 0; }
  snprintf(why, wn, "%d volume element(s) at 0, %d mute element(s) on", nv, nm);
  return 1;
}

static atomic_ulong g_cb, g_frames, g_overloads;
static OSStatus zero_ioproc(AudioObjectID dev, const AudioTimeStamp *now, const AudioBufferList *in, const AudioTimeStamp *intime,
                            AudioBufferList *out, const AudioTimeStamp *outtime, void *ctx) {
  (void)dev; (void)now; (void)in; (void)intime; (void)outtime; (void)ctx;
  for (UInt32 i = 0; i < out->mNumberBuffers; i++) memset(out->mBuffers[i].mData, 0, out->mBuffers[i].mDataByteSize);
  if (out->mNumberBuffers && out->mBuffers[0].mNumberChannels)
    atomic_fetch_add(&g_frames, out->mBuffers[0].mDataByteSize / (sizeof(Float32) * out->mBuffers[0].mNumberChannels));
  atomic_fetch_add(&g_cb, 1);
  return noErr;
}
static OSStatus overload_listener(AudioObjectID o, UInt32 n, const AudioObjectPropertyAddress *a, void *c) {
  (void)o; (void)n; (void)a; (void)c;
  atomic_fetch_add(&g_overloads, 1);
  return noErr;
}

static void hal_state(AudioObjectID d, const char *tag) {
  UInt32 run = 0, some = 0, buf = 0, lat = 0, saf = 0, src = 0; Float64 rate = 0; char cc[5] = "-";
  get_u32(d, kAudioDevicePropertyDeviceIsRunning, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain, &run);
  get_u32(d, kAudioDevicePropertyDeviceIsRunningSomewhere, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain, &some);
  get_u32(d, kAudioDevicePropertyBufferFrameSize, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain, &buf);
  get_u32(d, kAudioDevicePropertyLatency, kAudioObjectPropertyScopeOutput, kAudioObjectPropertyElementMain, &lat);
  get_u32(d, kAudioDevicePropertySafetyOffset, kAudioObjectPropertyScopeOutput, kAudioObjectPropertyElementMain, &saf);
  get_f64(d, kAudioDevicePropertyNominalSampleRate, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain, &rate);
  if (get_u32(d, kAudioDevicePropertyDataSource, kAudioObjectPropertyScopeOutput, kAudioObjectPropertyElementMain, &src)) fourcc(src, cc);
  printf("DATA hal.%s running=%u running_somewhere=%u rate=%.0f buffer_frames=%u latency=%u safety_offset=%u data_source=%s\n",
         tag, run, some, rate, buf, lat, saf, cc);
  // Physical format of each output stream.
  AudioObjectPropertyAddress a = {kAudioDevicePropertyStreams, kAudioObjectPropertyScopeOutput, kAudioObjectPropertyElementMain};
  UInt32 sz = 0;
  if (!AudioObjectGetPropertyDataSize(d, &a, 0, NULL, &sz) && sz) {
    AudioStreamID *ss = malloc(sz);
    if (!AudioObjectGetPropertyData(d, &a, 0, NULL, &sz, ss))
      for (UInt32 i = 0; i < sz / sizeof *ss; i++) {
        AudioStreamBasicDescription f = {0}; UInt32 fsz = sizeof f; char id[5];
        AudioObjectPropertyAddress fa = {kAudioStreamPropertyPhysicalFormat, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
        if (AudioObjectGetPropertyData(ss[i], &fa, 0, NULL, &fsz, &f)) continue;
        fourcc(f.mFormatID, id);
        printf("DATA hal.%s.stream%u format=%s rate=%.0f channels=%u bits=%u flags=0x%x\n", tag, i, id, f.mSampleRate,
               f.mChannelsPerFrame, f.mBitsPerChannel, f.mFormatFlags);
      }
    free(ss);
  }
  fflush(stdout);
}

// ---------------------------------------------------------------- IORegistry
// Snapshot: "entry-id|class|name|key/path" -> value text, for every IOService
// entry whose class or name is audio-related. Scalars only; data blobs as
// their length; keys that could identify the Mac or its owner are skipped.
typedef struct { char **k, **v; size_t n, cap; } Snap;

static void snap_add(Snap *s, const char *k, const char *v) {
  if (s->n == s->cap) { s->cap = s->cap ? s->cap * 2 : 1024; s->k = realloc(s->k, s->cap * sizeof *s->k); s->v = realloc(s->v, s->cap * sizeof *s->v); }
  s->k[s->n] = strdup(k); s->v[s->n] = strdup(v); s->n++;
}
static const char *snap_get(const Snap *s, const char *k) {
  for (size_t i = 0; i < s->n; i++) if (!strcmp(s->k[i], k)) return s->v[i];
  return NULL;
}
static void snap_free(Snap *s) {
  for (size_t i = 0; i < s->n; i++) { free(s->k[i]); free(s->v[i]); }
  free(s->k); free(s->v); memset(s, 0, sizeof *s);
}

static int ci_contains(const char *h, const char *n) {
  size_t ln = strlen(n);
  for (; *h; h++) if (!strncasecmp(h, n, ln)) return 1;
  return 0;
}
static int any_of(const char *s, const char *const *words) {
  for (; *words; words++) if (ci_contains(s, *words)) return 1;
  return 0;
}
static const char *const AUDIO_WORDS[] = {"audio", "speaker", "amp", "codec", "dsp", "sn01", "tas2", "cs35", "cs42", "ssm3", "lsid", "lsm", "mca", NULL};
static const char *const PRIVATE_WORDS[] = {"serial", "uuid", "mac-address", "macaddress", "ethernet", "ssid", "bssid",
                                            "user", "owner", "account", "email", "host", "location", "imei", "ecid", "udid",
                                            "chip-id", "unique", "board-id", "nonce", "hash", NULL};
// Keys printed in full in every snapshot (estimator/state candidates).
static const char *const STATE_WORDS[] = {"lsid", "lsm", "system id", "systemid", "sysid", "rdc", "resist", "temperat", "therm",
                                          "protect", "estimat", "impedance", "coil", "pilot", "speakerid", "speaker id", "thiele",
                                          "excurs", "displace", "safety", "current state", "is running", NULL};

static void flatten(Snap *s, const char *prefix, CFTypeRef v, int depth) {
  char val[160];
  CFTypeID t = CFGetTypeID(v);
  if (t == CFDictionaryGetTypeID()) {
    if (depth > 3) return;
    CFIndex n = CFDictionaryGetCount(v);
    const void **ks = malloc(sizeof(void *) * (size_t)(n ? n : 1)), **vs = malloc(sizeof(void *) * (size_t)(n ? n : 1));
    CFDictionaryGetKeysAndValues(v, ks, vs);
    for (CFIndex i = 0; i < n; i++) {
      char key[128], path[512];
      if (CFGetTypeID(ks[i]) != CFStringGetTypeID()) continue;
      cfstr(ks[i], key, sizeof key);
      if (any_of(key, PRIVATE_WORDS)) continue;
      snprintf(path, sizeof path, "%s/%s", prefix, key);
      flatten(s, path, vs[i], depth + 1);
    }
    free(ks); free(vs);
    return;
  }
  if (t == CFArrayGetTypeID()) {
    if (depth > 3) return;
    CFIndex n = CFArrayGetCount(v);
    for (CFIndex i = 0; i < n && i < 32; i++) {
      char path[512]; snprintf(path, sizeof path, "%s[%ld]", prefix, (long)i);
      flatten(s, path, CFArrayGetValueAtIndex(v, i), depth + 1);
    }
    return;
  }
  if (t == CFNumberGetTypeID()) {
    if (CFNumberIsFloatType(v)) { double d = 0; CFNumberGetValue(v, kCFNumberDoubleType, &d); snprintf(val, sizeof val, "%.6g", d); }
    else { long long q = 0; CFNumberGetValue(v, kCFNumberLongLongType, &q); snprintf(val, sizeof val, "%lld", q); }
  } else if (t == CFBooleanGetTypeID()) {
    snprintf(val, sizeof val, "%s", CFBooleanGetValue(v) ? "true" : "false");
  } else if (t == CFStringGetTypeID()) {
    char tmp[512]; cfstr(v, tmp, sizeof tmp);
    if (strlen(tmp) > 64) snprintf(val, sizeof val, "string[%zu]", strlen(tmp));
    else snprintf(val, sizeof val, "\"%s\"", tmp);
  } else if (t == CFDataGetTypeID()) {
    snprintf(val, sizeof val, "data[%ld]", (long)CFDataGetLength(v));
  } else return;
  snap_add(s, prefix, val);
}

static void snapshot(Snap *s, int *entries) {
  io_iterator_t it;
  *entries = 0;
  if (IORegistryCreateIterator(kIOMainPortDefault, kIOServicePlane, kIORegistryIterateRecursively, &it) != KERN_SUCCESS) return;
  io_registry_entry_t e;
  while ((e = IOIteratorNext(it))) {
    io_name_t cls = "", nm = "";
    IOObjectGetClass(e, cls);
    IORegistryEntryGetName(e, nm);
    if (any_of(cls, AUDIO_WORDS) || any_of(nm, AUDIO_WORDS)) {
      CFMutableDictionaryRef props = NULL;
      if (IORegistryEntryCreateCFProperties(e, &props, kCFAllocatorDefault, 0) == KERN_SUCCESS && props) {
        uint64_t id = 0; IORegistryEntryGetRegistryEntryID(e, &id);
        char prefix[256]; snprintf(prefix, sizeof prefix, "%llx|%s|%s|", (unsigned long long)id, cls, nm);
        (*entries)++;
        flatten(s, prefix, props, 0);
        CFRelease(props);
      }
    }
    IOObjectRelease(e);
  }
  IOObjectRelease(it);
}

static void print_state_keys(const Snap *s, const char *tag) {
  int n = 0;
  for (size_t i = 0; i < s->n; i++) {
    const char *key = strrchr(s->k[i], '|');
    if (key && !strstr(key, "IOPowerManagement") && any_of(key, STATE_WORDS)) { printf("DATA ioreg.%s %s=%s\n", tag, s->k[i], s->v[i]); n++; }
  }
  printf("DATA ioreg.%s.state_keys=%d\n", tag, n);
  fflush(stdout);
}

static int print_diff(const Snap *a, const Snap *b, const char *tag) {
  int n = 0;
  for (size_t i = 0; i < b->n; i++) {
    const char *old = snap_get(a, b->k[i]);
    if (!old) { printf("DATA ioreg.%s.new %s=%s\n", tag, b->k[i], b->v[i]); n++; }
    else if (strcmp(old, b->v[i])) { printf("DATA ioreg.%s.changed %s: %s -> %s\n", tag, b->k[i], old, b->v[i]); n++; }
  }
  for (size_t i = 0; i < a->n; i++)
    if (!snap_get(b, a->k[i])) { printf("DATA ioreg.%s.gone %s (was %s)\n", tag, a->k[i], a->v[i]); n++; }
  printf("DATA ioreg.%s.differences=%d\n", tag, n);
  fflush(stdout);
  return n;
}

// The AID (device tree acoustic-id, little-endian u32) and whether the per-unit
// calibration property exists (its length only, never its contents).
static void device_tree_ids(void) {
  io_iterator_t it;
  int found = 0;
  if (IORegistryCreateIterator(kIOMainPortDefault, kIODeviceTreePlane, kIORegistryIterateRecursively, &it) != KERN_SUCCESS) {
    result("aid", "FAIL", "cannot iterate the device tree plane");
    return;
  }
  io_registry_entry_t e;
  while ((e = IOIteratorNext(it))) {
    CFTypeRef aid = IORegistryEntryCreateCFProperty(e, CFSTR("acoustic-id"), kCFAllocatorDefault, 0);
    if (aid && CFGetTypeID(aid) == CFDataGetTypeID() && CFDataGetLength(aid) >= 4) {
      const UInt8 *p = CFDataGetBytePtr(aid);
      UInt32 v = (UInt32)p[0] | (UInt32)p[1] << 8 | (UInt32)p[2] << 16 | (UInt32)p[3] << 24;
      CFTypeRef ts = IORegistryEntryCreateCFProperty(e, CFSTR("speaker-thiele-small"), kCFAllocatorDefault, 0);
      long tsl = ts && CFGetTypeID(ts) == CFDataGetTypeID() ? (long)CFDataGetLength(ts) : -1;
      if (ts) CFRelease(ts);
      printf("DATA aid=%u\nDATA speaker_thiele_small_bytes=%ld\n", v, tsl);
      found++;
    }
    if (aid) CFRelease(aid);
    IOObjectRelease(e);
  }
  IOObjectRelease(it);
  if (found == 1) result("aid", "PASS", "one audio node carries acoustic-id");
  else result("aid", "FAIL", "%d nodes carry acoustic-id (expected 1)", found);

  // speakerID of the built-in speaker engine (AppleEmbeddedAudioDevice, device UID "Speaker").
  io_iterator_t si;
  if (IOServiceGetMatchingServices(kIOMainPortDefault, IOServiceMatching("AppleEmbeddedAudioDevice"), &si) == KERN_SUCCESS) {
    io_service_t s;
    while ((s = IOIteratorNext(si))) {
      CFTypeRef uid = IORegistryEntryCreateCFProperty(s, CFSTR("device UID"), kCFAllocatorDefault, 0);
      char u[64]; cfstr(uid, u, sizeof u);
      if (uid) CFRelease(uid);
      if (!strcmp(u, "Speaker")) {
        CFTypeRef sp = IORegistryEntryCreateCFProperty(s, CFSTR("speakerID"), kCFAllocatorDefault, 0);
        long long q = -1;
        if (sp && CFGetTypeID(sp) == CFNumberGetTypeID()) CFNumberGetValue(sp, kCFNumberLongLongType, &q);
        if (sp) CFRelease(sp);
        printf("DATA speaker_id=%lld\n", q);
      }
      IOObjectRelease(s);
    }
    IOObjectRelease(si);
  }
}

// ---------------------------------------------------------------- AudioComponent
// Parameter IDs of the six-channel lsm2 extension that speakerguardd v2's
// compiler appends (our own map, speakerguardd companion::extension_ids()).
static int lsm2_extension_id(UInt32 id) {
  if (id == 3 || id == 21512) return 1;
  for (UInt32 b = 0; b < 15; b++) if (id >= 3109 + 1024 * b && id < 3125 + 1024 * b) return 1;
  for (UInt32 t = 0; t < 3; t++) if (id >= 18448 + 1024 * t && id < 18450 + 1024 * t) return 1;
  if (id >= 24576 && id < 24584) return 1;
  const UInt32 blk[3] = {25600, 26624, 27648};
  for (int i = 0; i < 3; i++) if (id >= blk[i] && id < blk[i] + 4) return 1;
  if (id >= 1048599 && id < 1048604) return 1;
  for (UInt32 c = 0; c < 8; c++) if (id == 1058849 + 1024 * c) return 1;
  if (id >= 1067008 && id < 1067020) return 1;
  if (id >= 1068032 && id < 1068044) return 1;
  return 0;
}
static int cmp_u32(const void *a, const void *b) { UInt32 x = *(const UInt32 *)a, y = *(const UInt32 *)b; return x < y ? -1 : x > y; }

static void component(const char *st) {
  UInt32 sub = (UInt32)st[0] << 24 | (UInt32)st[1] << 16 | (UInt32)st[2] << 8 | (UInt32)st[3];
  AudioComponentDescription d = {kAudioUnitType_Effect, sub, kAudioUnitManufacturer_Apple, 0, 0};
  AudioComponent c = AudioComponentFindNext(NULL, &d);
  char step[32]; snprintf(step, sizeof step, "component.%s", st);
  if (!c) { result(step, "INFO", "not registered (AudioComponentFindNext: none for aufx/%s/appl)", st); return; }
  UInt32 ver = 0; AudioComponentGetVersion(c, &ver);
  CFStringRef nm = NULL; char name[128] = "";
  if (!AudioComponentCopyName(c, &nm) && nm) { cfstr(nm, name, sizeof name); CFRelease(nm); }
  AudioUnit au = NULL;
  OSStatus s = AudioComponentInstanceNew(c, &au);
  if (s || !au) { result(step, "INFO", "registered '%s' version 0x%x; instance refused (%d)", name, ver, (int)s); return; }
  UInt32 sz = 0; Boolean w = 0; UInt32 n = 0, ext = 0; char hex[65] = "-";
  if (!AudioUnitGetPropertyInfo(au, kAudioUnitProperty_ParameterList, kAudioUnitScope_Global, 0, &sz, &w) && sz) {
    UInt32 *ids = malloc(sz);
    if (!AudioUnitGetProperty(au, kAudioUnitProperty_ParameterList, kAudioUnitScope_Global, 0, ids, &sz)) {
      n = sz / sizeof *ids;
      qsort(ids, n, sizeof *ids, cmp_u32);
      for (UInt32 i = 0; i < n; i++) ext += lsm2_extension_id(ids[i]);
      unsigned char md[CC_SHA256_DIGEST_LENGTH];
      CC_SHA256(ids, (CC_LONG)(n * sizeof *ids), md);
      for (int i = 0; i < CC_SHA256_DIGEST_LENGTH; i++) snprintf(hex + 2 * i, 3, "%02x", md[i]);
    }
    free(ids);
  }
  AudioComponentInstanceDispose(au);
  result(step, "INFO", "registered '%s' version 0x%x; %u global parameters (id list sha256 %s); %u in the six-channel lsm2 extension map",
         name, ver, n, hex, ext);
}

// ---------------------------------------------------------------- main
static double now_s(void) {
  static mach_timebase_info_data_t tb;
  if (!tb.denom) mach_timebase_info(&tb);
  return (double)mach_absolute_time() * tb.numer / tb.denom / 1e9;
}

int main(int argc, char **argv) {
  int seconds = 30, stream = 1, leave = 0;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--seconds") && i + 1 < argc) seconds = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--no-stream")) stream = 0;
    else if (!strcmp(argv[i], "--leave-muted")) leave = 1;
    else { fprintf(stderr, "usage: lsidprobe [--seconds N] [--no-stream] [--leave-muted]\n"); return 2; }
  }
  if (seconds < 5 || seconds > 120) { fprintf(stderr, "--seconds must be 5..120\n"); return 2; }
  setvbuf(stdout, NULL, _IOLBF, 0);
  printf("DATA probe=lsidprobe-1 stream=%d seconds=%d\n", stream, seconds);

  device_tree_ids();
  component("lsid");
  component("lsm2");
  component("spp3");
  component("atsp");

  char name[256] = "", why[256] = "";
  AudioObjectID dev = find_speaker(name, sizeof name, why, sizeof why);
  if (!dev) { result("speaker_device", "FAIL", "%s", why); g_safety_fail = 1; }
  else result("speaker_device", "PASS", "built-in '%s' (internal speaker)", name);

  Snap before = {0}, during = {0}, after = {0};
  int ents = 0;
  snapshot(&before, &ents);
  printf("DATA ioreg.audio_entries=%d keys=%zu\n", ents, before.n);
  print_state_keys(&before, "before");
  if (dev) hal_state(dev, "before");

  if (!stream || !dev) {
    result("muted_stream", "SKIP", stream ? "no internal speaker device" : "--no-stream");
    snap_free(&before);
    return g_safety_fail;
  }

  // Volume 0 and mute on, then read back.
  Levels saved = {.nel = out_channels(dev) + 1};
  if (saved.nel > MAXEL) saved.nel = MAXEL;
  read_levels(dev, &saved);
  for (UInt32 e = 0; e < saved.nel; e++) {
    if (saved.mute_set[e]) set_u32(dev, kAudioDevicePropertyMute, kAudioObjectPropertyScopeOutput, e, 1);
    if (saved.vol_set[e]) set_f32(dev, kAudioDevicePropertyVolumeScalar, kAudioObjectPropertyScopeOutput, e, 0.0f);
  }
  usleep(300000);
  if (!levels_silent(dev, why, sizeof why)) {
    result("volume_mute", "FAIL", "%s: no stream started", why);
    g_safety_fail = 1;
    snap_free(&before);
    return 1;
  }
  result("volume_mute", "PASS", "%s", why);

  AudioObjectPropertyAddress ola = {kAudioDeviceProcessorOverload, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
  AudioObjectAddPropertyListener(dev, &ola, overload_listener, NULL);
  AudioDeviceIOProcID pid = NULL;
  if (AudioDeviceCreateIOProcID(dev, zero_ioproc, NULL, &pid) || AudioDeviceStart(dev, pid)) {
    result("muted_stream", "FAIL", "could not start the zero stream");
    if (pid) AudioDeviceDestroyIOProcID(dev, pid);
  } else {
    double t0 = now_s(), next_snap = t0 + 2.0;
    int snaps = 0, stopped_early = 0;
    printf("DATA stream.start\n");
    while (now_s() - t0 < seconds) {
      usleep(200000);
      UInt32 src = 0;
      int src_ok = !get_u32(dev, kAudioDevicePropertyDataSource, kAudioObjectPropertyScopeOutput, kAudioObjectPropertyElementMain, &src) || src == 'ispk';
      if (!levels_silent(dev, why, sizeof why) || !src_ok) {
        AudioDeviceStop(dev, pid);
        result("watchdog", "FAIL", "%s at %.1f s: stream stopped", src_ok ? why : "output left the internal speaker", now_s() - t0);
        g_safety_fail = 1; stopped_early = 1;
        break;
      }
      if (now_s() >= next_snap && snaps < 3) {
        Snap s = {0}; int en = 0;
        snapshot(&s, &en);
        char tag[16]; snprintf(tag, sizeof tag, "t%.0f", now_s() - t0);
        print_diff(snaps ? &during : &before, &s, tag);
        if (snaps == 1) print_state_keys(&s, tag);
        snap_free(&during); during = s;
        hal_state(dev, tag);
        snaps++;
        next_snap = t0 + (snaps == 1 ? seconds / 2.0 : seconds - 1.5);
      }
    }
    if (!stopped_early) {
      AudioDeviceStop(dev, pid);
      result("watchdog", "PASS", "volume 0 and mute held for the whole stream (checked every 200 ms)");
    }
    double dur = now_s() - t0;
    AudioDeviceDestroyIOProcID(dev, pid);
    result("muted_stream", stopped_early ? "FAIL" : "PASS", "%.1f s of digital silence: %lu IOProc calls, %lu frames, %lu overloads",
           dur, (unsigned long)atomic_load(&g_cb), (unsigned long)atomic_load(&g_frames), (unsigned long)atomic_load(&g_overloads));
  }
  AudioObjectRemovePropertyListener(dev, &ola, overload_listener, NULL);

  sleep(2);
  snapshot(&after, &ents);
  print_diff(during.n ? &during : &before, &after, "after");
  hal_state(dev, "after");

  if (!leave) {
    // Volume first while still muted, then mute last.
    for (UInt32 e = 0; e < saved.nel; e++)
      if (saved.vol_set[e]) set_f32(dev, kAudioDevicePropertyVolumeScalar, kAudioObjectPropertyScopeOutput, e, saved.vol[e]);
    for (UInt32 e = 0; e < saved.nel; e++)
      if (saved.mute_set[e]) set_u32(dev, kAudioDevicePropertyMute, kAudioObjectPropertyScopeOutput, e, saved.mute[e]);
    result("restore", "INFO", "previous volume/mute restored (nothing is playing)");
  } else {
    result("restore", "INFO", "--leave-muted: volume stays 0, mute on");
  }
  snap_free(&before); snap_free(&during); snap_free(&after);
  return g_safety_fail;
}
