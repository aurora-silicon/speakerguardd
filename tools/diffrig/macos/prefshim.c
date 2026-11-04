// SPDX-License-Identifier: MIT
// Rig preference shim: answers CoreFoundation preference lookups inside the rig host process
// only, so the protection units' own logging switches can be enabled without touching the
// user's real defaults. Every key looked up is reported on stderr ("prefshim: ...").
// Values come from the environment: RIGPREF_<key>=<integer or string>.
// Load with DYLD_INSERT_LIBRARIES=prefshim.dylib (our own unsigned host binary).
#include <CoreFoundation/CoreFoundation.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define INTERPOSE(rep, orig) \
    __attribute__((used)) static struct { const void *r, *o; } _i_##orig \
    __attribute__((section("__DATA,__interpose"))) = {(const void *)rep, (const void *)orig};

static const char *lookup(CFStringRef key, CFStringRef app, char *kbuf, size_t n) {
    kbuf[0] = 0;
    if (key) CFStringGetCString(key, kbuf, (CFIndex)n, kCFStringEncodingUTF8);
    char abuf[256] = "";
    if (app) CFStringGetCString(app, abuf, sizeof abuf, kCFStringEncodingUTF8);
    char env[400];
    snprintf(env, sizeof env, "RIGPREF_%s", kbuf);
    const char *v = getenv(env);
    if (getenv("RIGPREF_TRACE")) fprintf(stderr, "prefshim: key=%s app=%s -> %s\n", kbuf, abuf, v ? v : "(none)");
    return v;
}

static CFPropertyListRef mk(const char *v) {
    char *end;
    if (!strncmp(v, "a:", 2)) { // a:1.0,2.0,... -> CFArray of CFNumber(double)
        CFMutableArrayRef a = CFArrayCreateMutable(NULL, 0, &kCFTypeArrayCallBacks);
        const char *q = v + 2;
        while (*q) { double d = strtod(q, &end); CFNumberRef n = CFNumberCreate(NULL, kCFNumberDoubleType, &d);
            CFArrayAppendValue(a, n); CFRelease(n); q = *end == ',' ? end + 1 : end; if (end == q && !*q) break; if (!*end) break; }
        return a;
    }
    long l = strtol(v, &end, 0);
    if (*v && !*end) return CFNumberCreate(NULL, kCFNumberLongType, &l);
    double d = strtod(v, &end);
    if (*v && !*end) return CFNumberCreate(NULL, kCFNumberDoubleType, &d);
    if (!strcmp(v, "true")) return CFRetain(kCFBooleanTrue);
    if (!strcmp(v, "false")) return CFRetain(kCFBooleanFalse);
    return CFStringCreateWithCString(NULL, v, kCFStringEncodingUTF8);
}

static CFPropertyListRef my_CopyAppValue(CFStringRef key, CFStringRef app) {
    char k[256]; const char *v = lookup(key, app, k, sizeof k);
    return v ? mk(v) : CFPreferencesCopyAppValue(key, app);
}
static CFPropertyListRef my_CopyValue(CFStringRef key, CFStringRef app, CFStringRef u, CFStringRef h) {
    char k[256]; const char *v = lookup(key, app, k, sizeof k);
    return v ? mk(v) : CFPreferencesCopyValue(key, app, u, h);
}
static CFIndex my_GetInt(CFStringRef key, CFStringRef app, Boolean *ok) {
    char k[256]; const char *v = lookup(key, app, k, sizeof k);
    if (v) { if (ok) *ok = true; return strtol(v, NULL, 0); }
    return CFPreferencesGetAppIntegerValue(key, app, ok);
}
static Boolean my_GetBool(CFStringRef key, CFStringRef app, Boolean *ok) {
    char k[256]; const char *v = lookup(key, app, k, sizeof k);
    if (v) { if (ok) *ok = true; return !strcmp(v, "true") || strtol(v, NULL, 0) != 0; }
    return CFPreferencesGetAppBooleanValue(key, app, ok);
}
INTERPOSE(my_CopyAppValue, CFPreferencesCopyAppValue)
INTERPOSE(my_CopyValue, CFPreferencesCopyValue)
INTERPOSE(my_GetInt, CFPreferencesGetAppIntegerValue)
INTERPOSE(my_GetBool, CFPreferencesGetAppBooleanValue)

// ---- path redirection: RIGREDIR_FROM=<prefix> RIGREDIR_TO=<dir> (log files land in the rig dir) ----
#include <fcntl.h>
#include <stdarg.h>
#include <string.h>
#include <sys/stat.h>
static const char *redir(const char *p, char *buf, size_t n) {
    const char *f = getenv("RIGREDIR_FROM"), *t = getenv("RIGREDIR_TO");
    if (!p || !f || !t) return p;
    size_t fl = strlen(f);
    if (strncmp(p, f, fl)) return p;
    snprintf(buf, n, "%s%s", t, p + fl);
    if (getenv("RIGPREF_TRACE")) fprintf(stderr, "prefshim: redirect %s -> %s\n", p, buf);
    return buf;
}
static int my_open(const char *p, int fl, ...) {
    char b[2048]; mode_t m = 0;
    if (fl & O_CREAT) { va_list ap; va_start(ap, fl); m = (mode_t)va_arg(ap, int); va_end(ap); }
    return open(redir(p, b, sizeof b), fl, m);
}
static int my_mkdir(const char *p, mode_t m) { char b[2048]; return mkdir(redir(p, b, sizeof b), m); }
static FILE *my_fopen(const char *p, const char *mo) { char b[2048]; return fopen(redir(p, b, sizeof b), mo); }
static int my_stat(const char *p, struct stat *s) { char b[2048]; return stat(redir(p, b, sizeof b), s); }
static int my_access(const char *p, int m) { char b[2048]; return access(redir(p, b, sizeof b), m); }
INTERPOSE(my_open, open)
INTERPOSE(my_mkdir, mkdir)
INTERPOSE(my_fopen, fopen)
INTERPOSE(my_stat, stat)
INTERPOSE(my_access, access)
