/* SPDX-License-Identifier: MIT */
/* Host-side test of the LADSPA wrapper: dlopen, controls as a filter-chain
 * sets them, a full-scale tone at resonance must come out limited and a
 * -20 dBFS tone untouched; the latency port reports the delay. */
#include <dlfcn.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef float D;
typedef struct Desc {
	unsigned long id; const char *label; int props; const char *name, *maker, *copy;
	unsigned long nports; const int *pd; const char *const *pn; const void *ph; void *impl;
	void *(*inst)(const struct Desc *, unsigned long);
	void (*conn)(void *, unsigned long, D *);
	void (*act)(void *);
	void (*run)(void *, unsigned long);
	void *ra, *srag;
	void (*deact)(void *);
	void (*clean)(void *);
} Desc;

int main(int argc, char **argv)
{
	void *so = dlopen(argc > 1 ? argv[1] : "./speakerguard-xlim.so", RTLD_NOW);
	if (!so) { printf("FAIL dlopen %s\n", dlerror()); return 1; }
	const Desc *(*fn)(unsigned long) = (const Desc *(*)(unsigned long))dlsym(so, "ladspa_descriptor");
	const Desc *d = fn(0);
	if (!d || fn(1)) { printf("FAIL descriptor\n"); return 1; }
	printf("%s: %lu ports\n", d->label, d->nports);
	D ctl[32] = {0};
	float vals[][2] = { {4, 7.30}, {5, 7.33}, {6, 4.3}, {7, 0.74}, {8, 0.16}, {9, 1720}, {10, 0.14}, {11, 0.565}, {12, 0}, {13, 1.0}, {14, 0.9}, {15, 200}, {16, 4} };
	enum { N = 48000 };
	static float in[N], out[N], out2[N];
	int fails = 0;
	for (int t = 0; t < 2; t++) {
		double a = t ? 1.0 : 0.1;
		void *h = d->inst(d, 48000);
		for (unsigned k = 0; k < sizeof(vals) / sizeof(vals[0]); k++) { ctl[(int)vals[k][0]] = vals[k][1]; d->conn(h, (unsigned long)vals[k][0], &ctl[(int)vals[k][0]]); }
		d->conn(h, 17, &ctl[17]); d->conn(h, 18, &ctl[18]);
		for (int i = 0; i < N; i++) in[i] = (float)(a * sin(2 * M_PI * 522.0 * i / 48000.0));
		d->conn(h, 0, in); d->conn(h, 1, in); d->conn(h, 2, out); d->conn(h, 3, out2);
		d->act(h);
		double gmin = 0;
		for (int i = 0; i < N; i += 256) {
			d->conn(h, 0, in + i); d->conn(h, 1, in + i); d->conn(h, 2, out + i); d->conn(h, 3, out2 + i);
			d->run(h, 256 < N - i ? 256 : N - i);
			if (ctl[17] < gmin) gmin = ctl[17];
		}
		double pk = 0;
		for (int i = N / 2; i < N; i++) pk = fmax(pk, fabs(out[i]));
		printf("%.2f FS at 522 Hz: output peak %.3f FS, min gain %.1f dB, latency %.0f frames\n", a, pk, gmin, ctl[18]);
		if (t == 0 && (gmin != 0.0 || fabs(pk - 0.1) > 1e-3)) { printf("FAIL quiet tone changed\n"); fails++; }
		/* 0.565 mm / (0.196 mm/V * 7.3 V) = 0.395 FS; held at 90% of the limit */
		if (t == 1 && !(pk < 0.40 && pk > 0.30)) { printf("FAIL loud tone not limited to ~0.36 FS\n"); fails++; }
		if (ctl[18] != 191) { printf("FAIL latency %g\n", ctl[18]); fails++; }
		d->clean(h);
	}
	printf("%s\n", fails ? "FAILED" : "ok");
	return fails != 0;
}
