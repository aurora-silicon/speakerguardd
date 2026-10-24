/* SPDX-License-Identifier: MIT */
/*
 * Randomized tests of the excursion limiter against an oracle written apart
 * from it: the driver's continuous-time equation of motion,
 *
 *   Mms x'' + (Rms + Bl^2/Re) x' + Kms x = (Bl/Re) v(t),
 *
 * integrated with fourth-order Runge-Kutta, eight steps a sample, the
 * voltage interpolated linearly between samples (what the amplifier's
 * reconstruction does, near enough below a few kHz).  The limiter's own
 * predictor is a bilinear-mapped biquad; the oracle shares none of its code.
 *
 * Every case is random signals (tones, bursts out of silence, chirps, square
 * waves, steps, impulses, noise, in any mix, each channel its own), random
 * block sizes and, for some, another window and release.  After each case:
 *
 *  (a) the oracle's cone never passes Xmax on either channel;
 *  (b) no output sample passes the sample-peak ceiling;
 *  (c) the output is the input delayed by the reported latency, times a
 *      gain in [0, 1]: never louder, never another sign;
 *  (d) where the limiter never took gain, the output is the delayed input,
 *      bit for bit.
 *
 * XLIM_CASES (default 300) and XLIM_SEED (default 1) set the run.
 */
#include "xlim_core.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RATE 48000.0

static int failures;

/* xorshift64*: the cases depend only on the seed */
static uint64_t rs;
static double urand(void)
{
	rs ^= rs >> 12;
	rs ^= rs << 25;
	rs ^= rs >> 27;
	return (double)((rs * 2685821657736338717ull) >> 11) / 9007199254740992.0;
}
static double range(double a, double b) { return a + (b - a) * urand(); }

/* The oracle: the cone of one channel driven by the output. */
struct oracle {
	double m, r, k, g;   /* Mms, Rms + Bl^2/Re, Kms, Bl/Re */
	double x, u;         /* displacement, velocity */
	double vprev;        /* the previous sample's voltage */
	double xpk, upk;
};

static void oracle_init(struct oracle *o, const struct xlim_params *p)
{
	memset(o, 0, sizeof(*o));
	o->m = p->mms;
	o->r = p->rms + p->bl * p->bl / p->re;
	o->k = p->kms;
	o->g = p->bl / p->re;
}

static inline void deriv(const struct oracle *o, double x, double u, double v, double *dx, double *du)
{
	*dx = u;
	*du = (o->g * v - o->r * u - o->k * x) / o->m;
}

/* One sample: the voltage ramps from the previous sample's to this one. */
static void oracle_step(struct oracle *o, double v)
{
	const int sub = 8;
	const double h = 1.0 / (RATE * sub);
	int i;

	for (i = 0; i < sub; i++) {
		double v0 = o->vprev + (v - o->vprev) * i / sub;
		double vm = o->vprev + (v - o->vprev) * (i + 0.5) / sub;
		double v1 = o->vprev + (v - o->vprev) * (i + 1.0) / sub;
		double k1x, k1u, k2x, k2u, k3x, k3u, k4x, k4u;

		deriv(o, o->x, o->u, v0, &k1x, &k1u);
		deriv(o, o->x + 0.5 * h * k1x, o->u + 0.5 * h * k1u, vm, &k2x, &k2u);
		deriv(o, o->x + 0.5 * h * k2x, o->u + 0.5 * h * k2u, vm, &k3x, &k3u);
		deriv(o, o->x + h * k3x, o->u + h * k3u, v1, &k4x, &k4u);
		o->x += h / 6.0 * (k1x + 2 * k2x + 2 * k3x + k4x);
		o->u += h / 6.0 * (k1u + 2 * k2u + 2 * k3u + k4u);
		if (fabs(o->x) > o->xpk) o->xpk = fabs(o->x);
		if (fabs(o->u) > o->upk) o->upk = fabs(o->u);
	}
	o->vprev = v;
}

/* A random program for one channel, n frames. */
static void program(float *s, size_t n)
{
	double *acc = calloc(n, sizeof(double));
	int parts = 1 + (int)(urand() * 4), j;
	size_t i;

	for (j = 0; j < parts; j++) {
		int kind = (int)(urand() * 7);
		double a = urand() < 0.3 ? range(0.9, 1.3) : range(0.0, 1.0);
		size_t on = urand() < 0.5 ? 0 : (size_t)(urand() * n * 0.7);
		size_t off = urand() < 0.5 ? n : on + (size_t)(urand() * (n - on));
		/* near resonance (the worst place) for kind 1, else anywhere */
		double f = kind == 1 ? range(400.0, 650.0) : exp(range(log(20.0), log(3000.0)));
		double ph = range(0, 2 * M_PI);

		for (i = on; i < off; i++) {
			double t = (double)(i - on) / RATE, v = 0;
			switch (kind) {
			case 0: /* a tone, its edges abrupt */
			case 1:
				v = a * sin(2 * M_PI * f * t + ph);
				break;
			case 2: { /* a chirp, 20 Hz to 2 kHz over the part */
				double dur = (double)(off - on) / RATE + 1e-9;
				double k = log(2000.0 / 20.0) / dur;
				v = a * sin(2 * M_PI * 20.0 * (exp(k * t) - 1) / k + ph);
				break;
			}
			case 3: /* a square wave: its edges ring the cone */
				v = a * (sin(2 * M_PI * f * 0.2 * t + ph) >= 0 ? 1.0 : -1.0);
				break;
			case 4: /* a step held to the part's end */
				v = (ph < M_PI ? a : -a);
				break;
			case 5: /* impulses */
				v = urand() < 0.002 ? (urand() < 0.5 ? a : -a) : 0.0;
				break;
			default: /* noise */
				v = a * range(-1.0, 1.0);
				break;
			}
			acc[i] += v;
		}
	}
	for (i = 0; i < n; i++)
		s[i] = (float)fmax(-1.5, fmin(1.5, acc[i]));
	free(acc);
}

static int run_case(int id, uint64_t seed)
{
	struct xlim_params p;
	struct xlim l;
	size_t n, i, c, d;
	float *in[XLIM_CHANNELS], *out[XLIM_CHANNELS];
	int bad = 0;

	rs = seed * 0x9e3779b97f4a7c15ull + 1;
	xlim_params_j700(&p, RATE);
	if (urand() < 0.3) {
		p.window = (size_t)(range(0.002, 0.010) * RATE);
		p.release = range(0.02, 0.5);
	}
	n = (size_t)(range(0.15, 0.6) * RATE);
	for (c = 0; c < XLIM_CHANNELS; c++) {
		in[c] = calloc(n, sizeof(float));
		out[c] = calloc(n, sizeof(float));
	}
	program(in[0], n);
	if (urand() < 0.5)
		memcpy(in[1], in[0], n * sizeof(float));
	else
		program(in[1], n);
	if (xlim_init(&l, &p) != 0) {
		printf("FAIL case %d: init\n", id);
		return 1;
	}
	for (i = 0; i < n;) {
		size_t b = 1 + (size_t)(urand() * (urand() < 0.2 ? 4096 : 300));
		if (b > n - i)
			b = n - i;
		const float *bi[XLIM_CHANNELS] = { in[0] + i, in[1] + i };
		float *bo[XLIM_CHANNELS] = { out[0] + i, out[1] + i };
		xlim_run(&l, bi, bo, b);
		i += b;
	}
	d = p.window + p.hold - 1;
	for (c = 0; c < XLIM_CHANNELS; c++) {
		struct oracle o;

		oracle_init(&o, &p);
		for (i = 0; i < n; i++)
			oracle_step(&o, out[c][i] * p.v_fs[c]);
		/* (a) */
		if (o.xpk > p.x_max) {
			printf("FAIL case %d (seed %llu) ch %zu: cone %.4f mm, limit %.3f\n", id, (unsigned long long)seed, c, 1e3 * o.xpk, 1e3 * p.x_max);
			bad = 1;
		}
		for (i = 0; i < n; i++) {
			double y = out[c][i], x = i >= d ? in[c][i - d] : 0.0;
			/* (b) */
			if (fabs(y) > p.peak + 1e-6) {
				printf("FAIL case %d ch %zu: sample %zu at %.6f FS\n", id, c, i, y);
				bad = 1;
				break;
			}
			/* (c) */
			if (fabs(y) > fabs(x) * (1 + 1e-6) + 1e-9 || y * x < 0) {
				printf("FAIL case %d ch %zu: sample %zu is %.9g, the delayed input %.9g\n", id, c, i, y, x);
				bad = 1;
				break;
			}
			/* (d) */
			if (l.min_gain == 1.0 && y != x) {
				printf("FAIL case %d ch %zu: gain never moved, yet sample %zu is %.9g, not %.9g\n", id, c, i, y, x);
				bad = 1;
				break;
			}
		}
	}
	xlim_free(&l);
	for (c = 0; c < XLIM_CHANNELS; c++) {
		free(in[c]);
		free(out[c]);
	}
	return bad;
}

int main(void)
{
	const char *ec = getenv("XLIM_CASES"), *es = getenv("XLIM_SEED");
	int cases = ec ? atoi(ec) : 300;
	uint64_t seed0 = es ? strtoull(es, NULL, 10) : 1;
	struct xlim_params p;
	int id;

	/* the oracle itself: DC and the resonance peak, from the equation */
	xlim_params_j700(&p, RATE);
	{
		struct oracle o;
		size_t i;
		double f0 = sqrt(p.kms / p.mms) / (2 * M_PI);
		oracle_init(&o, &p);
		for (i = 0; i < 48000; i++)
			oracle_step(&o, 1.0);
		if (fabs(o.x - p.bl / (p.re * p.kms)) > 1e-9) {
			printf("FAIL oracle DC %g m/V\n", o.x);
			failures++;
		}
		oracle_init(&o, &p);
		for (i = 0; i < 48000; i++) {
			if (i == 24000)
				o.xpk = 0;
			oracle_step(&o, sin(2 * M_PI * f0 * (double)i / RATE));
		}
		double want = (p.bl / p.re) / ((p.rms + p.bl * p.bl / p.re) * 2 * M_PI * f0);
		if (fabs(o.xpk - want) / want > 0.01) {
			printf("FAIL oracle at f0: %g vs %g m/V\n", o.xpk, want);
			failures++;
		}
	}
	for (id = 0; id < cases; id++)
		failures += run_case(id, seed0 + (uint64_t)id);
	printf("random: %d cases from seed %llu, %s (%d failures)\n", cases, (unsigned long long)seed0, failures ? "FAILED" : "ok", failures);
	return failures != 0;
}
