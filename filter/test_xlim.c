/* SPDX-License-Identifier: MIT */
/* Tests for the excursion limiter: the model, transparency below the limits,
 * the limits held on tones and bursts at and around resonance, latency. */
#include "xlim_core.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define RATE 48000.0

/* Peak displacement and velocity of channel 0's output, through an
 * independent copy of the model (what the cone actually does). */
struct cone { struct xlim_biquad x, u; double vfs, xpk, upk; };

static void cone_init(struct cone *c, const struct xlim_params *p)
{
	struct xlim l;
	xlim_init(&l, p);
	c->x = l.x[0];
	c->u = l.u[0];
	c->x.z1 = c->x.z2 = c->u.z1 = c->u.z2 = 0;
	c->vfs = p->v_fs[0];
	c->xpk = c->upk = 0;
	xlim_free(&l);
}

static void cone_feed(struct cone *c, const float *s, size_t n, size_t skip)
{
	size_t i;
	for (i = 0; i < n; i++) {
		double v = s[i] * c->vfs;
		double x = fabs(xlim_model_x(&c->x, v));
		double u = fabs(xlim_model_x(&c->u, v));
		if (i >= skip) {
			if (x > c->xpk) c->xpk = x;
			if (u > c->upk) c->upk = u;
		}
	}
}

/* Run a tone (amplitude a FS, frequency f) for secs with an optional silent
 * lead-in; returns the output buffer for channel 0. */
static float *run_tone(struct xlim_params *p, double a, double f, double lead, double secs, double *min_gain)
{
	struct xlim l;
	size_t n = (size_t)((lead + secs) * RATE), i;
	float *in = calloc(n, sizeof(float)), *out = calloc(n, sizeof(float));
	const float *ins[2] = { in, in };
	float *outs[2] = { out, calloc(n, sizeof(float)) };

	for (i = (size_t)(lead * RATE); i < n; i++)
		in[i] = (float)(a * sin(2 * M_PI * f * (double)i / RATE));
	if (xlim_init(&l, p)) { printf("init failed\n"); exit(2); }
	/* odd block sizes, as a host may use */
	for (i = 0; i < n;) {
		size_t b = 1 + (i * 7919) % 333;
		if (b > n - i) b = n - i;
		const float *bi[2] = { ins[0] + i, ins[1] + i };
		float *bo[2] = { outs[0] + i, outs[1] + i };
		xlim_run(&l, bi, bo, b);
		i += b;
	}
	*min_gain = l.min_gain;
	xlim_free(&l);
	free(in);
	free(outs[1]);
	return out;
}

int main(void)
{
	struct xlim_params p;
	double f0, mg;
	size_t i;

	xlim_params_j700(&p, RATE);
	f0 = sqrt(p.kms / p.mms) / (2 * M_PI);

	/* the model: 0.1 mm/V below resonance, Qts ~1.96 at 522 Hz */
	{
		struct xlim l;
		double x = 0;
		xlim_init(&l, &p);
		for (i = 0; i < 48000; i++)
			x = xlim_model_x(&l.x[0], 1.0);
		CHECK(fabs(x - p.bl / (p.re * p.kms)) < 1e-7, "DC %g m/V", x);
		xlim_free(&l);
		struct cone c;
		cone_init(&c, &p);
		float *t = calloc(48000, sizeof(float));
		for (i = 0; i < 48000; i++) t[i] = (float)(sin(2 * M_PI * f0 * i / RATE) / p.v_fs[0]);
		cone_feed(&c, t, 48000, 24000);
		double want = (p.bl / p.re) / ((p.rms + p.bl * p.bl / p.re) * 2 * M_PI * f0);
		CHECK(fabs(c.xpk - want) / want < 0.01, "|H(f0)| %g vs %g m/V", c.xpk, want);
		printf("model: f0 %.0f Hz, %.3f mm/V below it, %.3f mm/V at it\n", f0, 1e3 * p.bl / (p.re * p.kms), 1e3 * c.xpk);
		free(t);
	}

	/* transparency: -20 dBFS sines anywhere in the band pass untouched (delayed) */
	{
		double fs[] = { 50, 150, 300, 522, 1000, 4000, 12000 };
		for (size_t k = 0; k < sizeof(fs) / sizeof(fs[0]); k++) {
			float *o = run_tone(&p, 0.1, fs[k], 0.0, 0.5, &mg);
			CHECK(mg == 1.0, "%.0f Hz at -20 dBFS: gain dipped to %g", fs[k], mg);
			size_t d = p.window + p.hold - 1;
			double err = 0;
			for (i = d; i < (size_t)(0.5 * RATE); i++) {
				double want = 0.1 * sin(2 * M_PI * fs[k] * (double)(i - d) / RATE);
				err = fmax(err, fabs(o[i] - want));
			}
			CHECK(err < 1e-6, "%.0f Hz: output differs from the delayed input by %g", fs[k], err);
			free(o);
		}
	}

	/* the displacement limit holds on loud tones and on bursts out of silence */
	{
		double fs[] = { 60, 150, 300, 400, 522, 650, 800, 1200 };
		double as[] = { 0.25, 0.5, 1.0 };
		for (size_t k = 0; k < sizeof(fs) / sizeof(fs[0]); k++)
			for (size_t j = 0; j < 3; j++) {
				struct cone c;
				float *o = run_tone(&p, as[j], fs[k], 0.05, 0.6, &mg);
				cone_init(&c, &p);
				cone_feed(&c, o, (size_t)(0.65 * RATE), 0);
				CHECK(c.xpk <= p.x_max * 1.0001, "%.0f Hz at %.2f FS: cone reached %.3f mm (limit %.3f), gain %g",
				      fs[k], as[j], 1e3 * c.xpk, 1e3 * p.x_max, mg);
				if (fs[k] == 522 && as[j] == 1.0)
					printf("522 Hz full scale: gain %.1f dB, cone %.3f mm (unlimited %.3f mm)\n", 20 * log10(mg), 1e3 * c.xpk,
					       1e3 * p.v_fs[0] * 1.963e-4);
				free(o);
			}
	}

	/* velocity: held when enabled */
	{
		struct xlim_params q = p;
		struct cone c;
		q.v_max = 0.6;
		float *o = run_tone(&q, 0.5, 1500, 0.05, 0.4, &mg);
		cone_init(&c, &q);
		cone_feed(&c, o, (size_t)(0.45 * RATE), 0);
		CHECK(c.upk <= q.v_max * 1.0001, "velocity %.3f m/s over %.3f", c.upk, q.v_max);
		printf("1.5 kHz at -6 dBFS with a 0.6 m/s limit: gain %.1f dB, cone velocity %.3f m/s\n", 20 * log10(mg), c.upk);
		free(o);
	}

	/* a music-like signal: noise with a bass line, all limits, random blocks */
	{
		struct xlim l;
		struct cone c;
		size_t n = 3 * 48000;
		float *a = calloc(n, sizeof(float)), *b = calloc(n, sizeof(float));
		const float *ins[2] = { a, a };
		float *outs[2] = { b, calloc(n, sizeof(float)) };
		unsigned s = 12345;
		for (i = 0; i < n; i++) {
			s = s * 1103515245u + 12345u;
			double noise = ((s >> 8) & 0xffff) / 32768.0 - 1.0;
			a[i] = (float)fmax(-1.0, fmin(1.0, 0.25 * noise + 0.7 * sin(2 * M_PI * 110 * i / RATE) * (i % 24000 < 12000)));
		}
		xlim_init(&l, &p);
		xlim_run(&l, ins, outs, n);
		cone_init(&c, &p);
		cone_feed(&c, b, n, 0);
		CHECK(c.xpk <= p.x_max * 1.0001, "music-like: cone %.3f mm", 1e3 * c.xpk);
		printf("music-like: min gain %.1f dB, cone %.3f mm\n", 20 * log10(l.min_gain), 1e3 * c.xpk);
		xlim_free(&l);
		free(a); free(b); free(outs[1]);
	}

	/* latency: an impulse comes out exactly window + hold - 1 frames later,
	 * unchanged, and the J700's total is inside the 352-frame budget */
	{
		struct xlim l;
		size_t n = 2000, lat = p.window + p.hold - 1;
		float *a = calloc(n, sizeof(float)), *b = calloc(n, sizeof(float)), *b2 = calloc(n, sizeof(float));
		const float *ins[2] = { a, a };
		float *outs[2] = { b, b2 };
		a[100] = 0.05f;
		xlim_init(&l, &p);
		xlim_run(&l, ins, outs, n);
		for (i = 0; i < n; i++)
			CHECK(b[i] == (i == 100 + lat ? 0.05f : 0.0f), "impulse response at %zu: %g", i, b[i]);
		CHECK(lat <= 352, "latency %zu frames, over the 352 budget", lat);
		printf("latency: %zu frames (%.2f ms) at 48 kHz\n", lat, 1e3 * (double)lat / RATE);
		xlim_free(&l);
		free(a); free(b); free(b2);
	}

	/* a sample that is not a number is silence: the output stays finite and
	 * the limiter keeps limiting after it */
	{
		struct xlim l;
		struct cone c;
		size_t n = 48000;
		float *a = calloc(n, sizeof(float)), *b = calloc(n, sizeof(float)), *b2 = calloc(n, sizeof(float));
		const float *ins[2] = { a, a };
		float *outs[2] = { b, b2 };
		int finite = 1;
		for (i = 0; i < n; i++)
			a[i] = (float)sin(2 * M_PI * f0 * (double)i / RATE);
		a[1000] = NAN;
		a[2000] = INFINITY;
		a[3000] = -INFINITY;
		xlim_init(&l, &p);
		xlim_run(&l, ins, outs, n);
		for (i = 0; i < n; i++)
			finite &= isfinite(b[i]) && isfinite(b2[i]);
		CHECK(finite, "a non-finite sample came out");
		cone_init(&c, &p);
		cone_feed(&c, b, n, 0);
		CHECK(c.xpk <= p.x_max * 1.0001, "after non-finite samples the cone reached %.3f mm", 1e3 * c.xpk);
		xlim_free(&l);
		free(a); free(b); free(b2);
	}

	/* ten minutes of limiting leave no drift: a quiet tone is bit-exact
	 * again afterwards (the running mean is recomputed once a window) */
	{
		struct xlim l;
		size_t blk = 4800, k, d = p.window + p.hold - 1;
		float *a = calloc(blk, sizeof(float)), *b = calloc(blk, sizeof(float)), *b2 = calloc(blk, sizeof(float));
		const float *ins[2] = { a, a };
		float *outs[2] = { b, b2 };
		unsigned s = 777;
		double err = 0;
		xlim_init(&l, &p);
		for (k = 0; k < 6000; k++) {
			for (i = 0; i < blk; i++) {
				s = s * 1103515245u + 12345u;
				a[i] = (float)(0.6 * (((s >> 8) & 0xffff) / 32768.0 - 1.0) + 0.6 * sin(2 * M_PI * 90.0 * (double)(k * blk + i) / RATE));
			}
			xlim_run(&l, ins, outs, blk);
		}
		/* five seconds of release, then the quiet tone */
		for (k = 0; k < 60; k++) {
			for (i = 0; i < blk; i++)
				a[i] = (float)(0.1 * sin(2 * M_PI * 1000.0 * (double)(k * blk + i) / RATE));
			xlim_run(&l, ins, outs, blk);
			if (k >= 55)
				for (i = 0; i < blk; i++) {
					double want = 0.1 * sin(2 * M_PI * 1000.0 * (double)(k * blk + i - d) / RATE);
					err = fmax(err, fabs((double)b[i] - (double)(float)want));
				}
		}
		CHECK(err == 0.0 && l.gain == 1.0, "after ten minutes: gain %.17g, error %g", l.gain, err);
		xlim_free(&l);
		free(a); free(b); free(b2);
	}

	/* invalid parameters are refused */
	{
		struct xlim l;
		struct xlim_params q = p;
		q.margin = 0;
		CHECK(xlim_init(&l, &q) != 0, "margin 0 accepted");
		q = p; q.window = 1;
		CHECK(xlim_init(&l, &q) != 0, "window 1 accepted");
		q = p; q.kms = NAN;
		CHECK(xlim_init(&l, &q) != 0, "NaN accepted");
	}

	printf("%s (%d failures)\n", failures ? "FAILED" : "ok", failures);
	return failures != 0;
}
