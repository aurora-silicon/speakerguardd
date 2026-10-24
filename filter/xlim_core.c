/* SPDX-License-Identifier: MIT */
#include "xlim_core.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/*
 * Voltage to displacement of a closed-box driver without coil inductance:
 *
 *   X(s)/V(s) = (Bl/Re) / (Mms s^2 + (Rms + Bl^2/Re) s + Kms)
 *
 * and velocity is s X(s).  Both are mapped to the z-plane with the bilinear
 * transform, prewarped at the resonance so the peak sits where it belongs.
 */
static void design(struct xlim_biquad *x, struct xlim_biquad *u, const struct xlim_params *p)
{
	double w0 = sqrt(p->kms / p->mms);
	double k = w0 / tan(w0 / (2.0 * p->rate));
	double rt = p->rms + p->bl * p->bl / p->re;
	double g = p->bl / p->re;
	double a0 = p->mms * k * k + rt * k + p->kms;
	double a1 = 2.0 * (p->kms - p->mms * k * k);
	double a2 = p->mms * k * k - rt * k + p->kms;

	memset(x, 0, sizeof(*x));
	memset(u, 0, sizeof(*u));
	x->b0 = g / a0;
	x->b1 = 2.0 * g / a0;
	x->b2 = g / a0;
	u->b0 = g * k / a0;
	u->b1 = 0.0;
	u->b2 = -g * k / a0;
	x->a1 = u->a1 = a1 / a0;
	x->a2 = u->a2 = a2 / a0;
}

static inline double biquad(struct xlim_biquad *f, double in)
{
	double out = f->b0 * in + f->z1;

	f->z1 = f->b1 * in - f->a1 * out + f->z2;
	f->z2 = f->b2 * in - f->a2 * out;
	return out;
}

double xlim_model_x(struct xlim_biquad *x, double volts)
{
	return biquad(x, volts);
}

void xlim_params_j700(struct xlim_params *p, double rate)
{
	memset(p, 0, sizeof(*p));
	p->rate = rate;
	p->v_fs[0] = 7.30;
	p->v_fs[1] = 7.33;
	p->re = 4.2 + 0.1;
	p->bl = 0.74;
	p->mms = 0.16e-3;
	p->kms = 1720.0;
	p->rms = 0.14;
	p->x_max = 0.565e-3;
	p->v_max = 0.0;
	p->peak = 1.0;
	p->margin = 0.9;
	p->release = 0.200;
	/* 4 ms: the 3.333 ms attack plus the 0.667 ms look-ahead */
	p->window = (size_t)(0.004 * rate + 0.5);
}

int xlim_init(struct xlim *l, const struct xlim_params *p)
{
	size_t c;

	memset(l, 0, sizeof(*l));
	if (!(p->rate > 0) || !(p->re > 0) || !(p->bl > 0) || !(p->mms > 0) || !(p->kms > 0) ||
	    !(p->rms >= 0) || !(p->x_max > 0) || !(p->v_max >= 0) || !(p->peak >= 0) ||
	    !(p->margin > 0 && p->margin <= 1) || !(p->release > 0) || p->window < 2 || p->window > 1 << 16)
		return -1;
	for (c = 0; c < XLIM_CHANNELS; c++)
		if (!(p->v_fs[c] > 0))
			return -1;
	l->p = *p;
	for (c = 0; c < XLIM_CHANNELS; c++) {
		design(&l->x[c], &l->u[c], p);
		l->delay[c] = calloc(p->window, sizeof(float));
		if (!l->delay[c])
			goto fail;
	}
	l->req = calloc(p->window, sizeof(double));
	l->dq = calloc(p->window, sizeof(unsigned long long));
	l->held = calloc(p->window, sizeof(double));
	if (!l->req || !l->dq || !l->held)
		goto fail;
	for (c = 0; c < p->window; c++)
		l->held[c] = 1.0;
	l->held_sum = (double)p->window;
	l->gain = 1.0;
	l->min_gain = 1.0;
	l->rel = 1.0 - exp(-1.0 / (p->release * p->rate));
	return 0;
fail:
	xlim_free(l);
	return -1;
}

void xlim_free(struct xlim *l)
{
	size_t c;

	for (c = 0; c < XLIM_CHANNELS; c++) {
		free(l->delay[c]);
		l->delay[c] = NULL;
	}
	free(l->req);
	free(l->dq);
	free(l->held);
	l->req = l->held = NULL;
	l->dq = NULL;
}

/*
 * The gain that sample n needs: every limit at margin * limit, given the
 * model's prediction for it at unity gain.  A linear model scales with the
 * gain, so this is the largest gain under which no limit is passed.
 */
static double required(struct xlim *l, const double s[XLIM_CHANNELS])
{
	const struct xlim_params *p = &l->p;
	double g = 1.0;
	size_t c;

	for (c = 0; c < XLIM_CHANNELS; c++) {
		double v = s[c] * p->v_fs[c];
		double x = fabs(biquad(&l->x[c], v));
		double u = fabs(biquad(&l->u[c], v));
		double a = fabs(s[c]);

		if (x * g > p->margin * p->x_max)
			g = p->margin * p->x_max / x;
		if (p->v_max > 0 && u * g > p->margin * p->v_max)
			g = p->margin * p->v_max / u;
		if (p->peak > 0 && a * g > p->peak)
			g = p->peak / a;
	}
	return g;
}

void xlim_run(struct xlim *l, const float *const in[XLIM_CHANNELS], float *const out[XLIM_CHANNELS], size_t frames)
{
	const size_t w = l->p.window;
	size_t i, c;

	for (i = 0; i < frames; i++) {
		unsigned long long n = l->n;
		size_t slot = (size_t)(n % w);
		double s[XLIM_CHANNELS], r, h, g;

		for (c = 0; c < XLIM_CHANNELS; c++)
			s[c] = in[c][i];
		r = required(l, s);
		/*
		 * Sliding minimum over the frames n - w + 1 .. n: a deque of frame
		 * indices whose required gains increase from front to back.  The
		 * expired front goes first, before its ring slot is reused.
		 */
		while (l->dq_len && l->dq[l->dq_head] + w <= n) {
			l->dq_head = (l->dq_head + 1) % w;
			l->dq_len--;
		}
		while (l->dq_len && l->req[l->dq[(l->dq_head + l->dq_len - 1) % w] % w] >= r)
			l->dq_len--;
		l->req[slot] = r;
		l->dq[(l->dq_head + l->dq_len) % w] = n;
		l->dq_len++;
		h = l->req[l->dq[l->dq_head] % w];
		/*
		 * Mean of the held minimum over the window: a ramp of w frames
		 * that is at or under every required gain of the frame it is
		 * applied to, the one w - 1 frames back (every window in the mean
		 * contains it).
		 */
		l->held_sum += h - l->held[slot];
		l->held[slot] = h;
		g = l->held_sum / (double)w;
		/* release slowly; never above what the window allows */
		if (g > l->gain)
			g = l->gain + (g - l->gain) * l->rel;
		l->gain = g;
		if (g < l->min_gain)
			l->min_gain = g;
		for (c = 0; c < XLIM_CHANNELS; c++) {
			/* the frame w - 1 back sits in the next slot */
			float d = l->delay[c][(slot + 1) % w];
			l->delay[c][slot] = (float)s[c];
			out[c][i] = (float)(d * g);
		}
		l->n = n + 1;
	}
}
