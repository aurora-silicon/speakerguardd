/* SPDX-License-Identifier: MIT */
/*
 * speakerguard-xlim: a look-ahead excursion, velocity and sample-peak
 * limiter for loudspeakers, from the driver's lumped electro-mechanical
 * model.  It predicts the cone displacement (and velocity) the signal will
 * cause, and lowers the gain just enough, ahead of time, that neither passes
 * its limit.  The thermal and power limits stay with speakerguardd.
 */
#ifndef XLIM_CORE_H
#define XLIM_CORE_H

#include <stddef.h>

#define XLIM_CHANNELS 2

/* The driver and the limits.  SI units unless noted. */
struct xlim_params {
	double rate;          /* sample rate (Hz) */
	double v_fs[XLIM_CHANNELS]; /* amplifier volts at digital full scale */
	double re;            /* coil DC resistance plus series resistance (ohm) */
	double bl;            /* force factor (T m) */
	double mms;           /* moving mass (kg) */
	double kms;           /* suspension stiffness (N/m) */
	double rms;           /* mechanical damping (kg/s) */
	double x_max;         /* displacement limit (m) */
	double v_max;         /* velocity limit (m/s), 0 = off */
	double peak;          /* sample-peak ceiling (full scale), 0 = off */
	double margin;        /* the limits are held at margin * limit (0 < margin <= 1) */
	double release;       /* release time constant (s) */
	size_t window;        /* the attack ramp, frames */
	size_t hold;          /* frames the ramp ends before the frame that needs it: the
	                       * cone's memory (the latency is window + hold - 1) */
};

struct xlim_biquad {
	double b0, b1, b2, a1, a2, z1, z2;
};

struct xlim {
	struct xlim_params p;
	struct xlim_biquad x[XLIM_CHANNELS], u[XLIM_CHANNELS];
	/* the frames the buffers hold: the largest window + hold xlim_configure takes */
	size_t cap;
	/* configured: until then (or after an invalid first set) the output is silence */
	int ok;
	/* the input delayed by window + hold - 1 frames */
	float *delay[XLIM_CHANNELS];
	/* sliding minimum of the required gain (a monotonic deque of positions) */
	double *req;
	unsigned long long *dq;
	size_t dq_head, dq_len;
	/* running mean of the held minimum over the window */
	double *held;
	double held_sum;
	unsigned long long n; /* frames seen */
	double gain, rel;
	double min_gain;      /* the lowest gain applied since the last read */
};

/*
 * Allocation and configuration are apart, so that a real-time host never
 * allocates while it runs: xlim_alloc() takes the memory for a window and
 * hold of up to `cap` frames together (0, or -1 for no memory);
 * xlim_configure() takes a parameter set and starts from silence, without
 * allocating (0, or -1 for an invalid set, which leaves the limiter as it
 * was).  xlim_init() is both at once.
 */
int xlim_alloc(struct xlim *l, size_t cap);
int xlim_configure(struct xlim *l, const struct xlim_params *p);
int xlim_init(struct xlim *l, const struct xlim_params *p);
void xlim_free(struct xlim *l);
/* Process `frames` frames from in[c] into out[c] (may alias).  A sample that
 * is not finite is taken as silence.  Unconfigured, the output is silence. */
void xlim_run(struct xlim *l, const float *const in[XLIM_CHANNELS], float *const out[XLIM_CHANNELS], size_t frames);
/* The displacement (m) and velocity (m/s) the model predicts for one channel's
 * next input sample (for tests; advances that channel's model state). */
double xlim_model_x(struct xlim_biquad *x, double volts);
/* The MacBook Neo (J700) driver's values and limits. */
void xlim_params_j700(struct xlim_params *p, double rate);

#endif
