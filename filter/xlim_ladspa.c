/* SPDX-License-Identifier: MIT */
/*
 * LADSPA wrapper for the excursion limiter, for a PipeWire filter-chain:
 *
 *   { type = ladspa  plugin = "speakerguard-xlim"
 *     label = speakerguard_xlim  name = xlim
 *     control = { "Xmax (mm)" = 0.565 ... } }
 *
 * with links into xlim:in_l and xlim:in_r and out of xlim:out_l and
 * xlim:out_r.  Its "latency" output reports the delay to the host.
 *
 * The plugin ABI below is the LADSPA 1.1 interface, declared here so the
 * build needs no SDK header.
 */
#include "xlim_core.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef float LADSPA_Data;
typedef int LADSPA_Properties;
typedef int LADSPA_PortDescriptor;
typedef int LADSPA_PortRangeHintDescriptor;
typedef struct {
	LADSPA_PortRangeHintDescriptor HintDescriptor;
	LADSPA_Data LowerBound;
	LADSPA_Data UpperBound;
} LADSPA_PortRangeHint;
typedef void *LADSPA_Handle;
typedef struct LADSPA_Descriptor {
	unsigned long UniqueID;
	const char *Label;
	LADSPA_Properties Properties;
	const char *Name;
	const char *Maker;
	const char *Copyright;
	unsigned long PortCount;
	const LADSPA_PortDescriptor *PortDescriptors;
	const char *const *PortNames;
	const LADSPA_PortRangeHint *PortRangeHints;
	void *ImplementationData;
	LADSPA_Handle (*instantiate)(const struct LADSPA_Descriptor *, unsigned long);
	void (*connect_port)(LADSPA_Handle, unsigned long, LADSPA_Data *);
	void (*activate)(LADSPA_Handle);
	void (*run)(LADSPA_Handle, unsigned long);
	void (*run_adding)(LADSPA_Handle, unsigned long);
	void (*set_run_adding_gain)(LADSPA_Handle, LADSPA_Data);
	void (*deactivate)(LADSPA_Handle);
	void (*cleanup)(LADSPA_Handle);
} LADSPA_Descriptor;

#define PORT_INPUT 0x1
#define PORT_OUTPUT 0x2
#define PORT_CONTROL 0x4
#define PORT_AUDIO 0x8
#define HINT_BELOW 0x1
#define HINT_ABOVE 0x2
#define PROP_HARD_RT 0x4

enum {
	P_IN_L, P_IN_R, P_OUT_L, P_OUT_R,
	P_VFS_L, P_VFS_R, P_RE, P_BL, P_MMS, P_KMS, P_RMS,
	P_XMAX, P_VMAX, P_PEAK, P_MARGIN, P_RELEASE, P_WINDOW, P_HOLD,
	P_GAIN, P_LATENCY, P_COUNT
};

static const char *const names[P_COUNT] = {
	"in_l", "in_r", "out_l", "out_r",
	"Vfs L (V)", "Vfs R (V)", "Re (ohm)", "Bl (T m)", "Mms (g)", "Kms (N/m)", "Rms (kg/s)",
	"Xmax (mm)", "Vmax (m/s)", "Peak (FS)", "Margin", "Release (ms)", "Window (ms)", "Hold (ms)",
	"Gain (dB)", "latency",
};

static const LADSPA_PortDescriptor ports[P_COUNT] = {
	PORT_INPUT | PORT_AUDIO, PORT_INPUT | PORT_AUDIO, PORT_OUTPUT | PORT_AUDIO, PORT_OUTPUT | PORT_AUDIO,
	PORT_INPUT | PORT_CONTROL, PORT_INPUT | PORT_CONTROL, PORT_INPUT | PORT_CONTROL, PORT_INPUT | PORT_CONTROL,
	PORT_INPUT | PORT_CONTROL, PORT_INPUT | PORT_CONTROL, PORT_INPUT | PORT_CONTROL, PORT_INPUT | PORT_CONTROL,
	PORT_INPUT | PORT_CONTROL, PORT_INPUT | PORT_CONTROL, PORT_INPUT | PORT_CONTROL, PORT_INPUT | PORT_CONTROL,
	PORT_INPUT | PORT_CONTROL, PORT_INPUT | PORT_CONTROL, PORT_OUTPUT | PORT_CONTROL, PORT_OUTPUT | PORT_CONTROL,
};

static const LADSPA_PortRangeHint hints[P_COUNT] = {
	[P_VFS_L] = { HINT_BELOW | HINT_ABOVE, 0.1f, 50.0f },
	[P_VFS_R] = { HINT_BELOW | HINT_ABOVE, 0.1f, 50.0f },
	[P_RE] = { HINT_BELOW | HINT_ABOVE, 0.5f, 50.0f },
	[P_BL] = { HINT_BELOW | HINT_ABOVE, 0.01f, 20.0f },
	[P_MMS] = { HINT_BELOW | HINT_ABOVE, 0.001f, 100.0f },
	[P_KMS] = { HINT_BELOW | HINT_ABOVE, 1.0f, 1e6f },
	[P_RMS] = { HINT_BELOW | HINT_ABOVE, 0.0f, 100.0f },
	[P_XMAX] = { HINT_BELOW | HINT_ABOVE, 0.001f, 50.0f },
	[P_VMAX] = { HINT_BELOW | HINT_ABOVE, 0.0f, 100.0f },
	[P_PEAK] = { HINT_BELOW | HINT_ABOVE, 0.0f, 1.0f },
	[P_MARGIN] = { HINT_BELOW | HINT_ABOVE, 0.1f, 1.0f },
	[P_RELEASE] = { HINT_BELOW | HINT_ABOVE, 1.0f, 10000.0f },
	[P_WINDOW] = { HINT_BELOW | HINT_ABOVE, 0.1f, 50.0f },
	[P_HOLD] = { HINT_BELOW | HINT_ABOVE, 0.0f, 20.0f },
};

struct inst {
	unsigned long rate;
	LADSPA_Data *port[P_COUNT];
	struct xlim l;
	struct xlim_params cur;
	int ready;
};

/* The longest window and hold the controls allow (their upper bounds): the
 * memory is taken once, at instantiation, never in the audio thread. */
#define WINDOW_MAX_MS 50.0
#define HOLD_MAX_MS 20.0

/* The J700's values are the defaults for unconnected or unset controls. */
static void params_from_ports(struct inst *s, struct xlim_params *p)
{
	xlim_params_j700(p, (double)s->rate);
#define TAKE(pi, field, scale) do { if (s->port[pi]) p->field = (double)*s->port[pi] * (scale); } while (0)
	TAKE(P_VFS_L, v_fs[0], 1.0);
	TAKE(P_VFS_R, v_fs[1], 1.0);
	TAKE(P_RE, re, 1.0);
	TAKE(P_BL, bl, 1.0);
	TAKE(P_MMS, mms, 1e-3);
	TAKE(P_KMS, kms, 1.0);
	TAKE(P_RMS, rms, 1.0);
	TAKE(P_XMAX, x_max, 1e-3);
	TAKE(P_VMAX, v_max, 1.0);
	TAKE(P_PEAK, peak, 1.0);
	TAKE(P_MARGIN, margin, 1.0);
	TAKE(P_RELEASE, release, 1e-3);
#undef TAKE
	if (s->port[P_WINDOW]) {
		double ms = (double)*s->port[P_WINDOW];
		/* out of range (or not a number): an invalid set, refused */
		p->window = ms > 0.0 && ms <= WINDOW_MAX_MS ? (size_t)(ms * 1e-3 * (double)s->rate + 0.5) : 0;
	}
	if (s->port[P_HOLD]) {
		double ms = (double)*s->port[P_HOLD];
		/* out of range: refused through an invalid window */
		if (ms >= 0.0 && ms <= HOLD_MAX_MS)
			p->hold = (size_t)(ms * 1e-3 * (double)s->rate + 0.5);
		else
			p->window = 0;
	}
}

static int same(const struct xlim_params *a, const struct xlim_params *b)
{
	return memcmp(a, b, sizeof(*a)) == 0;
}

static LADSPA_Handle instantiate(const LADSPA_Descriptor *d, unsigned long rate)
{
	struct inst *s;
	double cap = (WINDOW_MAX_MS + HOLD_MAX_MS) * 1e-3 * (double)rate + 2.0;

	(void)d;
	if (rate == 0 || cap > (double)(1 << 17))
		return NULL;
	s = calloc(1, sizeof(*s));
	if (!s)
		return NULL;
	s->rate = rate;
	if (xlim_alloc(&s->l, (size_t)cap) != 0) {
		free(s);
		return NULL;
	}
	return s;
}

static void connect_port(LADSPA_Handle h, unsigned long port, LADSPA_Data *data)
{
	struct inst *s = h;

	if (port < P_COUNT)
		s->port[port] = data;
}

/* Take the controls when they changed, without allocating (the audio
 * thread calls this): an invalid set leaves the previous one, or, before
 * any, the output silent (never unlimited).  A change starts the limiter
 * again from silence. */
static void configure(struct inst *s)
{
	struct xlim_params p;

	params_from_ports(s, &p);
	if (s->ready && same(&p, &s->cur))
		return;
	if (xlim_configure(&s->l, &p) != 0)
		return;
	s->cur = p;
	s->ready = 1;
}

static void activate(LADSPA_Handle h)
{
	struct inst *s = h;

	s->ready = 0;
	configure(s);
}

static void run(LADSPA_Handle h, unsigned long frames)
{
	struct inst *s = h;
	unsigned long i;

	configure(s);
	if (!s->ready || !s->port[P_IN_L] || !s->port[P_IN_R] || !s->port[P_OUT_L] || !s->port[P_OUT_R]) {
		for (i = 0; i < frames; i++) {
			if (s->port[P_OUT_L]) s->port[P_OUT_L][i] = 0.0f;
			if (s->port[P_OUT_R]) s->port[P_OUT_R][i] = 0.0f;
		}
		return;
	}
	{
		const float *in[XLIM_CHANNELS] = { s->port[P_IN_L], s->port[P_IN_R] };
		float *out[XLIM_CHANNELS] = { s->port[P_OUT_L], s->port[P_OUT_R] };
		s->l.min_gain = 1.0;
		xlim_run(&s->l, in, out, frames);
	}
	if (s->port[P_GAIN])
		*s->port[P_GAIN] = (LADSPA_Data)(20.0 * log10(s->l.min_gain > 1e-9 ? s->l.min_gain : 1e-9));
	if (s->port[P_LATENCY])
		*s->port[P_LATENCY] = (LADSPA_Data)(s->cur.window + s->cur.hold - 1);
}

static void cleanup(LADSPA_Handle h)
{
	struct inst *s = h;

	xlim_free(&s->l);
	free(s);
}

static const LADSPA_Descriptor descriptor = {
	.UniqueID = 0x5a6e01,
	.Label = "speakerguard_xlim",
	.Properties = PROP_HARD_RT,
	.Name = "speakerguard excursion/velocity/peak limiter",
	.Maker = "The Aurora Silicon Contributors",
	.Copyright = "MIT",
	.PortCount = P_COUNT,
	.PortDescriptors = ports,
	.PortNames = names,
	.PortRangeHints = hints,
	.instantiate = instantiate,
	.connect_port = connect_port,
	.activate = activate,
	.run = run,
	.cleanup = cleanup,
};

__attribute__((visibility("default"))) const LADSPA_Descriptor *ladspa_descriptor(unsigned long index)
{
	return index == 0 ? &descriptor : NULL;
}
