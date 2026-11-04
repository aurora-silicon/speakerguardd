# SPDX-License-Identifier: MIT
CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra -std=c11 -D_DEFAULT_SOURCE
LIBDIR ?= /usr/lib64

all: speakerguard-xlim.so
speakerguard-xlim.so: xlim_ladspa.c xlim_core.c xlim_core.h
	$(CC) $(CFLAGS) -fPIC -shared -fvisibility=hidden -o $@ xlim_ladspa.c xlim_core.c -lm
test: test_xlim test_random test_ladspa speakerguard-xlim.so
	./test_xlim && ./test_random && ./test_ladspa ./speakerguard-xlim.so
test_xlim: test_xlim.c xlim_core.c xlim_core.h
	$(CC) $(CFLAGS) -o $@ test_xlim.c xlim_core.c -lm
test_random: test_random.c xlim_core.c xlim_core.h
	$(CC) $(CFLAGS) -o $@ test_random.c xlim_core.c -lm
test_ladspa: test_ladspa.c
	$(CC) $(CFLAGS) -o $@ test_ladspa.c -ldl -lm
install: speakerguard-xlim.so
	install -Dpm0755 speakerguard-xlim.so $(DESTDIR)$(LIBDIR)/ladspa/speakerguard-xlim.so
clean:
	rm -f speakerguard-xlim.so test_xlim test_random test_ladspa
.PHONY: all test install clean
