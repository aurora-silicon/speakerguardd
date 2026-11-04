# Builds the macOS reference host. Requires the Xcode command-line tools.
CFLAGS ?= -O2
all: rig_host prefshim.dylib au_discover
rig_host: rig_host.m
	clang -fobjc-arc $(CFLAGS) -framework AudioToolbox -framework Foundation $< -o $@
au_discover: au_discover.m
	clang -fobjc-arc $(CFLAGS) -framework AudioToolbox -framework Foundation $< -o $@
prefshim.dylib: prefshim.c
	clang -dynamiclib $(CFLAGS) -framework CoreFoundation $< -o $@
clean:
	rm -f rig_host au_discover prefshim.dylib
.PHONY: all clean
