# SPDX-License-Identifier: MIT
PREFIX ?= /usr
BINDIR ?= $(PREFIX)/bin
SHAREDIR ?= $(PREFIX)/share/speakerguardd
UNITDIR ?= $(PREFIX)/lib/systemd/system
UDEVDIR ?= $(PREFIX)/lib/udev/rules.d
TARGET ?= aarch64-unknown-linux-musl

all:
	cargo build --release --target $(TARGET)

test:
	cargo test

install: all
	install -dDm0755 $(DESTDIR)$(BINDIR) $(DESTDIR)$(SHAREDIR)/apple $(DESTDIR)$(UNITDIR) $(DESTDIR)$(UDEVDIR)
	install -pm0755 target/$(TARGET)/release/speakerguardd $(DESTDIR)$(BINDIR)/
	install -pm0644 conf/apple/*.conf $(DESTDIR)$(SHAREDIR)/apple/
	install -pm0644 speakerguardd.service $(DESTDIR)$(UNITDIR)/
	install -pm0644 95-speakerguardd.rules $(DESTDIR)$(UDEVDIR)/

.PHONY: all test install
