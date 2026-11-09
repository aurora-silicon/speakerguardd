# aurora-speaker-upgrade: make install DESTDIR=... PREFIX=/usr
PREFIX ?= /usr
LIBDIR ?= $(PREFIX)/lib/aurora-speaker-upgrade
UNITDIR ?= $(PREFIX)/lib/systemd/system
V2UNIT ?= speakerguardd-v2.service
PY := $(wildcard aurora_speaker_upgrade/*.py aurora_speaker_upgrade/asahi_firmware/*.py)

install:
	install -Dm0755 aurora-speaker-upgrade $(DESTDIR)$(PREFIX)/bin/aurora-speaker-upgrade
	sed -i 's#/usr/lib/aurora-speaker-upgrade#$(LIBDIR)#' $(DESTDIR)$(PREFIX)/bin/aurora-speaker-upgrade
	for f in $(PY); do install -Dm0644 $$f $(DESTDIR)$(LIBDIR)/$$f; done
	sed -i 's#^CATALOG = .*#CATALOG = "$(PREFIX)/share/aurora-speaker-upgrade/restore-images.json"#' $(DESTDIR)$(LIBDIR)/aurora_speaker_upgrade/ipsw.py
	install -Dm0644 share/restore-images.json $(DESTDIR)$(PREFIX)/share/aurora-speaker-upgrade/restore-images.json
	install -Dm0644 systemd/aurora-speaker-upgrade.service $(DESTDIR)$(UNITDIR)/aurora-speaker-upgrade.service
	install -Dm0644 systemd/50-aurora-speaker-upgrade.conf $(DESTDIR)$(UNITDIR)/$(V2UNIT).d/50-aurora-speaker-upgrade.conf

check:
	python3 -I -m unittest discover -s tests -v

.PHONY: install check
