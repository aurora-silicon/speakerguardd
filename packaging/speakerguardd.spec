%global shortcommit %(c=%{commit}; echo ${c:0:12})
%global debug_package %{nil}

Name:           speakerguardd
Version:        0.1.10
Release:        15.pr8.%{shortcommit}%{?dist}
Summary:        Speaker protection daemon
License:        MIT
URL:            https://github.com/aurora-silicon/speakerguardd
Source0:        speakerguardd-%{commit}.tar.gz
Source1:        speakerguardd-vendor-%{commit}.tar.gz
Source2:        use-feedback-guard
Source3:        50-j700-guard.conf
ExclusiveArch:  aarch64
BuildRequires:  cargo
BuildRequires:  rust
BuildRequires:  systemd-rpm-macros
BuildRequires:  python3

%description
Speaker protection for Apple Silicon laptops. The unit is not enabled and
does not displace speakersafetyd without the device's existing owner policy.

%prep
%autosetup -n speakerguardd-%{commit} -a 1
cat >> .cargo/config.toml <<'CFG'

[source.crates-io]
replace-with = "vendored-sources"
[source.vendored-sources]
directory = "vendor"
CFG

%build
export SPEAKERGUARDD_COMMIT=%{commit}
cargo build --release --locked --offline

%check
cargo test --release --locked --offline checkpoint
cargo test --release --locked --offline durable_boot_tests
cargo test --release --locked --offline
python3 tools/test-preserve-recordings.py

%install
install -Dpm0755 target/release/speakerguardd %{buildroot}%{_bindir}/speakerguardd
for c in j316 j413 j616 j700; do
  install -Dpm0644 conf/apple/$c.conf %{buildroot}%{_datadir}/speakerguardd/apple/$c.conf
done
install -Dpm0644 speakerguardd.service %{buildroot}%{_unitdir}/speakerguardd.service
install -Dpm0644 95-speakerguardd.rules %{buildroot}%{_udevrulesdir}/95-speakerguardd.rules
install -Dpm0755 tools/suspend-checkpoint %{buildroot}%{_prefix}/lib/systemd/system-sleep/speakerguardd
install -Dpm0755 %{SOURCE2} %{buildroot}%{_libexecdir}/speakerguardd/use-feedback-guard
install -Dpm0755 tools/preserve-recordings %{buildroot}%{_libexecdir}/speakerguardd/preserve-recordings
install -Dpm0644 %{SOURCE3} %{buildroot}%{_unitdir}/speakersafetyd.service.d/50-j700-guard.conf
install -Dpm0644 LICENSE %{buildroot}%{_datadir}/licenses/speakerguardd/LICENSE
install -dm0755 %{buildroot}%{_datadir}/aurora-public
printf 'aurora-silicon/speakerguardd fix/instant-boot-thermal-state %s\n' %{commit} > %{buildroot}%{_datadir}/aurora-public/speakerguardd.source

%post
%systemd_post speakerguardd.service
%preun
%systemd_preun speakerguardd.service
%postun
%systemd_postun_with_restart speakerguardd.service

%files
%license LICENSE
%{_bindir}/speakerguardd
%dir %{_datadir}/speakerguardd
%dir %{_datadir}/speakerguardd/apple
%{_datadir}/speakerguardd/apple/*.conf
%{_unitdir}/speakerguardd.service
%dir %{_unitdir}/speakersafetyd.service.d
%{_unitdir}/speakersafetyd.service.d/50-j700-guard.conf
%{_udevrulesdir}/95-speakerguardd.rules
%{_prefix}/lib/systemd/system-sleep/speakerguardd
%dir %{_libexecdir}/speakerguardd
%{_libexecdir}/speakerguardd/use-feedback-guard
%{_libexecdir}/speakerguardd/preserve-recordings
%{_datadir}/aurora-public/speakerguardd.source

%changelog
* Tue Oct 27 2026 Aurora Silicon - 0.1.10-15.pr8
- Govern a measured cross-OS hot start without the cold-start latch.
- Package the PR 23 recording preservation helper for local override migration.

* Tue Oct 27 2026 Aurora Silicon - 0.1.10-14.pr8
- Base checkpoint work on deployed pr8 and fail closed on untrusted cross-boot time.
