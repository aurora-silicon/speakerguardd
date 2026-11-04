#!/usr/bin/env python3
"""Pin the 62 shared J613 measurement/protection entries to their public source.

Data: AsahiLinux/speakersafetyd 8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f,
conf/apple/j613.conf -> j413.conf. MIT, The Asahi Linux Contributors.
This is a provenance test, not physical or mode qualification.
"""
import configparser
from pathlib import Path
import unittest

SOURCE_ENTRIES = [('Globals', 'sense_pcm', '2'), ('Globals', 't_ambient', '50.0'), ('Globals', 't_hysteresis', '5.0'), ('Globals', 't_window', '20.0'), ('Globals', 'channels', '8'), ('Globals', 'period', '4096'), ('Controls', 'vsense', 'VSENSE Switch'), ('Controls', 'isense', 'ISENSE Switch'), ('Controls', 'amp_gain', 'Amp Gain Volume'), ('Controls', 'volume', 'Speaker Volume'), ('Speaker/Left Woofer', 'group', '1'), ('Speaker/Left Woofer', 'tr_coil', '32.10'), ('Speaker/Left Woofer', 'tr_magnet', '20.00'), ('Speaker/Left Woofer', 'tau_coil', '4.70'), ('Speaker/Left Woofer', 'tau_magnet', '66.50'), ('Speaker/Left Woofer', 't_limit', '140.0'), ('Speaker/Left Woofer', 't_headroom', '10.0'), ('Speaker/Left Woofer', 'z_nominal', '4.00'), ('Speaker/Left Woofer', 'a_t_20c', '0.00370036'), ('Speaker/Left Woofer', 'is_scale', '3.75'), ('Speaker/Left Woofer', 'vs_scale', '14'), ('Speaker/Left Woofer', 'is_chan', '0'), ('Speaker/Left Woofer', 'vs_chan', '1'), ('Speaker/Right Woofer', 'group', '1'), ('Speaker/Right Woofer', 'tr_coil', '32.10'), ('Speaker/Right Woofer', 'tr_magnet', '20.00'), ('Speaker/Right Woofer', 'tau_coil', '4.70'), ('Speaker/Right Woofer', 'tau_magnet', '66.50'), ('Speaker/Right Woofer', 't_limit', '140.0'), ('Speaker/Right Woofer', 't_headroom', '10.0'), ('Speaker/Right Woofer', 'z_nominal', '4.00'), ('Speaker/Right Woofer', 'a_t_20c', '0.00370036'), ('Speaker/Right Woofer', 'is_scale', '3.75'), ('Speaker/Right Woofer', 'vs_scale', '14'), ('Speaker/Right Woofer', 'is_chan', '2'), ('Speaker/Right Woofer', 'vs_chan', '3'), ('Speaker/Left Tweeter', 'group', '0'), ('Speaker/Left Tweeter', 'tr_coil', '104.90'), ('Speaker/Left Tweeter', 'tr_magnet', '200.0'), ('Speaker/Left Tweeter', 'tau_coil', '1.85'), ('Speaker/Left Tweeter', 'tau_magnet', '70.00'), ('Speaker/Left Tweeter', 't_limit', '140.0'), ('Speaker/Left Tweeter', 't_headroom', '10.0'), ('Speaker/Left Tweeter', 'z_nominal', '3.72'), ('Speaker/Left Tweeter', 'a_t_20c', '0.00371053'), ('Speaker/Left Tweeter', 'is_scale', '3.75'), ('Speaker/Left Tweeter', 'vs_scale', '14'), ('Speaker/Left Tweeter', 'is_chan', '4'), ('Speaker/Left Tweeter', 'vs_chan', '5'), ('Speaker/Right Tweeter', 'group', '0'), ('Speaker/Right Tweeter', 'tr_coil', '104.90'), ('Speaker/Right Tweeter', 'tr_magnet', '200.0'), ('Speaker/Right Tweeter', 'tau_coil', '1.85'), ('Speaker/Right Tweeter', 'tau_magnet', '70.00'), ('Speaker/Right Tweeter', 't_limit', '140.0'), ('Speaker/Right Tweeter', 't_headroom', '10.0'), ('Speaker/Right Tweeter', 'z_nominal', '3.72'), ('Speaker/Right Tweeter', 'a_t_20c', '0.00371053'), ('Speaker/Right Tweeter', 'is_scale', '3.75'), ('Speaker/Right Tweeter', 'vs_scale', '14'), ('Speaker/Right Tweeter', 'is_chan', '6'), ('Speaker/Right Tweeter', 'vs_chan', '7')]

class J613Sources(unittest.TestCase):
    def test_aid20_metadata_and_tas2764_gain(self):
        # Verified 25G83 AID20 plist SHA256:
        # 85d16f9c8305f01374d852c00b40b2bb01bfec048db57a2ad69715ae8686e108
        # OL_thermal.Reb_ref and Thermal_GainReleaseTime_s. Inactive in VI mode.
        profile = configparser.ConfigParser()
        profile.read(Path(__file__).resolve().parents[1] / "conf/apple/j613.conf")
        self.assertEqual(profile["Globals"].getfloat("release_s"), 20.0)
        # Public TAS2764 gain: 11 dBV + 0.5 dB per index, index capped at 15.
        self.assertEqual(profile["Globals"].getint("amp_gain_index"), 15)
        self.assertEqual(profile["Globals"].getfloat("amp_gain_dbv"), 18.5)
        self.assertTrue(profile["Globals"].getboolean("measured_vi"))
        self.assertTrue(profile["Globals"].getboolean("strict_admission"))
        self.assertFalse(profile["Globals"].getboolean("shadow"))
        self.assertEqual(profile["Globals"]["governor"], "asahi")
        for section in profile.sections():
            if section.startswith("Speaker/"):
                self.assertEqual(profile[section].getfloat("r_ff"), 4.0 if "Woofer" in section else 3.72)
                self.assertNotIn("z_min", profile[section])
                self.assertEqual(profile[section].getfloat("p_limit_1s"), 0.0)
                self.assertEqual(profile[section].getfloat("p_limit_60s"), 0.0)

    def test_public_measured_power_values(self):
        profile = configparser.ConfigParser()
        profile.read(Path(__file__).resolve().parents[1] / "conf/apple/j613.conf")
        self.assertEqual(len(SOURCE_ENTRIES), 62)
        for section, key, expected in SOURCE_ENTRIES:
            with self.subTest(section=section, key=key):
                actual = profile[section][key]
                try:
                    self.assertEqual(float(actual), float(expected))
                except ValueError:
                    self.assertEqual(actual, expected)
        for section in profile.sections():
            if not section.startswith("Speaker/"):
                continue
            for key in ("tr_coil", "tr_magnet", "tau_coil", "tau_magnet"):
                self.assertEqual(float(profile[section]["vi_" + key]), float(profile[section][key]))

if __name__ == "__main__":
    unittest.main()
