// Copyright (c) 2026 The Aurora Silicon Contributors
// SPDX-License-Identifier: MIT

//! Float32 elementary functions with the results the voicing units get from
//! their platform (macOS 26 on Arm).
//!
//! A voicing box stores Float32 filter words and gains that it computes with
//! the platform's own `expf`, `exp10f`, `sinf` and `cosf`. Those are not
//! correctly rounded: about one result in a hundred is a unit in the last
//! place away from the correctly rounded value, and any other libm differs
//! from them on a different set of arguments. A box that must match the unit
//! word for word therefore computes them the same way:
//!
//! - `expf` and `exp10f`: the argument times `128/ln 2` (or `128 log2 10`)
//!   in binary64 split into an integer `k` and a remainder `r`; the scale
//!   `2^(k/128)` from a 128-entry table of `2^(j/128)` with the integer's
//!   bits added to the exponent field; `s + s*r*(c1 + c2 r)` in fused binary64
//!   operations, rounded once to Float32.
//! - `sinf` and `cosf`: binary64 minimax polynomials of degree five (sine)
//!   and eight (cosine) on `|x| < pi/4`, a fused reduction by `pi/2` above,
//!   rounded once to Float32.
//!
//! Every function is checked against platform results for our own inputs in
//! the tests. Outside the range the boxes use (`|x| > 10^4` for the sine and
//! cosine) the correctly rounded value is returned instead.

/// `2^(j/128)` (correctly rounded binary64) minus `j << 45`, so that adding
/// an integer `k`'s bits shifted by 45 gives `2^(k/128)`'s bits.
const EXP2_TABLE: [u64; 128] = [
    0x3ff0000000000000,
    0x3feff63da9fb3335,
    0x3fefec9a3e778061,
    0x3fefe315e86e7f85,
    0x3fefd9b0d3158574,
    0x3fefd06b29ddf6de,
    0x3fefc74518759bc8,
    0x3fefbe3ecac6f383,
    0x3fefb5586cf9890f,
    0x3fefac922b7247f7,
    0x3fefa3ec32d3d1a2,
    0x3fef9b66affed31b,
    0x3fef9301d0125b51,
    0x3fef8abdc06c31cc,
    0x3fef829aaea92de0,
    0x3fef7a98c8a58e51,
    0x3fef72b83c7d517b,
    0x3fef6af9388c8dea,
    0x3fef635beb6fcb75,
    0x3fef5be084045cd4,
    0x3fef54873168b9aa,
    0x3fef4d5022fcd91d,
    0x3fef463b88628cd6,
    0x3fef3f49917ddc96,
    0x3fef387a6e756238,
    0x3fef31ce4fb2a63f,
    0x3fef2b4565e27cdd,
    0x3fef24dfe1f56381,
    0x3fef1e9df51fdee1,
    0x3fef187fd0dad990,
    0x3fef1285a6e4030b,
    0x3fef0cafa93e2f56,
    0x3fef06fe0a31b715,
    0x3fef0170fc4cd831,
    0x3feefc08b26416ff,
    0x3feef6c55f929ff1,
    0x3feef1a7373aa9cb,
    0x3feeecae6d05d866,
    0x3feee7db34e59ff7,
    0x3feee32dc313a8e5,
    0x3feedea64c123422,
    0x3feeda4504ac801c,
    0x3feed60a21f72e2a,
    0x3feed1f5d950a897,
    0x3feece086061892d,
    0x3feeca41ed1d0057,
    0x3feec6a2b5c13cd0,
    0x3feec32af0d7d3de,
    0x3feebfdad5362a27,
    0x3feebcb299fddd0d,
    0x3feeb9b2769d2ca7,
    0x3feeb6daa2cf6642,
    0x3feeb42b569d4f82,
    0x3feeb1a4ca5d920f,
    0x3feeaf4736b527da,
    0x3feead12d497c7fd,
    0x3feeab07dd485429,
    0x3feea9268a5946b7,
    0x3feea76f15ad2148,
    0x3feea5e1b976dc09,
    0x3feea47eb03a5585,
    0x3feea34634ccc320,
    0x3feea23882552225,
    0x3feea155d44ca973,
    0x3feea09e667f3bcd,
    0x3feea012750bdabf,
    0x3fee9fb23c651a2f,
    0x3fee9f7df9519484,
    0x3fee9f75e8ec5f74,
    0x3fee9f9a48a58174,
    0x3fee9feb564267c9,
    0x3feea0694fde5d3f,
    0x3feea11473eb0187,
    0x3feea1ed0130c132,
    0x3feea2f336cf4e62,
    0x3feea427543e1a12,
    0x3feea589994cce13,
    0x3feea71a4623c7ad,
    0x3feea8d99b4492ed,
    0x3feeaac7d98a6699,
    0x3feeace5422aa0db,
    0x3feeaf3216b5448c,
    0x3feeb1ae99157736,
    0x3feeb45b0b91ffc6,
    0x3feeb737b0cdc5e5,
    0x3feeba44cbc8520f,
    0x3feebd829fde4e50,
    0x3feec0f170ca07ba,
    0x3feec49182a3f090,
    0x3feec86319e32323,
    0x3feecc667b5de565,
    0x3feed09bec4a2d33,
    0x3feed503b23e255d,
    0x3feed99e1330b358,
    0x3feede6b5579fdbf,
    0x3feee36bbfd3f37a,
    0x3feee89f995ad3ad,
    0x3feeee07298db666,
    0x3feef3a2b84f15fb,
    0x3feef9728de5593a,
    0x3feeff76f2fb5e47,
    0x3fef05b030a1064a,
    0x3fef0c1e904bc1d2,
    0x3fef12c25bd71e09,
    0x3fef199bdd85529c,
    0x3fef20ab5fffd07a,
    0x3fef27f12e57d14b,
    0x3fef2f6d9406e7b5,
    0x3fef3720dcef9069,
    0x3fef3f0b555dc3fa,
    0x3fef472d4a07897c,
    0x3fef4f87080d89f2,
    0x3fef5818dcfba487,
    0x3fef60e316c98398,
    0x3fef69e603db3285,
    0x3fef7321f301b460,
    0x3fef7c97337b9b5f,
    0x3fef864614f5a129,
    0x3fef902ee78b3ff6,
    0x3fef9a51fbc74c83,
    0x3fefa4afa2a490da,
    0x3fefaf482d8e67f1,
    0x3fefba1bee615a27,
    0x3fefc52b376bba97,
    0x3fefd0765b6e4540,
    0x3fefdbfdad9cbe14,
    0x3fefe7c1819e90d8,
    0x3feff3c22b8f71f1,
];

const EXP_C2: f64 = f64::from_bits(0x3eee_bfbd_ff30_d656); // 0x1.ebfbdff30d656p-17
const EXP_C1: f64 = f64::from_bits(0x3f76_2e44_53e1_0dae); // 0x1.62e4453e10daep-8

/// `2^(k/128) * (1 + r (c1 + c2 r))` for the integer `k` (two's-complement
/// bits) and remainder `r`, rounded once to Float32.
fn exp2_128(k: u64, r: f64) -> f32 {
    let scale = f64::from_bits(EXP2_TABLE[(k & 127) as usize].wrapping_add(k.wrapping_shl(45)));
    let p = libm::fma(EXP_C2, r, EXP_C1);
    libm::fma(p * r, scale, scale) as f32
}

/// The result of an overflowing or underflowing exponential.
fn exp_out_of_range(x: f32) -> f32 {
    if x.is_nan() {
        x * f32::MAX
    } else if x < 0.0 {
        f32::MIN_POSITIVE * f32::MIN_POSITIVE
    } else {
        x * f32::MAX
    }
}

/// The platform's Float32 `e^x`.
pub fn expf(x: f32) -> f32 {
    if (x.to_bits() >> 23) & 0xff >= 0x86 {
        return exp_out_of_range(x);
    }
    const SCALE: f64 = f64::from_bits(0x4067_1547_652b_82fe); // 128 / ln 2
    const SHIFT: f64 = f64::from_bits(0x4338_0000_0000_0000); // 1.5 * 2^52
    let xd = f64::from(x);
    let shifted = libm::fma(SCALE, xd, SHIFT);
    let k = shifted - SHIFT;
    exp2_128(shifted.to_bits(), libm::fma(SCALE, xd, -k))
}

/// The platform's Float32 `10^x`: the reduction rounds half away from zero.
pub fn exp10f(x: f32) -> f32 {
    if (x.to_bits() >> 23) & 0xff >= 0x85 {
        return exp_out_of_range(x);
    }
    const SCALE: f64 = f64::from_bits(0x407a_934f_0979_a371); // 128 * log2(10)
    let z = f64::from(x) * SCALE;
    let n = libm::round(z);
    exp2_128(n as i64 as u64, z - n)
}

const SIN: [f64; 3] = [
    f64::from_bits(0xbf29_943e_0fc4_818f), // -0x1.9943e0fc4818fp-13
    f64::from_bits(0x3f81_1073_afd1_45eb), // 0x1.11073afd145ebp-7
    f64::from_bits(0xbfc5_5554_5268_a02a), // -0x1.555545268a02ap-3
];
const COS: [f64; 4] = [
    f64::from_bits(0x3ef9_9169_fc93_8f4a), // 0x1.99169fc938f4ap-16
    f64::from_bits(0xbf56_c07f_1694_5df4), // -0x1.6c07f16945df4p-10
    f64::from_bits(0x3fa5_5553_c789_9699), // 0x1.55553c7899699p-5
    f64::from_bits(0xbfdf_ffff_fcb8_2e97), // -0x1.ffffffcb82e97p-2
];
const INV_HALF_PI: f64 = f64::from_bits(0x3fe4_5f30_6dc9_c883); // 2/pi
const HALF_PI_HI: f64 = f64::from_bits(0x3ff9_21fb_5444_2d18);
const HALF_PI_LO: f64 = f64::from_bits(0x3c91_a626_3314_5c07);
const LARGE_INV_PI_HI: f64 = f64::from_bits(0x3fd4_5f30_6e00_0000);
const LARGE_INV_PI_LO: f64 = f64::from_bits(0xbdeb_1bbe_ad60_3d8b);
const LARGE: [f64; 5] = [
    f64::from_bits(0xc010_a8b9_fc97_b22c), // -0x1.0a8b9fc97b22cp+2
    f64::from_bits(0xc00d_31f4_a0a5_5230), // -0x1.d31f4a0a55230p+1
    f64::from_bits(0x402e_f88f_0b12_50c1), // 0x1.ef88f0b1250c1p+3
    f64::from_bits(0x4005_37b6_744f_a779), // 0x1.537b6744fa779p+1
    f64::from_bits(0x3fb3_9502_de83_d49c), // 0x1.39502de83d49cp-4
];
const FRAC_PI_4_UP: u32 = 0x3f49_0fdb;
const TINY: u32 = 0x3980_0000; // 2^-12
const REDUCE_TWICE: u32 = 0x42f0_0000; // 120
const TRIG_RANGE: f32 = 1.0e4;

fn sin_poly(x: f64) -> f64 {
    let z = x * x;
    let x3 = x * z;
    let a = libm::fma(z, SIN[0], SIN[1]);
    let x5 = z * x3;
    let b = libm::fma(x3, SIN[2], x);
    libm::fma(x5, a, b)
}

fn cos_poly(z: f64) -> f64 {
    let a = libm::fma(z, COS[0], COS[1]);
    let b = libm::fma(z, COS[2], COS[3]);
    let p = libm::fma(z * z, a, b);
    libm::fma(z, p, 1.0)
}

fn with_sign_bit(x: f64, bit: i64) -> f64 {
    f64::from_bits(x.to_bits() ^ (((bit & 1) as u64) << 63))
}

/// The platform's Float32 sine.
pub fn sinf(x: f32) -> f32 {
    let magnitude = x.to_bits() & 0x7fff_ffff;
    if magnitude >= 0x7f80_0000 {
        // an infinity is invalid, a NaN stays a NaN
        return x * 0.0;
    }
    if x.abs() > TRIG_RANGE {
        return libm::sin(f64::from(x)) as f32;
    }
    if magnitude < FRAC_PI_4_UP {
        if magnitude < TINY {
            return libm::fmaf(x, f32::from_bits(0x4c80_0000), x) * f32::from_bits(0x3280_0000);
        }
        return sin_poly(f64::from(x)) as f32;
    }
    let xd = f64::from(x);
    if magnitude >= REDUCE_TWICE {
        let high = xd * LARGE_INV_PI_HI;
        let n = libm::rint(high);
        let r = (high - n) + xd * LARGE_INV_PI_LO;
        let signed = with_sign_bit(r, n as i64);
        let z = r * r;
        let a = libm::fma(z, z + LARGE[0], LARGE[2]);
        let b = libm::fma(z, z + LARGE[1], LARGE[3]);
        return ((signed * a) * (LARGE[4] * b)) as f32;
    }
    let n = libm::rint(xd * INV_HALF_PI);
    let r = libm::fma(-n, HALF_PI_HI, xd);
    let n = n as i64;
    let v = if n & 1 != 0 { cos_poly(r * r) } else { sin_poly(r) };
    with_sign_bit(v, n >> 1) as f32
}

/// The platform's Float32 cosine.
pub fn cosf(x: f32) -> f32 {
    let magnitude = x.to_bits() & 0x7fff_ffff;
    if magnitude >= 0x7f80_0000 {
        // an infinity is invalid, a NaN stays a NaN
        return x * 0.0;
    }
    if x.abs() > TRIG_RANGE {
        return libm::cos(f64::from(x)) as f32;
    }
    let x = x.abs();
    if magnitude < FRAC_PI_4_UP {
        if magnitude < TINY {
            return (f32::from_bits(0x4c80_0000) - x) * f32::from_bits(0x3280_0000);
        }
        let xd = f64::from(x);
        return cos_poly(xd * xd) as f32;
    }
    let xd = f64::from(x);
    let n = libm::rint(xd * INV_HALF_PI);
    let mut r = libm::fma(-n, HALF_PI_HI, xd);
    if magnitude >= REDUCE_TWICE {
        r = libm::fma(-n, HALF_PI_LO, r);
    }
    let q = n as i64 + 1;
    let v = if q & 1 != 0 { cos_poly(r * r) } else { sin_poly(r) };
    with_sign_bit(v, q >> 1) as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_scale_table_is_two_to_the_j_over_128() {
        for (j, &w) in EXP2_TABLE.iter().enumerate() {
            let v = f64::from_bits(w + ((j as u64) << 45));
            let want = libm::exp2(j as f64 / 128.0);
            assert!((v - want).abs() <= want * 1.2e-16, "{j}");
        }
    }

    #[test]
    fn constants_are_the_stated_values() {
        // a quadratic close to the Taylor terms of e^(r ln2/128)
        let h = core::f64::consts::LN_2 / 128.0;
        assert!((EXP_C1 / h - 1.0).abs() < 1e-6);
        assert!((EXP_C2 / (h * h / 2.0) - 1.0).abs() < 1e-3);
        assert!((INV_HALF_PI - 2.0 / core::f64::consts::PI).abs() < 1e-16);
        assert_eq!(HALF_PI_HI + HALF_PI_LO, core::f64::consts::FRAC_PI_2);
    }

    /// Platform results for our own arguments (macOS 26.6.2, Arm), each one
    /// where the correctly rounded `e^x` and `cos y` differ from it:
    /// `(x, expf(x), y, cosf(y), sinf(y))` as Float32 bits.
    #[test]
    fn matches_the_platform_where_correct_rounding_does_not() {
        let cases: [(u32, u32, u32, u32, u32); 6] = [
            (0xbf922462, 0x3ea376a4, 0x3fe53b41, 0xbe5f8aa3, 0x3f79d35a),
            (0xc0cb818f, 0x3ae2c526, 0xbf5be338, 0x3f273af9, 0xbf41d471),
            (0xc0f92466, 0x39d9ea24, 0xc01569d5, 0xbf3110c8, 0xbf38e37b),
            (0x410e317c, 0x45e22ee6, 0x3f5a1986, 0x3f289478, 0x3f40a83f),
            (0x41957704, 0x4cf7f8e3, 0x3f95f5ac, 0x3ec705a8, 0x3f6bde1f),
            (0xc1866acc, 0x3358b5eb, 0x3f8559b3, 0x3f0131de, 0x3f5d0225),
        ];
        for (x, e, y, c, s) in cases {
            let (x, y) = (f32::from_bits(x), f32::from_bits(y));
            assert_eq!(expf(x).to_bits(), e, "exp {x}");
            assert_ne!((libm::exp(f64::from(x)) as f32).to_bits(), e);
            assert_eq!(cosf(y).to_bits(), c, "cos {y}");
            assert_eq!(sinf(y).to_bits(), s, "sin {y}");
        }
    }

    #[test]
    fn close_to_the_true_functions() {
        let mut x = -19.75f32;
        while x < 19.75 {
            let ulp = |v: f32| f64::from(f32::from_bits(v.to_bits() + 1) - v).abs();
            let e = libm::exp(f64::from(x));
            assert!((f64::from(expf(x)) - e).abs() <= ulp(e as f32), "exp {x}");
            let c = libm::cos(f64::from(x));
            assert!((f64::from(cosf(x)) - c).abs() <= ulp(c as f32).max(1e-45), "cos {x}");
            let s = libm::sin(f64::from(x));
            assert!((f64::from(sinf(x)) - s).abs() <= ulp(s as f32).max(1e-45), "sin {x}");
            x += 0.0137;
        }
        assert_eq!(expf(0.0), 1.0);
        assert_eq!(cosf(0.0), 1.0);
        assert_eq!(sinf(-0.0).to_bits(), (-0.0f32).to_bits());
        assert!(expf(-200.0) == 0.0 && expf(200.0).is_infinite());
        assert_eq!(exp10f(2.0), 100.0);
    }
}
