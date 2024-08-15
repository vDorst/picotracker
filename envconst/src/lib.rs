#![no_std]
#![deny(clippy::pedantic)]

/// Convert HEX str into array [u8; N]
#[must_use]
pub const fn hex_to_u8<const N: usize>(input: &str) -> [u8; N] {
    let mut ret = [0; N];

    let chars = input.as_bytes();

    if chars.len() == N * 2 {
        let mut idx = 0;
        loop {
            let c = chars[idx];
            let val = match c {
                b'0'..=b'9' => c - b'0',
                b'a'..=b'f' => c + 10 - b'a',
                b'A'..=b'F' => c + 10 - b'A',
                _ => 0,
            };

            ret[idx >> 1] = ret[idx >> 1] << 4 | val;
            idx += 1;
            if idx == N * 2 {
                break;
            }
        }
    }

    ret
}

/// Reverse the order of an array
#[must_use]
pub const fn reverse<const N: usize>(input: [u8; N]) -> [u8; N] {
    let mut ret = [0; N];

    // ret.reverse();

    let mut idx = 0;
    loop {
        ret[idx] = input[N - 1 - idx];

        idx += 1;
        if idx == N {
            break;
        }
    }

    ret
}

#[cfg(test)]
mod tests {
    use super::*;

    const TEST_UPPER_CASE: &str = "00112233445566778899AABBCCDDEEFF";
    const TEST_LOW_CASE: &str = "00112233445566778899aabbccddeeff";
    const TEST_MIXED: &str = "F0E1D2C3B4A5968778695a4b3c2d1e0f";
    const OUTPUT_UP: [u8; 16] = hex_to_u8::<16>(TEST_UPPER_CASE);
    const OUTPUT_LOW: [u8; 16] = hex_to_u8::<16>(TEST_LOW_CASE);

    const OUTPUT_REV: [u8; 16] = reverse(hex_to_u8::<16>(TEST_MIXED));

    #[test]
    fn normal() {
        let p = hex_to_u8::<8>(&TEST_LOW_CASE[0..16]);
        assert_eq!(p, [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77]);

        let p_rev: [u8; 8] = reverse(p);
        assert_eq!(p_rev, [0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00]);

        assert_eq!(
            &OUTPUT_UP,
            &[
                0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD,
                0xEE, 0xFF
            ]
        );

        assert_eq!(
            &OUTPUT_LOW,
            &[
                0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD,
                0xEE, 0xFF
            ]
        );

        assert_eq!(
            hex_to_u8::<16>(TEST_MIXED),
            [
                0xF0, 0xE1, 0xD2, 0xC3, 0xB4, 0xA5, 0x96, 0x87, 0x78, 0x69, 0x5A, 0x4B, 0x3C, 0x2D,
                0x1E, 0x0F
            ]
        );

        assert_eq!(
            OUTPUT_REV,
            [
                0x0F, 0x1E, 0x2D, 0x3C, 0x4B, 0x5A, 0x69, 0x78, 0x87, 0x96, 0xA5, 0xB4, 0xC3, 0xD2,
                0xE1, 0xF0
            ]
        );
    }
}
