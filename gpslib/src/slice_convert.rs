pub fn slice_to_hex(data: &[u8]) -> u8 {
    let mut ret: u8 = 0;
    for &byte in &data[0..data.len().min(2)] {
        ret = ret.wrapping_shl(4);
        ret |= match byte {
            b'0'..=b'9' => byte - b'0',
            b'A'..=b'F' => byte + 10 - b'A',
            b'a'..=b'f' => byte + 10 - b'a',
            _ => 0,
        }
    }

    ret
}

pub fn slice_to_u32(data: &[u8]) -> u32 {
    let mut ret: u32 = 0;
    for &byte in &data[0..data.len().min(4)] {
        ret *= 10;
        ret += u32::from(match byte {
            b'0'..=b'9' => byte - b'0',
            _ => 0,
        });
    }

    ret
}

#[cfg(test)]
mod test {
    use super::{slice_to_hex, slice_to_u32};

    const TESTDATA_HEX: &[(&[u8], u8)] = &[
        (b"00", 0x00),
        (b"FF", 0xFF),
        (b"fF", 0xFF),
        (b"Ff", 0xFF),
        (b"FFF", 0xFF),
        (b"oo", 0x00),
        (b"oF", 0x0F),
        (b"Fo", 0xF0),
        (b"*FF", 0x0F),
        (b"F", 0x0F),
        (b"1", 0x01),
        (b"a", 0x0A),
        (b"9", 0x09),
    ];

    const TESTDATA_U32: &[(&[u8], u32)] = &[(b"00", 0x00), (b"FF", 0x00), (b"08", 8)];

    #[test]
    fn test_slice_to_hex() {
        for (inp, out) in TESTDATA_HEX {
            let ret = slice_to_hex(inp);
            assert_eq!(ret, *out);
        }
    }

    #[test]
    fn test_slice_to_u32() {
        for (inp, out) in TESTDATA_U32 {
            let ret = slice_to_u32(inp);
            assert_eq!(ret, *out);
        }
    }
}
