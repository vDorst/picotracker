#![deny(clippy::pedantic)]
#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::missing_errors_doc)]
#![allow(clippy::missing_panics_doc)]

use core::num::NonZeroU8;
use core::ops::RangeInclusive;

/// Minimal Modbus Frame Length `<ADDR> <FUNC+0x80> <ERRORCODE> <CRC_HIGH> <CRC_LOW>`
const MODBUS_FRAME_LEN_MIN: u8 = 5;
/// Maximum Modbus RTU `FrameLength`.
pub const MODBUS_FRAME_LEN_MAX: u8 = 255;

/// Valid Modbus address range
const MODBUS_SLAVE_ADDR_RANGE: RangeInclusive<u8> = 1..=247;

#[derive(Debug, Default, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ModbusStateError {
    #[default]
    None,
    InvalidAddr,
    MoreDataAtferT35,
    SerialError,
    TooLong,
    TooShort,
    Crc,
}

#[derive(Debug, Default, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ModbusState {
    #[default]
    Idle,
    InTrans(ModbusStateError),
    WaitForT3_5(ModbusStateError),
}

pub struct Modbus<const N: usize> {
    pub buf: [u8; N],
    idx: u8,
    addr: ModbusAddr,
    pub state: ModbusState,
    // tmr: Capture<TIM9, 10000>,
    //queue: Producer<'static, Box<SERMB>, 4>,
}

/// CRC-16-IBM/CRC-16-ANSI
#[must_use]
pub fn crc16(data: &[u8]) -> u16 {
    data.iter().fold(0xFFFF_u16, |mut crc, &byte| {
        crc ^= u16::from(byte);
        for _ in 0..u8::BITS {
            let is_lsb_set = (crc & 0x0001) != 0;
            crc >>= 1;
            if is_lsb_set {
                crc ^= 0xA001;
            }
        }
        crc
    })
}

pub struct ModbusAddr(Option<NonZeroU8>);

impl ModbusAddr {
    /// Validates modbus address
    pub fn new(addr: Option<u8>) -> Option<Self> {
        if addr.is_some_and(|addr| !MODBUS_SLAVE_ADDR_RANGE.contains(&(addr))) {
            return None;
        }
        let addr = addr.and_then(NonZeroU8::new);
        Some(Self(addr))
    }
}

#[derive(Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ModbusError {
    #[default]
    None,
    Nok,
    Crc,
}

impl<const N: usize> Modbus<N> {
    #[must_use]
    pub fn new(addr: ModbusAddr) -> Self {
        Self {
            buf: [0; N],
            idx: 0,
            addr,
            state: ModbusState::default(),
        }
    }

    #[must_use]
    pub fn idx(&self) -> usize {
        usize::from(self.idx)
    }

    #[allow(clippy::result_unit_err)]
    pub fn push(&mut self, byte: u8) -> Result<(), ()> {
        let idx = self.idx();
        if idx == self.buf.len() {
            Err(())
        } else {
            self.buf[idx] = byte;
            self.idx += 1;
            Ok(())
        }
    }

    #[must_use]
    pub fn is_state_idle(&self) -> bool {
        self.state == ModbusState::Idle
    }

    pub fn reset(&mut self) {
        self.idx = 0;
        self.state = ModbusState::default();
    }

    /// Should be called by the UART RX interrupt
    pub fn char_recv(&mut self, data: u8) {
        match &self.state {
            ModbusState::Idle => {
                self.state = if self.addr.0.is_none()
                    || self.addr.0.is_some_and(|addr| u8::from(addr) == data)
                    || data == 0
                {
                    // Clear the buffer.
                    self.idx = 0;
                    let _ = self.push(data);
                    ModbusState::InTrans(ModbusStateError::None)
                } else {
                    ModbusState::InTrans(ModbusStateError::InvalidAddr)
                };
            }
            ModbusState::InTrans(err) => {
                if matches!(err, ModbusStateError::None) && self.push(data).is_err() {
                    self.state = ModbusState::InTrans(ModbusStateError::TooLong);
                }
            }
            ModbusState::WaitForT3_5(_) => {
                self.state = ModbusState::InTrans(ModbusStateError::MoreDataAtferT35);
            }
        }
    }

    /// Should be called bij de t1.5/t3.5 timer interrupt
    pub fn timer_handle(&mut self) -> Result<u8, ModbusStateError> {
        match self.state {
            ModbusState::WaitForT3_5(mbs_err) => {
                // Go back to Idle
                self.state = ModbusState::Idle;

                // self.tmr.disable_timer();
                if matches!(mbs_err, ModbusStateError::None) {
                    if self.idx >= MODBUS_FRAME_LEN_MIN {
                        let crc_start = self.idx() - 2;
                        let crc_data = &self.buf[0..crc_start];
                        let crc_calc = crc16(crc_data).to_le_bytes();
                        let crc_calc = crc_calc.as_slice();
                        let crc_recv = &self.buf[crc_start..self.idx()];
                        if crc_calc == crc_recv {
                            // if let Some(buf) = SERMB::alloc() {
                            //     let mut d = buf.init(Vec::new());
                            //     d.clone_from(&self.buf);
                            //     let _ = self.queue.enqueue(d);
                            // }
                            #[cfg(feature = "std")]
                            println!("Got valid data D: {:02x?}", crc_data);
                            Ok(u8::try_from(crc_start & 0xFF).unwrap())
                        } else {
                            // #[cfg(feature = "defmt")]
                            // defmt::println!("CRC C: {:X} R {:X} D: {:X}", crc_calc, crc_recv, crc_data);
                            #[cfg(feature = "std")]
                            println!(
                                "CRC C: {:02x?} R {:02x?} D: {:02x?}",
                                crc_calc, crc_recv, crc_data
                            );
                            Err(ModbusStateError::Crc)
                        }
                    } else {
                        Err(ModbusStateError::TooShort)
                    }
                } else {
                    Err(mbs_err)
                }
            }
            ModbusState::InTrans(mbs_err) => {
                self.state = ModbusState::WaitForT3_5(mbs_err);
                Err(mbs_err)
            }
            ModbusState::Idle => Err(ModbusStateError::None),
        }
    }
}

pub fn create_request(buf: &mut [u8], addr: u8, reg: u16, len: u8) -> Option<NonZeroU8> {
    if buf.len() < 8 || !(1..=125).contains(&len) {
        return None;
    }
    buf[0] = addr;
    buf[1] = 0x03;
    buf[2..4].copy_from_slice(reg.to_be_bytes().as_slice());
    buf[4..6].copy_from_slice(u16::from(len).to_be_bytes().as_slice());

    let crc = crc16(&buf[0..6]);
    buf[6..8].copy_from_slice(crc.to_le_bytes().as_slice());

    NonZeroU8::new(8)
}

#[cfg(test)]
mod test {
    use super::{
        create_request, Modbus, ModbusAddr, ModbusState, ModbusStateError, MODBUS_FRAME_LEN_MAX,
    };
    use core::num::NonZeroU8;

    #[test]
    fn create_requests() {
        let mut buf = [0u8; 8];

        // valid
        let ret = create_request(&mut buf, 0x01, 0x1234, 1);
        assert_eq!(ret, NonZeroU8::new(8));
        assert_eq!(&buf, b"\x01\x03\x12\x34\x00\x01\xC0\xBC");

        // buffer to small
        let ret = create_request(&mut buf[0..7], 0x01, 0x1234, 1);
        assert!(ret.is_none());

        // len = 0
        let ret = create_request(&mut buf, 0x01, 0x1234, 0);
        assert!(ret.is_none());

        // len > 125
        let ret = create_request(&mut buf, 0x01, 0x1234, 126);
        assert!(ret.is_none());
    }

    const VALID_TEST_SET: [&[u8]; 3] = [
        b"\x01\x03\x12\x34\x00\x01\xC0\xBC",
        b"\x01\x03\x00\x00\x00\x02\xC4\x0B",
        b"\x01\x03\x06\x00\x00\x00\x00\x00\x03\x61\x74",
    ];

    const VALID_TEST_LONG: &[u8; MODBUS_FRAME_LEN_MAX as usize] = &[
        0x01, 0x03, 252, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xae,
    ];

    #[test]
    fn basic() {
        let mut mb = Modbus::<32>::new(ModbusAddr(NonZeroU8::new(0x01)));

        // Valid
        for data in VALID_TEST_SET {
            for &byte in data {
                mb.char_recv(byte);
            }

            assert_eq!(mb.state, ModbusState::InTrans(ModbusStateError::None));
            assert_eq!(mb.idx(), data.len());

            assert_eq!(mb.timer_handle(), Err(ModbusStateError::None));
            assert_eq!(mb.state, ModbusState::WaitForT3_5(ModbusStateError::None));

            assert_eq!(mb.idx(), data.len());

            assert_eq!(
                mb.timer_handle(),
                Ok(u8::try_from(data.len() & 0xFF).unwrap() - 2)
            );
            assert_eq!(mb.state, ModbusState::Idle);
        }
    }

    #[test]
    fn crc_error() {
        let mut mb = Modbus::<32>::new(ModbusAddr(NonZeroU8::new(0x01)));

        // Valid
        for data in VALID_TEST_SET {
            for &byte in &data[0..data.len() - 1] {
                mb.char_recv(byte);
            }
            mb.char_recv(0xFF);

            assert_eq!(mb.state, ModbusState::InTrans(ModbusStateError::None));

            assert_eq!(mb.timer_handle(), Err(ModbusStateError::None));
            assert_eq!(mb.state, ModbusState::WaitForT3_5(ModbusStateError::None));

            assert_eq!(mb.timer_handle(), Err(ModbusStateError::Crc));
            assert_eq!(mb.state, ModbusState::Idle);
        }
    }

    #[test]
    fn parse_values() {
        let data = &[1, 3, 6, 0, 0, 6, 0xC9, 0xBE, 0x6A];
        let mut buf = 0_u64.to_ne_bytes();
        buf[2..].copy_from_slice(&data[3..9]);

        let val = u64::from_be_bytes(buf);

        assert_eq!(val, 113_884_778);
    }

    #[test]
    fn packet_too_long() {
        let mut mb = Modbus::<37>::new(ModbusAddr(NonZeroU8::new(0x01)));

        for &byte in VALID_TEST_LONG {
            mb.char_recv(byte);
        }

        assert_eq!(
            mb.state,
            ModbusState::InTrans(crate::ModbusStateError::TooLong)
        );

        assert_eq!(mb.timer_handle(), Err(ModbusStateError::TooLong));
        assert_eq!(
            mb.state,
            ModbusState::WaitForT3_5(ModbusStateError::TooLong)
        );

        assert_eq!(mb.timer_handle(), Err(ModbusStateError::TooLong));
        assert_eq!(mb.state, ModbusState::Idle);
    }

    #[test]
    fn packet_too_short() {
        let mut mb = Modbus::<32>::new(ModbusAddr(NonZeroU8::new(0x01)));

        for &byte in &VALID_TEST_LONG[0..4] {
            mb.char_recv(byte);
        }

        assert_eq!(
            mb.state,
            ModbusState::InTrans(crate::ModbusStateError::None)
        );

        assert_eq!(mb.timer_handle(), Err(ModbusStateError::None));
        assert_eq!(mb.state, ModbusState::WaitForT3_5(ModbusStateError::None));

        assert_eq!(mb.timer_handle(), Err(ModbusStateError::TooShort));
        assert_eq!(mb.state, ModbusState::Idle);
    }

    #[test]
    fn packet_max_packet() {
        let mut mb = Modbus::<256>::new(ModbusAddr(NonZeroU8::new(0x01)));

        for &byte in VALID_TEST_LONG {
            mb.char_recv(byte);
        }

        assert_eq!(
            mb.state,
            ModbusState::InTrans(crate::ModbusStateError::None)
        );

        assert_eq!(mb.timer_handle(), Err(ModbusStateError::None));
        assert_eq!(mb.state, ModbusState::WaitForT3_5(ModbusStateError::None));

        assert_eq!(mb.timer_handle(), Ok(VALID_TEST_LONG.len() as u8 - 2));
        assert_eq!(mb.state, ModbusState::Idle);
    }
}
