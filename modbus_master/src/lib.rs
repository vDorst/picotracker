#![deny(clippy::pedantic)]
#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::missing_errors_doc)]
#![allow(clippy::missing_panics_doc)]

use core::num::NonZeroU8;
use core::ops::RangeInclusive;

/// Minimal Modbus Frame Length `<ADDR> <FUNC+0x80> <ERRORCODE> <CRC_HIGH> <CRC_LOW>`
const MODBUS_FRAME_LEN_MIN: u8 = 5;

/// Valid Modbus address range
const MODBUS_ADDR_RANGE: RangeInclusive<u8> = 1..=247;
/// Modbus broadcast address
const MODBUS_ADDR_BROADCAST: u8 = 0;

#[derive(Debug, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ModbusError {
    /// Our address don´t match
    InvalidAddr,
    /// Received some data in t3.5 wait time.
    MoreDataWithinT35Wait,
    /// Received more data then our buffer
    TooLong,
    /// Received too less data
    TooShort,
    /// Crc don´t match
    Crc,
    /// Serial Parity error
    Parity,
    /// Serial OverRun error
    OverRun,
    /// Serial Frame error
    Frame,
}

type MbTmrResult = Result<NonZeroU8, ModbusError>;

#[derive(Debug, Default, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ModbusState {
    /// Idle, waiting for a new frame
    #[default]
    Idle,
    /// Receiving a frame in progress
    Reception(u8),
    /// Receiving a frame in with error or not ours
    Emission(ModbusError),
    /// Waiting for t3.5 to expire
    WaitForT3_5(MbTmrResult),
}

pub struct Modbus<const N: usize> {
    pub buf: [u8; N],
    pub state: ModbusState,
    addr: ModbusAddr,
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
        if addr.is_some_and(|addr| !MODBUS_ADDR_RANGE.contains(&(addr))) {
            return None;
        }
        let addr = addr.and_then(NonZeroU8::new);
        Some(Self(addr))
    }
}

impl<const N: usize> Modbus<N> {
    #[must_use]
    pub fn new(addr: ModbusAddr) -> Self {
        Self {
            buf: [0; N],
            addr,
            state: ModbusState::default(),
        }
    }

    #[must_use]
    pub fn is_state_idle(&self) -> bool {
        self.state == ModbusState::Idle
    }

    pub fn reset(&mut self) {
        self.state = ModbusState::default();
    }

    /// Gives address our own or broadcast.
    fn accept_address(&self, address: u8) -> bool {
        self.addr.0.is_some_and(|addr| u8::from(addr) == address)
            || self.addr.0.is_none()
            || address == MODBUS_ADDR_BROADCAST
    }

    /// Should be called by the UART RX interrupt
    pub fn serial_recv(&mut self, data: u8) {
        match &mut self.state {
            ModbusState::Idle => {
                self.state = if self.accept_address(data) {
                    self.buf[0] = data;
                    ModbusState::Reception(0)
                } else {
                    ModbusState::Emission(ModbusError::InvalidAddr)
                };
            }
            ModbusState::Reception(idx) => {
                if usize::from(*idx) < (N - 1) {
                    *idx += 1;
                    self.buf[usize::from(*idx)] = data;
                } else {
                    self.state = ModbusState::Emission(ModbusError::TooLong);
                }
            }
            ModbusState::Emission(_) => (),
            ModbusState::WaitForT3_5(_) => {
                self.state = ModbusState::Emission(ModbusError::MoreDataWithinT35Wait);
            }
        }
    }

    /// Should be called by the UART RX interrupt incase of an serial error
    /// Error is ignored when `Idle` or there is already an error or `WaitForT3_5`.
    pub fn serial_error(&mut self, mbs_err: ModbusError) {
        match self.state {
            ModbusState::Reception(_) => self.state = ModbusState::Emission(mbs_err),
            ModbusState::Idle | ModbusState::Emission(_) | ModbusState::WaitForT3_5(_) => (),
        }
    }

    /// Should be called by the t1.5/t3.5 timer
    pub fn timer_handle(&mut self) -> Option<MbTmrResult> {
        match self.state {
            ModbusState::WaitForT3_5(mbs_err) => {
                // Go back to Idle
                self.state = ModbusState::Idle;

                Some(mbs_err)
            }
            ModbusState::Reception(idx) => {
                // To fit idx in a u8 the address is not counted!
                // first byte of the buffer is filled once state is changed to `Reception`;
                let idx = usize::from(idx) + 1;
                let mbs_err = if idx >= usize::from(MODBUS_FRAME_LEN_MIN) {
                    let crc_start = idx - 2;
                    let crc_data = &self.buf[0..crc_start];
                    let crc_calc = crc16(crc_data).to_le_bytes();
                    let crc_calc = crc_calc.as_slice();
                    let crc_recv = &self.buf[crc_start..idx];
                    if crc_calc == crc_recv {
                        // if let Some(buf) = SERMB::alloc() {
                        //     let mut d = buf.init(Vec::new());
                        //     d.clone_from(&self.buf);
                        //     let _ = self.queue.enqueue(d);
                        // }
                        #[cfg(feature = "std")]
                        println!("Got valid data D: {:02x?}", crc_data);
                        Ok(NonZeroU8::try_from(u8::try_from(crc_start & 0xFF).unwrap()).unwrap())
                    } else {
                        // #[cfg(feature = "defmt")]
                        // defmt::println!("CRC C: {:X} R {:X} D: {:X}", crc_calc, crc_recv, crc_data);
                        #[cfg(feature = "std")]
                        println!(
                            "CRC C: {:02x?} R {:02x?} D: {:02x?}",
                            crc_calc, crc_recv, crc_data
                        );
                        Err(ModbusError::Crc)
                    }
                } else {
                    Err(ModbusError::TooShort)
                };
                self.state = ModbusState::WaitForT3_5(mbs_err);
                None
            }
            ModbusState::Emission(mbs_err) => {
                self.state = ModbusState::WaitForT3_5(Err(mbs_err));
                None
            }
            ModbusState::Idle => None,
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
    use super::{create_request, Modbus, ModbusAddr, ModbusError, ModbusState};
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

    const VALID_TEST_LONG: &[u8; 256] = &[
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
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xee,
        0x0e,
    ];

    #[test]
    fn basic() {
        let mut mb = Modbus::<32>::new(ModbusAddr(NonZeroU8::new(0x01)));

        // Valid
        for data in VALID_TEST_SET {
            for &byte in data {
                mb.serial_recv(byte);
            }

            let idx_data_without_crc =
                NonZeroU8::try_from(u8::try_from(data.len() - 2).unwrap()).unwrap();

            assert_eq!(
                mb.state,
                ModbusState::Reception(u8::from(idx_data_without_crc) + 1)
            );
            assert_eq!(mb.timer_handle(), None);
            assert_eq!(mb.state, ModbusState::WaitForT3_5(Ok(idx_data_without_crc)));

            assert_eq!(mb.timer_handle(), Some(Ok(idx_data_without_crc)));
            assert_eq!(mb.state, ModbusState::Idle);
        }
    }

    #[test]
    fn crc_error() {
        let mut mb = Modbus::<32>::new(ModbusAddr(NonZeroU8::new(0x01)));

        // Valid
        for data in VALID_TEST_SET {
            for &byte in &data[0..data.len() - 1] {
                mb.serial_recv(byte);
            }
            mb.serial_recv(0xF0);

            let idx: u8 = u8::try_from(data.len() - 1).unwrap();
            assert_eq!(mb.state, ModbusState::Reception(idx));
            assert_eq!(mb.timer_handle(), None);

            assert_eq!(mb.state, ModbusState::WaitForT3_5(Err(ModbusError::Crc)));
            assert_eq!(mb.timer_handle(), Some(Err(ModbusError::Crc)));
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
        let mut mb = Modbus::<256>::new(ModbusAddr(NonZeroU8::new(0x01)));

        for &byte in VALID_TEST_LONG {
            mb.serial_recv(byte);
        }
        mb.serial_recv(0xFF);

        assert_eq!(mb.state, ModbusState::Emission(crate::ModbusError::TooLong));
        assert_eq!(mb.timer_handle(), None);
        assert_eq!(
            mb.state,
            ModbusState::WaitForT3_5(Err(ModbusError::TooLong))
        );

        assert_eq!(mb.timer_handle(), Some(Err(ModbusError::TooLong)));
        assert_eq!(mb.state, ModbusState::Idle);
    }

    #[test]
    fn packet_too_short() {
        let mut mb = Modbus::<32>::new(ModbusAddr(NonZeroU8::new(0x01)));

        assert_eq!(mb.state, ModbusState::Idle);

        for &byte in &VALID_TEST_LONG[0..4] {
            mb.serial_recv(byte);
        }

        assert_eq!(mb.state, ModbusState::Reception(3));

        assert_eq!(mb.timer_handle(), None);
        assert_eq!(
            mb.state,
            ModbusState::WaitForT3_5(Err(ModbusError::TooShort))
        );
        assert_eq!(mb.timer_handle(), Some(Err(ModbusError::TooShort)));
        assert_eq!(mb.state, ModbusState::Idle);
    }

    #[test]
    fn packet_max_packet() {
        let mut mb = Modbus::<256>::new(ModbusAddr(NonZeroU8::new(0x01)));

        for &byte in VALID_TEST_LONG {
            mb.serial_recv(byte);
        }

        assert_eq!(mb.state, ModbusState::Reception(u8::MAX));
        assert_eq!(mb.timer_handle(), None);

        assert_eq!(
            mb.state,
            ModbusState::WaitForT3_5(Ok(NonZeroU8::new(254).unwrap()))
        );
        assert_eq!(mb.timer_handle(), Some(Ok(NonZeroU8::new(254).unwrap())));

        assert_eq!(mb.state, ModbusState::Idle);
    }

    #[test]
    fn packet_valid_serial_error() {
        let mut mb = Modbus::<256>::new(ModbusAddr(NonZeroU8::new(0x01)));

        for &byte in VALID_TEST_LONG {
            mb.serial_recv(byte);
        }

        assert_eq!(mb.state, ModbusState::Reception(u8::MAX));

        mb.serial_error(ModbusError::Parity);

        assert_eq!(mb.timer_handle(), None);

        assert_eq!(mb.state, ModbusState::WaitForT3_5(Err(ModbusError::Parity)));
        assert_eq!(mb.timer_handle(), Some(Err(ModbusError::Parity)));

        assert_eq!(mb.state, ModbusState::Idle);
    }
}
