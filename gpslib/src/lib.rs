// This file is based on https://github.com/MechanicalPython/adafruit_gps
#![deny(clippy::pedantic)]
#![feature(async_fn_in_trait)]
#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::missing_errors_doc)]
#![allow(clippy::missing_panics_doc)]

mod slice_convert;
use core::str::FromStr;
use slice_convert::{slice_to_hex, slice_to_u32};

use chrono::{DateTime, NaiveDate, NaiveDateTime, NaiveTime, Utc};

pub use chrono;

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Compass {
    North,
    East,
    South,
    West,
}

impl TryFrom<u8> for Compass {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            b'N' => Ok(Self::North),
            b'W' => Ok(Self::West),
            b'S' => Ok(Self::South),
            b'E' => Ok(Self::East),
            _ => Err(()),
        }
    }
}

impl TryFrom<char> for Compass {
    type Error = ();

    fn try_from(value: char) -> Result<Self, Self::Error> {
        match value {
            'N' => Ok(Self::North),
            'W' => Ok(Self::West),
            'S' => Ok(Self::South),
            'E' => Ok(Self::East),
            _ => Err(()),
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PosMode {
    NoFix,
    Autonomous,
    Differntial,
}

impl TryFrom<u8> for PosMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            b'N' => Ok(Self::NoFix),
            b'A' => Ok(Self::Autonomous),
            b'D' => Ok(Self::Differntial),
            _ => Err(()),
        }
    }
}

impl TryFrom<char> for PosMode {
    type Error = ();

    fn try_from(value: char) -> Result<Self, Self::Error> {
        match value {
            'N' => Ok(Self::NoFix),
            'A' => Ok(Self::Autonomous),
            'D' => Ok(Self::Differntial),
            _ => Err(()),
        }
    }
}

#[derive(Debug, PartialEq)]
pub struct PosData {
    pub lastitude: (Compass, f32),
    /// Longitude in format "dddmm.mmmm" (degrees and minutes
    pub longitude: (Compass, f32),
}

#[derive(Debug, PartialEq)]
// #[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// RMC, Recommended Minimum Position Data (including position, velocity and time
pub struct GpsRmc {
    pub timedate: DateTime<Utc>,

    /// Speed in knots
    pub speed: f32,
    /// Course over ground in degree
    pub cod: f32,
    pub pos_mode: PosMode,
    pub data: Option<PosData>,
}

impl GpsRmc {
    pub fn parse(line: &[u8]) -> Result<Self, usize> {
        //  "$GNRMC,084629.000,A,3150.7822,N,11711.9323,E,0.00,119.00,240715,,,D";
        let mut ret = Self {
            timedate: NaiveDateTime::default().and_utc(),
            speed: 0.0,
            cod: 0.0,
            data: Some(PosData {
                lastitude: (Compass::North, 0.0),
                longitude: (Compass::North, 0.0),
            }),
            pos_mode: PosMode::NoFix,
        };

        let mut time = NaiveTime::default();
        let mut date = NaiveDate::default();

        let mut lines = line.split(|c| *c == b',');

        for n in 0..13 {
            let slice = lines.next().ok_or(n)?;
            match n {
                0 => {
                    if slice != b"$GNRMC" {
                        return Err(n);
                    }
                }
                1 => {
                    if let Some(t) = NaiveTime::from_hms_milli_opt(
                        slice_to_u32(&slice[0..2]),
                        slice_to_u32(&slice[2..4]),
                        slice_to_u32(&slice[4..6]),
                        slice_to_u32(&slice[7..10]),
                    ) {
                        time = t;
                    } else {
                        return Err(n);
                    }
                }
                2 => {
                    if slice[0] != b'A' {
                        ret.data = None;
                    }
                }
                3 => {
                    if let Some(data) = &mut ret.data {
                        data.lastitude.1 =
                            f32::from_str(core::str::from_utf8(slice).map_err(|_| n)?)
                                .map_err(|_| n)?;
                    }
                }
                5 => {
                    if let Some(data) = &mut ret.data {
                        data.longitude.1 =
                            f32::from_str(core::str::from_utf8(slice).map_err(|_| n)?)
                                .map_err(|_| n)?;
                    }
                }
                7 => {
                    ret.speed = f32::from_str(core::str::from_utf8(slice).map_err(|_| n)?)
                        .map_err(|_| n)?;
                }
                8 => {
                    ret.cod = f32::from_str(core::str::from_utf8(slice).map_err(|_| n)?)
                        .map_err(|_| n)?;
                }
                10 | 11 => (),
                4 => {
                    if let Some(data) = &mut ret.data {
                        data.lastitude.0 = Compass::try_from(slice[0]).map_err(|()| n)?;
                    }
                }
                6 => {
                    if let Some(data) = &mut ret.data {
                        data.longitude.0 = Compass::try_from(slice[0]).map_err(|()| n)?;
                    }
                }

                9 => {
                    if let Some(t) = NaiveDate::from_ymd_opt(
                        i32::try_from(slice_to_u32(&slice[4..6])).map_err(|_| n)? + 2000,
                        slice_to_u32(&slice[2..4]),
                        slice_to_u32(&slice[0..2]),
                    ) {
                        date = t;
                    } else {
                        return Err(n);
                    }
                }
                12 => ret.pos_mode = PosMode::try_from(slice[0]).map_err(|()| n)?,
                _ => return Err(n),
            }
        }

        ret.timedate = date.and_time(time).and_utc();

        Ok(ret)
    }
}

pub type GPSType = GpsRmc;

#[repr(u8)]
#[derive(Debug, PartialEq, Eq)]
enum RecvState {
    WaitForStart,
    WaitForDataType,
    GetData,
}

pub struct Gps {
    buf: [u8; 100],
    idx: u8,
    state: RecvState,
}

impl Gps {
    #[must_use]
    pub fn new() -> Self {
        Self {
            buf: [0; 100],
            idx: 0,
            state: RecvState::WaitForStart,
        }
    }

    fn idx(&self) -> usize {
        usize::from(self.idx)
    }

    pub fn reset_state(&mut self) {
        self.idx = 0;
        self.state = RecvState::WaitForStart;
    }

    pub fn as_mut_data(&mut self) -> &mut [u8] {
        let idx = self.idx();
        &mut self.buf[idx..=idx]
    }

    #[must_use]
    pub fn as_data(&self) -> &[u8] {
        let idx = self.idx();
        &self.buf[0..idx]
    }

    #[must_use]
    pub fn is_valid_checksum(&self) -> bool {
        let s = self.as_data();
        let star = s[s.len() - 4];
        let checksum = &s[s.len() - 3..s.len() - 1];
        let body = &s[1..s.len() - 4];

        if star != b'*' {
            // Check third last item is a *
            // println!("no star {}", star);
            return false;
        }

        let expected_checksum = slice_to_hex(checksum);
        let actual: u8 = body.iter().fold(0, |acc, &byte| acc ^ byte);
        actual == expected_checksum
    }

    pub fn process(&mut self) -> Option<GPSType> {
        let char = self.buf[self.idx()];
        match self.state {
            RecvState::WaitForStart => {
                if char == b'$' {
                    self.idx += 1;
                    self.state = RecvState::WaitForDataType;
                }
            }
            RecvState::WaitForDataType => {
                self.idx += 1;
                if char == b',' {
                    let line = self.as_data();
                    // println!("GPSWAIT: {} \"{}\"", line.len(), core::str::from_utf8(line).unwrap());
                    if line == b"$GNRMC," {
                        self.state = RecvState::GetData;
                    } else {
                        self.reset_state();
                    }
                }
            }
            RecvState::GetData => {
                if char == b'\n' {
                    if self.buf[self.idx() - 1] == b'\r' {
                        let valid = self.is_valid_checksum();
                        self.idx -= 2;
                        let mut line = self.as_data();
                        line = &line[0..line.len() - 2];
                        if valid {
                            // if let Ok(s) = core::str::from_utf8(line) {
                            //     let mut g = GPSType::new();
                            //     let _ = g.push_str(s);
                            //     self.reset_state();
                            //     return Some(g);
                            // }
                            // #[cfg(feature = "defmt")]
                            // defmt::println!("{}", core::str::from_utf8(line).unwrap());
                            if let Ok(s) = GPSType::parse(line) {
                                self.reset_state();
                                return Some(s);
                            }
                        }
                    }
                    self.reset_state();
                } else {
                    self.idx += 1;
                    if self.idx() >= self.buf.len() {
                        self.reset_state();
                    }
                }
            }
        }

        None
    }
}

impl Default for Gps {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod test {
    use chrono::NaiveDate;

    use crate::{Compass, GpsRmc, PosData, RecvState};

    use super::{GPSType, Gps};

    const DATA_GOOD: &[u8] =
        b"$GNRMC,084629.000,A,3150.7822,N,11711.9323,E,0.00,119.00,240715,,,D*7C\r\n";

    const DATA_WRONG_CRC: &[u8] =
        b"$GNGGA,131613.000,5132.7314,N,00005.9099,W,1,9,1.17,42.4,M,47.0,M,,*61\r\n";

    const DATA_ONLY_LF: &[u8] =
        b"$GNGGA,131613.000,5132.7314,N,00005.9099,W,1,9,1.17,42.4,M,47.0,M,,*60\n";

    const DATA_NO_FIX: &[u8] = b"$GNRMC,231521.793,V,,,,,0.03,237.77,240923,,,N";

    #[test]
    fn basic() {
        let mut gps = Gps::new();

        let mut ret: Option<GPSType> = None;

        for &byte in DATA_GOOD {
            let buf = gps.as_mut_data();
            let b = buf.get_mut(0).unwrap();
            *b = byte;
            ret = gps.process();
        }

        // b"$GNRMC,084629.000,A,3150.7822,N,11711.9323,E,0.00,119.00,240715,,,D*7C\r\n";
        match ret {
            // Some(data) => assert_eq!(data.as_bytes(), &DATA_GOOD[0..DATA_GOOD.len() - 6]),
            Some(data) => assert_eq!(
                data,
                GpsRmc {
                    timedate: NaiveDate::from_ymd_opt(2015, 7, 24)
                        .unwrap()
                        .and_hms_milli_opt(8, 46, 29, 000)
                        .unwrap()
                        .and_utc(),
                    data: Some(PosData {
                        lastitude: (Compass::North, 3150.7822),
                        longitude: (Compass::East, 11_711.933),
                    }),

                    speed: 0.0,
                    cod: 119.00,
                    pos_mode: crate::PosMode::Differntial,
                }
            ),
            None => panic!("Expect Data"),
        }
        assert_eq!(gps.state, RecvState::WaitForStart);
    }

    #[test]
    fn no_fix() {
        let ret = GpsRmc::parse(DATA_NO_FIX);
        // "$GNRMC,231521.793,V,,,,,0.03,237.77,240923,,,N\r\n";
        match ret {
            // Some(data) => assert_eq!(data.as_bytes(), &DATA_GOOD[0..DATA_GOOD.len() - 6]),
            Ok(data) => assert_eq!(
                data,
                GpsRmc {
                    timedate: NaiveDate::from_ymd_opt(2023, 9, 24)
                        .unwrap()
                        .and_hms_milli_opt(23, 15, 21, 793)
                        .unwrap()
                        .and_utc(),
                    speed: 0.03,
                    cod: 237.77,
                    data: None,
                    pos_mode: crate::PosMode::NoFix,
                }
            ),
            Err(e) => panic!("Expect Data: {e}"),
        }
    }

    #[test]
    fn wrong_crc() {
        let mut gps = Gps::new();

        let mut ret: Option<GPSType> = None;

        for &byte in DATA_WRONG_CRC {
            let buf = gps.as_mut_data();
            let b = buf.get_mut(0).unwrap();
            *b = byte;
            ret = gps.process();
        }

        assert!(ret.is_none());
        assert_eq!(gps.state, RecvState::WaitForStart);
    }

    #[test]
    fn wrong_line_ending() {
        let mut gps = Gps::new();

        let mut ret: Option<GPSType> = None;

        for &byte in DATA_ONLY_LF {
            let buf = gps.as_mut_data();
            let b = buf.get_mut(0).unwrap();
            *b = byte;
            ret = gps.process();
        }

        assert!(ret.is_none());

        assert_eq!(gps.state, RecvState::WaitForStart);
    }
}
