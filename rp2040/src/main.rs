//! This example shows how to use SPI (Serial Peripheral Interface) in the RP2040 chip.
//!
//! Example written for a display using the ST7789 chip. Possibly the Waveshare Pico-ResTouch
//! (https://www.waveshare.com/wiki/Pico-ResTouch-LCD-2.8)

#![no_std]
#![no_main]
#![allow(unused_imports)]
// #![feature(try_blocks)]

use core::cell::RefCell;
use core::f32::consts::PI;
use core::fmt::write;
use core::num::NonZeroI8;

use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_embedded_hal::shared_bus::SpiDeviceError;
use embassy_embedded_hal::SetConfig;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use embassy_rp::i2c::I2c;
use embassy_rp::pac::watchdog::regs::Tick;
use embassy_rp::peripherals::{I2C1, SPI1};
use embassy_rp::spi::{self, Async, Blocking, Config, Spi};
use embassy_rp::uart::{Async as UartAsync, Config as UConfig, InterruptHandler, Parity, StopBits, Uart};
use embassy_rp::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex, RawMutex};
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Ticker, Timer};
use embedded_graphics::image::{GetPixel, Image, ImageRawLE};
use embedded_graphics::mono_font::{ascii, MonoTextStyle};
use embedded_graphics::pixelcolor::raw::{LittleEndian, RawU16};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::Text;
use embedded_hal_1::digital::OutputPin;
use embedded_hal_async::spi::{Operation, SpiBus, SpiDevice};
use embedded_io_async::{Read, Write};
use gpslib::chrono::{Datelike, FixedOffset, TimeZone, Timelike};
use gpslib::{Gps, GpsRmc, PosMode};
use heapless::{String, Vec};

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::lorawan_radio::LorawanRadio;
use lora_phy::sx126x::{self, Sx1261, Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::LoRa;
use lorawan_device::async_device::{region, Device, EmbassyTimer, JoinMode, JoinResponse, SendResponse};
use lorawan_device::default_crypto::DefaultFactory as Crypto;
use lorawan_device::{AppEui, AppKey, DevEui};

use micromath::F32Ext;
use modbus_master::{create_request, Modbus, ModbusAddr, ModbusError};
use sht3x::SHT3x;
use st7735_embassy::{buffer_size, Orientation, ST7735, ST7735IF};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

const LORAWAN_REGION: region::Region = region::Region::EU868; // warning: set this appropriately for the region
const MAX_TX_POWER: u8 = 14;

const LORAWAN_DEVEUI: [u8; 8] = envconst::reverse(envconst::hex_to_u8::<8>(env!("DEVEUI")));
const LORAWAN_APPEUI: [u8; 8] = envconst::reverse(envconst::hex_to_u8::<8>(env!("APPEUI")));
const LORAWAN_APPKEY: [u8; 16] = envconst::hex_to_u8::<16>(env!("APPKEY"));

const DISPLAY_FREQ: u32 = 12_000_000;
const FB_SIZE: (usize, usize) = (160, 80);

bind_interrupts!(struct Irqs {
    UART0_IRQ => InterruptHandler<peripherals::UART0>;
    UART1_IRQ => InterruptHandler<peripherals::UART1>;
    I2C1_IRQ =>  embassy_rp::i2c::InterruptHandler<peripherals::I2C1>;
});

type GpsType = GpsRmc;

type GpsSend = Sender<'static, NoopRawMutex, GpsType, 1>;

type LoraType = Vec<u8, 48>;
type LoraStatus = String<16>;

type SpiDev = SpiDeviceWithConfig<'static, NoopRawMutex, Spi<'static, SPI1, Async>, Output<'static>>;
use core::convert::TryInto;

use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyle};

#[embassy_executor::task]
async fn lora_task(
    lora_spi: SpiDeviceWithConfig<'static, NoopRawMutex, Spi<'static, SPI1, Async>, Output<'static>>,
    lora_data_recv: Receiver<'static, NoopRawMutex, LoraType, 1>,
    lora_status_send: Sender<'static, NoopRawMutex, LoraStatus, 1>,
    reset: Output<'static>,
    dio1: Input<'static>,
    busy: Input<'static>,
) {
    let config = sx126x::Config {
        chip: Sx1262,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
        use_dcdc: true,
        rx_boost: false,
    };

    let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();

    let lora = {
        match LoRa::new(Sx126x::new(lora_spi, iv, config), true, Delay).await {
            Ok(l) => l,
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }
    };

    let radio: LorawanRadio<_, _, { MAX_TX_POWER }> = lora.into();
    let region: region::Configuration = region::Configuration::new(LORAWAN_REGION);

    let mut device: Device<_, Crypto, _, _> =
        Device::new(region, radio, EmbassyTimer::new(), embassy_rp::clocks::RoscRng);

    // device.set_datarate(region::DR::_4);

    loop {
        defmt::info!("Joining LoRaWAN network");
        let msg: LoraStatus = "Lora: Try join. ".try_into().unwrap();

        lora_status_send.send(msg).await;

        match device
            .join(&JoinMode::OTAA {
                deveui: DevEui::from(LORAWAN_DEVEUI),
                appeui: AppEui::from(LORAWAN_APPEUI),
                appkey: AppKey::from(LORAWAN_APPKEY),
            })
            .await
        {
            Ok(resp) => {
                let msg = match resp {
                    JoinResponse::JoinSuccess => "Lora: Joined    ",
                    JoinResponse::NoJoinAccept => "Lora: NoJoin    ",
                };
                defmt::info!("LoRaWAN network: {}", msg);
                let msg: LoraStatus = msg.try_into().unwrap();
                lora_status_send.send(msg).await;
                break;
            }
            Err(err) => {
                println!("Lora: Error: {:?}", err);
                let msg = match err {
                    lorawan_device::async_device::Error::Radio(e) => "Lora: SErr: Rad.",
                    lorawan_device::async_device::Error::Mac(e) => "Lora: SErr: Mac.",
                };
                lora_status_send.send(msg.try_into().unwrap()).await;
            }
        };

        Timer::after(Duration::from_secs(10)).await;
    }

    loop {
        println!("Wait for data!");
        let data = lora_data_recv.receive().await;
        let ret = device.send(&data, 1, false).await;

        let msg: &str = match ret {
            Ok(resp) => {
                let msg = match resp {
                    SendResponse::NoAck => "Lora: NoAck     ",
                    SendResponse::SessionExpired => "Lora: SessionExp",
                    SendResponse::DownlinkReceived(e) => "Lora: GotRec    ",
                    SendResponse::RxComplete => "Lora: RxComplete",
                };
                defmt::info!("LoRaWAN network: {}", msg);
                // let msg: LoraStatus = msg.try_into().unwrap();
                println!("Send Done");
                "Lora: Sended"
            }
            Err(e) => {
                println!("Lora: Error: {:?}", e);
                let msg = match e {
                    lorawan_device::async_device::Error::Radio(e) => "Lora: SErr: Rad.",
                    lorawan_device::async_device::Error::Mac(e) => "Lora: SErr: Mac.",
                };

                msg
            }
        };

        if let Ok(msg) = msg.try_into() {
            lora_status_send.send(msg).await;
        } else {
            error!("MSG: {} doesn't fit!", msg);
        }

        println!("sleep 5 min");
        Timer::after(Duration::from_secs(5 * 60)).await;
    }
}

#[embassy_executor::task]
async fn display_task(
    display: &'static mut Dsp,
    gps_recv: Receiver<'static, NoopRawMutex, GpsType, 1>,
    gps_pps: Input<'static>,
    mut bl: Output<'static>,
    lora_recv: Receiver<'static, NoopRawMutex, LoraStatus, 1>,
) {
    let mut delay = Delay;

    let mut step: u8 = 0;
    let rad = 20.0;

    let mut g: GpsType;
    let mut sp: String<25> = "No GPS update".try_into().unwrap();

    let mut lora_s: LoraStatus = "No LORA update".try_into().unwrap();

    let area = Rectangle {
        top_left: Point::new(0, 0),
        size: Size::new(160, 80),
    };
    let _ = display.clear(Rgb565::BLUE);

    let (ex, ey) = {
        let s = display.size();
        let x = s.width as u16 - 1;
        let y = s.height as u16 - 1;
        (x, y)
    };

    display.flush().await.unwrap();
    bl.set_high();

    let center = Point::new(80 - 1, 40 - 1);
    let mut frame = Ticker::every(Duration::from_hz(1));
    let mut ret: f32;

    let mut col_iter = [Rgb565::RED, Rgb565::GREEN, Rgb565::BLUE].into_iter().cycle();
    let mut col = col_iter.next().unwrap();

    let style = MonoTextStyle::new(&ascii::FONT_8X13, Rgb565::WHITE);
    Text::new(
        "Rust",
        Point::new(0, (style.font.baseline + style.font.character_size.height) as i32),
        style,
    )
    .draw(display)
    .unwrap();

    display.flush().await.unwrap();

    // loop
    loop {
        if step >= 60 {
            step = 0;
            col = col_iter.next().unwrap();
        }

        // Line
        ret = f32::from(step) * PI * 2.0 / 60.0;
        let (sx, sy) = (ret.sin() * rad, ret.cos() * rad);
        let end = Point::new((sx.trunc()) as i32 + center.x, center.y - (sy.trunc()) as i32);
        let line = Line::new(center, end).into_styled(PrimitiveStyle::with_stroke(col, 3));
        unwrap!(line.draw(display));

        if let Ok(p) = lora_recv.try_receive() {
            lora_s = p;
        }

        if let Ok(p) = gps_recv.try_receive() {
            g = p;

            if matches!(g.pos_mode, PosMode::NoFix) || g.data.is_none() {
                sp = "GPS: No Fix".try_into().unwrap();
            } else {
                sp = "".try_into().unwrap();
                let time = g.timedate.with_timezone(&FixedOffset::east_opt(2 * 3600).unwrap());

                let hour = time.time().hour();
                let min = time.time().minute();
                let sec = time.time().second();

                u32_to_double(hour, &mut sp);
                let _ = sp.push_str(":");
                u32_to_double(min, &mut sp);
                let _ = sp.push_str(":");
                u32_to_double(sec, &mut sp);
                let _ = sp.push_str(" ");

                let day = time.day();
                let month = time.month();
                let year = time.year() as u32 % 100;

                u32_to_double(day, &mut sp);
                let _ = sp.push_str("-");
                u32_to_double(month, &mut sp);
                let _ = sp.push_str("-'");
                u32_to_double(year, &mut sp);
            }
        }

        let _ = display.fill_solid(
            &Rectangle {
                top_left: Point { x: 0, y: 0 },
                size: Size::new(159, style.font.character_size.height),
            },
            Rgb565::BLACK,
        );
        Text::new(&sp, Point::new(0, (style.font.baseline) as i32), style)
            .draw(display)
            .unwrap();

        let lora_txt = Text::new(
            &lora_s,
            Point::new(0, (style.font.baseline + (5 * style.font.character_size.height)) as i32),
            style,
        );

        sp.clear();
        if let Some(td) = TEMPDATA.try_take() {
            let _ = sp.push_str("T:");
            let temp: i16 = td.t.into();
            i16_to_str(temp, &mut sp);
            let _ = sp.push(' ');
            let hum: u8 = td.h.into();
            u32_to_double(u32::from(hum), &mut sp);
            let _ = sp.push('%');

            let temp_txt = Text::new(
                &sp,
                Point::new(0, (style.font.baseline + (4 * style.font.character_size.height)) as i32),
                style,
            );
            temp_txt.draw(display).unwrap();
        }

        let clear_rec = lora_txt
            .bounding_box()
            .resized_width(160, embedded_graphics::geometry::AnchorX::Right);

        let _ = display.fill_solid(&clear_rec, Rgb565::BLACK);

        lora_txt.draw(display).unwrap();

        let gps_col = if gps_pps.is_high() { Rgb565::GREEN } else { Rgb565::BLUE };

        display.set_pixel(159, 0, gps_col.into_storage());

        display.flush().await.unwrap();

        frame.next().await;
        // let line = Line::new(center, end).into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 3));
        // unwrap!(line.draw(display));

        display.clear(Rgb565::BLACK);

        step += 1;
    }
}

#[embassy_executor::task]
async fn gps_handle(mut uart: Uart<'static, peripherals::UART0, UartAsync>, send: GpsSend) {
    let mut gps = Gps::new();

    loop {
        match uart.read(gps.as_mut_data()).await {
            Ok(()) => {
                if let Some(msg) = gps.process() {
                    // println!("{}", msg.pos_mode);
                    send.send(msg).await;
                }
            }
            Err(e) => {
                println!("GPS: Uart error: {}", e);
                gps.reset_state();
            }
        }
    }
}

#[embassy_executor::task]
async fn tempdata_handle(mut i2c1: I2c<'static, I2C1, embassy_rp::i2c::Async>) {
    let mut sht = SHT3x::new(&mut i2c1);

    let _ = sht.reset().await;
    let mut tick = Ticker::every(Duration::from_hz(1));
    Timer::after(Duration::from_millis(10)).await;

    sht.write(sht3x::CMD::AUTO_1MPS_HIGH).await;

    loop {
        tick.next().await;
        match sht.get_measurement().await {
            Ok((t, h)) => {
                println!("t: {} h {}", t, h);
                TEMPDATA.signal(TempData { t, h });
            }
            Err(e) => {
                println!("TEMP: I2C error: {}", e);
            }
        }
    }
}

type MBR = (u16, ModBusTypes, &'static str);

pub enum ModBusTypes {
    Uint32,
    Uint48,
    Uint16,
    Uint8,
    Char8Array32,
}

#[derive(Debug)]
pub enum ModBusTypesDecode {
    Uint32(u32),
    Uint48(u64),
    Uint16(u16),
    Uint8(u8),
    Char8Array32([u8; 32]),
}

impl defmt::Format for ModBusTypesDecode {
    fn format(&self, fmt: Formatter) {
        match self {
            ModBusTypesDecode::Uint32(val) => val.format(fmt),
            ModBusTypesDecode::Uint48(val) => val.format(fmt),
            ModBusTypesDecode::Uint16(val) => val.format(fmt),
            ModBusTypesDecode::Uint8(val) => val.format(fmt),
            ModBusTypesDecode::Char8Array32(val) => {
                let mut buf: [u8; 32] = val.clone();
                let null = buf.iter().position(|c| *c == b'\0').unwrap_or(val.len());
                let data = &mut buf[0..null];
                data.iter_mut().for_each(|c| {
                    if !c.is_ascii() {
                        *c = b'?';
                    }
                });
                let s = unsafe { core::str::from_utf8_unchecked(data) };
                s.format(fmt)
            }
        }
    }
}

impl core::fmt::Display for ModBusTypesDecode {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ModBusTypesDecode::Uint32(val) => f.write_fmt(format_args!("{val}")),
            ModBusTypesDecode::Uint48(val) => f.write_fmt(format_args!("{val}")),
            ModBusTypesDecode::Uint16(val) => f.write_fmt(format_args!("{val}")),
            ModBusTypesDecode::Uint8(val) => f.write_fmt(format_args!("{val}")),
            ModBusTypesDecode::Char8Array32(val) => {
                let mut buf: [u8; 32] = val.clone();
                let null = buf.iter().position(|c| *c == b'\0').unwrap_or(val.len());
                let data = &mut buf[0..null];
                data.iter_mut().for_each(|c| {
                    if !c.is_ascii() {
                        *c = b'?';
                    }
                });
                let s = unsafe { core::str::from_utf8_unchecked(data) };
                f.write_str(s)
            }
        }
    }
}

#[repr(u8)]
#[derive(Debug, Format)]
pub enum ModBusResponceError {
    IllegalFunctionCode = 0x01,
    IllegelDataAddress = 0x02,
    IllegelDataValue = 0x03,
    ServerFailure = 0x04,
    Acknowledge = 0x05,
    ServerBusy = 0x06,
    GatewayNoPath = 0x0A,
    GatewayRemoteTargetNoResponce = 0x0B,
    InvalidFrame = 0xFE,
    Other = 0xFF,
}

impl ModBusResponceError {
    pub fn try_from(val: u8) -> Self {
        match val {
            0x01 => Self::IllegalFunctionCode,
            0x02 => Self::IllegelDataAddress,
            0x03 => Self::IllegelDataValue,
            0x04 => Self::ServerFailure,
            0x05 => Self::Acknowledge,
            0x06 => Self::ServerBusy,
            0x0A => Self::GatewayNoPath,
            0x0B => Self::GatewayRemoteTargetNoResponce,
            _ => Self::Other,
        }
    }
}

impl ModBusTypes {
    pub fn register_no(&self) -> u8 {
        match self {
            ModBusTypes::Uint32 => 2,
            ModBusTypes::Uint48 => 3,
            ModBusTypes::Uint16 => 1,
            ModBusTypes::Uint8 => 1,
            ModBusTypes::Char8Array32 => 16,
        }
    }

    pub fn parse(&self, data: &[u8]) -> Result<ModBusTypesDecode, ModBusResponceError> {
        if data.len() < 3 {
            return Err(ModBusResponceError::InvalidFrame);
        }

        if data[1] & 0x80 == 0x80 {
            return Err(ModBusResponceError::try_from(data[2]));
        }

        if usize::from(data[2]) + 3 != data.len() || usize::from(data[2]) != usize::from(self.register_no()) * 2 {
            defmt::error!("DECODE: {:02x}", data);
            return Err(ModBusResponceError::InvalidFrame);
        }

        let data = &data[3..];
        Ok(match self {
            ModBusTypes::Uint32 => ModBusTypesDecode::Uint32(u32::from_be_bytes(data[0..4].try_into().unwrap())),
            ModBusTypes::Uint48 => {
                let mut buf = 0_u64.to_ne_bytes();
                buf[2..].copy_from_slice(&data[0..6]);
                ModBusTypesDecode::Uint48(u64::from_be_bytes(buf.try_into().unwrap()))
            }
            ModBusTypes::Uint16 => ModBusTypesDecode::Uint16(u16::from_be_bytes(data[0..2].try_into().unwrap())),
            ModBusTypes::Uint8 => ModBusTypesDecode::Uint8(data[1]),
            ModBusTypes::Char8Array32 => {
                let mut buf: [u8; 32] = [0; 32];
                buf[..].copy_from_slice(data);
                ModBusTypesDecode::Char8Array32(buf)
            }
        })
    }
}

// Runtime variables
const RUN_TIME_VARIABLES: [MBR; 10] = [
    (0x23c, ModBusTypes::Uint32, "FlowRate"),
    (0x236, ModBusTypes::Uint48, "Total"),
    (0x230, ModBusTypes::Uint48, "AccTotal"),
    (0x204, ModBusTypes::Uint16, "ErrorStatus"),
    (0x00FF, ModBusTypes::Uint16, "BatteryStatus"),
    (0x1208, ModBusTypes::Char8Array32, "ProductModelName"),
    (0x00A2, ModBusTypes::Uint32, "SoftwareVersion"),
    (0x00A5, ModBusTypes::Uint32, "SerialNumber"),
    (0x1200, ModBusTypes::Uint8, "ProductSeriesId"),
    (0x0093, ModBusTypes::Uint8, "E_COM_DELAY"),
];

#[embassy_executor::task]
async fn modbus_handle(mut uart: Uart<'static, peripherals::UART1, UartAsync>) {
    let mut mb = Modbus::<40>::new(ModbusAddr::new(1).unwrap());

    let mut buf = [0_u8; 16];
    let mut tick = Ticker::every(Duration::from_secs(5));

    // let mut regs = RUN_TIME_VARIABLES.iter().cycle();
    loop {
        // let (reg, ty, name) = regs.next().unwrap();

        defmt::info!("MB: Readout!");
        for (reg, ty, name) in RUN_TIME_VARIABLES {
            // defmt::println!("### MB Send request for {}[{:04x}] no: {}", name, reg, ty.register_no());
            if let Some(req) = create_request(&mut buf, 1, reg, ty.register_no()) {
                let size: usize = usize::from(u8::from(req));
                if let Err(e) = uart.write(&buf[0..size]).await {
                    defmt::println!("MB ERR: {:?}", e);
                } else {
                    // defmt::println!("MB Wait for request! {}", name);
                    let _ = uart.blocking_flush();
                    mb.reset();
                    let mut time = 500;
                    loop {
                        match select(uart.read(&mut buf[0..1]), Timer::after(Duration::from_millis(time))).await {
                            Either::First(ret) => {
                                time = 10;

                                match ret {
                                    Ok(()) => {
                                        let byte = buf[0];
                                        mb.serial_recv(byte);
                                    }
                                    Err(e) => {
                                        let ser_err = match e {
                                            embassy_rp::uart::Error::Overrun => ModbusError::OverRun,
                                            embassy_rp::uart::Error::Parity => ModbusError::Parity,
                                            _ => ModbusError::Frame,
                                        };
                                        mb.serial_error(ser_err);
                                        println!("MB UART E {:?}", e);
                                        break;
                                    }
                                }
                            }
                            Either::Second(()) => {
                                // println!("MB: idx {} state: {}", mb.idx(), mb.state);
                                if mb.is_state_idle() {
                                    println!("MB E: Timeout");
                                    break;
                                }
                                match mb.timer_handle() {
                                    None => (),
                                    Some(Err(e)) => {
                                        error!("MB TE {:?}", e);
                                        break;
                                    }
                                    Some(Ok(v)) => {
                                        let data = &mb.buf[0..usize::from(u8::from(v))];
                                        let val = ty.parse(data);
                                        match val {
                                            Ok(v) => {
                                                println!("MB DATA: {=str}[{}]: {} {:02x}", name, reg, v, data,)
                                            }
                                            Err(e) => println!(
                                                "MB DATA: {=str}[{}]: {} NO {} {:02x}",
                                                name,
                                                reg,
                                                e,
                                                ty.register_no(),
                                                data,
                                            ),
                                        }

                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        // defmt::info!("MB: Done");
        tick.next().await;
    }
}

fn u32_to_double<const N: usize>(mut num: u32, st: &mut String<N>) {
    let mut s = [0; 2];

    for p in s.iter_mut().rev() {
        *p = (num % 10) as u8;
        num /= 10;
    }

    for num in s {
        let _ = st.push(char::from_digit(u32::from(num), 10).unwrap());
    }
}

fn i16_to_str<const N: usize>(num: i16, st: &mut String<N>) {
    let mut s = [0; 3];

    let sign = num.is_positive();

    let mut num = num.abs() as u16;

    num /= 10;

    for p in s.iter_mut().rev() {
        *p = (num % 10) as u8;
        num /= 10;
    }

    st.push(if sign { '+' } else { '-' });

    for (idx, num) in s.into_iter().enumerate() {
        if idx == 2 {
            st.push('.');
        }
        let _ = st.push(char::from_digit(u32::from(num), 10).unwrap());
    }
}

type SpiBusT = Mutex<NoopRawMutex, Spi<'static, SPI1, Async>>;

type Dsp = ST7735<
    SpiDeviceWithConfig<'static, NoopRawMutex, Spi<'static, SPI1, Async>, Output<'static>>,
    Output<'static>,
    Output<'static>,
    160,
    80,
    { buffer_size(160, 80) },
>;
static DSP: StaticCell<Dsp> = StaticCell::new();
static SPI_BUS: StaticCell<SpiBusT> = StaticCell::new();
static GPS_CHANNEL: StaticCell<Channel<NoopRawMutex, GpsType, 1>> = StaticCell::new();
static LORA_CHANNEL: StaticCell<Channel<NoopRawMutex, LoraType, 1>> = StaticCell::new();
static LORASTATUS_CHANNEL: StaticCell<Channel<NoopRawMutex, LoraStatus, 1>> = StaticCell::new();

struct TempData {
    t: sht3x::Tmp,
    h: sht3x::Hum,
}

type TempSignal = Signal<CriticalSectionRawMutex, TempData>;

static TEMPDATA: TempSignal = TempSignal::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Hello World!");

    // GPS
    let gps_pps = Input::new(p.PIN_16, Pull::Down);
    // Standby Switch
    let _gps_sw_sb = Input::new(p.PIN_17, Pull::Down);
    let gps_tx = p.PIN_0;
    let gps_rx = p.PIN_1;

    let mut config = UConfig::default();
    config.baudrate = 9600;

    let gps_uart = Uart::new(p.UART0, gps_tx, gps_rx, Irqs, p.DMA_CH2, p.DMA_CH3, config);

    // Modbus
    let mb_tx = p.PIN_4;
    let mb_rx = p.PIN_5;
    let mut config = UConfig::default();
    config.baudrate = 9600;
    config.stop_bits = StopBits::STOP2;
    config.parity = Parity::ParityNone;

    let mb_uart = Uart::new(p.UART1, mb_tx, mb_rx, Irqs, p.DMA_CH4, p.DMA_CH5, config);

    // Onboard LCD
    let bl = p.PIN_25;
    let rst = p.PIN_13;
    let display_cs = p.PIN_9;
    let dcx = p.PIN_8;
    let miso = p.PIN_12;
    let mosi = p.PIN_11;
    let clk = p.PIN_10;

    let mut lora_config = spi::Config::default();
    lora_config.frequency = 12_000_000;
    lora_config.phase = spi::Phase::CaptureOnSecondTransition;
    lora_config.polarity = spi::Polarity::IdleHigh;

    let spi = Spi::new(p.SPI1, clk, mosi, miso, p.DMA_CH0, p.DMA_CH1, Config::default());

    // Commit these two lines below to make it compile right with lora.
    let spi_bus = Mutex::<NoopRawMutex, _>::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);

    let dcx = Output::new(dcx, Level::Low);
    let rst = Output::new(rst, Level::Low);
    // dcx: 0 = command, 1 = data

    // Enable LCD backlight
    let mut bl = Output::new(bl, Level::Low);

    // I2C for Tempsensor
    let sda = p.PIN_18;
    let scl = p.PIN_19;
    let config = embassy_rp::i2c::Config::default();
    let mut i2c1 = I2c::new_async(p.I2C1, scl, sda, Irqs, config);

    let gps_channel = GPS_CHANNEL.init(Channel::new());
    let gps_send = gps_channel.sender();
    let gps_recv = gps_channel.receiver();

    info!("Hello2!");
    spawner.must_spawn(tempdata_handle(i2c1));

    spawner.must_spawn(gps_handle(gps_uart, gps_send));
    info!("Hello3!");

    // spawner.must_spawn(modbus_handle(mb_uart));
    let display_cs = Output::new(display_cs, Level::High);

    // create SPI
    let mut display_config = spi::Config::default();
    display_config.frequency = DISPLAY_FREQ;
    display_config.phase = spi::Phase::CaptureOnSecondTransition;
    display_config.polarity = spi::Polarity::IdleHigh;
    let display_spi = SpiDeviceWithConfig::new(spi_bus, display_cs, display_config.clone());

    let lora_channel = LORA_CHANNEL.init(Channel::new());
    let lora_send: Sender<NoopRawMutex, LoraType, 1> = lora_channel.sender();
    let lora_recv = lora_channel.receiver();

    let lora_channel = LORASTATUS_CHANNEL.init(Channel::new());
    let lorast_send = lora_channel.sender();
    let lorast_recv = lora_channel.receiver();

    let dsp_config = st7735_embassy::Config {
        // Color is 'Bgr'
        rgb: st7735_embassy::PixelColor::BGR,
        // Color are Inverted
        inverted: true,
        // Orientation Landscape
        orientation: Orientation::Landscape,
    };

    let display = DSP.init(Dsp::new(display_spi, dcx, rst, dsp_config));

    // Waveshare RP2040 has a offset of 1, 26
    display.set_offset(1, 26);

    display.init(&mut Delay).await.unwrap();

    let area = Rectangle {
        top_left: Point::zero(),
        size: Size::new(FB_SIZE.0 as u32, FB_SIZE.1 as u32),
    };
    display.fill_solid(&area, Rgb565::GREEN).unwrap();
    display.flush().await.unwrap();

    bl.set_high();

    info!("Hello4!");
    spawner.must_spawn(display_task(display, gps_recv, gps_pps, bl, lorast_recv));

    let nss = Output::new(p.PIN_3.degrade(), Level::High);
    let reset = Output::new(p.PIN_15.degrade(), Level::High);
    let dio1 = Input::new(p.PIN_20.degrade(), Pull::None);
    let busy = Input::new(p.PIN_2.degrade(), Pull::None);

    // let lora_spi: SpiDeviceWithConfig::new( <'_, NoopRawMutex, Spi<'_, SPI1, Async>> =
    //     SpiBusWithConfig::new(spi_bus, lora_config);

    let lora_spi: SpiDeviceWithConfig<'_, NoopRawMutex, Spi<'_, SPI1, Async>, Output<'_>> =
        SpiDeviceWithConfig::new(spi_bus, nss, lora_config.clone());

    info!("Hello5! dsp = {}", core::mem::size_of::<Dsp>());
    spawner.must_spawn(lora_task(lora_spi, lora_recv, lorast_send, reset, dio1, busy));

    let mut counter: u32 = 0xDEADBEAF;

    Timer::after(Duration::from_secs(100)).await;

    loop {
        println!("Queue up lora data {:08x}", counter);
        let mut ld = LoraType::new();
        let _ = ld.extend_from_slice(counter.to_be_bytes().as_slice());
        lora_send.send(ld).await;
        counter = counter.wrapping_add(1);
    }
}
