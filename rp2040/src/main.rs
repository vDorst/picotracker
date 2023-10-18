//! This example shows how to use SPI (Serial Peripheral Interface) in the RP2040 chip.
//!
//! Example written for a display using the ST7789 chip. Possibly the Waveshare Pico-ResTouch
//! (https://www.waveshare.com/wiki/Pico-ResTouch-LCD-2.8)

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_imports)]
#![feature(try_blocks)]

use core::cell::RefCell;
use core::f32::consts::PI;
use core::fmt::write;
use core::num::NonZeroI8;

use defmt::*;
use display_interface::prelude::*;
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_embedded_hal::shared_bus::SpiDeviceError;
use embassy_embedded_hal::SetConfig;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_lora::iv::GenericSx126xInterfaceVariant;
use embassy_lora::LoraTimer;
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use embassy_rp::peripherals::SPI1;
use embassy_rp::spi::{self, Async, Blocking, Config, Spi};
use embassy_rp::uart::{Async as UartAsync, Config as UConfig, InterruptHandler, Parity, StopBits, Uart};
use embassy_rp::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Ticker, Timer};
use embedded_graphics::framebuffer::{buffer_size, Framebuffer};
use embedded_graphics::image::{GetPixel, Image, ImageRawLE};
use embedded_graphics::mono_font::{ascii, MonoTextStyle};
use embedded_graphics::pixelcolor::raw::LittleEndian;
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
use lora_phy::mod_params::*;
use lora_phy::sx1261_2::SX1261_2;
use lora_phy::LoRa;
use lorawan::default_crypto::DefaultFactory as Crypto;
use lorawan_device::async_device::lora_radio::LoRaRadio;
use lorawan_device::async_device::{Device, JoinMode};
use lorawan_device::{region, AppEui, AppKey, DevEui};
use micromath::F32Ext;
use mipidsi::asynch::models::Model;
use mipidsi::models::ST7789;
use mipidsi::{Builder, Display, ModelOptions, Orientation};
use modbus_master::{create_request, Modbus, ModbusAddr, ModbusError};
use static_cell::{make_static, StaticCell};
use {defmt_rtt as _, panic_probe as _};

const LORAWAN_REGION: region::Region = region::Region::EU868; // warning: set this appropriately for the region

const LORAWAN_DEVEUI: [u8; 8] = envconst::reverse(envconst::hex_to_u8::<8>(env!("DEVEUI")));
const LORAWAN_APPEUI: [u8; 8] = envconst::reverse(envconst::hex_to_u8::<8>(env!("APPEUI")));
const LORAWAN_APPKEY: [u8; 16] = envconst::hex_to_u8::<16>(env!("APPKEY"));

const DISPLAY_FREQ: u32 = 64_000_000;
const FB_SIZE: (usize, usize) = (160, 80);

bind_interrupts!(struct Irqs {
    UART0_IRQ => InterruptHandler<peripherals::UART0>;
    UART1_IRQ => InterruptHandler<peripherals::UART1>;
});

type GpsType = GpsRmc;

type GpsSend = Sender<'static, NoopRawMutex, GpsType, 1>;

type LoraType = Vec<u8, 48>;
type LoraStatus = String<16>;

#[embassy_executor::task]
async fn lora_task(
    lora_spi: SpiDeviceWithConfig<'static, NoopRawMutex, Spi<'static, SPI1, Async>, Output<'static, AnyPin>>,
    lora_data_recv: Receiver<'static, NoopRawMutex, LoraType, 1>,
    lora_status_send: Sender<'static, NoopRawMutex, LoraStatus, 1>,
    reset: Output<'static, AnyPin>,
    dio1: Input<'static, AnyPin>,
    busy: Input<'static, AnyPin>,
) {
    let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();

    let lora = {
        match LoRa::new(
            SX1261_2::new(BoardType::RpPicoWaveshareSx1262, lora_spi, iv),
            true,
            Delay,
        )
        .await
        {
            Ok(l) => l,
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }
    };

    let radio = LoRaRadio::new(lora);
    let region: region::Configuration = region::Configuration::new(LORAWAN_REGION);

    let mut device: Device<_, Crypto, _, _> = Device::new(region, radio, LoraTimer::new(), embassy_rp::clocks::RoscRng);

    device.set_datarate(region::DR::_4);

    loop {
        defmt::info!("Joining LoRaWAN network");
        let msg: LoraStatus = "Lora: Try join.".into();

        lora_status_send.send(msg).await;

        match device
            .join(&JoinMode::OTAA {
                deveui: DevEui::from(LORAWAN_DEVEUI),
                appeui: AppEui::from(LORAWAN_APPEUI),
                appkey: AppKey::from(LORAWAN_APPKEY),
            })
            .await
        {
            Ok(()) => {
                defmt::info!("LoRaWAN network joined");
                let msg: LoraStatus = "Lora: joined".into();
                lora_status_send.send(msg).await;
                break;
            }
            Err(err) => {
                info!("Radio error = {}", err);
                let msg: LoraStatus = "Lora: join Err".into();
                lora_status_send.send(msg).await;
            }
        };

        Timer::after(Duration::from_secs(10)).await;
    }

    loop {
        println!("Wait for data!");
        let data = lora_data_recv.receive().await;
        let ret = device.send(&data, 1, false).await;

        let msg = match ret {
            Ok(()) => {
                println!("Send Done");
                "Lora: Sended".into()
            }
            Err(e) => {
                println!("Error: {:?}", e);
                "Lora: Send Err".into()
            }
        };

        lora_status_send.send(msg).await;

        println!("sleep 5 min");
        Timer::after(Duration::from_secs(5 * 60)).await;
    }
}

#[embassy_executor::task]
async fn display_task(
    display_spi: SpiDeviceWithConfig<
        'static,
        NoopRawMutex,
        Spi<'static, SPI1, Async>,
        Output<'static, peripherals::PIN_9>,
    >,
    gps_recv: Receiver<'static, NoopRawMutex, GpsType, 1>,
    gps_pps: Input<'static, peripherals::PIN_16>,
    display_dcx: Output<'static, peripherals::PIN_8>,
    display_rst: Output<'static, peripherals::PIN_13>,
    mut bl: Output<'static, peripherals::PIN_25>,
    lora_recv: Receiver<'static, NoopRawMutex, LoraStatus, 1>,
) {
    // display interface abstraction from SPI and DC
    let di = SPIInterface::new(display_spi, display_dcx);

    let mut delay = Delay;

    // create driver, display.set_offset(1, 26);
    let mut display = Builder::st7789(di)
        .with_display_size(FB_SIZE.0 as u16, FB_SIZE.1 as u16)
        .with_window_offset_handler(|_e| (1, 26))
        .with_color_order(mipidsi::ColorOrder::Bgr)
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .with_orientation(Orientation::LandscapeInverted(true))
        .async_init(&mut delay, Some(display_rst))
        .await
        .unwrap();

    let mut fb = Framebuffer::<
        Rgb565,
        _,
        LittleEndian,
        { FB_SIZE.0 },
        { FB_SIZE.1 },
        { buffer_size::<Rgb565>(FB_SIZE.0, FB_SIZE.1) },
    >::new();

    let data = fb.data_mut();
    let pix_iter = unsafe { core::slice::from_raw_parts_mut(data.as_mut_ptr() as *mut u16, data.len() / 2) };
    let _ = display
        .async_set_pixels_raw(0, 0, FB_SIZE.0 as u16, FB_SIZE.1 as u16, pix_iter)
        .await;

    let mut step: u8 = 0;
    let rad = 20.0;

    let mut g: GpsType;
    let mut sp: String<25> = "No GPS update".into();

    let mut lora_s: LoraStatus = "No LORA update".into();

    let area = Rectangle {
        top_left: Point::new(0, 0),
        size: Size::new(160, 80),
    };
    let _ = fb.fill_solid(&area, Rgb565::BLACK);

    let (ex, ey) = {
        let s = fb.size();
        let x = s.width as u16 - 1;
        let y = s.height as u16 - 1;
        (x, y)
    };

    let data = fb.data_mut();
    let pix_iter = unsafe { core::slice::from_raw_parts_mut(data.as_mut_ptr() as *mut u16, data.len() / 2) };
    let _ = display.async_set_pixels_raw(0, 0, ex, ey, pix_iter).await;
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
    .draw(&mut fb)
    .unwrap();

    // loop
    loop {
        if step >= 60 {
            step = 0;
            col = col_iter.next().unwrap();
        }

        if let Ok(p) = lora_recv.try_receive() {
            lora_s = p;
        }

        if let Ok(p) = gps_recv.try_receive() {
            g = p;

            if matches!(g.pos_mode, PosMode::NoFix) || g.data.is_none() {
                sp = "GPS: No Fix".into();
            } else {
                sp = "".into();
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

        let _ = fb.fill_solid(
            &Rectangle {
                top_left: Point { x: 0, y: 0 },
                size: Size::new(159, style.font.character_size.height),
            },
            Rgb565::BLACK,
        );
        Text::new(&sp, Point::new(0, (style.font.baseline) as i32), style)
            .draw(&mut fb)
            .unwrap();

        let lora_txt = Text::new(
            &lora_s,
            Point::new(0, (style.font.baseline + (5 * style.font.character_size.height)) as i32),
            style,
        );

        let clear_rec = lora_txt
            .bounding_box()
            .resized_width(160, embedded_graphics::geometry::AnchorX::Right);

        let _ = fb.fill_solid(&clear_rec, Rgb565::BLACK);

        lora_txt.draw(&mut fb).unwrap();

        ret = f32::from(step) * PI * 2.0 / 60.0;

        let (sx, sy) = (ret.sin() * rad, ret.cos() * rad);

        let end = Point::new((sx.trunc()) as i32 + center.x, center.y - (sy.trunc()) as i32);

        // defmt::println!("x = {}, y = {}, {} {}", end.x, end.y, sx, sy);

        let line = Line::new(center, end).into_styled(PrimitiveStyle::with_stroke(col, 3));
        unwrap!(line.draw(&mut fb));

        let gps_col = if gps_pps.is_high() {
            Rgb565::GREEN
        } else {
            Rgb565::BLACK
        };

        fb.set_pixel(Point::new(159, 0), gps_col);

        let mut pix_iter = fb
            .data()
            .chunks_exact(2)
            .map(|x| u16::from_le_bytes(x.try_into().unwrap()))
            .collect::<Vec<u16, { 160 * 80 }>>();

        frame.next().await;
        let _ = display.async_set_pixels_raw(0, 0, ex, ey, &mut pix_iter).await;

        let line = Line::new(center, end).into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK, 3));
        unwrap!(line.draw(&mut fb));

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

type SpiBusT = Mutex<NoopRawMutex, Spi<'static, SPI1, Async>>;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    static SPI_BUS: StaticCell<SpiBusT> = StaticCell::new();

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
    let bl = Output::new(bl, Level::Low);

    let gps_channel = make_static!(Channel::<NoopRawMutex, GpsType, 1>::new());
    let gps_send = gps_channel.sender();
    let gps_recv = gps_channel.receiver();

    spawner.must_spawn(gps_handle(gps_uart, gps_send));

    spawner.must_spawn(modbus_handle(mb_uart));

    let display_cs = Output::new(display_cs, Level::High);

    // create SPI
    let mut display_config = spi::Config::default();
    display_config.frequency = DISPLAY_FREQ;
    display_config.phase = spi::Phase::CaptureOnSecondTransition;
    display_config.polarity = spi::Polarity::IdleHigh;
    let display_spi = SpiDeviceWithConfig::new(spi_bus, display_cs, display_config.clone());

    let lora_channel = make_static!(Channel::<NoopRawMutex, LoraType, 1>::new());
    let lora_send: Sender<NoopRawMutex, LoraType, 1> = lora_channel.sender();
    let lora_recv = lora_channel.receiver();

    let lora_channel = make_static!(Channel::<NoopRawMutex, LoraStatus, 1>::new());
    let lorast_send = lora_channel.sender();
    let lorast_recv = lora_channel.receiver();

    spawner.must_spawn(display_task(display_spi, gps_recv, gps_pps, dcx, rst, bl, lorast_recv));

    let nss = Output::new(p.PIN_3.degrade(), Level::High);
    let reset = Output::new(p.PIN_15.degrade(), Level::High);
    let dio1 = Input::new(p.PIN_20.degrade(), Pull::None);
    let busy = Input::new(p.PIN_2.degrade(), Pull::None);

    // let lora_spi: SpiDeviceWithConfig::new( <'_, NoopRawMutex, Spi<'_, SPI1, Async>> =
    //     SpiBusWithConfig::new(spi_bus, lora_config);

    let lora_spi: SpiDeviceWithConfig<'_, NoopRawMutex, Spi<'_, SPI1, Async>, Output<'_, AnyPin>> =
        SpiDeviceWithConfig::new(spi_bus, nss, lora_config.clone());

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

use core::convert::TryInto;

use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyle};
