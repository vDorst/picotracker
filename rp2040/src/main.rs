//! This example shows how to use SPI (Serial Peripheral Interface) in the RP2040 chip.
//!
//! Example written for a display using the ST7789 chip. Possibly the Waveshare Pico-ResTouch
//! (https://www.waveshare.com/wiki/Pico-ResTouch-LCD-2.8)

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_imports)]
#![feature(try_blocks)]
#![feature(async_fn_in_trait)]
#![feature(impl_trait_projections)]

use core::cell::RefCell;
use core::f32::consts::PI;

use defmt::*;
use display_interface::prelude::*;
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_embedded_hal::shared_bus::SpiDeviceError;
use embassy_embedded_hal::SetConfig;
use embassy_executor::Spawner;
use embassy_lora::iv::GenericSx126xInterfaceVariant;
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::peripherals::SPI1;
use embassy_rp::spi::{self, Async, Blocking, Config, Spi};
use embassy_rp::uart::{Async as UartAsync, Config as UConfig, InterruptHandler, Uart};
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
use heapless::{String, Vec};
use lora_phy::mod_params::*;
use lora_phy::sx1261_2::SX1261_2;
use lora_phy::LoRa;
use micromath::F32Ext;
use mipidsi::asynch::models::Model;
use mipidsi::models::ST7789;
use mipidsi::{Builder, Display, ModelOptions, Orientation};
use static_cell::{make_static, StaticCell};
use {defmt_rtt as _, panic_probe as _};

const LORA_FREQUENCY_IN_HZ: u32 = 868_500_000; // warning: set this appropriately for the region

const DISPLAY_FREQ: u32 = 64_000_000;

bind_interrupts!(struct Irqs {
    UART0_IRQ => InterruptHandler<peripherals::UART0>;
});

enum UartState {
    WaitForStart,
    WaitForDataType,
    GetData,
}

type GPSType = String<80>;

fn slice_to_hex(data: &[u8]) -> u8 {
    let mut ret: u8 = 0;
    for &byte in data[0..data.len().min(2)].iter() {
        // println!("{} {:02x}", byte, byte);
        ret = ret.wrapping_shl(4);
        ret |= match byte {
            b'0'..=b'9' => byte - b'0',
            b'A'..=b'F' => byte + 10 - b'A',
            _ => 0,
        }
    }
    // println!("{:02x}", ret);
    ret
}

type GpsSend = Sender<'static, NoopRawMutex, GPSType, 1>;
type GpsRevc = Receiver<'static, NoopRawMutex, GPSType, 1>;

struct GPS {
    pub buf: [u8; 100],
    pub idx: usize,
    pub state: UartState,
}

impl GPS {
    fn new() -> Self {
        Self {
            buf: [0; 100],
            idx: 0,
            state: UartState::WaitForStart,
        }
    }

    pub fn reset_state(&mut self) {
        self.idx = 0;
        self.state = UartState::WaitForStart;
    }

    pub fn as_mut_data(&mut self) -> &mut [u8] {
        let idx = self.idx;
        &mut self.buf[idx..idx + 1]
    }

    pub fn as_data(&self) -> &[u8] {
        let idx = self.idx;
        &self.buf[0..idx]
    }

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
        let char = self.buf[self.idx];
        match self.state {
            UartState::WaitForStart => {
                if char == b'$' {
                    self.idx += 1;
                    self.state = UartState::WaitForDataType;
                }
            }
            UartState::WaitForDataType => {
                self.idx += 1;
                if char == b',' {
                    let line = self.as_data();
                    // println!("GPSWAIT: {} \"{}\"", line.len(), core::str::from_utf8(line).unwrap());
                    if line == b"$GNGGA," {
                        self.state = UartState::GetData;
                    } else {
                        self.reset_state();
                    }
                }
            }
            UartState::GetData => {
                if char == b'\n' {
                    let valid = self.is_valid_checksum();
                    let mut line = self.as_data();
                    if line.last() == Some(&b'\n') {
                        line = &line[0..line.len() - 1];
                    }
                    line = &line[0..line.len() - 4];
                    if valid {
                        if let Ok(s) = core::str::from_utf8(line) {
                            let mut g = GPSType::new();
                            let _ = g.push_str(s);
                            self.reset_state();
                            return Some(g);
                        }
                    }
                    self.reset_state();
                } else {
                    self.idx += 1;
                    if self.idx >= self.buf.len() {
                        self.reset_state();
                    }
                }
            }
        }

        None
    }
}

#[embassy_executor::task]
async fn gps_handle(mut uart: Uart<'static, peripherals::UART0, UartAsync>, send: GpsSend) {
    let mut gps = GPS::new();

    loop {
        match uart.read(gps.as_mut_data()).await {
            Ok(()) => {
                if let Some(msg) = gps.process() {
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, Spi<'static, SPI1, Async>>> = StaticCell::new();

    let p = embassy_rp::init(Default::default());
    info!("Hello World!");

    // GPS
    let gps_pps = Input::new(p.PIN_16, Pull::None);
    // Standby Switch
    let gps_sw_sb = Input::new(p.PIN_17, Pull::None);
    let gps_tx = p.PIN_0;
    let gps_rx = p.PIN_1;

    let mut config = UConfig::default();
    config.baudrate = 9600;

    let gps_uart = Uart::new(p.UART0, gps_tx, gps_rx, Irqs, p.DMA_CH2, p.DMA_CH3, config);

    // Onboard LCD
    let bl = p.PIN_25;
    let rst = p.PIN_13;
    let display_cs = p.PIN_9;
    let dcx = p.PIN_8;
    let miso = p.PIN_12;
    let mosi = p.PIN_11;
    let clk = p.PIN_10;

    // create SPI
    let mut display_config = spi::Config::default();
    display_config.frequency = DISPLAY_FREQ;
    display_config.phase = spi::Phase::CaptureOnSecondTransition;
    display_config.polarity = spi::Polarity::IdleHigh;

    let spi = Spi::new(p.SPI1, clk, mosi, miso, p.DMA_CH0, p.DMA_CH1, Config::default());

    // Commit these two lines below to make it compile right with lora.
    let spi_bus = Mutex::<NoopRawMutex, _>::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);

    let display_spi = SpiDeviceWithConfig::new(&spi_bus, Output::new(display_cs, Level::High), display_config.clone());

    let lora_spi = SpiBusWithConfig::new(&spi_bus, display_config);

    let dcx = Output::new(dcx, Level::Low);
    let rst = Output::new(rst, Level::Low);
    // dcx: 0 = command, 1 = data

    // Enable LCD backlight
    let mut bl = Output::new(bl, Level::Low);

    // display interface abstraction from SPI and DC
    let di = SPIInterface::new(display_spi, dcx);

    let mut delay = Delay;

    let fb_size = (160, 80);

    // create driver, display.set_offset(1, 26);
    let mut display = Builder::st7789(di)
        .with_display_size(fb_size.0, fb_size.1)
        .with_window_offset_handler(|_e| (1, 26))
        .with_color_order(mipidsi::ColorOrder::Bgr)
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .with_orientation(Orientation::LandscapeInverted(true))
        .async_init(&mut delay, Some(rst))
        .await
        .unwrap();

    let mut fb = Framebuffer::<Rgb565, _, LittleEndian, 160, 80, { buffer_size::<Rgb565>(160, 80) }>::new();

    let area = Rectangle {
        top_left: Point::new(0, 0),
        size: Size::new(160, 80),
    };
    let _ = fb.fill_solid(&area, Rgb565::RED);

    // Draw a circle with top-left at `(22, 22)` with a diameter of `20` and a white stroke
    let circle = Circle::new(Point::new(80, 40), 20).into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN));
    unwrap!(circle.draw(&mut fb));

    let line =
        Line::new(Point::new(0, 0), Point::new(159, 79)).into_styled(PrimitiveStyle::with_stroke(Rgb565::BLUE, 1));
    unwrap!(line.draw(&mut fb));

    let (ex, ey) = {
        let s = fb.size();
        let x = s.width as u16 - 1;
        let y = s.height as u16 - 1;
        (x, y)
    };

    // let mut pix_iter = fb
    //     .data()
    //     .chunks_exact(2)
    //     .map(|x| u16::from_le_bytes(x.try_into().unwrap()))
    //     .collect::<Vec<u16, { 160 * 80 }>>();
    //

    let data = fb.data_mut();
    let mut pix_iter = unsafe { core::slice::from_raw_parts_mut(data.as_mut_ptr() as *mut u16, data.len() / 2) };
    let _ = display.async_set_pixels_raw(0, 0, ex, ey, &mut pix_iter).await;
    bl.set_high();

    let center = Point::new(80 - 1, 40 - 1);
    let mut frame = Ticker::every(Duration::from_hz(60));
    let mut ret: f32;

    let mut col_iter = [Rgb565::RED, Rgb565::GREEN, Rgb565::BLUE].into_iter().cycle();
    let mut col = col_iter.next().unwrap();
    let _ = fb.fill_solid(&area, Rgb565::BLACK);

    let style = MonoTextStyle::new(&ascii::FONT_6X12, Rgb565::WHITE);
    Text::new(
        "Rene\nRust",
        Point::new(0, (style.font.baseline + style.font.character_size.height) as i32),
        style,
    )
    .draw(&mut fb)
    .unwrap();

    let mut step: u8 = 0;

    let rad = 20.0;

    let channel = make_static!(Channel::<NoopRawMutex, GPSType, 1>::new());
    let send = channel.sender();

    let mut g: GPSType;
    let mut sp = "No GPS update";

    spawner.must_spawn(gps_handle(gps_uart, send));
    let recv = channel.receiver();

    loop {
        if step >= 60 {
            step = 0;
            col = col_iter.next().unwrap();
        }

        if gps_sw_sb.is_high() {
            sp = &"GPS: Standby";
        }

        if let Ok(p) = recv.try_receive() {
            g = p;
            if g.len() > 13 {
                sp = &g[7..7 + 6];
            } else {
                sp = &g;
            }
        }

        let _ = fb.fill_solid(
            &Rectangle {
                top_left: Point { x: 0, y: 0 },
                size: Size::new(159, style.font.character_size.height),
            },
            Rgb565::BLACK,
        );
        Text::new(sp, Point::new(0, (style.font.baseline) as i32), style)
            .draw(&mut fb)
            .unwrap();

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

    // let nss = Output::new(p.PIN_3.degrade(), Level::High);
    // let reset = Output::new(p.PIN_15.degrade(), Level::High);
    // let dio1 = Input::new(p.PIN_20.degrade(), Pull::None);
    // let busy = Input::new(p.PIN_2.degrade(), Pull::None);

    // let iv = GenericSx126xInterfaceVariant::new(nss, reset, dio1, busy, None, None).unwrap();

    // let mut lora = {
    //     match LoRa::new(
    //         SX1261_2::new(BoardType::RpPicoWaveshareSx1262, lora_spi, iv),
    //         false,
    //         &mut delay,
    //     )
    //     .await
    //     {
    //         Ok(l) => l,
    //         Err(err) => {
    //             info!("Radio error = {}", err);
    //             return;
    //         }
    //     }
    // };

    // let mut debug_indicator = Output::new(p.PIN_24, Level::Low);

    // let mut receiving_buffer = [00u8; 100];

    // let mdltn_params = {
    //     match lora.create_modulation_params(
    //         SpreadingFactor::_10,
    //         Bandwidth::_250KHz,
    //         CodingRate::_4_8,
    //         LORA_FREQUENCY_IN_HZ,
    //     ) {
    //         Ok(mp) => mp,
    //         Err(err) => {
    //             info!("Radio error = {}", err);
    //             return;
    //         }
    //     }
    // };

    // let rx_pkt_params = {
    //     match lora.create_rx_packet_params(4, false, receiving_buffer.len() as u8, true, false, &mdltn_params) {
    //         Ok(pp) => pp,
    //         Err(err) => {
    //             info!("Radio error = {}", err);
    //             return;
    //         }
    //     }
    // };

    // match lora
    //     .prepare_for_rx(&mdltn_params, &rx_pkt_params, None, true, false, 0, 0x00ffffffu32)
    //     .await
    // {
    //     Ok(()) => {}
    //     Err(err) => {
    //         info!("Radio error = {}", err);
    //         return;
    //     }
    // };

    // loop {
    //     receiving_buffer = [00u8; 100];
    //     match lora.rx(&rx_pkt_params, &mut receiving_buffer).await {
    //         Ok((received_len, _rx_pkt_status)) => {
    //             if (received_len == 3)
    //                 && (receiving_buffer[0] == 0x01u8)
    //                 && (receiving_buffer[1] == 0x02u8)
    //                 && (receiving_buffer[2] == 0x03u8)
    //             {
    //                 info!("rx successful");
    //                 debug_indicator.set_high();
    //                 Timer::after(Duration::from_secs(5)).await;
    //                 debug_indicator.set_low();
    //             } else {
    //                 info!("rx unknown packet");
    //             }
    //         }
    //         Err(err) => info!("rx unsuccessful = {}", err),
    //     }
    // }
}

pub struct SpiBusWithConfig<'a, M: RawMutex, BUS: SetConfig> {
    bus: &'a Mutex<M, BUS>,
    config: BUS::Config,
}

impl<'a, M: RawMutex, BUS: SetConfig> SpiBusWithConfig<'a, M, BUS> {
    /// Create a new `SpiDeviceWithConfig`.
    pub fn new(bus: &'a Mutex<M, BUS>, config: BUS::Config) -> Self {
        Self { bus, config }
    }
}

impl<'a, M, BUS> embedded_hal_async::spi::ErrorType for SpiBusWithConfig<'a, M, BUS>
where
    BUS: embedded_hal_async::spi::ErrorType + SetConfig,
    M: RawMutex,
{
    type Error = BUS::Error;
}

impl<M, BUS> embedded_hal_async::spi::SpiBus for SpiBusWithConfig<'_, M, BUS>
where
    M: RawMutex,
    BUS: embedded_hal_async::spi::SpiBus + SetConfig,
{
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut bus = self.bus.lock().await;
        bus.set_config(&self.config);

        bus.read(words).await
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        let mut bus = self.bus.lock().await;
        bus.set_config(&self.config);

        bus.write(words).await
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let mut bus = self.bus.lock().await;
        bus.set_config(&self.config);

        bus.transfer(read, write).await
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let mut bus = self.bus.lock().await;
        bus.set_config(&self.config);

        bus.transfer_in_place(words).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        let mut bus = self.bus.lock().await;

        bus.flush().await
    }
}

use core::convert::TryInto;

use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyle};

/// SPI communication error
#[derive(Debug)]
struct CommError;
