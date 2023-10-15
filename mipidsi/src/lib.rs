#![no_std]
// associated re-typing not supported in rust yet
#![allow(clippy::type_complexity)]
#![warn(missing_docs)]
#![cfg_attr(
    all(feature = "async"),
    allow(incomplete_features),
    feature(async_fn_in_trait)
)]

//! This crate provides a generic display driver to connect to TFT displays
//! that implement the [MIPI Display Command Set](https://www.mipi.org/specifications/display-command-set).
//!
//! Uses [display_interface](https://crates.io/crates/display-interface) to talk to the hardware via transports.
//!
//! An optional batching of draws is supported via the `batch` feature (default on)
//!
//! ### List of supported models
//!
//! * ST7789
//! * ST7735
//! * ILI9486
//! * ILI9341
//! * ILI9342C
//!
//! ## Example
//! ```rust ignore
//! // create a DisplayInterface from SPI and DC pin, with no manual CS control
//! let di = SPIInterfaceNoCS::new(spi, dc);
//! // create the ILI9486 display driver from the display interface and optional RST pin
//! let mut display = Builder::ili9486(di)
//!     .init(&mut delay, Some(rst));
//! // clear the display to black
//! display.clear(Rgb666::BLACK).unwrap();
//! ```
//!
//! ## Troubleshooting
//! See [document](https://github.com/almindor/mipidsi/blob/master/docs/TROUBLESHOOTING.md)
//!

use dcs::Dcs;
use display_interface::WriteOnlyDataCommand;

pub mod error;
use embedded_hal::digital::OutputPin;
pub use error::Error;

pub mod options;
pub use options::*;

mod builder;
pub use builder::Builder;

pub mod dcs;

pub mod models;
use models::Model;

#[cfg(feature = "async")]
pub mod asynch;

mod graphics;

#[cfg(feature = "batch")]
mod batch;

#[cfg(feature = "framebuffer")]
pub mod framebuffer;

///
/// Display driver to connect to TFT displays.
///
pub struct Display<DI, MODEL, RST> {
    // DCS provider
    dcs: Dcs<DI>,
    // Model
    model: MODEL,
    // Reset pin
    rst: Option<RST>,
    // Model Options, includes current orientation
    options: ModelOptions,
    // Current MADCTL value copy for runtime updates
    madctl: dcs::SetAddressMode,
}

impl<DI, M, RST> Display<DI, M, RST> {
    ///
    /// Returns currently set [Orientation]
    ///
    pub fn orientation(&self) -> Orientation {
        self.options.orientation()
    }

    ///
    /// Release resources allocated to this driver back.
    /// This returns the display interface, reset pin and and the model deconstructing the driver.
    ///
    pub fn release(self) -> (DI, M, Option<RST>) {
        (self.dcs.release(), self.model, self.rst)
    }
}

impl<DI, M, RST> Display<DI, M, RST>
where
    DI: WriteOnlyDataCommand,
    M: Model,
    RST: OutputPin,
{
    ///
    /// Sets display [Orientation]
    ///
    pub fn set_orientation(&mut self, orientation: Orientation) -> Result<(), Error> {
        self.madctl = self.madctl.with_orientation(orientation); // set orientation
        self.dcs.write_command(self.madctl)?;

        Ok(())
    }

    ///
    /// Sets a pixel color at the given coords.
    ///
    /// # Arguments
    ///
    /// * `x` - x coordinate
    /// * `y` - y coordinate
    /// * `color` - the color value in pixel format of the display [Model]
    ///
    pub fn set_pixel(&mut self, x: u16, y: u16, color: M::ColorFormat) -> Result<(), Error> {
        self.set_address_window(x, y, x, y)?;
        self.model
            .write_pixels(&mut self.dcs, core::iter::once(color))?;

        Ok(())
    }

    ///
    /// Sets pixel colors in given rectangle bounds.
    ///
    /// # Arguments
    ///
    /// * `sx` - x coordinate start
    /// * `sy` - y coordinate start
    /// * `ex` - x coordinate end
    /// * `ey` - y coordinate end
    /// * `colors` - anything that can provide `IntoIterator<Item = u16>` to iterate over pixel data
    ///
    pub fn set_pixels<T>(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
        colors: T,
    ) -> Result<(), Error>
    where
        T: IntoIterator<Item = M::ColorFormat>,
    {
        self.set_address_window(sx, sy, ex, ey)?;
        self.model.write_pixels(&mut self.dcs, colors)?;

        Ok(())
    }

    ///
    /// Sets pixel colors in given rectangle bounds.
    ///
    /// # Arguments
    ///
    /// * `sx` - x coordinate start
    /// * `sy` - y coordinate start
    /// * `ex` - x coordinate end
    /// * `ey` - y coordinate end
    /// * `colors` - a buffer of u16
    ///
    pub fn set_pixels_raw(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
        colors: &mut [u16],
    ) -> Result<(), Error> {
        self.set_address_window(sx, sy, ex, ey)?;
        self.model.write_pixels_raw(&mut self.dcs, colors)?;

        Ok(())
    }

    ///
    /// Sets scroll region
    /// # Arguments
    ///
    /// * `tfa` - Top fixed area
    /// * `vsa` - Vertical scrolling area
    /// * `bfa` - Bottom fixed area
    ///
    pub fn set_scroll_region(&mut self, tfa: u16, vsa: u16, bfa: u16) -> Result<(), Error> {
        let vscrdef = dcs::SetScrollArea::new(tfa, vsa, bfa);
        self.dcs.write_command(vscrdef)
    }

    ///
    /// Sets scroll offset "shifting" the displayed picture
    /// # Arguments
    ///
    /// * `offset` - scroll offset in pixels
    ///
    pub fn set_scroll_offset(&mut self, offset: u16) -> Result<(), Error> {
        let vscad = dcs::SetScrollStart::new(offset);
        self.dcs.write_command(vscad)
    }

    // Sets the address window for the display.
    fn set_address_window(&mut self, sx: u16, sy: u16, ex: u16, ey: u16) -> Result<(), Error> {
        // add clipping offsets if present
        let offset = self.options.window_offset();
        let (sx, sy, ex, ey) = (sx + offset.0, sy + offset.1, ex + offset.0, ey + offset.1);

        self.dcs.write_command(dcs::SetColumnAddress::new(sx, ex))?;
        self.dcs.write_command(dcs::SetPageAddress::new(sy, ey))
    }

    ///
    /// Configures the tearing effect output.
    ///
    pub fn set_tearing_effect(&mut self, tearing_effect: TearingEffect) -> Result<(), Error> {
        self.dcs
            .write_command(dcs::SetTearingEffect(tearing_effect))
    }
}

#[cfg(feature = "async")]
impl<DI, M, RST> Display<DI, M, RST>
where
    DI: display_interface::AsyncWriteOnlyDataCommand,
    M: crate::asynch::models::Model,
    RST: OutputPin,
{
    ///  Async version of [Self::set_orientation]
    pub async fn async_set_orientation(&mut self, orientation: Orientation) -> Result<(), Error> {
        self.madctl = self.madctl.with_orientation(orientation); // set orientation
        self.dcs.async_write_command(self.madctl).await?;

        Ok(())
    }

    ///  Async version of [Self::set_pixel]
    pub async fn async_set_pixel(
        &mut self,
        x: u16,
        y: u16,
        color: M::ColorFormat,
    ) -> Result<(), Error> {
        self.async_set_address_window(x, y, x, y).await?;
        self.model
            .write_pixels(&mut self.dcs, core::iter::once(color))
            .await?;

        Ok(())
    }

    ///  Async version of [Self::set_pixels]
    pub async fn async_set_pixels<T>(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
        colors: T,
    ) -> Result<(), Error>
    where
        T: IntoIterator<Item = M::ColorFormat>,
    {
        self.async_set_address_window(sx, sy, ex, ey).await?;
        self.model.write_pixels(&mut self.dcs, colors).await?;

        Ok(())
    }

    ///
    /// Sets pixel colors in given rectangle bounds.
    ///
    /// # Arguments
    ///
    /// * `sx` - x coordinate start
    /// * `sy` - y coordinate start
    /// * `ex` - x coordinate end
    /// * `ey` - y coordinate end
    /// * `colors` - a buffer of u16
    ///
    pub async fn async_set_pixels_raw(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
        colors: &mut [u16],
    ) -> Result<(), Error> {
        self.async_set_address_window(sx, sy, ex, ey).await?;
        self.model.write_pixels_raw(&mut self.dcs, colors).await?;

        Ok(())
    }

    ///  Async version of [Self::set_scroll_region]
    pub async fn async_set_scroll_region(
        &mut self,
        tfa: u16,
        vsa: u16,
        bfa: u16,
    ) -> Result<(), Error> {
        let vscrdef = dcs::SetScrollArea::new(tfa, vsa, bfa);
        self.dcs.async_write_command(vscrdef).await
    }

    ///  Async version of [Self::set_scroll_offset]
    pub async fn async_set_scroll_offset(&mut self, offset: u16) -> Result<(), Error> {
        let vscad = dcs::SetScrollStart::new(offset);
        self.dcs.async_write_command(vscad).await
    }

    // Sets the address window for the display.
    async fn async_set_address_window(
        &mut self,
        sx: u16,
        sy: u16,
        ex: u16,
        ey: u16,
    ) -> Result<(), Error> {
        // add clipping offsets if present
        let offset = self.options.window_offset();
        let (sx, sy, ex, ey) = (sx + offset.0, sy + offset.1, ex + offset.0, ey + offset.1);

        self.dcs
            .async_write_command(dcs::SetColumnAddress::new(sx, ex))
            .await?;
        self.dcs
            .async_write_command(dcs::SetPageAddress::new(sy, ey))
            .await
    }

    ///  Async version of [Self::set_tearing_effect]
    pub async fn async_set_tearing_effect(
        &mut self,
        tearing_effect: TearingEffect,
    ) -> Result<(), Error> {
        self.dcs
            .async_write_command(dcs::SetTearingEffect(tearing_effect))
            .await
    }
}
