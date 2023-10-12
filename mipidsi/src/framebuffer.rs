use core::convert::TryFrom;
use embedded_graphics_core::{
    prelude::{DrawTarget, OriginDimensions, PixelColor, Point, Size},
    Pixel,
};

pub struct FrameBuffer<C, const N: usize> {
    pixels: [C; N],
    size: Size,
}

impl<C: PixelColor, const N: usize> FrameBuffer<C, N> {
    pub fn new(init_colour: C, size: Size) -> Self {
        Self {
            pixels: [init_colour; N],
            size,
        }
    }

    fn point_to_idx(&self, point: Point) -> Option<usize> {
        let px = u32::try_from(point.x).ok()?;
        let py = u32::try_from(point.y).ok()?;
        if px > self.size.width || py > self.size.height {
            return None;
        }
        Some(((py * self.size.width) + px) as usize)
    }

    pub fn draw_pixel(&mut self, point: Point, color: C) {
        if let Some(idx) = self.point_to_idx(point) {
            self.pixels[idx] = color;
        }
    }
}

impl<C: PixelColor, const N: usize> DrawTarget for FrameBuffer<C, N>
where
    C: PixelColor,
{
    type Color = C;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for pixel in pixels.into_iter() {
            let Pixel(point, color) = pixel;

            self.draw_pixel(point, color);
        }

        Ok(())
    }
}

impl<C: PixelColor, const N: usize> OriginDimensions for FrameBuffer<C, N> {
    fn size(&self) -> Size {
        self.size
    }
}
