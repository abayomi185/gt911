//! GT911 Touchscreen Controller

#![no_std]

use core::fmt;

use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    i2c::I2c,
};

// I²C addresses for the GT911
const PRIMARY_I2C_ADDR: u8 = 0x5D;
const SECONDARY_I2C_ADDR: u8 = 0x14;

// Default I²C address for the GT911
const I2C_ADDR: u8 = PRIMARY_I2C_ADDR;

const GT911_COMMAND: u16 = 0x8040;
const GT911_PRODUCT_ID: u16 = 0x8140;
const GT911_POINT_STATUS: u16 = 0x814E;
const GT911_POINT_START: u16 = 0x814F;
const GT911_X_OUTPUT_MAX_LOW: u16 = 0x8048;
const GT911_X_OUTPUT_MAX_HIGH: u16 = 0x8049;
const GT911_Y_OUTPUT_MAX_LOW: u16 = 0x804A;
const GT911_Y_OUTPUT_MAX_HIGH: u16 = 0x804B;
const GT911_POINT_1: u16 = 0x814F;

/// Error type for the GT911 driver
#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    Init(E),
    Pin,
    Callback,
}

#[derive(Debug)]
pub enum CallbackError {
    GenericError,
}

impl<E: fmt::Debug> fmt::Display for Error<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::I2c(e) => write!(f, "I2C error: {:?}", e),
            Error::Init(e) => write!(f, "Init error: {:?}", e),
            Error::Pin => write!(f, "Pin error"),
            Error::Callback => write!(f, "Callback error"),
        }
    }
}

/// GT911 Options
#[derive(Default)]
pub struct GT911Options {
    /// Resolution (w, h) for display.
    pub resolution: (u16, u16),
}

#[derive(Default, Copy, Clone)]
pub struct GT911Point {
    id: u8,
    x: u16,
    y: u16,
    size: u16,
}

/// GT911 Builder
pub struct GT911Builder<I2C, RESET, DELAY, CB> {
    /// Underlying I²C peripheral
    i2c: I2C,
    /// Reset pin
    reset: RESET,
    /// Delay provider
    delay: DELAY,
    /// Options
    options: GT911Options,
    // Builder callback
    callback: Option<CB>,
}

impl<I2C, RESET, DELAY, CB> GT911Builder<I2C, RESET, DELAY, CB>
where
    I2C: I2c,
    RESET: OutputPin,
    DELAY: DelayNs,
    CB: FnMut() -> bool,
{
    /// Creates a new GT911Builder instance.
    pub fn new(i2c: I2C, reset: RESET, delay: DELAY) -> Self {
        Self {
            i2c,
            reset,
            delay,
            options: GT911Options::default(),
            callback: None,
        }
    }

    /// Sets the resolution for GT911 touchscreen
    pub fn resolution(mut self, width: u16, height: u16) -> Self {
        assert!(width != 0 && height != 0);

        self.options.resolution = (width, height);
        self
    }

    /// Sets the callback function which will be called after creating [GT911] in the build method
    pub fn callback(mut self, callback: CB) -> Self {
        self.callback = Some(callback);
        self
    }

    /// Builds the GT911 touch controller with the provided options.
    pub fn build(self) -> Result<GT911<I2C, RESET, DELAY>, Error<I2C::Error>> {
        let mut gt911 = GT911::new(self.i2c, self.reset, self.delay)?;

        if let Some(mut callback) = self.callback {
            match !callback() {
                true => return Err(Error::Callback),
                false => (),
            }
        }

        gt911.set_resolution(self.options.resolution.0, self.options.resolution.1)?;

        Ok(gt911)
    }
}

/// GT911 driver
pub struct GT911<I2C, RESET, DELAY> {
    /// Underlying I²C peripheral
    i2c: I2C,
    /// Reset pin
    reset: RESET,
    /// Delay provider
    delay: DELAY,
}

impl<I2C, RESET, DELAY> GT911<I2C, RESET, DELAY>
where
    I2C: I2c,
    RESET: OutputPin,
    DELAY: DelayNs,
{
    /// Create a new instance of the driver and initialize the device
    fn new(i2c: I2C, reset: RESET, delay: DELAY) -> Result<Self, Error<I2C::Error>> {
        let mut gt911 = Self { i2c, reset, delay };
        gt911.reset()?;
        // Additional initialization can be done here if needed
        Ok(gt911)
    }

    /// Perform a hardware reset
    fn reset(&mut self) -> Result<(), Error<I2C::Error>> {
        self.reset.set_low().map_err(|_| Error::Pin)?;
        self.delay.delay_ms(1);

        self.reset.set_high().map_err(|_| Error::Pin)?;
        self.delay.delay_ms(10);

        Ok(())
    }

    /// Set resolution of touchscreen
    fn set_resolution(&mut self, width: u16, height: u16) -> Result<(), Error<I2C::Error>> {
        let width_buf = [(width >> 8) as u8, width as u8];
        let height_buf = [(height >> 8) as u8, height as u8];

        self.write_register(GT911_X_OUTPUT_MAX_HIGH, &width_buf[0..1])?; // High byte of width
        self.write_register(GT911_X_OUTPUT_MAX_LOW, &width_buf[1..2])?; // Low byte of width

        self.write_register(GT911_Y_OUTPUT_MAX_HIGH, &height_buf[0..1])?; // High byte of height
        self.write_register(GT911_Y_OUTPUT_MAX_LOW, &height_buf[1..2])?; // Low byte of height
        Ok(())
    }

    /// Get resolution of touchscreen
    pub fn get_resolution(&mut self) -> Result<(u16, u16), Error<I2C::Error>> {
        let mut width_buf = [0u8; 2];
        let mut height_buf = [0u8; 2];

        self.read_register(GT911_X_OUTPUT_MAX_HIGH, &mut width_buf[0..1])?;
        self.read_register(GT911_X_OUTPUT_MAX_LOW, &mut width_buf[1..2])?;

        self.read_register(GT911_Y_OUTPUT_MAX_HIGH, &mut height_buf[0..1])?;
        self.read_register(GT911_Y_OUTPUT_MAX_LOW, &mut height_buf[1..2])?;

        let width = u16::from_be_bytes(width_buf);
        let height = u16::from_be_bytes(height_buf);

        Ok((width, height))
    }

    /// Write data to a register
    fn write_register(&mut self, reg: u16, data: &[u8]) -> Result<(), Error<I2C::Error>> {
        let mut buf = [0u8; 2 + 128];
        buf[0] = (reg >> 8) as u8;
        buf[1] = reg as u8;
        buf[2..(2 + data.len())].copy_from_slice(data);
        self.i2c
            .write(I2C_ADDR, &buf[..(2 + data.len())])
            .map_err(|e| {
                log::error!("I2C write failed for register {:#04x}: {:?}", reg, e);
                Error::I2c(e)
            })
    }

    /// Read data from a register
    fn read_register(&mut self, reg: u16, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        let reg_buf = [(reg >> 8) as u8, reg as u8];
        self.i2c.write_read(I2C_ADDR, &reg_buf, buf).map_err(|e| {
            log::error!("I2C write-read failed for register {:#04x}: {:?}", reg, e);
            Error::I2c(e)
        })
    }

    /// Get point from data buffer
    fn get_point(data: &[u8]) -> GT911Point {
        GT911Point {
            id: data[0],
            x: (data[1] as u16) << 8 | data[2] as u16,
            y: (data[3] as u16) << 8 | data[4] as u16,
            size: (data[5] as u16) << 8 | data[6] as u16,
        }
    }

    /// Read points from controller
    pub fn read_points(&mut self) -> Result<[GT911Point; 5], Error<I2C::Error>> {
        let mut points_status = [0u8; 1];
        self.read_register(GT911_POINT_STATUS, &mut points_status)?;

        let mut touch_buf = [0u8; 7];
        let mut touch_points = [GT911Point::default(); 5]; // Up to 5 touches

        let buffer_status = points_status[0] >> 7 & 1;
        let _proximity_valid = points_status[0] >> 5 & 1;
        let _have_key = points_status[0] >> 4 & 1;
        let _is_large_detect = points_status[0] >> 6 & 1;
        let touches = points_status[0] & 0xF;

        match buffer_status == 1 && touches > 0 {
            true => {
                for index in 0..touches {
                    self.read_register(GT911_POINT_1 + (index as u16 * 8), &mut touch_buf)
                        .ok();
                    touch_points[index as usize] = Self::get_point(&touch_buf);
                }
                // Reset the buffer for the next series of touches
                self.write_register(GT911_POINT_STATUS, &[0]).ok();
            }
            false => (),
        }

        Ok(touch_points)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
}
