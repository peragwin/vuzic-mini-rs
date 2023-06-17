use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use log::*;
// use esp_idf_hal::prelude::*;
// use esp_idf_hal::units::*;
use anyhow::{anyhow, Result};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::{gpio, i2c::I2C0};
use esp_idf_sys as _;
use std::thread;
use std::time::Duration;

use smart_leds::hsv::{hsv2rgb, Hsv};
use smart_leds_trait::SmartLedsWrite;
use ws2812_esp32_rmt_driver::Ws2812Esp32Rmt;

use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use lsm6ds3tr::{AccelerometerOutput, GyroscopeFullScale, GyroscopeOutput, Lsm6ds3tr};
use nalgebra::{UnitQuaternion, Vector3};

struct IMU<'a> {
    lsm: Lsm6ds3tr<I2cDriver<'a>>,
    moment: UnitQuaternion<f32>,
}

impl IMU<'_> {
    pub fn new(i2c: I2C0, sda: gpio::Gpio8, scl: gpio::Gpio2) -> Result<Self> {
        let i2c = I2cDriver::new(i2c, sda, scl, &I2cConfig::new().baudrate(400_000.into()))?;

        // let mut res = [0u8];
        // i2c.write_read(0x6b, &[0xF], &mut res, 1000)?;
        // if res[0] != 0x69 {
        //     return Err(anyhow!("failed to read lsm6ds33 whoami: {}", res[0]));
        // }

        let mut lsm = Lsm6ds3tr::new(i2c, 0x6a)
            .map_err(|e| e.1)
            .map_err(|e| anyhow!("failed to create lsm6ds: {:?}", e))?;
        lsm.set_gyroscope_output(GyroscopeOutput::Rate1_66k)
            .map_err(|e| anyhow!("{:?}", e))?;
        lsm.set_gyroscope_scale(GyroscopeFullScale::Dps500)
            .map_err(|e| anyhow!("{:?}", e))?;
        lsm.set_accelerometer_output(AccelerometerOutput::PowerDown)
            .map_err(|e| anyhow!("{:?}", e))?;
        Ok(Self {
            lsm,
            moment: UnitQuaternion::identity(),
        })
    }

    fn read_gyro(&mut self) -> Option<Vector3<f32>> {
        // self.lsm
        //     .gyro_data_available()
        //     .ok()
        //     .filter(|&b| b)
        //     .and_then(|_| {
        self.lsm
            .read_gyro()
            .ok()
            .map(|(x, y, z)| Vector3::new(x, y, z))
        // })
    }

    pub fn update(&mut self, dt: Duration) {
        if let Some(val) = self.read_gyro() {
            let da = val * dt.as_secs_f32();
            let dq = UnitQuaternion::from_euler_angles(da.x, da.y, da.z);
            self.moment *= dq;
        }
    }

    pub fn get_orientation(&self) -> (f32, f32, f32) {
        self.moment.euler_angles()
    }
}

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_sys::link_patches();
    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let mut status_led = gpio::PinDriver::output(peripherals.pins.gpio21).unwrap();
    status_led.set_high().unwrap();

    let mut led_enable = gpio::PinDriver::output(peripherals.pins.gpio1).unwrap();
    led_enable.set_high().unwrap();

    let mut leds = Ws2812Esp32Rmt::new(0, 7).unwrap();
    const NUM_LEDS: usize = 60;

    let imu = IMU::new(
        peripherals.i2c0,
        peripherals.pins.gpio8,
        peripherals.pins.gpio2,
    );
    if imu.is_err() {
        status_led.set_low().unwrap();
        error!("failed to create imu: {}", imu.err().unwrap());
        loop {
            thread::sleep(Duration::from_millis(1000));
        }
    }
    let mut imu = imu.unwrap();
    let mut last = std::time::Instant::now();

    loop {
        let dt = last.elapsed();
        last = std::time::Instant::now();
        imu.update(dt);

        let (yaw, pitch, roll) = imu.get_orientation();
        let yaw = (255.0 * yaw / 2.0 / std::f32::consts::PI + 128.0) as i32;
        let pitch = (255.0 * pitch / 2.0 / std::f32::consts::PI + 128.0) as i32;
        let roll = (255.0 * roll / 2.0 / std::f32::consts::PI + 128.0) as i32;
        info!("yaw: {}, pitch: {}, roll: {}", yaw, pitch, roll);
        let hue = roll as u8;

        let pixels = (0..NUM_LEDS).map(|i| {
            hsv2rgb(Hsv {
                hue: hue.wrapping_add((255 / NUM_LEDS * i) as u8),
                sat: 255,
                val: 16,
            })
        });

        leds.write(pixels).unwrap();
        thread::sleep(Duration::from_millis(5));
        // hue = hue.wrapping_add(1);
    }

    // let n = 1;

    // while n == 1 {
    //     led.set_high().unwrap();
    //     thread::sleep(Duration::from_millis(1000));

    //     led.set_low().unwrap();
    //     thread::sleep(Duration::from_millis(1000));

    //     info!("blink");
    // }
}
