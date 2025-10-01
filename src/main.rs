#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;

use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{Async, Config, Error, I2c, Instance, InterruptHandler};

use core::marker::PhantomData;
use defmt_rtt as _;
use panic_probe as _;

// Program metadata for `picotool info`
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"5p49v-rs"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_description!(c"VersaClock 5p49v Rust Driver"),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

/// PLL structure
pub struct Pll5p49v<T> {
    i2c: PhantomData<T>,
}

impl<T> Pll5p49v<T>
where
    T: Instance,
{
    // I2C address of the PLL chip
    const I2C_ADDRESS: u16 = 0x6A;

    pub fn new(_i2c: &I2c<'_, T, Async>) -> Self {
        Self { i2c: PhantomData }
    }

    pub async fn write_config(
        &self,
        i2c: &mut I2c<'static, T, Async>,
        clock_fq_hz: u32,
        vco_fq_hz: u32,
        output1_fq_hz: u32,
        output2_fq_hz: u32,
        output3_fq_hz: u32,
        output4_fq_hz: u32,
    ) -> Result<(), Error> {
        // Calculate feedback divider, integer (upper 32 bits) and fractional (lower 32 bits)
        let feedback_divider = ((vco_fq_hz as u64) << 32) / (clock_fq_hz as u64);

        defmt::debug!(
            "feedback_divider, int: {}, frac: {} ({})",
            (feedback_divider >> 32) as u32,
            feedback_divider as u32,
            (feedback_divider as u32 as f64) / ((1u64 << 32) as f64)
        );

        let mut sigma_delta_order = 3; // 0=off, 1..3=order

        if (feedback_divider & 0xFFFFFFFF) == 0 {
            sigma_delta_order = 0; // Bypass Sigma Delta Modulator
        }

        // Calculate output dividers, integer (upper 32 bits) and fractional (lower 32 bits)

        let od1 = ((vco_fq_hz as u64) << 31) / (output1_fq_hz as u64);
        let od2 = ((vco_fq_hz as u64) << 31) / (output2_fq_hz as u64);
        let od3 = ((vco_fq_hz as u64) << 31) / (output3_fq_hz as u64);
        let od4 = ((vco_fq_hz as u64) << 31) / (output4_fq_hz as u64);

        defmt::debug!(
            "od1, int: {}, frac: {} ({})",
            (od1 >> 32) as u32,
            od1 as u32,
            (od1 as u32 as f64) / ((1u64 << 32) as f64)
        );

        defmt::debug!(
            "od2, int: {}, frac: {} ({})",
            (od2 >> 32) as u32,
            od2 as u32,
            (od2 as u32 as f64) / ((1u64 << 32) as f64)
        );

        defmt::debug!(
            "od3, int: {}, frac: {} ({})",
            (od3 >> 32) as u32,
            od3 as u32,
            (od3 as u32 as f64) / ((1u64 << 32) as f64)
        );

        defmt::debug!(
            "od4, int: {}, frac: {} ({})",
            (od4 >> 32) as u32,
            od4 as u32,
            (od4 as u32 as f64) / ((1u64 << 32) as f64)
        );

        const EN_GLOBAL_SHUTDOWN: bool = false; // default: false
        const SP: bool = false; // default: false
        const EN_XTAL: bool = false; // default: false
        const EN_CLKIN: bool = true; // default: true
        const PRIMSRC: bool = true; // default: true

        const TEST_MODE_VCO_BAND: bool = false; // default: true
        const VCO_BAND: u8 = 0x0D; // default: 0x0D

        const CALIBRATION_START: bool = true; // default: true - looks like does not matter in the main init array
        const VCO_MONITOR_EN: bool = false; // default: false - looks like does not work for 5p49v6965

        const PROG_ARRAY_SIZE: usize = 107;

        let prog_array: [u8; PROG_ARRAY_SIZE] = [
            0x00, // Send the start register address
            // Registers 0x00 - 0x16
            0x61,
            0x0F,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0xFF,
            0x01,
            0xC0,
            0x00,
            0xB6,
            0xB4,
            0x92,
            0x00 + if EN_GLOBAL_SHUTDOWN { 0x01 } else { 0 }
                + if SP { 0x02 } else { 0 }
                + if EN_CLKIN { 0x40 } else { 0 }
                + if EN_XTAL { 0x80 } else { 0 }, // 0x10 - Primary Source and Shutdown Register
            0x00 + if TEST_MODE_VCO_BAND { 0x20 } else { 0 } + VCO_BAND, // 0x11 VCO Band and Factory Reserved Bits
            0x81,                                  // 0x12 - Crystal X1 Load Capacitor Register
            0x80 + if PRIMSRC { 0x02 } else { 0 }, // 0x13 -  Factory Reserved Bit
            0x00,
            0x03,
            0x84,
            // End Registers 0x00 - 0x16
            (feedback_divider >> 36) as u8, // 0x17 Feedback divider integer
            (((feedback_divider >> 28) as u8) & 0xF0) + (sigma_delta_order << 2), // 0x18 Feedback divider integer
            //
            (feedback_divider >> 24) as u8, // 0x19 Feedback divider fraction
            (feedback_divider >> 16) as u8, // 0x1A Feedback divider fraction
            (feedback_divider >> 8) as u8,  // 0x1B Feedback divider fraction
            0x1F + if CALIBRATION_START { 0x80 } else { 0 }, // 0x1C Factory Reserved Bits
            0xFD + if VCO_MONITOR_EN { 0x02 } else { 0 }, // 0x1D Factory Reserved Bits; NO! not Select VCO automatically
            0xC8,
            0x80,
            0x00,
            0x81,
            ((od1 >> 30) as u8) & 0b11,      // 0x22 OD1 fraction
            (od1 >> 22) as u8,               // 0x23 OD1 fraction
            (od1 >> 14) as u8,               // 0x24 OD1 fraction
            ((od1 >> 6) as u8) & 0b11111100, // 0x25 OD1 fraction
            0x00,
            0x00,
            0x00,
            0x00,
            0x04,
            0x00,
            0x00,                       // Registers 0x26 - 0x2C
            (od1 >> 36) as u8,          // 0x2D OD1 integer
            ((od1 >> 28) as u8) & 0xF0, // 0x2E OD1 integer
            0x00,
            0x00,
            0x81,                            // Registers 0x2F - 0x31
            ((od2 >> 30) as u8) & 0b11,      // 0x32 OD2 fraction
            (od2 >> 22) as u8,               // 0x33 OD2 fraction
            (od2 >> 14) as u8,               // 0x34 OD2 fraction
            ((od2 >> 6) as u8) & 0b11111100, // 0x35 OD2 fraction
            0x00,
            0x00,
            0x00,
            0x00,
            0x04,
            0x00,
            0x00,                       // Registers 0x36 - 0x3C
            (od2 >> 36) as u8,          // 0x3D OD2 integer
            ((od2 >> 28) as u8) & 0xF0, // 0x3E OD2 integer
            0x00,
            0x00,
            0x81,                            // Registers 0x3F - 0x41
            ((od3 >> 30) as u8) & 0b11,      // 0x42 OD3 fraction
            (od3 >> 22) as u8,               // 0x43 OD3 fraction
            (od3 >> 14) as u8,               // 0x44 OD3 fraction
            ((od3 >> 6) as u8) & 0b11111100, // 0x45 OD3 fraction
            0x00,
            0x00,
            0x00,
            0x00,
            0x04,
            0x00,
            0x00,                       // Registers 0x46 - 0x4C
            (od3 >> 36) as u8,          // 0x4D OD3 integer
            ((od3 >> 28) as u8) & 0xF0, // 0x4E OD3 integer
            0x00,
            0x00,
            0x81,                            // Registers 0x4F - 0x51
            ((od4 >> 30) as u8) & 0b11,      // 0x52 OD4 fraction
            (od4 >> 22) as u8,               // 0x53 OD4 fraction
            (od4 >> 14) as u8,               // 0x54 OD4 fraction
            ((od4 >> 6) as u8) & 0b11111100, // 0x55 OD4 fraction
            0x00,
            0x00,
            0x00,
            0x00,
            0x04,
            0x00,
            0x00,                       // Registers 0x56 - 0x5C
            (od4 >> 36) as u8,          // 0x5D OD4 integer
            ((od4 >> 28) as u8) & 0xF0, // 0x5E OD4 integer
            0x00,                       // Register 0x5F
            0x3B,
            0x01, // 0x60, 0x61 - Clock1 output configuration
            0x3B,
            0x01, // 0x62, 0x63 - Clock2 output configuration
            0x3B,
            0x01, // 0x64, 0x65 - Clock3 output configuration
            0x3B,
            0x01, // 0x66, 0x67 - Clock4 output configuration
            0xFF,
            0xFC, // Registers 0x68 - 0x69 (all outputs enabled, 3.3V out, fastest slew rate)
        ];

        i2c.write_async(Self::I2C_ADDRESS, prog_array).await
    }

    pub async fn calibrate_vco(&self, i2c: &mut I2c<'static, T, Async>) -> Result<(), Error> {
        // Read 7th bit of the register 0x1C
        // write 0-1-0 to this bit

        let mut resp_buff = [0u8; 1];
        i2c.write_read_async(Self::I2C_ADDRESS, [0x1C], &mut resp_buff)
            .await?;

        i2c.write_async(Self::I2C_ADDRESS, [0x1c, resp_buff[0] & 0x7F])
            .await?;

        Timer::after_millis(1).await;

        i2c.write_async(Self::I2C_ADDRESS, [0x1c, resp_buff[0] | 0x80])
            .await?;

        Timer::after_millis(1).await;

        i2c.write_async(Self::I2C_ADDRESS, [0x1c, resp_buff[0] & 0x7F])
            .await?;

        Timer::after_millis(1).await;

        // Read register 0x99, output 5 MSBs as a VCO index

        let mut resp_buff = [0u8; 1];
        i2c.write_read_async(Self::I2C_ADDRESS, [0x99], &mut resp_buff)
            .await?;

        defmt::debug!("VCO index: {}", resp_buff[0] >> 3);

        Ok(())
    }
}

// Interrupt handler for async mode
embassy_rp::bind_interrupts!(struct Irqs {
    I2C1_IRQ => InterruptHandler<embassy_rp::peripherals::I2C1>;
});

/// Main function
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Status LED
    let mut led = Output::new(p.PIN_25, Level::Low);

    // i2c pins
    let sda = p.PIN_14;
    let scl = p.PIN_15;

    // create async i2c instance
    let mut i2c = I2c::new_async(p.I2C1, scl, sda, Irqs, Config::default());

    //    let pll = Pll5p49v::<embassy_rp::peripherals::I2C1>::new(&i2c);
    let pll = Pll5p49v::new(&i2c);

    let clock_fq_hz = 10000000u32;
    let vco_fq_hz = 2700000000u32;

    let (output1_fq_hz, output2_fq_hz, output3_fq_hz, output4_fq_hz) =
        (40000000, 25000000, 24000000, 28800000);

    // Program frequencied

    let rc = pll
        .write_config(
            &mut i2c,
            clock_fq_hz,
            vco_fq_hz,
            output1_fq_hz,
            output2_fq_hz,
            output3_fq_hz,
            output4_fq_hz,
        )
        .await;

    match rc {
        Ok(_) => defmt::debug!("Programmed OK"),
        Err(e) => defmt::error!("Error programming PLL: {}", e),
    }

    // Calibrate VCO
    // 5th generation does not need this
    // 6th generation requires this

    let rc = pll.calibrate_vco(&mut i2c).await;

    match rc {
        Ok(_) => defmt::debug!("VCO calibrated OK"),
        Err(e) => defmt::error!("Error calibrating VCO: {}", e),
    }

    led.set_high(); // OK
    defmt::debug!("Finished!");
}
