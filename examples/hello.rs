//#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;
extern crate stm32f4xx_hal as hal;
use cortex_m_rt::entry;
use hal::i2c::I2c;
use hal::gpio::gpioa;
use hal::prelude::*;
use cortex_m_semihosting::{hio, hprintln};
use core::fmt::Write;
use hal::time::KiloHertz;
use core::mem;



use hal::nb::block;

#[entry]
fn main() -> ! {
    //let mut u16_buf : [u8; 2] = [0; 2];
    //u16_buf.copy_from_slice(&buf[0..2]);
    //let u1 = u16::from_le_bytes(u16_buf);
    //println!("{:?}", u16_buf);

    let p = hal::stm32::Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let mut gpiob = p.GPIOB.split();
    let scl = gpiob.pb6.into_alternate_af4();
    let sda = gpiob.pb7.into_alternate_af4();
    let mut i2c = I2c::i2c1(p.I2C1, (scl, sda), 100.khz(),  clocks);

    let DEV_ADDR = 0x68;//0xd0;

    let mem_addr = [0x3b];
    let mut buf: [u8; 14] = [0; 14];

    //i2c.write(DEV_ADDR, &mem_addr).unwrap();
    i2c.read(DEV_ADDR, &mut buf).unwrap();

    let mut stdout = match hio::hstdout() {
        Ok(fd)  => fd,
        Err(()) => panic!("cannot use stdout"),
    };

    write!(stdout, "{:?}", buf).unwrap();

    //hprintln!("Hello, world!").unwrap();

    loop {
    }
}
