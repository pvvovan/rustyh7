#![no_main]
#![no_std]

mod sys;

static RODATA: &[u8] = b"Hello, world!";
static mut BSS: u8 = 0;
static mut DATA: u16 = 113;

pub fn main() -> ! {
    let _x = RODATA;
    let _y = unsafe { &BSS };
    let _z = unsafe { &DATA };

    loop {}
}

#[no_mangle]
pub extern "C" fn HardFault() -> ! {
    loop {}
}
