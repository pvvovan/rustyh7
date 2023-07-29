#![no_main]
#![no_std]

mod isr;

use core::panic::PanicInfo;

#[panic_handler]
fn panic(_panic: &PanicInfo<'_>) -> ! {
    loop {}
}

#[no_mangle]
pub extern "C" fn DefaultExceptionHandler() {
    loop {}
}

#[no_mangle]
pub extern "C" fn HardFault() {
    loop {}
}

#[no_mangle]
extern "C" fn main() -> ! {
    tim_start();

    let x = RODATA;
    let y = unsafe { &mut BSS };
    let z = unsafe { &mut DATA };
    let mut _fl: f32 = 0.0;

    loop {
        *z += x[0] as u16;
        *y += 2;
        _fl += 1.1;

        if *y > 253 {
            *y = 0;
            *z = 1;
        }
    }
}

static RODATA: &[u8] = b"Hello, world!";
static mut BSS: u8 = 0;
static mut DATA: u16 = 1;

fn tim_start() {
    /* Enable SysTick: 1. Program reload value */
    const SYST_RVR: *mut u32 = 0xE000_E014 as *mut u32;
    const SYST_RVR_RELOAD: u32 = 6_400_000;
    unsafe {
        core::ptr::write_volatile(SYST_RVR, SYST_RVR_RELOAD);
    }
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

    /* Enable SysTick: 2. Clear current value */
    const SYST_CVR: *mut u32 = 0xE000_E018 as *mut u32;
    unsafe {
        core::ptr::write_volatile(SYST_CVR, 0);
    }
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

    /* Enable SysTick: 3. Program Control and Status register */
    const SYST_CSR: *mut u32 = 0xE000_E010 as *mut u32;
    const SYST_CSR_CLKSOURCE: u32 = 1 << 2;
    const SYST_CSR_TICKINT: u32 = 1 << 1;
    const SYST_CSR_ENABLE: u32 = 1 << 0;
    unsafe {
        core::ptr::write_volatile(
            SYST_CSR,
            SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE,
        );
    }
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
}
