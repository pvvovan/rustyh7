#![no_main]
#![no_std]

mod gpio;
mod isr;

use core::panic::PanicInfo;

#[panic_handler]
fn panic(_panic: &PanicInfo<'_>) -> ! {
    loop {}
}

#[no_mangle]
extern "C" fn DefaultExceptionHandler() {
    loop {}
}

#[no_mangle]
extern "C" fn HardFault() {
    loop {}
}

#[no_mangle]
extern "C" fn main() -> ! {
    tim_start();
    flash_setlatency();

    let x = RODATA;
    let y = unsafe { &mut BSS };
    let z = unsafe { &mut DATA };
    let mut _fl: f32 = 0.0;

    // let _gpio_a = gpio::Gpio::take(
    //     gpio::Port::GpioA,
    //     &[
    //         gpio::Pin::Pin0,
    //         gpio::Pin::Pin1,
    //         gpio::Pin::Pin2,
    //         gpio::Pin::Pin3,
    //         gpio::Pin::Pin4,
    //         gpio::Pin::Pin5,
    //         gpio::Pin::Pin6,
    //         gpio::Pin::Pin7,
    //         gpio::Pin::Pin8,
    //         gpio::Pin::Pin9,
    //         gpio::Pin::Pin10,
    //         gpio::Pin::Pin11,
    //         gpio::Pin::Pin12,
    //         gpio::Pin::Pin13,
    //         gpio::Pin::Pin14,
    //         gpio::Pin::Pin15,
    //     ],
    // );
    let gpio_b = gpio::Gpio::take(gpio::Port::GpioB, &[gpio::Pin::Pin0, gpio::Pin::Pin14]);
    // let _gpio_c = gpio::Gpio::take(gpio::Port::GpioC, &[gpio::Pin::Pin0]);

    gpio_b.set(&[gpio::Pin::Pin0]);
    gpio_b.reset(&[gpio::Pin::Pin14]);

    gpio_b.reset(&[gpio::Pin::Pin0]);
    gpio_b.set(&[gpio::Pin::Pin14]);

    loop {
        *z += x[0] as u16;
        *y += 2;
        _fl += 1.1;

        if *y > 253 {
            let mut lock = isr::lock();
            let _lock2 = isr::lock();
            *y = 0;
            *z = 1;
            lock.unlock();
        }

        delay(1);
        gpio_b.set(&[gpio::Pin::Pin0]);
        gpio_b.reset(&[gpio::Pin::Pin14]);

        delay(1);
        gpio_b.reset(&[gpio::Pin::Pin0]);
        gpio_b.set(&[gpio::Pin::Pin14]);
    }
}

fn delay(seconds: u32) {
    let ori_sec = isr::sys_seconds();
    loop {
        if isr::sys_seconds() - ori_sec >= seconds {
            break;
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

fn flash_setlatency() {
    const FLASH_BASE: u32 = 0x5200_2000;
    const FLASH_ACR: *mut u32 = FLASH_BASE as *mut u32;
    const FLASH_ACR_LATENCY_3WS: u32 = 0x0000_0003;
    const FLASH_ACR_LATENCY_MSK: u32 = 0xFFFF_FFF0;

    let mut latency = unsafe { core::ptr::read(FLASH_ACR) };
    latency = latency & FLASH_ACR_LATENCY_MSK;
    latency = latency | FLASH_ACR_LATENCY_3WS;
    unsafe {
        core::ptr::write(FLASH_ACR, latency);
    }
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
    loop {
        if unsafe { core::ptr::read(FLASH_ACR) } & FLASH_ACR_LATENCY_3WS != 0 {
            break;
        }
    }
}
