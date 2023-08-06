#![no_main]
#![no_std]

mod gpio;
mod isr;

use core::arch::asm;
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
    enable_cache();
    flash_setlatency();
    powercontrol_init();
    clock_config();
    tim_start();

    let x = RODATA;
    let y = unsafe { &mut BSS };
    let z = unsafe { &mut DATA };
    let mut _fl: f32 = 0.0;

    let gpio_b = gpio::Gpio::take(gpio::Port::GpioB, &[gpio::Pin::Pin0, gpio::Pin::Pin14]);

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

    /* Enable SysTick: 2. Clear current value */
    const SYST_CVR: *mut u32 = 0xE000_E018 as *mut u32;
    unsafe {
        core::ptr::write_volatile(SYST_CVR, 0);
    }

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
}

fn flash_setlatency() {
    const FLASH_BASE: u32 = 0x5200_2000;
    const FLASH_ACR: *mut u32 = FLASH_BASE as *mut u32;
    const FLASH_ACR_LATENCY_3WS: u32 = 0x0000_0003;
    const FLASH_ACR_LATENCY_MSK: u32 = 0xFFFF_FFF0;

    let mut latency = unsafe { core::ptr::read_volatile(FLASH_ACR) };
    latency = latency & FLASH_ACR_LATENCY_MSK;
    latency = latency | FLASH_ACR_LATENCY_3WS;
    unsafe {
        core::ptr::write_volatile(FLASH_ACR, latency);
    }

    loop {
        if unsafe { core::ptr::read_volatile(FLASH_ACR) } & FLASH_ACR_LATENCY_3WS != 0 {
            break;
        }
    }
}

fn powercontrol_init() {
    const PWR_BASE: u32 = 0x5802_4800;
    const PWR_CR3: *mut u32 = (PWR_BASE + 0x00C) as *mut u32;

    const PWR_CR3_BYPASS: u32 = 1 << 0;
    const PWR_CR3_LDOEN: u32 = 1 << 1;
    const PWR_CR3_SCEN: u32 = 1 << 2;

    let mut cr3 = unsafe { core::ptr::read_volatile(PWR_CR3) };
    cr3 = cr3 & (!PWR_CR3_BYPASS) & (!PWR_CR3_SCEN);
    cr3 = cr3 | PWR_CR3_LDOEN;
    unsafe { core::ptr::write_volatile(PWR_CR3, cr3) }

    /* When increasing the performance, the voltage scaling must be changed before
    increasing the system frequency. */
    const PWR_D3CR: *mut u32 = (PWR_BASE + 0x018) as *mut u32;
    const PWR_D3CR_VOS: u32 = 0b11 << 14;
    unsafe { core::ptr::write_volatile(PWR_D3CR, !PWR_D3CR_VOS) }

    const PWR_D3CR_VOSRDY: u32 = 1 << 13;
    loop {
        if unsafe { core::ptr::read_volatile(PWR_D3CR) & PWR_D3CR_VOSRDY } != 0 {
            break;
        }
    }
}

fn enable_cache() {
    const SBC_CCR: *mut u32 = 0xE000_ED14 as *mut u32;

    /* Enable I-Cache */
    unsafe {
        asm!("dsb");
        asm!("isb");

        /* Invalidate instruction cache */
        const SBC_ICIALLU: *mut u32 = 0xE000_EF50 as *mut u32;
        core::ptr::write_volatile(SBC_ICIALLU, 0);
        asm!("dsb 0xF");
        asm!("isb 0xF");

        /* Enable L1 instruction cache */
        const SCB_CCR_IC: u32 = 1 << 17;
        let mut ccr = core::ptr::read_volatile(SBC_CCR);
        ccr = ccr | SCB_CCR_IC;
        core::ptr::write_volatile(SBC_CCR, ccr);
        asm!("dsb sy");
        asm!("isb sy");
    }

    /* Enable D-Cache */
    unsafe {
        /* return if DCache is already enabled */
        const SCB_CCR_DC: u32 = 1 << 16;
        if core::ptr::read_volatile(SBC_CCR) & SCB_CCR_DC != 0 {
            return;
        }

        /* select Level 1 data cache */
        const SCB_CSSELR: *mut u32 = 0xE000_ED84 as *mut u32;
        core::ptr::write_volatile(SCB_CSSELR, 0);
        asm!("dsb 0xF");

        /* invalidate D-Cache */
        const SCB_CCSIDR: *mut u32 = 0xE000_ED80 as *mut u32;
        let ccsidr = core::ptr::read_volatile(SCB_CCSIDR);
        const SCB_CCSIDR_NUMSETS_POS: u8 = 13;
        const SCB_CCSIDR_NUMSETS_MSK: u32 = 0x7FFF << SCB_CCSIDR_NUMSETS_POS;
        let num_sets = ((ccsidr) & SCB_CCSIDR_NUMSETS_MSK) >> SCB_CCSIDR_NUMSETS_POS;
        for num_set in (0..=num_sets).rev() {
            const SCB_CCSIDR_ASSOCIATIVITY_POS: u8 = 3;
            const SCB_CCSIDR_ASSOCIATIVITY_MSK: u32 = 0x3FF << SCB_CCSIDR_ASSOCIATIVITY_POS;
            let num_ways =
                ((ccsidr) & SCB_CCSIDR_ASSOCIATIVITY_MSK) >> SCB_CCSIDR_ASSOCIATIVITY_POS;
            for num_way in (0..=num_ways).rev() {
                const SCB_DCISW: *mut u32 = 0xE000_EF60 as *mut u32;
                const SCB_DCISW_SET_POS: u8 = 5;
                const SCB_DCISW_SET_MSK: u32 = 0x1FF << SCB_DCISW_SET_POS;
                const SCB_DCISW_WAY_POS: u8 = 30;
                const SCB_DCISW_WAY_MSK: u32 = 0x3 << SCB_DCISW_WAY_POS;
                core::ptr::write_volatile(
                    SCB_DCISW,
                    ((num_set << SCB_DCISW_SET_POS) & SCB_DCISW_SET_MSK)
                        | ((num_way << SCB_DCISW_WAY_POS) & SCB_DCISW_WAY_MSK),
                );
                core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
            }
        }
        asm!("dsb 0xF");

        /* enable D-Cache */
        let mut ccr = core::ptr::read_volatile(SBC_CCR);
        ccr = ccr | SCB_CCR_DC;
        core::ptr::write_volatile(SBC_CCR, ccr);
        asm!("dsb 0xF");
        asm!("isb 0xF");
    }
}

fn clock_config() {
    const RCC_BASE: u32 = 0x5802_4400;
    const RCC_CR: *mut u32 = RCC_BASE as *mut u32;

    /* Enable HSE external oscillator (HSE Bypass) */
    const RCC_CR_HSEBYP: u32 = 1 << 18;
    let mut cr = unsafe { core::ptr::read_volatile(RCC_CR) };
    cr |= RCC_CR_HSEBYP;
    unsafe { core::ptr::write_volatile(RCC_CR, cr) }
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

    /* Enable HSE crystal oscillator (HSE ON) */
    const RCC_CR_HSEON: u32 = 1 << 16;
    cr = unsafe { core::ptr::read_volatile(RCC_CR) };
    cr |= RCC_CR_HSEON;
    unsafe { core::ptr::write_volatile(RCC_CR, cr) }
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

    /* Wait till HSE is ready */
    loop {
        const RCC_CR_HSERDY: u32 = 1 << 17;
        cr = unsafe { core::ptr::read_volatile(RCC_CR) };
        if cr & RCC_CR_HSERDY != 0 {
            break;
        }
    }
}
