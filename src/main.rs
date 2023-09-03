#![no_main]
#![no_std]

mod flash;
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
    // enable_cache();
    flash::setlatency();
    powercontrol_init();
    clock_config();
    tim_start();

    let mut val = flash::read();

    let x = RODATA;
    let y = unsafe { &mut BSS };
    let z = unsafe { &mut DATA };
    let mut _fl: f32 = 0.0;

    let gpio_b = gpio::Gpio::take(gpio::Port::GpioB, &[gpio::Pin::Pin0, gpio::Pin::Pin14]).unwrap();
    let _gpio_b = gpio::Gpio::take(gpio::Port::_GpioA, &[]).unwrap();

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

        delay(1 + val as u32);
        gpio_b.set(&[gpio::Pin::Pin0]);
        gpio_b.reset(&[gpio::Pin::Pin14]);

        val += 0.25;
        flash::write(val);

        delay(1 + val as u32);
        gpio_b.reset(&[gpio::Pin::Pin0]);
        gpio_b.set(&[gpio::Pin::Pin14]);

        val = flash::read();
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
    const SYST_RVR_RELOAD: u32 = 5_500_000;
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
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

    /* Set HSE as PLL source */
    const RCC_PLLCKSELR: *mut u32 = (RCC_BASE + 0x028) as *mut u32;
    const RCC_PLLCKSELR_PLLSRC: u32 = 0x3 << 0;
    const RCC_PLLSOURCE_HSE: u32 = 1 << 1;
    let mut pllckselr = unsafe { core::ptr::read_volatile(RCC_PLLCKSELR) };
    pllckselr &= !RCC_PLLCKSELR_PLLSRC;
    pllckselr |= RCC_PLLSOURCE_HSE;

    /* Set PLL1 M Coefficient */
    const RCC_PLLCKSELR_DIVM1_POS: u8 = 4;
    const RCC_PLLCKSELR_DIVM1: u32 = 0x3F << RCC_PLLCKSELR_DIVM1_POS;
    pllckselr &= !RCC_PLLCKSELR_DIVM1;
    pllckselr |= 4 << RCC_PLLCKSELR_DIVM1_POS; // Divide by 4
    unsafe { core::ptr::write_volatile(RCC_PLLCKSELR, pllckselr) };
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

    /* Set PLL1 N Coefficient */
    const RCC_PLL1DIVR: *mut u32 = (RCC_BASE + 0x030) as *mut u32;
    const RCC_PLL1DIVR_N1_POS: u8 = 0;
    const RCC_PLL1DIVR_N1: u32 = 0x1FF << RCC_PLL1DIVR_N1_POS;
    let mut pll1divr = unsafe { core::ptr::read_volatile(RCC_PLL1DIVR) };
    pll1divr &= !RCC_PLL1DIVR_N1;
    pll1divr |= (275 - 1) << RCC_PLL1DIVR_N1_POS; // Multiply by 275

    /* Set PLL1 Q Coefficient */
    const RCC_PLL1DIVR_Q1_POS: u8 = 16;
    const RCC_PLL1DIVR_Q1: u32 = 0x7F << RCC_PLL1DIVR_Q1_POS;
    pll1divr &= !RCC_PLL1DIVR_Q1;
    pll1divr |= (4 - 1) << RCC_PLL1DIVR_Q1_POS; // Divide by 4

    /* Set PLL1 R Coefficient */
    const RCC_PLL1DIVR_R1_POS: u8 = 24;
    const RCC_PLL1DIVR_R1: u32 = 0x7F << RCC_PLL1DIVR_R1_POS;
    pll1divr &= !RCC_PLL1DIVR_R1;
    pll1divr |= (2 - 1) << RCC_PLL1DIVR_R1_POS; // Divide by 2

    /* Set PLL1 P Coefficient */
    const RCC_PLL1DIVR_P1_POS: u8 = 9;
    const RCC_PLL1DIVR_P1: u32 = 0x7F << RCC_PLL1DIVR_P1_POS;
    pll1divr &= !RCC_PLL1DIVR_P1;
    pll1divr |= (1 - 1) << RCC_PLL1DIVR_P1_POS; // 0000000: pll1_p_ck = vco1_ck
    unsafe { core::ptr::write_volatile(RCC_PLL1DIVR, pll1divr) };
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

    /* Enable PLL1P and PLL1R */
    const RCC_PLLCFGR: *mut u32 = (RCC_BASE + 0x02C) as *mut u32;
    const RCC_PLLCFGR_DIVP1EN: u32 = 1 << 16;
    const RCC_PLLCFGR_DIVR1EN: u32 = 1 << 18;
    let mut pllcfgr = unsafe { core::ptr::read_volatile(RCC_PLLCFGR) };
    pllcfgr |= RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVR1EN;

    /* Set PLL1 VCO Input Range */
    const RCC_PLLCFGR_PLL1RGE: u32 = 0x3 << 2;
    const RCC_PLLINPUTRANGE_2_4: u32 = 0x1 << 2;
    pllcfgr &= !RCC_PLLCFGR_PLL1RGE;
    pllcfgr |= RCC_PLLINPUTRANGE_2_4;

    /* Set PLL1 VCO OutputRange: Wide VCO range: 192 to 836 MHz (default after reset) */
    const RCC_PLLCFGR_PLL1VCOSEL: u32 = 0x1 << 1;
    pllcfgr &= !RCC_PLLCFGR_PLL1VCOSEL;
    unsafe { core::ptr::write_volatile(RCC_PLLCFGR, pllcfgr) };
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

    /* Enable PLL1 */
    const RCC_CR_PLL1ON: u32 = 1 << 24;
    cr = unsafe { core::ptr::read_volatile(RCC_CR) };
    cr |= RCC_CR_PLL1ON;
    unsafe { core::ptr::write_volatile(RCC_CR, cr) }
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

    /* Wait till PLL is ready */
    const RCC_CR_PLL1RDY: u32 = 1 << 25;
    loop {
        cr = unsafe { core::ptr::read_volatile(RCC_CR) };
        if cr & RCC_CR_PLL1RDY != 0 {
            break;
        }
    }
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

    /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
    const RCC_D1CFGR: *mut u32 = (RCC_BASE + 0x018) as *mut u32;
    const RCC_D1CFGR_HPRE_POS: u8 = 0;
    const RCC_D1CFGR_HPRE: u32 = 0xF << RCC_D1CFGR_HPRE_POS;
    const RCC_D1CFGR_HPRE_DIV2: u32 = 0x1 << 3;
    let mut d1cfgr = unsafe { core::ptr::read_volatile(RCC_D1CFGR) };
    d1cfgr &= !RCC_D1CFGR_HPRE;
    d1cfgr |= RCC_D1CFGR_HPRE_DIV2;
    unsafe { core::ptr::write_volatile(RCC_D1CFGR, d1cfgr) };
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

    /* Configure PLL1 as the system clock source */
    const RCC_CFGR: *mut u32 = (RCC_BASE + 0x010) as *mut u32;
    const RCC_CFGR_SW: u32 = 0x7 << 0;
    const RCC_CFGR_SW_PLL1: u32 = 3;
    let mut cfgr = unsafe { core::ptr::read_volatile(RCC_CFGR) };
    cfgr &= !RCC_CFGR_SW;
    cfgr |= RCC_CFGR_SW_PLL1;
    unsafe { core::ptr::write_volatile(RCC_CFGR, cfgr) };
    core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

    /* Wait till System clock is ready */
    const RCC_CFGR_SWS: u32 = 0x7 << 3;
    const RCC_CFGR_SWS_PLL1: u32 = 0x00000018;
    loop {
        cfgr = unsafe { core::ptr::read_volatile(RCC_CFGR) };
        if cfgr & RCC_CFGR_SWS == RCC_CFGR_SWS_PLL1 {
            break;
        }
    }
}
