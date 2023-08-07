pub enum Pin {
    Pin0 = 0,
    _Pin1 = 1,
    _Pin2 = 2,
    _Pin3 = 3,
    _Pin4 = 4,
    _Pin5 = 5,
    _Pin6 = 6,
    _Pin7 = 7,
    _Pin8 = 8,
    _Pin9 = 9,
    _Pin10 = 10,
    _Pin11 = 11,
    _Pin12 = 12,
    _Pin13 = 13,
    Pin14 = 14,
    _Pin15 = 15,
}
impl Copy for Pin {}
impl Clone for Pin {
    fn clone(&self) -> Self {
        *self
    }
}

const GPIO_SIZE: usize = 3;
const GPIO_BASE: [u32; GPIO_SIZE] = [0x5802_0000, 0x5802_0400, 0x5802_0800];

#[derive(Clone)]
pub enum Port {
    _GpioA = 0,
    GpioB = 1,
    _GpioC = 2,
}

pub struct Gpio {
    base: u32,
}

static mut GPIO_TAKEN: [u8; GPIO_SIZE] = [0; GPIO_SIZE];

unsafe fn set_bits(addr: *mut u32, bits: u32) {
    let mut val = core::ptr::read_volatile(addr);
    val = val | bits;
    core::ptr::write_volatile(addr, val);
}

const GPIO_BSRR: u32 = 0x18;

impl Gpio {
    pub fn take(port: Port, pins: &[Pin]) -> Option<Self> {
        let mut taken: u8;
        unsafe {
            core::arch::asm!(
                "LDREXB {flag}, [{addr}]",
                addr = in(reg) &GPIO_TAKEN[port.clone() as usize],
                flag = out(reg) taken
            );
        }
        if taken != 0 {
            unsafe {
                core::arch::asm!("CLREX");
            }
            return None;
        }

        taken = 1;
        let err: u32;
        unsafe {
            core::arch::asm!(
                "STREXB {err}, {flag}, [{addr}]",
                err = out(reg) err,
                addr = in(reg) &GPIO_TAKEN[port.clone() as usize],
                flag = in(reg) taken
            );
        }
        if err != 0 {
            return None;
        }
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

        const RCC_AHB4ENR: *mut u32 = (0x5802_4400 + 0x00E0) as *mut u32;
        const RCC_AHB4ENR_GPIOAEN: u32 = 1 << 0;
        const RCC_AHB4ENR_GPIOBEN: u32 = 1 << 1;
        const RCC_AHB4ENR_GPIOCEN: u32 = 1 << 2;
        match port {
            Port::_GpioA => unsafe {
                set_bits(RCC_AHB4ENR, RCC_AHB4ENR_GPIOAEN);
            },
            Port::GpioB => unsafe {
                set_bits(RCC_AHB4ENR, RCC_AHB4ENR_GPIOBEN);
            },
            Port::_GpioC => unsafe {
                set_bits(RCC_AHB4ENR, RCC_AHB4ENR_GPIOCEN);
            },
        }
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);

        let gpio = Gpio {
            base: GPIO_BASE[port as usize],
        };
        gpio.init(pins);

        return Some(gpio);
    }

    pub fn set(&self, pins: &[Pin]) {
        let gpio_bsrr: *mut u32 = (self.base + GPIO_BSRR) as *mut u32;
        let mut set_mask = 0u32;
        for pin in pins {
            set_mask = set_mask | 1u32 << *pin as u8;
        }
        unsafe {
            core::ptr::write_volatile(gpio_bsrr, set_mask);
        }
    }

    pub fn reset(&self, pins: &[Pin]) {
        let gpio_bsrr: *mut u32 = (self.base + GPIO_BSRR) as *mut u32;
        let mut reset_mask = 0u32;
        for pin in pins {
            reset_mask = reset_mask | 1u32 << (*pin as u8 + 16);
        }
        unsafe {
            core::ptr::write_volatile(gpio_bsrr, reset_mask);
        }
    }

    fn init(&self, pins: &[Pin]) {
        let mut mode_mask = 0u32;
        for pin in pins {
            mode_mask = mode_mask | (1u32 << (*pin as u8) * 2);
        }

        let moder: *mut u32 = self.base as *mut u32;
        unsafe {
            core::ptr::write_volatile(moder, mode_mask);
        }
    }
}
