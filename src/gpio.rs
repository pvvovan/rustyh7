use crate::isr;

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

const GPIO_BASE: [u32; 3] = [0x5802_0000, 0x5802_0400, 0x5802_0800];

pub enum Port {
    _GpioA = 0,
    GpioB = 1,
    _GpioC = 2,
}

pub struct Gpio {
    base: u32,
}

static mut GPIOA_TAKEN: bool = false;
static mut GPIOB_TAKEN: bool = false;
static mut GPIOC_TAKEN: bool = false;

fn mark_taken(gpio_taken: &mut bool) {
    if *gpio_taken {
        panic!("GPIO taken");
    } else {
        *gpio_taken = true;
    }
}

const RCC_AHB4ENR: *mut u32 = (0x5802_4400 + 0x00E0) as *mut u32;
const RCC_AHB4ENR_GPIOAEN: u32 = 1 << 0;
const RCC_AHB4ENR_GPIOBEN: u32 = 1 << 1;
const RCC_AHB4ENR_GPIOCEN: u32 = 1 << 2;

unsafe fn set_bits(addr: *mut u32, bits: u32) {
    let mut val = core::ptr::read_volatile(addr);
    val = val | bits;
    core::ptr::write_volatile(addr, val);
}

const GPIO_BSRR: u32 = 0x18;

impl Gpio {
    pub fn take(port: Port, pins: &[Pin]) -> Self {
        let mut atomic = isr::lock();

        match port {
            Port::_GpioA => unsafe {
                mark_taken(&mut GPIOA_TAKEN);
                set_bits(RCC_AHB4ENR, RCC_AHB4ENR_GPIOAEN);
            },
            Port::GpioB => unsafe {
                mark_taken(&mut GPIOB_TAKEN);
                set_bits(RCC_AHB4ENR, RCC_AHB4ENR_GPIOBEN);
            },
            Port::_GpioC => unsafe {
                mark_taken(&mut GPIOC_TAKEN);
                set_bits(RCC_AHB4ENR, RCC_AHB4ENR_GPIOCEN);
            },
        }

        let gpio = Gpio {
            base: GPIO_BASE[port as usize],
        };
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        gpio.init(pins);

        atomic.unlock();
        return gpio;
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
