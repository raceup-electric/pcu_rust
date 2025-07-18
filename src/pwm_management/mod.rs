use embassy_stm32::timer::simple_pwm::SimplePwmChannel;
use {defmt_rtt as _, panic_probe as _};

pub struct PwmDualController<'a, T, const N: usize>
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
{
    left: SimplePwmChannel<'a, T>,
    right: SimplePwmChannel<'a, T>,
    duty_left: u16,
    duty_right: u16,
    enablement: [bool; N],
    enable_check: &'a dyn Fn(&[bool; N]) -> bool,
    enabled: bool,
    debug_pin_left: embassy_stm32::gpio::Output<'a>,
    debug_pin_right: embassy_stm32::gpio::Output<'a>,
}

impl<'a, T, const N: usize> PwmDualController<'a, T, N>
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
{
    pub fn new(
        left: SimplePwmChannel<'a, T>,
        right: SimplePwmChannel<'a, T>,
        enable_check: &'a dyn Fn(&[bool; N]) -> bool,
        debug_pin_left: embassy_stm32::gpio::Output<'a>,
        debug_pin_right: embassy_stm32::gpio::Output<'a>,
    ) -> Self {
        let enablement = [false; N];
        let enabled = enable_check(&enablement);

        Self {
            left,
            right,
            duty_left: 0,
            duty_right: 0,
            enablement,
            enable_check,
            enabled,
            debug_pin_left,
            debug_pin_right,
        }
    }

    pub fn set_duty_right(&mut self, duty_cycle: u16) {
        self.right
            .set_duty_cycle_fraction(duty_cycle, 0_u16.wrapping_sub(1));
        self.duty_right = duty_cycle;
        self.update_debug_pin();
    }

    pub fn set_duty_left(&mut self, duty_cycle: u16) {
        self.left
            .set_duty_cycle_fraction(duty_cycle, 0_u16.wrapping_sub(1));
        self.duty_left = duty_cycle;
        self.update_debug_pin()
    }

    pub fn set_duty(&mut self, duty_cycle: u16) {
        self.set_duty_left(duty_cycle);
        self.set_duty_right(duty_cycle);
    }

    #[allow(dead_code)]
    pub fn enable(&mut self, level: usize) {
        self.enablement[level] = true;
        self.update_enabled();
    }

    #[allow(dead_code)]
    pub fn disable(&mut self, level: usize) {
        self.enablement[level] = false;
        self.update_enabled();
    }

    pub fn set_level(&mut self, level: usize, state: bool) {
        self.enablement[level] = state;
        self.update_enabled();
    }

    fn update_enabled(&mut self) {
        if self.enabled != (self.enable_check)(&self.enablement) {
            self.enabled ^= true;
            match self.enabled {
                true => {
                    self.left.enable();
                    self.right.enable();
                }
                false => {
                    self.left.disable();
                    self.right.disable();
                }
            }
            self.update_debug_pin();
        }
    }

    fn update_debug_pin(&mut self) {
        match self.enabled && self.duty_left != 0 {
            true => self.debug_pin_left.set_high(),
            false => self.debug_pin_left.set_low(),
        }
        match self.enabled && self.duty_right != 0 {
            true => self.debug_pin_right.set_high(),
            false => self.debug_pin_right.set_low(),
        }
    }

    pub fn duty_left(&self) -> u16 {
        self.duty_left
    }

    pub fn duty_right(&self) -> u16 {
        self.duty_right
    }
}
