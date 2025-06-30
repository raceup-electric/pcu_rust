use core::{cmp::min, sync::atomic::{AtomicBool, Ordering}};

use embassy_stm32::timer::simple_pwm::SimplePwmChannel;
use embassy_time::Duration;
use {defmt_rtt as _, panic_probe as _};

static IS_POWER_INCREASING : AtomicBool = AtomicBool::new(false);

pub struct PwmDualController<'a, T, const N: usize>
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
{
    left: SimplePwmChannel<'a, T>,
    right: SimplePwmChannel<'a, T>,
    duty: u16,
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
            duty: 0,
            enablement,
            enable_check,
            enabled,
            debug_pin_left,
            debug_pin_right,
        }
    }

    fn set_duty_right(&mut self, duty_cycle: u16) {
        self.right
            .set_duty_cycle_fraction(duty_cycle, 0_u16.wrapping_sub(1));
        self.update_debug_pin();
    }

    fn set_duty_left(&mut self, duty_cycle: u16) {
        self.left
            .set_duty_cycle_fraction(duty_cycle, 0_u16.wrapping_sub(1));
        self.update_debug_pin()
    }

    #[allow(dead_code)]
    pub async fn set_duty(&mut self, duty_cycle: u16) {
        if duty_cycle < self.duty
        {
            self.set_duty_left(duty_cycle);
            self.set_duty_right(duty_cycle);
            self.duty = duty_cycle;
        }
        else
        {
            let mut ticker = embassy_time::Ticker::every(Duration::from_millis(200));
            loop {
                if self.duty == duty_cycle
                {
                    self.duty = duty_cycle;
                    IS_POWER_INCREASING.store(false, Ordering::SeqCst); //INFO: come su c++20
                    return ;
                }
                else if self.duty > duty_cycle
                {
                    self.set_duty_left(duty_cycle);
                    self.set_duty_right(duty_cycle);
                    self.duty = duty_cycle;
                    IS_POWER_INCREASING.store(false, Ordering::SeqCst);
                    return ;
                }
                ticker.next().await;
                match IS_POWER_INCREASING.swap(true, Ordering::SeqCst) {
                    true => {},
                    false => {
                        self.duty = min(duty_cycle, self.duty.saturating_add(13107)); //20% al tick
                        self.set_duty_left(self.duty);
                        self.set_duty_left(self.duty);
                    },
                }
            }
        }
        return ;
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
        match self.enabled && self.duty != 0 {
            true => self.debug_pin_left.set_high(),
            false => self.debug_pin_left.set_low(),
        }
        match self.enabled && self.duty != 0 {
            true => self.debug_pin_right.set_high(),
            false => self.debug_pin_right.set_low(),
        }
    }
}
