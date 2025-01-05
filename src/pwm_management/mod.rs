

use defmt::*;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::time::khz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel};
use embassy_time::Timer;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};
use log::info;

pub fn set_pwm_real<T>(ch: &mut SimplePwmChannel<T>, duty_cycle: f32)
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
{
  if !ch.is_enabled() {
    info!("Channel is not enabled!")
  }
  if duty_cycle < 0_f32 || duty_cycle > 1_f32 {
    info!("Wrong duty cycle value")
  }

  ch.set_duty_cycle(((ch.max_duty_cycle() as f32) * duty_cycle) as u16);
}