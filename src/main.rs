#![no_std]
#![no_main]

use core::str::from_utf8;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed, OutputType,};
use embassy_stm32::time::khz;
use embassy_time::Timer;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
use defmt::*;
use embassy_stm32::interrupt::typelevel::TIM2;
use {defmt_rtt as _, panic_probe as _};

mod types;
mod can_management;

mod pwm_management;

use types::{CanMsg};
use can_management::{CanController, can_operation};
use crate::pwm_management::set_pwm_real;

static CAN: StaticCell<Mutex<CriticalSectionRawMutex, CanController>> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = embassy_stm32::init(Default::default());

    let can_rx_pin = Input::new(&mut p.PB12, Pull::Up);
    core::mem::forget(can_rx_pin);

    let can = CanController::new_can2(p.CAN2, p.PB12, p.PB13, 500_000, p.CAN1, p.PA11, p.PA12).await;
    let can_mutex = Mutex::new(can);
    let can = StaticCell::init(&CAN, can_mutex);
    spawner.spawn(read_can(can)).unwrap();

    let ch1_pin = PwmPin::new_ch1(p.PA0, OutputType::PushPull);
    let mut pwm = SimplePwm::new(p.TIM2, Some(ch1_pin), None, None, None, khz(25), Default::default());
    let mut ch1 = pwm.ch1();
    ch1.enable();


    set_pwm_real(&mut ch1 , 0.5);
    set_pwm_real(&mut ch1 , 0_f32);
    set_pwm_real(&mut ch1 , 1_f32);

    loop {
        embassy_time::Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
async fn read_can(
    can: &'static Mutex<CriticalSectionRawMutex, CanController<'static>>
){
    loop {
        let mut can_data = can.lock().await;
        match can_data.read().await {
            Ok(frame) => info!("Out: {}", frame.byte(0)),
            Err(_) => info!("No messages"),
        }
        drop(can_data);
        embassy_time::Timer::after_millis(100).await;
    }
}
