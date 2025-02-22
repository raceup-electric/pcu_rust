#![no_std]
#![no_main]

use defmt::{panic, *};
use embassy_executor::Spawner;
#[allow(unused)]
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::time::khz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

mod can_management;
mod pins;
mod pwm_management;

use crate::pwm_management::PwmDualController;
use can_management::{can_operation, messages, CanController};
use pins::Pins;


struct Pcu {
    pub can: &'static mut CanController<'static>,

    pub pwm_fanrad: &'static mut PwmDualController<'static, embassy_stm32::peripherals::TIM2, 1>,
    pub pwm_fanbatt: &'static mut PwmDualController<'static, embassy_stm32::peripherals::TIM3, 1>,
    pub pwm_pump: &'static mut PwmDualController<'static, embassy_stm32::peripherals::TIM4, 1>,

    pub fault_pumpl: &'static mut Input<'static>,
    pub fault_pumpr: &'static mut Input<'static>,
    pub fault_fanbattl: &'static mut Input<'static>,
    pub fault_fanbattr: &'static mut Input<'static>,
    pub fault_12v: &'static mut Input<'static>,
    pub fault_dv: &'static mut Input<'static>,
    pub fault_24v: &'static mut Input<'static>,

    pub enable_embedded: &'static mut Output<'static>,
    pub enable_dv: &'static mut Output<'static>,
    pub enable_12v: &'static mut Output<'static>,
    pub enable_24v: &'static mut Output<'static>,
    pub enable_brake: &'static mut Output<'static>,

    pub rf: &'static mut Output<'static>,
}


impl Pcu {
    fn handle_driver(&mut self, message : messages::Driver) {
        self.enable_brake.set_level(if message.brake() < 5 {Level::High} else {Level::Low});
    }

fn handle_pcu(&mut self, message :messages::Pcu) {
        match message.mode_raw() {
            val if val == messages::Pcu::MODE_MIN as u8 => {
                message.set_mode()
            },
            val if val == messages::Pcu::MODE_MIN + 1 as u8 => {
            },
            val if val == messages::Pcu::MODE_MAX as u8 => {
            }
            _ => (),
        }
    }
}


static CAN: StaticCell<CanController> = StaticCell::new();

static PWM_FANRAD: StaticCell<PwmDualController<'_, embassy_stm32::peripherals::TIM2, 1>> =
    StaticCell::new();
static PWM_FANBATT: StaticCell<PwmDualController<'_, embassy_stm32::peripherals::TIM3, 1>> =
    StaticCell::new();
static PWM_PUMP: StaticCell<PwmDualController<'_, embassy_stm32::peripherals::TIM4, 1>> =
    StaticCell::new();

static FAULT_PUMPR: StaticCell<embassy_stm32::gpio::Input> = StaticCell::new();
static FAULT_FANBATTL: StaticCell<embassy_stm32::gpio::Input> = StaticCell::new();
static FAULT_FANBATTR: StaticCell<embassy_stm32::gpio::Input> = StaticCell::new();
static FAULT_DV: StaticCell<embassy_stm32::gpio::Input> = StaticCell::new();
static FAULT_12V: StaticCell<embassy_stm32::gpio::Input> = StaticCell::new();
static FAULT_24V: StaticCell<embassy_stm32::gpio::Input> = StaticCell::new();

static ENABLE_EMBEDDED: StaticCell<embassy_stm32::gpio::Output> = StaticCell::new();
static ENABLE_DV: StaticCell<embassy_stm32::gpio::Output> = StaticCell::new();
static ENABLE_12V: StaticCell<embassy_stm32::gpio::Output> = StaticCell::new();
static ENABLE_24V: StaticCell<embassy_stm32::gpio::Output> = StaticCell::new();
static ENABLE_BRAKE: StaticCell<embassy_stm32::gpio::Output> = StaticCell::new();

static RF: StaticCell<embassy_stm32::gpio::Output> = StaticCell::new();

static PCU: StaticCell<Mutex<CriticalSectionRawMutex, Pcu>> = StaticCell::new();

fn pwm_enable_check(a: &[bool; 1]) -> bool {
    a.iter().all(|&e| e)
}



#[embassy_executor::main]
async fn main(spawner: Spawner) {

static FAULT_PUMPL: StaticCell<embassy_stm32::gpio::Input> = StaticCell::new();
    let p = embassy_stm32::init(Default::default());
    let pins = Pins::new(p);

    let mut _ef = Output::new(pins.pin_ef, Level::High, Speed::Low);

    let can = CanController::new_can2(
        pins.can_2,
        pins.pin_can_rx,
        pins.pin_can_tx,
        500_000,
        pins.can_1,
        pins.pin_usb_minus,
        pins.pin_usb_plus,
    )
    .await;

    let fanradl = PwmPin::new_ch3(pins.pin_fanradl_pwm, OutputType::PushPull);
    let fanradr = PwmPin::new_ch4(pins.pin_fanradr_pwm, OutputType::PushPull);
    let fanrad_pwm_driver = SimplePwm::new(
        pins.timer_fanrad,
        None,
        None,
        Some(fanradl),
        Some(fanradr),
        khz(25),
        Default::default(),
    );
    let fanrad_pwm_channels = fanrad_pwm_driver.split();
    let mut fanradl_pwm_ch = fanrad_pwm_channels.ch3;
    let mut fanradr_pwm_ch = fanrad_pwm_channels.ch4;

    let fanbattl = PwmPin::new_ch2(pins.pin_fanbattl_pwm, OutputType::PushPull);
    let fanbattr = PwmPin::new_ch1(pins.pin_fanbattr_pwm, OutputType::PushPull);
    let fanbatt_pwm_driver = SimplePwm::new(
        pins.timer_fanbatt,
        Some(fanbattr),
        Some(fanbattl),
        None,
        None,
        khz(25),
        Default::default(),
    );
    let fanbatt_pwm_channels = fanbatt_pwm_driver.split();
    let mut fanbattr_pwm_ch = fanbatt_pwm_channels.ch1;
    let mut fanbattl_pwm_ch = fanbatt_pwm_channels.ch2;

    let pumpl = PwmPin::new_ch2(pins.pin_pumpl_pwm, OutputType::PushPull);
    let pumpr = PwmPin::new_ch1(pins.pin_pumpr_pwm, OutputType::PushPull);
    let pump_pwm_driver = SimplePwm::new(
        pins.timer_pump,
        Some(pumpr),
        Some(pumpl),
        None,
        None,
        khz(25),
        Default::default(),
    );
    let pump_pwm_channels = pump_pwm_driver.split();
    let mut pumpr_pwm_ch = pump_pwm_channels.ch1;
    let mut pumpl_pwm_ch = pump_pwm_channels.ch2;

    let pwm_fanrad: PwmDualController<'_, embassy_stm32::peripherals::TIM2, 1> =
        PwmDualController::new(
            fanradl_pwm_ch,
            fanradr_pwm_ch,
            &pwm_enable_check,
            Output::new(pins.pin_fanradl_enable, Level::Low, Speed::Low),
            Output::new(pins.pin_fanradr_enable, Level::Low, Speed::Low),
        );

    let pwm_fanbatt: PwmDualController<'_, embassy_stm32::peripherals::TIM3, 1> =
        PwmDualController::new(
            fanbattl_pwm_ch,
            fanbattr_pwm_ch,
            &pwm_enable_check,
            Output::new(pins.pin_fanbattl_enable, Level::Low, Speed::Low),
            Output::new(pins.pin_fanbattr_enable, Level::Low, Speed::Low),
        );

    let pwm_pump: PwmDualController<'_, embassy_stm32::peripherals::TIM4, 1> =
        PwmDualController::new(
            pumpr_pwm_ch,
            pumpl_pwm_ch,
            &pwm_enable_check,
            Output::new(pins.pin_pumpl_enable, Level::Low, Speed::Low),
            Output::new(pins.pin_pumpr_enable, Level::Low, Speed::Low),
        );

    let can = StaticCell::init(&CAN, can);

    let pwm_fanrad = StaticCell::init(&PWM_FANRAD, pwm_fanrad);
    let pwm_fanbatt = StaticCell::init(&PWM_FANBATT, pwm_fanbatt);
    let pwm_pump = StaticCell::init(&PWM_PUMP, pwm_pump);

    let fault_pumpl = StaticCell::init(&FAULT_PUMPL, Input::new(pins.pin_pumpl_fault, Pull::None));
    let fault_pumpr = StaticCell::init(&FAULT_PUMPR, Input::new(pins.pin_pumpr_fault, Pull::None));
    let fault_fanbattl = StaticCell::init(
        &FAULT_FANBATTL,
        Input::new(pins.pin_fanbattl_fault, Pull::None),
    );
    let fault_fanbattr = StaticCell::init(
        &FAULT_FANBATTR,
        Input::new(pins.pin_fanbattr_fault, Pull::None),
    );
    let fault_12v = StaticCell::init(&FAULT_12V, Input::new(pins.pin_12v_fault, Pull::None));
    let fault_dv = StaticCell::init(&FAULT_DV, Input::new(pins.pin_dv_fault, Pull::None));
    let fault_24v = StaticCell::init(&FAULT_24V, Input::new(pins.pin_24v_fault, Pull::None));

    let enable_embedded = StaticCell::init(
        &ENABLE_EMBEDDED,
        Output::new(pins.pin_embedded_enable, Level::Low, Speed::Low),
    );
    let enable_12v = StaticCell::init(
        &ENABLE_12V,
        Output::new(pins.pin_12v_enable, Level::High, Speed::Low),
    );
    let enable_dv = StaticCell::init(
        &ENABLE_DV,
        Output::new(pins.pin_dv_enable, Level::Low, Speed::Low),
    );
    let enable_24v = StaticCell::init(
        &ENABLE_DV,
        Output::new(pins.pin_24v_enable, Level::High, Speed::Low),
    );
    let enable_brake = StaticCell::init(
        &ENABLE_BRAKE,
        Output::new(pins.pin_brake_enable, Level::Low, Speed::High),
    );

    let rf = StaticCell::init(&RF, Output::new(pins.pin_rf, Level::Low, Speed::Low));

    let pcu = Pcu {
        can,
        pwm_fanrad,
        pwm_fanbatt,
        pwm_pump,
        fault_fanbattl,
        fault_fanbattr,
        enable_dv,
        enable_24v,
        enable_brake,
        fault_pumpl,
        fault_pumpr,
        enable_12v,
        enable_embedded,
        fault_12v,
        fault_24v,
        fault_dv,
        rf,
    };

    let pcu = StaticCell::init(&PCU, Mutex::new(pcu));

    spawner.spawn(can_reader(pcu)).unwrap();
    spawner.spawn(fault_detection(pcu)).unwrap();
}

#[embassy_executor::task]
async fn can_reader(pcu_mutex: &'static Mutex<CriticalSectionRawMutex, Pcu>) {
    loop {
    let mut pcu = pcu_mutex.lock().await;
    match pcu.can.read().await{
        Ok(message) => {
            match message.id() {
                val if val == messages::Pcu::MESSAGE_ID as u16 => {
                    pcu.handle_pcu(message._bytes());
                }
                val if val == messages::Driver::MESSAGE_ID as u16 => {
                        pcu.handle_driver(message._bytes());
                }
                _ => (),
            }
            drop(pcu);
        },
        Err(_) => {
            drop(pcu);
            embassy_time::Timer::after_millis(20).await;
        }
    };
    }
}

#[embassy_executor::task]
async fn fault_detection(pcu_mutex: &'static Mutex<CriticalSectionRawMutex, Pcu>) {
    loop {
    let mut pcu = pcu_mutex.lock().await;
    let mut message = match messages::PcuFault::new(false, false, false, false, false, false, false)
    {
        Ok(res) => res,
        Err(_) => return,
    };

    if pcu.fault_24v.is_high() {
        let _ = message.set_fault_24v(true);
        pcu.enable_24v.set_low();
    }
    if pcu.fault_12v.is_high() {
        let _ = message.set_fault_12v(true);
        pcu.enable_12v.set_low();
    }
    if pcu.fault_dv.is_high() {
        let _ = message.set_fault_dv(true);
        pcu.enable_dv.set_low();
    }
    if pcu.fault_pumpl.is_high() {
        let _ = message.set_fault_pumpl(true);
        pcu.pwm_pump.set_duty_left(0);
    }
    if pcu.fault_pumpr.is_high() {
        let _ = message.set_fault_pumpr(true);
        pcu.pwm_pump.set_duty_right(0);
    }
    if pcu.fault_fanbattl.is_high() {
        let _ = message.set_fault_fanbattl(true);
        pcu.pwm_fanbatt.set_duty_left(0);
    }
    if pcu.fault_fanbattr.is_high() {
        let _ = message.set_fault_fanbattr(true);
        pcu.pwm_fanbatt.set_duty_right(0);
    }
    can_operation(
        messages::PcuFault::MESSAGE_ID as u16,
        &[message.raw()[0], 0, 0, 0, 0, 0, 0, 0],
        &mut pcu.can,
    ).await;
    embassy_time::Timer::after_secs(5).await;
    }
}

fn scale_f32_to_u16(val: f32) -> u16 {
    let scaled = (val / 100.0) * (u16::MAX as f32);
    scaled.clamp(0.0, u16::MAX as f32) as u16
}
