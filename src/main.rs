#![no_std]
#![no_main]

use core::ops::Index;

use embassy_executor::{Executor, Spawner};
use embassy_futures::select::*;
use embassy_stm32::{
    adc::Adc,
    can::{filter::*, CanRx, CanTx, Fifo, StandardId},
    gpio::{Input, Level, Output, OutputType, Pull, Speed},
    time::hz,
    timer::simple_pwm::{PwmPin, SimplePwm},
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, signal::Signal,
};
use embassy_time::Duration;

use defmt::*;
use static_cell::StaticCell;

mod can_management;
mod pins;
mod pwm_management;
use crate::can_management::{messages as can_2, CanController, CanFrame};
use crate::pins::{AssignedResources, Brake, Can, Enables, Faults, Pwm, Senses, Sw, Usb};
use crate::pwm_management::PwmDualController;

static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

static CAN_PWM_CHANNEL: Signal<CriticalSectionRawMutex, can_2::PcuModeM0> = Signal::new();
static CAN_RF_CHANNEL: Signal<CriticalSectionRawMutex, can_2::PcuModeM1> = Signal::new();
static CAN_ENABLES_CHANNEL: Signal<CriticalSectionRawMutex, can_2::PcuModeM2> = Signal::new();
static CAN_DRIVER_CHANNEL: Signal<CriticalSectionRawMutex, bool> = Signal::new();

static CAN_WRITER: Channel<CriticalSectionRawMutex, (u16, [u8; 8]), 20> = Channel::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let r = split_resources!(p);

    let can_peripherals = r.can;
    let usb_peripherals = r.usb;
    let mut can = CanController::new_can2(
        can_peripherals.can_2,
        can_peripherals.can_rx,
        can_peripherals.can_tx,
        500_000,
        can_peripherals.can_1,
        usb_peripherals.usb_minus,
        usb_peripherals.usb_plus,
    )
    .await;
    can.can.modify_filters().enable_bank(
        0,
        Fifo::Fifo0,
        BankConfig::List16([
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(
                can_2::Driver::MESSAGE_ID as u16
            ))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
        ]),
    );
    can.can.modify_filters().enable_bank(
        0,
        Fifo::Fifo1,
        BankConfig::List16([
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(
                can_2::Pcu::MESSAGE_ID as u16
            ))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
        ]),
    );

    let (_can_tx, _can_rx) = can.can.split();

    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(task_senses(r.senses)));
        unwrap!(spawner.spawn(task_enables(r.enables)));
        unwrap!(spawner.spawn(pwm(r.pwm)));
        unwrap!(spawner.spawn(read_can(_can_rx)));
        unwrap!(spawner.spawn(write_can(_can_tx)));
        unwrap!(spawner.spawn(fault_detection(r.faults)));
    });
}

fn pwm_enable_check(a: &[bool; 1]) -> bool {
    a.iter().all(|&e| e)
}

fn percent_to_duty(val: u8) -> u16 {
    (val as u32 * 65_535 / 100) as u16
}

#[embassy_executor::task]
async fn pwm(pins: Pwm) {
    let fanradl = PwmPin::new_ch4(pins.pwm_fanradl, OutputType::PushPull);
    let fanradr = PwmPin::new_ch3(pins.pwm_fanradr, OutputType::PushPull);
    let fanrad_pwm_driver = SimplePwm::new(
        pins.timer_fanrad,
        None,
        None,
        Some(fanradr),
        Some(fanradl),
        hz(25_000),
        Default::default(),
    );
    let fanrad_pwm_channels = fanrad_pwm_driver.split();
    let fanradl_pwm_ch = fanrad_pwm_channels.ch3;
    let fanradr_pwm_ch = fanrad_pwm_channels.ch4;

    let fanbattl = PwmPin::new_ch2(pins.pwm_fanbattl, OutputType::PushPull);
    let fanbattr = PwmPin::new_ch1(pins.pwm_fanbattr, OutputType::PushPull);
    let fanbatt_pwm_driver = SimplePwm::new(
        pins.timer_fanbatt,
        Some(fanbattr),
        Some(fanbattl),
        None,
        None,
        hz(25_000),
        Default::default(),
    );
    let fanbatt_pwm_channels = fanbatt_pwm_driver.split();
    let fanbattr_pwm_ch = fanbatt_pwm_channels.ch1;
    let fanbattl_pwm_ch = fanbatt_pwm_channels.ch2;

    let pumpl = PwmPin::new_ch2(pins.pwm_pumpl, OutputType::PushPull);
    let pumpr = PwmPin::new_ch1(pins.pwm_pumpr, OutputType::PushPull);
    let pump_pwm_driver = SimplePwm::new(
        pins.timer_pump,
        Some(pumpr),
        Some(pumpl),
        None,
        None,
        hz(25_000),
        Default::default(),
    );
    let pump_pwm_channels = pump_pwm_driver.split();
    let pumpr_pwm_ch = pump_pwm_channels.ch1;
    let pumpl_pwm_ch = pump_pwm_channels.ch2;

    let mut pwm_fanrad: PwmDualController<'_, embassy_stm32::peripherals::TIM2, 1> =
        PwmDualController::new(
            fanradl_pwm_ch,
            fanradr_pwm_ch,
            &pwm_enable_check,
            Output::new(pins.enable_fanradl, Level::Low, Speed::Low),
            Output::new(pins.enable_fanradr, Level::Low, Speed::Low),
        );

    let mut pwm_fanbatt: PwmDualController<'_, embassy_stm32::peripherals::TIM3, 1> =
        PwmDualController::new(
            fanbattl_pwm_ch,
            fanbattr_pwm_ch,
            &pwm_enable_check,
            Output::new(pins.enable_fanbattl, Level::Low, Speed::Low),
            Output::new(pins.enable_fanbattr, Level::Low, Speed::Low),
        );

    let mut pwm_pump: PwmDualController<'_, embassy_stm32::peripherals::TIM4, 1> =
        PwmDualController::new(
            pumpr_pwm_ch,
            pumpl_pwm_ch,
            &pwm_enable_check,
            Output::new(pins.enable_pumpl, Level::Low, Speed::Low),
            Output::new(pins.enable_pumpr, Level::Low, Speed::Low),
        );

    loop {
        let message = CAN_PWM_CHANNEL.wait().await;
        // radiator fans
        pwm_fanrad.set_duty_left(percent_to_duty(message.fanrad_speed_left()));
        pwm_fanrad.set_duty_right(percent_to_duty(message.fanrad_speed_right()));
        pwm_fanrad.set_level(
            0,
            message.fanrad_enable_left() || message.fanrad_enable_right(),
        );
        // battery fans
        pwm_fanbatt.set_duty_left(percent_to_duty(message.fanbatt_speed_left()));
        pwm_fanbatt.set_duty_right(percent_to_duty(message.fanbatt_speed_right()));
        pwm_fanbatt.set_level(
            0,
            message.fanbatt_enable_left() || message.fanbatt_enable_right(),
        );
        // pump
        pwm_pump.set_duty_left(percent_to_duty(message.pump_speed_left()));
        pwm_pump.set_duty_right(percent_to_duty(message.pump_speed_right()));
        pwm_pump.set_level(
            0,
            message.pump_enable_left() || message.pump_enable_right(),
        );
    }
}

#[embassy_executor::task]
async fn task_enables(enables: Enables) {
    let _enable_ef = Output::new(enables.enable_ef, Level::High, Speed::Low);
    let mut enabe_rf = Output::new(enables.enable_rf, Level::Low, Speed::Low);
    let mut enable_emb = Output::new(enables.enable_emb, Level::Low, Speed::Low);
    let mut enable_dv = Output::new(enables.enable_dv, Level::Low, Speed::Low);
    let _enable_24v = Output::new(enables.enable_24v, Level::High, Speed::Low);
    match select(CAN_RF_CHANNEL.wait(), CAN_ENABLES_CHANNEL.wait()).await {
        Either::First(mes) => {
            enabe_rf.set_level(if mes.rf() { Level::High } else { Level::Low });
            let message = can_2::PcuRfAck::new(enabe_rf.is_set_high());
            if let Ok(some) = message {
                CAN_WRITER.send((
                    can_2::PcuRfAck::MESSAGE_ID as u16,
                    pad_array::<1, 8>(some.raw(), 0),
                )).await;
            }
        }
        Either::Second(mes) => {
            enable_emb.set_level(if mes.enable_embedded() {
                Level::High
            } else {
                Level::Low
            });
            enable_dv.set_level(if mes.enable_embedded() {
                Level::High
            } else {
                Level::Low
            });
        }
    }
}

#[repr(usize)]
enum FaultEnum {
    V12 = 0,
    Dv = 1,
    V24 = 2,
    Pumpl = 3,
    Pumpr = 4,
    Fanbattl = 5,
    Fanbattr = 6,
}

impl<T> Index<FaultEnum> for [T] {
    type Output = T;

    fn index(&self, idx: FaultEnum) -> &Self::Output {
        &self[idx as usize]
    }
}

//fn update_fault_state(pin: &ExtiInput, state: &mut bool) {
//    if pin.is_high() {
//        *state = true;
//    } else {
//        *state = false;
//    }
//}

fn pad_array<const N: usize, const P: usize>(input: &[u8; N], pad_value: u8) -> [u8; P] {
    let mut output = [pad_value; P];

    let len = N.min(P);
    (0..len).for_each(|i| {
        output[i] = input[i];
    });

    output
}

#[embassy_executor::task]
async fn fault_detection(faults: Faults) {
    let faults = [
        Input::new(faults.fault_dv, Pull::Down),
        Input::new(faults.fault_24v, Pull::Down),
        Input::new(faults.fault_pumpl, Pull::Down),
        Input::new(faults.fault_pumpr, Pull::Down),
        Input::new(faults.fault_fanbattl, Pull::Down),
        Input::new(faults.fault_fanbattr, Pull::Down),
    ];

    let mut ticker = embassy_time::Ticker::every(Duration::from_millis(5000));

    loop {
        ticker.next().await;
        let message = can_2::PcuFault::new(
            faults[FaultEnum::V12].is_high(),
            faults[FaultEnum::Dv].is_high(),
            faults[FaultEnum::V24].is_high(),
            faults[FaultEnum::Pumpl].is_high(),
            faults[FaultEnum::Pumpr].is_high(),
            faults[FaultEnum::Fanbattl].is_high(),
            faults[FaultEnum::Fanbattr].is_high(),
        );
        if let Ok(mes) = message {
            CAN_WRITER.send((
                can_2::PcuFault::MESSAGE_ID as u16,
                pad_array::<1, 8>(mes.raw(), 0),
            )).await;
        }
    }
}

fn switch_texas(val: u16) -> f32 {
    (val as f32) * 3.3 * 1300.0 / (680.0 * 4096.0)
}

fn switch_infineon(val: u16) -> f32 {
    (val as f32) * 3.3 * 52000.0 / (4096.0 * 2000.0)
}

#[embassy_executor::task]
async fn task_senses(mut senses: Senses) {
    let mut adc_1 = Adc::new(senses.adc_1);
    let mut adc_2 = Adc::new(senses.adc_2);
    let mut adc_3 = Adc::new(senses.adc_3);

    loop {
        let _val_dv = switch_texas(adc_1.blocking_read(&mut senses.sense_dv));
        let _val_24v = switch_texas(adc_1.blocking_read(&mut senses.sense_24v));
        let _val_pumpl = switch_texas(adc_1.blocking_read(&mut senses.sense_pumpl));
        let _val_pumpr = switch_texas(adc_1.blocking_read(&mut senses.sense_pumpr));
        let _val_fanbattl = switch_texas(adc_2.blocking_read(&mut senses.sense_fanbattl));
        let _val_fanbattr = switch_texas(adc_2.blocking_read(&mut senses.sense_fanbattr));
        let _val_fanradl = switch_infineon(adc_2.blocking_read(&mut senses.sense_fanradl));
        let _val_fanradr = switch_infineon(adc_3.blocking_read(&mut senses.sense_fanradr));
        let _val_emb = switch_infineon(adc_3.blocking_read(&mut senses.sense_emb));
        let _val_steeract = switch_infineon(adc_2.blocking_read(&mut senses.steeract_sense));
        embassy_time::Timer::after_millis(5000).await;
    }
}

#[embassy_executor::task]
async fn read_can(mut rx: CanRx<'static>) {
    loop {
        let envelope = unwrap!(rx.read().await);
        let frame = CanFrame::from_envelope(envelope);
        let message =
            can_2::Messages::from_can_message(frame.id().into(), frame._bytes().as_slice()).ok();
        if message.is_some() {
            let message = unwrap!(message);
            match message {
                can_2::Messages::Pcu(mes) => match mes.mode_raw() {
                    0 => {
                        CAN_PWM_CHANNEL.signal(can_2::PcuModeM0::new_from_raw(*mes.raw()));
                    }
                    1 => {
                        CAN_RF_CHANNEL.signal(can_2::PcuModeM1::new_from_raw(*mes.raw()));
                    }
                    2 => {
                        CAN_ENABLES_CHANNEL.signal(can_2::PcuModeM2::new_from_raw(*mes.raw()));
                    }
                    _ => {}
                },
                can_2::Messages::Driver(mes) => {
                    CAN_DRIVER_CHANNEL.signal(mes.brake() > 5);
                }
                _ => {}
            }
        }
    }
}

#[embassy_executor::task]
async fn write_can(mut tx: CanTx<'static>) {
    loop {
        let (id, mes) = CAN_WRITER.receive().await;
        let message = embassy_stm32::can::Frame::new_standard(id, &mes);
        if let Ok(some) = message {
            tx.write(&some).await;
        }
    }
}
