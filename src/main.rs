#![no_std]
#![no_main]

use core::{
    cmp::{max, min},
    ops::Index,
};

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
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex, signal::Signal,
};
use embassy_time::{Duration, Ticker};

use defmt::*;
use embedded_hal::blocking::can;
use static_cell::StaticCell;

mod can_management;
mod pins;
mod pwm_management;
use crate::can_management::{
    messages::{self as can_2, BmsLvEmergency},
    CanController, CanFrame,
};
use crate::pins::*;
use crate::pwm_management::PwmDualController;

static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

static CAN_PWM_CHANNEL: Signal<CriticalSectionRawMutex, can_2::CoolingControl> = Signal::new();
static CAN_BMS_EMERGENCY_CHANNEL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
static CAN_MISSIONSTATUS_CHANNEL: Signal<CriticalSectionRawMutex, can_2::CarStatus> = Signal::new();

static CAN_RF_CHANNEL: Signal<CriticalSectionRawMutex, can_2::PcuModeM1> = Signal::new();
static CAN_ENABLES_CHANNEL: Signal<CriticalSectionRawMutex, can_2::PcuModeM2> = Signal::new();
static CAN_DRIVER_CHANNEL: Signal<CriticalSectionRawMutex, bool> = Signal::new();

static CAN_WRITER: Channel<CriticalSectionRawMutex, (u16, usize, [u8; 8]), 20> = Channel::new();

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
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(
                can_2::CoolingControl::MESSAGE_ID as u16
            ))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(
                can_2::BmsLvEmergency::MESSAGE_ID as u16
            ))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
        ]),
    );
    can.can.modify_filters().enable_bank(
        1,
        Fifo::Fifo1,
        BankConfig::List16([
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(
                can_2::Pcu::MESSAGE_ID as u16
            ))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(
                can_2::CarMissionStatus::MESSAGE_ID as u16
            ))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
        ]),
    );
    let (mut _can_tx, mut _can_rx) = can.can.split();

    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(task_senses(r.senses)));
        unwrap!(spawner.spawn(task_enables(r.enables)));
        unwrap!(spawner.spawn(pwm(r.pwm)));
        unwrap!(spawner.spawn(read_can(_can_rx)));
        unwrap!(spawner.spawn(write_can(_can_tx)));
        unwrap!(spawner.spawn(fault_detection(r.faults)));
        unwrap!(spawner.spawn(task_brake(r.brake)));
        unwrap!(spawner.spawn(task_asms(r.asms)));
    });
}

#[embassy_executor::task]
async fn task_asms(asms: Asms) {
    let asms_sens = Input::new(asms.sense_asms, Pull::Down);
    let mut ticker = Ticker::every(Duration::from_millis(50));
    loop {
        ticker.next().await;
        let asms_curr_state = asms_sens.is_high();
        CAN_WRITER
            .send((
                can_2::Asms::MESSAGE_ID as u16,
                can_2::Asms::DLC as usize,
                pad_array::<1, 8>(can_2::Asms::new(asms_curr_state).ok().unwrap().raw(), 0),
            ))
            .await;
    }
}

#[embassy_executor::task]
async fn task_brake(brake: Brake) {
    let mut brake_light = Output::new(brake.enable_brake, Level::Low, Speed::High);
    loop {
        let brake_enable = CAN_DRIVER_CHANNEL.wait().await;
        if brake_enable {
            brake_light.set_high();
        } else {
            brake_light.set_low();
        }
    }
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
        hz(50),
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
        hz(50),
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

    if false && CAN_BMS_EMERGENCY_CHANNEL.signaled() { //HACK:
        let fault_mes = can_2::PcuFault::new(true, true, true, true, true, true, true)
            .ok()
            .unwrap();
        loop {
            CAN_WRITER
                .send((
                    can_2::PcuFault::MESSAGE_ID as u16,
                    can_2::PcuFault::DLC as usize,
                    pad_array::<1, 8>(fault_mes.raw(), 0),
                ))
                .await;
            embassy_time::Timer::after_millis(10).await;
        }
    }

    //NOTE:
    //il pwm si setta da 0 a 2^16 (0-100%)
    //ventole droni lavorano linearmente fra 5% e 10%
    //pompe idem
    //ventole batteria invece lavorano da 100% a 0% (spente a 100 e al massimo a 0)

    pwm_fanbatt.set_level(0, true);
    pwm_fanbatt.duty = percent_to_duty(100);
    pwm_fanbatt.set_duty_left(percent_to_duty(100));
    pwm_fanbatt.set_duty_right(percent_to_duty(100));

    //INFO: calib max
    pwm_fanrad.set_level(0, false);
    pwm_fanrad.duty = percent_to_duty(10);
    pwm_fanrad.set_level(0, true);
    pwm_fanrad.set_duty_left(pwm_fanrad.duty);
    pwm_fanrad.set_duty_right(pwm_fanrad.duty);

    pwm_pump.set_level(0, false);
    pwm_pump.duty = percent_to_duty(10);
    pwm_pump.set_level(0, true);
    pwm_pump.set_duty_left(pwm_pump.duty);
    pwm_pump.set_duty_right(pwm_pump.duty);

    embassy_time::Timer::after_millis(7_500).await;

    //INFO: calib min
    pwm_fanrad.duty = percent_to_duty(5);
    pwm_fanrad.set_duty_left(pwm_fanrad.duty);
    pwm_fanrad.set_duty_right(pwm_fanrad.duty);

    pwm_pump.duty = percent_to_duty(5);
    pwm_pump.set_duty_left(pwm_pump.duty);
    pwm_pump.set_duty_right(pwm_pump.duty);

    embassy_time::Timer::after_millis(7_500).await;

    let mut fan_on = false;
    //INFO: ready

    let mut prev_state = can_2::CarStatusRunningStatus::SystemOff;
    let mut curr_state = can_2::CarStatusRunningStatus::SystemOff;
    let mut def_pumpl: u16 = 5750;
    let mut def_pumpr: u16 = 5750;
    let mut def_batt: u16 = percent_to_duty(40);
    let mut def_droni: u16 = 6226;

    //INFO: uncomment those lines to get them working with lv
    // pwm_pump.set_duty_left(5900);
    // pwm_pump.set_duty_right(5570);
    // pwm_fanrad.set_duty(3770).await;
    // pwm_fanbatt.set_duty(percent_to_duty(80)).await; //logica inversa
    loop {
        embassy_time::Timer::after_micros(50).await;
        if false && CAN_BMS_EMERGENCY_CHANNEL.signaled() {
            let fault_mes = can_2::PcuFault::new(true, true, true, true, true, true, true)
                .ok()
                .unwrap();
            loop {
                CAN_WRITER
                    .send((
                        can_2::PcuFault::MESSAGE_ID as u16,
                        can_2::PcuFault::DLC as usize,
                        pad_array::<1, 8>(fault_mes.raw(), 0),
                    ))
                    .await;
                embassy_time::Timer::after_millis(10).await;
            }
        } else if CAN_MISSIONSTATUS_CHANNEL.signaled() {
            curr_state = CAN_MISSIONSTATUS_CHANNEL.wait().await.running_status();
            if curr_state != prev_state {
                match curr_state {
                    can_2::CarStatusRunningStatus::Running => {
                        pwm_fanrad.set_duty(def_droni).await;
                        pwm_fanbatt.set_duty(def_batt).await;
                        pwm_pump.set_duty_left(def_pumpl);
                        pwm_pump.set_duty_right(def_pumpr);
                    }
                    _ => {
                        pwm_fanrad.set_duty(percent_to_duty(5)).await;
                        pwm_fanbatt.set_duty(percent_to_duty(100)).await;
                        pwm_pump.set_duty_left(percent_to_duty(5));
                        pwm_pump.set_duty_right(percent_to_duty(5));
                    }
                }
            }
            prev_state = curr_state;
        } else if CAN_PWM_CHANNEL.signaled() {
            let mes = CAN_PWM_CHANNEL.wait().await;
            pwm_fanrad.set_duty(mes.pwm_fanrad()).await;
            pwm_fanbatt.set_duty(mes.pwm_fanbatt()).await;
            pwm_pump.set_duty_left(mes.pwm_pumpl());
            pwm_pump.set_duty_right(mes.pwm_pumpr());
            def_batt = mes.pwm_fanbatt();
            def_pumpr = mes.pwm_pumpr();
            def_pumpl = mes.pwm_pumpl();
            def_droni = mes.pwm_fanrad();
        }
    }
}

#[embassy_executor::task]
async fn task_enables(enables: Enables) {
    let _enable_ef = Output::new(enables.enable_ef, Level::High, Speed::Low);
    let mut enabe_rf = Output::new(enables.enable_rf, Level::Low, Speed::Low);
    let mut enable_emb = Output::new(enables.enable_emb, Level::Low, Speed::Low);
    let mut enable_dv = Output::new(enables.enable_dv, Level::Low, Speed::Low);
    let mut _enable_24v = Output::new(enables.enable_24v, Level::High, Speed::Low);

    loop {
        match select(CAN_RF_CHANNEL.wait(), CAN_ENABLES_CHANNEL.wait()).await {
            Either::First(mes) => {
                enabe_rf.set_level(if mes.rf() { Level::High } else { Level::Low });
                let message = can_2::PcuRfAck::new(enabe_rf.is_set_high());
                if let Ok(some) = message {
                    CAN_WRITER
                        .send((
                            can_2::PcuRfAck::MESSAGE_ID as u16,
                            can_2::PcuRfAck::DLC as usize,
                            pad_array::<1, 8>(some.raw(), 0),
                        ))
                        .await;
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
}

#[repr(usize)]
enum FaultEnum {
    Dv = 0,
    V24 = 1,
    Pumpl = 2,
    Pumpr = 3,
    Fanbattl = 4,
    Fanbattr = 5,
}

impl<T> Index<FaultEnum> for [T] {
    type Output = T;

    fn index(&self, idx: FaultEnum) -> &Self::Output {
        &self[idx as usize]
    }
}

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

    let mut ticker = embassy_time::Ticker::every(Duration::from_millis(100));

    loop {
        ticker.next().await;
        let message = can_2::PcuFault::new(
            false,
            faults[FaultEnum::Dv].is_high(),
            faults[FaultEnum::V24].is_high(),
            faults[FaultEnum::Pumpl].is_high(),
            faults[FaultEnum::Pumpr].is_high(),
            faults[FaultEnum::Fanbattr].is_high(),
            faults[FaultEnum::Fanbattl].is_high(),
        );
        if let Ok(mes) = message {
            CAN_WRITER
                .send((
                    can_2::PcuFault::MESSAGE_ID as u16,
                    can_2::PcuFault::DLC as usize,
                    pad_array::<1, 8>(mes.raw(), 0),
                ))
                .await;
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
    let mut msg_1 = can_2::PcuAdc1::new(0f32, 0f32, 0f32).ok().unwrap();
    let mut msg_2 = can_2::PcuAdc2::new(0f32, 0f32, 0f32, 0f32).ok().unwrap();
    let mut msg_3 = can_2::PcuAdc3::new(0f32, 0f32, 0f32).ok().unwrap();

    loop {
        let val_dv = switch_texas(adc_1.blocking_read(&mut senses.sense_dv));
        let val_24v = switch_texas(adc_1.blocking_read(&mut senses.sense_24v));
        let val_pumpl = switch_texas(adc_1.blocking_read(&mut senses.sense_pumpl));
        let val_pumpr = switch_texas(adc_1.blocking_read(&mut senses.sense_pumpr));
        let val_fanbattl = switch_texas(adc_2.blocking_read(&mut senses.sense_fanbattl));
        let val_fanbattr = switch_texas(adc_2.blocking_read(&mut senses.sense_fanbattr));
        let val_fanradl = switch_infineon(adc_2.blocking_read(&mut senses.sense_fanradl));
        let val_fanradr = switch_infineon(adc_3.blocking_read(&mut senses.sense_fanradr));
        let val_emb = switch_infineon(adc_3.blocking_read(&mut senses.sense_emb));
        let val_steeract = switch_infineon(adc_2.blocking_read(&mut senses.steeract_sense));

        let _ = msg_1.set_adc_24v(val_24v);
        let _ = msg_1.set_adc_pumpl(val_pumpl);
        let _ = msg_1.set_adc_pumpr(val_pumpr);
        let _ = msg_2.set_adc_fanradl(val_fanradl);
        let _ = msg_2.set_adc_fanradr(val_fanradr);
        let _ = msg_2.set_adc_fanbattl(val_fanbattl);
        let _ = msg_2.set_adc_fanbattr(val_fanbattr);
        let _ = msg_3.set_adc_dv(val_dv);
        let _ = msg_3.set_adc_emb(val_emb);
        let _ = msg_3.set_adc_steeract(val_steeract);

        CAN_WRITER
            .send((
                can_2::PcuAdc1::MESSAGE_ID as u16,
                can_2::PcuAdc1::DLC as usize,
                pad_array::<6, 8>(msg_1.raw(), 0),
            ))
            .await;
        CAN_WRITER
            .send((
                can_2::PcuAdc2::MESSAGE_ID as u16,
                can_2::PcuAdc2::DLC as usize,
                msg_2.raw().clone(),
            ))
            .await;
        CAN_WRITER
            .send((
                can_2::PcuAdc3::MESSAGE_ID as u16,
                can_2::PcuAdc3::DLC as usize,
                pad_array::<6, 8>(msg_3.raw(), 0),
            ))
            .await;

        embassy_time::Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn read_can(mut can: CanRx<'static>) {
    loop {
        if let Ok(envelope) = can.try_read() {
            let frame = CanFrame::from_envelope(envelope);
            let message =
                can_2::Messages::from_can_message(u32::from(frame.id()), frame._bytes().as_slice())
                    .ok();
            if message.is_some() {
                let message = unwrap!(message);
                match message {
                    can_2::Messages::Pcu(mes) => match mes.mode_raw() {
                        1 => {
                            CAN_RF_CHANNEL.signal(can_2::PcuModeM1::new_from_raw(*mes.raw()));
                        }
                        2 => {
                            CAN_ENABLES_CHANNEL.signal(can_2::PcuModeM2::new_from_raw(*mes.raw()));
                        }
                        _ => {}
                    },
                    can_2::Messages::EbsBrakeReq(mes) => {
                        CAN_DRIVER_CHANNEL.signal(mes.req());
                    }
                    can_2::Messages::Driver(mes) => {
                        CAN_DRIVER_CHANNEL.signal(mes.brake() > 5);
                    }
                    can_2::Messages::CarStatus(mes) => {
                        CAN_MISSIONSTATUS_CHANNEL.signal(mes);
                    }
                    can_2::Messages::BmsLvEmergency(_) => {
                        CAN_BMS_EMERGENCY_CHANNEL.signal(true);
                    }
                    can_2::Messages::CoolingControl(mes) => {
                        CAN_PWM_CHANNEL.signal(mes);
                    }
                    _ => {}
                }
            }
        }
        embassy_time::Timer::after_micros(50).await;
    }
}

#[embassy_executor::task]
async fn write_can(mut can: CanTx<'static>) {
    loop {
        let (id, dlc, mes) = CAN_WRITER.receive().await;
        let message = CanFrame::new(id, &mes[..dlc]);
        let _ = can.try_write(&message.frame());
        embassy_time::Timer::after_micros(50).await;
    }
}
