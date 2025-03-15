#![no_std]
#![no_main]

use core::ops::Index;

use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_stm32::{
    adc::{Adc, AdcChannel},
    can::{
        filter::*,
        Fifo,
        StandardId,
        CanRx,
        CanTx,
    },
    exti::*,
    gpio::{Input, Level, Output, OutputType, Pull, Speed},
    interrupt,
    interrupt::{InterruptExt, Priority},
    timer::simple_pwm::{PwmPin, SimplePwm},
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::Channel,
};
use embassy_futures::select::*;

use static_cell::StaticCell;
use defmt::{panic, *};

mod can_management;
mod pins;
mod pwm_management;
use crate::pins::{AssignedResources, Brake, Can, Enables, Faults, Pwm, Senses, Sw, Usb};
use crate::pwm_management::PwmDualController;
use crate::can_management::{can_operation, messages as can_2, CanController, CanFrame};

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[interrupt]
unsafe fn UART4() {
    EXECUTOR_HIGH.on_interrupt()
}

//static CAN_PWM_CHANNEL: Channel<CriticalSectionRawMutex, can_2::, 1> = Channel::new();
static CAN_ENABLES_CHANNEL: Channel<CriticalSectionRawMutex, [u8;8], 1> = Channel::new();
static CAN_DRIVER_CHANNEL: Channel<CriticalSectionRawMutex, bool , 1> = Channel::new();


#[embassy_executor::main]
async fn main(spawner: Spawner) {
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
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(can_2::Driver::MESSAGE_ID as u16))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
        ]),
    );
    can.can.modify_filters().enable_bank(
        0,
        Fifo::Fifo1,
        BankConfig::List16([
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(can_2::Pcu::MESSAGE_ID as u16))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
            ListEntry16::data_frames_with_id(unwrap!(StandardId::new(0x1))),
        ]),
    );

    let (can_tx, can_rx) = can.can.split();


    interrupt::UART4.set_priority(Priority::P6);
    let spawner = EXECUTOR_HIGH.start(interrupt::UART4);

    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(task_senses(r.senses)));
    });
}

#[embassy_executor::task]
async fn task_enables(enables: Enables) {
    let _enable_ef = Output::new(enables.enable_ef, Level::High, Speed::Low);
    let enabe_rf = Output::new(enables.enable_rf, Level::Low, Speed::Low);
    let enable_emb = Output::new(enables.enable_emb, Level::Low, Speed::Low);
    let enable_dv = Output::new(enables.enable_dv, Level::Low, Speed::Low);
    let _enable_12v = Output::new(enables.enable_12v, Level::High, Speed::Low);
    let _enable_24v = Output::new(enables.enable_24v, Level::High, Speed::Low);
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

fn update_fault_state(pin: &ExtiInput, state: &mut bool) {
    if pin.is_high() {
        *state = true;
    } else {
        *state = false;
    }
}

#[embassy_executor::task]
async fn fault_detection(faults: Faults) {
    let faults = [
        ExtiInput::new(faults.fault_12v, faults.exti_12v, Pull::None),
        ExtiInput::new(faults.fault_dv, faults.exti_dv, Pull::None),
        ExtiInput::new(faults.fault_24v, faults.exti_24v, Pull::None),
        ExtiInput::new(faults.fault_pumpl, faults.exti_pumpl, Pull::None),
        ExtiInput::new(faults.fault_pumpr, faults.exti_pumpr, Pull::None),
        ExtiInput::new(faults.fault_fanbattl, faults.exti_fanbattl, Pull::None),
        ExtiInput::new(faults.fault_fanbattr, faults.exti_fanbattr, Pull::None),
    ];

    let mut states = [false; 7];

    loop {
        let futures =
        [
            faults[FaultEnum::V12].wait_for_rising_edge(),
            faults[FaultEnum::Dv].wait_for_falling_edge(),
            //faults[FaultEnum::Pumpl].wait_for_high()
        ];
        let result = embassy_futures::select::select_array(futures).await;

        for (i, pin) in faults.iter().enumerate() {
            update_fault_state(pin, &mut states[i]);
        }

        //let message = can_2::PcuFault::new(
        //    states[FaultEnum::V12],
        //    states[FaultEnum::Dv],
        //    states[FaultEnum::V24],
        //    states[FaultEnum::Pumpl],
        //    states[FaultEnum::Pumpr],
        //    states[FaultEnum::Fanbattl],
        //    states[FaultEnum::Fanbattr],
        //);
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
        let _val_12v = switch_texas(adc_1.blocking_read(&mut senses.sense_12v));
        let _val_dv = switch_texas(adc_3.blocking_read(&mut senses.sense_dv));
        let _val_24v = switch_texas(adc_3.blocking_read(&mut senses.sense_24v));
        let _val_pumpl = switch_texas(adc_2.blocking_read(&mut senses.sense_pumpl));
        let _val_pumpr = switch_texas(adc_2.blocking_read(&mut senses.sense_pumpr));
        let _val_fanbattl = switch_texas(adc_2.blocking_read(&mut senses.sense_fanbattl));
        let _val_fanbattr = switch_texas(adc_1.blocking_read(&mut senses.sense_fanbattr));
        let _val_fanradl = switch_infineon(adc_1.blocking_read(&mut senses.sense_fanradl));
        let _val_fanradr = switch_infineon(adc_1.blocking_read(&mut senses.sense_fanradr));
        let _val_emb = switch_infineon(adc_1.blocking_read(&mut senses.sense_emb));
        let _val_steeract = switch_infineon(adc_1.blocking_read(&mut senses.steeract_sense));
        embassy_time::Timer::after_millis(5000).await;
    }
}

#[embassy_executor::task]
async fn read_can(mut rx: CanRx<'static>) {
    loop{
        let envelope = unwrap!(rx.read().await);
        let frame = CanFrame::from_envelope(envelope);
        match frame.id() as u32 {
            val if val == can_2::Driver::MESSAGE_ID => {
                CAN_DRIVER_CHANNEL.clear();
                CAN_DRIVER_CHANNEL.send(frame._bytes());
            },
            val if val == can_2::Pcu::MESSAGE_ID => {
                CAN_PCU_CHANNEL.clear();
                CAN_DRIVER_CHANNEL
            },
            _ => (),
        }

    }
}

#[embassy_executor::task]
async fn write_can() {}
