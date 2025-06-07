use assign_resources::assign_resources;
use embassy_stm32::peripherals;

assign_resources! {

    faults: Faults {
        fault_dv: PA9,
        fault_24v: PC8,
        fault_pumpl: PC15,
        fault_pumpr: PB9,
        fault_fanbattl: PB15,
        fault_fanbattr: PD2,
    }

    senses: Senses {
        sense_dv: PB0,
        sense_24v: PC4,
        sense_pumpl: PA6,
        sense_pumpr: PA7,
        sense_fanbattl: PA4,
        sense_fanbattr: PA5,
        sense_fanradl: PA3,
        sense_fanradr: PA2,
        sense_emb: PA1,
        steeract_sense: PA0,
        adc_1: ADC1,
        adc_2: ADC2,
        adc_3: ADC3,
    }

    brake: Brake {
        enable_brake: PC12,
    }

    pwm: Pwm {
        pwm_fanradl: PB11,
        enable_fanradl: PC1,
        pwm_fanradr: PB10,
        enable_fanradr: PC2,
        pwm_fanbattl: PC7,
        enable_fanbattl: PB5,
        pwm_fanbattr: PC6,
        enable_fanbattr: PC11,
        pwm_pumpl: PB7,
        enable_pumpl: PC3,
        pwm_pumpr: PB6,
        enable_pumpr: PB8,
        timer_fanrad: TIM2,
        timer_fanbatt: TIM3,
        timer_pump: TIM4,
    }

    enables: Enables {
        enable_ef: PC14,
        enable_rf: PC13,
        enable_emb: PC0,
        enable_dv: PA8,
        enable_24v: PB14,
    }

    usb: Usb {
        usb_minus: PA11,
        usb_plus: PA12,
    }

    can : Can {
        can_rx: PB12,
        can_tx: PB13,
        can_2: CAN2,
        can_1: CAN1,
    }

    asms : Asms {
        sense_asms: PA10,
        exti_asms: EXTI10,
    }

    sw: Sw {
        swdio: PA13,
        swclk: PA14,
        swo: PB3,
    }
}
