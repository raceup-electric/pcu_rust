use assign_resources::assign_resources;
use embassy_stm32::peripherals;


assign_resources! {
    faults: Faults {
        fault_12v: PC8,
        exti_12v: EXTI8,
        fault_dv: PB14, // TODO: scambiare nel pcb 12v_enable e dv_fault
        exti_dv: EXTI14,
        fault_24v: PC10,
        exti_24v: EXTI10,
        fault_pumpl: PC15,
        exti_pumpl: EXTI15,
        fault_pumpr: PB9,
        exti_pumpr: EXTI9,
        fault_fanbattl: PB4,
        exti_fanbattl: EXTI4,
        fault_fanbattr: PD2,
        exti_fanbattr: EXTI2,
    }

    senses: Senses {
        sense_12v: PA0,
        sense_dv: PA1,
        sense_24v: PA2,
        sense_pumpl: PA3,
        sense_pumpr: PA4,
        sense_fanbattl: PA5,
        sense_fanbattr: PA6,
        sense_fanradl: PA7,
        sense_fanradr: PB0,
        sense_emb: PB1,
        steeract_sense: PC4,
        adc_1: ADC1,
        adc_2: ADC2,
        adc_3: ADC3,
    }

    brake: Brake {
        enable_brake: PC12,
    }

    pwm: Pwm {
        pwm_fanradl: PB10,
        fanradl_enable: PC1,
        pwm_fanradr: PB11,
        fanradr_enable: PC2,
        pwm_fanbattl: PC7,
        enable_fanbattl: PB5,
        pwm_fanbattr: PC6,
        enable_fanbattr: PC11,
        pwm_pumpl: PB7,
        enable_pumpl: PC3,
        pwm_pumpr: PB6,
        enable_pumpr: PB8,
    }

    enables: Enables {
        enable_ef: PC14,
        enable_rf: PC13,
        enable_emb: PC0,
        enable_dv: PA15,
        enable_12v: PA9,
        enable_24v: PC5,
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

    sw: Sw {
        swdio: PA13,
        swclk: PA14,
        swo: PB3,
    }
}
