
pub struct Pins {
    // can
    pub pin_can_tx : embassy_stm32::peripherals::PB13,
    pub pin_can_rx : embassy_stm32::peripherals::PB12,
    pub can_1 : embassy_stm32::peripherals::CAN1,
    pub can_2 : embassy_stm32::peripherals::CAN2,
    // usb
    pub pin_usb_minus : embassy_stm32::peripherals::PA11,
    pub pin_usb_plus : embassy_stm32::peripherals::PA12,
    // jtag
    pub pin_swdio : embassy_stm32::peripherals::PA13,
    pub pin_swclk : embassy_stm32::peripherals::PA14,
    pub pin_swo : embassy_stm32::peripherals::PB3,
    // 12v
    pub pin_12v_enable : embassy_stm32::peripherals::PB14,
    pub pin_12v_fault : embassy_stm32::peripherals::PC8,
    pub pin_12v_sense : embassy_stm32::peripherals::PA0,
    // dv
    pub pin_dv_enable : embassy_stm32::peripherals::PA15,
    pub pin_dv_fault : embassy_stm32::peripherals::PA9,
    pub pin_dv_sense : embassy_stm32::peripherals::PA1,
    // 24v
    pub pin_24v_enable : embassy_stm32::peripherals::PC5,
    pub pin_24v_fault : embassy_stm32::peripherals::PC10,
    pub pin_24v_sense : embassy_stm32::peripherals::PA2,
    // pumpl
    pub pin_pumpl_enable : embassy_stm32::peripherals::PC3,
    pub pin_pumpl_fault : embassy_stm32::peripherals::PC15,
    pub pin_pumpl_sense : embassy_stm32::peripherals::PA3,
    pub pin_pumpl_pwm : embassy_stm32::peripherals::PB7,
    // pumpr
    pub pin_pumpr_enable : embassy_stm32::peripherals::PB8,
    pub pin_pumpr_fault : embassy_stm32::peripherals::PB9,
    pub pin_pumpr_sense : embassy_stm32::peripherals::PA4,
    pub pin_pumpr_pwm : embassy_stm32::peripherals::PB6,
    // fanbattl
    pub pin_fanbattl_enable : embassy_stm32::peripherals::PB5,
    pub pin_fanbattl_fault : embassy_stm32::peripherals::PB4,
    pub pin_fanbattl_sense : embassy_stm32::peripherals::PA5,
    pub pin_fanbattl_pwm : embassy_stm32::peripherals::PC7,
    // fanbattr
    pub pin_fanbattr_enable : embassy_stm32::peripherals::PC11,
    pub pin_fanbattr_fault : embassy_stm32::peripherals::PD2,
    pub pin_fanbattr_sense : embassy_stm32::peripherals::PA6,
    pub pin_fanbattr_pwm : embassy_stm32::peripherals::PC6,
    // fanradl
    pub pin_fanradl_enable : embassy_stm32::peripherals::PC1,
    pub pin_fanradl_sense : embassy_stm32::peripherals::PA7,
    pub pin_fanradl_pwm : embassy_stm32::peripherals::PB10,
    // fanradr
    pub pin_fanradr_enable : embassy_stm32::peripherals::PC2,
    pub pin_fanradr_sense : embassy_stm32::peripherals::PB0,
    pub pin_fanradr_pwm : embassy_stm32::peripherals::PB11,
    // embedded
    pub pin_embedded_enable : embassy_stm32::peripherals::PC0,
    pub pin_embedded_sense : embassy_stm32::peripherals::PB1,
    // steeract
    pub pin_steeract_sense : embassy_stm32::peripherals::PC4,
    // brake
    pub pin_brake_enable : embassy_stm32::peripherals::PC12,
    // rf 
    pub pin_rf : embassy_stm32::peripherals::PC13,
    // ef
    pub pin_ef : embassy_stm32::peripherals::PC14,
    // asms
    pub pin_asms_sens : embassy_stm32::peripherals::PA10,

    // timers
    // pwm
    pub timer_fanrad : embassy_stm32::peripherals::TIM2,
    pub timer_fanbatt : embassy_stm32::peripherals::TIM3,
    pub timer_pump : embassy_stm32::peripherals::TIM4,
}

pub struct Timers {
   
}


impl Pins {
    pub fn new(p: embassy_stm32::Peripherals) -> Self {
        Self {
            // can
            pin_can_tx: p.PB13,
            pin_can_rx: p.PB12,
            can_1: p.CAN1,
            can_2: p.CAN2,
            // usb
            pin_usb_minus: p.PA11,
            pin_usb_plus: p.PA12,
            // jtag
            pin_swdio: p.PA13,
            pin_swclk: p.PA14,
            pin_swo: p.PB3,
            // 12v
            pin_12v_enable: p.PB14,
            pin_12v_fault: p.PC8,
            pin_12v_sense: p.PA0,
            // dv
            pin_dv_enable: p.PA15,
            pin_dv_fault: p.PA9,
            pin_dv_sense: p.PA1,
            // 24v
            pin_24v_enable: p.PC5,
            pin_24v_fault: p.PC10,
            pin_24v_sense: p.PA2,
            // pumpl
            pin_pumpl_enable: p.PC3,
            pin_pumpl_fault: p.PC15,
            pin_pumpl_sense: p.PA3,
            pin_pumpl_pwm: p.PB7,
            // pumpr
            pin_pumpr_enable: p.PB8,
            pin_pumpr_fault: p.PB9,
            pin_pumpr_sense: p.PA4,
            pin_pumpr_pwm: p.PB6,
            // fanbattl
            pin_fanbattl_enable: p.PB5,
            pin_fanbattl_fault: p.PB4,
            pin_fanbattl_sense: p.PA5,
            pin_fanbattl_pwm: p.PC7,
            // fanbattr
            pin_fanbattr_enable: p.PC11,
            pin_fanbattr_fault: p.PD2,
            pin_fanbattr_sense: p.PA6,
            pin_fanbattr_pwm: p.PC6,
            // fanradl
            pin_fanradl_enable: p.PC1,
            pin_fanradl_sense: p.PA7,
            pin_fanradl_pwm: p.PB10,
            // fanradr
            pin_fanradr_enable: p.PC2,
            pin_fanradr_sense: p.PB0,
            pin_fanradr_pwm: p.PB11,
            // embedded
            pin_embedded_enable: p.PC0,
            pin_embedded_sense: p.PB1,
            // steeract
            pin_steeract_sense: p.PC4,
            // brake
            pin_brake_enable: p.PC12,
            // rf
            pin_rf: p.PC13,
            // ef
            pin_ef: p.PC14,
            // asms
            pin_asms_sens: p.PA10,

            // timers
            // pwm
            timer_fanrad: p.TIM2,
            timer_fanbatt: p.TIM3,
            timer_pump: p.TIM4,
        }
    }
}

