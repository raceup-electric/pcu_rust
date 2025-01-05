#[repr(u16)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
//  Can0.setRXFilter(0, 0x130, 0x7FF, false); // dbc
//              start   length
//  fan_enable  0       1           Water radiator fan enable
//  fan_speed   1       7           Water radiator fan speed in %
//  pump_enable 8       1           Water pump enable
//  pump_speed  9       7           Water pump speed
pub enum CanMsg {
    PcuId = 0x130,
}

impl CanMsg {
    pub fn as_raw(&self) -> u16 {
        *self as u16
    }
}
