// Generated code!
#![allow(unused_comparisons, unreachable_patterns)]
#![allow(clippy::let_and_return, clippy::eq_op)]
#![allow(clippy::excessive_precision, clippy::manual_range_contains, clippy::absurd_extreme_comparisons)]
#![deny(clippy::arithmetic_side_effects)]

//! Message definitions from file `"can2.dbc"`
//!
//! - Version: `Version("")`

use core::ops::BitOr;
use bitvec::prelude::*;
#[cfg(feature = "arb")]
use arbitrary::{Arbitrary, Unstructured};

/// All messages
#[derive(Clone)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum Messages {
    /// TanksEBS
    TanksEbs(TanksEbs),
    /// PcuFault
    PcuFault(PcuFault),
    /// Paddle
    Paddle(Paddle),
    /// Driver
    Driver(Driver),
    /// BmsLv1
    BmsLv1(BmsLv1),
    /// BmsLv2
    BmsLv2(BmsLv2),
    /// BmsHv1
    BmsHv1(BmsHv1),
    /// BmsHv2
    BmsHv2(BmsHv2),
    /// Imu1
    Imu1(Imu1),
    /// Imu2
    Imu2(Imu2),
    /// Imu3
    Imu3(Imu3),
    /// IMUCalib
    ImuCalib(ImuCalib),
    /// Map
    Map(Map),
    /// CarStatus
    CarStatus(CarStatus),
    /// CarSettings
    CarSettings(CarSettings),
    /// CarMission
    CarMission(CarMission),
    /// CheckASB
    CheckAsb(CheckAsb),
    /// LapStart
    LapStart(LapStart),
    /// Temp1
    Temp1(Temp1),
    /// Temp2
    Temp2(Temp2),
    /// SuspRear
    SuspRear(SuspRear),
    /// RESERVED2
    Reserved2(Reserved2),
    /// SuspFront
    SuspFront(SuspFront),
    /// TempFrontR
    TempFrontR(TempFrontR),
    /// InvVolt
    InvVolt(InvVolt),
    /// Pcu
    Pcu(Pcu),
    /// Calib
    Calib(Calib),
    /// CalibAck
    CalibAck(CalibAck),
    /// PcuRfAck
    PcuRfAck(PcuRfAck),
    /// Lem
    Lem(Lem),
}

impl Messages {
    /// Read message from CAN frame
    #[inline(never)]
    pub fn from_can_message(id: u32, payload: &[u8]) -> Result<Self, CanError> {
        
        let res = match id {
            60 => Messages::TanksEbs(TanksEbs::try_from(payload)?),
            81 => Messages::PcuFault(PcuFault::try_from(payload)?),
            82 => Messages::Paddle(Paddle::try_from(payload)?),
            83 => Messages::Driver(Driver::try_from(payload)?),
            84 => Messages::BmsLv1(BmsLv1::try_from(payload)?),
            85 => Messages::BmsLv2(BmsLv2::try_from(payload)?),
            87 => Messages::BmsHv1(BmsHv1::try_from(payload)?),
            88 => Messages::BmsHv2(BmsHv2::try_from(payload)?),
            96 => Messages::Imu1(Imu1::try_from(payload)?),
            97 => Messages::Imu2(Imu2::try_from(payload)?),
            98 => Messages::Imu3(Imu3::try_from(payload)?),
            99 => Messages::ImuCalib(ImuCalib::try_from(payload)?),
            100 => Messages::Map(Map::try_from(payload)?),
            101 => Messages::CarStatus(CarStatus::try_from(payload)?),
            102 => Messages::CarSettings(CarSettings::try_from(payload)?),
            103 => Messages::CarMission(CarMission::try_from(payload)?),
            104 => Messages::CheckAsb(CheckAsb::try_from(payload)?),
            112 => Messages::LapStart(LapStart::try_from(payload)?),
            256 => Messages::Temp1(Temp1::try_from(payload)?),
            257 => Messages::Temp2(Temp2::try_from(payload)?),
            258 => Messages::SuspRear(SuspRear::try_from(payload)?),
            259 => Messages::Reserved2(Reserved2::try_from(payload)?),
            260 => Messages::SuspFront(SuspFront::try_from(payload)?),
            261 => Messages::TempFrontR(TempFrontR::try_from(payload)?),
            288 => Messages::InvVolt(InvVolt::try_from(payload)?),
            304 => Messages::Pcu(Pcu::try_from(payload)?),
            305 => Messages::Calib(Calib::try_from(payload)?),
            306 => Messages::CalibAck(CalibAck::try_from(payload)?),
            308 => Messages::PcuRfAck(PcuRfAck::try_from(payload)?),
            962 => Messages::Lem(Lem::try_from(payload)?),
            n => return Err(CanError::UnknownMessageId(n)),
        };
        Ok(res)
    }
}

/// TanksEBS
///
/// - ID: 60 (0x3c)
/// - Size: 2 bytes
#[derive(Clone, Copy)]
pub struct TanksEbs {
    raw: [u8; 2],
}

impl TanksEbs {
    pub const MESSAGE_ID: u32 = 60;
    
    pub const PRESS_LEFT_TANK_MIN: f32 = 6_f32;
    pub const PRESS_LEFT_TANK_MAX: f32 = 10_f32;
    pub const PRESS_RIGHT_TANK_MIN: f32 = 6_f32;
    pub const PRESS_RIGHT_TANK_MAX: f32 = 10_f32;
    
    /// Construct new TanksEBS from values
    pub fn new(system_check: bool, press_left_tank: f32, press_right_tank: f32, sanity_left_sensor: bool, sanity_right_sensor: bool) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_system_check(system_check)?;
        res.set_press_left_tank(press_left_tank)?;
        res.set_press_right_tank(press_right_tank)?;
        res.set_sanity_left_sensor(sanity_left_sensor)?;
        res.set_sanity_right_sensor(sanity_right_sensor)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// system_check
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn system_check(&self) -> bool {
        self.system_check_raw()
    }
    
    /// Get raw value of system_check
    ///
    /// - Start bit: 0
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn system_check_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[0..1].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of system_check
    #[inline(always)]
    pub fn set_system_check(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[0..1].store_le(value);
        Ok(())
    }
    
    /// press_left_tank
    ///
    /// - Min: 6
    /// - Max: 10
    /// - Unit: "Bar"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn press_left_tank(&self) -> f32 {
        self.press_left_tank_raw()
    }
    
    /// Get raw value of press_left_tank
    ///
    /// - Start bit: 1
    /// - Signal size: 5 bits
    /// - Factor: 0.25
    /// - Offset: 6
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn press_left_tank_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[1..6].load_le::<u8>();
        
        let factor = 0.25_f32;
        let offset = 6_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of press_left_tank
    #[inline(always)]
    pub fn set_press_left_tank(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 6_f32 || 10_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 60 });
        }
        let factor = 0.25_f32;
        let offset = 6_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[1..6].store_le(value);
        Ok(())
    }
    
    /// press_right_tank
    ///
    /// - Min: 6
    /// - Max: 10
    /// - Unit: "Bar"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn press_right_tank(&self) -> f32 {
        self.press_right_tank_raw()
    }
    
    /// Get raw value of press_right_tank
    ///
    /// - Start bit: 6
    /// - Signal size: 5 bits
    /// - Factor: 0.25
    /// - Offset: 6
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn press_right_tank_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[6..11].load_le::<u8>();
        
        let factor = 0.25_f32;
        let offset = 6_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of press_right_tank
    #[inline(always)]
    pub fn set_press_right_tank(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 6_f32 || 10_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 60 });
        }
        let factor = 0.25_f32;
        let offset = 6_f32;
        let value = ((value - offset) / factor) as u8;
        
        self.raw.view_bits_mut::<Lsb0>()[6..11].store_le(value);
        Ok(())
    }
    
    /// sanity_left_sensor
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn sanity_left_sensor(&self) -> bool {
        self.sanity_left_sensor_raw()
    }
    
    /// Get raw value of sanity_left_sensor
    ///
    /// - Start bit: 11
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn sanity_left_sensor_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[11..12].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of sanity_left_sensor
    #[inline(always)]
    pub fn set_sanity_left_sensor(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[11..12].store_le(value);
        Ok(())
    }
    
    /// sanity_right_sensor
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn sanity_right_sensor(&self) -> bool {
        self.sanity_right_sensor_raw()
    }
    
    /// Get raw value of sanity_right_sensor
    ///
    /// - Start bit: 12
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn sanity_right_sensor_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[12..13].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of sanity_right_sensor
    #[inline(always)]
    pub fn set_sanity_right_sensor(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[12..13].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for TanksEbs {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for TanksEbs {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("TanksEbs")
                .field("system_check", &self.system_check())
                .field("press_left_tank", &self.press_left_tank())
                .field("press_right_tank", &self.press_right_tank())
                .field("sanity_left_sensor", &self.sanity_left_sensor())
                .field("sanity_right_sensor", &self.sanity_right_sensor())
            .finish()
        } else {
            f.debug_tuple("TanksEbs").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for TanksEbs {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let system_check = u.int_in_range(0..=1)? == 1;
        let press_left_tank = u.float_in_range(6_f32..=10_f32)?;
        let press_right_tank = u.float_in_range(6_f32..=10_f32)?;
        let sanity_left_sensor = u.int_in_range(0..=1)? == 1;
        let sanity_right_sensor = u.int_in_range(0..=1)? == 1;
        TanksEbs::new(system_check,press_left_tank,press_right_tank,sanity_left_sensor,sanity_right_sensor).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// PcuFault
///
/// - ID: 81 (0x51)
/// - Size: 1 bytes
/// - Transmitter: PCU
#[derive(Clone, Copy)]
pub struct PcuFault {
    raw: [u8; 1],
}

impl PcuFault {
    pub const MESSAGE_ID: u32 = 81;
    
    
    /// Construct new PcuFault from values
    pub fn new(fault_12v: bool, fault_dv: bool, fault_24v: bool, fault_pumpl: bool, fault_pumpr: bool, fault_fanbattr: bool, fault_fanbattl: bool) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 1] };
        res.set_fault_12v(fault_12v)?;
        res.set_fault_dv(fault_dv)?;
        res.set_fault_24v(fault_24v)?;
        res.set_fault_pumpl(fault_pumpl)?;
        res.set_fault_pumpr(fault_pumpr)?;
        res.set_fault_fanbattr(fault_fanbattr)?;
        res.set_fault_fanbattl(fault_fanbattl)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 1] {
        &self.raw
    }
    
    /// fault_12v
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "on"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn fault_12v(&self) -> bool {
        self.fault_12v_raw()
    }
    
    /// Get raw value of fault_12v
    ///
    /// - Start bit: 0
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn fault_12v_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[0..1].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of fault_12v
    #[inline(always)]
    pub fn set_fault_12v(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[0..1].store_le(value);
        Ok(())
    }
    
    /// fault_dv
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "on"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn fault_dv(&self) -> bool {
        self.fault_dv_raw()
    }
    
    /// Get raw value of fault_dv
    ///
    /// - Start bit: 1
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn fault_dv_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[1..2].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of fault_dv
    #[inline(always)]
    pub fn set_fault_dv(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[1..2].store_le(value);
        Ok(())
    }
    
    /// fault_24v
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "on"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn fault_24v(&self) -> bool {
        self.fault_24v_raw()
    }
    
    /// Get raw value of fault_24v
    ///
    /// - Start bit: 2
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn fault_24v_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[2..3].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of fault_24v
    #[inline(always)]
    pub fn set_fault_24v(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[2..3].store_le(value);
        Ok(())
    }
    
    /// fault_pumpl
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "on"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn fault_pumpl(&self) -> bool {
        self.fault_pumpl_raw()
    }
    
    /// Get raw value of fault_pumpl
    ///
    /// - Start bit: 3
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn fault_pumpl_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[3..4].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of fault_pumpl
    #[inline(always)]
    pub fn set_fault_pumpl(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[3..4].store_le(value);
        Ok(())
    }
    
    /// fault_pumpr
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "on"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn fault_pumpr(&self) -> bool {
        self.fault_pumpr_raw()
    }
    
    /// Get raw value of fault_pumpr
    ///
    /// - Start bit: 4
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn fault_pumpr_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[4..5].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of fault_pumpr
    #[inline(always)]
    pub fn set_fault_pumpr(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[4..5].store_le(value);
        Ok(())
    }
    
    /// fault_fanbattr
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "on"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn fault_fanbattr(&self) -> bool {
        self.fault_fanbattr_raw()
    }
    
    /// Get raw value of fault_fanbattr
    ///
    /// - Start bit: 5
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn fault_fanbattr_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[5..6].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of fault_fanbattr
    #[inline(always)]
    pub fn set_fault_fanbattr(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[5..6].store_le(value);
        Ok(())
    }
    
    /// fault_fanbattl
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "on"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn fault_fanbattl(&self) -> bool {
        self.fault_fanbattl_raw()
    }
    
    /// Get raw value of fault_fanbattl
    ///
    /// - Start bit: 6
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn fault_fanbattl_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[6..7].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of fault_fanbattl
    #[inline(always)]
    pub fn set_fault_fanbattl(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[6..7].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for PcuFault {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 1 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 1];
        raw.copy_from_slice(&payload[..1]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for PcuFault {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("PcuFault")
                .field("fault_12v", &self.fault_12v())
                .field("fault_dv", &self.fault_dv())
                .field("fault_24v", &self.fault_24v())
                .field("fault_pumpl", &self.fault_pumpl())
                .field("fault_pumpr", &self.fault_pumpr())
                .field("fault_fanbattr", &self.fault_fanbattr())
                .field("fault_fanbattl", &self.fault_fanbattl())
            .finish()
        } else {
            f.debug_tuple("PcuFault").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for PcuFault {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let fault_12v = u.int_in_range(0..=1)? == 1;
        let fault_dv = u.int_in_range(0..=1)? == 1;
        let fault_24v = u.int_in_range(0..=1)? == 1;
        let fault_pumpl = u.int_in_range(0..=1)? == 1;
        let fault_pumpr = u.int_in_range(0..=1)? == 1;
        let fault_fanbattr = u.int_in_range(0..=1)? == 1;
        let fault_fanbattl = u.int_in_range(0..=1)? == 1;
        PcuFault::new(fault_12v,fault_dv,fault_24v,fault_pumpl,fault_pumpr,fault_fanbattr,fault_fanbattl).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// Paddle
///
/// - ID: 82 (0x52)
/// - Size: 1 bytes
/// - Transmitter: SW
#[derive(Clone, Copy)]
pub struct Paddle {
    raw: [u8; 1],
}

impl Paddle {
    pub const MESSAGE_ID: u32 = 82;
    
    pub const REGEN_MIN: u8 = 0_u8;
    pub const REGEN_MAX: u8 = 100_u8;
    
    /// Construct new Paddle from values
    pub fn new(regen: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 1] };
        res.set_regen(regen)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 1] {
        &self.raw
    }
    
    /// regen
    ///
    /// % of regen paddle travel
    ///
    /// - Min: 0
    /// - Max: 100
    /// - Unit: "%"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn regen(&self) -> u8 {
        self.regen_raw()
    }
    
    /// Get raw value of regen
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn regen_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        signal
    }
    
    /// Set value of regen
    #[inline(always)]
    pub fn set_regen(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 100_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 82 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Paddle {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 1 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 1];
        raw.copy_from_slice(&payload[..1]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Paddle {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Paddle")
                .field("regen", &self.regen())
            .finish()
        } else {
            f.debug_tuple("Paddle").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Paddle {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let regen = u.int_in_range(0..=100)?;
        Paddle::new(regen).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// Driver
///
/// - ID: 83 (0x53)
/// - Size: 4 bytes
/// - Transmitter: ATC
#[derive(Clone, Copy)]
pub struct Driver {
    raw: [u8; 4],
}

impl Driver {
    pub const MESSAGE_ID: u32 = 83;
    
    pub const THROTTLE_MIN: u8 = 0_u8;
    pub const THROTTLE_MAX: u8 = 100_u8;
    pub const BRAKE_MIN: u8 = 0_u8;
    pub const BRAKE_MAX: u8 = 100_u8;
    pub const STEERING_MIN: i16 = -120_i16;
    pub const STEERING_MAX: i16 = 120_i16;
    
    /// Construct new Driver from values
    pub fn new(throttle: u8, brake: u8, steering: i16, no_implausibility: bool, bre_implausibility: bool, pad_implausibility: bool, pot_implausibility: bool) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 4] };
        res.set_throttle(throttle)?;
        res.set_brake(brake)?;
        res.set_steering(steering)?;
        res.set_no_implausibility(no_implausibility)?;
        res.set_bre_implausibility(bre_implausibility)?;
        res.set_pad_implausibility(pad_implausibility)?;
        res.set_pot_implausibility(pot_implausibility)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 4] {
        &self.raw
    }
    
    /// throttle
    ///
    /// % of throttle
    ///
    /// - Min: 0
    /// - Max: 100
    /// - Unit: "%"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn throttle(&self) -> u8 {
        self.throttle_raw()
    }
    
    /// Get raw value of throttle
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn throttle_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        signal
    }
    
    /// Set value of throttle
    #[inline(always)]
    pub fn set_throttle(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 100_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 83 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
    /// brake
    ///
    /// % of brake pedal
    ///
    /// - Min: 0
    /// - Max: 100
    /// - Unit: "%"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn brake(&self) -> u8 {
        self.brake_raw()
    }
    
    /// Get raw value of brake
    ///
    /// - Start bit: 8
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn brake_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[8..16].load_le::<u8>();
        
        signal
    }
    
    /// Set value of brake
    #[inline(always)]
    pub fn set_brake(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 100_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 83 });
        }
        self.raw.view_bits_mut::<Lsb0>()[8..16].store_le(value);
        Ok(())
    }
    
    /// steering
    ///
    /// Steering angle in milli radians
    ///
    /// - Min: -120
    /// - Max: 120
    /// - Unit: "deg"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn steering(&self) -> i16 {
        self.steering_raw()
    }
    
    /// Get raw value of steering
    ///
    /// - Start bit: 16
    /// - Signal size: 12 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn steering_raw(&self) -> i16 {
        let signal = self.raw.view_bits::<Lsb0>()[16..28].load_le::<u16>();
        
        let signal  = i16::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of steering
    #[inline(always)]
    pub fn set_steering(&mut self, value: i16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < -120_i16 || 120_i16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 83 });
        }
        let value = u16::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[16..28].store_le(value);
        Ok(())
    }
    
    /// no_implausibility
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn no_implausibility(&self) -> bool {
        self.no_implausibility_raw()
    }
    
    /// Get raw value of no_implausibility
    ///
    /// - Start bit: 28
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn no_implausibility_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[28..29].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of no_implausibility
    #[inline(always)]
    pub fn set_no_implausibility(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[28..29].store_le(value);
        Ok(())
    }
    
    /// bre_implausibility
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn bre_implausibility(&self) -> bool {
        self.bre_implausibility_raw()
    }
    
    /// Get raw value of bre_implausibility
    ///
    /// - Start bit: 29
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn bre_implausibility_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[29..30].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of bre_implausibility
    #[inline(always)]
    pub fn set_bre_implausibility(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[29..30].store_le(value);
        Ok(())
    }
    
    /// pad_implausibility
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn pad_implausibility(&self) -> bool {
        self.pad_implausibility_raw()
    }
    
    /// Get raw value of pad_implausibility
    ///
    /// - Start bit: 30
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn pad_implausibility_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[30..31].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of pad_implausibility
    #[inline(always)]
    pub fn set_pad_implausibility(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[30..31].store_le(value);
        Ok(())
    }
    
    /// pot_implausibility
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn pot_implausibility(&self) -> bool {
        self.pot_implausibility_raw()
    }
    
    /// Get raw value of pot_implausibility
    ///
    /// - Start bit: 31
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn pot_implausibility_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[31..32].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of pot_implausibility
    #[inline(always)]
    pub fn set_pot_implausibility(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[31..32].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Driver {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 4 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 4];
        raw.copy_from_slice(&payload[..4]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Driver {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Driver")
                .field("throttle", &self.throttle())
                .field("brake", &self.brake())
                .field("steering", &self.steering())
                .field("no_implausibility", &self.no_implausibility())
                .field("bre_implausibility", &self.bre_implausibility())
                .field("pad_implausibility", &self.pad_implausibility())
                .field("pot_implausibility", &self.pot_implausibility())
            .finish()
        } else {
            f.debug_tuple("Driver").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Driver {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let throttle = u.int_in_range(0..=100)?;
        let brake = u.int_in_range(0..=100)?;
        let steering = u.int_in_range(-120..=120)?;
        let no_implausibility = u.int_in_range(0..=1)? == 1;
        let bre_implausibility = u.int_in_range(0..=1)? == 1;
        let pad_implausibility = u.int_in_range(0..=1)? == 1;
        let pot_implausibility = u.int_in_range(0..=1)? == 1;
        Driver::new(throttle,brake,steering,no_implausibility,bre_implausibility,pad_implausibility,pot_implausibility).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// BmsLv1
///
/// - ID: 84 (0x54)
/// - Size: 7 bytes
/// - Transmitter: BMSHV
#[derive(Clone, Copy)]
pub struct BmsLv1 {
    raw: [u8; 7],
}

impl BmsLv1 {
    pub const MESSAGE_ID: u32 = 84;
    
    pub const MAX_VOLT_MIN: f32 = 0_f32;
    pub const MAX_VOLT_MAX: f32 = 0_f32;
    pub const MIN_VOLT_MIN: f32 = 0_f32;
    pub const MIN_VOLT_MAX: f32 = 0_f32;
    pub const AVG_VOLT_MIN: f32 = 0_f32;
    pub const AVG_VOLT_MAX: f32 = 0_f32;
    pub const SOC_MIN: u8 = 0_u8;
    pub const SOC_MAX: u8 = 100_u8;
    
    /// Construct new BmsLv1 from values
    pub fn new(max_volt: f32, min_volt: f32, avg_volt: f32, soc: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 7] };
        res.set_max_volt(max_volt)?;
        res.set_min_volt(min_volt)?;
        res.set_avg_volt(avg_volt)?;
        res.set_soc(soc)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 7] {
        &self.raw
    }
    
    /// max_volt
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "mV"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn max_volt(&self) -> f32 {
        self.max_volt_raw()
    }
    
    /// Get raw value of max_volt
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn max_volt_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of max_volt
    #[inline(always)]
    pub fn set_max_volt(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 84 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
    /// min_volt
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "mV"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn min_volt(&self) -> f32 {
        self.min_volt_raw()
    }
    
    /// Get raw value of min_volt
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn min_volt_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of min_volt
    #[inline(always)]
    pub fn set_min_volt(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 84 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// avg_volt
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "mV"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn avg_volt(&self) -> f32 {
        self.avg_volt_raw()
    }
    
    /// Get raw value of avg_volt
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn avg_volt_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of avg_volt
    #[inline(always)]
    pub fn set_avg_volt(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 84 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// soc
    ///
    /// - Min: 0
    /// - Max: 100
    /// - Unit: "%"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn soc(&self) -> u8 {
        self.soc_raw()
    }
    
    /// Get raw value of soc
    ///
    /// - Start bit: 48
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn soc_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[48..56].load_le::<u8>();
        
        signal
    }
    
    /// Set value of soc
    #[inline(always)]
    pub fn set_soc(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 100_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 84 });
        }
        self.raw.view_bits_mut::<Lsb0>()[48..56].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for BmsLv1 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 7 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 7];
        raw.copy_from_slice(&payload[..7]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for BmsLv1 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("BmsLv1")
                .field("max_volt", &self.max_volt())
                .field("min_volt", &self.min_volt())
                .field("avg_volt", &self.avg_volt())
                .field("soc", &self.soc())
            .finish()
        } else {
            f.debug_tuple("BmsLv1").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for BmsLv1 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let max_volt = u.float_in_range(0_f32..=0_f32)?;
        let min_volt = u.float_in_range(0_f32..=0_f32)?;
        let avg_volt = u.float_in_range(0_f32..=0_f32)?;
        let soc = u.int_in_range(0..=100)?;
        BmsLv1::new(max_volt,min_volt,avg_volt,soc).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// BmsLv2
///
/// - ID: 85 (0x55)
/// - Size: 7 bytes
/// - Transmitter: BMSHV
#[derive(Clone, Copy)]
pub struct BmsLv2 {
    raw: [u8; 7],
}

impl BmsLv2 {
    pub const MESSAGE_ID: u32 = 85;
    
    pub const MAX_TEMP_MIN: u16 = 0_u16;
    pub const MAX_TEMP_MAX: u16 = 0_u16;
    pub const MIN_TEMP_MIN: u16 = 0_u16;
    pub const MIN_TEMP_MAX: u16 = 0_u16;
    pub const AVG_TEMP_MIN: u16 = 0_u16;
    pub const AVG_TEMP_MAX: u16 = 0_u16;
    pub const FAN_SPEED_MIN: u8 = 0_u8;
    pub const FAN_SPEED_MAX: u8 = 100_u8;
    
    /// Construct new BmsLv2 from values
    pub fn new(max_temp: u16, min_temp: u16, avg_temp: u16, fan_speed: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 7] };
        res.set_max_temp(max_temp)?;
        res.set_min_temp(min_temp)?;
        res.set_avg_temp(avg_temp)?;
        res.set_fan_speed(fan_speed)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 7] {
        &self.raw
    }
    
    /// max_temp
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn max_temp(&self) -> u16 {
        self.max_temp_raw()
    }
    
    /// Get raw value of max_temp
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn max_temp_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of max_temp
    #[inline(always)]
    pub fn set_max_temp(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 85 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
    /// min_temp
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn min_temp(&self) -> u16 {
        self.min_temp_raw()
    }
    
    /// Get raw value of min_temp
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn min_temp_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        signal
    }
    
    /// Set value of min_temp
    #[inline(always)]
    pub fn set_min_temp(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 85 });
        }
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// avg_temp
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn avg_temp(&self) -> u16 {
        self.avg_temp_raw()
    }
    
    /// Get raw value of avg_temp
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn avg_temp_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of avg_temp
    #[inline(always)]
    pub fn set_avg_temp(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 85 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// fan_speed
    ///
    /// - Min: 0
    /// - Max: 100
    /// - Unit: "%"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn fan_speed(&self) -> u8 {
        self.fan_speed_raw()
    }
    
    /// Get raw value of fan_speed
    ///
    /// - Start bit: 48
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn fan_speed_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[48..56].load_le::<u8>();
        
        signal
    }
    
    /// Set value of fan_speed
    #[inline(always)]
    pub fn set_fan_speed(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 100_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 85 });
        }
        self.raw.view_bits_mut::<Lsb0>()[48..56].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for BmsLv2 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 7 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 7];
        raw.copy_from_slice(&payload[..7]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for BmsLv2 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("BmsLv2")
                .field("max_temp", &self.max_temp())
                .field("min_temp", &self.min_temp())
                .field("avg_temp", &self.avg_temp())
                .field("fan_speed", &self.fan_speed())
            .finish()
        } else {
            f.debug_tuple("BmsLv2").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for BmsLv2 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let max_temp = u.int_in_range(0..=0)?;
        let min_temp = u.int_in_range(0..=0)?;
        let avg_temp = u.int_in_range(0..=0)?;
        let fan_speed = u.int_in_range(0..=100)?;
        BmsLv2::new(max_temp,min_temp,avg_temp,fan_speed).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// BmsHv1
///
/// - ID: 87 (0x57)
/// - Size: 7 bytes
/// - Transmitter: BMSHV
#[derive(Clone, Copy)]
pub struct BmsHv1 {
    raw: [u8; 7],
}

impl BmsHv1 {
    pub const MESSAGE_ID: u32 = 87;
    
    pub const MAX_VOLT_MIN: f32 = 0_f32;
    pub const MAX_VOLT_MAX: f32 = 0_f32;
    pub const MIN_VOLT_MIN: f32 = 0_f32;
    pub const MIN_VOLT_MAX: f32 = 0_f32;
    pub const AVG_VOLT_MIN: f32 = 0_f32;
    pub const AVG_VOLT_MAX: f32 = 0_f32;
    pub const SOC_MIN: u8 = 0_u8;
    pub const SOC_MAX: u8 = 100_u8;
    
    /// Construct new BmsHv1 from values
    pub fn new(max_volt: f32, min_volt: f32, avg_volt: f32, soc: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 7] };
        res.set_max_volt(max_volt)?;
        res.set_min_volt(min_volt)?;
        res.set_avg_volt(avg_volt)?;
        res.set_soc(soc)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 7] {
        &self.raw
    }
    
    /// max_volt
    ///
    /// Maximum cell voltage in mv
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "mV"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn max_volt(&self) -> f32 {
        self.max_volt_raw()
    }
    
    /// Get raw value of max_volt
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn max_volt_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of max_volt
    #[inline(always)]
    pub fn set_max_volt(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 87 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
    /// min_volt
    ///
    /// Minimum cell voltage in mv
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "mV"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn min_volt(&self) -> f32 {
        self.min_volt_raw()
    }
    
    /// Get raw value of min_volt
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn min_volt_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of min_volt
    #[inline(always)]
    pub fn set_min_volt(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 87 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// avg_volt
    ///
    /// Average cell voltage in mv
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "mV"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn avg_volt(&self) -> f32 {
        self.avg_volt_raw()
    }
    
    /// Get raw value of avg_volt
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn avg_volt_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of avg_volt
    #[inline(always)]
    pub fn set_avg_volt(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 87 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// soc
    ///
    /// HV battery SOC in %
    ///
    /// - Min: 0
    /// - Max: 100
    /// - Unit: "%"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn soc(&self) -> u8 {
        self.soc_raw()
    }
    
    /// Get raw value of soc
    ///
    /// - Start bit: 48
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn soc_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[48..56].load_le::<u8>();
        
        signal
    }
    
    /// Set value of soc
    #[inline(always)]
    pub fn set_soc(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 100_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 87 });
        }
        self.raw.view_bits_mut::<Lsb0>()[48..56].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for BmsHv1 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 7 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 7];
        raw.copy_from_slice(&payload[..7]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for BmsHv1 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("BmsHv1")
                .field("max_volt", &self.max_volt())
                .field("min_volt", &self.min_volt())
                .field("avg_volt", &self.avg_volt())
                .field("soc", &self.soc())
            .finish()
        } else {
            f.debug_tuple("BmsHv1").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for BmsHv1 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let max_volt = u.float_in_range(0_f32..=0_f32)?;
        let min_volt = u.float_in_range(0_f32..=0_f32)?;
        let avg_volt = u.float_in_range(0_f32..=0_f32)?;
        let soc = u.int_in_range(0..=100)?;
        BmsHv1::new(max_volt,min_volt,avg_volt,soc).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// BmsHv2
///
/// - ID: 88 (0x58)
/// - Size: 7 bytes
/// - Transmitter: BMSHV
#[derive(Clone, Copy)]
pub struct BmsHv2 {
    raw: [u8; 7],
}

impl BmsHv2 {
    pub const MESSAGE_ID: u32 = 88;
    
    pub const MAX_TEMP_MIN: u16 = 0_u16;
    pub const MAX_TEMP_MAX: u16 = 0_u16;
    pub const MIN_TEMP_MIN: u16 = 0_u16;
    pub const MIN_TEMP_MAX: u16 = 0_u16;
    pub const AVG_TEMP_MIN: u16 = 0_u16;
    pub const AVG_TEMP_MAX: u16 = 0_u16;
    pub const FAN_SPEED_MIN: u8 = 0_u8;
    pub const FAN_SPEED_MAX: u8 = 100_u8;
    
    /// Construct new BmsHv2 from values
    pub fn new(max_temp: u16, min_temp: u16, avg_temp: u16, fan_speed: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 7] };
        res.set_max_temp(max_temp)?;
        res.set_min_temp(min_temp)?;
        res.set_avg_temp(avg_temp)?;
        res.set_fan_speed(fan_speed)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 7] {
        &self.raw
    }
    
    /// max_temp
    ///
    /// Maximum cell temperature in celsius
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn max_temp(&self) -> u16 {
        self.max_temp_raw()
    }
    
    /// Get raw value of max_temp
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn max_temp_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of max_temp
    #[inline(always)]
    pub fn set_max_temp(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 88 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
    /// min_temp
    ///
    /// Minimum cell temperature in celsius
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn min_temp(&self) -> u16 {
        self.min_temp_raw()
    }
    
    /// Get raw value of min_temp
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn min_temp_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        signal
    }
    
    /// Set value of min_temp
    #[inline(always)]
    pub fn set_min_temp(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 88 });
        }
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// avg_temp
    ///
    /// Average cell temperature in celsius
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn avg_temp(&self) -> u16 {
        self.avg_temp_raw()
    }
    
    /// Get raw value of avg_temp
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn avg_temp_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of avg_temp
    #[inline(always)]
    pub fn set_avg_temp(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 88 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// fan_speed
    ///
    /// % of HV battery fan speed
    ///
    /// - Min: 0
    /// - Max: 100
    /// - Unit: "%"
    /// - Receivers: VCU, SW
    #[inline(always)]
    pub fn fan_speed(&self) -> u8 {
        self.fan_speed_raw()
    }
    
    /// Get raw value of fan_speed
    ///
    /// - Start bit: 48
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn fan_speed_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[48..56].load_le::<u8>();
        
        signal
    }
    
    /// Set value of fan_speed
    #[inline(always)]
    pub fn set_fan_speed(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 100_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 88 });
        }
        self.raw.view_bits_mut::<Lsb0>()[48..56].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for BmsHv2 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 7 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 7];
        raw.copy_from_slice(&payload[..7]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for BmsHv2 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("BmsHv2")
                .field("max_temp", &self.max_temp())
                .field("min_temp", &self.min_temp())
                .field("avg_temp", &self.avg_temp())
                .field("fan_speed", &self.fan_speed())
            .finish()
        } else {
            f.debug_tuple("BmsHv2").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for BmsHv2 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let max_temp = u.int_in_range(0..=0)?;
        let min_temp = u.int_in_range(0..=0)?;
        let avg_temp = u.int_in_range(0..=0)?;
        let fan_speed = u.int_in_range(0..=100)?;
        BmsHv2::new(max_temp,min_temp,avg_temp,fan_speed).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// Imu1
///
/// - ID: 96 (0x60)
/// - Size: 8 bytes
/// - Transmitter: SMU
#[derive(Clone, Copy)]
pub struct Imu1 {
    raw: [u8; 8],
}

impl Imu1 {
    pub const MESSAGE_ID: u32 = 96;
    
    pub const ACC_X_MIN: i32 = 0_i32;
    pub const ACC_X_MAX: i32 = 0_i32;
    pub const ACC_Y_MIN: i32 = 0_i32;
    pub const ACC_Y_MAX: i32 = 0_i32;
    
    /// Construct new Imu1 from values
    pub fn new(acc_x: i32, acc_y: i32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_acc_x(acc_x)?;
        res.set_acc_y(acc_y)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// acc_x
    ///
    /// Acceleration on x axis in m/s^2
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "m/s^2"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn acc_x(&self) -> i32 {
        self.acc_x_raw()
    }
    
    /// Get raw value of acc_x
    ///
    /// - Start bit: 0
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn acc_x_raw(&self) -> i32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..32].load_le::<u32>();
        
        let signal  = i32::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of acc_x
    #[inline(always)]
    pub fn set_acc_x(&mut self, value: i32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_i32 || 0_i32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 96 });
        }
        let value = u32::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..32].store_le(value);
        Ok(())
    }
    
    /// acc_y
    ///
    /// Acceleration on y axis in m/s^2
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "m/s^2"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn acc_y(&self) -> i32 {
        self.acc_y_raw()
    }
    
    /// Get raw value of acc_y
    ///
    /// - Start bit: 32
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn acc_y_raw(&self) -> i32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..64].load_le::<u32>();
        
        let signal  = i32::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of acc_y
    #[inline(always)]
    pub fn set_acc_y(&mut self, value: i32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_i32 || 0_i32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 96 });
        }
        let value = u32::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..64].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Imu1 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Imu1 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Imu1")
                .field("acc_x", &self.acc_x())
                .field("acc_y", &self.acc_y())
            .finish()
        } else {
            f.debug_tuple("Imu1").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Imu1 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let acc_x = u.int_in_range(0..=0)?;
        let acc_y = u.int_in_range(0..=0)?;
        Imu1::new(acc_x,acc_y).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// Imu2
///
/// - ID: 97 (0x61)
/// - Size: 8 bytes
/// - Transmitter: SMU
#[derive(Clone, Copy)]
pub struct Imu2 {
    raw: [u8; 8],
}

impl Imu2 {
    pub const MESSAGE_ID: u32 = 97;
    
    pub const ACC_Z_MIN: i32 = 0_i32;
    pub const ACC_Z_MAX: i32 = 0_i32;
    pub const OMEGA_X_MIN: i32 = 0_i32;
    pub const OMEGA_X_MAX: i32 = 0_i32;
    
    /// Construct new Imu2 from values
    pub fn new(acc_z: i32, omega_x: i32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_acc_z(acc_z)?;
        res.set_omega_x(omega_x)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// acc_z
    ///
    /// Acceleration on z axis in m/s^2
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "m/s^2"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn acc_z(&self) -> i32 {
        self.acc_z_raw()
    }
    
    /// Get raw value of acc_z
    ///
    /// - Start bit: 0
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn acc_z_raw(&self) -> i32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..32].load_le::<u32>();
        
        let signal  = i32::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of acc_z
    #[inline(always)]
    pub fn set_acc_z(&mut self, value: i32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_i32 || 0_i32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 97 });
        }
        let value = u32::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..32].store_le(value);
        Ok(())
    }
    
    /// omega_x
    ///
    /// Angular speed on x axis in rad/s
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "rad/s"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn omega_x(&self) -> i32 {
        self.omega_x_raw()
    }
    
    /// Get raw value of omega_x
    ///
    /// - Start bit: 32
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn omega_x_raw(&self) -> i32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..64].load_le::<u32>();
        
        let signal  = i32::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of omega_x
    #[inline(always)]
    pub fn set_omega_x(&mut self, value: i32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_i32 || 0_i32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 97 });
        }
        let value = u32::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..64].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Imu2 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Imu2 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Imu2")
                .field("acc_z", &self.acc_z())
                .field("omega_x", &self.omega_x())
            .finish()
        } else {
            f.debug_tuple("Imu2").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Imu2 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let acc_z = u.int_in_range(0..=0)?;
        let omega_x = u.int_in_range(0..=0)?;
        Imu2::new(acc_z,omega_x).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// Imu3
///
/// - ID: 98 (0x62)
/// - Size: 8 bytes
/// - Transmitter: SMU
#[derive(Clone, Copy)]
pub struct Imu3 {
    raw: [u8; 8],
}

impl Imu3 {
    pub const MESSAGE_ID: u32 = 98;
    
    pub const OMEGA_Y_MIN: i32 = 0_i32;
    pub const OMEGA_Y_MAX: i32 = 0_i32;
    pub const OMEGA_Z_MIN: i32 = 0_i32;
    pub const OMEGA_Z_MAX: i32 = 0_i32;
    
    /// Construct new Imu3 from values
    pub fn new(omega_y: i32, omega_z: i32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_omega_y(omega_y)?;
        res.set_omega_z(omega_z)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// omega_y
    ///
    /// Angular speed on x axis in rad/s
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "rad/s"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn omega_y(&self) -> i32 {
        self.omega_y_raw()
    }
    
    /// Get raw value of omega_y
    ///
    /// - Start bit: 0
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn omega_y_raw(&self) -> i32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..32].load_le::<u32>();
        
        let signal  = i32::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of omega_y
    #[inline(always)]
    pub fn set_omega_y(&mut self, value: i32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_i32 || 0_i32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 98 });
        }
        let value = u32::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[0..32].store_le(value);
        Ok(())
    }
    
    /// omega_z
    ///
    /// Angular speed on x axis in rad/s
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "rad/s"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn omega_z(&self) -> i32 {
        self.omega_z_raw()
    }
    
    /// Get raw value of omega_z
    ///
    /// - Start bit: 32
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn omega_z_raw(&self) -> i32 {
        let signal = self.raw.view_bits::<Lsb0>()[32..64].load_le::<u32>();
        
        let signal  = i32::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of omega_z
    #[inline(always)]
    pub fn set_omega_z(&mut self, value: i32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_i32 || 0_i32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 98 });
        }
        let value = u32::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..64].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Imu3 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Imu3 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Imu3")
                .field("omega_y", &self.omega_y())
                .field("omega_z", &self.omega_z())
            .finish()
        } else {
            f.debug_tuple("Imu3").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Imu3 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let omega_y = u.int_in_range(0..=0)?;
        let omega_z = u.int_in_range(0..=0)?;
        Imu3::new(omega_y,omega_z).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// IMUCalib
///
/// - ID: 99 (0x63)
/// - Size: 1 bytes
/// - Transmitter: SMU
///
/// RESERVER FOR IMU mask - DO NOT USE
#[derive(Clone, Copy)]
pub struct ImuCalib {
    raw: [u8; 1],
}

impl ImuCalib {
    pub const MESSAGE_ID: u32 = 99;
    
    
    /// Construct new IMUCalib from values
    pub fn new(start_imu_calibration: bool) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 1] };
        res.set_start_imu_calibration(start_imu_calibration)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 1] {
        &self.raw
    }
    
    /// start_imu_calibration
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn start_imu_calibration(&self) -> bool {
        self.start_imu_calibration_raw()
    }
    
    /// Get raw value of start_imu_calibration
    ///
    /// - Start bit: 0
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn start_imu_calibration_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[0..1].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of start_imu_calibration
    #[inline(always)]
    pub fn set_start_imu_calibration(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[0..1].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for ImuCalib {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 1 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 1];
        raw.copy_from_slice(&payload[..1]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for ImuCalib {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("ImuCalib")
                .field("start_imu_calibration", &self.start_imu_calibration())
            .finish()
        } else {
            f.debug_tuple("ImuCalib").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for ImuCalib {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let start_imu_calibration = u.int_in_range(0..=1)? == 1;
        ImuCalib::new(start_imu_calibration).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// Map
///
/// - ID: 100 (0x64)
/// - Size: 2 bytes
/// - Transmitter: SW
#[derive(Clone, Copy)]
pub struct Map {
    raw: [u8; 2],
}

impl Map {
    pub const MESSAGE_ID: u32 = 100;
    
    pub const POWER_MIN: u8 = 1_u8;
    pub const POWER_MAX: u8 = 12_u8;
    pub const REGEN_MIN: u8 = 1_u8;
    pub const REGEN_MAX: u8 = 12_u8;
    pub const TORQUE_REP_MIN: u8 = 0_u8;
    pub const TORQUE_REP_MAX: u8 = 12_u8;
    
    /// Construct new Map from values
    pub fn new(power: u8, regen: u8, torque_rep: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_power(power)?;
        res.set_regen(regen)?;
        res.set_torque_rep(torque_rep)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// power
    ///
    /// Map selected number
    ///
    /// - Min: 1
    /// - Max: 12
    /// - Unit: "map"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn power(&self) -> u8 {
        self.power_raw()
    }
    
    /// Get raw value of power
    ///
    /// - Start bit: 0
    /// - Signal size: 4 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn power_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..4].load_le::<u8>();
        
        signal
    }
    
    /// Set value of power
    #[inline(always)]
    pub fn set_power(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 1_u8 || 12_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 100 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..4].store_le(value);
        Ok(())
    }
    
    /// regen
    ///
    /// Map selected for regen braking
    ///
    /// - Min: 1
    /// - Max: 12
    /// - Unit: "map"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn regen(&self) -> u8 {
        self.regen_raw()
    }
    
    /// Get raw value of regen
    ///
    /// - Start bit: 4
    /// - Signal size: 4 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn regen_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[4..8].load_le::<u8>();
        
        signal
    }
    
    /// Set value of regen
    #[inline(always)]
    pub fn set_regen(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 1_u8 || 12_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 100 });
        }
        self.raw.view_bits_mut::<Lsb0>()[4..8].store_le(value);
        Ok(())
    }
    
    /// torque_rep
    ///
    /// Map selected for torque repeartition
    ///
    /// - Min: 0
    /// - Max: 12
    /// - Unit: "map"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn torque_rep(&self) -> u8 {
        self.torque_rep_raw()
    }
    
    /// Get raw value of torque_rep
    ///
    /// - Start bit: 8
    /// - Signal size: 4 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn torque_rep_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[8..12].load_le::<u8>();
        
        signal
    }
    
    /// Set value of torque_rep
    #[inline(always)]
    pub fn set_torque_rep(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 12_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 100 });
        }
        self.raw.view_bits_mut::<Lsb0>()[8..12].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Map {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Map {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Map")
                .field("power", &self.power())
                .field("regen", &self.regen())
                .field("torque_rep", &self.torque_rep())
            .finish()
        } else {
            f.debug_tuple("Map").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Map {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let power = u.int_in_range(1..=12)?;
        let regen = u.int_in_range(1..=12)?;
        let torque_rep = u.int_in_range(0..=12)?;
        Map::new(power,regen,torque_rep).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// CarStatus
///
/// - ID: 101 (0x65)
/// - Size: 2 bytes
/// - Transmitter: VCU
#[derive(Clone, Copy)]
pub struct CarStatus {
    raw: [u8; 2],
}

impl CarStatus {
    pub const MESSAGE_ID: u32 = 101;
    
    pub const SPEED_MIN: u8 = 0_u8;
    pub const SPEED_MAX: u8 = 0_u8;
    
    /// Construct new CarStatus from values
    pub fn new(hv: bool, r2d: bool, air1: bool, rf: bool, air2: bool, precharge: bool, speed: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_hv(hv)?;
        res.set_r2d(r2d)?;
        res.set_air1(air1)?;
        res.set_rf(rf)?;
        res.set_air2(air2)?;
        res.set_precharge(precharge)?;
        res.set_speed(speed)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// HV
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "on"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn hv(&self) -> bool {
        self.hv_raw()
    }
    
    /// Get raw value of HV
    ///
    /// - Start bit: 0
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn hv_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[0..1].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of HV
    #[inline(always)]
    pub fn set_hv(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[0..1].store_le(value);
        Ok(())
    }
    
    /// R2D
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "on"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn r2d(&self) -> bool {
        self.r2d_raw()
    }
    
    /// Get raw value of R2D
    ///
    /// - Start bit: 1
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn r2d_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[1..2].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of R2D
    #[inline(always)]
    pub fn set_r2d(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[1..2].store_le(value);
        Ok(())
    }
    
    /// AIR1
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "closed"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn air1(&self) -> bool {
        self.air1_raw()
    }
    
    /// Get raw value of AIR1
    ///
    /// - Start bit: 3
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn air1_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[3..4].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of AIR1
    #[inline(always)]
    pub fn set_air1(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[3..4].store_le(value);
        Ok(())
    }
    
    /// RF
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "enabled"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn rf(&self) -> bool {
        self.rf_raw()
    }
    
    /// Get raw value of RF
    ///
    /// - Start bit: 2
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn rf_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[2..3].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of RF
    #[inline(always)]
    pub fn set_rf(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[2..3].store_le(value);
        Ok(())
    }
    
    /// AIR2
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "closed"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn air2(&self) -> bool {
        self.air2_raw()
    }
    
    /// Get raw value of AIR2
    ///
    /// - Start bit: 4
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn air2_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[4..5].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of AIR2
    #[inline(always)]
    pub fn set_air2(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[4..5].store_le(value);
        Ok(())
    }
    
    /// precharge
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "done"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn precharge(&self) -> bool {
        self.precharge_raw()
    }
    
    /// Get raw value of precharge
    ///
    /// - Start bit: 5
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn precharge_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[5..6].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of precharge
    #[inline(always)]
    pub fn set_precharge(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[5..6].store_le(value);
        Ok(())
    }
    
    /// speed
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "km/h"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn speed(&self) -> u8 {
        self.speed_raw()
    }
    
    /// Get raw value of speed
    ///
    /// - Start bit: 8
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn speed_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[8..16].load_le::<u8>();
        
        signal
    }
    
    /// Set value of speed
    #[inline(always)]
    pub fn set_speed(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 0_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 101 });
        }
        self.raw.view_bits_mut::<Lsb0>()[8..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CarStatus {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CarStatus {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CarStatus")
                .field("hv", &self.hv())
                .field("r2d", &self.r2d())
                .field("air1", &self.air1())
                .field("rf", &self.rf())
                .field("air2", &self.air2())
                .field("precharge", &self.precharge())
                .field("speed", &self.speed())
            .finish()
        } else {
            f.debug_tuple("CarStatus").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CarStatus {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let hv = u.int_in_range(0..=1)? == 1;
        let r2d = u.int_in_range(0..=1)? == 1;
        let air1 = u.int_in_range(0..=1)? == 1;
        let rf = u.int_in_range(0..=1)? == 1;
        let air2 = u.int_in_range(0..=1)? == 1;
        let precharge = u.int_in_range(0..=1)? == 1;
        let speed = u.int_in_range(0..=0)?;
        CarStatus::new(hv,r2d,air1,rf,air2,precharge,speed).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// CarSettings
///
/// - ID: 102 (0x66)
/// - Size: 8 bytes
/// - Transmitter: VCU
#[derive(Clone, Copy)]
pub struct CarSettings {
    raw: [u8; 8],
}

impl CarSettings {
    pub const MESSAGE_ID: u32 = 102;
    
    pub const MAX_REGEN_CURRENT_MIN: u8 = 0_u8;
    pub const MAX_REGEN_CURRENT_MAX: u8 = 150_u8;
    pub const PWR_LIMIT_MIN: u8 = 0_u8;
    pub const PWR_LIMIT_MAX: u8 = 80_u8;
    pub const SPEED_LIM_MIN: u8 = 0_u8;
    pub const SPEED_LIM_MAX: u8 = 0_u8;
    pub const MAX_POS_TRQ_MIN: u8 = 0_u8;
    pub const MAX_POS_TRQ_MAX: u8 = 0_u8;
    pub const MAX_NEG_TRQ_MIN: i8 = 0_i8;
    pub const MAX_NEG_TRQ_MAX: i8 = 0_i8;
    pub const FRONT_MOTOR_REPARTITION_MIN: u8 = 0_u8;
    pub const FRONT_MOTOR_REPARTITION_MAX: u8 = 0_u8;
    pub const REAR_MOTOR_REPARTITION_MIN: u8 = 0_u8;
    pub const REAR_MOTOR_REPARTITION_MAX: u8 = 0_u8;
    
    /// Construct new CarSettings from values
    pub fn new(max_regen_current: u8, pwr_limit: u8, speed_lim: u8, max_pos_trq: u8, max_neg_trq: i8, front_motor_repartition: u8, rear_motor_repartition: u8, torque_vectoring: bool) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_max_regen_current(max_regen_current)?;
        res.set_pwr_limit(pwr_limit)?;
        res.set_speed_lim(speed_lim)?;
        res.set_max_pos_trq(max_pos_trq)?;
        res.set_max_neg_trq(max_neg_trq)?;
        res.set_front_motor_repartition(front_motor_repartition)?;
        res.set_rear_motor_repartition(rear_motor_repartition)?;
        res.set_torque_vectoring(torque_vectoring)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// max_regen_current
    ///
    /// Maximum Regen Current
    ///
    /// - Min: 0
    /// - Max: 150
    /// - Unit: "A"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn max_regen_current(&self) -> u8 {
        self.max_regen_current_raw()
    }
    
    /// Get raw value of max_regen_current
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn max_regen_current_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        signal
    }
    
    /// Set value of max_regen_current
    #[inline(always)]
    pub fn set_max_regen_current(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 150_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 102 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
    /// pwr_limit
    ///
    /// Maximum power limit
    ///
    /// - Min: 0
    /// - Max: 80
    /// - Unit: "kW"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn pwr_limit(&self) -> u8 {
        self.pwr_limit_raw()
    }
    
    /// Get raw value of pwr_limit
    ///
    /// - Start bit: 8
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn pwr_limit_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[8..16].load_le::<u8>();
        
        signal
    }
    
    /// Set value of pwr_limit
    #[inline(always)]
    pub fn set_pwr_limit(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 80_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 102 });
        }
        self.raw.view_bits_mut::<Lsb0>()[8..16].store_le(value);
        Ok(())
    }
    
    /// speed_lim
    ///
    /// Maximum Speed Limit
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "krpm"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn speed_lim(&self) -> u8 {
        self.speed_lim_raw()
    }
    
    /// Get raw value of speed_lim
    ///
    /// - Start bit: 16
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn speed_lim_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[16..24].load_le::<u8>();
        
        signal
    }
    
    /// Set value of speed_lim
    #[inline(always)]
    pub fn set_speed_lim(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 0_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 102 });
        }
        self.raw.view_bits_mut::<Lsb0>()[16..24].store_le(value);
        Ok(())
    }
    
    /// max_pos_trq
    ///
    /// Maximum Positive Torque
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Nm"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn max_pos_trq(&self) -> u8 {
        self.max_pos_trq_raw()
    }
    
    /// Get raw value of max_pos_trq
    ///
    /// - Start bit: 24
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn max_pos_trq_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[24..32].load_le::<u8>();
        
        signal
    }
    
    /// Set value of max_pos_trq
    #[inline(always)]
    pub fn set_max_pos_trq(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 0_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 102 });
        }
        self.raw.view_bits_mut::<Lsb0>()[24..32].store_le(value);
        Ok(())
    }
    
    /// max_neg_trq
    ///
    /// Maximum Negative Torque
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "Nm"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn max_neg_trq(&self) -> i8 {
        self.max_neg_trq_raw()
    }
    
    /// Get raw value of max_neg_trq
    ///
    /// - Start bit: 32
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Signed
    #[inline(always)]
    pub fn max_neg_trq_raw(&self) -> i8 {
        let signal = self.raw.view_bits::<Lsb0>()[32..40].load_le::<u8>();
        
        let signal  = i8::from_ne_bytes(signal.to_ne_bytes());
        signal
    }
    
    /// Set value of max_neg_trq
    #[inline(always)]
    pub fn set_max_neg_trq(&mut self, value: i8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_i8 || 0_i8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 102 });
        }
        let value = u8::from_ne_bytes(value.to_ne_bytes());
        self.raw.view_bits_mut::<Lsb0>()[32..40].store_le(value);
        Ok(())
    }
    
    /// front_motor_repartition
    ///
    /// Torque Repartition Front
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "%"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn front_motor_repartition(&self) -> u8 {
        self.front_motor_repartition_raw()
    }
    
    /// Get raw value of front_motor_repartition
    ///
    /// - Start bit: 40
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn front_motor_repartition_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[40..48].load_le::<u8>();
        
        signal
    }
    
    /// Set value of front_motor_repartition
    #[inline(always)]
    pub fn set_front_motor_repartition(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 0_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 102 });
        }
        self.raw.view_bits_mut::<Lsb0>()[40..48].store_le(value);
        Ok(())
    }
    
    /// rear_motor_repartition
    ///
    /// Torque Repartition Rear
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "%"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn rear_motor_repartition(&self) -> u8 {
        self.rear_motor_repartition_raw()
    }
    
    /// Get raw value of rear_motor_repartition
    ///
    /// - Start bit: 48
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn rear_motor_repartition_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[48..56].load_le::<u8>();
        
        signal
    }
    
    /// Set value of rear_motor_repartition
    #[inline(always)]
    pub fn set_rear_motor_repartition(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 0_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 102 });
        }
        self.raw.view_bits_mut::<Lsb0>()[48..56].store_le(value);
        Ok(())
    }
    
    /// torque_vectoring
    ///
    /// Torque Vectoring enabled
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "on"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn torque_vectoring(&self) -> bool {
        self.torque_vectoring_raw()
    }
    
    /// Get raw value of torque_vectoring
    ///
    /// - Start bit: 56
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn torque_vectoring_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[56..57].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of torque_vectoring
    #[inline(always)]
    pub fn set_torque_vectoring(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[56..57].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CarSettings {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CarSettings {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CarSettings")
                .field("max_regen_current", &self.max_regen_current())
                .field("pwr_limit", &self.pwr_limit())
                .field("speed_lim", &self.speed_lim())
                .field("max_pos_trq", &self.max_pos_trq())
                .field("max_neg_trq", &self.max_neg_trq())
                .field("front_motor_repartition", &self.front_motor_repartition())
                .field("rear_motor_repartition", &self.rear_motor_repartition())
                .field("torque_vectoring", &self.torque_vectoring())
            .finish()
        } else {
            f.debug_tuple("CarSettings").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CarSettings {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let max_regen_current = u.int_in_range(0..=150)?;
        let pwr_limit = u.int_in_range(0..=80)?;
        let speed_lim = u.int_in_range(0..=0)?;
        let max_pos_trq = u.int_in_range(0..=0)?;
        let max_neg_trq = u.int_in_range(0..=0)?;
        let front_motor_repartition = u.int_in_range(0..=0)?;
        let rear_motor_repartition = u.int_in_range(0..=0)?;
        let torque_vectoring = u.int_in_range(0..=1)? == 1;
        CarSettings::new(max_regen_current,pwr_limit,speed_lim,max_pos_trq,max_neg_trq,front_motor_repartition,rear_motor_repartition,torque_vectoring).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// CarMission
///
/// - ID: 103 (0x67)
/// - Size: 1 bytes
/// - Transmitter: SW
#[derive(Clone, Copy)]
pub struct CarMission {
    raw: [u8; 1],
}

impl CarMission {
    pub const MESSAGE_ID: u32 = 103;
    
    pub const MISSION_MIN: u8 = 0_u8;
    pub const MISSION_MAX: u8 = 7_u8;
    
    /// Construct new CarMission from values
    pub fn new(mission: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 1] };
        res.set_mission(mission)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 1] {
        &self.raw
    }
    
    /// Mission
    ///
    /// - Min: 0
    /// - Max: 7
    /// - Unit: ""
    /// - Receivers: VCU
    #[inline(always)]
    pub fn mission(&self) -> CarMissionMission {
        let signal = self.raw.view_bits::<Lsb0>()[0..3].load_le::<u8>();
        
        match signal {
            7 => CarMissionMission::DvInspection,
            6 => CarMissionMission::DvEbsTest,
            5 => CarMissionMission::DvTrackdrive,
            4 => CarMissionMission::DvAutocross,
            3 => CarMissionMission::DvSkidpad,
            2 => CarMissionMission::DvAcceleration,
            1 => CarMissionMission::Manualy,
            0 => CarMissionMission::None,
            _ => CarMissionMission::_Other(self.mission_raw()),
        }
    }
    
    /// Get raw value of Mission
    ///
    /// - Start bit: 0
    /// - Signal size: 3 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn mission_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..3].load_le::<u8>();
        
        signal
    }
    
    /// Set value of Mission
    #[inline(always)]
    pub fn set_mission(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 7_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 103 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..3].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CarMission {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 1 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 1];
        raw.copy_from_slice(&payload[..1]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CarMission {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CarMission")
                .field("mission", &self.mission())
            .finish()
        } else {
            f.debug_tuple("CarMission").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CarMission {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let mission = u.int_in_range(0..=7)?;
        CarMission::new(mission).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}
/// Defined values for Mission
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum CarMissionMission {
    DvInspection,
    DvEbsTest,
    DvTrackdrive,
    DvAutocross,
    DvSkidpad,
    DvAcceleration,
    Manualy,
    None,
    _Other(u8),
}

impl From<CarMissionMission> for u8 {
    fn from(val: CarMissionMission) -> u8 {
        match val {
            CarMissionMission::DvInspection => 7,
            CarMissionMission::DvEbsTest => 6,
            CarMissionMission::DvTrackdrive => 5,
            CarMissionMission::DvAutocross => 4,
            CarMissionMission::DvSkidpad => 3,
            CarMissionMission::DvAcceleration => 2,
            CarMissionMission::Manualy => 1,
            CarMissionMission::None => 0,
            CarMissionMission::_Other(x) => x,
        }
    }
}


/// CheckASB
///
/// - ID: 104 (0x68)
/// - Size: 1 bytes
#[derive(Clone, Copy)]
pub struct CheckAsb {
    raw: [u8; 1],
}

impl CheckAsb {
    pub const MESSAGE_ID: u32 = 104;
    
    pub const RESPONSE_STATUS_MIN: u8 = 0_u8;
    pub const RESPONSE_STATUS_MAX: u8 = 2_u8;
    
    /// Construct new CheckASB from values
    pub fn new(response_status: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 1] };
        res.set_response_status(response_status)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 1] {
        &self.raw
    }
    
    /// response_status
    ///
    /// - Min: 0
    /// - Max: 2
    /// - Unit: ""
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn response_status(&self) -> CheckAsbResponseStatus {
        let signal = self.raw.view_bits::<Lsb0>()[0..3].load_le::<u8>();
        
        match signal {
            2 => CheckAsbResponseStatus::Error,
            1 => CheckAsbResponseStatus::Failure,
            0 => CheckAsbResponseStatus::Success,
            _ => CheckAsbResponseStatus::_Other(self.response_status_raw()),
        }
    }
    
    /// Get raw value of response_status
    ///
    /// - Start bit: 0
    /// - Signal size: 3 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn response_status_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..3].load_le::<u8>();
        
        signal
    }
    
    /// Set value of response_status
    #[inline(always)]
    pub fn set_response_status(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 2_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 104 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..3].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CheckAsb {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 1 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 1];
        raw.copy_from_slice(&payload[..1]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CheckAsb {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CheckAsb")
                .field("response_status", &self.response_status())
            .finish()
        } else {
            f.debug_tuple("CheckAsb").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CheckAsb {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let response_status = u.int_in_range(0..=2)?;
        CheckAsb::new(response_status).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}
/// Defined values for response_status
#[derive(Clone, Copy, PartialEq)]
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum CheckAsbResponseStatus {
    Error,
    Failure,
    Success,
    _Other(u8),
}

impl From<CheckAsbResponseStatus> for u8 {
    fn from(val: CheckAsbResponseStatus) -> u8 {
        match val {
            CheckAsbResponseStatus::Error => 2,
            CheckAsbResponseStatus::Failure => 1,
            CheckAsbResponseStatus::Success => 0,
            CheckAsbResponseStatus::_Other(x) => x,
        }
    }
}


/// LapStart
///
/// - ID: 112 (0x70)
/// - Size: 1 bytes
/// - Transmitter: SW
#[derive(Clone, Copy)]
pub struct LapStart {
    raw: [u8; 1],
}

impl LapStart {
    pub const MESSAGE_ID: u32 = 112;
    
    pub const START_MIN: u8 = 0_u8;
    pub const START_MAX: u8 = 1_u8;
    
    /// Construct new LapStart from values
    pub fn new(start: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 1] };
        res.set_start(start)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 1] {
        &self.raw
    }
    
    /// start
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "start"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn start(&self) -> u8 {
        self.start_raw()
    }
    
    /// Get raw value of start
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn start_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        signal
    }
    
    /// Set value of start
    #[inline(always)]
    pub fn set_start(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 1_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 112 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for LapStart {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 1 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 1];
        raw.copy_from_slice(&payload[..1]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for LapStart {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("LapStart")
                .field("start", &self.start())
            .finish()
        } else {
            f.debug_tuple("LapStart").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for LapStart {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let start = u.int_in_range(0..=1)?;
        LapStart::new(start).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// Temp1
///
/// - ID: 256 (0x100)
/// - Size: 8 bytes
/// - Transmitter: SMU
#[derive(Clone, Copy)]
pub struct Temp1 {
    raw: [u8; 8],
}

impl Temp1 {
    pub const MESSAGE_ID: u32 = 256;
    
    pub const TEMP_MOTOR_POST_L_MIN: u16 = 0_u16;
    pub const TEMP_MOTOR_POST_L_MAX: u16 = 0_u16;
    pub const TEMP_MOTOR_PRE_L_MIN: u16 = 0_u16;
    pub const TEMP_MOTOR_PRE_L_MAX: u16 = 0_u16;
    pub const TEMP_MOTOR_PRE_R_MIN: u16 = 0_u16;
    pub const TEMP_MOTOR_PRE_R_MAX: u16 = 0_u16;
    pub const TEMP_COLDPLATE_PRE_R_MIN: u16 = 0_u16;
    pub const TEMP_COLDPLATE_PRE_R_MAX: u16 = 0_u16;
    
    /// Construct new Temp1 from values
    pub fn new(temp_motor_post_l: u16, temp_motor_pre_l: u16, temp_motor_pre_r: u16, temp_coldplate_pre_r: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_temp_motor_post_l(temp_motor_post_l)?;
        res.set_temp_motor_pre_l(temp_motor_pre_l)?;
        res.set_temp_motor_pre_r(temp_motor_pre_r)?;
        res.set_temp_coldplate_pre_r(temp_coldplate_pre_r)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// temp_motor_post_L
    ///
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn temp_motor_post_l(&self) -> u16 {
        self.temp_motor_post_l_raw()
    }
    
    /// Get raw value of temp_motor_post_L
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn temp_motor_post_l_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of temp_motor_post_L
    #[inline(always)]
    pub fn set_temp_motor_post_l(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 256 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
    /// temp_motor_pre_L
    ///
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn temp_motor_pre_l(&self) -> u16 {
        self.temp_motor_pre_l_raw()
    }
    
    /// Get raw value of temp_motor_pre_L
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn temp_motor_pre_l_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        signal
    }
    
    /// Set value of temp_motor_pre_L
    #[inline(always)]
    pub fn set_temp_motor_pre_l(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 256 });
        }
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// temp_motor_pre_R
    ///
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn temp_motor_pre_r(&self) -> u16 {
        self.temp_motor_pre_r_raw()
    }
    
    /// Get raw value of temp_motor_pre_R
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn temp_motor_pre_r_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of temp_motor_pre_R
    #[inline(always)]
    pub fn set_temp_motor_pre_r(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 256 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// temp_coldplate_pre_R
    ///
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn temp_coldplate_pre_r(&self) -> u16 {
        self.temp_coldplate_pre_r_raw()
    }
    
    /// Get raw value of temp_coldplate_pre_R
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn temp_coldplate_pre_r_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        signal
    }
    
    /// Set value of temp_coldplate_pre_R
    #[inline(always)]
    pub fn set_temp_coldplate_pre_r(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 256 });
        }
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Temp1 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Temp1 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Temp1")
                .field("temp_motor_post_l", &self.temp_motor_post_l())
                .field("temp_motor_pre_l", &self.temp_motor_pre_l())
                .field("temp_motor_pre_r", &self.temp_motor_pre_r())
                .field("temp_coldplate_pre_r", &self.temp_coldplate_pre_r())
            .finish()
        } else {
            f.debug_tuple("Temp1").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Temp1 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let temp_motor_post_l = u.int_in_range(0..=0)?;
        let temp_motor_pre_l = u.int_in_range(0..=0)?;
        let temp_motor_pre_r = u.int_in_range(0..=0)?;
        let temp_coldplate_pre_r = u.int_in_range(0..=0)?;
        Temp1::new(temp_motor_post_l,temp_motor_pre_l,temp_motor_pre_r,temp_coldplate_pre_r).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// Temp2
///
/// - ID: 257 (0x101)
/// - Size: 8 bytes
/// - Transmitter: SMU
#[derive(Clone, Copy)]
pub struct Temp2 {
    raw: [u8; 8],
}

impl Temp2 {
    pub const MESSAGE_ID: u32 = 257;
    
    pub const TEMP_COLD_PRE_L_MIN: u16 = 0_u16;
    pub const TEMP_COLD_PRE_L_MAX: u16 = 0_u16;
    pub const TEMP_COLD_POST_R_MIN: u16 = 0_u16;
    pub const TEMP_COLD_POST_R_MAX: u16 = 0_u16;
    pub const TEMP_COLD_POST_L_MIN: u16 = 0_u16;
    pub const TEMP_COLD_POST_L_MAX: u16 = 0_u16;
    pub const TEMP_MOT_POST_R_MIN: u16 = 0_u16;
    pub const TEMP_MOT_POST_R_MAX: u16 = 0_u16;
    
    /// Construct new Temp2 from values
    pub fn new(temp_cold_pre_l: u16, temp_cold_post_r: u16, temp_cold_post_l: u16, temp_mot_post_r: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_temp_cold_pre_l(temp_cold_pre_l)?;
        res.set_temp_cold_post_r(temp_cold_post_r)?;
        res.set_temp_cold_post_l(temp_cold_post_l)?;
        res.set_temp_mot_post_r(temp_mot_post_r)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// temp_cold_pre_L
    ///
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn temp_cold_pre_l(&self) -> u16 {
        self.temp_cold_pre_l_raw()
    }
    
    /// Get raw value of temp_cold_pre_L
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn temp_cold_pre_l_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of temp_cold_pre_L
    #[inline(always)]
    pub fn set_temp_cold_pre_l(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 257 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
    /// temp_cold_post_R
    ///
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn temp_cold_post_r(&self) -> u16 {
        self.temp_cold_post_r_raw()
    }
    
    /// Get raw value of temp_cold_post_R
    ///
    /// - Start bit: 16
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn temp_cold_post_r_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[16..32].load_le::<u16>();
        
        signal
    }
    
    /// Set value of temp_cold_post_R
    #[inline(always)]
    pub fn set_temp_cold_post_r(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 257 });
        }
        self.raw.view_bits_mut::<Lsb0>()[16..32].store_le(value);
        Ok(())
    }
    
    /// temp_cold_post_L
    ///
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn temp_cold_post_l(&self) -> u16 {
        self.temp_cold_post_l_raw()
    }
    
    /// Get raw value of temp_cold_post_L
    ///
    /// - Start bit: 32
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn temp_cold_post_l_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[32..48].load_le::<u16>();
        
        signal
    }
    
    /// Set value of temp_cold_post_L
    #[inline(always)]
    pub fn set_temp_cold_post_l(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 257 });
        }
        self.raw.view_bits_mut::<Lsb0>()[32..48].store_le(value);
        Ok(())
    }
    
    /// temp_mot_post_R
    ///
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn temp_mot_post_r(&self) -> u16 {
        self.temp_mot_post_r_raw()
    }
    
    /// Get raw value of temp_mot_post_R
    ///
    /// - Start bit: 48
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn temp_mot_post_r_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[48..64].load_le::<u16>();
        
        signal
    }
    
    /// Set value of temp_mot_post_R
    #[inline(always)]
    pub fn set_temp_mot_post_r(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 257 });
        }
        self.raw.view_bits_mut::<Lsb0>()[48..64].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Temp2 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Temp2 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Temp2")
                .field("temp_cold_pre_l", &self.temp_cold_pre_l())
                .field("temp_cold_post_r", &self.temp_cold_post_r())
                .field("temp_cold_post_l", &self.temp_cold_post_l())
                .field("temp_mot_post_r", &self.temp_mot_post_r())
            .finish()
        } else {
            f.debug_tuple("Temp2").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Temp2 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let temp_cold_pre_l = u.int_in_range(0..=0)?;
        let temp_cold_post_r = u.int_in_range(0..=0)?;
        let temp_cold_post_l = u.int_in_range(0..=0)?;
        let temp_mot_post_r = u.int_in_range(0..=0)?;
        Temp2::new(temp_cold_pre_l,temp_cold_post_r,temp_cold_post_l,temp_mot_post_r).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// SuspRear
///
/// - ID: 258 (0x102)
/// - Size: 3 bytes
/// - Transmitter: SMU
#[derive(Clone, Copy)]
pub struct SuspRear {
    raw: [u8; 3],
}

impl SuspRear {
    pub const MESSAGE_ID: u32 = 258;
    
    pub const SUSP_RR_MIN: f32 = 0_f32;
    pub const SUSP_RR_MAX: f32 = 0_f32;
    pub const SUSP_RL_MIN: f32 = 0_f32;
    pub const SUSP_RL_MAX: f32 = 0_f32;
    
    /// Construct new SuspRear from values
    pub fn new(susp_rr: f32, susp_rl: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 3] };
        res.set_susp_rr(susp_rr)?;
        res.set_susp_rl(susp_rl)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 3] {
        &self.raw
    }
    
    /// susp_rr
    ///
    /// RR suspension travel in mm
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "mm"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn susp_rr(&self) -> f32 {
        self.susp_rr_raw()
    }
    
    /// Get raw value of susp_rr
    ///
    /// - Start bit: 0
    /// - Signal size: 12 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn susp_rr_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..12].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of susp_rr
    #[inline(always)]
    pub fn set_susp_rr(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 258 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[0..12].store_le(value);
        Ok(())
    }
    
    /// susp_rl
    ///
    /// RL suspension travel in mm
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "mm"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn susp_rl(&self) -> f32 {
        self.susp_rl_raw()
    }
    
    /// Get raw value of susp_rl
    ///
    /// - Start bit: 12
    /// - Signal size: 12 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn susp_rl_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[12..24].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of susp_rl
    #[inline(always)]
    pub fn set_susp_rl(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 258 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[12..24].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for SuspRear {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 3 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 3];
        raw.copy_from_slice(&payload[..3]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for SuspRear {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("SuspRear")
                .field("susp_rr", &self.susp_rr())
                .field("susp_rl", &self.susp_rl())
            .finish()
        } else {
            f.debug_tuple("SuspRear").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for SuspRear {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let susp_rr = u.float_in_range(0_f32..=0_f32)?;
        let susp_rl = u.float_in_range(0_f32..=0_f32)?;
        SuspRear::new(susp_rr,susp_rl).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// RESERVED2
///
/// - ID: 259 (0x103)
/// - Size: 0 bytes
///
/// RESERVER FOR SMU mask - DO NOT USE
#[derive(Clone, Copy)]
pub struct Reserved2 {
    raw: [u8; 0],
}

impl Reserved2 {
    pub const MESSAGE_ID: u32 = 259;
    
    
    /// Construct new RESERVED2 from values
    pub fn new() -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 0] };
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 0] {
        &self.raw
    }
    
}

impl core::convert::TryFrom<&[u8]> for Reserved2 {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 0 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 0];
        raw.copy_from_slice(&payload[..0]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Reserved2 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Reserved2")
            .finish()
        } else {
            f.debug_tuple("Reserved2").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Reserved2 {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        Reserved2::new().map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// SuspFront
///
/// - ID: 260 (0x104)
/// - Size: 3 bytes
/// - Transmitter: ATC
#[derive(Clone, Copy)]
pub struct SuspFront {
    raw: [u8; 3],
}

impl SuspFront {
    pub const MESSAGE_ID: u32 = 260;
    
    pub const SUSP_FR_MIN: f32 = 0_f32;
    pub const SUSP_FR_MAX: f32 = 0_f32;
    pub const SUSP_FL_MIN: f32 = 0_f32;
    pub const SUSP_FL_MAX: f32 = 0_f32;
    
    /// Construct new SuspFront from values
    pub fn new(susp_fr: f32, susp_fl: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 3] };
        res.set_susp_fr(susp_fr)?;
        res.set_susp_fl(susp_fl)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 3] {
        &self.raw
    }
    
    /// susp_fr
    ///
    /// FR suspension travel in mm
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "mm"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn susp_fr(&self) -> f32 {
        self.susp_fr_raw()
    }
    
    /// Get raw value of susp_fr
    ///
    /// - Start bit: 0
    /// - Signal size: 12 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn susp_fr_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[0..12].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of susp_fr
    #[inline(always)]
    pub fn set_susp_fr(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 260 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[0..12].store_le(value);
        Ok(())
    }
    
    /// susp_fl
    ///
    /// FL suspension travel in mm
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "mm"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn susp_fl(&self) -> f32 {
        self.susp_fl_raw()
    }
    
    /// Get raw value of susp_fl
    ///
    /// - Start bit: 12
    /// - Signal size: 12 bits
    /// - Factor: 0.1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn susp_fl_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Lsb0>()[12..24].load_le::<u16>();
        
        let factor = 0.1_f32;
        let offset = 0_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of susp_fl
    #[inline(always)]
    pub fn set_susp_fl(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 260 });
        }
        let factor = 0.1_f32;
        let offset = 0_f32;
        let value = ((value - offset) / factor) as u16;
        
        self.raw.view_bits_mut::<Lsb0>()[12..24].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for SuspFront {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 3 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 3];
        raw.copy_from_slice(&payload[..3]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for SuspFront {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("SuspFront")
                .field("susp_fr", &self.susp_fr())
                .field("susp_fl", &self.susp_fl())
            .finish()
        } else {
            f.debug_tuple("SuspFront").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for SuspFront {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let susp_fr = u.float_in_range(0_f32..=0_f32)?;
        let susp_fl = u.float_in_range(0_f32..=0_f32)?;
        SuspFront::new(susp_fr,susp_fl).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// TempFrontR
///
/// - ID: 261 (0x105)
/// - Size: 3 bytes
/// - Transmitter: ATC
#[derive(Clone, Copy)]
pub struct TempFrontR {
    raw: [u8; 3],
}

impl TempFrontR {
    pub const MESSAGE_ID: u32 = 261;
    
    pub const TEMP_MOT_POT_FR_MIN: u16 = 0_u16;
    pub const TEMP_MOT_POT_FR_MAX: u16 = 0_u16;
    pub const TEMP_MOT_PRE_FR_MIN: u16 = 0_u16;
    pub const TEMP_MOT_PRE_FR_MAX: u16 = 0_u16;
    
    /// Construct new TempFrontR from values
    pub fn new(temp_mot_pot_fr: u16, temp_mot_pre_fr: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 3] };
        res.set_temp_mot_pot_fr(temp_mot_pot_fr)?;
        res.set_temp_mot_pre_fr(temp_mot_pre_fr)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 3] {
        &self.raw
    }
    
    /// temp_mot_pot_FR
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn temp_mot_pot_fr(&self) -> u16 {
        self.temp_mot_pot_fr_raw()
    }
    
    /// Get raw value of temp_mot_pot_FR
    ///
    /// - Start bit: 0
    /// - Signal size: 10 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn temp_mot_pot_fr_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..10].load_le::<u16>();
        
        signal
    }
    
    /// Set value of temp_mot_pot_FR
    #[inline(always)]
    pub fn set_temp_mot_pot_fr(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 261 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..10].store_le(value);
        Ok(())
    }
    
    /// temp_mot_pre_FR
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "C"
    /// - Receivers: Vector__XXX
    #[inline(always)]
    pub fn temp_mot_pre_fr(&self) -> u16 {
        self.temp_mot_pre_fr_raw()
    }
    
    /// Get raw value of temp_mot_pre_FR
    ///
    /// - Start bit: 10
    /// - Signal size: 10 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn temp_mot_pre_fr_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[10..20].load_le::<u16>();
        
        signal
    }
    
    /// Set value of temp_mot_pre_FR
    #[inline(always)]
    pub fn set_temp_mot_pre_fr(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 0_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 261 });
        }
        self.raw.view_bits_mut::<Lsb0>()[10..20].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for TempFrontR {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 3 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 3];
        raw.copy_from_slice(&payload[..3]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for TempFrontR {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("TempFrontR")
                .field("temp_mot_pot_fr", &self.temp_mot_pot_fr())
                .field("temp_mot_pre_fr", &self.temp_mot_pre_fr())
            .finish()
        } else {
            f.debug_tuple("TempFrontR").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for TempFrontR {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let temp_mot_pot_fr = u.int_in_range(0..=0)?;
        let temp_mot_pre_fr = u.int_in_range(0..=0)?;
        TempFrontR::new(temp_mot_pot_fr,temp_mot_pre_fr).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// InvVolt
///
/// - ID: 288 (0x120)
/// - Size: 2 bytes
/// - Transmitter: VCU
#[derive(Clone, Copy)]
pub struct InvVolt {
    raw: [u8; 2],
}

impl InvVolt {
    pub const MESSAGE_ID: u32 = 288;
    
    pub const CAR_VOLTAGE_MIN: u16 = 0_u16;
    pub const CAR_VOLTAGE_MAX: u16 = 600_u16;
    
    /// Construct new InvVolt from values
    pub fn new(car_voltage: u16) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 2] };
        res.set_car_voltage(car_voltage)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 2] {
        &self.raw
    }
    
    /// car_voltage
    ///
    /// Voltage seen from car side (inverter) in volts
    ///
    /// - Min: 0
    /// - Max: 600
    /// - Unit: "V"
    /// - Receivers: VCU, SW, BMSHV
    #[inline(always)]
    pub fn car_voltage(&self) -> u16 {
        self.car_voltage_raw()
    }
    
    /// Get raw value of car_voltage
    ///
    /// - Start bit: 0
    /// - Signal size: 16 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn car_voltage_raw(&self) -> u16 {
        let signal = self.raw.view_bits::<Lsb0>()[0..16].load_le::<u16>();
        
        signal
    }
    
    /// Set value of car_voltage
    #[inline(always)]
    pub fn set_car_voltage(&mut self, value: u16) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u16 || 600_u16 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 288 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..16].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for InvVolt {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 2 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 2];
        raw.copy_from_slice(&payload[..2]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for InvVolt {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("InvVolt")
                .field("car_voltage", &self.car_voltage())
            .finish()
        } else {
            f.debug_tuple("InvVolt").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for InvVolt {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let car_voltage = u.int_in_range(0..=600)?;
        InvVolt::new(car_voltage).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// Pcu
///
/// - ID: 304 (0x130)
/// - Size: 7 bytes
/// - Transmitter: VCU
#[derive(Clone, Copy)]
pub struct Pcu {
    raw: [u8; 7],
}

impl Pcu {
    pub const MESSAGE_ID: u32 = 304;
    
    pub const MODE_MIN: u8 = 0_u8;
    pub const MODE_MAX: u8 = 2_u8;
    pub const PUMP_SPEED_LEFT_MIN: u8 = 0_u8;
    pub const PUMP_SPEED_LEFT_MAX: u8 = 100_u8;
    pub const PUMP_SPEED_RIGHT_MIN: u8 = 0_u8;
    pub const PUMP_SPEED_RIGHT_MAX: u8 = 100_u8;
    pub const FANRAD_SPEED_LEFT_MIN: u8 = 0_u8;
    pub const FANRAD_SPEED_LEFT_MAX: u8 = 100_u8;
    pub const FANRAD_SPEED_RIGHT_MIN: u8 = 0_u8;
    pub const FANRAD_SPEED_RIGHT_MAX: u8 = 100_u8;
    pub const FANBATT_SPEED_LEFT_MIN: u8 = 0_u8;
    pub const FANBATT_SPEED_LEFT_MAX: u8 = 100_u8;
    pub const FANBATT_SPEED_RIGHT_MIN: u8 = 0_u8;
    pub const FANBATT_SPEED_RIGHT_MAX: u8 = 100_u8;
    
    /// Construct new Pcu from values
    pub fn new(mode: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 7] };
        res.set_mode(mode)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 7] {
        &self.raw
    }
    
    /// Get raw value of mode
    ///
    /// - Start bit: 0
    /// - Signal size: 2 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn mode_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..2].load_le::<u8>();
        
        signal
    }
    
    pub fn mode(&mut self) -> Result<PcuMode, CanError> {
        match self.mode_raw() {
            multiplexor => Err(CanError::InvalidMultiplexor { message_id: 304, multiplexor: multiplexor.into() }),
        }
    }
    /// Set value of mode
    #[inline(always)]
    fn set_mode(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 2_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 304 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..2].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Pcu {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 7 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 7];
        raw.copy_from_slice(&payload[..7]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Pcu {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Pcu")
            .finish()
        } else {
            f.debug_tuple("Pcu").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Pcu {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let mode = u.int_in_range(0..=2)?;
        Pcu::new(mode).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}
/// Defined values for multiplexed signal Pcu
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum PcuMode {
}


/// Calib
///
/// - ID: 305 (0x131)
/// - Size: 1 bytes
/// - Transmitter: SW
#[derive(Clone, Copy)]
pub struct Calib {
    raw: [u8; 1],
}

impl Calib {
    pub const MESSAGE_ID: u32 = 305;
    
    pub const POSITION_MIN: u8 = 0_u8;
    pub const POSITION_MAX: u8 = 1_u8;
    
    /// Construct new Calib from values
    pub fn new(position: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 1] };
        res.set_position(position)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 1] {
        &self.raw
    }
    
    /// Get raw value of position
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn position_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        signal
    }
    
    pub fn position(&mut self) -> Result<CalibPosition, CanError> {
        match self.position_raw() {
            multiplexor => Err(CanError::InvalidMultiplexor { message_id: 305, multiplexor: multiplexor.into() }),
        }
    }
    /// Set value of position
    #[inline(always)]
    fn set_position(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 1_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 305 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Calib {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 1 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 1];
        raw.copy_from_slice(&payload[..1]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Calib {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Calib")
            .finish()
        } else {
            f.debug_tuple("Calib").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Calib {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let position = u.int_in_range(0..=1)?;
        Calib::new(position).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}
/// Defined values for multiplexed signal Calib
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum CalibPosition {
}


/// CalibAck
///
/// - ID: 306 (0x132)
/// - Size: 1 bytes
/// - Transmitter: ATC
#[derive(Clone, Copy)]
pub struct CalibAck {
    raw: [u8; 1],
}

impl CalibAck {
    pub const MESSAGE_ID: u32 = 306;
    
    pub const POSITION_MIN: u8 = 0_u8;
    pub const POSITION_MAX: u8 = 1_u8;
    
    /// Construct new CalibAck from values
    pub fn new(position: u8) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 1] };
        res.set_position(position)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 1] {
        &self.raw
    }
    
    /// Get raw value of position
    ///
    /// - Start bit: 0
    /// - Signal size: 8 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn position_raw(&self) -> u8 {
        let signal = self.raw.view_bits::<Lsb0>()[0..8].load_le::<u8>();
        
        signal
    }
    
    pub fn position(&mut self) -> Result<CalibAckPosition, CanError> {
        match self.position_raw() {
            multiplexor => Err(CanError::InvalidMultiplexor { message_id: 306, multiplexor: multiplexor.into() }),
        }
    }
    /// Set value of position
    #[inline(always)]
    fn set_position(&mut self, value: u8) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_u8 || 1_u8 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 306 });
        }
        self.raw.view_bits_mut::<Lsb0>()[0..8].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for CalibAck {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 1 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 1];
        raw.copy_from_slice(&payload[..1]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for CalibAck {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("CalibAck")
            .finish()
        } else {
            f.debug_tuple("CalibAck").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for CalibAck {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let position = u.int_in_range(0..=1)?;
        CalibAck::new(position).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}
/// Defined values for multiplexed signal CalibAck
#[cfg_attr(feature = "debug", derive(Debug))]
pub enum CalibAckPosition {
}


/// PcuRfAck
///
/// - ID: 308 (0x134)
/// - Size: 1 bytes
/// - Transmitter: PCU
#[derive(Clone, Copy)]
pub struct PcuRfAck {
    raw: [u8; 1],
}

impl PcuRfAck {
    pub const MESSAGE_ID: u32 = 308;
    
    
    /// Construct new PcuRfAck from values
    pub fn new(rf_signal_ack: bool) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 1] };
        res.set_rf_signal_ack(rf_signal_ack)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 1] {
        &self.raw
    }
    
    /// rf_signalAck
    ///
    /// - Min: 0
    /// - Max: 1
    /// - Unit: "on"
    /// - Receivers: VCU
    #[inline(always)]
    pub fn rf_signal_ack(&self) -> bool {
        self.rf_signal_ack_raw()
    }
    
    /// Get raw value of rf_signalAck
    ///
    /// - Start bit: 0
    /// - Signal size: 1 bits
    /// - Factor: 1
    /// - Offset: 0
    /// - Byte order: LittleEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn rf_signal_ack_raw(&self) -> bool {
        let signal = self.raw.view_bits::<Lsb0>()[0..1].load_le::<u8>();
        
        signal == 1
    }
    
    /// Set value of rf_signalAck
    #[inline(always)]
    pub fn set_rf_signal_ack(&mut self, value: bool) -> Result<(), CanError> {
        let value = value as u8;
        self.raw.view_bits_mut::<Lsb0>()[0..1].store_le(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for PcuRfAck {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 1 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 1];
        raw.copy_from_slice(&payload[..1]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for PcuRfAck {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("PcuRfAck")
                .field("rf_signal_ack", &self.rf_signal_ack())
            .finish()
        } else {
            f.debug_tuple("PcuRfAck").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for PcuRfAck {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let rf_signal_ack = u.int_in_range(0..=1)? == 1;
        PcuRfAck::new(rf_signal_ack).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}

/// Lem
///
/// - ID: 962 (0x3c2)
/// - Size: 8 bytes
/// - Transmitter: LEM
#[derive(Clone, Copy)]
pub struct Lem {
    raw: [u8; 8],
}

impl Lem {
    pub const MESSAGE_ID: u32 = 962;
    
    pub const CURRENT_MIN: f32 = 0_f32;
    pub const CURRENT_MAX: f32 = 0_f32;
    
    /// Construct new Lem from values
    pub fn new(current: f32) -> Result<Self, CanError> {
        let mut res = Self { raw: [0u8; 8] };
        res.set_current(current)?;
        Ok(res)
    }
    
    /// Access message payload raw value
    pub fn raw(&self) -> &[u8; 8] {
        &self.raw
    }
    
    /// current
    ///
    /// Current seen from LEM on car side (PDB)
    ///
    /// - Min: 0
    /// - Max: 0
    /// - Unit: "mA"
    /// - Receivers: VCU, SW, BMSHV
    #[inline(always)]
    pub fn current(&self) -> f32 {
        self.current_raw()
    }
    
    /// Get raw value of current
    ///
    /// - Start bit: 7
    /// - Signal size: 32 bits
    /// - Factor: 1
    /// - Offset: -2147483648
    /// - Byte order: BigEndian
    /// - Value type: Unsigned
    #[inline(always)]
    pub fn current_raw(&self) -> f32 {
        let signal = self.raw.view_bits::<Msb0>()[0..32].load_be::<u32>();
        
        let factor = 1_f32;
        let offset = -2147483648_f32;
        (signal as f32) * factor + offset
    }
    
    /// Set value of current
    #[inline(always)]
    pub fn set_current(&mut self, value: f32) -> Result<(), CanError> {
        #[cfg(feature = "range_checked")]
        if value < 0_f32 || 0_f32 < value {
            return Err(CanError::ParameterOutOfRange { message_id: 962 });
        }
        let factor = 1_f32;
        let offset = -2147483648_f32;
        let value = ((value - offset) / factor) as u32;
        
        self.raw.view_bits_mut::<Msb0>()[0..32].store_be(value);
        Ok(())
    }
    
}

impl core::convert::TryFrom<&[u8]> for Lem {
    type Error = CanError;
    
    #[inline(always)]
    fn try_from(payload: &[u8]) -> Result<Self, Self::Error> {
        if payload.len() != 8 { return Err(CanError::InvalidPayloadSize); }
        let mut raw = [0u8; 8];
        raw.copy_from_slice(&payload[..8]);
        Ok(Self { raw })
    }
}

#[cfg(feature = "debug")]
impl core::fmt::Debug for Lem {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if f.alternate() {
            f.debug_struct("Lem")
                .field("current", &self.current())
            .finish()
        } else {
            f.debug_tuple("Lem").field(&self.raw).finish()
        }
    }
}

#[cfg(feature = "arb")]
impl<'a> Arbitrary<'a> for Lem {
    fn arbitrary(u: &mut Unstructured<'a>) -> Result<Self, arbitrary::Error> {
        let current = u.float_in_range(0_f32..=0_f32)?;
        Lem::new(current).map_err(|_| arbitrary::Error::IncorrectFormat)
    }
}


/// This is just to make testing easier
#[allow(dead_code)]
fn main() {}

#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(any(feature = "debug", feature = "std"), derive(Debug))]
pub enum CanError {
    UnknownMessageId(u32),
    /// Signal parameter is not within the range
    /// defined in the dbc
    ParameterOutOfRange {
        /// dbc message id
        message_id: u32,
    },
    InvalidPayloadSize,
    /// Multiplexor value not defined in the dbc
    InvalidMultiplexor {
        /// dbc message id
        message_id: u32,
        /// Multiplexor value not defined in the dbc
        multiplexor: u16,
    },
}

#[cfg(feature = "std")]
use std::error::Error;
#[cfg(feature = "std")]
use std::fmt;

#[cfg(feature = "std")]
impl fmt::Display for CanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

#[cfg(feature = "std")]
impl Error for CanError {}
#[cfg(feature = "arb")]
trait UnstructuredFloatExt {
    fn float_in_range(&mut self, range: core::ops::RangeInclusive<f32>) -> arbitrary::Result<f32>;
}

#[cfg(feature = "arb")]
impl UnstructuredFloatExt for arbitrary::Unstructured<'_> {
    fn float_in_range(&mut self, range: core::ops::RangeInclusive<f32>) -> arbitrary::Result<f32> {
        let min = range.start();
        let max = range.end();
        let steps = u32::MAX;
        let factor = (max - min) / (steps as f32);
        let random_int: u32 = self.int_in_range(0..=steps)?;
        let random = min + factor * (random_int as f32);
        Ok(random)
    }
}

