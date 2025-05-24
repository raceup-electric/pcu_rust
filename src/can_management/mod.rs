pub mod frame;
pub mod can_controller;
pub use can_controller::CanController;
#[allow(dead_code)]
pub mod messages;
pub use frame::CanFrame;


#[macro_export]
macro_rules! get_byte {
    ($value:expr, $byte_num:expr) => {
        (($value >> ($byte_num * 8)) & 0xFF) as u8
    };
    
    ($array:expr, $byte_num:expr, slice) => {
        $array.get($byte_num).copied().unwrap_or(0)
    };
}
