pub mod frame;
pub mod can_controller;
pub use can_controller::{CanError,CanController};
use crate::CanMsg;
use defmt::info;
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

pub async fn can_operation(id: u16, message: &[u8;8], can: &mut CanController<'_>) {
    let frame_send = CanFrame::new(id ,message);
    match can.write(&frame_send).await {
        Ok(_) => {
            info!("Message sent! {}", &frame_send.id());
            for i in 0..frame_send.len() {
                info!("Byte: {}: {}", i, &frame_send.byte(i));
            }
        }

        Err(CanError::Timeout) => {
            info!("Timeout Can connection");
        }

        Err(_) => {
            info!("Can write error");
        }
    }
}