use super::super::Bus;
use crate::error::WriteError;
use crate::protocol::Instruction;

#[super::super::bisync]
impl<SerialPort, Buffer> Bus<SerialPort, Buffer>
where
    SerialPort: super::super::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    /// Reset the multiturn encoder (and clear the accosicated error) on a specific motor with a backup battery.
    /// If tolerance is non-zero, BEAR tries to find the multi-turn value to match the expected position within the tolerance.
    /// If tolerance is zero, BEAR modifies the homing offset to match the expected position.
    /// A save_config() action will be performed automatically once set_posi() is successfully excuted.
    ///
    /// Compatible with Panda, Kodiak and Mountain motors.
    ///
    pub async fn set_absolute_position(
        &mut self,
        motor_id: u8,
        position: f32,
        tolerance: f32,
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_packet(motor_id, Instruction::SetAbsPos as u8, 8, |buffer| {
            buffer[..4].copy_from_slice(&position.to_le_bytes());
            buffer[4..].copy_from_slice(&tolerance.to_le_bytes());
            Ok(())
        })
        .await
    }
}
