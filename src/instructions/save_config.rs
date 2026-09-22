use super::super::Bus;
use crate::error::WriteError;
use crate::protocol::Instruction;

#[super::super::bisync]
impl<SerialPort> Bus<SerialPort>
where
    SerialPort: super::super::SerialPort,
{
    /// Saves the config registers of a specific motor.
    ///
    /// The saved config registers persist on reboot.
    pub async fn save_config(&mut self, motor_id: u8) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_packet(motor_id, Instruction::SaveCfg as u8, 0, |_| Ok(())).await
    }
}
