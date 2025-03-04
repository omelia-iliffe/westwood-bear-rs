use crate::error::WriteError;
use crate::protocol::Instruction;
use crate::Bus;

impl<SerialPort, Buffer> Bus<SerialPort, Buffer>
where
    SerialPort: crate::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    pub fn save_config(&mut self, motor_id: u8) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_packet(motor_id, Instruction::SaveCfg as u8, 0, |_| Ok(()))
    }
}
