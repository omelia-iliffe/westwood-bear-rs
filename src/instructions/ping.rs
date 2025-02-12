use crate::error::TransferError;
use crate::protocol::{Instruction, Response};
use crate::Bus;

impl<SerialPort, Buffer> Bus<SerialPort, Buffer>
where
    SerialPort: crate::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    pub fn ping(&mut self, motor_id: u8) -> Result<Response<&[u8]>, TransferError<SerialPort::Error>> {
        self.write_packet(motor_id, Instruction::Ping as u8, 0, |_| Ok(()))?;
        let r = self.read_response(4)?;
        Ok(r)
    }
}
