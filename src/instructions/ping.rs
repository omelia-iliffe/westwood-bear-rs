use super::super::Bus;
use crate::error::TransferError;
use crate::protocol::{Instruction, Response};

#[super::super::bisync]
impl<SerialPort, Buffer> Bus<SerialPort, Buffer>
where
    SerialPort: super::super::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    /// Ping a speific motor by ID
    pub async fn ping(&mut self, motor_id: u8) -> Result<Response<&[u8]>, TransferError<SerialPort::Error>> {
        self.write_packet(motor_id, Instruction::Ping as u8, 0, |_| Ok(())).await?;
        let r = self.read_response(4).await?;
        Ok(r)
    }
}
