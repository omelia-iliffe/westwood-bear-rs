use crate::bus::Bus;
use crate::error::TransferError;
use crate::protocol::{Instruction, RegisterType, Response};

impl<SerialPort, Buffer> Bus<SerialPort, Buffer>
where
    SerialPort: crate::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    pub(crate) fn read_raw(
        &mut self,
        motor_id: u8,
        instruction_id: u8,
        addr: u8,
        length: u8,
    ) -> Result<Response<&[u8]>, TransferError<SerialPort::Error>> {
        self.write_packet(motor_id, instruction_id, 1, |buffer| {
            buffer[0] = addr;
            Ok(())
        })?;
        let r = self.read_response(length + 1)?;

        Ok(r)
    }

    pub fn read<R: crate::Register>(
        &mut self,
        motor_id: u8,
    ) -> Result<Response<R::Inner>, TransferError<SerialPort::Error>> {
        let inst = match R::REG_TYPE {
            RegisterType::Config => Instruction::ReadCfg,
            RegisterType::Status => Instruction::ReadStat,
        };
        let r = self.read_raw(motor_id, inst as u8, R::ADDR, 4)?;
        let r = Response {
            motor_id: r.motor_id,
            error: r.error,
            data: R::decode(r.data)?,
        };
        Ok(r)
    }
}
