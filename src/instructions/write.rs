use crate::bus::Bus;
use crate::error::WriteError;
use crate::registers::WritableRegister;

impl<SerialPort, Buffer> Bus<SerialPort, Buffer>
where
    SerialPort: crate::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    pub(crate) fn write_raw(
        &mut self,
        motor_id: u8,
        instruction: u8,
        addr: u8,
        data: &[u8],
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_packet(motor_id, instruction, data.len() + 1, |buffer| {
            buffer[0] = addr;
            buffer[1..].copy_from_slice(data);
            Ok(())
        })
    }

    pub fn write<R: WritableRegister>(
        &mut self,
        motor_id: u8,
        data: R::Inner,
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_packet(motor_id, R::WRITE_INST as u8, R::ENCODED_SIZE as usize + 1, |buffer| {
            buffer[0] = R::ADDR;
            R::encode(data, &mut buffer[1..])?;
            Ok(())
        })
    }
}
