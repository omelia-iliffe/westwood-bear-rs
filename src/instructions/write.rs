use crate::bus::Bus;
use crate::error::WriteError;
use crate::registers::WritableRegister;
use crate::Address;

impl<SerialPort, Buffer> Bus<SerialPort, Buffer>
where
    SerialPort: crate::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    pub fn write<R: WritableRegister>(
        &mut self,
        motor_id: u8,
        data: R::Inner,
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_packet(
            motor_id,
            R::RegisterType::WRITE_INST as u8,
            R::ENCODED_SIZE as usize + 1,
            |buffer| {
                buffer[0] = R::ADDRESS.as_byte();
                R::encode(data, &mut buffer[1..])?;
                Ok(())
            },
        )
    }
}
