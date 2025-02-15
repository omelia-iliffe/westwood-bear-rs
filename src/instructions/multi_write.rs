use crate::error::WriteError;
use crate::registers::WritableRegister;
use crate::{Address, Bus, Register};

#[derive(Debug)]
pub struct MultiWrite<Addr> {
    address: Addr,
    data: [u8; 4],
}

impl<Addr> MultiWrite<Addr> {
    pub fn new<R>(data: R::Inner) -> Self
    where
        R: WritableRegister,
        R: Register<RegisterType = Addr>,
    {
        Self {
            address: R::ADDRESS,
            data: R::encode_bytes(data),
        }
    }
}

impl<SerialPort, Buffer> Bus<SerialPort, Buffer>
where
    SerialPort: crate::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    /// Writes data to multiple registers on the same motor
    pub fn multi_write<'a, I, Addr>(&mut self, motor_id: u8, data: &'a I) -> Result<(), WriteError<SerialPort::Error>>
    where
        &'a I: IntoIterator,
        <&'a I as IntoIterator>::IntoIter: ExactSizeIterator,
        <&'a I as IntoIterator>::Item: core::borrow::Borrow<MultiWrite<Addr>>,
        Addr: Address,
    {
        use core::borrow::Borrow;
        let data = data.into_iter();
        let parameter_count = data.len() * 5;
        self.write_packet(motor_id, Addr::WRITE_INST as u8, parameter_count, |buffer| {
            let mut offset = 0;
            for d in data {
                let buffer = &mut buffer[offset..];
                let d = d.borrow();
                buffer[0] = d.address.as_byte();
                buffer[1..][..5].copy_from_slice(&d.data);
                offset += 5;
            }
            Ok(())
        })?;
        Ok(())
    }
}
