use log::trace;
use crate::checksum;
use crate::error::{BufferTooSmallError, WriteError};

const HEADER_SIZE: usize = 4;
// PACKET
// | HEADER    | ID | LEN | INST | ADDR | PARAM        | CRC |
// | 255, 255  | 2  | 7   | 3    | 5    | 0, 0, 48, 65 | 125 |

pub struct Bus<SerialPort = serial2::SerialPort> {
    /// The underlying stream (normally a serial port).
    serial_port: SerialPort,
    /// The baud rate of the serial port, if known.
    baud_rate: u32,
    /// The buffer for reading incoming messages.
    read_buffer: Vec<u8>,
    /// The total number of valid bytes in the read buffer.
    pub(crate) read_len: usize,
    /// The number of leading bytes in the read buffer that have already been used.
    pub(crate) used_bytes: usize,
    /// The buffer for outgoing messages.
    write_buffer: Vec<u8>,
}


impl<SerialPort> Bus<SerialPort>
{
    /// Write a packet to the bus.
    pub fn make_packet<F>(
        buffer: &mut [u8],
        packet_id: u8,
        instruction_id: u8,
        parameter_count: usize,
        encode_parameters: F,
    ) -> Result<&[u8], BufferTooSmallError>
    where
        F: FnOnce(&mut [u8]) -> Result<(), crate::error::BufferTooSmallError>,
    {
        let len = parameter_count + 2; // + CRC, INST

        // Check if the buffer can hold the message.
        BufferTooSmallError::check(HEADER_SIZE + len, buffer.len())?;

        // Add the header, with a placeholder for the length field.
        buffer[0] = 0xFF;
        buffer[1] = 0xFF;
        buffer[2] = packet_id;
        buffer[3] = len as u8;
        buffer[4] = instruction_id;
        encode_parameters(&mut buffer[5..][..parameter_count])?;


        // Add checksum.
        let checksum_index = HEADER_SIZE + parameter_count + 1;
        let checksum = checksum::calculate_checksum(&buffer[2..checksum_index]);
        buffer[checksum_index] = checksum;

        Ok(&buffer[..checksum_index + 1])
    }
}

impl<SerialPort> Bus<SerialPort> where SerialPort: crate::SerialPort{
    pub fn write_packet(&mut self, packet: &[u8]) -> Result<(), WriteError<SerialPort::Error>> {

        // Throw away old data in the read buffer and the kernel read buffer.
        // We don't do this when reading a reply, because we might receive multiple replies for one instruction,
        // and read() can potentially read more than one reply per syscall.
        self.read_len = 0;
        self.used_bytes = 0;
        self.serial_port.discard_input_buffer().map_err(WriteError::DiscardBuffer)?;
         trace!("sending packet: {:02X?}", packet);
         self.serial_port.write_all(packet).map_err(WriteError::Write)?;
        Ok(())
    }

}