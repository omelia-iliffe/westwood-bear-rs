use crate::error::{BufferTooSmallError, MotorError, ReadError, TransferError, WriteError};
use crate::protocol::{Response, PACKET_ERROR, PACKET_ID, PACKET_LEN};
use crate::{checksum, ErrorFlags};
use core::time::Duration;
use log::{debug, trace};

/// Default buffer type.
///
/// Defaults to [`Vec<u8>`] if the `"alloc"` or `"std"` feature is enabled.
/// Otherwise, defaults to `&'mut static [u8]`.
#[cfg(feature = "alloc")]
pub type DefaultBuffer = alloc::vec::Vec<u8>;

/// Default buffer type.
///
/// Defaults to [`Vec<u8>`] if the `"alloc"` or `"std"` feature is enabled.
/// Otherwise, defaults to `&'mut static [u8]`.
#[cfg(not(feature = "alloc"))]
pub type DefaultBuffer = &'static mut [u8];

const HEADER_PREFIX: [u8; 2] = [0xFF, 0xFF];
const HEADER_SIZE: usize = 4;
// PACKET
// | HEADER    | ID | LEN | INST | ADDR | PARAM        | CRC |
// | 255, 255  | 2  | 7   | 3    | 5    | 0, 0, 48, 65 | 125 |

macro_rules! make_client_struct {
    ($($DefaultSerialPort:ty)?) => {
        pub struct Bus<SerialPort $(= $DefaultSerialPort)?, Buffer = DefaultBuffer>
        where
            SerialPort: crate::SerialPort,
            Buffer: AsRef<[u8]> + AsMut<[u8]>,
        {
            /// The underlying stream (normally a serial port).
            pub(crate) serial_port: SerialPort,
            /// The baud rate of the serial port, if known.
            pub(crate) baud_rate: u32,
            /// The buffer for reading incoming messages.
            pub(crate) read_buffer: Buffer,
            /// The total number of valid bytes in the read buffer.
            pub(crate) read_len: usize,
            /// The number of leading bytes in the read buffer that have already been used.
            pub(crate) used_bytes: usize,
            /// The buffer for outgoing messages.
            pub(crate) write_buffer: Buffer,
        }
    };
}

#[cfg(feature = "serial2")]
make_client_struct!(serial2::SerialPort);

#[cfg(not(feature = "serial2"))]
make_client_struct!();

impl<SerialPort, Buffer> core::fmt::Debug for Bus<SerialPort, Buffer>
where
    SerialPort: crate::SerialPort + core::fmt::Debug,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Bus")
            .field("serial_port", &self.serial_port)
            .field("baud_rate", &self.baud_rate)
            .finish_non_exhaustive()
    }
}
#[cfg(feature = "serial2")]
impl Bus<serial2::SerialPort, Vec<u8>> {
    /// Open a serial port with the given baud rate.
    ///
    /// This will allocate a new read and write buffer of 128 bytes each.
    /// Use [`Self::open_with_buffers()`] if you want to use a custom buffers.
    pub fn open(path: impl AsRef<std::path::Path>, baud_rate: u32) -> std::io::Result<Self> {
        let serial_port = serial2::SerialPort::open(path, baud_rate)?;
        let bus = Bus::with_buffers_and_baud_rate(serial_port, vec![0; 128], vec![0; 128], baud_rate);
        Ok(bus)
    }
}
#[cfg(feature = "serial2")]
impl<Buffer> Bus<serial2::SerialPort, Buffer>
where
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    /// Open a serial port with the given baud rate.
    ///
    /// This will allocate a new read and write buffer of 128 bytes each.
    pub fn open_with_buffers(
        path: impl AsRef<std::path::Path>,
        baud_rate: u32,
        read_buffer: Buffer,
        write_buffer: Buffer,
    ) -> std::io::Result<Self> {
        let serial_port = serial2::SerialPort::open(path, baud_rate)?;
        let bus = Bus::with_buffers_and_baud_rate(serial_port, read_buffer, write_buffer, baud_rate);
        Ok(bus)
    }
}
#[cfg(feature = "alloc")]
impl<SerialPort> Bus<SerialPort, Vec<u8>>
where
    SerialPort: crate::SerialPort,
{
    /// Create a new client using an open serial port.
    ///
    /// The serial port must already be configured in raw mode with the correct baud rate,
    /// character size (8), parity (disabled) and stop bits (1).
    ///
    /// This will allocate a new read and write buffer of 128 bytes each.
    /// Use [`Self::with_buffers()`] if you want to use a custom buffers.
    #[cfg(feature = "alloc")]
    pub fn new(serial_port: SerialPort) -> Result<Self, SerialPort::Error> {
        Bus::with_buffers(serial_port, vec![0; 128], vec![0; 128])
    }
}
impl<SerialPort, Buffer> Bus<SerialPort, Buffer>
where
    SerialPort: crate::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    /// Create a new bus using pre-allocated buffers.
    ///
    /// The serial port must already be configured in raw mode with the correct baud rate,
    /// character size (8), parity (disabled) and stop bits (1).
    pub fn with_buffers(
        serial_port: SerialPort,
        read_buffer: Buffer,
        write_buffer: Buffer,
    ) -> Result<Self, SerialPort::Error> {
        let baud_rate = serial_port.baud_rate()?;
        Ok(Self::with_buffers_and_baud_rate(
            serial_port,
            read_buffer,
            write_buffer,
            baud_rate,
        ))
    }

    /// Create a new bus using pre-allocated buffers.
    pub fn with_buffers_and_baud_rate(
        serial_port: SerialPort,
        read_buffer: Buffer,
        write_buffer: Buffer,
        baud_rate: u32,
    ) -> Self {
        let mut write_buffer = write_buffer;

        // Pre-fill write buffer with the header prefix.
        // TODO: return Err instead of panicking.
        assert!(write_buffer.as_mut().len() >= HEADER_SIZE + 3);
        write_buffer.as_mut()[..2].copy_from_slice(&HEADER_PREFIX);

        Self {
            serial_port,
            baud_rate,
            read_buffer,
            read_len: 0,
            used_bytes: 0,
            write_buffer,
        }
    }

    /// Set the baud rate of the underlying serial port.
    pub fn set_baud_rate(&mut self, baud_rate: u32) -> Result<(), SerialPort::Error> {
        self.serial_port.set_baud_rate(baud_rate)?;
        self.baud_rate = baud_rate;
        Ok(())
    }

    /// Write a raw instruction to a stream, and read a single raw response.
    ///
    /// This function also checks that the packet ID of the status response matches the one from the instruction.
    /// and that the error byte does not contain any [`crate::ERROR_FLAGS`]. [`crate::WARNING_FLAGS`] are allowed.
    pub(crate) fn transfer_single<F>(
        &mut self,
        packet_id: u8,
        instruction_id: u8,
        parameter_count: usize,
        expected_response_parameters: u8,
        encode_parameters: F,
    ) -> Result<Response<&[u8]>, TransferError<SerialPort::Error>>
    where
        F: FnOnce(&mut [u8]) -> Result<(), crate::error::BufferTooSmallError>,
    {
        self.write_packet(packet_id, instruction_id, parameter_count, encode_parameters)?;
        let response = self.read_response(expected_response_parameters)?;
        crate::error::InvalidPacketId::check(response.motor_id, packet_id)?;
        MotorError::check(response.warning)?;
        Ok(response)
    }
    /// Write a packet to the bus.
    pub(crate) fn make_packet<F>(
        buffer: &mut [u8],
        packet_id: u8,
        instruction_id: u8,
        parameter_count: usize,
        encode_parameters: F,
    ) -> Result<usize, BufferTooSmallError>
    where
        F: FnOnce(&mut [u8]) -> Result<(), crate::error::BufferTooSmallError>,
    {
        let len = parameter_count + 2; // + CRC, INST

        // Check if the buffer can hold the message.
        BufferTooSmallError::check(HEADER_SIZE + len, buffer.len())?;

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

        Ok(checksum_index + 1)
    }
    pub(crate) fn write_packet<F>(
        &mut self,
        packet_id: u8,
        instruction_id: u8,
        parameter_count: usize,
        encode_parameters: F,
    ) -> Result<(), WriteError<SerialPort::Error>>
    where
        F: FnOnce(&mut [u8]) -> Result<(), crate::error::BufferTooSmallError>,
    {
        let packet_len = Self::make_packet(
            self.write_buffer.as_mut(),
            packet_id,
            instruction_id,
            parameter_count,
            encode_parameters,
        )?;
        let packet = &self.write_buffer.as_ref()[..packet_len];
        // Throw away old data in the read buffer and the kernel read buffer.
        // We don't do this when reading a reply, because we might receive multiple replies for one instruction,
        // and read() can potentially read more than one reply per syscall.
        self.read_len = 0;
        self.used_bytes = 0;
        self.serial_port
            .discard_input_buffer()
            .map_err(WriteError::DiscardBuffer)?;
        trace!("sending packet: {:02X?}", packet);
        self.serial_port.write_all(packet).map_err(WriteError::Write)?;
        Ok(())
        // self.write_packet_raw(&self.write_buffer.as_ref()[..packet_len])
    }

    pub(crate) fn read_response(
        &mut self,
        expected_parameters: u8,
    ) -> Result<Response<&[u8]>, ReadError<SerialPort::Error>> {
        let timeout = message_transfer_time(expected_parameters as u32, self.baud_rate) + Duration::from_millis(1);
        self.read_response_timeout(timeout)
    }

    fn read_response_timeout(&mut self, timeout: Duration) -> Result<Response<&[u8]>, ReadError<SerialPort::Error>> {
        let deadline = self.serial_port.make_deadline(timeout);
        let packet = self.read_packet_deadline(deadline)?;
        let response = Response {
            motor_id: packet[PACKET_ID],
            warning: ErrorFlags::from_bits(packet[PACKET_ERROR]),
            data: &packet[5..],
        };
        Ok(response)
    }

    /// returns a packet including header + parameters
    fn read_packet_deadline(&mut self, deadline: SerialPort::Instant) -> Result<&[u8], ReadError<SerialPort::Error>> {
        // Check that the read buffer is large enough to hold atleast a instruction packet with 0 parameters.
        crate::error::BufferTooSmallError::check(HEADER_SIZE + 2, self.read_buffer.as_mut().len())?; //todo check size is correct

        let message_len = loop {
            self.remove_garbage();

            // The call to remove_garbage() removes all leading bytes that don't match a packet header.
            // So if there's enough bytes left, it's a packet header.
            if self.read_len > HEADER_SIZE {
                let read_buffer = &self.read_buffer.as_ref()[..self.read_len];
                let body_len = read_buffer[PACKET_LEN] as usize;

                // Check if the read buffer is large enough for the entire message.
                // We don't have to remove the read bytes, because `write_instruction()` already clears the read buffer.
                crate::error::BufferTooSmallError::check(HEADER_SIZE + body_len, self.read_buffer.as_mut().len())?;

                if self.read_len >= HEADER_SIZE + body_len {
                    break HEADER_SIZE + body_len;
                }
            }

            // Try to read more data into the buffer.
            let new_data = self
                .serial_port
                .read(&mut self.read_buffer.as_mut()[self.read_len..], &deadline)
                .map_err(ReadError::Io)?;
            if new_data == 0 {
                continue;
            }

            self.read_len += new_data;
        };

        let buffer = self.read_buffer.as_ref();
        let parameters_end = message_len - 1;
        trace!("read packet: {:02X?}", &buffer[..parameters_end]);

        let checksum_message = buffer[parameters_end];
        let checksum_computed = checksum::calculate_checksum(&buffer[2..parameters_end]);
        if checksum_message != checksum_computed {
            self.consume_read_bytes(message_len);
            return Err(crate::error::InvalidChecksum {
                message: checksum_message,
                computed: checksum_computed,
            }
            .into());
        }

        // Mark the whole message as "used_bytes", so that the next call to `remove_garbage()` removes it.
        self.used_bytes += message_len;
        let packet = &self.read_buffer.as_ref()[..parameters_end];
        Ok(packet)
    }
    /// Remove leading garbage data from the read buffer.
    fn remove_garbage(&mut self) {
        let read_buffer = self.read_buffer.as_mut();
        let garbage_len = find_header(&read_buffer[..self.read_len][self.used_bytes..]);
        if garbage_len > 0 {
            debug!("skipping {} bytes of leading garbage.", garbage_len);
            trace!("skipped garbage: {:02X?}", &read_buffer[..garbage_len]);
        }
        self.consume_read_bytes(self.used_bytes + garbage_len);
        debug_assert_eq!(self.used_bytes, 0);
    }
    fn consume_read_bytes(&mut self, len: usize) {
        debug_assert!(len <= self.read_len);
        self.read_buffer.as_mut().copy_within(len..self.read_len, 0);
        // Decrease both used_bytes and read_len together.
        // Some consumed bytes may be garbage instead of used bytes though.
        // So we use `saturating_sub` for `used_bytes` to cap the result at 0.
        self.used_bytes = self.used_bytes.saturating_sub(len);
        self.read_len -= len;
    }
}

/// Find the potential starting position of a header.
///
/// This will return the first possible position of the header prefix.
/// Note that if the buffer ends with a partial header prefix,
/// the start position of the partial header prefix is returned.
fn find_header(buffer: &[u8]) -> usize {
    for i in 0..buffer.len() {
        let possible_prefix = HEADER_PREFIX.len().min(buffer.len() - i);
        if buffer[i..].starts_with(&HEADER_PREFIX[..possible_prefix]) {
            return i;
        }
    }

    buffer.len()
}

/// Calculate the required time to transfer a message of a given size.
///
/// The size must include any headers and footers of the message.
pub(crate) fn message_transfer_time(message_size: u32, baud_rate: u32) -> Duration {
    let baud_rate = u64::from(baud_rate);
    let bits = u64::from(message_size) * 10; // each byte is 1 start bit, 8 data bits and 1 stop bit.
    let secs = bits / baud_rate;
    let subsec_bits = bits % baud_rate;
    let nanos = (subsec_bits * 1_000_000_000).div_ceil(baud_rate);
    Duration::new(secs, nanos as u32)
}
