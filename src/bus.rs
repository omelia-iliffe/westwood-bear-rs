use crate::error::{InvalidFrameLength, ReadError, TooManyParametersError, TransferError, WriteError};
use crate::log::{Hex, debug, trace};
use crate::protocol::{MAX_PACKET_SIZE, MAX_PARAMETER_COUNT, PACKET_ERROR, PACKET_ID, PACKET_LEN, Response};
use crate::{ErrorFlags, checksum};
use core::time::Duration;

const HEADER_PREFIX: [u8; 2] = [0xFF, 0xFF];
const HEADER_SIZE: usize = 4;

// Equality both ways: `>=` lets the read path drop its size checks, and `<=` keeps
// `len as u8` in `make_packet` from truncating.
const _: () = assert!(MAX_PACKET_SIZE == HEADER_SIZE + u8::MAX as usize);

/// The smallest body `LEN` can claim: it counts the instruction/error byte and the checksum.
const MIN_BODY_LEN: usize = 2;
// PACKET
// | HEADER    | ID | LEN | INST | ADDR | PARAM        | CRC |
// | 255, 255  | 2  | 7   | 3    | 5    | 0, 0, 48, 65 | 125 |

/// Bus for Westwood Robotics Bear Actuators.
///
/// Used to communicate with devices on the serial bus.
///
/// The `SerialPort` type argument must always be specified. Enable the `"serial2"`
/// or `"serial2-tokio"` feature to construct one from a path with `Bus::open`, or
/// provide your own [`SerialPort`](super::SerialPort) implementation (for example on
/// `no_std` targets).
///
/// The frame buffers are [`MAX_PACKET_SIZE`] and not configurable: `LEN` is one byte, so
/// that is at once the largest frame the protocol can describe and the smallest buffer that
/// can never refuse a legal one. Both are inline arrays, needing no allocator on `no_std`.
pub struct Bus<SerialPort>
where
    SerialPort: super::SerialPort,
{
    pub(crate) serial_port: SerialPort,
    pub(crate) baud_rate: u32,
    pub(crate) read_buffer: [u8; MAX_PACKET_SIZE],
    /// The total number of valid bytes in the read buffer.
    pub(crate) read_len: usize,
    /// The number of leading bytes in the read buffer that have already been used.
    pub(crate) used_bytes: usize,
    pub(crate) write_buffer: [u8; MAX_PACKET_SIZE],
    pub(crate) response_timeout_padding: Duration,
}

impl<SerialPort> core::fmt::Debug for Bus<SerialPort>
where
    SerialPort: super::SerialPort + core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Bus")
            .field("serial_port", &self.serial_port)
            .field("baud_rate", &self.baud_rate)
            .finish_non_exhaustive()
    }
}
// `<Port>::open` is synchronous in both bisync2 trees, so one macro is instantiated once
// per tree against that tree's default port type.
#[allow(unused_macros)]
macro_rules! make_serial2_bus_impls {
    ($DefaultSerialPort:ty) => {
        impl Bus<$DefaultSerialPort> {
            /// Open a serial port with the given baud rate.
            pub fn open(path: impl AsRef<std::path::Path>, baud_rate: u32) -> std::io::Result<Self> {
                let serial_port = <$DefaultSerialPort>::open(path, baud_rate)?;
                Ok(Bus::new_with_baud_rate(serial_port, baud_rate))
            }
        }
    };
}
#[cfg(feature = "serial2")]
#[super::only_sync]
make_serial2_bus_impls!(super::Serial2Port);
#[cfg(feature = "serial2-tokio")]
#[super::only_async]
make_serial2_bus_impls!(super::Serial2Port);
#[super::bisync]
impl<SerialPort> Bus<SerialPort>
where
    SerialPort: super::SerialPort,
{
    /// Create a new bus using an open serial port, reading the baud rate back off it.
    ///
    /// The serial port must already be configured in raw mode with the correct baud rate,
    /// character size (8), parity (disabled) and stop bits (1).
    ///
    /// Prefer [`Self::new_with_baud_rate`] when the rate was chosen rather than discovered:
    /// every message timeout is derived from it.
    pub fn new(serial_port: SerialPort) -> Result<Self, SerialPort::Error> {
        let baud_rate = serial_port.baud_rate()?;
        Ok(Self::new_with_baud_rate(serial_port, baud_rate))
    }

    /// Create a new bus using an open serial port and a known baud rate.
    ///
    /// The write buffer is not primed with the header prefix: `make_packet` writes it per
    /// packet, and staging a primed buffer in a local costs a `MAX_PACKET_SIZE` stack temporary.
    pub fn new_with_baud_rate(serial_port: SerialPort, baud_rate: u32) -> Self {
        Self {
            serial_port,
            baud_rate,
            read_buffer: [0u8; MAX_PACKET_SIZE],
            read_len: 0,
            used_bytes: 0,
            write_buffer: [0u8; MAX_PACKET_SIZE],
            response_timeout_padding: Duration::from_millis(3),
        }
    }

    /// Set the baud rate of the underlying serial port.
    ///
    /// Anything already buffered was sampled at the old rate and is dropped: a surviving
    /// `FF FF` pair is enough to make bytes from two signalling rates look like a real header.
    pub fn set_baud_rate(&mut self, baud_rate: u32) -> Result<(), SerialPort::Error> {
        self.serial_port.set_baud_rate(baud_rate)?;
        self.baud_rate = baud_rate;
        self.discard_read_buffer()?;
        Ok(())
    }

    /// Drop everything buffered, in this crate and in the kernel.
    fn discard_read_buffer(&mut self) -> Result<(), SerialPort::Error> {
        self.read_len = 0;
        self.used_bytes = 0;
        self.serial_port.discard_input_buffer()
    }

    /// Direct access to the underlying serial port.
    pub fn serial_port(&mut self) -> &mut SerialPort {
        &mut self.serial_port
    }

    /// The padding added to every message timeout calculation.
    pub fn response_timeout_padding(&self) -> Duration {
        self.response_timeout_padding
    }

    /// Set the padding added to every message timeout calculation.
    pub fn set_response_timeout_padding(&mut self, padding: Duration) {
        self.response_timeout_padding = padding;
    }

    /// Write a raw instruction to a stream, and read a single raw response.
    ///
    /// Checks that the packet ID of the status response matches the instruction's, and that the
    /// error byte carries no [`crate::ERROR_FLAGS`]. [`crate::WARNING_FLAGS`] are allowed.
    pub(crate) async fn transfer_single<F>(
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
        self.write_packet(packet_id, instruction_id, parameter_count, encode_parameters)
            .await?;
        let response = self.read_response(expected_response_parameters).await?;
        crate::error::InvalidPacketId::check(response.motor_id, packet_id)?;
        Ok(response)
    }
    /// Build a packet into `buffer`, returning its length.
    pub(crate) fn make_packet<F>(
        buffer: &mut [u8],
        packet_id: u8,
        instruction_id: u8,
        parameter_count: usize,
        encode_parameters: F,
    ) -> Result<usize, WriteError<SerialPort::Error>>
    where
        F: FnOnce(&mut [u8]) -> Result<(), crate::error::BufferTooSmallError>,
    {
        let len = parameter_count + 2; // + CRC, INST

        TooManyParametersError::check(parameter_count, MAX_PARAMETER_COUNT)?;

        buffer[0] = HEADER_PREFIX[0];
        buffer[1] = HEADER_PREFIX[1];
        buffer[2] = packet_id;
        buffer[3] = len as u8;
        buffer[4] = instruction_id;
        encode_parameters(&mut buffer[5..][..parameter_count])?;

        let checksum_index = HEADER_SIZE + parameter_count + 1;
        let checksum = checksum::calculate_checksum(&buffer[2..checksum_index]);
        buffer[checksum_index] = checksum;

        Ok(checksum_index + 1)
    }
    pub(crate) async fn write_packet<F>(
        &mut self,
        packet_id: u8,
        instruction_id: u8,
        parameter_count: usize,
        encode_parameters: F,
    ) -> Result<(), WriteError<SerialPort::Error>>
    where
        F: FnOnce(&mut [u8]) -> Result<(), crate::error::BufferTooSmallError>,
    {
        // Not done when reading a reply: one instruction can draw several, and a single
        // read() can return more than one.
        self.discard_read_buffer().map_err(WriteError::DiscardBuffer)?;
        self.send_packet(packet_id, instruction_id, parameter_count, encode_parameters)
            .await
    }

    /// Build a packet into the write buffer and send it, leaving the read buffer alone.
    ///
    /// Split out of [`Self::write_packet`] for the device side: a device waiting its turn in
    /// a bulk read is watching for the packets of the motors ahead of it, so discarding on
    /// send would throw away a predecessor's reply.
    pub(crate) async fn send_packet<F>(
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
        self.send_buffered_packet(packet_len).await
    }

    async fn send_buffered_packet(&mut self, packet_len: usize) -> Result<(), WriteError<SerialPort::Error>> {
        let packet = &self.write_buffer.as_ref()[..packet_len];
        trace!("sending packet: {}", Hex(packet));
        self.serial_port.write_all(packet).await.map_err(WriteError::Write)?;
        Ok(())
    }

    pub(crate) async fn read_response(
        &mut self,
        expected_parameters: u8,
    ) -> Result<Response<&[u8]>, ReadError<SerialPort::Error>> {
        let timeout = message_transfer_time(expected_parameters as u32, self.baud_rate) + self.response_timeout_padding;
        self.read_response_timeout(timeout).await
    }

    async fn read_response_timeout(
        &mut self,
        timeout: Duration,
    ) -> Result<Response<&[u8]>, ReadError<SerialPort::Error>> {
        let deadline = self.serial_port.make_deadline(timeout);
        let packet = self.read_packet_deadline(deadline).await?;
        let response = Response {
            motor_id: packet[PACKET_ID],
            warning: ErrorFlags::from_bits_truncate(packet[PACKET_ERROR]),
            data: &packet[5..],
        };
        Ok(response)
    }

    /// returns a packet including header + parameters
    pub(crate) async fn read_packet_deadline(
        &mut self,
        deadline: SerialPort::Instant,
    ) -> Result<&[u8], ReadError<SerialPort::Error>> {
        let message_len = loop {
            self.remove_garbage();

            // `remove_garbage` left a header at the front, if there are bytes enough for one.
            if self.read_len > HEADER_SIZE {
                let body_len = self.read_buffer[PACKET_LEN] as usize;

                // Too small to frame, and the checksum does not catch it: in `FF FF FF 00` the
                // byte it points at is the `LEN` byte. Resync rather than trust `body_len`.
                if body_len < MIN_BODY_LEN {
                    self.consume_read_bytes(HEADER_PREFIX.len());
                    return Err(InvalidFrameLength {
                        len: body_len,
                        min: MIN_BODY_LEN,
                    }
                    .into());
                }

                // No upper check: the assertion at the top of this file makes one unfailable.
                if self.read_len >= HEADER_SIZE + body_len {
                    break HEADER_SIZE + body_len;
                }
            }

            let new_data = self
                .serial_port
                .read(&mut self.read_buffer.as_mut()[self.read_len..], &deadline)
                .await
                .map_err(ReadError::Io)?;
            if new_data == 0 {
                continue;
            }

            self.read_len += new_data;
            // Not a buffer dump: a `Device` sits in this loop continuously.
            trace!("read {} bytes, {} buffered", new_data, self.read_len);
        };

        let buffer = self.read_buffer.as_ref();
        let parameters_end = message_len - 1;
        trace!("read packet: {}", Hex(&buffer[..parameters_end]));

        let checksum_message = buffer[parameters_end];
        let checksum_computed = checksum::calculate_checksum(&buffer[2..parameters_end]);
        if checksum_message != checksum_computed {
            // `message_len` came from the frame that just failed, so resync from the header.
            self.consume_read_bytes(HEADER_PREFIX.len());
            return Err(crate::error::InvalidChecksum {
                message: checksum_message,
                computed: checksum_computed,
            }
            .into());
        }

        // Mark the message used, so the next `remove_garbage()` removes it.
        self.used_bytes += message_len;
        let packet = &self.read_buffer.as_ref()[..parameters_end];
        Ok(packet)
    }
    /// Remove leading garbage data from the read buffer.
    fn remove_garbage(&mut self) {
        let read_buffer = self.read_buffer.as_ref();
        let garbage_len = find_header(&read_buffer[..self.read_len][self.used_bytes..]);
        if garbage_len > 0 {
            debug!("skipping {} bytes of leading garbage.", garbage_len);
            // `garbage_len` counts from `used_bytes`, not from 0.
            trace!(
                "skipped garbage: {}",
                Hex(&read_buffer[self.used_bytes..][..garbage_len])
            );
        }
        self.consume_read_bytes(self.used_bytes + garbage_len);
        debug_assert_eq!(self.used_bytes, 0);
    }
    fn consume_read_bytes(&mut self, len: usize) {
        debug_assert!(len <= self.read_len);
        self.read_buffer.as_mut().copy_within(len..self.read_len, 0);
        // Consumed bytes may be garbage rather than used bytes, hence `saturating_sub`.
        self.used_bytes = self.used_bytes.saturating_sub(len);
        self.read_len -= len;
    }
}

/// Find the first possible starting position of a header.
///
/// A buffer ending in a partial header prefix returns that partial prefix's position.
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
