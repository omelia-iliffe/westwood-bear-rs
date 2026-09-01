//! The device half of the bus: acting *as* a BEAR motor rather than driving one.
//!
//! [`Bus`] writes instructions and reads replies. [`Device`] does the opposite,
//! over the same framing code, so a device implementation gets header scanning,
//! length-driven reassembly and checksum validation for free.
//!
//! What a device is responsible for beyond framing:
//!
//! - **Answering only what it should.** Serve packets addressed to one of your
//!   IDs, plus [`Instruction::BulkComm`](crate::Instruction::BulkComm) on
//!   [`BROADCAST_ID`]; [`Packet::addresses`] is that filter. Check
//!   [`PacketKind::Status`] before it: the bus is half-duplex, so you also see
//!   other motors' replies, and those are addressed to nobody but are what
//!   drives bulk ordering.
//!
//! [`PacketKind::Status`]: crate::PacketKind::Status
//! [`Packet::addresses`]: crate::Packet::addresses
//! - **Replying only where the protocol says to.** Ping and the two read
//!   instructions get a reply. Writes and [`Instruction::SaveCfg`] get **none** —
//!   BEAR has no acknowledgement, so a client verifies by reading back.
//! - **Bulk ordering.** See [`BulkComm::predecessor`].
//!
//! [`Instruction::SaveCfg`]: crate::Instruction::SaveCfg
//!
//! # Replying to what you just read
//!
//! [`Self::read`] hands back a [`Packet`] borrowing this device's read buffer,
//! and [`Self::write_status`] needs the device mutably, so the two cannot
//! overlap. A handler whose reply depends on the request has to detach it
//! first with [`Packet::copy_into`]:
//!
//! ```text
//! let mut scratch = [0u8; MAX_PARAMETER_COUNT];
//! loop {
//!     let packet = device.read(timeout)?.copy_into(&mut scratch)?;
//!
//!     // Another motor's reply. `addresses` returns false for these, so the
//!     // filter below would drop the packets bulk ordering waits on.
//!     if let PacketKind::Status { .. } = packet.kind {
//!         if predecessor == Some(packet.id) {
//!             // Our turn: send the reply prepared when the bulk packet arrived.
//!         }
//!         continue;
//!     }
//!
//!     if !packet.addresses(my_id) {
//!         continue;
//!     }
//!
//!     if let PacketKind::ReadStat { registers } = packet.kind {
//!         device.write_status(my_id, ErrorFlags::empty(), registers.len() * 4, |out| {
//!             for (slot, register) in out.chunks_exact_mut(4).zip(registers) {
//!                 slot.copy_from_slice(&table[usize::from(*register)].to_le_bytes());
//!             }
//!             Ok(())
//!         })?;
//!     }
//! }
//! ```
//!
//! `predecessor` is [`BulkComm::predecessor`] for your position in the last
//! bulk packet. Size `scratch` for the traffic you serve;
//! [`MAX_PARAMETER_COUNT`] covers every legal packet.
//!
//! A device that only decides whether a packet concerns it never needs the
//! copy, which is why `read` does not make it for you. `tests/device.rs` has
//! the same loop as a working test.
//!
//! [`Packet`]: crate::Packet
//! [`Packet::copy_into`]: crate::Packet::copy_into

use crate::ErrorFlags;
use crate::error::{ReadError, WriteError};
use crate::protocol::{Packet, REGISTER_BYTES, STATUS_FLAG};
use core::time::Duration;
// `super`, not `crate`: `bisync2` compiles this file into both the synchronous and
// asynchronous trees, and each has its own `Bus` and `SerialPort`. Naming them
// through `crate` would pin both trees to the synchronous pair.
use super::Bus;
use super::bus::DefaultBuffer;

#[cfg(doc)]
use crate::{BulkComm, MAX_PACKET_SIZE, MAX_PARAMETER_COUNT};

/// A device on a BEAR bus: something a client sends instructions to.
///
/// Wraps a [`Bus`] purely to reuse its framing loop. The client instruction
/// methods on that `Bus` are not reachable through this type, because a device
/// answers the bus rather than driving it.
#[derive(Debug)]
pub struct Device<SerialPort, Buffer = DefaultBuffer>
where
    SerialPort: super::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    bus: Bus<SerialPort, Buffer>,
}

#[cfg(feature = "alloc")]
impl<SerialPort> Device<SerialPort, alloc::vec::Vec<u8>>
where
    SerialPort: super::SerialPort,
{
    /// Create a device using an open serial port, allocating [`MAX_PACKET_SIZE`]
    /// byte buffers.
    ///
    /// The serial port must already be configured in raw mode with the correct
    /// baud rate, character size (8), parity (disabled) and stop bits (1).
    pub fn new(serial_port: SerialPort) -> Result<Self, SerialPort::Error> {
        Ok(Self {
            bus: Bus::new(serial_port)?,
        })
    }
}

#[super::bisync]
impl<SerialPort, Buffer> Device<SerialPort, Buffer>
where
    SerialPort: super::SerialPort,
    Buffer: AsRef<[u8]> + AsMut<[u8]>,
{
    /// Create a device using pre-allocated buffers.
    ///
    /// [`MAX_PACKET_SIZE`] is the only size that can carry every legal frame,
    /// because the one-byte `LEN` field can describe nothing larger. Smaller
    /// buffers are workable if you know what will be on your bus, but size them
    /// against the traffic, not against the common case: a reply to a full
    /// config table read is already `ConfigRegister::COUNT * 4 + 6` = 130 bytes,
    /// and a client writing that table in one packet sends 161. A frame that
    /// does not fit is reported as `ReadError::BufferFull` and dropped.
    ///
    /// [`ConfigRegister::COUNT`]: crate::ConfigRegister::COUNT
    pub fn with_buffers(
        serial_port: SerialPort,
        read_buffer: Buffer,
        write_buffer: Buffer,
    ) -> Result<Self, SerialPort::Error> {
        Ok(Self {
            bus: Bus::with_buffers(serial_port, read_buffer, write_buffer)?,
        })
    }

    /// Create a device using pre-allocated buffers and a known baud rate.
    pub fn with_buffers_and_baud_rate(
        serial_port: SerialPort,
        read_buffer: Buffer,
        write_buffer: Buffer,
        baud_rate: u32,
    ) -> Self {
        Self {
            bus: Bus::with_buffers_and_baud_rate(serial_port, read_buffer, write_buffer, baud_rate),
        }
    }

    /// The baud rate the device believes the bus is running at.
    pub fn baud_rate(&self) -> u32 {
        self.bus.baud_rate
    }

    /// Change the baud rate.
    ///
    /// A client changes a motor's baud rate by writing
    /// [`ConfigRegister::BaudRate`](crate::ConfigRegister::BaudRate). Applying it
    /// means switching the port under a bus that is still running, so bytes in
    /// flight are lost either way; the motor firmware accepts that and so should
    /// a device.
    ///
    /// Whatever was already buffered is discarded with the switch. Those bytes
    /// were sampled at the old rate, and framing them afterwards would deliver a
    /// packet assembled across two signalling rates as though it were real.
    pub fn set_baud_rate(&mut self, baud_rate: u32) -> Result<(), SerialPort::Error> {
        self.bus.set_baud_rate(baud_rate)
    }

    /// Direct access to the underlying serial port.
    pub fn serial_port(&mut self) -> &mut SerialPort {
        self.bus.serial_port()
    }

    /// Build a deadline `timeout` from now.
    pub fn make_deadline(&self, timeout: Duration) -> SerialPort::Instant {
        self.bus.serial_port.make_deadline(timeout)
    }

    /// Read the next packet from the bus, waiting at most `timeout`.
    ///
    /// Returns everything on the wire, not only packets addressed to this
    /// device: filtering is the caller's job, because a device holding several
    /// IDs decides that for itself, and because other motors' replies are what
    /// drives bulk ordering.
    pub async fn read(&mut self, timeout: Duration) -> Result<Packet<&[u8]>, ReadError<SerialPort::Error>> {
        let deadline = self.bus.serial_port.make_deadline(timeout);
        self.read_deadline(deadline).await
    }

    /// Read the next packet from the bus against an existing deadline.
    ///
    /// Use this rather than [`Self::read`] when waiting for a predecessor's reply
    /// during a bulk read, so that repeated calls share one budget instead of
    /// each restarting the clock.
    pub async fn read_deadline(
        &mut self,
        deadline: SerialPort::Instant,
    ) -> Result<Packet<&[u8]>, ReadError<SerialPort::Error>> {
        let packet = self.bus.read_packet_deadline(deadline).await?;
        Ok(Packet::from_bytes(packet)?)
    }

    /// Send a status packet.
    ///
    /// `parameter_count` is in bytes and must be a multiple of 4: a reply
    /// carries one 4-byte little-endian word per register read.
    ///
    /// [`STATUS_FLAG`] is set here rather than trusted to the caller. A status
    /// packet without it is invisible to every motor sequenced behind this one
    /// in a bulk read, which then waits out its timeout and drops its own reply
    /// — a failure that shows up as an unrelated motor going quiet.
    pub async fn write_status<F>(
        &mut self,
        id: u8,
        error: ErrorFlags,
        parameter_count: usize,
        encode_parameters: F,
    ) -> Result<(), WriteError<SerialPort::Error>>
    where
        F: FnOnce(&mut [u8]) -> Result<(), crate::error::BufferTooSmallError>,
    {
        debug_assert_eq!(
            parameter_count % REGISTER_BYTES,
            0,
            "a status packet carries whole 4-byte registers"
        );
        // `make_packet` puts this byte where an instruction goes in a request,
        // which is exactly where the error byte goes in a reply.
        self.bus
            .send_packet(id, error.bits() | STATUS_FLAG, parameter_count, encode_parameters)
            .await
    }

    /// Send a status packet whose payload is already encoded.
    ///
    /// The common case on the reply path: the words were assembled while waiting
    /// for a turn in a bulk read, and only need framing.
    pub async fn write_status_bytes(
        &mut self,
        id: u8,
        error: ErrorFlags,
        parameters: &[u8],
    ) -> Result<(), WriteError<SerialPort::Error>> {
        self.write_status(id, error, parameters.len(), |buffer| {
            buffer.copy_from_slice(parameters);
            Ok(())
        })
        .await
    }
}
