//! Device-side view of the wire: the packets a motor sees on the bus.
//!
//! The client half of this crate builds instruction packets and parses status
//! replies. A device does the opposite, and it also has to recognise *other*
//! devices' replies, because that is how [`Instruction::BulkComm`] ordering
//! works (see [`BulkComm::predecessor`]).
//!
//! Parsing here is pure: [`Packet::from_bytes`] borrows the read buffer and does
//! no I/O and no allocation, so it can be unit tested against frames built by
//! the client half of this same crate.
//!
//! ```text
//! request   FF FF | id | len | inst  | parameters | checksum
//! status    FF FF | id | len | error | parameters | checksum
//! ```
//!
//! Byte 4 is the instruction in a request and the error byte in a reply. The two
//! are told apart by bit 7: a motor always sets it in the error byte
//! ([`STATUS_FLAG`]) and no instruction opcode reaches `0x80`.
//!
//! # Borrowing
//!
//! A parsed packet points into the buffer it came from, which for a
//! [`Device`](crate::Device) is the device's own read buffer. Sending a reply
//! needs that same device mutably, so a reply whose contents depend on the
//! request has to detach the request first: see [`Packet::copy_into`].

use crate::error::{BufferTooSmallError, ExpectedCount, InvalidMessage, InvalidParameterCount};
use crate::protocol::{PACKET_ERROR, PACKET_ID, REGISTER_BYTES};
use crate::{ConfigRegister, Instruction, StatusRegister};

/// Bit 7 of the error byte, set on every status packet a motor sends.
///
/// It is not an error flag. It exists so that a device waiting its turn in a
/// bulk read can tell a predecessor's reply from an instruction without
/// tracking bus direction, and the motor firmware gates on it directly. A device
/// that forgets to set it is invisible to every motor sequenced after it, which
/// then time out and silently drop their own replies.
pub const STATUS_FLAG: u8 = 0x80;

/// Broadcast ID. Only [`Instruction::BulkComm`] is accepted on it.
///
/// Every other instruction is point to point. [`Packet::addresses`] enforces
/// that: a device answering, say, a broadcast ping would transmit at the same
/// moment as every other motor on a half-duplex bus, and all of the replies
/// would collide.
pub const BROADCAST_ID: u8 = 0xFE;

/// A packet observed on the bus by a device.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Packet<T> {
    /// The ID field: the addressed motor for a request, the responder for a reply.
    pub id: u8,

    /// What the packet turned out to be.
    pub kind: PacketKind<T>,
}

/// The kinds of packet a device can see.
///
/// `T` is the parameter payload, `&[u8]` when borrowing the read buffer and an
/// owned buffer after [`Packet::copy_into`] or [`Packet::into_owned`].
// Not `Eq`: `SetAbsPos` carries `f32`.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum PacketKind<T> {
    /// [`Instruction::Ping`]. Reply with the identification word.
    Ping,

    /// [`Instruction::ReadStat`]. One byte per [`StatusRegister`] requested.
    ReadStat {
        /// Register indices, one byte each.
        registers: T,
    },

    /// [`Instruction::WriteStat`]. Repeating index-then-value. **No reply.**
    WriteStat {
        /// Repeating `{ index, 4-byte little-endian value }`.
        parameters: T,
    },

    /// [`Instruction::ReadCfg`]. One byte per [`ConfigRegister`] requested.
    ReadCfg {
        /// Register indices, one byte each.
        registers: T,
    },

    /// [`Instruction::WriteCfg`]. Repeating index-then-value. **No reply.**
    WriteCfg {
        /// Repeating `{ index, 4-byte little-endian value }`.
        parameters: T,
    },

    /// [`Instruction::SaveCfg`]. Persist the config table. **No reply.**
    SaveCfg,

    /// [`Instruction::SetAbsPos`]. **No reply.**
    SetAbsPos {
        /// Expected absolute position.
        theta: f32,
        /// Tolerance around it.
        tolerance: f32,
    },

    /// [`Instruction::BulkComm`], broadcast.
    ///
    /// Already parsed: the nibble-packed layout is validated when the frame is
    /// decoded, and the result is kept rather than thrown away, so walking the
    /// packet costs nothing extra in the reply window.
    BulkComm {
        /// The parsed parameter block.
        bulk: BulkComm<T>,
    },

    /// A status packet, sent by another motor rather than by the client.
    ///
    /// Seeing one from the right ID is the signal to send your own reply during
    /// a bulk read.
    Status {
        /// The error byte verbatim, [`STATUS_FLAG`] included.
        error: u8,
        /// The reply payload: 4 bytes per register read.
        parameters: T,
    },

    /// An opcode this crate does not know.
    ///
    /// The motor firmware answers these by latching a communication warning and
    /// sending nothing, so a device should do the same rather than guess.
    Unknown {
        /// The unrecognised opcode.
        instruction: u8,
        /// Whatever followed it.
        parameters: T,
    },
}

impl<T> PacketKind<T> {
    /// Rebuild this kind around a different payload representation.
    fn map<U>(self, payload: impl FnOnce(T) -> U) -> PacketKind<U> {
        match self {
            PacketKind::Ping => PacketKind::Ping,
            PacketKind::SaveCfg => PacketKind::SaveCfg,
            PacketKind::SetAbsPos { theta, tolerance } => PacketKind::SetAbsPos { theta, tolerance },
            PacketKind::ReadStat { registers } => PacketKind::ReadStat {
                registers: payload(registers),
            },
            PacketKind::ReadCfg { registers } => PacketKind::ReadCfg {
                registers: payload(registers),
            },
            PacketKind::WriteStat { parameters } => PacketKind::WriteStat {
                parameters: payload(parameters),
            },
            PacketKind::WriteCfg { parameters } => PacketKind::WriteCfg {
                parameters: payload(parameters),
            },
            PacketKind::BulkComm { bulk } => PacketKind::BulkComm {
                bulk: bulk.map(payload),
            },
            PacketKind::Status { error, parameters } => PacketKind::Status {
                error,
                parameters: payload(parameters),
            },
            PacketKind::Unknown {
                instruction,
                parameters,
            } => PacketKind::Unknown {
                instruction,
                parameters: payload(parameters),
            },
        }
    }
}

impl<T: AsRef<[u8]>> PacketKind<T> {
    /// The parameter bytes this packet carries.
    ///
    /// Empty for the kinds that carry none, and for [`PacketKind::SetAbsPos`],
    /// whose two values are decoded into the variant rather than left as bytes.
    pub fn parameters(&self) -> &[u8] {
        match self {
            PacketKind::Ping | PacketKind::SaveCfg | PacketKind::SetAbsPos { .. } => &[],
            PacketKind::ReadStat { registers } | PacketKind::ReadCfg { registers } => registers.as_ref(),
            PacketKind::WriteStat { parameters } | PacketKind::WriteCfg { parameters } => parameters.as_ref(),
            PacketKind::BulkComm { bulk } => bulk.parameters(),
            PacketKind::Status { parameters, .. } => parameters.as_ref(),
            PacketKind::Unknown { parameters, .. } => parameters.as_ref(),
        }
    }
}

/// A register write parsed out of a [`PacketKind::WriteStat`] or
/// [`PacketKind::WriteCfg`] parameter block.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RegisterWrite {
    /// The register index. Raw rather than a [`StatusRegister`] /
    /// [`ConfigRegister`], because a motor answers any in-range index and a
    /// device emulating one has to do the same.
    pub register: u8,

    /// The value, still little-endian. Decode with [`RegisterWrite::f32`] or
    /// [`RegisterWrite::u32`] according to the register's type.
    pub value: [u8; REGISTER_BYTES],
}

impl RegisterWrite {
    /// Decode the value as an `f32`.
    pub fn f32(&self) -> f32 {
        f32::from_le_bytes(self.value)
    }

    /// Decode the value as a `u32`.
    pub fn u32(&self) -> u32 {
        u32::from_le_bytes(self.value)
    }
}

/// Iterator over the writes in a write instruction's parameter block.
#[derive(Debug, Clone)]
pub struct RegisterWrites<'a> {
    parameters: &'a [u8],
}

/// Bytes per entry in a write instruction: the register index plus its value.
const WRITE_STRIDE: usize = 1 + REGISTER_BYTES;

impl Iterator for RegisterWrites<'_> {
    type Item = RegisterWrite;

    fn next(&mut self) -> Option<Self::Item> {
        let (chunk, rest) = self.parameters.split_at_checked(WRITE_STRIDE)?;
        self.parameters = rest;
        Some(RegisterWrite {
            register: chunk[0],
            // `split_at_checked` guarantees the length, so this cannot fail.
            value: chunk[1..].try_into().ok()?,
        })
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.len();
        (remaining, Some(remaining))
    }
}

impl ExactSizeIterator for RegisterWrites<'_> {
    fn len(&self) -> usize {
        self.parameters.len() / WRITE_STRIDE
    }
}

/// Walk a write instruction's parameter block.
///
/// The block is validated to be a whole number of entries when the packet is
/// parsed, so the iterator never yields a partial write.
pub fn register_writes(parameters: &[u8]) -> RegisterWrites<'_> {
    RegisterWrites { parameters }
}

impl<'a> Packet<&'a [u8]> {
    /// Parse a packet that the framing layer has already checksummed.
    ///
    /// `packet` is the frame from the header up to but not including the
    /// checksum byte, which is what the read path hands back.
    pub fn from_bytes(packet: &'a [u8]) -> Result<Self, InvalidMessage> {
        // Header, id, len, and the instruction/error byte. Anything shorter is
        // not a frame the checksum layer should have passed up.
        if packet.len() < PACKET_ERROR + 1 {
            return Err(InvalidParameterCount {
                actual: packet.len(),
                expected: ExpectedCount::Min(PACKET_ERROR + 1),
            }
            .into());
        }

        let id = packet[PACKET_ID];
        let instruction = packet[PACKET_ERROR];
        let parameters = &packet[PACKET_ERROR + 1..];

        // Bit 7 separates a motor's reply from a client's request. Checked
        // before the opcode match so a reply can never be mistaken for an
        // instruction that happens to share its low bits.
        if instruction & STATUS_FLAG != 0 {
            return Ok(Packet {
                id,
                kind: PacketKind::Status {
                    error: instruction,
                    parameters,
                },
            });
        }

        let kind = match instruction {
            x if x == Instruction::Ping as u8 => {
                exact(parameters, 0)?;
                PacketKind::Ping
            },
            x if x == Instruction::SaveCfg as u8 => {
                exact(parameters, 0)?;
                PacketKind::SaveCfg
            },
            x if x == Instruction::ReadStat as u8 => {
                registers(parameters, StatusRegister::COUNT)?;
                PacketKind::ReadStat { registers: parameters }
            },
            x if x == Instruction::ReadCfg as u8 => {
                registers(parameters, ConfigRegister::COUNT)?;
                PacketKind::ReadCfg { registers: parameters }
            },
            x if x == Instruction::WriteStat as u8 => {
                writes(parameters)?;
                PacketKind::WriteStat { parameters }
            },
            x if x == Instruction::WriteCfg as u8 => {
                writes(parameters)?;
                PacketKind::WriteCfg { parameters }
            },
            x if x == Instruction::SetAbsPos as u8 => {
                exact(parameters, 2 * REGISTER_BYTES)?;
                PacketKind::SetAbsPos {
                    // Lengths are checked above, so neither slice can fail.
                    theta: f32::from_le_bytes(parameters[..4].try_into().unwrap_or_default()),
                    tolerance: f32::from_le_bytes(parameters[4..].try_into().unwrap_or_default()),
                }
            },
            x if x == Instruction::BulkComm as u8 => PacketKind::BulkComm {
                // Parsed once, here. The nibble-packed layout has to be
                // validated before the packet is handed up anyway, so keeping
                // the result spares the device a second walk in the reply
                // window.
                bulk: BulkComm::parse(parameters)?,
            },
            instruction => PacketKind::Unknown {
                instruction,
                parameters,
            },
        };

        Ok(Packet { id, kind })
    }

    /// Copy the payload into `buffer`, detaching the packet from the read buffer.
    ///
    /// A packet returned by [`Device::read`] borrows the device, and sending a
    /// reply needs that device mutably, so the two cannot overlap. Any handler
    /// whose reply depends on the request has to copy first:
    ///
    /// ```text
    /// let mut scratch = [0u8; ConfigRegister::COUNT];
    /// let packet = device.read(timeout)?.copy_into(&mut scratch)?;
    /// // `device` is free again here.
    /// if let PacketKind::ReadStat { registers } = packet.kind {
    ///     device.write_status(id, ErrorFlags::empty(), registers.len() * 4, |out| { .. })?;
    /// }
    /// ```
    ///
    /// The copy is explicit rather than hidden inside `read` because a device
    /// that only has to decide whether a packet concerns it at all never needs
    /// one. With the `alloc` feature, [`Self::into_owned`] does the same without
    /// a caller-supplied buffer.
    ///
    /// [`Device::read`]: crate::Device::read
    pub fn copy_into(self, buffer: &mut [u8]) -> Result<Packet<&[u8]>, BufferTooSmallError> {
        let parameters = self.kind.parameters();
        BufferTooSmallError::check(parameters.len(), buffer.len())?;

        let copy = &mut buffer[..parameters.len()];
        copy.copy_from_slice(parameters);

        Ok(Packet {
            id: self.id,
            kind: self.kind.map(|_| &*copy),
        })
    }

    /// Copy the payload into an owned buffer. The allocating [`Self::copy_into`].
    #[cfg(feature = "alloc")]
    pub fn into_owned(self) -> Packet<alloc::vec::Vec<u8>> {
        Packet {
            id: self.id,
            kind: self.kind.map(<[u8]>::to_vec),
        }
    }
}

impl<T> Packet<T> {
    /// Is this packet addressed to `id`, or to every device on the bus?
    ///
    /// A device's dispatch filter. It takes the ID to test rather than reading
    /// one off the packet, because a device may hold several IDs and has to try
    /// each.
    ///
    /// Two cases are not addressed to anyone:
    ///
    /// - A [`PacketKind::Status`], whose ID field names the motor that *sent*
    ///   it. Matching that against your own would make you answer your own reply.
    /// - Anything other than [`Instruction::BulkComm`] on [`BROADCAST_ID`].
    ///   Bulk is the only broadcast instruction the protocol defines, and it
    ///   carries its own per-motor reply ordering. Replying to a broadcast of
    ///   any other kind would put every motor on the half-duplex bus into
    ///   transmit at once.
    pub fn addresses(&self, id: u8) -> bool {
        match self.kind {
            PacketKind::Status { .. } => false,
            PacketKind::BulkComm { .. } => self.id == id || self.id == BROADCAST_ID,
            _ => self.id == id,
        }
    }
}

/// A parsed [`Instruction::BulkComm`] parameter block.
///
/// ```text
/// [motor_count][(read_count << 4) | write_count]
/// [read register indices ..][write register indices ..]
/// then per motor: [id][write value ..]
/// ```
///
/// The counts share one byte as nibbles, so neither can exceed 15. Only the
/// three counts are stored: every section is at a fixed offset once they are
/// known, so the accessors slice the original block rather than duplicating it,
/// and the type stays generic over borrowed and owned payloads alike.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BulkComm<T> {
    parameters: T,
    read_count: usize,
    write_count: usize,
    motor_count: usize,
}

impl<T> BulkComm<T> {
    /// `motor_count` and the packed register counts.
    const HEADER_LEN: usize = 2;

    /// Rebuild around a different payload representation. The counts are already
    /// validated against the block, and copying does not change it.
    fn map<U>(self, payload: impl FnOnce(T) -> U) -> BulkComm<U> {
        BulkComm {
            parameters: payload(self.parameters),
            read_count: self.read_count,
            write_count: self.write_count,
            motor_count: self.motor_count,
        }
    }
}

impl<T: AsRef<[u8]>> BulkComm<T> {
    /// Parse the parameter block of a [`PacketKind::BulkComm`].
    ///
    /// [`Packet::from_bytes`] does this for you; the result is kept in the
    /// variant. Call it directly only when decoding a block from somewhere else.
    pub fn parse(parameters: T) -> Result<Self, InvalidMessage> {
        let bytes = parameters.as_ref();

        let Some(header) = bytes.get(..Self::HEADER_LEN) else {
            return Err(InvalidParameterCount {
                actual: bytes.len(),
                expected: ExpectedCount::Min(Self::HEADER_LEN),
            }
            .into());
        };

        let motor_count = header[0] as usize;
        let read_count = (header[1] >> 4) as usize;
        let write_count = (header[1] & 0x0F) as usize;
        let write_len = write_count * REGISTER_BYTES;

        let expected = Self::HEADER_LEN + read_count + write_count + motor_count * (1 + write_len);
        if bytes.len() != expected {
            return Err(InvalidParameterCount {
                actual: bytes.len(),
                expected: ExpectedCount::Exact(expected),
            }
            .into());
        }

        Ok(BulkComm {
            parameters,
            read_count,
            write_count,
            motor_count,
        })
    }

    /// The whole parameter block, header included.
    pub fn parameters(&self) -> &[u8] {
        self.parameters.as_ref()
    }

    /// The [`StatusRegister`] indices to read from every listed motor.
    ///
    /// Empty means write-only, in which case no motor replies at all.
    pub fn read_registers(&self) -> &[u8] {
        &self.parameters.as_ref()[Self::HEADER_LEN..][..self.read_count]
    }

    /// The [`StatusRegister`] indices to write on every listed motor.
    pub fn write_registers(&self) -> &[u8] {
        &self.parameters.as_ref()[Self::HEADER_LEN + self.read_count..][..self.write_count]
    }

    /// The per-motor block: `motor_count` entries of `1 + write_len` bytes.
    fn motors(&self) -> &[u8] {
        &self.parameters.as_ref()[Self::HEADER_LEN + self.read_count + self.write_count..]
    }

    /// Bytes per entry in the per-motor block: the ID plus its write values.
    fn stride(&self) -> usize {
        1 + self.write_count * REGISTER_BYTES
    }

    /// How many motors the packet addresses.
    pub fn motor_count(&self) -> usize {
        self.motor_count
    }

    /// Bytes each motor's reply payload will carry.
    pub fn reply_len(&self) -> usize {
        self.read_count * REGISTER_BYTES
    }

    /// Walk the addressed motors in packet order.
    pub fn entries(&self) -> BulkCommEntries<'_> {
        BulkCommEntries {
            motors: self.motors(),
            write_registers: self.write_registers(),
            stride: self.stride(),
            position: 0,
        }
    }

    /// Find a motor's entry, or `None` if this packet does not address it.
    pub fn find(&self, motor_id: u8) -> Option<BulkCommEntry<'_>> {
        self.entries().find(|entry| entry.motor_id == motor_id)
    }

    /// The ID whose reply immediately precedes `position`, or `None` at the head
    /// of the list.
    ///
    /// This is the whole of bulk read ordering. The motor at position 0 replies
    /// as soon as its return delay elapses; every other motor waits until it has
    /// seen a [`PacketKind::Status`] from the ID this returns, then sends. A
    /// device holding a contiguous run of IDs therefore waits once and then
    /// emits its whole run back to back.
    pub fn predecessor(&self, position: usize) -> Option<u8> {
        position
            .checked_sub(1)
            .and_then(|previous| self.entries().nth(previous))
            .map(|entry| entry.motor_id)
    }
}

/// One motor's entry in a [`BulkComm`] packet.
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BulkCommEntry<'a> {
    /// Position in the packet's motor list, which fixes the reply order.
    pub position: usize,

    /// The addressed motor.
    pub motor_id: u8,

    /// This motor's write values, 4 bytes per entry in
    /// [`BulkComm::write_registers`]. Empty on a read-only bulk.
    pub write_data: &'a [u8],

    /// The register indices `write_data` belongs to.
    ///
    /// Private, and carried rather than passed back in: the two are only
    /// meaningful together, and pairing `write_data` with a register list from
    /// anywhere else silently drops or invents writes.
    write_registers: &'a [u8],
}

impl<'a> BulkCommEntry<'a> {
    /// The register indices this entry's [`Self::write_data`] belongs to.
    ///
    /// The same slice for every entry in a packet: a bulk write sets the same
    /// registers on every motor it lists, with per-motor values.
    pub fn write_registers(&self) -> &'a [u8] {
        self.write_registers
    }

    /// Walk this motor's write values alongside their register indices.
    ///
    /// The pairing cannot be wrong: [`BulkComm::parse`] has already checked that
    /// the block holds exactly `write_registers.len() * 4` bytes per motor.
    pub fn writes(&self) -> impl Iterator<Item = RegisterWrite> + 'a {
        debug_assert_eq!(
            self.write_data.len(),
            self.write_registers.len() * REGISTER_BYTES,
            "bulk entry write data does not match its register list"
        );
        self.write_registers
            .iter()
            .copied()
            .zip(self.write_data.chunks_exact(REGISTER_BYTES))
            .map(|(register, value)| RegisterWrite {
                register,
                value: value.try_into().unwrap_or_default(),
            })
    }
}

/// Iterator over the motors addressed by a [`BulkComm`] packet.
#[derive(Debug, Clone)]
pub struct BulkCommEntries<'a> {
    motors: &'a [u8],
    write_registers: &'a [u8],
    stride: usize,
    position: usize,
}

impl<'a> Iterator for BulkCommEntries<'a> {
    type Item = BulkCommEntry<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let (entry, rest) = self.motors.split_at_checked(self.stride)?;
        self.motors = rest;
        let position = self.position;
        self.position += 1;
        Some(BulkCommEntry {
            position,
            motor_id: entry[0],
            write_data: &entry[1..],
            write_registers: self.write_registers,
        })
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.len();
        (remaining, Some(remaining))
    }
}

impl ExactSizeIterator for BulkCommEntries<'_> {
    fn len(&self) -> usize {
        self.motors.len() / self.stride
    }
}

/// Require an exact parameter count.
fn exact(parameters: &[u8], expected: usize) -> Result<(), InvalidMessage> {
    if parameters.len() == expected {
        Ok(())
    } else {
        Err(InvalidParameterCount {
            actual: parameters.len(),
            expected: ExpectedCount::Exact(expected),
        }
        .into())
    }
}

/// A read instruction carries one byte per register and must ask for at least
/// one. The motor firmware refuses a request for more registers than the table
/// holds by sending nothing at all, so the count is bounded here too.
fn registers(parameters: &[u8], max: usize) -> Result<(), InvalidMessage> {
    if parameters.is_empty() {
        return Err(InvalidParameterCount {
            actual: 0,
            expected: ExpectedCount::Min(1),
        }
        .into());
    }
    if parameters.len() > max {
        return Err(InvalidParameterCount {
            actual: parameters.len(),
            expected: ExpectedCount::Max(max),
        }
        .into());
    }
    Ok(())
}

/// A write instruction carries a whole number of index-plus-value entries.
fn writes(parameters: &[u8]) -> Result<(), InvalidMessage> {
    if parameters.is_empty() || !parameters.len().is_multiple_of(WRITE_STRIDE) {
        return Err(InvalidParameterCount {
            actual: parameters.len(),
            expected: ExpectedCount::Min(WRITE_STRIDE),
        }
        .into());
    }
    Ok(())
}
