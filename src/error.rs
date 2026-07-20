//! The error types from communcication errors and motor error states

use core::fmt::{Display, Formatter, Result as FmtResult};
use derive_more::{Display, Error, From};

/// An error that can occur during a read/write transfer.
#[derive(Debug, Display, Error, From)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TransferError<E> {
    /// The write of failed.
    #[from]
    WriteError(WriteError<E>),

    /// The read failed.
    #[from(ReadError<E>, InvalidMessage, InvalidPacketId)]
    ReadError(ReadError<E>),
}

/// An error that can occur during a write transfer.
#[derive(Debug, Display, Error, From)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WriteError<E> {
    /// The write buffer is too small to contain the whole stuffed message.
    BufferTooSmall(BufferTooSmallError),

    /// A bulk request asked for more registers than the wire format can encode.
    TooManyRegisters(TooManyRegistersError),

    /// Failed to discard the input buffer before writing the instruction.
    #[from(skip)]
    DiscardBuffer(E),

    /// Failed to write the instruction.
    #[from(skip)]
    Write(E),
}

/// A bulk request specified more registers than the wire format can encode.
///
/// The bulk packet packs the read and write register counts into a single byte,
/// one 4-bit nibble each, so each direction supports at most 15 registers.
#[derive(Debug, Display, Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[display("too many bulk registers: {} requested, but at most {} are supported", self.count, self.max)]
pub struct TooManyRegistersError {
    /// The number of registers requested.
    pub count: usize,

    /// The maximum number of registers supported.
    pub max: usize,
}

impl TooManyRegistersError {
    /// Check that a register count fits within the supported maximum.
    pub fn check(count: usize, max: usize) -> Result<(), Self> {
        if count <= max {
            Ok(())
        } else {
            Err(Self { count, max })
        }
    }
}

/// The buffer is too small to hold the entire message.
///
/// Consider increasing the size of the buffer.
/// Keep in mind that the write buffer needs to be large enough to account for byte stuffing.
#[derive(Debug, Display, Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[display("buffer is too small: need {} bytes, but the size is {}", self.required_size, self.total_size)]
pub struct BufferTooSmallError {
    /// The required size of the buffer.
    pub required_size: usize,

    /// The total size of the buffer.
    pub total_size: usize,
}

/// An error that can occur during a read transfer.
#[derive(Debug, Display, Error, From)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReadError<E> {
    /// The read buffer is too small to contain the whole stuffed message.
    #[from]
    BufferFull(BufferTooSmallError),

    /// Failed to read from the serial port.
    #[from(skip)]
    Io(E),

    /// The received message is invalid.
    #[from(InvalidMessage, InvalidChecksum, InvalidPacketId)]
    InvalidMessage(InvalidMessage),
}

/// The received message is not valid.
#[derive(Debug, Clone, Eq, PartialEq, Display, Error, From)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InvalidMessage {
    /// The message checksum is invalid.
    InvalidChecksum(InvalidChecksum),

    /// The message has an invalid packet ID.
    InvalidPacketId(InvalidPacketId),

    /// The message has an invalid parameter count.
    InvalidParameterCount(InvalidParameterCount),
}

/// The received message has an invalid checksum value.
#[derive(Debug, Clone, Eq, PartialEq, Display, Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[display("invalid checksum, message claims {:#02X}, computed {:#02X}", self.message, self.computed)]
pub struct InvalidChecksum {
    /// The checksum from the messsage.
    pub message: u8,

    /// The actual checksum.
    pub computed: u8,
}

/// The received message has an invalid or unexpected packet ID.
#[derive(Debug, Clone, Eq, PartialEq, Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidPacketId {
    /// The actual packet ID.
    pub actual: u8,

    /// The expected packet ID (if a specific ID was expected).
    pub expected: Option<u8>,
}
impl Display for InvalidPacketId {
    fn fmt(&self, f: &mut Formatter) -> FmtResult {
        if let Some(expected) = self.expected {
            write!(
                f,
                "invalid packet ID, expected {:#02X}, got {:#02X}",
                expected, self.actual
            )
        } else {
            write!(f, "invalid packet ID: {:#02X}", self.actual)
        }
    }
}

/// The expected number of parameters.
#[derive(Debug, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ExpectedCount {
    /// The exact number of expected parameters.
    Exact(usize),

    /// An upper limit on the expected number of parameters.
    Max(usize),

    /// A lower limit on the expected number of parameters.
    Min(usize),
}

impl core::error::Error for ExpectedCount {}

impl Display for ExpectedCount {
    fn fmt(&self, f: &mut Formatter) -> FmtResult {
        match self {
            Self::Exact(x) => write!(f, "exactly {}", x),
            Self::Max(x) => write!(f, "at most {}", x),
            Self::Min(x) => write!(f, "at least {}", x),
        }
    }
}
/// The received message has an invalid or unexpected parameter count.
#[derive(Debug, Clone, Eq, PartialEq, Display, Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[display("invalid parameter count, expected {}, got {}", self.expected, self.actual)]
pub struct InvalidParameterCount {
    /// The actual parameter count.
    pub actual: usize,

    /// The expected parameter count.
    pub expected: ExpectedCount,
}

impl BufferTooSmallError {
    /// Check if a buffer is large enough for the required total size.
    pub fn check(required_size: usize, total_size: usize) -> Result<(), Self> {
        if required_size <= total_size {
            Ok(())
        } else {
            Err(Self {
                required_size,
                total_size,
            })
        }
    }
}

impl InvalidPacketId {
    /// Check if the packet ID matches the expected value.
    pub fn check(actual: u8, expected: u8) -> Result<(), Self> {
        if actual == expected {
            Ok(())
        } else {
            Err(Self {
                actual,
                expected: Some(expected),
            })
        }
    }
}

impl InvalidParameterCount {
    /// Check if the parameter count matches the expected count.
    pub fn check(actual: usize, expected: usize) -> Result<(), Self> {
        if actual == expected {
            Ok(())
        } else {
            Err(Self {
                actual,
                expected: ExpectedCount::Exact(expected),
            })
        }
    }

    /// Check if the parameter count is at or below the max count.
    pub fn check_max(actual: usize, max: usize) -> Result<(), Self> {
        if actual <= max {
            Ok(())
        } else {
            Err(Self {
                actual,
                expected: ExpectedCount::Max(max),
            })
        }
    }

    /// Check if the parameter count is at or above the min count.
    pub fn check_min(actual: usize, min: usize) -> Result<(), Self> {
        if actual >= min {
            Ok(())
        } else {
            Err(Self {
                actual,
                expected: ExpectedCount::Min(min),
            })
        }
    }
}
