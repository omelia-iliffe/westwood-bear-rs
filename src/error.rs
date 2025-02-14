use crate::protocol::ERROR_FLAGS;
use crate::ErrorFlags;
use core::fmt::{Display, Formatter, Result as FmtResult};
use derive_more::{Display, Error, From};

/// An error that can occur during a read/write transfer.
#[derive(Debug, Display, Error, From)]
pub enum TransferError<E> {
    /// The write of failed.
    #[from]
    WriteError(WriteError<E>),

    /// The read failed.
    #[from(ReadError<E>, InvalidMessage, InvalidPacketId, MotorError)]
    ReadError(ReadError<E>),
}

/// An error that can occur during a write transfer.
#[derive(Debug, Display, Error, From)]
pub enum WriteError<E> {
    /// The write buffer is too small to contain the whole stuffed message.
    BufferTooSmall(BufferTooSmallError),

    /// Failed to discard the input buffer before writing the instruction.
    #[from(skip)]
    DiscardBuffer(E),

    /// Failed to write the instruction.
    #[from(skip)]
    Write(E),
}

/// The buffer is too small to hold the entire message.
///
/// Consider increasing the size of the buffer.
/// Keep in mind that the write buffer needs to be large enough to account for byte stuffing.
#[derive(Debug, Display, Error)]
#[display("buffer is too small: need {} bytes, but the size is {}", self.required_size, self.total_size)]
pub struct BufferTooSmallError {
    /// The required size of the buffer.
    pub required_size: usize,

    /// The total size of the buffer.
    pub total_size: usize,
}

/// An error that can occur during a read transfer.
#[derive(Debug, Display, Error, From)]
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

    /// The motor reported an error instead of a valid response.
    ///
    /// This error is not returned when a motor signals a hardware error,
    /// since the instruction has still been exectuted.
    ///
    /// Instead, the `alert` bit in the response will be set.
    #[from]
    MotorError(MotorError),
}

/// The received message is not valid.
#[derive(Debug, Clone, Eq, PartialEq, Display, Error, From)]
pub enum InvalidMessage {
    /// The message checksum is invalid.
    InvalidChecksum(InvalidChecksum),

    /// The message has an invalid packet ID.
    InvalidPacketId(InvalidPacketId),

    /// The message has an invalid parameter count.
    InvalidParameterCount(InvalidParameterCount),
}

/// An error reported by the motor.
#[derive(Debug, Clone, Eq, PartialEq, Display, Error)]
pub struct MotorError {
    /// The raw error as returned by the motor.
    pub flags: ErrorFlags,
}

/// The received message has an invalid checksum value.
#[derive(Debug, Clone, Eq, PartialEq, Display, Error)]
#[display("invalid checksum, message claims {:#02X}, computed {:#02X}", self.message, self.computed)]
pub struct InvalidChecksum {
    /// The checksum from the messsage.
    pub message: u8,

    /// The actual checksum.
    pub computed: u8,
}

/// The received message has an invalid or unexpected packet ID.
#[derive(Debug, Clone, Eq, PartialEq, Error)]
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

impl MotorError {
    /// Check for a motor error in the response.
    ///
    /// This ignores the `alert` bit,
    /// since it indicates a hardware error and not a failed instruction.
    pub fn check(flags: Option<ErrorFlags>) -> Result<(), Self> {
        // Ignore the alert bit for this check.
        // If the alert bit is set, the motor encountered an error, but the instruction was still executed.
        let Some(flags) = flags else { return Ok(()) };

        if flags.intersects(ERROR_FLAGS) {
            Err(Self { flags })
        } else {
            Ok(())
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
