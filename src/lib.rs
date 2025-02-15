#![warn(missing_docs)]
#![warn(missing_debug_implementations)]
#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "serial2")]
/// Public re-export of the serial2 crate.
pub use serial2;
mod bus;
pub use bus::Bus;
mod checksum;
pub mod error;
mod instructions;
mod protocol;
pub use protocol::*;
mod serial_port;

pub use serial_port::SerialPort;
