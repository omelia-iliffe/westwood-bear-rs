//! An implementation of the protocol used by Westwood Robotics Bear actuators
//!
//! This crate aims to provide an easy to use implementation of the procotol used to communicate with the Bear Motors.
//!
//! The main interface of the library is the [`Bus`], which is used to interact with devices on the serial communication bus.
//! The [`Bus`] struct exposes methods for communicating, such as [`Bus::read`], [`Bus::write`], [`Bus::save_config`] and [`Bus::ping`].
//! Additionally, helper functions exist for reading/write a specific register such as [`Bus::write_torque_enable`] and [`Bus::read_present_pos`].
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
