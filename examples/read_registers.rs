use std::time::Duration;

use clap::Parser;
use strum::IntoEnumIterator;
use ww_bear::{Bus, ConfigRegister, StatusRegister};

#[derive(Parser)]
struct Args {
    /// Serial port (e.g. /dev/ttyUSB0)
    port: String,
    /// Motor ID
    #[arg(short, long)]
    id: u8,
    /// Baud rate
    #[arg(short, long, default_value_t = 8_000_000)]
    baud: u32,
}

/// Formats a register's 4 bytes as the type its table entry holds.
fn format_value(bytes: &[u8], is_u32: bool) -> String {
    match <[u8; 4]>::try_from(bytes) {
        Ok(b) if is_u32 => u32::from_le_bytes(b).to_string(),
        Ok(b) => f32::from_le_bytes(b).to_string(),
        Err(_) => format!("{bytes:02x?}"),
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();
    let mut bus = Bus::open(&args.port, args.baud)?;

    // The default response timeout is very tight (~1 ms). USB-serial adapters add
    // variable latency, so give replies more headroom to avoid spurious timeouts.
    bus.set_response_timeout_padding(Duration::from_millis(10));

    bus.ping(args.id)?;

    println!("Config table, motor {}:", args.id);
    for register in ConfigRegister::iter() {
        let is_u32 = matches!(
            register,
            ConfigRegister::Id
                | ConfigRegister::Mode
                | ConfigRegister::BaudRate
                | ConfigRegister::WatchdogTimeout
                | ConfigRegister::ReturnTimeDelay
        );
        match bus.read_config(args.id, register) {
            Ok(r) => println!(
                "  {:#04x} {:<16} {}",
                register as u8,
                register.to_string(),
                format_value(r.data, is_u32)
            ),
            Err(e) => println!(
                "  {:#04x} {:<16} read failed: {e}",
                register as u8,
                register.to_string()
            ),
        }
    }

    println!("Status table, motor {}:", args.id);
    for register in StatusRegister::iter() {
        let is_u32 = register == StatusRegister::TorqueEnable;
        match bus.read_status(args.id, register) {
            Ok(r) => println!(
                "  {:#04x} {:<16} {}",
                register as u8,
                register.to_string(),
                format_value(r.data, is_u32)
            ),
            Err(e) => println!(
                "  {:#04x} {:<16} read failed: {e}",
                register as u8,
                register.to_string()
            ),
        }
    }

    Ok(())
}
