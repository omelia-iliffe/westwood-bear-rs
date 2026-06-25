use clap::Parser;
use std::thread;
use std::time::Duration;
use ww_bear::{BulkWriteData, Bus, StatusRegister};

#[derive(Parser)]
struct Args {
    /// Serial port (e.g. /dev/ttyUSB0)
    port: String,
    /// Motor IDs, comma separated (e.g. 1,2,3)
    #[arg(short, long, value_delimiter = ',')]
    ids: Vec<u8>,
    /// Goal position in radians, written to every motor
    position: f32,
    /// Baud rate
    #[arg(short, long, default_value_t = 8_000_000)]
    baud: u32,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();
    let mut bus = Bus::open(&args.port, args.baud)?;

    // The default response timeout is very tight (~1 ms). USB-serial adapters add
    // variable latency, so give replies more headroom to avoid spurious timeouts.
    bus.set_return_time_delay(Duration::from_millis(10));

    // Mode is a config register, so it can't be set in bulk. Enable position mode
    // and torque on each motor individually first.
    for &id in &args.ids {
        bus.ping(id)?;
        bus.write_mode(id, 2)?;
        bus.write_torque_enable(id, 1)?;
    }
    println!("Connected to motors {:?}", args.ids);

    // Bulk write: set the same goal position on every motor in a single packet.
    // `from_f32` pairs a motor id with the little-endian bytes for one f32 register.
    let devices = args
        .ids
        .iter()
        .map(|&motor_id| BulkWriteData::from_f32(motor_id, args.position));
    bus.bulk_write(devices, &[StatusRegister::GoalPos])?;
    println!("Goal position set to {:.4} rad on all motors", args.position);

    thread::sleep(Duration::from_secs(2));

    // Bulk read: read present position and velocity from every motor in a single packet.
    // Each reply's `data` is the concatenated read registers (4 bytes each), in request order.
    bus.bulk_read(
        &args.ids,
        &[StatusRegister::PresentPos, StatusRegister::PresentVel],
        |response| match response {
            Ok(response) => {
                let pos = response.f32(0).unwrap();
                let vel = response.f32(1).unwrap();
                println!(
                    "motor {:>3}: pos {pos:.4} rad, vel {vel:.4} rad/s (warning: {:?})",
                    response.motor_id, response.warning,
                );
            },
            Err(e) => eprintln!("reply failed to read: {e}"),
        },
    )?;

    Ok(())
}
