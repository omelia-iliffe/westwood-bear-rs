use clap::Parser;
use std::thread;
use std::time::Duration;
use ww_bear::{Bus, StatusRegister};

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
    // `write_data` has one entry per motor, each holding 4 encoded bytes per write register.
    let goal_bytes: Vec<[u8; 4]> = args.ids.iter().map(|_| args.position.to_le_bytes()).collect();
    let write_data: Vec<&[u8]> = goal_bytes.iter().map(|b| b.as_slice()).collect();
    bus.bulk_write(&args.ids, &[StatusRegister::GoalPos], &write_data)?;
    println!("Goal position set to {:.4} rad on all motors", args.position);

    thread::sleep(Duration::from_secs(2));

    // Bulk read: read present position and velocity from every motor in a single packet.
    // Each reply's `data` is the concatenated read registers (4 bytes each), in request order.
    bus.bulk_read(
        &args.ids,
        &[StatusRegister::PresentPos, StatusRegister::PresentVel],
        |response| {
            let pos = f32::from_le_bytes(response.data[0..4].try_into().unwrap());
            let vel = f32::from_le_bytes(response.data[4..8].try_into().unwrap());
            println!(
                "motor {:>3}: pos {pos:.4} rad, vel {vel:.4} rad/s (warning: {:?})",
                response.motor_id, response.warning,
            );
        },
    )?;

    Ok(())
}
