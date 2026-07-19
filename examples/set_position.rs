use clap::Parser;
use std::thread;
use std::time::Duration;
use ww_bear::Bus;

#[derive(Parser)]
struct Args {
    /// Serial port (e.g. /dev/ttyUSB0)
    port: String,
    /// Motor ID
    #[arg(short, long)]
    id: u8,
    /// Goal position in radians
    position: f32,
    /// Baud rate
    #[arg(short, long, default_value_t = 8_000_000)]
    baud: u32,
    /// Return time delay in milliseconds
    #[arg(short, long, default_value_t = 20)]
    return_time_delay: u64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();
    let mut bus = Bus::open(&args.port, args.baud)?;
    bus.set_return_time_delay(Duration::from_millis(args.return_time_delay));

    bus.ping(args.id)?;
    println!("Connected to motor {}", args.id);

    bus.write_mode(args.id, 2)?;
    bus.write_torque_enable(args.id, 1)?;
    bus.write_goal_pos(args.id, args.position)?;
    println!("Goal position set to {:.4} rad", args.position);

    thread::sleep(Duration::from_secs(2));

    let present = bus.read_present_pos(args.id)?.data;
    println!("Present position:   {present:.4} rad");

    Ok(())
}
