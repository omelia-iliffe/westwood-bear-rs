use clap::Parser;
use std::time::Duration;
use ww_bear::Bus;

#[derive(Parser)]
struct Args {
    /// Serial port (e.g. /dev/ttyUSB0)
    port: String,
    /// Motor ID
    #[arg(short, long)]
    id: u8,
    /// Target position in radians
    position: f32,
    /// Tolerance in radians (0 = adjust homing offset; non-zero = find nearest multi-turn match)
    #[arg(short, long, default_value_t = 0.0)]
    tolerance: f32,
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

    bus.set_absolute_position(args.id, args.position, args.tolerance)?;
    println!("Absolute position set to {:.4} rad (tolerance {:.4} rad)", args.position, args.tolerance);

    let pos = bus.read_present_pos(args.id)?;
    println!("Present position: {:.4} rad", pos.data);

    Ok(())
}
