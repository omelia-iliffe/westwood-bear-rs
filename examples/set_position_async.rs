use clap::Parser;
use tokio::time::{Duration, sleep};
use ww_bear::asynchronous::Bus;

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
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();
    let mut bus = Bus::open(&args.port, args.baud)?;

    bus.ping(args.id).await?;
    println!("Connected to motor {}", args.id);

    bus.write_mode(args.id, 2).await?;
    bus.write_torque_enable(args.id, 1).await?;
    bus.write_goal_pos(args.id, args.position).await?;
    println!("Goal position set to {:.4} rad", args.position);

    sleep(Duration::from_secs(2)).await;

    let present = bus.read_present_pos(args.id).await?.data;
    println!("Present position:   {present:.4} rad");

    Ok(())
}
