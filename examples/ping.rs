use clap::Parser;
use ww_bear::Bus;

#[derive(Parser)]
struct Args {
    /// Serial ports to scan, comma-separated (e.g. /dev/ttyUSB0,/dev/ttyUSB1)
    #[arg(short, long, value_delimiter = ',', required = true)]
    port: Vec<String>,
    /// Motor IDs to scan, comma-separated. If omitted, scans all IDs (1–253).
    #[arg(short, long, value_delimiter = ',')]
    ids: Vec<u8>,
    /// Baud rates to try, comma-separated. Defaults to 8000000.
    #[arg(short, long, value_delimiter = ',', default_values_t = vec![8_000_000u32])]
    baud: Vec<u32>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();

    let ids: Vec<u8> = if args.ids.is_empty() {
        (1..=253).collect()
    } else {
        args.ids
    };

    for port in &args.port {
        for baud in &args.baud {
            println!("Scanning {} at {} baud...", port, baud);
            let mut bus = match Bus::open(port, *baud) {
                Err(e) => {
                    log::error!("erroring opening port {port}, {e:?}");
                    continue;
                },
                Ok(bus) => bus,
            };
            for id in &ids {
                match bus.ping(*id) {
                    Err(e) => log::debug!("  ID {id}: {e}"),
                    Ok(_) => match bus.read_present_pos(*id) {
                        Ok(r) => {
                            if r.warning.is_empty() {
                                println!("  ID {id}: {:.4} rad", r.data);
                            } else {
                                println!("  ID {id}: {:.4} rad  [warnings: {}]", r.data, r.warning);
                            }
                        },
                        Err(e) => println!("  ID {id}: found (position read failed: {e})"),
                    },
                }
            }
        }
    }

    Ok(())
}
