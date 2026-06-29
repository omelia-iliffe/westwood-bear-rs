# ww-bear

A Rust driver for [Westwood Robotics](https://www.westwoodrobotics.io/) BEAR actuators.

Provides both synchronous and asynchronous interfaces for communicating with BEAR motors over a serial bus.

## Usage

```rust
use ww_bear::Bus;

let mut bus = Bus::open("/dev/ttyUSB0", 8_000_000)?;

bus.ping(1)?;
bus.write_mode(1, 2)?;
bus.write_torque_enable(1, 1)?;
bus.write_goal_pos(1, 1.57)?;

let pos = bus.read_present_pos(1)?.data;
println!("present position: {pos:.4} rad");
```

### Async

An async interface is available under `ww_bear::asynchronous::Bus` with the same API:

```rust
use ww_bear::asynchronous::Bus;

let mut bus = Bus::open("/dev/ttyUSB0", 8_000_000)?;
let pos = bus.read_present_pos(1).await?.data;
```

### Bulk

Read and/or write the same status registers across several motors in a single packet. Each
read reply borrows the bus' shared buffer, so replies are handed to a callback:

```rust
use ww_bear::{BulkWriteData, Bus, StatusRegister};

let mut bus = Bus::open("/dev/ttyUSB0", 8_000_000)?;
let ids = [1, 2, 3];

// Bulk write: same goal position on every motor. `from_f32` pairs a motor id with the
// little-endian bytes for one f32 register.
let devices = ids.iter().map(|&motor_id| BulkWriteData::from_f32(motor_id, 1.57));
bus.bulk_write(devices, &[StatusRegister::GoalPos])?;

// Bulk read: present position from every motor. Each reply is a `Result`, so one
// motor failing to respond doesn't abort reading the others.
bus.bulk_read(&ids, &[StatusRegister::PresentPos], |response| match response {
    Ok(response) => {
        let pos = response.f32(0).unwrap();
        println!("motor {}: {pos:.4} rad", response.motor_id);
    }
    Err(e) => eprintln!("reply failed to read: {e}"),
})?;
```

Bulk is only available for status registers. With the `alloc` feature, `bulk_read_alloc`
returns the replies as owned `Vec`s instead of using a callback.

## Supported instructions

| Instruction       | Supported |
|-------------------|-----------|
| Ping              | ✓         |
| Read              | ✓         |
| Write             | ✓         |
| SaveConfig        | ✓         |
| SetAbsolutePos    | ✓         |
| BulkComm          | ✓         |

## Features

| Feature   | Default | Description |
|-----------|---------|-------------|
| `std`     | yes     | Enables `std` support. Disable for `no_std` use. |
| `alloc`   | yes     | Enables heap allocation (implied by `std`). |
| `serial2` | yes     | Enables `Bus::open()` via the `serial2`/`serial2-tokio` crates. |
| `defmt`   | no      | Enables `defmt` logging and derives for embedded targets. |

### `no_std`

This crate is `no_std` compatible. Disable default features and provide your own `SerialPort` implementation:

```toml
[dependencies]
ww-bear = { version = "...", default-features = false }
```

## Examples

| Example              | Description |
|----------------------|-------------|
| `ping`               | Scan one or more ports/baud rates for motors and print their positions. |
| `set_position`       | Set a motor's goal position. |
| `set_position_async` | Async version of `set_position`. |
| `set_abs_position`   | Reset a motor's absolute position (requires backup battery). |
| `setup_motor`        | Interactive wizard to configure a motor's gains, limits, and ID. |
| `bulk`               | Bulk write a goal position to several motors, then bulk read their state. |
| `bulk_async`         | Async version of `bulk`. |

```sh
cargo run --example ping -- --port /dev/ttyUSB0 --baud 8000000
cargo run --example set_position -- --id 1 /dev/ttyUSB0 1.57
```

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT) at your option.

The structure and layout of this crate is heavily influenced by the [dynamixel2-rs](https://github.com/robohouse-delft/dynamixel2-rs) crate, licensed under the BSD 2-Clause license.
