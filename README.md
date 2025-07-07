# westwood robotics bear protocol

An implementation of the protocol used to comunicate with the WestWood Robotics Bear Actuators.

Supported instructions are `Read`, `Write` and `SaveConfig`. `BulkReadWrite` is currently not supported.

## Features
- `std`
- `alloc`
- `serial2`:  
  enables support for the `serial2` crate and adds helper methods for opening a port.

### `no_std`
This crate is `no_std` compatible. Disable the default features to exclude `std`.

## License
Licensed under either of <a href="LICENSE-APACHE">Apache License, Version
2.0</a> or <a href="LICENSE-MIT">MIT license</a> at your option.

The structure and layout of this crate is heavily influenced by [robohouse-delft/dynamixel2-rs](https://github.com/robohouse-delft/dynamixel2-rs) crate, which is licensed under the BSD 2-Clause license.

