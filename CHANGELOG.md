# WW BEAR ChangeLog

## [0.3.0] - 2026-08-05
### Features
- Add bulk read/write support
- Add f32/u32 encode/decode helpers and rename bulk_comm
- [**breaking**] Decouple tokio from the serial2 backend (#3)
- **(examples)** Add return-time-delay arg, remove setup-motor example
- Validate bulk read replies by id and length
### Bug Fixes
- Remove dbg! usage, update readme, bumb version
- Reject bulk requests with too many registers; correct feature docs
- [**breaking**] Remove unused `ErrorStatus` & `WarningStatus` registers
### Refactor
- Pair motor ids with data instead of passing two parallel slices, deliver each bulk reply as a Result, add async bulk example
- [**breaking**] Rename return_time_delay to response_timeout_padding
### Documentation
- Remove setup_motor from examples table
### Styling
- Pad packet ID in error messages to 0x0E form

## [0.2.0] - 2026-05-25
### Features
- Add `set_abs_position`
- Add/improve examples
- Add experimental `return_time_delay` register
- Add async support behind the `bisync` crate
- Add `ErrorFlag` and catch motor errors
- Add bus constructors
### Bug Fixes
- [**breaking**] Mark `Instruction`, `ConfigRegister`, and `StatusRegister` as `non_exhaustive`
- [**breaking**] `ErrorFlags` never populated
- Fix message timeout
- Fix `no_std` build

## [0.1.0]
### Features
- Initial release: working read/write/ping
