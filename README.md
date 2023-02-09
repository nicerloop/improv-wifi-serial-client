# Improv WiFi serial client

This is a [Rust](https://www.rust-lang.org) command-line client for [Improv Wifi](https://www.improv-wifi.com/) [serial](https://www.improv-wifi.com/serial/) protocol.

Build for and tested with [ESPHome](https://esphome.io) [Improv via Serial](https://esphome.io/components/improv_serial.html) component.

## Dependencies

- [Rust](https://www.rust-lang.org)
- [serialport](https://crates.io/crates/serialport)
- [num_enum](https://crates.io/crates/num_enum)
- [clap](https://crates.io/crates/clap)
    -[clap-verbosity-flag](https://crates.io/crates/clap-verbosity-flag)
- [log](https://crates.io/crates/log)
    - [env_logger](https://crates.io/crates/env_logger)
    - [trace](https://crates.io/crates/trace)

## Built

```bash
cargo build
```

## Run

```bash
cargo run --
```

### Sub-commands
- list
- info \<device>
- state \<device>
- scan \<device>
- provision \<device> \<SSID> \<password>

## Install

### Cargo

```bash
cargo install
```

### Homebrew (macOS)

```bash
brew install nicerloop/nicerloop/improv-wifi-serial-client
```
