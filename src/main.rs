use clap::{Args, Parser, Subcommand};
use core::time::Duration;
use env_logger;
use num_enum::TryFromPrimitive;
use serialport::{ClearBuffer, SerialPort, SerialPortType};
use std::fmt::Debug;
use std::io::{Error, ErrorKind, Result, Write};
use std::str;
use trace::trace;

#[derive(Parser)]
#[command(author, version, about, long_about = None)]
struct Parameters {
    #[clap(flatten)]
    verbose: clap_verbosity_flag::Verbosity,
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// List available serial devices
    List {
        /// List USB serial devices (default)
        #[arg(short, long)]
        usb: bool,
        /// List PCI serial devices
        #[arg(short, long)]
        pci: bool,
        /// List Bluetooth serial devices
        #[arg(short, long)]
        bluetooth: bool,
        /// List other serial devices
        #[arg(short, long)]
        other: bool,
        /// List all serial devices
        #[arg(short, long)]
        any: bool,
    },
    /// Request device state
    State {
        #[clap(flatten)]
        device: Device,
    },
    /// Request device information
    Info {
        #[clap(flatten)]
        device: Device,
    },
    /// List WiFi networks scanned by device
    Scan {
        #[clap(flatten)]
        device: Device,
    },
    /// Set WiFi credentials on device
    Provision {
        #[clap(flatten)]
        device: Device,
        /// WiFi network SSID
        ssid: String,
        /// WiFi network password
        password: String,
    },
}

#[derive(Args)]
struct Device {
    /// Device path
    path: String,
    /// Timeout seconds (default: 60)
    #[arg(short, long)]
    timeout: Option<u64>,
}

fn main() -> Result<()> {
    let parameters = Parameters::parse();
    env_logger::Builder::new()
        .filter_level(parameters.verbose.log_level_filter())
        .init();
    match &parameters.command {
        Command::List {
            usb,
            pci,
            bluetooth,
            other,
            any,
        } => ImprovSerial::list(*usb, *pci, *bluetooth, *other, *any),
        Command::State { device } => ImprovSerial::new(&device.path, &device.timeout)?.state(),
        Command::Info { device } => ImprovSerial::new(&device.path, &device.timeout)?.info(),
        Command::Scan { device } => ImprovSerial::new(&device.path, &device.timeout)?.scan(),
        Command::Provision {
            device,
            ssid,
            password,
        } => ImprovSerial::new(&device.path, &device.timeout)?.provision(ssid, password),
    }
}

struct ImprovSerial {
    port: Box<dyn SerialPort>,
}

impl Debug for ImprovSerial {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ImprovSerial")
            .field("name", &self.port.name())
            .finish()
    }
}

// TODO investigate using a module to avoid having to trace::init_depth_var!()
// https://docs.rs/trace/0.1.6/trace/
trace::init_depth_var!();

#[trace(logging)]
impl ImprovSerial {
    fn list(usb: bool, pci: bool, bluetooth: bool, other: bool, any: bool) -> Result<()> {
        let default = !usb && !pci && !bluetooth && !other;
        let ports = serialport::available_ports()?;
        for port in &ports {
            log::debug!("{:?}", port);
            match port.port_type {
                SerialPortType::UsbPort(_) if usb || any || default => {}
                SerialPortType::PciPort if pci || any => {}
                SerialPortType::BluetoothPort if bluetooth || any => {}
                SerialPortType::Unknown if other || any => {}
                _ if other || any => {}
                _ => continue,
            }
            println!("{}", port.port_name);
        }
        return Ok(());
    }

    fn new(path: &str, timeout: &Option<u64>) -> Result<Self> {
        // default timeout is 0 which means not blocking
        // https://github.com/serialport/serialport-rs/issues/12
        // use 60s as default timeout
        let builder =
            serialport::new(path, 115200).timeout(Duration::from_secs(timeout.unwrap_or(60)));
        log::debug!("{:?}", &builder);
        let port = builder.open()?;
        port.clear(ClearBuffer::All)?;
        return Ok(Self { port });
    }

    fn state(&mut self) -> Result<()> {
        self.send_rpc_command(ImprovSerialRpcCommand::RequestCurrentState, vec![])?;
        let state = self.receive_current_state()?;
        println!("{:#?}", state);
        if let ImprovSerialCurrentState::Provisioned = state {
            let values = self.receive_rpc_response(ImprovSerialRpcCommand::RequestCurrentState)?;
            println!("{}", values[0]);
        }
        return Ok(());
    }

    fn info(&mut self) -> Result<()> {
        self.send_rpc_command(ImprovSerialRpcCommand::RequestDeviceInformation, vec![])?;
        let values = self.receive_rpc_response(ImprovSerialRpcCommand::RequestDeviceInformation)?;
        println!("firmware   : {}", values[0]);
        println!("version    : {}", values[1]);
        println!("name       : {}", values[3]);
        println!("chipFamily : {}", values[2]);
        return Ok(());
    }

    fn scan(&mut self) -> Result<()> {
        self.send_rpc_command(ImprovSerialRpcCommand::RequestScannedWifiNetworks, vec![])?;
        loop {
            let values =
                self.receive_rpc_response(ImprovSerialRpcCommand::RequestScannedWifiNetworks)?;
            if values.len() == 0 {
                return Ok(());
            }
            println!(
                "SSID: {}, RSSI: {}, Auth required: {}",
                values[0], values[1], values[2]
            );
        }
    }

    fn provision(&mut self, ssid: &str, password: &str) -> Result<()> {
        let data = vec![ssid, password];
        self.send_rpc_command(ImprovSerialRpcCommand::SendWifiSettings, data)?;
        let values = self.receive_rpc_response(ImprovSerialRpcCommand::SendWifiSettings)?;
        println!("{}", values[0]);
        return Ok(());
    }

    fn send_rpc_command(&mut self, command: ImprovSerialRpcCommand, data: Vec<&str>) -> Result<()> {
        let mut data = ImprovSerial::rpc_command_data(data);
        let data_len: u8 = data.len().try_into().map_err(|_| ErrorKind::InvalidInput)?;
        data.insert(0, command as u8);
        data.insert(1, data_len);
        self.write_packet_to_stream(ImprovSerialPacketType::RpcCommand, data)?;
        return Ok(());
    }

    fn rpc_command_data(data: Vec<&str>) -> Vec<u8> {
        let mut command_data: Vec<u8> = vec![];
        for s in data {
            let l: u8 = s.len().try_into().unwrap();
            command_data.push(l);
            command_data.append(&mut s.as_bytes().to_owned());
        }
        return command_data;
    }

    fn write_packet_to_stream(
        &mut self,
        packet_type: ImprovSerialPacketType,
        data: Vec<u8>,
    ) -> Result<()> {
        let payload = ImprovSerial::payload(packet_type as u8, data);
        let buffer = &payload[..];
        log::debug!("{:02X?}", buffer);
        log::debug!("{:?}", ImprovSerial::decode_packet(buffer.to_vec()));
        self.port.write_all(buffer)?;
        return Ok(());
    }

    fn payload(msg_type: u8, data: Vec<u8>) -> Vec<u8> {
        let mut payload: Vec<u8> = vec![];
        for v in ImprovSerial::PACKET_HEADER.as_bytes() {
            payload.push(*v);
        }
        payload.push(ImprovSerial::PROTOCOL_VERSION);
        payload.push(msg_type);
        payload.push(data.len().try_into().unwrap());
        for v in &data {
            payload.push(*v);
        }
        let mut sum: u8 = 0;
        for v in &payload {
            sum = sum.wrapping_add(*v);
        }
        payload.push(sum);
        payload.push(b'\n');
        return payload;
    }

    fn receive_current_state(&mut self) -> Result<ImprovSerialCurrentState> {
        loop {
            let packet = self.read_incoming_packet()?;
            match packet {
                ImprovSerialPacket::ErrorState { state } => match state {
                    ImprovSerialErrorState::NoError => {}
                    _ => {
                        return Err(Error::new(ErrorKind::InvalidData, "state"));
                    }
                },
                ImprovSerialPacket::CurrentState { state } => {
                    return Ok(state);
                }
                ImprovSerialPacket::RpcCommand { command, values } => panic!(
                    "received RPC command {:#?} with values {:#?}",
                    command, values
                ),
                ImprovSerialPacket::RpcResult { command, values } => panic!(
                    "received RPC result for RPC command {:#?} with values {:#?}",
                    command, values
                ),
            }
        }
    }

    fn receive_rpc_response(
        &mut self,
        expected_command: ImprovSerialRpcCommand,
    ) -> Result<Vec<String>> {
        loop {
            let packet = self.read_incoming_packet()?;
            match packet {
                ImprovSerialPacket::ErrorState { state } => match state {
                    ImprovSerialErrorState::NoError => {}
                    _ => {
                        return Err(Error::new(ErrorKind::InvalidData, "state"));
                    }
                },
                ImprovSerialPacket::CurrentState { state } => {
                    println!("{:#?}", state);
                }
                ImprovSerialPacket::RpcCommand { command, values } => panic!(
                    "received RPC command {:#?} with values {:#?}",
                    command, values
                ),

                ImprovSerialPacket::RpcResult { command, values } => {
                    if command as u8 == expected_command as u8 {
                        return Ok(values);
                    } else {
                        println!(
                            "received RPC result for command {:#?} with values {:#?}",
                            command, values
                        );
                        return Err(Error::new(ErrorKind::InvalidData, "command"));
                    }
                }
            }
        }
    }

    const PACKET_HEADER: &str = "IMPROV";
    const PROTOCOL_VERSION: u8 = 1;

    fn read_incoming_packet(&mut self) -> Result<ImprovSerialPacket> {
        let mut line = vec![];
        loop {
            self.read_until(b'\n', &mut line)?;
            log::debug!("{:02X?}", &line);
            if line.len() >= 7 {
                // "NotUtf8" is longer than "IMPROV" and is so always different
                let header = str::from_utf8(&line[0..6]).unwrap_or("NotUtf8");
                let version = line[6];
                log::debug!("header: {}, version: {}", header, version);
                if ImprovSerial::PACKET_HEADER.eq(header)
                    && ImprovSerial::PROTOCOL_VERSION == version
                {
                    break;
                }
            }
            let array = &line[..];
            // TODO explorer mapping source level to output level
            std::io::stdout().write_all(&array)?;
            line.clear();
        }
        let packet = ImprovSerial::decode_packet(line)?;
        return Ok(packet);
    }

    fn read_until(&mut self, eol: u8, buf: &mut Vec<u8>) -> Result<usize> {
        let mut buffer = [0];
        let mut count: usize = 0;
        loop {
            match self.port.read(&mut buffer) {
                Ok(s) => {
                    if s > 0 {
                        buf.push(buffer[0]);
                        count = count + 1;
                        if eol == buffer[0] {
                            return Ok(count);
                        }
                    }
                }
                Err(error) => {
                    return Err(error);
                }
            }
        }
    }

    fn decode_packet(line: Vec<u8>) -> Result<ImprovSerialPacket> {
        let payload = &line[6..];
        let version = payload[0];
        let packet_type = payload[1];
        let packet_length = payload[2];
        let data = &payload[3..3 + packet_length as usize];
        if version != ImprovSerial::PROTOCOL_VERSION {
            eprintln!("Received unsupported version {}", version);
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                "Unsupported version",
            ));
        }
        let packet_checksum = payload[3 + packet_length as usize];
        let mut calculated_checksum: u8 = 0;
        for v in &line[..line.len() - 2] {
            calculated_checksum = calculated_checksum.wrapping_add(*v);
        }
        if calculated_checksum != packet_checksum {
            println!(
                "Received invalid checksum {}. Expected {}",
                packet_checksum, calculated_checksum
            );
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                "Invalid checksum",
            ));
        }
        match ImprovSerialPacketType::try_from(packet_type).unwrap() {
            ImprovSerialPacketType::ErrorState => {
                let error_state = ImprovSerialErrorState::try_from(data[0]).unwrap();
                return Ok(ImprovSerialPacket::ErrorState { state: error_state });
            }
            ImprovSerialPacketType::CurrentState => {
                let current_state = ImprovSerialCurrentState::try_from(data[0]).unwrap();
                return Ok(ImprovSerialPacket::CurrentState {
                    state: current_state,
                });
            }
            ImprovSerialPacketType::RpcCommand => {
                let rpc_command = data[0];
                let mut result: Vec<String> = vec![];
                let total_length: usize = data[1] as usize;
                let mut idx: usize = 2;
                while idx < 2 + total_length {
                    result.push(
                        String::from_utf8(data[idx + 1..idx + data[idx] as usize + 1].to_vec())
                            .unwrap(),
                    );
                    idx += data[idx] as usize + 1;
                }
                let rpc_command = ImprovSerialRpcCommand::try_from(rpc_command).unwrap();
                return Ok(ImprovSerialPacket::RpcCommand {
                    command: rpc_command,
                    values: result,
                });
            }
            ImprovSerialPacketType::RpcResult => {
                let rpc_command = data[0];
                let mut result: Vec<String> = vec![];
                let total_length: usize = data[1] as usize;
                let mut idx: usize = 2;
                while idx < 2 + total_length {
                    result.push(
                        String::from_utf8(data[idx + 1..idx + data[idx] as usize + 1].to_vec())
                            .unwrap(),
                    );
                    idx += data[idx] as usize + 1;
                }
                let rpc_command = ImprovSerialRpcCommand::try_from(rpc_command).unwrap();
                return Ok(ImprovSerialPacket::RpcResult {
                    command: rpc_command,
                    values: result,
                });
            }
        }
    }
}

#[repr(u8)]
#[derive(TryFromPrimitive, Debug)]
enum ImprovSerialPacketType {
    CurrentState = 0x01,
    ErrorState = 0x02,
    RpcCommand = 0x03,
    RpcResult = 0x04,
}

#[derive(Debug)]
#[repr(u8)]
#[derive(TryFromPrimitive)]
enum ImprovSerialCurrentState {
    Ready = 0x02,
    Provisioning = 0x03,
    Provisioned = 0x04,
}

#[derive(Debug)]
#[repr(u8)]
#[derive(TryFromPrimitive)]
enum ImprovSerialErrorState {
    NoError = 0x00,
    InvalidRpcPacket = 0x01,
    UnknownRpcCommand = 0x02,
    UnableToConnect = 0x03,
    UnknownError = 0xFF,
}

#[derive(Debug)]
#[repr(u8)]
#[derive(TryFromPrimitive, Clone, Copy)]
enum ImprovSerialRpcCommand {
    SendWifiSettings = 0x01,
    RequestCurrentState = 0x02,
    RequestDeviceInformation = 0x03,
    RequestScannedWifiNetworks = 0x04,
}

#[derive(Debug)]
enum ImprovSerialPacket {
    CurrentState {
        state: ImprovSerialCurrentState,
    },
    ErrorState {
        state: ImprovSerialErrorState,
    },
    RpcCommand {
        command: ImprovSerialRpcCommand,
        values: Vec<String>,
    },
    RpcResult {
        command: ImprovSerialRpcCommand,
        values: Vec<String>,
    },
}
