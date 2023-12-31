use chrono::prelude::*;
use clap::{App, Arg};
use std::{convert::TryInto, time::Duration};
use ublox::*;

const BAUD_RATES: [u32; 7] = [9600, 19200, 38400, 57600, 115200, 230400, 460800];
const NUM_TRIES: usize = 3;
struct Device {
    port: Box<dyn serialport::SerialPort>,
    parser: Parser<Vec<u8>>,
}

impl Device {
    pub fn new(port: Box<dyn serialport::SerialPort>) -> Device {
        let parser = Parser::default();
        Device { port, parser }
    }

    pub fn write_all(&mut self, data: &[u8]) -> std::io::Result<()> {
        self.port.write_all(data)
    }

    pub fn update<T: FnMut(PacketRef)>(&mut self, mut cb: T) -> std::io::Result<()> {
        loop {
            let mut local_buf = [0; 100];
            let nbytes = self.read_port(&mut local_buf)?;
            if nbytes == 0 {
                break;
            }

            // parser.consume adds the buffer to its internal buffer, and
            // returns an iterator-like object we can use to process the packets
            let mut it = self.parser.consume(&local_buf[..nbytes]);
            loop {
                match it.next() {
                    Some(Ok(packet)) => {
                        cb(packet);
                    }
                    Some(Err(_)) => {
                        // Received a malformed packet, ignore it
                    }
                    None => {
                        // We've eaten all the packets we have
                        break;
                    }
                }
            }
        }
        Ok(())
    }

    pub fn wait_for_ack<T: UbxPacketMeta>(&mut self) -> std::io::Result<()> {
        let mut found_packet = false;
        while !found_packet {
            self.update(|packet| {
                if let PacketRef::AckAck(ack) = packet {
                    if ack.class() == T::CLASS && ack.msg_id() == T::ID {
                        found_packet = true;
                    }
                }
            })?;
        }
        Ok(())
    }

    /// Reads the serial port, converting timeouts into "no data received"
    fn read_port(&mut self, output: &mut [u8]) -> std::io::Result<usize> {
        match self.port.read(output) {
            Ok(b) => Ok(b),
            Err(e) => {
                if e.kind() == std::io::ErrorKind::TimedOut {
                    Ok(0)
                } else {
                    Err(e)
                }
            }
        }
    }
}

fn try_baud_rate(port_name: &str, baud_rate: u32) -> Result<usize, Box<dyn std::error::Error>> {
    let settings = serialport::SerialPortSettings {
        baud_rate,
        data_bits: serialport::DataBits::Eight,
        flow_control: serialport::FlowControl::None,
        parity: serialport::Parity::None,
        stop_bits: serialport::StopBits::One,
        timeout: std::time::Duration::from_millis(10000),
    };

    let mut max_byte_read = 0;
    for _ in 0..NUM_TRIES {
        match serialport::open_with_settings(port_name, &settings) {
            Ok(mut port) => {
                let mut buffer = vec![0u8; 128];
                let byte_read = port.read(&mut buffer)?;
                if byte_read > max_byte_read {
                    max_byte_read = byte_read;
                }
            }
            Err(err) => {
                eprintln!("Error opening serial port: {}", err);
                std::process::exit(10);
            }
        }
    }

    Ok(max_byte_read)
}

fn get_gps_data(device: &mut Device, monitor: bool, status: bool) {
    if monitor {
        println!("Opened u-blox device, waiting for solutions...");
    }
    loop {
        device
            .update(|packet| match packet {
                PacketRef::MonVer(packet) => {
                    if !status && !monitor {
                        if packet.software_version() == "ROM SPG 5.10 (7b202e)"
                            && packet.hardware_version() == "000A0000"
                        {
                            std::process::exit(0);
                        } else {
                            println!("Error: HW/SW version did not match");
                            std::process::exit(1);
                        }
                    }
                    println!(
                        "SW version: {} HW version: {}",
                        packet.software_version(),
                        packet.hardware_version()
                    );
                    if status && !monitor {
                        if packet.software_version() == "ROM SPG 5.10 (7b202e)"
                            && packet.hardware_version() == "000A0000"
                        {
                            std::process::exit(0);
                        } else {
                            println!("Error: HW/SW did not match");
                            std::process::exit(1);
                        }
                    }
                    println!("{:?}", packet);
                }
                PacketRef::NavPvt(sol) => {
                    let has_time = sol.fix_type() == GpsFix::Fix3D
                        || sol.fix_type() == GpsFix::GPSPlusDeadReckoning
                        || sol.fix_type() == GpsFix::TimeOnlyFix;
                    let has_posvel = sol.fix_type() == GpsFix::Fix3D
                        || sol.fix_type() == GpsFix::GPSPlusDeadReckoning;

                    if has_posvel {
                        let pos: Position = (&sol).into();
                        let vel: Velocity = (&sol).into();
                        println!(
                            "Latitude: {:.5} Longitude: {:.5} Altitude: {:.2}m",
                            pos.lat, pos.lon, pos.alt
                        );
                        println!(
                            "Speed: {:.2} m/s Heading: {:.2} degrees",
                            vel.speed, vel.heading
                        );
                        println!("Sol: {:?}", sol);
                    }

                    if has_time {
                        let time: DateTime<Utc> = (&sol).try_into().unwrap();
                        println!("Time: {:?}", time);
                    }
                }
                PacketRef::NavStatus(sol) => {
                    println!("{:?}", sol);
                }
                _ => {}
            })
            .unwrap();
    }
}

fn configure_port(baud: u32, port: &str, auto_baud_rate_applied: u32) {
    let s = serialport::SerialPortSettings {
        baud_rate: auto_baud_rate_applied,
        data_bits: serialport::DataBits::Eight,
        flow_control: serialport::FlowControl::None,
        parity: serialport::Parity::None,
        stop_bits: serialport::StopBits::One,
        timeout: Duration::from_millis(1),
    };
    match serialport::open_with_settings(port, &s) {
        Ok(port) => {
            let mut device = Device::new(port);
            device
                .write_all(
                    &CfgPrtUartBuilder {
                        portid: UartPortId::Uart1,
                        reserved0: 0,
                        tx_ready: 0,
                        mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
                        baud_rate: baud,
                        in_proto_mask: InProtoMask::all(),
                        out_proto_mask: OutProtoMask::UBLOX,
                        flags: 0,
                        reserved5: 0,
                    }
                    .into_packet_bytes(),
                )
                .unwrap();
        }
        Err(err) => {
            eprintln!("Error opening serial port: {}", err);
            std::process::exit(10);
        }
    }
}

fn main() {
    let matches = App::new("Maivin u-blox configuration tool")
        .about("Configure and test the gps u-blox device")
        .arg(
            Arg::with_name("port")
                .short("p")
                .long("port")
                .takes_value(true)
                .required(true)
                .help("Serial port to open"),
        )
        .arg(
            Arg::with_name("baud")
                .short("b")
                .long("baud")
                .takes_value(true)
                .help("Baud rate of the port"),
        )
        .arg(
            Arg::with_name("monitor")
                .short("m")
                .long("monitor")
                .takes_value(false)
                .help("Displays all the data from GPS"),
        )
        .arg(
            Arg::with_name("status")
                .short("s")
                .long("status")
                .takes_value(false)
                .help("Display the HW and SW verison of the GPS"),
        )
        .arg(
            Arg::with_name("debug")
                .short("d")
                .long("debug")
                .takes_value(false)
                .help("Show all the debug messages"),
        )
        .get_matches();

    let port = matches.value_of("port").unwrap();
    let baud: u32 = match matches.value_of("baud") {
        Some(val) => {
            let b: u32 = val.parse().expect("Could not parse baudrate as an integer");
            if BAUD_RATES.contains(&b) {
                b
            } else {
                println!(
                    "Unknown Baud Rate Found! Please choose from {:?}",
                    BAUD_RATES
                );
                std::process::exit(1);
            }
        }
        None => 1,
    };

    let mut baud_update = false;
    let monitor = matches.is_present("monitor");
    let status = matches.is_present("status");
    let debug = matches.is_present("debug");

    let mut max_byte_read = 0;
    let mut auto_baud_rate_applied = 0;
    for &baud_rate in &BAUD_RATES {
        match try_baud_rate(port, baud_rate) {
            Ok(byte_read) => {
                if byte_read > max_byte_read {
                    max_byte_read = byte_read;
                    auto_baud_rate_applied = baud_rate;
                }
            }
            Err(_) => {
                continue;
            }
        }
    }
    if auto_baud_rate_applied == 0 {
        if debug {
            println!("Error: No Suitable Baud Rate can be found");
        } else {
            println!("Error: Unable to get a suitable baud rate");
            std::process::exit(1);
        }
    } else if debug {
        println!(
            "Baud rate {} is working on port: {}",
            auto_baud_rate_applied, port
        );
        println!("Read {} out of 128 bytes", max_byte_read);
    }

    if baud != 1 && auto_baud_rate_applied != baud {
        baud_update = true;
    }

    if baud_update {
        if monitor || debug {
            println!("Changing baud rate from {auto_baud_rate_applied} to {baud}");
        }
        configure_port(baud, port, auto_baud_rate_applied);
        auto_baud_rate_applied = baud;
    }

    let s = serialport::SerialPortSettings {
        baud_rate: auto_baud_rate_applied,
        data_bits: serialport::DataBits::Eight,
        flow_control: serialport::FlowControl::None,
        parity: serialport::Parity::None,
        stop_bits: serialport::StopBits::One,
        timeout: Duration::from_millis(1),
    };
    match serialport::open_with_settings(port, &s) {
        Ok(port) => {
            let mut device = Device::new(port);
            device
                .write_all(
                    &CfgPrtUartBuilder {
                        portid: UartPortId::Uart1,
                        reserved0: 0,
                        tx_ready: 0,
                        mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
                        baud_rate: auto_baud_rate_applied,
                        in_proto_mask: InProtoMask::all(),
                        out_proto_mask: OutProtoMask::UBLOX,
                        flags: 0,
                        reserved5: 0,
                    }
                    .into_packet_bytes(),
                )
                .unwrap();
            device.wait_for_ack::<CfgPrtUart>().unwrap();
            device
                .write_all(
                    &CfgMsgAllPortsBuilder::set_rate_for::<NavPvt>([0, 1, 0, 0, 0, 0])
                        .into_packet_bytes(),
                )
                .unwrap();
            device.wait_for_ack::<CfgMsgAllPorts>().unwrap();

            device
                .write_all(
                    &CfgMsgAllPortsBuilder::set_rate_for::<NavStatus>([0, 1, 0, 0, 0, 0])
                        .into_packet_bytes(),
                )
                .unwrap();
            device.wait_for_ack::<CfgMsgAllPorts>().unwrap();

            // Send a packet request for the MonVer packet
            device
                .write_all(&UbxPacketRequest::request_for::<MonVer>().into_packet_bytes())
                .unwrap();
            get_gps_data(&mut device, monitor, status);
        }

        Err(err) => {
            eprintln!("Error opening serial port: {}", err);
            std::process::exit(10);
        }
    }
}
