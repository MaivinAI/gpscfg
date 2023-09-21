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
        let mut serial_port = serialport::open_with_settings(port_name, &settings)?;
        let mut buffer = vec![0u8; 128];
        let byte_read = serial_port.read(&mut buffer)?;
        if byte_read > max_byte_read {
            max_byte_read = byte_read;
        }
    }
    Ok(max_byte_read)
}

fn main() {
    let matches = App::new("ublox CLI example program")
        .about("Demonstrates usage of the Rust ublox API")
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
                .short("s")
                .long("baud")
                .takes_value(true)
                .help("Baud rate of the port"),
        )
        .get_matches();

    println!("Auto Selecting Baud Rate for GPS...");
    let port = matches.value_of("port").unwrap();
    let port_name = port;
    let mut max_byte_read = 0;
    let mut correct_baud_rate = 0;
    for &baud_rate in &BAUD_RATES {
        match try_baud_rate(port_name, baud_rate) {
            Ok(byte_read) => {
                if byte_read > max_byte_read {
                    max_byte_read = byte_read;
                    correct_baud_rate = baud_rate;
                }
            }
            Err(_) => {
                println!("Baud rate {} is not working", baud_rate);
                continue;
            }
        }
    }
    if correct_baud_rate == 0 {
        correct_baud_rate = 38400;
        println!(
            "ERROR: No Suitable Baud Rate can be found, setting baud rate to {correct_baud_rate}"
        );
    } else if correct_baud_rate != 0 {
        println!(
            "Baud rate {} is working on port: {}",
            correct_baud_rate, port_name
        );
        println!("Read {} out of 128 bytes", max_byte_read);
    }

    let s = serialport::SerialPortSettings {
        baud_rate: correct_baud_rate,
        data_bits: serialport::DataBits::Eight,
        flow_control: serialport::FlowControl::None,
        parity: serialport::Parity::None,
        stop_bits: serialport::StopBits::One,
        timeout: Duration::from_millis(1),
    };
    let port = serialport::open_with_settings(port, &s).unwrap();
    let mut device = Device::new(port);

    // Configure the device to talk UBX
    device
        .write_all(
            &CfgPrtUartBuilder {
                portid: UartPortId::Uart1,
                reserved0: 0,
                tx_ready: 0,
                mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
                baud_rate: correct_baud_rate,
                in_proto_mask: InProtoMask::all(),
                out_proto_mask: OutProtoMask::UBLOX,
                flags: 0,
                reserved5: 0,
            }
            .into_packet_bytes(),
        )
        .unwrap();
    device.wait_for_ack::<CfgPrtUart>().unwrap();

    // Enable the NavPosVelTime packet
    device
        .write_all(
            &CfgMsgAllPortsBuilder::set_rate_for::<NavPosVelTime>([0, 1, 0, 0, 0, 0])
                .into_packet_bytes(),
        )
        .unwrap();
    device.wait_for_ack::<CfgMsgAllPorts>().unwrap();

    // Send a packet request for the MonVer packet
    device
        .write_all(&UbxPacketRequest::request_for::<MonVer>().into_packet_bytes())
        .unwrap();

    // Start reading data
    println!("Opened u-blox device, waiting for solutions...");
    loop {
        device
            .update(|packet| match packet {
                PacketRef::MonVer(packet) => {
                    println!(
                        "SW version: {} HW version: {}",
                        packet.software_version(),
                        packet.hardware_version()
                    );
                    println!("{:?}", packet);
                }
                PacketRef::NavPosVelTime(sol) => {
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
                _ => {}
            })
            .unwrap();
    }
}
