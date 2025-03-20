use socketcan::{CANFilter, CANFrame, CANSocket};

pub struct CanBus {
    socket: CANSocket,
}

impl CanBus {
    pub fn new(interface: &str) -> Result<Self, String> {
        let socket = CANSocket::open(interface).map_err(|e| e.to_string())?;
        Ok(CanBus { socket })
    }

    pub fn set_filter(&self, filters: Vec<CANFilter>) -> Result<(), String> {
        self.socket.set_filters(&filters).map_err(|e| e.to_string())
    }

    pub fn send_frame(&self, frame: CANFrame) -> Result<(), String> {
        self.socket.write_frame(&frame).map_err(|e| e.to_string())
    }

    pub fn receive_frame(&self) -> Result<CANFrame, String> {
        self.socket.read_frame().map_err(|e| e.to_string())
    }
}

fn main() {
    let can_bus = CanBus::new("vcan0").expect("Failed to create CAN bus");

    let frame =
        CANFrame::new(0x123, &[1, 2, 3, 4], false, false).expect("Failed to create CAN frame");
    can_bus.send_frame(frame).expect("Failed to send CAN frame");

    let received_frame = can_bus
        .receive_frame()
        .expect("Failed to receive CAN frame");
    println!("Received frame: {:?}", received_frame);
}
