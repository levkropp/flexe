// Separate-process test adapter for the pinned upstream Agon eZ80 core.
// It has no substitute VDP: UART, flow control and VSync come from Flexe.
use agon_ez80_emulator::{gpio, AgonMachine, AgonMachineConfig, RamInit, SerialLink};
use std::{
    collections::VecDeque,
    io::{Read, Write},
    net::{TcpListener, TcpStream},
    sync::{
        atomic::{AtomicBool, AtomicI32, Ordering},
        mpsc, Arc, Mutex,
    },
};
struct Link {
    stream: TcpStream,
    queue: Arc<Mutex<VecDeque<u8>>>,
    cts: Arc<AtomicBool>,
}
impl SerialLink for Link {
    fn send(&mut self, b: u8) {
        self.stream.write_all(&[0, 1, 0, b]).unwrap();
    }
    fn recv(&mut self) -> Option<u8> {
        self.queue.lock().unwrap().pop_front()
    }
    fn read_clear_to_send(&mut self) -> bool {
        self.cts.load(Ordering::Relaxed)
    }
}
struct Dummy;
impl SerialLink for Dummy {
    fn send(&mut self, _: u8) {}
    fn recv(&mut self) -> Option<u8> {
        None
    }
    fn read_clear_to_send(&mut self) -> bool {
        true
    }
}
fn main() {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 4 {
        eprintln!("usage: {} PORT MOS.bin SDCARD_DIRECTORY", args[0]);
        std::process::exit(2);
    }
    let port: u16 = args[1].parse().expect("PORT must be 0..65535");
    let listener = TcpListener::bind(("127.0.0.1", port)).unwrap();
    println!("LISTEN {}", listener.local_addr().unwrap().port());
    std::io::stdout().flush().unwrap();
    let (mut stream, _) = listener.accept().unwrap();
    stream.set_nodelay(true).unwrap();
    let mut magic = [0u8; 15];
    stream.read_exact(&mut magic).unwrap();
    assert_eq!(&magic, b"FLEXE-AGON-MOS\n");
    let queue = Arc::new(Mutex::new(VecDeque::new()));
    let cts = Arc::new(AtomicBool::new(true));
    let shutdown = Arc::new(AtomicBool::new(false));
    let gpios = Arc::new(gpio::GpioSet::new());
    let exit = Arc::new(AtomicI32::new(0));
    let (tx, _rx) = mpsc::channel();
    let mut machine = AgonMachine::new(AgonMachineConfig {
        uart0_link: Box::new(Link {
            stream: stream.try_clone().unwrap(),
            queue: queue.clone(),
            cts: cts.clone(),
        }),
        uart1_link: Box::new(Dummy),
        soft_reset: Arc::new(AtomicBool::new(false)),
        emulator_shutdown: shutdown.clone(),
        exit_status: exit.clone(),
        paused: Arc::new(AtomicBool::new(false)),
        clockspeed_hz: 18_432_000,
        ram_init: RamInit::Zero,
        mos_bin: args[2].clone().into(),
        gpios: gpios.clone(),
        tx_gpio_vga_frame: tx,
        interrupt_precision: 16,
        external_ram_size: 512 * 1024,
    });
    machine.set_sdcard_directory(args[3].clone().into());
    let reader_exit = exit.clone();
    std::thread::spawn(move || {
        let code = loop {
            let mut hdr = [0u8; 3];
            if stream.read_exact(&mut hdr).is_err() {
                break 1;
            }
            let len = u16::from_le_bytes([hdr[1], hdr[2]]) as usize;
            if len > 256 {
                break 1;
            }
            let mut payload = vec![0; len];
            if stream.read_exact(&mut payload).is_err() {
                break 1;
            }
            match hdr[0] {
                0 if len > 0 => queue.lock().unwrap().extend(payload),
                1 if len == 0 => {
                    gpios.b.set_input_pin(1, true);
                    gpios.b.set_input_pin(1, false);
                }
                2 if len == 1 => cts.store(payload[0] != 0, Ordering::Relaxed),
                3 if len == 0 => break reader_exit.load(Ordering::Relaxed),
                _ => break 1,
            }
        };
        shutdown.store(true, Ordering::Relaxed);
        std::process::exit(code);
    });
    machine.start(None);
    std::process::exit(exit.load(Ordering::Relaxed));
}
