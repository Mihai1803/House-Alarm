#![no_std]
#![no_main]

#![allow(unused_imports)]
#![allow(unused_mut)]



use embassy_executor::Spawner;

// USB driver
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::{bind_interrupts, peripherals::USB};
use log::info;

// Channel
use embassy_sync::channel::{Channel, Sender};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

// Futures
use embassy_futures::join::{join, Join};
use embassy_futures::select::Select;
use embassy_futures::select::Either::{First, Second};


// GPIO
use embassy_rp::gpio::{Input, Pull, Output, Level};
use embassy_rp::pwm::{Pwm, Config as PwmConfig};

// Time
use embassy_time::Delay;
use embassy_time::Timer;
use embassy_time::Instant;

// LCD
use embassy_rp::i2c::{I2c, Config as I2cConfig, InterruptHandler as I2cInterruptHandler};
use embassy_rp::peripherals::{I2C1, PIN_0, PIN_1, PIN_2, PWM_CH1};
use ag_lcd::{Cursor, LcdDisplay};
use port_expander::dev::pcf8574::Pcf8574;
use panic_halt as _;


bind_interrupts!(struct Irqs {
    // Use for the serial over USB driver
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    I2C1_IRQ => I2cInterruptHandler<I2C1>;
});

#[derive(PartialEq)]
enum Motion {
  MotionDetected,
  MotionNotDetected
}

enum Direction {
  Forward,
  Backward
}


const DISPLAY_FREQ: u32 = 200_000;
const TOP: u16 = 0x8000;
const PANIC_DISTANCE: u32 = 10;

static MOTION_CHANNEL: Channel<ThreadModeRawMutex, Motion, 64> = Channel::new();
static DISTANCE_CHANNEL: Channel<ThreadModeRawMutex, u32, 64> = Channel::new();

// The task used by the serial port driver over USB
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn detect_motion(motion_sensor: Input<'static, PIN_0>, channel_sender: Sender<'static, ThreadModeRawMutex, Motion, 64>) {
  loop {
    match  motion_sensor.is_high() {
        true => {
          channel_sender.send(Motion::MotionDetected).await;
        }
        false => {
          channel_sender.send(Motion::MotionNotDetected).await;
        }
    }
    Timer::after_secs(10).await;
  }
}

#[embassy_executor::task]
async fn calculate_distance(mut trigger: Output<'static, PIN_1>, mut echo: Input<'static, PIN_2>, channel_sender: Sender<'static, ThreadModeRawMutex, u32, 64>) {
  loop {
    // start (trigger) the pin by making a high -> low transition 
    trigger.set_high();
    Timer::after_micros(10).await;
    trigger.set_low();

    // wait for echo pin to go high
    while echo.is_low() {}

    // start measuring time
    let mut start_time = Instant::now();

    // wait for echo pin to go low
    while echo.is_high() {}

    // stop measuring time (duration)
    let mut end_time = start_time.elapsed();

    // calculate distance in cm
    let distance = end_time.as_micros() as u32 * 343 / 2 / 10000;

    channel_sender.send(distance).await;

    Timer::after_secs(1).await;
  }
}

#[embassy_executor::task]
async fn servomotor(mut direction: Direction, mut servo: Pwm<'static, PWM_CH1>, mut config: PwmConfig) {
  loop {
      match direction {
          Direction::Forward => {
            if config.compare_b + config.top == config.top {
              direction = Direction::Backward;
            }
            config.compare_b += config.top / 10;
            servo.set_config(&config);
          }
          Direction::Backward => {
            if config.compare_b - config.top == 0 {
              direction = Direction::Forward;
            }
            config.compare_b -= config.top / 10;
            servo.set_config(&config);
          }
      }
      Timer::after_secs(1).await;
  }
}




#[embassy_executor::main]
async fn main(spawner: Spawner) {

    let peripherals = embassy_rp::init(Default::default());

    
    // Start the serial port over USB driver
    let driver = Driver::new(peripherals.USB, Irqs);
    spawner.spawn(logger_task(driver)).unwrap();

    // Display initialization
    let sda = peripherals.PIN_14;
    let scl = peripherals.PIN_15;

    let delay = Delay;

    let mut lcd_config = I2cConfig::default();
    lcd_config.frequency = DISPLAY_FREQ;

    let i2c = I2c::new_async(peripherals.I2C1, scl, sda, Irqs, lcd_config.clone());
    let mut i2c_expander = Pcf8574::new(i2c, true, true, true);

    let mut lcd: LcdDisplay<_, _> = LcdDisplay::new_pcf8574(&mut i2c_expander, delay)
    .with_cursor(Cursor::Off)
    .with_reliable_init(10000)
    .build();

    // Write to LCD
    lcd.print("Hello World");

    // motion sensor
    let motion_sensor = Input::new(peripherals.PIN_0, Pull::Up);
    spawner.spawn(detect_motion(motion_sensor, MOTION_CHANNEL.sender())).unwrap();

    // distance sensor
    let mut trigger = Output::new(peripherals.PIN_1, Level::Low);
    let mut echo =  Input::new(peripherals.PIN_2, Pull::None);
    spawner.spawn(calculate_distance(trigger, echo, DISTANCE_CHANNEL.sender())).unwrap();

    // servomotor
    let mut servo_config: PwmConfig = Default::default();
    servo_config.top = TOP;
    servo_config.compare_b = 0;

    let mut servo = Pwm::new_output_b(peripherals.PWM_CH1, peripherals.PIN_3, servo_config.clone());
    spawner.spawn(servomotor(Direction::Forward, servo, servo_config)).unwrap();

    // buzzer
    let mut buzzer_config: PwmConfig = Default::default();
    buzzer_config.top = TOP;
    buzzer_config.compare_a = 0;

    let mut buzzer = Pwm::new_output_a(peripherals.PWM_CH2, peripherals.PIN_4, buzzer_config.clone());

    
    loop {
      let (motion, distance) = join(MOTION_CHANNEL.receive(), DISTANCE_CHANNEL.receive()).await;

      if motion == Motion::MotionDetected && distance <= PANIC_DISTANCE {
        // start buzzer
        buzzer_config.compare_a = buzzer_config.top / 2;
        buzzer.set_config(&buzzer_config);
      }


    }
}
