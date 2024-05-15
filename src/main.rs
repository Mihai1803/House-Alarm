#![no_std]
#![no_main]

#![allow(unused_imports)]
#![allow(unused_mut)]


use embassy_executor::Spawner;
use embassy_rp::config;

use heapless::String;
use fixed::types::extra::U4;
use fixed::FixedU16;

// USB driver
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::{bind_interrupts, peripherals::USB};
use log::info;

// Channel
use embassy_sync::channel::{Channel, Sender};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

// Futures
use embassy_futures::join::{join, Join};
use embassy_futures::select::{select, select4, Select};
use embassy_futures::select::Either::{First as First2, Second as Second2};
use embassy_futures::select::Either4::{First as First4, Second as Second4, Third as Third4, Fourth as Fourth4};



// GPIO
use embassy_rp::gpio::{Input, Pull, Output, Level};
use embassy_rp::pwm::{Pwm, Config as PwmConfig};

// Time
use embassy_time::Delay;
use embassy_time::Timer;
use embassy_time::Instant;
use embassy_time::Duration;

// LCD
use embassy_rp::i2c::{I2c, Config as I2cConfig, InterruptHandler as I2cInterruptHandler};
use embassy_rp::peripherals::{I2C1, PIN_0, PIN_1, PIN_2, PIN_5, PIN_6, PIN_16, PIN_17, PIN_18, PIN_19, PIN_20, PIN_21, PIN_22, PIN_26, PWM_CH1};
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

enum Button {
  StopButton,
  SubmitButton,
}


const DISPLAY_FREQ: u32 = 200_000;
const TOP_BUZZER: u16 = 0x8000;
const TOP_SERVO: u16 = 2500;
const PANIC_DISTANCE: u32 = 70;

static mut TEST_PASSWORD: &'static str = "123A";

static MOTION_CHANNEL: Channel<ThreadModeRawMutex, Motion, 64> = Channel::new();
static DISTANCE_CHANNEL: Channel<ThreadModeRawMutex, u32, 64> = Channel::new();
static BUTTON_CHANNEL: Channel<ThreadModeRawMutex, Button, 64> = Channel::new();

// The task used by the serial port driver over USB
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::task]
async fn detect_motion(motion_sensor: Input<'static, PIN_0>, channel_sender: Sender<'static, ThreadModeRawMutex, Motion, 64>) {
  loop {
    match motion_sensor.is_high() {
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
  let mut count = 0;
  loop {
      count += 1;
      match direction {
          Direction::Forward => {
            if count == 9 {
              count = 0;
              direction = Direction::Backward;
            } else {
              config.compare_b += config.top / 10;
              servo.set_config(&config);
              Timer::after_secs(1).await;
            }
          }
          Direction::Backward => {
            if count == 9 {
              count = 0;
              direction = Direction::Forward;
            } else {
              config.compare_b -= config.top / 10;
              servo.set_config(&config);
              Timer::after_secs(1).await;
            }
          }
      }
      Timer::after_secs(1).await;
  }
}

#[embassy_executor::task]
async fn buttons(mut stop_button: Input<'static, PIN_5>, mut submit_button: Input<'static, PIN_6>, channel_sender: Sender<'static, ThreadModeRawMutex, Button, 64>) {
  loop {
    let select_button = select(stop_button.wait_for_falling_edge(), submit_button.wait_for_falling_edge()).await;
  
    match select_button {
        First2(_) => {
          channel_sender.send(Button::StopButton).await;
        },
        Second2(_) => {
          channel_sender.send(Button::SubmitButton).await;
        }
    }
  }
}

fn map_button(column_index: usize, row_index: usize) -> &'static str {
  let map: [[&str; 4]; 4] = [
    ["1", "2", "3", "A"],
    ["4", "5", "6", "B"],
    ["7", "8", "9", "C"],
    ["*", "0", "#", "D"],
  ];
  return  map[row_index][column_index];
}

async fn keypad(column_1: &mut Output<'static, PIN_19>,
                column_2: &mut Output<'static, PIN_18>,
                column_3: &mut Output<'static, PIN_17>,
                column_4: &mut Output<'static, PIN_16>,
                row_1: &mut Input<'static, PIN_26>,
                row_2: &mut Input<'static, PIN_22>,
                row_3: &mut Input<'static, PIN_21>,
                row_4: &mut Input<'static, PIN_20>) -> String<8> {

  let mut keypad_code: String<8> = String::try_from("").unwrap();     
  for column_index in 1..=4 {
    match column_index {
      1 => column_1.set_high(),
      2 => column_2.set_high(),
      3 => column_3.set_high(),
      4 => column_4.set_high(),
      _ => unreachable!(),
    }

    info!("Press button on column {}", column_index);
        let button_pressed = select4(
          row_1.wait_for_rising_edge(),
          row_2.wait_for_rising_edge(),
          row_3.wait_for_rising_edge(),
          row_4.wait_for_rising_edge(),
        ).await;

        match button_pressed {
          First4(_) => {
            let button = map_button(column_index - 1, 0 as usize);
            keypad_code.push_str(&button).unwrap();
            info!("row1, button: {}", button);
          }
          Second4(_) => {
            let button = map_button(column_index - 1,1 as usize);
            keypad_code.push_str(&button).unwrap();
            info!("row2, button: {}", button);
          }
          Third4(_) => {
            let button = map_button(column_index - 1, 2 as usize);
            keypad_code.push_str(&button).unwrap();
            info!("row3, button: {}", button);
          }
          Fourth4(_) => {
            let button = map_button(column_index - 1, 3 as usize);
            keypad_code.push_str(&button).unwrap();
            info!("row4, button: {}", button);
          }
        }

        match column_index {
          1 => column_1.set_low(),
          2 => column_2.set_low(),
          3 => column_3.set_low(),
          4 => column_4.set_low(),
          _ => unreachable!(),
        }
    }
    return keypad_code;
    
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
 
    // motion sensor
    let motion_sensor = Input::new(peripherals.PIN_0, Pull::Up);
    spawner.spawn(detect_motion(motion_sensor, MOTION_CHANNEL.sender())).unwrap();

    // distance sensor
    let mut trigger = Output::new(peripherals.PIN_1, Level::Low);
    let mut echo =  Input::new(peripherals.PIN_2, Pull::None);
    spawner.spawn(calculate_distance(trigger, echo, DISTANCE_CHANNEL.sender())).unwrap();
    
    // servomotor
    let mut servo_config: PwmConfig = Default::default();
    servo_config.top = TOP_SERVO;
    servo_config.divider = FixedU16::<U4>::from_num(125);
    servo_config.compare_b = servo_config.top - 1; 
    let mut servo = Pwm::new_output_b(peripherals.PWM_CH1, peripherals.PIN_3, servo_config.clone());
    spawner.spawn(servomotor(Direction::Backward, servo, servo_config)).unwrap();
    
    // buzzer
    let mut buzzer_config: PwmConfig = Default::default();
    buzzer_config.top = TOP_BUZZER;
    buzzer_config.compare_a = 0;
    let mut buzzer = Pwm::new_output_a(peripherals.PWM_CH2, peripherals.PIN_4, buzzer_config.clone());
    
    // stop/submit button
    let mut stop_button = Input::new(peripherals.PIN_5, Pull::Up);
    let mut submit_button = Input::new(peripherals.PIN_6, Pull::Up);
    spawner.spawn(buttons(stop_button, submit_button, BUTTON_CHANNEL.sender())).unwrap();

    // keypad
    let mut column_4 = Output::new(peripherals.PIN_16, Level::Low);
    let mut column_3 = Output::new(peripherals.PIN_17, Level::Low);
    let mut column_2 = Output::new(peripherals.PIN_18, Level::Low);
    let mut column_1 = Output::new(peripherals.PIN_19, Level::Low);

    let mut row_4 = Input::new(peripherals.PIN_20, Pull::Down);
    let mut row_3 = Input::new(peripherals.PIN_21, Pull::Down);
    let mut row_2 = Input::new(peripherals.PIN_22, Pull::Down);
    let mut row_1 = Input::new(peripherals.PIN_26, Pull::Down);

    
    lcd.print("Checking...");
    loop {
  
      let (motion, distance) = join(MOTION_CHANNEL.receive(), DISTANCE_CHANNEL.receive()).await;

      if motion == Motion::MotionDetected && distance <= PANIC_DISTANCE {
        // start buzzer
        buzzer_config.compare_a = buzzer_config.top / 10;
        buzzer.set_config(&buzzer_config);


        // the buzzer will stop if the correct code is introduced or if the timer expires
        let button_or_timeout = select(BUTTON_CHANNEL.receive(), Timer::after_secs(10)).await;
        match button_or_timeout {
          First2(button) => {
            match button {
                  Button::StopButton => {
                    buzzer_config.compare_a = 0;
                    buzzer.set_config(&buzzer_config);
                    lcd.clear();
                    lcd.print("Alarm Stopped");
                    
                  }
                  Button::SubmitButton => {
                      lcd.clear();
                      lcd.print("Enter code:");

                      // get code from keypad
                      let code = keypad(&mut column_1, &mut column_2, &mut column_3, &mut column_4, &mut row_1, &mut row_2, &mut row_3, &mut row_4).await;
                     
                      // display code on lcd and wait 3 seconds before checking the code
                      lcd.print(&code);
                      Timer::after_secs(3).await;

                      info!("Code:{}", code);
                      // check if code is correct
                      // stop buzzer if code is correct
                      if unsafe { TEST_PASSWORD.as_bytes() == code.as_bytes() } {
                        buzzer_config.compare_a = 0;
                        buzzer.set_config(&buzzer_config);
                        lcd.clear();
                        lcd.print("Correct code");
                      } else {
                        lcd.clear();
                        lcd.print("Incorrect code");
                      }
                  }
            }
          }
          Second2(_) => {
            buzzer_config.compare_a = 0;
            buzzer.set_config(&buzzer_config);
          }  
        }
      
      }
      Timer::after_secs(5).await;
      lcd.clear();
      lcd.print("Checking...");
      Timer::after_secs(5).await;

    }
}
