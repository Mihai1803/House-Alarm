#![no_std]
#![no_main]

#![allow(unused_mut)]

use embassy_executor::Spawner;

// Pheripherals and utilites
use embassy_rp::peripherals::{I2C1, PIN_1, PIN_2, PIN_5, PIN_6, PIN_16, PIN_17, PIN_18, PIN_19, PIN_20, PIN_21, PIN_22, PIN_26, PIN_27, PWM_CH1};
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
use embassy_futures::join::join;
use embassy_futures::select::{select, select4};
use embassy_futures::select::Either::{First as First2, Second as Second2};
use embassy_futures::select::Either4::{First as First4, Second as Second4, Third as Third4, Fourth as Fourth4};

// GPIO / PWM
use embassy_rp::gpio::{Input, Pull, Output, Level};
use embassy_rp::pwm::{Pwm, Config as PwmConfig};

// Time
use embassy_time::Delay;
use embassy_time::Timer;
use embassy_time::Instant;

// LCD
use embassy_rp::i2c::{I2c, Config as I2cConfig, InterruptHandler as I2cInterruptHandler};
use ag_lcd::{Cursor, LcdDisplay};
use port_expander::dev::pcf8574::Pcf8574;
use panic_halt as _;

// MicroSD bus
use core::cell::RefCell;
use embassy_rp::spi::{Spi, Config as SpiConfig};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;

// MicroSD
use embedded_sdmmc::{SdCard, VolumeManager, Mode, VolumeIdx};
use embedded_sdmmc::TimeSource;
use embedded_sdmmc::Timestamp;

bind_interrupts!(struct Irqs {
  // Use for the serial over USB driver
  USBCTRL_IRQ => UsbInterruptHandler<USB>;
  I2C1_IRQ => I2cInterruptHandler<I2C1>;
});


const DISPLAY_FREQ: u32 = 200_000;
const TOP_BUZZER: u16 = 0x8000;
const TOP_SERVO: u16 = 2500;
const SOUND_VELOCITY: u32 = 340; 
const PANIC_DISTANCE: u32 = 70;

static MOTION_CHANNEL: Channel<ThreadModeRawMutex, Motion, 64> = Channel::new();
static DISTANCE_CHANNEL: Channel<ThreadModeRawMutex, u32, 64> = Channel::new();
static BUTTON_CHANNEL: Channel<ThreadModeRawMutex, Button, 64> = Channel::new();

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

#[derive(Default)]
struct Time();
impl TimeSource for Time {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
          year_since_1970: 0,
          zero_indexed_month: 0,
          zero_indexed_day: 0,
          hours: 0,
          minutes: 0,
          seconds: 0,
        }
    }
}


// The task used by the serial port driver over USB
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

// The task used to detect motion
#[embassy_executor::task]
async fn detect_motion(motion_sensor: Input<'static, PIN_27>, channel_sender: Sender<'static, ThreadModeRawMutex, Motion, 64>) {
  loop {
    match motion_sensor.is_high() {
        true => {
          channel_sender.send(Motion::MotionDetected).await;
        }
        false => {
          channel_sender.send(Motion::MotionNotDetected).await;
        }
    }
    Timer::after_micros(10).await;
    Timer::after_secs(5).await;
  }
}


// The task used to calculate the distance from an object to the sensor 
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

    // calculate distance in cm (the formula can be found in the datasheet)
    let distance = end_time.as_micros() as u32 * SOUND_VELOCITY / 2 / 10000;

    channel_sender.send(distance).await;

    Timer::after_secs(5).await;
  }
}


// The task used to control the servomotor using PWM
// In this context Forward means the motor is rotating to the left and Backward means the motor is rotating to the right
#[embassy_executor::task]
async fn servomotor(mut direction: Direction, mut servo: Pwm<'static, PWM_CH1>, mut config: PwmConfig) {
  let mut count = 0;
  loop {
      count += 1;
      match direction {
          Direction::Forward => {
            if count == 4 {
              count = 0;
              direction = Direction::Backward;
            } else {
              config.compare_b += config.top / 10;
              servo.set_config(&config);
              Timer::after_secs(1).await;
            }
          }
          Direction::Backward => {
            if count == 4 {
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

// The task used to select which button is pressed
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

// Function used to map the keys on the 16x16 keypad
fn map_button(column_index: usize, row_index: usize) -> &'static str {
  let map: [[&str; 4]; 4] = [
    ["1", "2", "3", "A"],
    ["4", "5", "6", "B"],
    ["7", "8", "9", "C"],
    ["*", "0", "#", "D"],
  ];
  return  map[row_index][column_index];
}

// Function used to get the input key from the keypad
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

    // Display initialization using I2C
    let sda = peripherals.PIN_14;
    let scl = peripherals.PIN_15;

    let lcd_delay = Delay;

    let mut lcd_config = I2cConfig::default();
    lcd_config.frequency = DISPLAY_FREQ;

    let i2c = I2c::new_async(peripherals.I2C1, scl, sda, Irqs, lcd_config.clone());
    let mut i2c_expander = Pcf8574::new(i2c, true, true, true);

    let mut lcd: LcdDisplay<_, _> = LcdDisplay::new_pcf8574(&mut i2c_expander, lcd_delay)
    .with_cursor(Cursor::Off)
    .with_reliable_init(10000)
    .build();
 
    // Spawn motion sensor task
    let motion_sensor = Input::new(peripherals.PIN_27, Pull::Up); 
    spawner.spawn(detect_motion(motion_sensor, MOTION_CHANNEL.sender())).unwrap();

    // Spawn distance sensor task
    let mut trigger = Output::new(peripherals.PIN_1, Level::Low);
    let mut echo =  Input::new(peripherals.PIN_2, Pull::None);
    spawner.spawn(calculate_distance(trigger, echo, DISTANCE_CHANNEL.sender())).unwrap();
    
    // Configure PWM for servormotor and spwan the task
    let mut servo_config: PwmConfig = Default::default();
    servo_config.top = TOP_SERVO;
    servo_config.divider = FixedU16::<U4>::from_num(125);
    servo_config.compare_b = servo_config.top - 1; 
    let mut servo = Pwm::new_output_b(peripherals.PWM_CH1, peripherals.PIN_3, servo_config.clone());
    spawner.spawn(servomotor(Direction::Backward, servo, servo_config)).unwrap();
    
    // Configure PWM for buzzer
    let mut buzzer_config: PwmConfig = Default::default();
    buzzer_config.top = TOP_BUZZER;
    buzzer_config.compare_a = 0;
    let mut buzzer = Pwm::new_output_a(peripherals.PWM_CH2, peripherals.PIN_4, buzzer_config.clone());
    
    // Spawn task for buttons
    let mut stop_button = Input::new(peripherals.PIN_5, Pull::Up);
    let mut submit_button = Input::new(peripherals.PIN_6, Pull::Up);
    spawner.spawn(buttons(stop_button, submit_button, BUTTON_CHANNEL.sender())).unwrap();

    // Keypad
    let mut column_4 = Output::new(peripherals.PIN_16, Level::Low);
    let mut column_3 = Output::new(peripherals.PIN_17, Level::Low);
    let mut column_2 = Output::new(peripherals.PIN_18, Level::Low);
    let mut column_1 = Output::new(peripherals.PIN_19, Level::Low);

    let mut row_4 = Input::new(peripherals.PIN_20, Pull::Down);
    let mut row_3 = Input::new(peripherals.PIN_21, Pull::Down);
    let mut row_2 = Input::new(peripherals.PIN_22, Pull::Down);
    let mut row_1 = Input::new(peripherals.PIN_26, Pull::Down);


    // MicroSD initialization using SPI
    let mut micro_sd_cs = Output::new(peripherals.PIN_9, Level::High);
    let mut clk = peripherals.PIN_10;
    let mut mosi = peripherals.PIN_11;
    let mut miso = peripherals.PIN_12;
      
    let mut micro_sd_delay = Delay;
    let mut micro_sd_config = SpiConfig::default();
    micro_sd_config.frequency = 400000;

    let mut spi = Spi::new_blocking(peripherals.SPI1, clk, mosi, miso, micro_sd_config.clone());
    let spi_bus = NoopMutex::new(RefCell::new(spi));
    let spi_device = SpiDevice::new(&spi_bus, micro_sd_cs);

    let mut dummy_cs = Output::new(peripherals.PIN_8, Level::High);
    let mut sdcard = SdCard::new(spi_device, dummy_cs, micro_sd_delay);
    let mut time_source = Time::default();

    Timer::after_millis(100).await;

    let mut volume_mgr = VolumeManager::new(sdcard, time_source);

  
    let mut volume0 = match volume_mgr.get_volume(VolumeIdx(0)) {
      Ok(volume) => volume, 
      Err(err) => {
        info!("Error opening volume: {:?}", err);
        return;
      }
    };

    Timer::after_millis(100).await;
  
    let mut root_dir = match volume_mgr.open_root_dir(&volume0) {
        Ok(root) => root,
        Err(err) => {
          info!("Error opening root directory: {:?}", err);
          return;
        }
    };

    Timer::after_millis(100).await;

    // print all files found in volume0
    volume_mgr
    .iterate_dir(&volume0, &root_dir, |ent| {
        info!(
            "/{}.{}",
            core::str::from_utf8(ent.name.base_name()).unwrap(),
            core::str::from_utf8(ent.name.extension()).unwrap()
        );
    })
    .unwrap();

    Timer::after_millis(100).await;

    lcd.print("Checking...");
    loop {
  
      // wait for the response from the motion and distance sensors
      let (motion, distance) = join(MOTION_CHANNEL.receive(), DISTANCE_CHANNEL.receive()).await;
      Timer::after_millis(100).await;
      info!("Distance {}", distance);
      if motion == Motion::MotionDetected && distance <= PANIC_DISTANCE {

        // start buzzer
        buzzer_config.compare_a = buzzer_config.top / 10;
        buzzer.set_config(&buzzer_config);

        lcd.clear();
        lcd.print("Alarm tiggered");

        Timer::after_millis(100).await;

        // log trigger to the MicroSD
        let mut successful_write = false;
        if let Ok(mut data_file) = volume_mgr.open_file_in_dir(&mut volume0, &root_dir, "data.txt", Mode::ReadWriteAppend) {
          let _write_count = volume_mgr.write(&mut volume0, &mut data_file, b"alarm trigger\n").unwrap();
          volume_mgr.close_file(&volume0, data_file).unwrap();
          successful_write = true;
        }

        if successful_write {
          info!("Success writing to the microSD");
        } else {
          info!("Could not write to the microSD");
        }


        // make sure that the channel is empty before waiting for select
        while let Ok(_) = BUTTON_CHANNEL.try_receive() {}

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
                     
                      // display code on lcd and wait 2 seconds before checking the code
                      lcd.print(&code);
                      Timer::after_secs(2).await;
                      info!("Code:{}", code);

                      // read password from microSD
                      let mut successful_read = false;
                      let mut password: String<8> = String::try_from("").unwrap();
                      if let Ok(mut file) = volume_mgr.open_file_in_dir(&mut volume0, &root_dir, "password.txt", Mode::ReadOnly) {
                        let mut buf = [0u8; 32];
                        let read_count = volume_mgr.read(&volume0, &mut file, &mut buf).unwrap();
                        volume_mgr.close_file(&volume0, file).unwrap();
                  
                        if read_count >= 2 {
                            for iterator in 0..read_count {
                              info!("bytes: {:?}", buf[iterator] as char);
                              password.push(buf[iterator] as char).unwrap();
                            }
                            info!("password: {:?}", password);
                            successful_read = true;
                        }
                      }

                      if successful_read {
                        info!("Success reading");
                      } else {
                        info!("Could not read from microSD");
                      }

                      // check if code is the same as the password
                      // stop buzzer if code is correct
                      if password.as_bytes() == code.as_bytes() {
                        buzzer_config.compare_a = 0;
                        buzzer.set_config(&buzzer_config);
                        lcd.clear();
                        lcd.print("Correct code");

                        // log succesful stop to the microSD
                        let mut successful_write = false;
                        if let Ok(mut correct_file) = volume_mgr.open_file_in_dir(&mut volume0, &root_dir, "correct.txt", Mode::ReadWriteAppend) {
                          let _write_count = volume_mgr.write(&mut volume0, &mut correct_file, b"succesful stop using code\n").unwrap();
                          volume_mgr.close_file(&volume0, correct_file).unwrap();
                          successful_write = true;
                        }

                        if successful_write {
                          info!("Success writing 'succesful stop' to the microSD");
                        } else {
                          info!("Could not write to the microSD");
                        }


                      } else {
                        lcd.clear();
                        lcd.print("Incorrect code");
                        Timer::after_secs(2).await; 
                        buzzer_config.compare_a = 0;
                        buzzer.set_config(&buzzer_config);

                        // log unsuccesful stop to the microSD
                        let mut successful_write = false;
                        if let Ok(mut incorrect_file) = volume_mgr.open_file_in_dir(&mut volume0, &root_dir, "NOTCOR~1.TXT", Mode::ReadWriteAppend) {
                          let _write_count = volume_mgr.write(&mut volume0, &mut incorrect_file, b"unsuccesful stop using code\n").unwrap();
                          volume_mgr.close_file(&volume0, incorrect_file).unwrap();
                          successful_write = true;
                        }

                        if successful_write {
                          info!("Success writing 'unsuccesful stop' to the microSD");
                        } else {
                          info!("Could not write to the microSD");
                        }
                        
                      }
                  }
            }
          }
          Second2(_) => {
            buzzer_config.compare_a = 0;
            buzzer.set_config(&buzzer_config);
            lcd.clear();
            lcd.print("Alarm stopped");
          }  
        }
      
      }
      Timer::after_secs(5).await;
      lcd.clear();
      lcd.print("Checking...");
      Timer::after_secs(5).await;

    }
}
