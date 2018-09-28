
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <EnableInterrupt.h>
#include <Servo.h>

#define PRINT_MOTORS  false
#define SERIAL_PORT_SPEED 57600

#define NUM_RC_CHANNELS 6

// === Pins configuration

#define MPU_INT_PIN  2

#define RL_CH_PIN     4   // CH1
#define FB_CH_PIN     5   // CH2
#define TR_CH_PIN     6   // CH3
#define YW_CH_PIN     8   // CH4
#define EMPTY_CH_PIN  9   // CH5  --- not in use --> not connected
#define PWR_CH_PIN    10  // CH6

#define MOTOR_FL_PIN  A3
#define MOTOR_FR_PIN  11
#define MOTOR_BL_PIN  7
#define MOTOR_BR_PIN  12

#define RECEIVER_VIN  A0
#define RECEIVER_GND  A1
// #define MPU_SDA  A4
// #define MPU_SCL  A5

#define RL_CH_ID    0   // CH1
#define FB_CH_ID    1   // CH2
#define TR_CH_ID    2   // CH3
#define YW_CH_ID    3   // CH4
#define EMPTY_CH_ID 4   // CH5
#define PWR_CH_ID   5   // CH6

#define MOTOR_SYNC_DELAY_MS  7 * 1000
#define LOOP_DELAY_MS  50

#define CH_MIN_VALUE  1000
#define CH_MAX_VALUE  2000

#define CH_TR_MIN   5   //  accessible min value
#define CH_TR_MAX   90  //  accessible max value

#define PWR_ON_VALUE  90
#define MOTOR_SYNC_VALUE  20  // connected, but not spinning
#define MOTOR_MIN_VALUE  58   // spinning slowest
#define MOTOR_MAX_VALUE  160  // spinning fastest

#define THROTTLE_STICK_CHANGE 3
#define MAX_STICK_CHANGE  8
#define MOTOR_CHANGE  1
#define MAX_ACCL_VALUE  4

#define ANGLE_CHANGE_UNIT 5
#define MOTOR_CHANGE_UNIT 1

#define POWER_OFF_MODE  false
#define POWER_ON_MODE   true

// ========== Global variables
MPU6050 mpu;
uint8_t dev_status;      // return status after each device operation (0 = success, !0 = error)
uint8_t mpu_int_status;   // holds actual interrupt status byte from MPU
bool dmp_ready = false;  // set true if DMP init was successful
uint16_t packet_size;    // expected DMP packet size (default is 42 bytes)
uint16_t fifo_count;     // count of all bytes currently in FIFO
uint8_t fifo_buffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 accl;       // [x, y, z]            accel sensor measurements
VectorInt16 real_accl;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 world_accl; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint16_t current_rc_values[NUM_RC_CHANNELS];
uint32_t pin_high_start[NUM_RC_CHANNELS];

volatile uint16_t rc_shared[NUM_RC_CHANNELS];
volatile bool mpu_interrupt = false;     // indicates whether MPU interrupt pin has gone high

int rcChannelPins[] = {
  RL_CH_PIN, FB_CH_PIN, TR_CH_PIN, 
  YW_CH_PIN, EMPTY_CH_PIN, PWR_CH_PIN
};

bool last_power_mode = POWER_OFF_MODE;   //  false = OFF, true = ON

Servo motor_f_l;
Servo motor_f_r;
Servo motor_b_l;
Servo motor_b_r;

int last_motor_f_l;
int last_motor_f_r;
int last_motor_b_l;
int last_motor_b_r;

// ========== Helper functions

void mpu_interrupt_handler() {
    mpu_interrupt = true;
}

void init_mpu() {
  // init connection library
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // connect to device
  Serial.println("Initializing I2C devices...");
  mpu.initialize();
  pinMode(MPU_INT_PIN, INPUT);

  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // load and configure the DMP
  Serial.println("Initializing DMP...");
  dev_status = mpu.dmpInitialize();

  // set gyro & accel. offsets
  mpu.setXGyroOffset(-21);
  mpu.setYGyroOffset(32);
  mpu.setZGyroOffset(67);
  
  mpu.setXAccelOffset(-4101);
  mpu.setYAccelOffset(1529);
  mpu.setZAccelOffset(1530);  // 1541

  // make sure it worked (returns 0 if so)
  if (dev_status == 0) {
    // turn on the DMP, now that it's ready
    Serial.println("Enabling DMP...");
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println("Enabling interrupt detection (Arduino external interrupt 0)...");
    enableInterrupt(MPU_INT_PIN, mpu_interrupt_handler, RISING);
//    attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmp_data_ready, RISING);
    mpu_int_status = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println("DMP ready! Waiting for first interrupt...");
    dmp_ready = true;

    // get expected DMP packet size for later comparison
    packet_size = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP Initialization failed (code ");
    Serial.print(dev_status);
    Serial.println(")");
  }
}

void rc_read_values() {
  noInterrupts();
  memcpy(current_rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();

  for (int i = 0; i < NUM_RC_CHANNELS; ++i) {
    current_rc_values[i] = cmap_pin(current_rc_values[i]);
  }
}

void pin_change_handler(uint8_t channel_id, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    pin_high_start[channel_id] = micros();
  } else {
    uint16_t high_length = (uint16_t)(micros() - pin_high_start[channel_id]);
    rc_shared[channel_id] = high_length;
  }
}

void rl_ch_change_handler()     { pin_change_handler(RL_CH_ID,    RL_CH_PIN); }
void fb_ch_change_handler()     { pin_change_handler(FB_CH_ID,    FB_CH_PIN); }
void tr_ch_change_handler()     { pin_change_handler(TR_CH_ID,    TR_CH_PIN); }
void yw_ch_change_handler()     { pin_change_handler(YW_CH_ID,    YW_CH_PIN); }
void empty_ch_change_handler()  { pin_change_handler(EMPTY_CH_ID, EMPTY_CH_PIN); }
void pwr_ch_change_handler()    { pin_change_handler(PWR_CH_ID,   PWR_CH_PIN); }

int clip_value(int original_value, int min_value, int max_value) {
  return min(max(min_value, original_value), max_value);
}

int cmap(int original_value, int original_min, int original_max, int target_min, int target_max) {
  // map original value to target range
  int mapped_value = map(original_value, original_min, original_max, target_min, target_max);

  // return clipped value
  return clip_value(mapped_value, target_min, target_max);
}

int cmap_pin(int original_value) {
  return cmap(original_value, CH_MIN_VALUE, CH_MAX_VALUE, 0, 100);
}

int cmap_stick(int original_value) {
  return cmap(original_value, 0, 100, -MAX_STICK_CHANGE, MAX_STICK_CHANGE);
}

int map_angle_to_axis_state(int angle_value) {
  return map(angle_value, -ANGLE_CHANGE_UNIT, ANGLE_CHANGE_UNIT, -MOTOR_CHANGE_UNIT, MOTOR_CHANGE_UNIT);
}

double cmap_accl(double original_value) {
  double source_range_max = MAX_ACCL_VALUE;
  double source_range_min = -source_range_max;
  double source_range_size = source_range_max - source_range_min;

  double target_range_max = MAX_STICK_CHANGE;
  double target_range_min = -target_range_max;
  double target_range_size = target_range_max - target_range_min;
  
  return target_range_min + target_range_size * (original_value - source_range_min) / source_range_size;
}

int mclip(int original_value) {
  if (MOTOR_SYNC_VALUE == original_value) {
    return MOTOR_SYNC_VALUE;
  }
  else {
    return clip_value(original_value, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
  }
}

void apply_motor_change(int f_l_change, int f_r_change, 
                        int b_l_change, int b_r_change)
{
  // calculate and write new values last values
  write_motor_values(
    
    last_motor_f_l + f_l_change,    last_motor_f_r + f_r_change,
    
    last_motor_b_l + b_l_change,    last_motor_b_r + b_r_change
  
  );
}

void write_motor_values(int f_l, int f_r, 
                        int b_l, int b_r) 
{
    // save last values
    last_motor_f_l = mclip(f_l);   last_motor_f_r = mclip(f_r);
    last_motor_b_l = mclip(b_l);   last_motor_b_r = mclip(b_r);
  
    // write values to motors
    motor_f_l.write(last_motor_f_l);   motor_f_r.write(last_motor_f_r);
    motor_b_l.write(last_motor_b_l);   motor_b_r.write(last_motor_b_r);

    // report values
    if (PRINT_MOTORS) {
      Serial.println("-------------------------------");
      Serial.print(last_motor_f_l);   Serial.print("     ");   Serial.println(last_motor_f_r);
      Serial.print(last_motor_b_l);   Serial.print("     ");   Serial.println(last_motor_b_r);
    }
}

// ========== Logic functions

void setup() {
  // begin serial connection
  Serial.begin(SERIAL_PORT_SPEED);

  // initiate output pins
  pinMode(RECEIVER_VIN, OUTPUT); 
  pinMode(RECEIVER_GND, OUTPUT);
  digitalWrite(RECEIVER_VIN, HIGH);
  digitalWrite(RECEIVER_GND, LOW);

  // init mpu
  init_mpu();
  
  // initiate input pins' mode
  for (int i = 0; i < NUM_RC_CHANNELS; ++i) {
    pinMode(rcChannelPins[i], INPUT);
  }

  // enable interrupt on rc pins
  enableInterrupt(RL_CH_PIN, rl_ch_change_handler, CHANGE);
  enableInterrupt(FB_CH_PIN, fb_ch_change_handler, CHANGE);
  enableInterrupt(TR_CH_PIN, tr_ch_change_handler, CHANGE);
  enableInterrupt(YW_CH_PIN, yw_ch_change_handler, CHANGE);
  enableInterrupt(EMPTY_CH_PIN, empty_ch_change_handler, CHANGE);
  enableInterrupt(PWR_CH_PIN, pwr_ch_change_handler, CHANGE);

  // attach each output pin to related servo object
  motor_f_l.attach(MOTOR_FL_PIN);
  motor_f_r.attach(MOTOR_FR_PIN);
  motor_b_l.attach(MOTOR_BL_PIN);
  motor_b_r.attach(MOTOR_BR_PIN);
  
  // run initialization code
  sync_init();
}

void sync_init() {
  // write motors syncronization signal value
  write_motor_values(MOTOR_SYNC_VALUE, MOTOR_SYNC_VALUE, MOTOR_SYNC_VALUE, MOTOR_SYNC_VALUE);

  // wait for syncronization process to complete
  delay(MOTOR_SYNC_DELAY_MS);
}

void handle_power_mode_change(bool current_power_mode) {
  if (current_power_mode == POWER_OFF_MODE) {
    write_motor_values(MOTOR_SYNC_VALUE, MOTOR_SYNC_VALUE, MOTOR_SYNC_VALUE, MOTOR_SYNC_VALUE);
  }
  else if (current_power_mode != last_power_mode) {
    // write value
    write_motor_values(MOTOR_MIN_VALUE, MOTOR_MIN_VALUE, MOTOR_MIN_VALUE, MOTOR_MIN_VALUE);
  }
  
  // store current mode
  last_power_mode = current_power_mode;
}

void loop() {
  // validate dmp is ready
  if (!dmp_ready) return;
  
  // read current rc values
  rc_read_values();

  // get power mode read
  int power_ch_value = current_rc_values[PWR_CH_ID];
  
  // run only if power is on
  if (power_ch_value > PWR_ON_VALUE) {
    // check for power mode change and handle it
    handle_power_mode_change(POWER_ON_MODE);

    // apply flight logic
    apply_flight_logic();
  }
  else {
    // check for power mode change and handle it
    handle_power_mode_change(POWER_OFF_MODE);
  }

  // wait for a while..
  delay(LOOP_DELAY_MS);
}

void apply_flight_logic() {
  // get throttle channel read
  int throttle_ch_value = \
    cmap(current_rc_values[TR_CH_ID], CH_TR_MIN, CH_TR_MAX, -THROTTLE_STICK_CHANGE, THROTTLE_STICK_CHANGE);

  // get yaw channel read --  positive = move CW command.
  int yaw_ch_value = cmap_stick(current_rc_values[YW_CH_ID]);

  // read acc sensor
  if (!mpu_interrupt) {
    return;
  }
  
  // reset interrupt flag and get INT_STATUS byte
  mpu_interrupt = false;
  mpu_int_status = mpu.getIntStatus();
  read_mpu_values();  // updates world_accl to currect acceleration sensor read

  // map angle to motor speed change for each axis
  int pitch_size = map_angle_to_axis_state(ypr[1]);   // positive = drone's nose is up
  int roll_size = map_angle_to_axis_state(ypr[2]);    // positive = drone tilts left
  
  Serial.print(ypr[0]);   Serial.print("\t");   Serial.print(ypr[1]);   Serial.print("\t");   Serial.println(ypr[2]);
  Serial.print(pitch_size);   Serial.print("\t");   Serial.println(roll_size);

//  // calculate overall expected acceleration on each axis
//  double margin = 1;
//  int move_cw = 0;  // todo: complete- get angular speed over z axis
//  int should_tilt_forward = (ypr[1] > margin ? 1 : (ypr[1] < -margin ? -1 : 0));
//  int should_tilt_right = (ypr[2] > margin ? 1 : (ypr[2] < -margin ? -1 : 0));

//  Serial.print(throttle_ch_value);   Serial.print("\t");
//  Serial.print(cmap_stick(current_rc_values[FB_CH_ID]));   Serial.print("\t");   
//  Serial.print(cmap_stick(current_rc_values[RL_CH_ID]));   Serial.print("\t"); 
//  Serial.println(cmap_stick(current_rc_values[YW_CH_ID]));

//  // add remote signals and limit tilt angles
//  double max_tilt_angle = 15;
//  if (abs(ypr[1]) < max_tilt_angle) {
//    should_tilt_forward += cmap_stick(current_rc_values[FB_CH_ID]);
//  }
//  
//  if (abs(ypr[2]) < max_tilt_angle) {
//    should_tilt_right += cmap_stick(current_rc_values[RL_CH_ID]);
//  }
  
//  // get YAW channel read
//  int yw_ch_change = 2 * cmap_stick(current_rc_values[YW_CH_ID]);

//  // calculate motor values
//  int fl_motor_value_change = throttle_ch_value + (-move_cw * MOTOR_CHANGE) + (-should_tilt_forward * MOTOR_CHANGE) + (should_tilt_right * MOTOR_CHANGE);
//  int fr_motor_value_change = throttle_ch_value + (move_cw * MOTOR_CHANGE)  + (-should_tilt_forward * MOTOR_CHANGE) + (-should_tilt_right * MOTOR_CHANGE);
//  int bl_motor_value_change = throttle_ch_value + (move_cw * MOTOR_CHANGE)  + (should_tilt_forward * MOTOR_CHANGE)  + (should_tilt_right * MOTOR_CHANGE);
//  int br_motor_value_change = throttle_ch_value + (-move_cw * MOTOR_CHANGE) + (should_tilt_forward * MOTOR_CHANGE)  + (-should_tilt_right * MOTOR_CHANGE);

//  // update motor values
//  apply_motor_change(
//    fl_motor_value_change, fr_motor_value_change,
//    bl_motor_value_change, br_motor_value_change
//  );
}

void read_mpu_values() {
  // get current FIFO count
  fifo_count = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpu_int_status & 0x10) || fifo_count == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
  }
  else if (mpu_int_status & 0x02) {  // otherwise, check for DMP data ready interrupt (this should happen frequently)
    // wait for correct available data length, should be a VERY short wait
    while (fifo_count < packet_size) fifo_count = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifo_buffer, packet_size);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifo_count -= packet_size;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifo_buffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // convert angles to degrees and set values according to sensor orientation
    ypr[0] *= 180/M_PI;
    ypr[1] *= -180/M_PI;    // so now positive is up
    ypr[2] *= 180/M_PI;     // positive is right side up
  }
}

