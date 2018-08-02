#include <EnableInterrupt.h>
#include <Servo.h>

#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  4

#define RC_CH_THR  0  // 0 < SLOWER < FASTER
#define RC_CH_RL   1  // 0 < LEFT < 50 < RIGHT < 100
#define RC_CH_FB   2  // 0 < BACKWARD < 50 < FORWARD < 100
#define RC_CH_YAW  3  // 0 < LEFT < 50 < RIGHT < 100

#define RC_CH_THR_INPUT  4
#define RC_CH_RL_INPUT   3
#define RC_CH_FB_INPUT   2
#define RC_CH_YAW_INPUT  5

#define X_STATE_PIN A0
#define Y_STATE_PIN A1
#define Z_STATE_PIN A2

#define M_F_L_PIN 9
#define M_F_R_PIN 10
#define M_B_L_PIN 11
#define M_B_R_PIN 12

#define MIN_VAL 1070
#define MAX_VAL 2000

#define MIN_RUN_VALUE 20
#define MAX_RUN_VALUE 165

#define SYNC_VALUE  10
#define CHANGE_STEPS  5

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

Servo motor_f_r;
Servo motor_f_l;
Servo motor_b_r;
Servo motor_b_l;

unsigned int last_motor_f_l;
unsigned int last_motor_f_r;
unsigned int last_motor_b_l;
unsigned int last_motor_b_r;

int start_stage;


void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch_thr()  { calc_input(RC_CH_THR, RC_CH_THR_INPUT); }
void calc_ch_rl()   { calc_input(RC_CH_RL, RC_CH_RL_INPUT); }
void calc_ch_fb()   { calc_input(RC_CH_FB, RC_CH_FB_INPUT); }
void calc_ch_yaw()  { calc_input(RC_CH_YAW, RC_CH_YAW_INPUT); }

void write_motor_values(int f_l, int f_r, 
                        int b_l, int b_r) 
{
    // save last values
    last_motor_f_l = clip_value(f_l);   last_motor_f_r = clip_value(f_r);
    last_motor_b_l = clip_value(b_l);   last_motor_b_r = clip_value(b_r);
  
    // write values to motors
    motor_f_l.write(last_motor_f_l);   motor_f_r.write(last_motor_f_r);
    motor_b_l.write(last_motor_b_l);   motor_b_r.write(last_motor_b_r);
}

unsigned int clip_value(int original_value) {
  return min(max(MIN_RUN_VALUE, original_value), MAX_RUN_VALUE);
}

void setup() {
  // begin serial connection
  Serial.begin(SERIAL_PORT_SPEED);

  // initiate input pins' mode
  pinMode(RC_CH_THR_INPUT, INPUT);
//  pinMode(RC_CH_RL_INPUT, INPUT);
//  pinMode(RC_CH_FB_INPUT, INPUT);
//  pinMode(RC_CH_YAW_INPUT, INPUT);

  pinMode(X_STATE_PIN, INPUT);
  pinMode(Y_STATE_PIN, INPUT);
  pinMode(Z_STATE_PIN, INPUT);
//  

  // enable interrupt on input pins
  enableInterrupt(RC_CH_THR_INPUT, calc_ch_thr, CHANGE);
//  enableInterrupt(RC_CH_RL_INPUT, calc_ch_rl, CHANGE);
//  enableInterrupt(RC_CH_FB_INPUT, calc_ch_fb, CHANGE);
//  enableInterrupt(RC_CH_YAW_INPUT, calc_ch_yaw, CHANGE);

  // attach each output pin to related servo object
  motor_f_r.attach(M_F_R_PIN);
  motor_f_l.attach(M_F_L_PIN);
  motor_b_r.attach(M_B_R_PIN);
  motor_b_l.attach(M_B_L_PIN);

  // verify motors are stopped
  write_motor_values(SYNC_VALUE, SYNC_VALUE, SYNC_VALUE, SYNC_VALUE);
  // wait for motors to connect
  delay(5 * 1000)

  // init start flag
  start_stage = 0;
}

void loop() {
  // read values from RF receiver  
  rc_read_values();

  // map readed values to motors range
  Serial.print("ORG_THL: "); Serial.print(rc_values[RC_CH_THR]);
  float thr_value = max(0, min(map(rc_values[RC_CH_THR], MIN_VAL, MAX_VAL, 0, 180), 160));
  Serial.print(" MAPPED THL: "); Serial.println(thr_value);

  // read accelerometer values
  int x_state = map(analogRead(X_STATE_PIN), 0, ?, -CHANGE_STEPS, CHANGE_STEPS)   // left-right balance. left < 0 < right
  int y_state = map(analogRead(Y_STATE_PIN), 0, ?, -CHANGE_STEPS, CHANGE_STEPS)   // back-front balance. back < 0 < front
  int z_state = map(analogRead(Z_STATE_PIN), 0, ?, -CHANGE_STEPS, CHANGE_STEPS)   // down-up balance. down < 0 < up

  // add thrutle to z value
  thr_value = map(thr_value, 0, 160, -CHANGE_STEPS, CHANGE_STEPS);
  z_state -= thr_value;

  // calculate axis changes
  int motor_f_l_change = (-x_state) + (y_state) + (-z_state);
  int motor_f_r_change = (x_state) + (y_state) + (-z_state);
  int motor_b_l_change = (-x_state) + (-y_state) + (-z_state);
  int motor_b_r_change = (x_state) + (-y_state) + (-z_state);

  // write motor values
  write_motor_values(
    last_motor_f_l + motor_f_l_change,    last_motor_f_r + motor_f_r_change, 
    last_motor_b_l + motor_b_l_change,    last_motor_b_r + motor_b_r_change
  );

  delay(100);
}

