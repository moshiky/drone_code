#include <EnableInterrupt.h>
#include <Servo.h>

#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  4

#define RC_CH_THR  0  // 0 < SLOWER < FASTER
#define RC_CH_RL   1  // 0 < LEFT < 50 < RIGHT < 100
#define RC_CH_FB   2  // 0 < BACKWARD < 50 < FORWARD < 100
#define RC_CH_YAW  3  // 0 < LEFT < 50 < RIGHT < 100

#define RC_CH_THR_INPUT  5
#define RC_CH_RL_INPUT   ?3?
#define RC_CH_FB_INPUT   2
#define RC_CH_YAW_INPUT  4

#define X_STATE_PIN 6
#define Y_STATE_PIN 7
#define Z_STATE_PIN 8

#define M_F_R_PIN ?3?
#define M_F_L_PIN 9
#define M_B_R_PIN 10
#define M_B_L_PIN 11

#define MIN_VAL 1070
#define MAX_VAL 2000

#define NEUTRAL_SPEED_PCS   10

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

Servo motor_f_r;
Servo motor_f_l;
Servo motor_b_r;
Servo motor_b_l;

unsigned int last_motor_f_r;
unsigned int last_motor_f_;
unsigned int last_motor_b_r;
unsigned int last_motor_b_l;

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

void write_motor_values(unsigned int f_l, unsigned int f_r, 
                        unsigned int b_l, unsigned int b_r) 
{
    // write values to motors
    motor_f_l.write(f_l);   motor_f_r.write(f_r);
    motor_b_l.write(b_l);   motor_b_r.write(b_r);

    // save last values
    last_motor_f_l = f_l;   last_motor_f_r = f_r;
    last_motor_b_l = b_l;   last_motor_b_r = b_r;
}

void setup() {
  // begin serial connection
  Serial.begin(SERIAL_PORT_SPEED);

  // initiate input pins' mode
  pinMode(RC_CH_THR_INPUT, INPUT);
  pinMode(RC_CH_RL_INPUT, INPUT);
  pinMode(RC_CH_FB_INPUT, INPUT);
  pinMode(RC_CH_YAW_INPUT, INPUT);

  pinMode(X_STATE_PIN, INPUT);
  pinMode(Y_STATE_PIN, INPUT);
  pinMode(Z_STATE_PIN, INPUT);
  

  // enable interrupt on input pins
  enableInterrupt(RC_CH_THR_INPUT, calc_ch_thr, CHANGE);
  enableInterrupt(RC_CH_RL_INPUT, calc_ch_rl, CHANGE);
  enableInterrupt(RC_CH_FB_INPUT, calc_ch_fb, CHANGE);
  enableInterrupt(RC_CH_YAW_INPUT, calc_ch_yaw, CHANGE);

  // attach each output pin to related servo object
  motor_f_r.attach(M_F_R_PIN);
  motor_f_l.attach(M_F_L_PIN);
  motor_b_r.attach(M_B_R_PIN);
  motor_b_l.attach(M_B_L_PIN);

  // init start flag
  start_stage = 0;

  // verify motors are stopped
  write_motor_values(0, 0, 0, 0);
}

void loop() {
  // read values from RF receiver  
  rc_read_values();

  // map readed values to motors range
  float thr_value = min(map(rc_values[RC_CH_THR], MIN_VAL, MAX_VAL, 0, 100), 85) / float(100);
  float rl_value = map(rc_values[RC_CH_RL], MIN_VAL, MAX_VAL, 0, 100) / float(100);
  float fb_value = map(rc_values[RC_CH_FB], MIN_VAL, MAX_VAL, 0, 100) / float(100);
  float yaw_value = map(rc_values[RC_CH_YAW], MIN_VAL, MAX_VAL, 0, 100) / float(100);

  // print RF values
  String to_print = "";
  to_print += "throttle=";
  to_print += thr_value;
  to_print += "\t rl=";
  to_print += rl_value;
  to_print += "\t fb=";
  to_print += fb_value;
  to_print += "\t yaw=";
  to_print += yaw_value;
  to_print += "\n";
  Serial.print(to_print); 

  if (0 == start_stage) {
    if (thr_value > 70) {
      Serial.println("Move throttle all the way down");
      start_stage = 1;
      // todo: validate the stick stays in this position for 2 secs
    }
  }
  else if (1 == start_stage) {
    if (thr_value < 10) {
      start_stage = 2;

      // init last motor values - set all to initial neutral speed
      write_motor_values(NEUTRAL_SPEED_PCS, NEUTRAL_SPEED_PCS, NEUTRAL_SPEED_PCS, NEUTRAL_SPEED_PCS);
    }
  }
  else {
  
    // read accelerometer values
    current_x_state = digitalRead(X_STATE_PIN);
    current_y_state = digitalRead(Y_STATE_PIN);
    current_z_state = digitalRead(Z_STATE_PIN);
  
    // calculate change values
    int max_change = 10;
    int fb_change = (-max_change) + 2 * max_change * fb_value;
    int rl_change = (-max_change) + 2 * max_change * rl_value;
    int yaw_change = 0 ; //(-max_change) + 2 * max_change * yaw_value;
  
    // calculate motors values
    int base_value = int(thr_value * 180);
    
  //  int motor_f_l_value = base_value + (-fb_change) + (rl_change) + (yaw_change);
  //  int motor_f_r_value = base_value + (-fb_change) + (-rl_change) + (-yaw_change);
  //  int motor_b_l_value = base_value + (fb_change) + (rl_change) + (-yaw_change);
  //  int motor_b_r_value = base_value + (fb_change) + (-rl_change) + (yaw_change);
    int motor_f_l_value = base_value;
    int motor_f_r_value = base_value;
    int motor_b_l_value = base_value;
    int motor_b_r_value = base_value;
  
    // print motors values
    to_print = "";
    to_print += "FL=";
    to_print += motor_f_l_value;
    to_print += "\t FR=";
    to_print += motor_f_r_value;
    to_print += "\nBL=";
    to_print += motor_b_l_value;
    to_print += "\t BR=";
    to_print += motor_b_r_value;
    to_print += "\n";
    Serial.print(to_print); 
  
    // write values to motors
    write_motor_values(motor_f_l_value, motor_f_r_value, motor_b_l_value, motor_b_r_value);
  }

  delay(500);
}

