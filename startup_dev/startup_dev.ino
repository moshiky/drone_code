
#include <EnableInterrupt.h>
#include <Servo.h>

#define SERIAL_PORT_SPEED 57600
#define NUM_RC_CHANNELS 6

#define RL_CH_PIN     2   // CH1
#define UD_CH_PIN     3   // CH2
#define TR_CH_PIN     4   // CH3
#define YW_CH_PIN     5   // CH4
#define EMPTY_CH_PIN  6   // CH5
#define PWR_CH_PIN    7   // CH6

#define RL_CH_ID    0   // CH1
#define UD_CH_ID    1   // CH2
#define TR_CH_ID    2   // CH3
#define YW_CH_ID    3   // CH4
#define EMPTY_CH_ID 4   // CH5
#define PWR_CH_ID   5   // CH6

#define DELAY_MS  100

#define CH_MIN_VALUE  1000
#define CH_MAX_VALUE  2000

#define PWR_ON_VALUE  90

// global variables
uint16_t current_rc_values[NUM_RC_CHANNELS];
uint32_t pin_high_start[NUM_RC_CHANNELS];
volatile uint16_t rc_shared[NUM_RC_CHANNELS];

int rcChannelPins[] = {
  RL_CH_PIN, UD_CH_PIN, TR_CH_PIN, 
  YW_CH_PIN, EMPTY_CH_PIN, PWR_CH_PIN
};

void rc_read_values() {
  noInterrupts();
  memcpy(current_rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
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
void ud_ch_change_handler()     { pin_change_handler(UD_CH_ID,    UD_CH_PIN); }
void tr_ch_change_handler()     { pin_change_handler(TR_CH_ID,    TR_CH_PIN); }
void yw_ch_change_handler()     { pin_change_handler(YW_CH_ID,    YW_CH_PIN); }
void empty_ch_change_handler()  { pin_change_handler(EMPTY_CH_ID, EMPTY_CH_PIN); }
void pwr_ch_change_handler()    { pin_change_handler(PWR_CH_ID,   PWR_CH_PIN); }

int cmap(int original_value, int original_min, int original_max, int target_min, int target_max) {
  // map original value to target range
  int mapped_value = map(original_value, original_min, original_max, target_min, target_max);

  // return clipped value
  return min(max(target_min, mapped_value), target_max);
}

int cmap_pin(int original_value) {
  return cmap(original_value, CH_MIN_VALUE, CH_MAX_VALUE, 0, 100);
}

void setup() {
  // begin serial connection
  Serial.begin(SERIAL_PORT_SPEED);

  // initiate input pins' mode
  for (int i = 0; i < NUM_RC_CHANNELS; ++i) {
    pinMode(rcChannelPins[i], INPUT);
  }

  // enable interrupt on rc pins
  enableInterrupt(RL_CH_PIN, rl_ch_change_handler, CHANGE);
  enableInterrupt(UD_CH_PIN, ud_ch_change_handler, CHANGE);
  enableInterrupt(TR_CH_PIN, tr_ch_change_handler, CHANGE);
  enableInterrupt(YW_CH_PIN, yw_ch_change_handler, CHANGE);
  enableInterrupt(EMPTY_CH_PIN, empty_ch_change_handler, CHANGE);
  enableInterrupt(PWR_CH_PIN, pwr_ch_change_handler, CHANGE);

}

void loop() {
  // read current rc values
  rc_read_values();

  // print current power read
  int power_ch_value = current_rc_values[PWR_CH_ID];
  power_ch_value = cmap_pin(power_ch_value);
  Serial.print("POWER = ");
  Serial.println(power_ch_value);

  // run only if power is on
  if (power_ch_value > PWR_ON_VALUE) {
    Serial.println("I'm ON");
  }
  else {
    Serial.println("I'm OFF");
  }

  // wait for a while..
  delay(DELAY_MS);
}
