
#include <Wire.h>

#define GYRO_CALIBRATION_SAMPLES  1000
#define ACC_ANGLE_WEIGHT  0.0005
#define LOOP_DELAY  50

#define GYRO_READ_DIV  131.0
#define ACC_READ_DIV  16384.0

const double ANGLE_TRAVEL_FACTOR = (float)LOOP_DELAY / 1000.0;
const double DEG_TO_RAD_FACTOR = PI / 180.0;
const double RAD_TO_DEG_FACTOR = 1.0 / DEG_TO_RAD_FACTOR;
const double RAD_ANGLE_TRAVEL_FACTOR = ANGLE_TRAVEL_FACTOR * DEG_TO_RAD_FACTOR;

//Declaring some global variables
double gyro_x, gyro_y, gyro_z;
double last_gyro_x, last_gyro_y, last_gyro_z;
double acc_x, acc_y, acc_z;
double acc_total_vector;

int temperature;
double gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
double angle_pitch, angle_roll;
double angle_roll_acc, angle_pitch_acc;
double angle_pitch_output, angle_roll_output;

bool set_gyro_angles;

void calibrate_gyro_offsets() {
  gyro_x_cal = 0.0;
  gyro_y_cal = 0.0;
  gyro_z_cal = 0.0;
  
  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES ; ++i) {
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(10);
  }
  
  gyro_x_cal /= GYRO_CALIBRATION_SAMPLES;
  gyro_y_cal /= GYRO_CALIBRATION_SAMPLES;
  gyro_z_cal /= GYRO_CALIBRATION_SAMPLES;
}

void substruct_gyro_offset() {
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
}


void setup() {
  Wire.begin();                                                        //Start I2C as master
  Serial.begin(9600);                                                  //Use only for debugging
  
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  last_gyro_x = 0.0;
  last_gyro_y = 0.0;
  last_gyro_z = 0.0;

  angle_pitch = 0.0;
  angle_roll = 0.0;

  set_gyro_angles = false;

  Serial.println("Calibrating gyro's offsets...");
  calibrate_gyro_offsets();
  Serial.println("done offsets calibration");
  
  loop_timer = micros();                                               //Reset the loop timer
}

void loop() {

  // read current values
  read_mpu_6050_data();

  // fix offsets
  substruct_gyro_offset();
  
  // gyro angle calculations
  angle_pitch += ((float)(gyro_x + last_gyro_x) / 2.0) * ANGLE_TRAVEL_FACTOR;
  angle_roll += ((float)(gyro_y + last_gyro_y) / 2.0) * ANGLE_TRAVEL_FACTOR;
  
//  // compencate yaw movement 
//  angle_pitch += angle_roll * sin(((float)(gyro_z + last_gyro_z) / 2.0) * RAD_ANGLE_TRAVEL_FACTOR);
//  angle_roll -= angle_pitch * sin(((float)(gyro_z + last_gyro_z) / 2.0) * RAD_ANGLE_TRAVEL_FACTOR);

  // store current values
  last_gyro_x = gyro_x;
  last_gyro_y = gyro_y;
  last_gyro_z = gyro_z;
  
  // accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));  //Calculate the total accelerometer vector
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * RAD_TO_DEG_FACTOR;
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * (-RAD_TO_DEG_FACTOR);
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.19951;
  angle_roll_acc -= -1.11375;

  if(set_gyro_angles) {
    angle_pitch = angle_pitch * (1.0 - ACC_ANGLE_WEIGHT) + angle_pitch_acc * ACC_ANGLE_WEIGHT;
    angle_roll = angle_roll * (1.0 - ACC_ANGLE_WEIGHT) + angle_roll_acc * ACC_ANGLE_WEIGHT;
  }
  else {
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.1 + angle_pitch * 0.9;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.1 + angle_roll * 0.9;      //Take 90% of the output roll value and add 10% of the raw roll value

  Serial.print("pitch= "); Serial.print(angle_pitch);
  Serial.print("\t\troll= "); Serial.println(angle_roll);

  while(micros() - loop_timer < LOOP_DELAY);
  loop_timer = micros();
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  
  acc_x = (Wire.read()<<8|Wire.read()) / ACC_READ_DIV;
  acc_y = (Wire.read()<<8|Wire.read()) / ACC_READ_DIV;
  acc_z = (Wire.read()<<8|Wire.read()) / ACC_READ_DIV;
  
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  
  gyro_x = (Wire.read()<<8|Wire.read()) / GYRO_READ_DIV;
  gyro_y = (Wire.read()<<8|Wire.read()) / GYRO_READ_DIV;
  gyro_z = (Wire.read()<<8|Wire.read()) / GYRO_READ_DIV;

}

void setup_mpu_6050_registers(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}














