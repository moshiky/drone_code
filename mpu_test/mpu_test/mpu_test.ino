#include<Wire.h>

#define NUM_READS 10

const int MPU=0x68; 
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
uint16_t AcX_log[NUM_READS], AcY_log[NUM_READS], AcZ_log[NUM_READS], GyX_log[NUM_READS], GyY_log[NUM_READS], GyZ_log[NUM_READS];

// -- help methods
void zero_arr(uint16_t arr[]) {
  memset(arr, 0, sizeof(arr));
}

void store_new_value(uint16_t arr[], int arr_size, uint16_t new_value) {
  for (int i = 0 ; i < arr_size-1 ; ++i) {
    arr[i] = arr[i+1];
  }
  arr[arr_size-1] = new_value;
}

uint16_t get_arr_avg(uint16_t arr[], int arr_size) {
  int arr_sum = 0;
  for (int i = 0 ; i < arr_size ; ++i) {
    arr_sum += arr[i];
  }
  return (uint16_t)(arr_sum / (float)arr_size);
}


// -- logic code

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  Serial.begin(9600);

  zero_arr(AcX_log);
  zero_arr(AcY_log);
  zero_arr(AcZ_log);
  zero_arr(GyX_log);
  zero_arr(GyY_log);
  zero_arr(GyZ_log);
}

void loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 12, true);  

  // read  
  AcX = Wire.read()<<8|Wire.read();    
  AcY = Wire.read()<<8|Wire.read();  
  AcZ = Wire.read()<<8|Wire.read();  
  GyX = Wire.read()<<8|Wire.read();  
  GyY = Wire.read()<<8|Wire.read();  
  GyZ = Wire.read()<<8|Wire.read();  

  // store new value in log
  store_new_value(AcX_log, NUM_READS, AcX);
  store_new_value(AcY_log, NUM_READS, AcY);
  store_new_value(AcZ_log, NUM_READS, AcZ);
  store_new_value(GyX_log, NUM_READS, GyX);
  store_new_value(GyY_log, NUM_READS, GyY);
  store_new_value(GyZ_log, NUM_READS, GyZ);

  // get avg. values over last K reads
  AcX = get_arr_avg(AcX_log, NUM_READS);
  AcY = get_arr_avg(AcY_log, NUM_READS);
  AcZ = get_arr_avg(AcZ_log, NUM_READS);
  GyX = get_arr_avg(GyX_log, NUM_READS);
  GyY = get_arr_avg(GyY_log, NUM_READS);
  GyZ = get_arr_avg(GyZ_log, NUM_READS);
  
  // print values
  Serial.print("AcX = "); Serial.print(AcX);  Serial.print("\t");
  Serial.print("AcY = "); Serial.print(AcY);  Serial.print("\t");
  Serial.print("AcZ = "); Serial.print(AcZ);  
  Serial.print("\t\t\t");
  
  Serial.print(GyX);  Serial.print("\t");
  Serial.print(GyY);  Serial.print("\t");
  Serial.print(GyZ);  
  Serial.println(" ");

  delay(200);
}
