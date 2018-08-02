
#define X_PIN A0
#define Y_PIN A1
#define Z_PIN A2

#define SERIAL_PORT_SPEED 57600

void setup() {
  // begin serial connection
  Serial.begin(SERIAL_PORT_SPEED);
  
  // initiate input pins' mode
  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(Z_PIN, INPUT);

}

void loop() {
  Serial.print("X="); 
  Serial.print(analogRead(X_PIN)); 
  Serial.print("        "); 

  Serial.print("Y="); 
  Serial.print(analogRead(Y_PIN)); 
  Serial.print("        "); 

  Serial.print("Z="); 
  Serial.println(analogRead(Z_PIN)); 

  delay(100);
}
