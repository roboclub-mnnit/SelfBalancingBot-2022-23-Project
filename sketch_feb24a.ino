#include "MPU9250.h"

MPU9250 mpu;
float roll_offset = 0, pitch_offset = 0;
uint32_t prev_ms;
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);
  while (!mpu.setup(0x68)) {
    Serial.println("MPU connection failed.");
    delay(5000);
  }
  mpu.calibrateAccelGyro();
  delay(2000);
  for(int i = 0; i < 200; i++){
    if(mpu.update()){
      roll_offset += mpu.getRoll();
      pitch_offset += mpu.getPitch(); 
    }
    else
      i--;
  }
  roll_offset /= 200;
  pitch_offset /= 200;
  Serial.print("Offsets: ");
  Serial.print(roll_offset);
  Serial.print('\t');
  Serial.println(pitch_offset);
  prev_ms = millis();
}

void loop() {
    if (mpu.update()) {
        if (millis() > prev_ms + 25) {
            print_roll_pitch_yaw();
            prev_ms = millis();
        }
    }
}

void print_roll_pitch_yaw() {
    float pitch = 0, roll = 0;
    for(int i = 0; i < 50; i++){
      pitch += mpu.getPitch();
      roll += mpu.getRoll();
    }
    pitch /= 50;
    roll /= 50;
    Serial.print(pitch-pitch_offset);
    Serial.print(" ");
    Serial.println(roll-roll_offset);
}
