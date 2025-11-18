#include <SPI.h>

#define GYRO_MISO 0
#define GYRO_MOSI 0
#define GYRO_CS 0
#define GYRO_SCK 0

#define MAG_MISO 0
#define MAG_MOSI 0
#define MAG_CS 0
#define MAG_SCK 0

#define ACC_X A0
#define ACC_Y A1
#define ACC_Z A2



uint16_t gyro[3];
int accel[3];
int mag[3];
int angle[3];

int timeinterval[3];
unsigned long currenttime;

void setup() {
  Serial.begin(9600);
  SPI.begin();
  pinMode(GYRO_CS, OUTPUT);
  pinMode(MAG_CS, OUTPUT);
  currenttime = micros();
}

void loop() {
  


}

void getGyroAxes(uint16_t* gyro){
  digitalWrite(GYRO_CS, LOW);

  uint8_t GYRO_ADDRESS = 0x43; 
  
  for(int i = 0; i < 3; i++){

    SPI.transfer(GYRO_ADDRESS);
    gyro[i]= SPI.transfer(0x00) << 8;
    GYRO_ADDRESS++;

    SPI.transfer(GYRO_ADDRESS);
    gyro[i] = gyro[i] | SPI.transfer(0x00);
    GYRO_ADDRESS++;
    
  }

  digitalWrite(GYRO_CS, HIGH);
}

void getAccelAxes(int* accel){
  accel[0] = analogRead(ACC_X);
  accel[1] = analogRead(ACC_Y);
  accel[2] = analogRead(ACC_Z);
}

void getAngles(){
  unsigned long newtimes = micros();
  unsigned long timedelta = newtimes - currenttime;
  angle[0] += gyro[0] * timedelta;
  angle[1] += gyro[1] * timedelta;
  angle[2] += gyro[2] * timedelta;
  currenttime = newtimes;
  
}





void getMagAxes(byte* mag){
    
}
