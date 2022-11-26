#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>

LSM6 imu;
Balboa32U4Motors motors;
int32_t gyro; // Current rotational speed around Y axis
int32_t gyroValZero; // Value to callibrate gyro
int32_t deltaAngle; // Integrated angle from gyro sensor.
int32_t gyroAngle;
int32_t accAngle; // Calculated angle from accelerometer
int32_t rollAngle; // 
int32_t preAngle;
int32_t mSpeed;

int32_t UpdateInterval = 10;
unsigned long curMill ;
unsigned long prevMill = millis();

int32_t balanceAngle = 8500;
int32_t curError;

void setup() {
  Serial.begin(9600);
  //digitalRead(14);
  // Initialize IMU.
  Wire.begin();
  if (!imu.init())
  {
    while(true)
    {
      Serial.println("Failed to detect and initialize internal measurment unit!");
      delay(200);
    }
  }
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b01011000); // 208 Hz, 1000 deg/s
  
  //Wait for IMU readings to stabilize
  delay(1000);

  updateGyroValZero();
  
}

int count = 0;
void loop() {
  /*if (count ++ > 1000){
  updateGyroValZero();
  count = 0;
  }*/
  
  sampleTime();
  imu.read();

  gyro = (imu.g.y - gyroValZero) / 29; //deg/s
  deltaAngle = gyro * UpdateInterval; // endring i dette intervallet. [deg/s]*mS = mdeg 
  gyroAngle = deltaAngle + preAngle;

  accAngle = (atan2(imu.a.z, imu.a.x)*180000/PI);//-86000;

  rollAngle = sensorFusion(gyroAngle, accAngle, 0.98);
  curError = rollAngle - balanceAngle;
  
  mSpeed = PID_controller(curError, 3.3, 0.001, 0.5);

  motors.setSpeeds(mSpeed, mSpeed);//PID_controller(inputToController), PID_controller(inputToController));
  
  plotVal();
  preAngle = rollAngle;
}


int32_t PID_controller(int32_t curError, float kP, float kI, float kD){
  int32_t preError, P, I, D, pid_int, output;
  unsigned long  lastReset;
  P = curError;
  
  if ( (curError < 50) && (curError > -50) && (count++ > 1000) ){//(millis() - lastReset == 5000)
    pid_int = 0; 
    lastReset = millis(); 
    }
  I = pid_int + (curError * UpdateInterval);
  
  //I = pid_int;
  D = ((curError - preError)/UpdateInterval );

  output = P*kP+I*kI+D*kD;
  output = output/100;
  output = speedCheck(output);

  preError = curError;
  Serial.print("P:"); Serial.print(P); Serial.print(", ");
  Serial.print("I:"); Serial.print(I); Serial.print(", ");
  Serial.print("D:"); Serial.print(D); Serial.print(", ");
  Serial.print("output:"); Serial.print(output); Serial.print(", ");
  return output;
}

int maxSpeed = 400;
int32_t speedCheck(int32_t input){
  if(input > maxSpeed){ return maxSpeed;}
  else if (input < -maxSpeed){ return -maxSpeed;}
  else{return input;}
  
}



int32_t sensorFusion(int32_t gyroAngle, int32_t accAngle, float ratioGyro){
  // https://wiki.ardumower.de/index.php?title=Arduino_code_techniques
  return (ratioGyro * gyroAngle+ (1-ratioGyro) * accAngle);
}

int delayVal;
void sampleTime(){
  //Restric time interval
  if (millis()-prevMill < UpdateInterval){
    delayVal = UpdateInterval-(millis()-prevMill);
    delay(delayVal);
  }
  curMill = prevMill;
  prevMill = millis();
}

void updateGyroValZero(){
  int32_t tot = 0;
  for (int i = 0 ; i < 100 ; i++){
    imu.read();
    tot += imu.g.y;
    delay(1);
    }
  gyroValZero = tot / 100;
  }

void plotVal(){
 
  Serial.print("rollAngle:"); Serial.print(rollAngle); Serial.print(", ");
  Serial.print("balanceAngle:"); Serial.print(balanceAngle); Serial.print(", ");

  Serial.print("mSpeed:"); Serial.print(mSpeed); Serial.print(", ");
  Serial.println();
}






