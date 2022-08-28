#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial Serial1(3,4);                        // SLAVE_1:(4,5) SLAVE_2:(6,7) SLAVE_3:(8,9) SLAVE_4:(10,11)

// positions in arrays 
#define PITCH 0                                                   
#define ROLL 1                                     
#define YAW 2                                       

#define MPU_ADDRESS 0x68

#define MOTOR_PIN1 8
#define MOTOR_PIN2 9
#define MOTOR_PIN3 7
#define MOTOR_PIN4 6

Servo motor_1;                                      // Motor front right
Servo motor_2;                                      // Motor front left
Servo motor_3;                                      // Motor back left
Servo motor_4;                                      // Motor back right

int THROTTLE_MINIMUM = 1000;                       
int THROTTLE_MAXIMUM = 1800;                       
int THROTTLE_HOVER = 1410;

float COMPLEMENTARY_FILTER = 0.98;                  
float filter = 0.8;     

double throttle = 1000;                       
float angle_desired[3] = {0.0, 0.0, 0.0};           

float gain_p[3] = {4.8, 4.8, 3};                
float gain_d[3] = {0.5, 0.5, 0.5};              

float error_current[3] = {0, 0, 0};                
float error_prev[3] = {0, 0, 0};               

float pid_current[3] = {0, 0, 0};                   
float pid_p[3] = {0, 0, 0};                        
float pid_d[3] = {0, 0, 0};                        

float angle_current[3];                            
float angle_acc[3];                                
float angle_gyro[3];                               
float angle_acc_offset[3] = {0.0, 0.0, 0.0};       
float angle_gyro_offset[3] = {0.0, 0.0, 0.0};      

float angle_acc_raw[3];                            
int16_t angle_gyro_raw[3];                         

float time_current;                                
float time_prev;                                   
double time_elapsed;                                

float rad_to_deg = 180 / 3.141592654;        
float acc_division = 16384.0;    
float gyro_division = = 131.0;  

void setup() {
  Serial1.begin(9600);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1A);
  Wire.write(0x06);
  Wire.endTransmission();

  time_current = millis();

  motor_1.attach(MOTOR_PIN1);
  motor_2.attach(MOTOR_PIN2);
  motor_3.attach(MOTOR_PIN3);
  motor_4.attach(MOTOR_PIN4);

  setSpeedForAllMotors(THROTTLE_MINIMUM);
  delay(3000);                                                  // delay for the motor fail-safe
  calibrateGyroOffset();                                        // gyro calibration for level starting point
  delay(3000);                                                  // delay for the motor fail-safe
}

void loop() {
  if (Serial1.available() > 0) {
    char command = Serial1.read();
    if (command == 'u') {
      throttle += 5;                                             // Move up
      if (throttle >= THROTTLE_MAXIMUM) {
        throttle = THROTTLE_MAXIMUM;
      }
    }
    else if (command == 'd') {
      throttle -= 5;                                             // Move down
      if (throttle <= THROTTLE_MINIMUM) {
        throttle = THROTTLE_MINIMUM;
      }
    }
    else if (command == 's') {
      throttle = THROTTLE_MINIMUM;                               // Turn off all motors
    } 
    else if (command == 't') {
      throttle += 25;                                            // Start take off      
    }
    else if (command == 'r') {
      angle_desired[ROLL] = angle_desired[ROLL] + 1;              // Move right
    } 
    else if (command == 'l') {
      angle_desired[ROLL] = angle_desired[ROLL] - 1;              // Move left
    } 
    else if (command == 'f') {
      angle_desired[PITCH] = angle_desired[PITCH] + 1;            // Move forward
    } 
    else if (command == 'b') {
      angle_desired[PITCH] = angle_desired[PITCH] - 1;            // Move back
    }
    else if (command == 'h') {                                 // hover point
      throttle = THROTTLE_HOVER;            
    } 
  }

  time_prev = time_current;
  time_current = millis();
  time_elapsed = (time_current - time_prev) / 1000;

  readGyro();
  readAcc();
  filterAngle();

  calculatePid();
  if (throttle > 1010) {
    setMotorPids();
  } 
  else {
    setSpeedForAllMotors(THROTTLE_MINIMUM);
  }
}

void calculatePid() {
  error_prev[PITCH] = error_current[PITCH];
  error_prev[ROLL] = error_current[ROLL];
  error_prev[YAW] = error_current[YAW];

  error_current[PITCH] = angle_current[PITCH] - angle_desired[PITCH];
  error_current[ROLL] = angle_current[ROLL] - angle_desired[ROLL];
  error_current[YAW] = angle_current[YAW] - angle_desired[YAW];

  pid_p[PITCH] = gain_p[PITCH] * error_current[PITCH];
  pid_p[ROLL] = gain_p[ROLL] * error_current[ROLL];
  pid_p[YAW] = gain_p[YAW] * error_current[YAW];

  float pid_d_new[3];

  pid_d_new[PITCH] = gain_d[PITCH] * (error_current[PITCH] - error_prev[PITCH]) / time_elapsed;
  pid_d_new[ROLL] = gain_d[ROLL] * (error_current[ROLL] - error_prev[ROLL]) / time_elapsed;
  pid_d_new[YAW] = gain_d[YAW] * (error_current[YAW] - error_prev[YAW]) / time_elapsed;

  pid_d[PITCH] = filter * pid_d[PITCH] + (1 - filter) * pid_d_new[PITCH];
  pid_d[ROLL] = filter * pid_d[ROLL] + (1 - filter) * pid_d_new[ROLL];
  pid_d[YAW] = filter * pid_d[YAW] + (1 - filter) * pid_d_new[YAW];

  pid_current[PITCH] = pid_p[PITCH] + pid_d[PITCH];
  pid_current[ROLL] = pid_p[ROLL] + pid_d[ROLL];
  pid_current[YAW] = pid_p[YAW] + pid_d[YAW];
}

void setMotorPids() {
  motor_1.writeMicroseconds(throttle + pid_current[PITCH] + pid_current[ROLL] + pid_current[YAW]);      // Set PID for front right motor
  motor_3.writeMicroseconds(throttle - pid_current[PITCH] - pid_current[ROLL] + pid_current[YAW]);      // Set PID for back left motor
  motor_2.writeMicroseconds(throttle + pid_current[PITCH] - pid_current[ROLL] - pid_current[YAW]);      // Set PID for front left motor
  motor_4.writeMicroseconds(throttle - pid_current[PITCH] + pid_current[ROLL] - pid_current[YAW]);      // Set PID for back right motor
}

void setSpeedForAllMotors(double value) {
  motor_1.writeMicroseconds(value);
  motor_2.writeMicroseconds(value);
  motor_3.writeMicroseconds(value);
  motor_4.writeMicroseconds(value);
}

void filterAngle() {
  float angle_new[3];

  angle_new[PITCH] = -(COMPLEMENTARY_FILTER * (-angle_current[PITCH] + angle_gyro[PITCH] * time_elapsed) + (1 - COMPLEMENTARY_FILTER) * angle_acc[PITCH]);   
  angle_new[ROLL] = COMPLEMENTARY_FILTER * (angle_current[ROLL] + angle_gyro[ROLL] * time_elapsed) + (1 - COMPLEMENTARY_FILTER) * angle_acc[ROLL];            
  angle_new[YAW] = COMPLEMENTARY_FILTER * (angle_current[YAW] + angle_gyro[YAW] * time_elapsed) + (1 - COMPLEMENTARY_FILTER) * angle_acc[YAW];                

  float value = 0.5; 

  angle_current[PITCH] = value * angle_current[PITCH] + (1 - value) * angle_new[PITCH];
  angle_current[ROLL] = value * angle_current[ROLL] + (1 - value) * angle_new[ROLL];
  angle_current[YAW] = value * angle_current[YAW] + (1 - value) * angle_new[YAW];
}

void readGyro() {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true);

  angle_gyro_raw[PITCH] = Wire.read() << 8 | Wire.read();
  angle_gyro_raw[ROLL] = Wire.read() << 8 | Wire.read();
  angle_gyro_raw[YAW] = Wire.read() << 8 | Wire.read();     

  angle_gyro[PITCH] = angle_gyro_raw[PITCH] / gyro_division;
  angle_gyro[ROLL] = angle_gyro_raw[ROLL] / gyro_division;
  angle_gyro[YAW] = angle_gyro_raw[YAW] / gyro_division;          

  angle_gyro[PITCH] = angle_gyro[PITCH] - angle_gyro_offset[PITCH];
  angle_gyro[ROLL] = angle_gyro[ROLL] - angle_gyro_offset[ROLL];
  angle_gyro[YAW] = angle_gyro[YAW] - angle_gyro_offset[YAW];
}

void readAcc() {

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true);

  angle_acc_raw[PITCH] = (Wire.read() << 8 | Wire.read()) / acc_division;
  angle_acc_raw[ROLL] = (Wire.read() << 8 | Wire.read()) / acc_division;
  angle_acc_raw[YAW] = (Wire.read() << 8 | Wire.read()) / acc_division;

  angle_acc[PITCH] = atan(angle_acc_raw[ROLL] / sqrt(pow(angle_acc_raw[PITCH], 2) + pow(angle_acc_raw[YAW], 2))) * rad_to_deg;
  angle_acc[ROLL] = atan(-1 * angle_acc_raw[PITCH] / sqrt(pow(angle_acc_raw[ROLL], 2) + pow(angle_acc_raw[YAW], 2))) * rad_to_deg;
  angle_acc[YAW] = atan(angle_acc_raw[PITCH] / angle_acc_raw[ROLL]);

  angle_acc[PITCH] = angle_acc[PITCH] - angle_acc_offset[PITCH];
  angle_acc[ROLL] = angle_acc[ROLL] - angle_acc_offset[ROLL];
  angle_acc[YAW] = angle_acc[YAW] - angle_acc_offset[YAW];
}

void calibrateGyroOffset() {
  float num = 2000.0;
  float gyro_avg[3] = {0.0, 0.0, 0.0};
  float acc_avg[3] = {0.0, 0.0, 0.0};
  
  // calculate average of gyro values to calibrate
  for (int i = 0; i < num; i++) {
    readGyro();
    gyro_avg[PITCH] += angle_gyro[PITCH];
    gyro_avg[ROLL] += angle_gyro[ROLL];
    gyro_avg[YAW] += angle_gyro[YAW];

    readAcc();
    acc_avg[PITCH] += angle_acc[PITCH];
    acc_avg[ROLL] += angle_acc[ROLL];
    acc_avg[YAW] += angle_acc[YAW];
  }

  angle_gyro_offset[PITCH] = gyro_avg[PITCH] / num;
  angle_gyro_offset[ROLL] = gyro_avg[ROLL] / num;
  angle_gyro_offset[YAW] = gyro_avg[YAW] / num;
  angle_acc_offset[PITCH] = acc_avg[PITCH] / num;
  angle_acc_offset[ROLL] = acc_avg[ROLL] / num;
  angle_acc_offset[YAW] = acc_avg[YAW] / num;
}
