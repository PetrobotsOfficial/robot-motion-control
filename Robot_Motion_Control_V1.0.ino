#include <SPI.h>
#include <Wire.h>
#define n1 5
double input_ary[n1];
bool init_flag=false;

// MotorPins
int inAL = 5;
int inBL = 4;
int spL  = 9;
int inAR = 8;
int inBR = 7;
int spR  = 6;
//Encoder
#define encodPinA1  2
#define encodPinA2  A0
#define encodPinB1  3
#define encodPinB2  A1
//goS
bool start_flag = true;
bool angle_register = false;
#define step0 0
#define step1 1
#define step2 2
#define step3 3
byte step=step0;
#define CW 1
#define CCW 0
byte turn_dir;
float motor_yaw;
float turn_target;
unsigned long lastMilli=0;
bool stop_flag=true;
bool stop_pt=false;
float error;
bool pss_mode = false;


//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00
#define INFO             0x01
#define AK8963_ST1       0x02
#define AK8963_XOUT_L   0x03
#define AK8963_XOUT_H  0x04
#define AK8963_YOUT_L  0x05
#define AK8963_YOUT_H  0x06
#define AK8963_ZOUT_L  0x07
#define AK8963_ZOUT_H  0x08
#define AK8963_ST2       0x09
#define AK8963_CNTL      0x0A
#define AK8963_ASTC      0x0C
#define AK8963_I2CDIS    0x0F
#define AK8963_ASAX      0x10
#define AK8963_ASAY      0x11
#define AK8963_ASAZ      0x12

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F
#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F
#define MOT_DUR          0x20
#define ZMOT_THR         0x21
#define ZRMOT_DUR        0x22

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A
#define PWR_MGMT_1       0x6B
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D
#define DMP_RW_PNT       0x6E
#define DMP_REG          0x6F
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

#define ADO 1
#if ADO
#define MPU9250_ADDRESS 0x68
#else
#define MPU9250_ADDRESS 0x68
#define AK8963_ADDRESS 0x0C
#endif

#define AHRS true
#define SerialDebug true

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS;
uint8_t Mmode = 0x02;
float aRes, gRes, mRes;

int intPin = 2;
int myLed = 13;

int16_t accelCount[3];
int16_t gyroCount[3];
int16_t magCount[3];
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
int16_t tempCount;
float   temperature;
float   SelfTest[6];

float GyroMeasError = PI * (40.0f / 180.0f);
float GyroMeasDrift = PI * (0.0f  / 180.0f);

float beta = sqrt(3.0f / 4.0f) * GyroMeasError;
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;
#define Kp 2.0f * 5.0f
#define Ki 0.0f

uint32_t delt_t = 0;
uint32_t count = 0, sumCount = 0;
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;
uint32_t lastUpdate = 0, firstUpdate = 0;
uint32_t Now = 0;

float ax, ay, az, gx, gy, gz, mx, my, mz;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float eInt[3] = {0.0f, 0.0f, 0.0f};
float magBias[3], magScale[3];
bool sf1=1,sf2=1,sf3=1,sf4=1;

#define outer_deadband 24
#define inner_deadband 3.5
#define straight_deadband 10.00
byte Step=1;
int data_count=0;
float angle_ary[8]={180,90,180,90,180,90,180,90};
bool dir_ary[8]=   {CW,CW,CCW,CW,CCW,CW,CCW,CW};
int dist_ary[8]= {5,8,5,8,5,8,5,8};

//float angle_ary[8]={90,180,90,180,90,180,90,180};
//bool dir_ary[8]=   {CW,CCW,CW,CCW,CW,CCW,CW,CCW};

//Encoder Variables
static volatile unsigned long debounce = 0;
static volatile unsigned long debounce1 = 0;
int channelA;
int channelB;
int change_left=0,change_right=0;
int input_left=0;
volatile unsigned long last_input_left=0;
 int input_right=0;
volatile unsigned long last_input_right=0;
double x=0,y=0,psi=0;
const double robot_length=12;
const double wheel_diameter=6.75; //cm
#define total_ticks 2000
const double Pi=3.141592654;
float distance=0;
float last_x=0;

void setup() {
  setup_pin();
}

void loop() {
  mpu9250();
  if (!AHRS) {
    delt_t = millis() - count;
    if (delt_t > 500) count = millis();
  }
  else {
    delt_t = millis() - count;
    if (delt_t > 50) {
      if (SerialDebug) {
        yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        yaw   *= 180.0f / PI;
        yaw   += 0.12; 
        motor_yaw = yaw + 180;
//        motor_yaw = filter_average(input_ary,motor_yaw);
        Serial.print("Yaw: ");
        Serial.print( motor_yaw, 2);
        Serial.print('\t');

        count = millis();
        sumCount = 0;
        sum = 0;  
//        Serial.print(Step); Serial.print('\t');
//        Serial.print("TArget Angle : "); Serial.print(angle_ary[data_count]);Serial.print('\t');
        
        switch(Step){
          case 1 : {general_turn(angle_ary[data_count]);break;}
          case 2 : {general_straight(angle_ary[data_count],dist_ary[data_count]);break;}
        }
        Serial.println();     
      }
    }
  }
}

void mpu9250(){
    if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    readAccelData(accelCount);
    getAres();

    ax = (float)accelCount[0] * aRes - accelBias[0];
    ay = (float)accelCount[1] * aRes - accelBias[1];
    az = (float)accelCount[2] * aRes - accelBias[2];

    readGyroData(gyroCount);
    getGres();

    gx = (float)gyroCount[0] * gRes;
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;

    readMagData(magCount);
    getMres();

    mx = (float)magCount[0] * mRes * magCalibration[0] - magBias[0];
    my = (float)magCount[1] * mRes * magCalibration[1] - magBias[1];
    mz = (float)magCount[2] * mRes * magCalibration[2] - magBias[2];
  }

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f);
  lastUpdate = Now;
  sum += deltat;
  sumCount++;
  MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f,  my,  mx, mz);
}

void setup_pin(){
  pinMode(inAL, OUTPUT);
  pinMode(inBL, OUTPUT);
  pinMode(spL, OUTPUT);

  pinMode(inAR, OUTPUT);
  pinMode(inBR, OUTPUT);
  pinMode(spR, OUTPUT);
  
  Wire.begin();
  Serial.begin(115200);


  Serial.println("MPU9250");
  Serial.println("9-DOF 16-bit");
  Serial.println("motion sensor");
  Serial.println("60 ug LSB");
  delay(800);

  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 ");
  Serial.print("I AM ");
  Serial.print(c, HEX);
  Serial.print(" I should be ");
  Serial.println(0x71, HEX);
  delay(800);

  if (c == 0x71)
  {
    Serial.println("MPU9250 is online...");
    calibrateMPU9250(gyroBias, accelBias);
    initMPU9250();
    Serial.println("MPU9250 initialized for active data mode....");
    byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
    delay(1000);

    initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode....");
    getMres();
    magcalMPU9250(magBias, magScale);


    if (SerialDebug) {
      Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
    }

    Serial.println("AK8963");
    Serial.println("ASAX ");
    Serial.println(magCalibration[0], 2);
    Serial.println("ASAY ");
    Serial.println(magCalibration[1], 2);
    Serial.println("ASAZ ");
    Serial.println(magCalibration[2], 2);
    delay(1000);
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while (1) ;
  }
  pinMode(encodPinA1, INPUT);
  pinMode(encodPinA2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encodPinA1), counter, RISING); 
  pinMode(encodPinB1, INPUT);
  pinMode(encodPinB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encodPinB1), counter1, RISING);
}

float aggTurnPid(float turn_target, float current_angle)   {          
  const float KpT=1.6;
  float pidTerm = 0;                                                      
  float error = 0;                                                           
  error = abs(current_angle - turn_target);
  pidTerm = error*KpT; 
  return pidTerm;
}

float pssTurnPid(float turn_target, float current_angle)   {          
  const float KiT=5.00;
  float pidTerm = 0;                                                      
  float error = 0;                                                           
  static float last_error = 0;
  error = abs(current_angle - turn_target);
  float errSum = error + last_error;
  pidTerm = errSum/KiT;
//  Serial.print(pidTerm);Serial.print('\t');  
  last_error=error;
  return pidTerm;
}

bool millis_delay(long last_time,long time_limit){
  if (millis()-last_time>time_limit)return false; //finish delay
  else return true;
}

void general_straight(float target_angle, int target_dist){
  float yaw_error = motor_yaw - target_angle;
  static unsigned long straight_time=0;
  static unsigned long stop_time=0;
  static bool straight_flag=true;
  static bool stop_flag =false;
  static byte mini_step=1;
  int left_base=90;
  int right_base=165;
  
  encoder();
  distance = x;
  if (mini_step==1){
    if(distance < target_dist){//move
      if (abs(yaw_error)<=straight_deadband) motor(1,left_base,1,right_base);
      else if (yaw_error>straight_deadband)  motor(1,left_base,0,right_base);
      else motor(0,left_base,1,right_base);    
      straight_flag=false;
      Serial.print("Moving Straight...");
    }  
    else{
      stop_flag=true;
      mini_step++;
    }
  }
  else if (mini_step==2){
  if(stop_flag) stop_time=millis();
  if(millis_delay(stop_time,1500)){
    motor(0);    
    stop_flag=false;
    Serial.print("Stop!...");    
  }  
  else{
    stop_time=0;
    Step= 1;
    mini_step--;
    data_count++;
    calc_distance();
  }
  }
}

void general_turn( float turn_angle){
  int pwm;
  static float noob_pss_pwm_corr=0;
  float agg_pwm_corr;
  static unsigned long stop_time_turn=0;
  static bool turn_stop_flag=true;
//  if(!angle_register) {
//    if (turn_dir==CCW) turn_target = turn_target + turn_angle;
//    else turn_target = turn_target - turn_angle;
//    Serial.print("Registering angle...");
//    Serial.println(turn_target);
//    angle_register = true;
//  }
  float error = motor_yaw - turn_angle;
  Serial.print("turn_angle : ");Serial.print(turn_angle);Serial.print('\t');
  if (abs(error) > outer_deadband){
    Serial.print("aggresive");        Serial.print('\t');
    agg_pwm_corr=aggTurnPid(turn_angle,motor_yaw);
    noob_pss_pwm_corr = 0;
  }
  else if (abs(error) > inner_deadband) {
      noob_pss_pwm_corr += 4.5;
      Serial.print("Passive");         Serial.print('\t');
      Serial.print(noob_pss_pwm_corr); Serial.print('\t');
      agg_pwm_corr = 0;
  }
  else{
      agg_pwm_corr = 0;
      noob_pss_pwm_corr = 0;
  }
  
  pwm = agg_pwm_corr + noob_pss_pwm_corr;
  pwm=constrain(pwm,0,180);
  Serial.print(pwm);        Serial.print('\t');

  if(abs(error) > inner_deadband){
    if(motor_yaw<turn_angle)motor(0,pwm,1,pwm+40);
    else motor(1,pwm,0,pwm+40);
    turn_stop_flag=true;
  }
  else {
    Serial.println("Reach point!");
    motor(0);
    if(turn_stop_flag) stop_time_turn=millis();
    if(millis_delay(stop_time_turn,600)){
      motor(0);    
      turn_stop_flag=false;
      Serial.print("Stop!...");
    }
    else {
      calc_distance();
      Step=2;          
    }
  }
}

//encoder_data_initiation
void encoder(){
  static unsigned long lastMilli_encoder=0;
  const int sample_time_encoder=100;
    if (!millis_delay(lastMilli_encoder,sample_time_encoder)){
      noInterrupts();
      lastMilli_encoder = millis();
      compute();
      print_result_encoder();    
      interrupts();
    }
}
void compute(){
  change_left =input_left -last_input_left ;
  last_input_left =input_left;
  change_right=input_right-last_input_right;
  last_input_right=input_right;
  double s_left =get_curve_length(change_left) ;
  double s_right=get_curve_length(change_right);
  psi+=(s_right-s_left)/(robot_length); 
  double distance=(s_left+s_right)/2;
  double new_psi=psi;
  if(psi<0) new_psi=2*Pi-psi;
    x += distance*cos(new_psi); 
    y += distance*sin(new_psi);
}
void print_result_encoder(){
//  Serial.print(change_left); Serial.print("\t");
//  Serial.print(change_right);  Serial.print("\t");
  Serial.print("x: "); Serial.print(x,4); Serial.print("\t");
//  Serial.print("y"); Serial.print(y,4); Serial.print("\t");
  Serial.print((psi/Pi)*180.0);  Serial.print("\t");
}

//distance_count
void calc_distance(){
  static bool init_encoder_flag=true;
  static bool end_encoder_flag=false;
  if(init_encoder_flag) {
    reset_dis();
    init_encoder_flag=false;
    end_encoder_flag=true;
    Serial.println("INIT INIT INIT INIT INIT INIT ");
  }
  else if(end_encoder_flag) {
//  distance = x;
  distance = y;
  Serial.print("Distance: ");Serial.print(distance);Serial.print('\t');
//  Serial.print("Last_X: ");Serial.print(last_x);Serial.print('\t');
//  Serial.print("X: ");Serial.print(x);Serial.print('\t');
  Serial.println();
  init_encoder_flag=true;
  end_encoder_flag=false;
  }
}
void reset_dis(){
  change_left=0,change_right=0;
  input_left=0;
  last_input_left=0;
  input_right=0;
  last_input_right=0;
  x=0,y=0,psi=0;
}
void counter() {
  if (micros() - debounce > 100){
    debounce = micros();
    channelA=digitalRead(encodPinA2);
     if(channelA==1) input_left++;
     else if(channelA==0)input_left--;
  } 
}
void counter1() {
  if (micros() - debounce1 > 100){
    debounce1 = micros();
    channelB=digitalRead(encodPinB2);
    if(channelB==1) input_right--;
    else if(channelB==0) input_right++;
  } 
}
double get_curve_length(int n){
 return ((float)n/(float)total_ticks)*2.0*Pi*(wheel_diameter/2);
}

//MPU
void getMres() {
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
      break;
  }
}

void getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }
}

void getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
    }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] =  (float)(rawData[2] - 128) / 256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}


void initMPU9250()
{

  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
  //  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x02); // Clear Fchoice bits [1:0]
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
  // writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG

  // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
  //  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}

void calibrateMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80);
  delay(100);

  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00);
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C); //fifo_reset
  delay(15);

  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);

  uint16_t  gyrosensitivity  = 131;
  uint16_t  accelsensitivity = 16384;

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging
//  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C); //fifo reset

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++) {
    if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for display in the main program
  dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
  dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
  dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;


}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
  float factoryTrim[6];
  uint8_t FS = 0;

  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

  for ( int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  for ( int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
  delay(25);  // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    destination[i]   = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]; // Report percent differences
    destination[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3]; // Report percent differences
  }

}


// Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}


void motor(bool dir_l, int spd_l,bool dir_r, int spd_r){  //
  digitalWrite(inAL, !dir_l);
  digitalWrite(inBL, dir_l);
  analogWrite(spL, spd_l);
  digitalWrite(inAR, !dir_r);
  digitalWrite(inBR, dir_r);
  analogWrite(spR, spd_r);
}
void motor(bool Stop){
  digitalWrite(inAL, Stop);
  digitalWrite(inBL, Stop);
  analogWrite(spL, 0);
  digitalWrite(inAR, Stop);
  digitalWrite(inBR, Stop);
  analogWrite(spR, 0);
}
//void MotorStraight(){
//  digitalWrite(inAL, LOW);
//  digitalWrite(inBL, HIGH);
//  analogWrite(spL, 60);
//  digitalWrite(inAR, LOW);
//  digitalWrite(inBR, HIGH);
//  analogWrite(spR, 60);
//
//}
//
//void MotorLeft(int pwm){
//  digitalWrite(inAL, HIGH);
//  digitalWrite(inBL, LOW);
//  analogWrite(spL, pwm);
//  digitalWrite(inAR, LOW);
//  digitalWrite(inBR, HIGH);
//  analogWrite(spR, pwm);
//
//}
//void MotorRight(int pwm){
//  digitalWrite(inAL, LOW);
//  digitalWrite(inBL, HIGH);
//  analogWrite(spL, pwm);
//  digitalWrite(inAR, HIGH);
//  digitalWrite(inBR, LOW);
//  analogWrite(spR, pwm);
//}
//
//void stop(){
//  digitalWrite(inAL, LOW);
//  digitalWrite(inBL, LOW);
//  analogWrite(spL, 0);
//  digitalWrite(inAR,LOW);
//  digitalWrite(inBR, LOW);
//  analogWrite(spR, 0);
//}


void swap(double &a,double &b){
  double temp = a;
  a=b;
  b=temp;
}

double filter_average(double ary[n1],double new_value){
  if (!init_flag){//Initialization to prevent low value at start
    for(int i=0;i<n1;i++) ary[i]=new_value;
    init_flag=true;
  }
  double sum=0;
  for(int i=0;i<n1-1;i++)swap(ary[i],ary[i+1]);
  ary[n1-1]=new_value;
  for(int i=0;i<n1;i++)sum+=ary[i];
  return (double)sum/(double)n1;
}
