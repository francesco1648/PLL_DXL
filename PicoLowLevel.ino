
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "AbsoluteEncoder.h"
#include "Battery.h"
#include "DynamixelSerial.h"
#include "TractionEncoder.h"
#include "MovingAvgFilter.h"
#include "ExpSmoothingFilter.h"
#include "Debug.h"
#include "mcp2515.h"
#include "Display.h"
#include "SmartMotor.h"
#include "Motor.h"
#include "PID.h"
#include "CanWrapper.h"

#include "include/definitions.h"
#include "include/mod_config.h"
#include "include/communication.h"

#include "Dynamixel_ll.h"



void okInterrupt();
void navInterrupt();
void sendFeedback();
void handleSetpoint(uint8_t msg_id, const byte *msg_data);

int time_bat = 0;
int time_tel = 0;
int time_data = 0;
int time_tel_avg = DT_TEL;

CanWrapper canW(5, 10000000UL, &SPI);
//------
float theta_dxl;
float phi_dxl;
int32_t valueToSend = 0;

const uint8_t motorIDs[] = {210, 211};
const uint8_t numMotors = sizeof(motorIDs) / sizeof(motorIDs[0]);

int32_t pos0_mot_2 = 0;
int32_t pos0_mot_3 = 0;
int32_t pos0_mot_4 = 0;
int32_t pos0_mot_5 = 0;
int32_t pos0_mot_6 = 0;
int32_t getpositions0[2] = {0, 0}; // Initialize positions to 0

int32_t pos_mot_2 = 0;
int32_t pos_mot_3 = 0;
int32_t pos_mot_4 = 0;
int32_t pos_mot_5 = 0;
int32_t pos_mot_6 = 0;
int32_t getpositions[2] = {0, 0}; // Initialize positions to 0


  float servo_data_1a=0.0f;
  float servo_data_1b=0.0f;
  float servo_data_float=0.0f;

#define ProfileAcceleration 10
#define ProfileVelocity 20

//------
SmartMotor motorTrLeft(DRV_TR_LEFT_PWM, DRV_TR_LEFT_DIR, ENC_TR_LEFT_A, ENC_TR_LEFT_B, false);
SmartMotor motorTrRight(DRV_TR_RIGHT_PWM, DRV_TR_RIGHT_DIR, ENC_TR_RIGHT_A, ENC_TR_RIGHT_B, true);

#ifdef MODC_YAW
AbsoluteEncoder encoderYaw(ABSOLUTE_ENCODER_ADDRESS);
#endif

#ifdef MODC_EE
DynamixelMotor motorEEPitch(SERVO_EE_PITCH_ID);
DynamixelMotor motorEEHeadPitch(SERVO_EE_HEAD_PITCH_ID);
DynamixelMotor motorEEHeadRoll(SERVO_EE_HEAD_ROLL_ID);
#endif

#ifdef MODC_ARM
DynamixelLL dxl(Serial1, 0);
DynamixelLL mot_Left_1(Serial1, motorIDs[0]);
DynamixelLL mot_Right_1(Serial1, motorIDs[1]);
DynamixelLL mot_2(Serial1, 112);  // ID = 3
DynamixelLL mot_3(Serial1, 113);  // ID = 4
DynamixelLL mot_4(Serial1, 214);  // ID = 5
DynamixelLL mot_5(Serial1, 215);  // ID = 6
DynamixelLL mot_6(Serial1, 216);


#endif

#ifdef MODC_JOINT
// Motor IDs for the two motors.
const uint8_t motorIDs[] = {SERVO_JOINT_1d_PITCH_ID, SERVO_JOINT_1s_PITCH_ID};
const uint8_t numMotors = sizeof(motorIDs) / sizeof(motorIDs[0]);

DynamixelLL dxlJOINT(Serial1, 0); // an instance for syncWrite (ID not used).
DynamixelLL motorJOINT1dPitch(Serial1, SERVO_JOINT_1d_PITCH_ID);
DynamixelLL motorJOINT1sPitch(Serial1, SERVO_JOINT_1s_PITCH_ID);
DynamixelLL motorJOINT2Roll(Serial1, SERVO_JOINT_2_ROLL_ID);
#endif

// WebManagement wm(CONF_PATH);

Display display;

void setup()
{
  Serial.begin(115200);
  Debug.setLevel(Levels::INFO); // comment to set debug verbosity to debug
  Wire1.setSDA(I2C_SENS_SDA);
  Wire1.setSCL(I2C_SENS_SCL);
  Wire1.begin();

  SPI.setRX(4);
  SPI.setCS(5);
  SPI.setSCK(6);
  SPI.setTX(7);
  SPI.begin();

  // LittleFS.begin();

  String hostname = WIFI_HOSTBASE + String(CAN_ID);
  // wm.begin(WIFI_SSID, WIFI_PWD, hostname.c_str());

  // CAN initialization
  canW.begin();

  // initializing PWM
  analogWriteFreq(PWM_FREQUENCY);  // switching frequency to 15kHz
  analogWriteRange(PWM_MAX_VALUE); // analogWrite range from 0 to 512, default is 255

  // initializing ADC
  analogReadResolution(12); // set precision to 12 bits, 0-4095 input

  // motor initialization
  motorTrLeft.begin();
  motorTrRight.begin();

  motorTrLeft.calibrate();
  motorTrRight.calibrate();

  #if defined MODC_EE
    Serial1.setRX(1);
    Serial1.setTX(0);
    Dynamixel.setSerial(&Serial1);
    Dynamixel.begin(19200);
  #endif

  Debug.println("BEGIN", Levels::INFO);

#ifdef MODC_YAW
  encoderYaw.update();
  encoderYaw.readAngle();
  encoderYaw.setZero();
#endif

#ifdef MODC_ARM
Serial1.setTX(0);
  Serial1.setRX(1);
  dxl.begin(1000000);
  mot_Left_1.begin(1000000);
  mot_Right_1.begin(1000000);
  mot_2.begin(1000000);
  mot_3.begin(1000000);
  mot_4.begin(1000000);
  mot_5.begin(1000000);
  mot_6.begin(1000000);

  mot_Right_1.setTorqueEnable(false); // Disable torque for safety
  mot_Left_1.setTorqueEnable(false); // Disable torque for safety
  mot_2.setTorqueEnable(false); // Disable torque for safety
  mot_3.setTorqueEnable(false); // Disable torque for safety
  mot_4.setTorqueEnable(false); // Disable torque for safety
  mot_5.setTorqueEnable(false); // Disable torque for safety
  mot_6.setTorqueEnable(false);

   delay(10);
  mot_6.setStatusReturnLevel(2);

  // Set the operating mode to Position Control Mode (Mode 3).
  mot_Left_1.setOperatingMode(4);
  mot_Right_1.setOperatingMode(4);
  mot_2.setOperatingMode(4);
  mot_3.setOperatingMode(4);
  mot_4.setOperatingMode(4);
  mot_5.setOperatingMode(4);
  mot_6.setOperatingMode(4);

  delay(10);
  // Initialize a known present position for troubleshooting.
  //homingOffset[0] = 500;
  //homingOffset[1] = 1500;
  getpositions[0] = 0;
  getpositions[1] = 0;
  //getLoads[0] = 0;
 // getLoads[1] = 0;

  // Enable or disable debug mode for troubleshooting
  mot_Left_1.setDebug(false);
  mot_Right_1.setDebug(false);
  dxl.setDebug(false);

  // Factory Reset and Reboot
  //mot_Left_1.factoryReset(0x02);
  //delay(5000);
  //mot_Left_1.reboot();
  //delay(3000);
  //mot_Right_1.factoryReset(0x02);
  //delay(5000);
  //mot_Right_1.reboot();
  //delay(3000);

  // Enable sync mode for multiple motor control.
  dxl.enableSync(motorIDs, numMotors);

  // Configure Drive Mode for each motor:
  mot_Left_1.setDriveMode(false, false, false);
  mot_Right_1.setDriveMode(false, false, false);
  mot_2.setDriveMode(false, false, false);
  mot_3.setDriveMode(false, false, false);
  mot_4.setDriveMode(false, false, false);
  mot_5.setDriveMode(false, false, false);
  mot_6.setDriveMode(false, false, false);


  // Set Operating Mode for each motor:
  dxl.setOperatingMode(4); // Extended Position Mode
  mot_2.setOperatingMode(4);
  mot_3.setOperatingMode(4);
  mot_4.setOperatingMode(4);
  mot_5.setOperatingMode(4);
  mot_6.setOperatingMode(4);

  // Set Homing Offset for each motor:
  //dxl.setHomingOffset(homingOffset);

  // Enable torque for both motors.
 /* dxl.setTorqueEnable(true);
 mot_2.setTorqueEnable(true);
  mot_3.setTorqueEnable(true);
  mot_4.setTorqueEnable(true);
  mot_5.setTorqueEnable(true);
  mot_6.setTorqueEnable(true);
 */
/*
mot1a 1780  mot1b 2957
mot2 2122
mot3 -1951
mot4 1159
mot5 5164
mot6 -1098
*/



  delay(10);
  // Set Profile Velocity and Profile Acceleration for smooth motion.
mot_Left_1.setProfileVelocity(ProfileVelocity);
mot_Left_1.setProfileAcceleration(ProfileAcceleration);
mot_Right_1.setProfileVelocity(ProfileVelocity);
mot_Right_1.setProfileAcceleration(ProfileAcceleration);
mot_2.setProfileVelocity(ProfileVelocity);
mot_2.setProfileAcceleration(ProfileAcceleration);
mot_3.setProfileVelocity(ProfileVelocity);
mot_3.setProfileAcceleration(ProfileAcceleration);
mot_4.setProfileVelocity(ProfileVelocity);
mot_4.setProfileAcceleration(ProfileAcceleration);
mot_5.setProfileVelocity(ProfileVelocity);
mot_5.setProfileAcceleration(ProfileAcceleration);
mot_6.setProfileVelocity(ProfileVelocity);
mot_6.setProfileAcceleration(ProfileAcceleration);




/*getpositions0[0] = 1780; // Initialize positions to 0
getpositions0[1] = 2957; // Initialize positions to 0
 dxl.setHomingOffset(getpositions); // Set homing offset to 0 for all motors
  mot_2.setHomingOffset(2122);
  mot_3.setHomingOffset(-1951);
  mot_4.setHomingOffset(1159);
  mot_5.setHomingOffset(5164);
  mot_6.setHomingOffset(-1098);*/

  // Enable torque for all motors.
  dxl.setTorqueEnable(true);
  mot_Left_1.setTorqueEnable(true);
  mot_Right_1.setTorqueEnable(true);
  mot_2.setTorqueEnable(true);
  mot_3.setTorqueEnable(true);
  mot_4.setTorqueEnable(true);
  mot_5.setTorqueEnable(true);
  mot_6.setTorqueEnable(true);


    getpositions0[0] = 1766 ; // Initialize positions to 0
  getpositions0[1] = 2763; // Initialize positions to 0
  pos0_mot_2 =4524;
  pos0_mot_3 = -52;
  pos0_mot_4 = 3255;
  pos0_mot_5 = 7308;
  pos0_mot_6 = -915; // Initialize positions to 0

dxl.setGoalPosition_EPCM(getpositions0);
mot_2.setGoalPosition_EPCM(pos0_mot_2);  // Address 65, Value 1, Size 1 byte
mot_3.setGoalPosition_EPCM(pos0_mot_3);  // Address 65, Value 1, Size 1 byte
mot_4.setGoalPosition_EPCM(pos0_mot_4);  // Address 65, Value 1, Size 1 byte
mot_5.setGoalPosition_EPCM(pos0_mot_5);  // Address 65, Value 1, Size 1 byte
mot_6.setGoalPosition_EPCM(pos0_mot_6);  // Address 65, Value 1, Size 1 byte
#endif

#ifdef MODC_JOINT
  Serial1.setTX(0);
  Serial1.setRX(1);
  dxlJOINT.begin(57600);
  dxlJOINT.enableSync(motorIDs, numMotors);

  // Set operating mode for all motors to position mode
  dxlJOINT.setOperatingMode(3);
  motorJOINT2Roll.setOperatingMode(3);

  // Declare and initialize the arrays
  uint32_t pos_1d1s[numMotors] = {0, 0};
  uint32_t pos_2 = 0;

  // Get present position for all motors
  dxlJOINT.getPresentPosition(pos_1d1s);
  motorJOINT2Roll.getPresentPosition(pos_2);

  // Set homing offset for all motors based on their present positions
  // first, convert data types to float
  dxlJOINT.setHomingOffset({static_cast<float>(pos_1d1s[0]), static_cast<float>(pos_1d1s[1])});
  motorJOINT2Roll.setHomingOffset(static_cast<float>(pos_2));

  // Configure Torque Mode for each motor:
  dxlJOINT.setTorqueEnable(true);
  motorJOINT2Roll.setTorqueEnable(true);
#endif

  // Display initialization
  display.begin();

  // Buttons initialization
  pinMode(BTNOK, INPUT_PULLUP);
  pinMode(BTNNAV, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTNOK), okInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTNNAV), navInterrupt, FALLING);
}

void loop()
{
 int time_cur = millis();
  uint8_t msg_id;
  byte msg_data[8];

  // update motors
  motorTrLeft.update();
  motorTrRight.update();

  // health checks
  if (time_cur - time_bat >= DT_BAT)
  {
    time_bat = time_cur;

    if (time_tel_avg > DT_TEL)
      Debug.println("Telemetry frequency below required: " + String(1000 / time_tel_avg) + " Hz", Levels::WARN);

    if (!battery.charged())
      Debug.println("Battery voltage low! " + String(battery.readVoltage()) + "v", Levels::WARN);
  }

  // send telemetry
  if (time_cur - time_tel >= DT_TEL)
  {
    time_tel_avg = (time_tel_avg + (time_cur - time_tel)) / 2;
    time_tel = time_cur;

    sendFeedback();
  }

  if (canW.readMessage(&msg_id, msg_data))
  {

    // Received CAN message with setpoint
    time_data = time_cur;
    handleSetpoint(msg_id, msg_data);
  }
  else if (time_cur - time_data > CAN_TIMEOUT && time_data != -1)
  {
    // if we do not receive data for more than a second stop motors
    time_data = -1;
    Debug.println("Stopping motors after timeout.", Levels::INFO);
    //motorTrLeft.stop();
    //motorTrRight.stop();
  }

  // wm.handle();
  display.handleGUI();



}



/**
 * @brief Handles the setpoint messages received via CAN bus.
 * @param msg_id ID of the received message.
 * @param msg_data Pointer to the message data.
 */
void handleSetpoint(uint8_t msg_id, const byte *msg_data)
{
  int32_t servo_data;


  Debug.println("RECEIVED CANBUS DATA");

  switch (msg_id)
  {
  case MOTOR_SETPOINT:
    float leftSpeed, rightSpeed;
    memcpy(&leftSpeed, msg_data, 4);
    memcpy(&rightSpeed, msg_data + 4, 4);
    motorTrLeft.setSpeed(leftSpeed);
    motorTrRight.setSpeed(rightSpeed);

    Debug.println("TRACTION DATA :\tleft: \t" + String(leftSpeed) + "\tright: \t" + String(rightSpeed));
    break;

  case DATA_EE_PITCH_SETPOINT:
    memcpy(&servo_data, msg_data, 4);
#ifdef MODC_EE
    motorEEPitch.moveSpeed(servo_data, SERVO_SPEED);
#endif
    Debug.print("PITCH END EFFECTOR MOTOR DATA : \t");
    Debug.println(servo_data);
    break;

  case DATA_EE_HEAD_PITCH_SETPOINT:
    memcpy(&servo_data, msg_data, 2);
#ifdef MODC_EE
    motorEEHeadPitch.moveSpeed(servo_data, SERVO_SPEED);
#endif
    Debug.print("HEAD PITCH END EFFECTOR MOTOR DATA : \t");
    Debug.println(servo_data);
    break;

  case DATA_EE_HEAD_ROLL_SETPOINT:
    memcpy(&servo_data, msg_data, 2);
#ifdef MODC_EE
    motorEEHeadRoll.moveSpeed(servo_data, SERVO_SPEED);
#endif
    Debug.print("HEAD ROLL END EFFECTOR MOTOR DATA : \t");
    Debug.println(servo_data);
    break;

  case ARM_PITCH_1a1b_SETPOINT:
    memcpy(&servo_data_1a, msg_data, 4);
    memcpy(&servo_data_1b, msg_data + 4, 4);
    theta_dxl = servo_data_1a;
    phi_dxl = servo_data_1b;
    getpositions[0] = (int32_t)(-((theta_dxl * (4096 / (2.0 * M_PI))) + (phi_dxl * (4096 / (2.0 * M_PI)))) / 2) + getpositions0[0];
    getpositions[1] = (int32_t)(((theta_dxl * (4096 / (2.0 * M_PI))) - (phi_dxl * (4096 / (2.0 * M_PI)))) / 2) + getpositions0[1];
#ifdef MODC_ARM
    dxl.setGoalPosition_EPCM(getpositions);
    Serial.println("ARM PITCH 1a1b SETPOINT: " + String(getpositions[0]) + ", " + String(getpositions[1]));
#endif
    Debug.print("PITCH ARM 1a MOTOR DATA : \t");
    Debug.println(getpositions[0]);
    Debug.print("PITCH ARM 1b MOTOR DATA : \t");
    Debug.println(getpositions[1]);
    break;

  case ARM_PITCH_2_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    Serial.println("servo_data: " + String(servo_data_float));
    valueToSend = (int32_t)(servo_data_float * (4096 / (2.0 * M_PI)));
    Serial.println("valueToSend: " + String(valueToSend));
    pos_mot_2 = valueToSend + pos0_mot_2;
#ifdef MODC_ARM
    mot_2.setGoalPosition_EPCM(pos_mot_2);
    Serial.println("ARM PITCH 2 SETPOINT: " + String(pos_mot_2));
#endif
    Debug.print("PITCH ARM 2 MOTOR DATA : \t");
    Debug.println(pos_mot_2);
    break;



  case ARM_ROLL_3_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    valueToSend = (int32_t)(servo_data_float * (4096 / (2.0 * M_PI)));
    pos_mot_3 = valueToSend + pos0_mot_3;
#ifdef MODC_ARM
    mot_3.setGoalPosition_EPCM(pos_mot_3);
    Serial.println("ARM ROLL 3 SETPOINT: " + String(pos_mot_3));
#endif
    Debug.print("ROLL ARM 3 MOTOR DATA : \t");
    Debug.println(pos_mot_3);
    break;



  case ARM_PITCH_4_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    valueToSend = (int32_t)(servo_data_float * (4096 / (2.0 * M_PI)));
    pos_mot_4 =  pos0_mot_4 + valueToSend ;
#ifdef MODC_ARM
    mot_4.setGoalPosition_EPCM(pos_mot_4);
    Serial.println("ARM PITCH 4 SETPOINT: " + String(pos_mot_4));
#endif
    Debug.print("PITCH ARM 4 MOTOR DATA : \t");
    Debug.println(pos_mot_4);
    break;




  case ARM_ROLL_5_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    valueToSend = (int32_t)(servo_data_float * (4096 / (2.0 * M_PI)));
    pos_mot_5 =   pos0_mot_5 - valueToSend;
#ifdef MODC_ARM
    mot_5.setGoalPosition_EPCM(pos0_mot_5);
    Serial.println("ARM ROLL 5 SETPOINT: " + String(pos0_mot_5));
    Serial.println("valueToSend: " + String(valueToSend));
    Serial.println("servo_data_float: " + String(servo_data_float));
#endif
    Debug.print("ROLL ARM 5 MOTOR DATA : \t");
    Debug.println(pos0_mot_5);
    break;



  case ARM_ROLL_6_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    valueToSend = (int32_t)(servo_data_float * (4096 / (2.0 * M_PI)));
    pos_mot_6 =   pos0_mot_6 - valueToSend;
#ifdef MODC_ARM
    mot_6.setGoalPosition_EPCM(pos_mot_6);
    Serial.println("ARM ROLL 5 SETPOINT: " + String(pos0_mot_5));
    Serial.println("valueToSend: " + String(valueToSend));
    Serial.println("servo_data_float: " + String(servo_data_float));
#endif
    Debug.print("ROLL ARM 5 MOTOR DATA : \t");
    Debug.println(pos0_mot_5);
    break;

  case JOINT_PITCH_1d1s_SETPOINT:
    memcpy(&servo_data, msg_data, 2);
#ifdef MODC_JOINT
    dxlJOINT.setGoalPosition_EPCM(servo_data);
#endif
    Debug.print("PITCH JOINT 1d1s MOTOR DATA : \t");
    Debug.println(servo_data);
    break;
  case JOINT_ROLL_2_SETPOINT:
    memcpy(&servo_data, msg_data, 2);
#ifdef MODC_JOINT
    motorJOINT2Roll.setGoalPosition_EPCM(servo_data);
#endif
    Debug.print("ROLL JOINT 2 MOTOR DATA : \t");
    Debug.println(servo_data);
    break;
  }
}




/**
 * @brief Sends feedback data over CAN bus.
 *
 * This function sends various feedback data including motor speeds, yaw angle, and end effector positions
 * if the respective modules are enabled.
 *
 * @note The function uses conditional compilation to include/exclude parts of the code based on the presence of specific modules.
 */
void sendFeedback()
{

  // send motor data
  float speeds[2] = {motorTrLeft.getSpeed(), motorTrRight.getSpeed()};
  canW.sendMessage(MOTOR_FEEDBACK, speeds, 8);

  // send yaw angle of the joint if this module has one
#ifdef MODC_YAW
  encoderYaw.update();
  float angle = encoderYaw.readAngle();
  canW.sendMessage(JOINT_YAW_FEEDBACK, &angle, 4);
#endif

  // send end effector data (if module has it)
#ifdef MODC_EE
  int pitch = motorEEPitch.readPosition();
  int headPitch = motorEEHeadPitch.readPosition();
  int headRoll = motorEEHeadRoll.readPosition();

  canW.sendMessage(DATA_EE_PITCH_FEEDBACK, &pitch, 4);
  canW.sendMessage(DATA_EE_HEAD_PITCH_FEEDBACK, &headPitch, 4);
  canW.sendMessage(DATA_EE_HEAD_ROLL_FEEDBACK, &headRoll, 4);
#endif

/*  // Send the present position data of the arm motors
#ifdef MODC_ARM
  int32_t posf_1a1b[2] = {0, 0}; // Declare and initialize the array
  int32_t posf_2 = 0;
  int32_t posf_3 = 0;
  int32_t posf_4 = 0;
  int32_t posf_5 = 0;

  dxlARM.getPresentPosition(posf_1a1b);
  motorARM2Pitch.getPresentPosition(posf_2);
  motorARM3Roll.getPresentPosition(posf_3);
  motorARM4Pitch.getPresentPosition(posf_4);
  motorARM5Roll.getPresentPosition(posf_5);

  canW.sendMessage(ARM_PITCH_1a1b_FEEDBACK, posf_1a1b, sizeof(posf_1a1b));
  canW.sendMessage(ARM_PITCH_2_FEEDBACK, &posf_2, sizeof(posf_2));
  canW.sendMessage(ARM_ROLL_3_FEEDBACK, &posf_3, sizeof(posf_3));
  canW.sendMessage(ARM_PITCH_4_FEEDBACK, &posf_4, sizeof(posf_4));
  canW.sendMessage(ARM_ROLL_5_FEEDBACK, &posf_5, sizeof(posf_5));
#endif

  // Send the present position data of the joint motors
#ifdef MODC_JOINT
  uint32_t pos_1d1s[numMotors] = {0, 0}; // Declare and initialize the array
  uint32_t pos_2 = 0;

  dxlJOINT.getPresentPosition(pos_1d1s);
  motorJOINT2Roll.getPresentPosition(pos_2);

  canW.sendMessage(JOINT_PITCH_1d1s_FEEDBACK, pos_1d1s, sizeof(pos_1d1s));
  canW.sendMessage(JOINT_ROLL_2_FEEDBACK, &pos_2, sizeof(pos_2));
#endif*/
}

void okInterrupt()
{
  display.okInterrupt();
}

void navInterrupt()
{
  display.navInterrupt();
}