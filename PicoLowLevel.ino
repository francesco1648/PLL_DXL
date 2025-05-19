
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

#define ProfileAcceleration 10
#define ProfileVelocity 20

void okInterrupt();
void navInterrupt();
void sendFeedback();
void handleSetpoint(uint8_t msg_id, const byte *msg_data);
float theta_dxl;
float phi_dxl;
int time_bat = 0;
int time_tel = 0;
int time_data = 0;
int time_tel_avg = DT_TEL;
int32_t servo_data_HL_old[2];
CanWrapper canW(5, 10000000UL, &SPI);
//------
int32_t valueToSend = 0;
int32_t pos0_1a1b[2] = {0, 0};
int32_t pos0_2 = 0;
int32_t pos0_3 = 0;
int32_t pos0_4 = 0;
int32_t pos0_5 = 0;

int32_t pos_1a1b[2] = {0, 0};
int32_t pos_2 = 0;
int32_t pos_3 = 0;
int32_t pos_4 = 0;
int32_t pos_5 = 0;

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

// Motor IDs for the two motors.
const uint8_t motorIDs[] = {SERVO_ARM_1a_PITCH_ID, SERVO_ARM_1b_PITCH_ID};
const uint8_t numMotors = sizeof(motorIDs) / sizeof(motorIDs[0]);

DynamixelLL dxlARM(Serial1, 0); // an instance for syncWrite (ID not used).
DynamixelLL motorARM1aPitch(Serial1, SERVO_ARM_1a_PITCH_ID);
DynamixelLL motorARM1bPitch(Serial1, SERVO_ARM_1b_PITCH_ID);
DynamixelLL motorARM2Pitch(Serial1, SERVO_ARM_2_PITCH_ID);
DynamixelLL motorARM3Roll(Serial1, SERVO_ARM_3_ROLL_ID);
DynamixelLL motorARM4Pitch(Serial1, SERVO_ARM_4_PITCH_ID);
DynamixelLL motorARM5Roll(Serial1, SERVO_ARM_5_ROLL_ID);
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
  dxlARM.begin(1000000);
  dxlARM.enableSync(motorIDs, numMotors);
  Serial.println("inizzializing motors");
  dxlARM.setTorqueEnable(false);
  motorARM2Pitch.setTorqueEnable(false);
  motorARM3Roll.setTorqueEnable(false);
  motorARM4Pitch.setTorqueEnable(false);
  motorARM5Roll.setTorqueEnable(false);
  // Set operating mode for all motors to position mode
  dxlARM.setOperatingMode(3);
  motorARM2Pitch.setOperatingMode(3);
  motorARM3Roll.setOperatingMode(3);
  motorARM4Pitch.setOperatingMode(3);
  motorARM5Roll.setOperatingMode(3);

  motorARM1aPitch.setProfileVelocity(ProfileVelocity);
  motorARM1aPitch.setProfileAcceleration(ProfileAcceleration);
  motorARM1bPitch.setProfileVelocity(ProfileVelocity);
  motorARM1bPitch.setProfileAcceleration(ProfileAcceleration);
  motorARM2Pitch.setProfileVelocity(ProfileVelocity);
  motorARM2Pitch.setProfileAcceleration(ProfileAcceleration);
  motorARM3Roll.setProfileVelocity(ProfileVelocity);
  motorARM3Roll.setProfileAcceleration(ProfileAcceleration);
  motorARM4Pitch.setProfileVelocity(ProfileVelocity);
  motorARM4Pitch.setProfileAcceleration(ProfileAcceleration);
  motorARM5Roll.setProfileVelocity(ProfileVelocity);
  motorARM5Roll.setProfileAcceleration(ProfileAcceleration);

  // Configure Drive Mode for each motor:
  motorARM1aPitch.setDriveMode(false, false, false);
  motorARM1bPitch.setDriveMode(false, false, false);
  motorARM2Pitch.setDriveMode(false, false, false);
  motorARM3Roll.setDriveMode(false, false, false);
  motorARM4Pitch.setDriveMode(false, false, false);
  motorARM5Roll.setDriveMode(false, false, false);

  // Declare and initialize the arrays

  // Get present position for all motors
  dxlARM.getPresentPosition(pos0_1a1b);
  motorARM2Pitch.getPresentPosition(pos0_2);
  motorARM3Roll.getPresentPosition(pos0_3);
  motorARM4Pitch.getPresentPosition(pos0_4);
  motorARM5Roll.getPresentPosition(pos0_5);

  // Set homing offset for all motors based on their present positions
  // first, convert data types to float
  /*dxlARM.setHomingOffset({static_cast<float>(pos_1a1b[0]), static_cast<float>(pos_1a1b[1])});
  motorARM2Pitch.setHomingOffset(static_cast<float>(pos_2));
  motorARM3Roll.setHomingOffset(static_cast<float>(pos_3));
  motorARM4Pitch.setHomingOffset(static_cast<float>(pos_4));
  motorARM5Roll.setHomingOffset(static_cast<float>(pos_5));
*/
  // Configure Torque Mode for each motor:
  dxlARM.setTorqueEnable(true);
  motorARM2Pitch.setTorqueEnable(true);
  motorARM3Roll.setTorqueEnable(true);
  motorARM4Pitch.setTorqueEnable(true);
  motorARM5Roll.setTorqueEnable(true);
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
    motorTrLeft.stop();
    motorTrRight.stop();
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
  float servo_data_1a;
  float servo_data_1b;
  float servo_data_float;

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
    pos_1a1b[0] = (int32_t)(-((theta_dxl * (4096 / (2.0 * M_PI))) + (phi_dxl * (4096 / (2.0 * M_PI)))) / 2) + pos0_1a1b[0];
    pos_1a1b[1] = (int32_t)(((theta_dxl * (4096 / (2.0 * M_PI))) - (phi_dxl * (4096 / (2.0 * M_PI)))) / 2) + pos0_1a1b[1];
#ifdef MODC_ARM
    dxlARM.setGoalPosition_EPCM(pos_1a1b);
    Serial.println("ARM PITCH 1a1b SETPOINT: " + String(pos_1a1b[0]) + ", " + String(pos_1a1b[1]));
#endif
    Debug.print("PITCH ARM 1a MOTOR DATA : \t");
    Debug.println(pos_1a1b[0]);
    Debug.print("PITCH ARM 1b MOTOR DATA : \t");
    Debug.println(pos_1a1b[1]);
    break;

  case ARM_PITCH_2_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    Serial.println("servo_data: " + String(servo_data_float));
    valueToSend = (int32_t)(servo_data_float * (4096 / (2.0 * M_PI)));
    Serial.println("valueToSend: " + String(valueToSend));
    pos_2 = valueToSend + pos0_2;
#ifdef MODC_ARM
    motorARM2Pitch.setGoalPosition_EPCM(pos_2);
    Serial.println("ARM PITCH 2 SETPOINT: " + String(pos_2));
#endif
    Debug.print("PITCH ARM 2 MOTOR DATA : \t");
    Debug.println(pos_2);
    break;
  case ARM_ROLL_3_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    valueToSend = (int32_t)(servo_data_float * (4096 / (2.0 * M_PI)));
    pos_3 = valueToSend + pos0_3;
#ifdef MODC_ARM
    motorARM3Roll.setGoalPosition_EPCM(pos_3);
    Serial.println("ARM ROLL 3 SETPOINT: " + String(pos_3));
#endif
    Debug.print("ROLL ARM 3 MOTOR DATA : \t");
    Debug.println(pos_3);
    break;
  case ARM_PITCH_4_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    valueToSend = (int32_t)(servo_data_float * (4096 / (2.0 * M_PI)));
    pos_4 = valueToSend + pos0_4;
#ifdef MODC_ARM
    motorARM4Pitch.setGoalPosition_EPCM(pos_4);
    Serial.println("ARM PITCH 4 SETPOINT: " + String(pos_4));
#endif
    Debug.print("PITCH ARM 4 MOTOR DATA : \t");
    Debug.println(pos_4);
    break;
  case ARM_ROLL_5_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    valueToSend = (int32_t)(servo_data_float * (4096 / (2.0 * M_PI)));
    pos_5 = valueToSend + pos0_5;
#ifdef MODC_ARM
    motorARM5Roll.setGoalPosition_EPCM(pos_5);
    Serial.println("ARM ROLL 5 SETPOINT: " + String(pos_5));
#endif
    Debug.print("ROLL ARM 5 MOTOR DATA : \t");
    Debug.println(pos_5);
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

  // Send the present position data of the arm motors
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
#endif
}

void okInterrupt()
{
  display.okInterrupt();
}

void navInterrupt()
{
  display.navInterrupt();
}