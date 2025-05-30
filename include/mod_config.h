#ifndef module_configuration_h
#define module_configuration_h

// MK1_MOD1, MK1_MOD2, MK2_MOD1, MK2_MOD2 are defined at build time


#if defined(MK1_MOD1)
#define CAN_ID    0x11  // MK1 first module (HEAD)
#define MODC_EE
#define SERVO_EE_PITCH_ID 6
#define SERVO_EE_HEAD_ROLL_ID 4
#define SERVO_EE_HEAD_PITCH_ID 2
#define SERVO_SPEED 200

#elif defined(MK1_MOD2)
#define CAN_ID    0x12  // MK1 second module (MIDDLE)
#define MODC_YAW

#elif defined(MK2_MOD1)
#define CAN_ID    0x21  // MK2 first module (HEAD)
#define MODC_ARM
#define SERVO_ARM_2_PITCH_ID 112
#define SERVO_ARM_3_ROLL_ID 113
#define SERVO_ARM_4_PITCH_ID 214
#define SERVO_ARM_5_ROLL_ID 215
#define SERVO_ARM_1a_PITCH_ID 210
#define SERVO_ARM_1b_PITCH_ID 211

#elif defined(MK2_MOD2)
#define CAN_ID    0x22  // MK2 second module (MIDDLE)
//#define MODC_JOINT
//#define SERVO_JOINT_1d_PITCH_ID 2
//#define SERVO_JOINT_1s_PITCH_ID 4
//#define SERVO_JOINT_2_ROLL_ID 6
#endif

#endif