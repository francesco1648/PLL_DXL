#ifndef COMMUNICATION_H
#define COMMUNICATION_H

// CAN bus packet identifiers

#define BATTERY_VOLTAGE                 0x11
#define BATTERY_PERCENT                 0x12
#define BATTERY_TEMPERATURE             0x13
#define MOTOR_SETPOINT                  0x21
#define MOTOR_FEEDBACK			        0x22
#define JOINT_YAW_FEEDBACK              0x32
#define DATA_EE_PITCH_SETPOINT          0x41
#define DATA_EE_HEAD_PITCH_SETPOINT     0x43
#define DATA_EE_HEAD_ROLL_SETPOINT      0x45
#define DATA_EE_PITCH_FEEDBACK          0x42
#define DATA_EE_HEAD_PITCH_FEEDBACK     0x44
#define DATA_EE_HEAD_ROLL_FEEDBACK      0x46

#define ARM_PITCH_1a1b_SETPOINT         0x51
#define ARM_PITCH_1a1b_FEEDBACK         0x52
#define ARM_PITCH_2_SETPOINT            0x53
#define ARM_PITCH_2_FEEDBACK            0x54
#define ARM_ROLL_3_SETPOINT             0x55
#define ARM_ROLL_3_FEEDBACK            0x56
#define ARM_PITCH_4_SETPOINT            0x57
#define ARM_PITCH_4_FEEDBACK            0x58
#define ARM_ROLL_5_SETPOINT            0x59
#define ARM_ROLL_5_FEEDBACK            0x5A
#define ARM_ROLL_6_SETPOINT            0x5B
#define ARM_ROLL_6_FEEDBACK            0x5C

#define JOINT_PITCH_1d1s_SETPOINT       0x61
#define JOINT_PITCH_1d1s_FEEDBACK       0x62
#define JOINT_ROLL_2_SETPOINT          0x63
#define JOINT_ROLL_2_FEEDBACK          0x64

// TODO: update to ROS2 equivalent
#define DATA_PITCH                      0x04 // Deprecated?

#endif
