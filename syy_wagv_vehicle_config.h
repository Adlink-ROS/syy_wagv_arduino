/*******************************************************************************

*******************************************************************************/
#ifndef SYY_VEHICLE_CONFIG_H_
#define SYY_VEHICLE_CONFIG_H_

//===== Math =====
//#define RAD_TO_CDEG           5729.5780   // centi-degree
//#define DEG2RAD(x)              (x * 0.01745329252)  // *PI/180
//#define RAD2DEG(x)              (x * 57.2957795131)  // *180/PI

//===== Control parameters =====
#define CUTOFF_FREQ_V		 		2
#define CUTOFF_FREQ_W		 		5		// omega

//===== Limits =====
#define MAX_VX				1
#define MAX_OMEGA_Z			PI/2		// MAX_WHEEL_VEL / WHEEL_SEP_DIV2 = 3.605
#define MIN_DRIVE_VEL		0.001		// 1 mm/s
#define MAX_WHEEL_VEL		1.0			// m/s
#define MAX_MOTOR_RPM  		2500   		// linear driving motor 2500/VEL_TO_MOTOR_RPM = 0.9817 m/s		

//===== Physical parameters =====
#define WHEEL_RAD			0.075			// meters
#define WHEEL_SEP			0.5548			// wheel seperation in meters
#define WHEEL_SEP_DIV2		0.2774			// wheel distance to center

//===== Encoder and wheel =====
#define STEP_PER_REV 		200000				// 10,000 step/m-rev * 20 gear ratio
#define STEP2RAD        	2*PI*0.000005 		// 2*pi / step-per-rev
#define VEL_TO_MOTOR_RPM    2546.479089470325	// =1(m/s) / ( (WHEEL_RAD*(2pi))(m/rev) ) * 60(sec/min) * 20(gear-ratio)

/*
static char *joint_states_name[] = { "front_right_steer_joint", "front_right_wheel_joint",
									 "back_left_steer_joint", "back_left_wheel_joint"  };
*/


#endif  //SYY_VEHICLE_CONFIG_H_
