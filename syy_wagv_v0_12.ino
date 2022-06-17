#include <stdint.h>

#include "syy_vehicle.h"
#include "bms_class.h"
#include "imu.h"
#include "syy_gpio.h"
#include "ros_comm.h"

#define LeftTxControl 23   //RS485 Direction control
#define RightTxControl 22   //RS485 Direction control
#define LeftTXPIN 18
#define RightTXPIN 16
#define IMU_INT_PIN 24

#define POWER_3V3_PIN 25
#define POWER_5V_PIN 26
#define LIGHT_R_PIN 40
#define LIGHT_B_PIN 41

#define BUMPER_PIN_F 30
#define BUMPER_PIN_B 31
#define FREE_MOTOR_PIN 32

#define IMU_PERD_MS 10			// 100 Hz
#define MOTOR_CMD_PERD_MS 50 	// = 1000 / MODBUS_RTU_COMM_RATE
#define PRINT_PERD_MS 750		//  1.5HZ
#define BATTERY_PERD_MS 1000	// check battery every 1 second
#define LED_BLINK_FLOOR 15
#define ROS_IMU_MS 25			// 40Hz
#define ROS_ODOM_MS 50		// 20Hz
#define ROS_CMD_TIMEOUT 1500	

SyyVehicle amr(Serial1, Serial2, LeftTxControl, RightTxControl, LeftTXPIN, RightTXPIN);
BMS_Class battery(Serial3);
Imu imu;
RosComm ros_comm(Serial);

//=== Vehicle ===
#define DEF_VEL 0.15
#define DEF_OMG 10*PI/180
double velocity, omega;

//=== Battery ===
int centi_v = 0; 
int centi_a = 0;
int centi_mah = 0;
int percent = 0;

//=== Imu ===
float quat[4];
float acc[3];
float gyo[3];
float mag[3];
uint32_t imu_measure_time;

//=== loop control ===
uint32_t now;
uint32_t last_imu_ms;
uint32_t last_motor_cmd_ms;
uint32_t last_motor_cmd_us;
uint32_t last_print_ms;
uint32_t last_battery_ms;
uint32_t last_ros_imu_ms;
uint32_t last_ros_odom_ms;
uint32_t last_cmd_update_ms;
bool swing = true;

uint32_t tm_start_ms, tm_end_ms, max_stopwatch_ms;



//=== IO ===
char printbuf[256];
bool reset_flag = true;
bool free_car = false;
bool last_free_car = false;
bool blink_on = true;
int blinker = 0;
SyyGpio btn_free_car(FREE_MOTOR_PIN, 5);
SyyGpio bumper_f(BUMPER_PIN_F, 2), bumper_b(BUMPER_PIN_B, 2);

void setup()
{
	// Start the built-in serial port, probably to Serial Monitor
	//Serial.begin(115200);
	delay(100);
	ros_comm.Init(115200);
	delay(100);
	Serial.write(RosTxHeader.text, RosTxHeader.len);
	Serial.println("SYY AMR controller");
	
	// Power
	pinMode(POWER_3V3_PIN, OUTPUT);
	pinMode(20, OUTPUT);		//IIC pin
	pinMode(21, OUTPUT);
	pinMode(LIGHT_R_PIN, OUTPUT);
	pinMode(LIGHT_B_PIN, OUTPUT);
	
	digitalWrite(POWER_3V3_PIN, LOW);
	digitalWrite(POWER_5V_PIN, LOW);
	digitalWrite(20, LOW);		//IIC pin
	digitalWrite(21, LOW);
	digitalWrite(IMU_INT_PIN, LOW);
	digitalWrite(LIGHT_R_PIN, LOW);
	digitalWrite(LIGHT_B_PIN, LOW);
	
	delay(1500);	// So IMU can properly power off
	digitalWrite(POWER_3V3_PIN, HIGH);
	digitalWrite(POWER_5V_PIN, HIGH);
	digitalWrite(LIGHT_R_PIN, HIGH);
	
	// Comm slave
	amr.Init();
	battery.Init(9600);
	imu.Init(IMU_INT_PIN);
	
	// GPIOs
	btn_free_car.Init();
	bumper_f.Init();
	bumper_b.Init();
	
	// Time keeping
	now = millis();
	last_imu_ms = now;
	last_motor_cmd_ms = now;
	last_battery_ms = now;
	last_print_ms = now;
	last_ros_imu_ms = now;
	last_ros_odom_ms = now;
	last_cmd_update_ms = now;
	last_motor_cmd_us = micros();
}

void loop()
{
	now = millis();
	//uint32_t loop_dur;		// loop duration
	
	//TODO timer overflow
	/*if(now < last_loop_ms)
	{
		loop_dur = UINT32_MAX - last_loop_ms;
		loop_dur += now;
	} else
	{
		loop_dur = now - last_loop_ms;
	}
	
	Serial.println(loop_dur);*/
	
	amr.Update_Comm(now);
	battery.update_comm(now);
	imu.update_comm();			// blocking IO, takes ~2ms if there's update	
	ros_comm.update_comm();

	///========== Scheduler list ==========
	//===== IMU =====
	if( (now - last_imu_ms) >= IMU_PERD_MS )
	{
		if(imu.new_imu_available())
		{	
			imu.get_data(quat, acc, gyo, mag, imu_measure_time);	
		}
		last_imu_ms = now;
	}	
	
	//===== Motor =====
	if( (now - last_motor_cmd_ms) >= MOTOR_CMD_PERD_MS)
	{
		/// serial input
		//update_key(velocity, omega, reset_flag, free_car);
		if(ros_comm.get_cmd(velocity, omega, reset_flag, free_car)) {
			last_cmd_update_ms = now;
		}
		if((now - last_cmd_update_ms) >= ROS_CMD_TIMEOUT) {
			velocity = 0;
			omega = 0;
		}
		
		/// GPIO input with simple debounce
		btn_free_car.update();
		bumper_f.update();
		bumper_b.update();
		
		if( btn_free_car.is_falling() ) {
			free_car = !free_car;	// reverse state
		}
		if( bumper_b.is_low() || bumper_f.is_low() ) {
			free_car = true;
		}
		
		/// Light control
		if(amr.is_alive())
		{
			if(free_car)
			{
				blinker = (blinker+1)%LED_BLINK_FLOOR;
				if( blinker == 0 ) { blink_on = !blink_on; }
				digitalWrite(LIGHT_R_PIN, blink_on);		// free car
				digitalWrite(LIGHT_B_PIN, LOW);
			}else
			{
				if( (abs(velocity)>0.0001) || (abs(omega)>0.00001) )
				{
					blinker = (blinker+1)%LED_BLINK_FLOOR;
					if( blinker == 0 ) { blink_on = !blink_on; }
					digitalWrite(LIGHT_R_PIN, LOW);
					digitalWrite(LIGHT_B_PIN, blink_on);	// moving
				}else
				{				
					digitalWrite(LIGHT_R_PIN, LOW);
					digitalWrite(LIGHT_B_PIN, HIGH);		// standby
				}
			}
		}else 
		{
			digitalWrite(LIGHT_R_PIN, HIGH);				// vehicle dead
			digitalWrite(LIGHT_B_PIN, LOW);
		}
		
		/// Motor command
		uint32_t now_us = micros();
		uint32_t delta_us;
		if(now_us > last_motor_cmd_us) {
			delta_us = now_us - last_motor_cmd_us;
		} else {								// micros timer overflowed
			uint32_t delta_us = UINT32_MAX - last_motor_cmd_us;
			delta_us += now_us;
		}
		
		if(free_car) {
			velocity = 0;
			omega = 0;
			amr.set_free();
		}else {
			amr.set_arm();
		}
		amr.set_velocity(velocity, omega, delta_us);
		amr.update(delta_us);
		if(reset_flag) {
			software_Reset();
			amr.reset();
			reset_flag = false;
		}
		
		
		if(amr.is_alive() && last_free_car!=free_car) {
			Serial.write(RosTxHeader.text, RosTxHeader.len);
			if(free_car) {
				Serial.println("Free!");
			}else {
				Serial.println("Arm!");
			}
			last_free_car = free_car;
		}
		
		last_motor_cmd_ms = now;
		last_motor_cmd_us = now_us;
	}

	//===== Battery Ping =====
	if( (now - last_battery_ms) >= BATTERY_PERD_MS)
	{
		if(battery._new_info_available())
		{
			battery.get_status(centi_v, centi_a, centi_mah, percent);
			//sprintf(printbuf, "Bat: %.2fV %.2fA, Remain: %.3f Ah (%d) %%", (float)centi_v/100,
			//                                                               (float)centi_a/100, 
			//                                                               (float)centi_mah/100, 
			//                                                               percent               );
			//Serial.println(printbuf);
			ros_comm.tx_batt(centi_v, centi_a, centi_mah, percent);
		}else 
		{
			Serial.write(RosTxHeader.text, RosTxHeader.len);
			Serial.println("No battery info");
		}
		battery.check_bms();
		last_battery_ms = now;
	}
	
	//===== Printing ===== (currently, this whole printing stuff takes ~3ms)
	if( (now - last_print_ms) >= PRINT_PERD_MS)
	{
		/*double x, y, th;		// Coordinate in odom frame
		double vx, wz;
		double vl, vr;
		amr.get_odom(x, y, th);
		amr.get_velocity(vx, wz);
		amr.get_wheel_vel(vl, vr);
		
		if(swing){
			Serial.println("==========");
			
			if( amr.is_alive() )
			{
				if(free_car) {
					Serial.print("Free ");
				}else {
					Serial.print("Arm ");
				}
				sprintf(printbuf, "pos [%.2f %.2f %.2f], vel [%.2f %.2f], vLR[%.2f %.2f]", x, y, th, vx, wz, vl, vr);
				Serial.println(printbuf);
			}else
			{
				Serial.println("Vehicle not alive, check and restart!");
			}
		}else{		
			sprintf(printbuf, "q0 %.2f Acc [%.2f %.2f %.2f]g, Gyro [%.2f %.2f %.2f]dps, Mag [%.2f %.2f %.2f]uT, %d ms", quat[0], 
																					acc[0], acc[1], acc[2],
																					gyo[0], gyo[1], gyo[2],
																					mag[0], mag[1], mag[2], imu_measure_time);
			Serial.println(printbuf);
			
			
			sprintf(printbuf, "LT max: %d ms", max_stopwatch_ms);
			Serial.println(printbuf);
		}*/
		//For DEBUG
		//tm_start_ms = millis(); //DEBUG
		//tm_end_ms = millis(); //DEBUG
		max_stopwatch_ms = Max(max_stopwatch_ms, tm_end_ms-tm_start_ms);//DEBUG
		//Serial.write(RosTxHeader.text, RosTxHeader.len);
		//sprintf(printbuf, "LT max: %d ms", max_stopwatch_ms);
		//Serial.println(printbuf);
		swing = !swing;
		last_print_ms = now;
	}
	
	if(now - last_ros_imu_ms >= ROS_IMU_MS)
	{
		ros_comm.tx_imu(quat, acc, gyo, mag, imu_measure_time);	
		last_ros_imu_ms = now;
	}
	if(now - last_ros_odom_ms >= ROS_ODOM_MS)
	{
		double x, y, th;		// Coordinate in odom frame
		double vx, wz;
		double vl, vr;
		amr.get_odom(x, y, th);
		amr.get_velocity(vx, wz);
		amr.get_wheel_vel(vl, vr);
		
		ros_comm.tx_odom( (float)x,(float)y,(float)th, (float)vx, (float)wz );		
		last_ros_odom_ms = now;
	}
		
}// loop()

// for keyboard control
void update_key(double &_vel, double &_omg, bool &_res, bool &free_car)
{
	char key;
	if(!Serial.available()) {
		return;
	}
	key = Serial.read();
	switch(key) {
		case 'w':
			_vel = DEF_VEL;
			_omg = 0;
			break;
		case 'x':
			_vel = -DEF_VEL;
			_omg = 0;
			break;
		case 'a':
			_vel = 0;
			_omg = DEF_OMG;
			break;
		case 'd':
			_vel = 0;
			_omg = -DEF_OMG;
			break;
		case 'q':
			_vel = DEF_VEL;
			_omg = DEF_OMG;
			break;
		case 'e':
			_vel = DEF_VEL;
			_omg = -DEF_OMG;
			break;
		case 'c':
			_vel = -DEF_VEL;
			_omg = -DEF_OMG;
			break;
		case 'z':
			_vel = -DEF_VEL;
			_omg = DEF_OMG;
			break;
		case '+':
			_vel += 0.05;
			break;
		case '-':
			_vel -= 0.05;
			break;
		case 'f':
			_vel = 0;
			_omg = 0;
			free_car = !free_car;
		case 'r':
			_vel = 0;
			_omg = 0;
			_res = true;
			break;
		default:
			_vel = 0;
			_omg = 0;
			break;
	}
}

void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
	pinMode(8, OUTPUT);
} 