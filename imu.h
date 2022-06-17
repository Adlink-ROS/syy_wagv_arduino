#ifndef EWING_IMU_H_
#define EWING_IMU_H_

#include <SparkFunMPU9250-DMP.h>
#include "ros_comm.h"

class Imu : public MPU9250_DMP 
{
public:
	/*const signed char orientation[9] = {
		1, 0, 0,
		0, 1, 0,
		0, 0, 1
	};*/
	void Init(int imu_int_pin)
	{
		_imu_int_pin = imu_int_pin;
		pinMode(_imu_int_pin, INPUT_PULLUP);
		Wire.setClock(400000);			// Set I2C to 400kHz
		if (begin() != INV_SUCCESS)
		{
			while (1)
			{
			  Serial.write(RosTxHeader.text, RosTxHeader.len);
			  Serial.println("No IMU comm... check connections, and restart.");
			  delay(5000);
			}
		}
		Serial.write(RosTxHeader.text, RosTxHeader.len);
		Serial.print("IMU init...");
		//resetFifo();
		setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS); 
		setSampleRate(1000); 		// Set accel/gyro sample rate to 4Hz
		setCompassSampleRate(40); 	// Set mag rate to 4Hz
		enableInterrupt();
		setIntLevel(INT_ACTIVE_LOW);
		setIntLatched(INT_LATCHED);
		//delay(200);
		//dmpSetOrientation(orientation);
		if( dmpBegin( DMP_FEATURE_6X_LP_QUAT | 		// Enable 6-axis quat
			          DMP_FEATURE_GYRO_CAL |
			          DMP_FEATURE_SEND_RAW_ACCEL|
			          DMP_FEATURE_SEND_CAL_GYRO,    // Use gyro calibration
			          100						)   // Set DMP FIFO rate to 10 Hz
					                 != INV_SUCCESS    )
		{
			while (1)
			{
				Serial.write(RosTxHeader.text, RosTxHeader.len);
				Serial.println("DMP begin failed, please restart.");
				if(Serial.available()) {
					char ch = Serial.read();
					if(ch == 'r') pinMode(8, OUTPUT);
				}
				delay(5000);
			}
			_alive = false;
		}
		Serial.println("IMU alive!");
		_alive = true; 

		_new_imu_available = false;
	}
	
	void update_comm(void)  // Check for new data in the FIFO
	{
		if( ( digitalRead(_imu_int_pin) == LOW ) )
		{
			if( fifoAvailable() )
			{
				// Use dmpUpdateFifo to update the ax, gx, mx, etc. values
				if( dmpUpdateFifo() == INV_SUCCESS )
				{
					computeEulerAngles();
					update(UPDATE_COMPASS);
					//update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
					_new_imu_available = true;
					
				}				
			}
		}
	}
	
	bool new_imu_available()
	{
		return _new_imu_available;
	}
	
	bool is_alive()
	{
		return _alive;
	}
	
	void get_data(float (&q)[4], float (&accel)[3], float (&gyro)[3], float (&mag)[3], unsigned long &data_time)
	{
		// After calling dmpUpdateFifo() the ax, gx, mx, etc. values
		// are all updated.
		// Quaternion values are, by default, stored in Q30 long
		// format. calcQuat turns them into a float between -1 and 1
		q[0] = calcQuat(qw);
		q[1] = calcQuat(qx);
		q[2] = calcQuat(qy);
		q[3] = calcQuat(qz);
		accel[0] = calcAccel(ax);
		accel[1] = calcAccel(ay);
		accel[2] = calcAccel(az);
		gyro[0] = calcGyro(gx);
		gyro[1] = calcGyro(gy);
		gyro[2] = calcGyro(gz);
		mag[0] = calcMag(mx);
		mag[1] = calcMag(my);
		mag[2] = calcMag(mz);
		data_time = time;
		_new_imu_available = false;
		return;
	}
	
	
private:
	bool _new_imu_available;
	bool _alive;
	int _imu_int_pin, _last_pin_status;
};


#endif