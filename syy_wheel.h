#ifndef SYY_WHEEL_
#define SYY_WHEEL_

#include "comm_utils.h"
#include "modbus_comm.h"
#include "syy_wagv_vehicle_config.h"
#include "ros_comm.h"

enum Wheel_Id
{
    LEFT = 0, 
    RIGHT = 1,
	WHEEL_COUNT
};

struct Modbus_Conf_t
{
	int id[Wheel_Id::WHEEL_COUNT];
	int baud[Wheel_Id::WHEEL_COUNT];
};
static Modbus_Conf_t _set_Modbus_Conf(void)
{
	Modbus_Conf_t temp;
	temp.id[Wheel_Id::LEFT] = 1;
	temp.id[Wheel_Id::RIGHT] = 1;
	temp.baud[Wheel_Id::LEFT] = 57600;
	temp.baud[Wheel_Id::RIGHT] = 57600;
	return temp;
};
static Modbus_Conf_t Modbus_Conf = _set_Modbus_Conf();

struct Wheel_Conf_t
{
	String name[Wheel_Id::WHEEL_COUNT];
	int16_t motor_dir[Wheel_Id::WHEEL_COUNT];		// motor drive direction
	int16_t step_dir[Wheel_Id::WHEEL_COUNT];		// encoder direction
};
static Wheel_Conf_t _set_Wheel_Conf(void)
{
	Wheel_Conf_t temp;
	temp.name[Wheel_Id::LEFT] = "Left";
	temp.name[Wheel_Id::RIGHT] = "Right";
	temp.motor_dir[Wheel_Id::LEFT] = 1;
	temp.motor_dir[Wheel_Id::RIGHT] = -1;
	temp.step_dir[Wheel_Id::LEFT] = 1;
	temp.step_dir[Wheel_Id::RIGHT] = -1;
	return temp;
};
static Wheel_Conf_t Wheel_Conf = _set_Wheel_Conf();

class SyyWheel {
public:
	ModbusComm motor; //TODO: consider this to be private

    SyyWheel(Wheel_Id id, USARTClass &port, int ctl_ch, int tx_ch) : motor(port, ctl_ch, tx_ch)
	{
        _wheel_Id = id;
        _curr_step = 0;
		_last_step = 0;
		
		_current_vel_cmd = 0;      // in metric unit: m/s
		_current_rpm_cmd = 0;
	
		_cum_step = 0;		// cumulative encoder value
		_step_diff = 0;		// encoder position difference between updates
		_ang_vel = 0;	// wheel's angular velocity
		_lin_vel = 0;	// wheel's linear velocity

		_angle = 0;		// wheel's current angle
		_dist = 0;		// wheel's accumulative distance
		
		_step_diff_since_last = 0;
		_step_alive = false;
		_alive = false;
		_free_motor = false;
    }
    
	bool Init()
	{
        bool rtn;
		_current_vel_cmd = 0;
		_current_rpm_cmd = 0;
		_curr_step = 0;
		_last_step = 0;
		Serial.write(RosTxHeader.text, RosTxHeader.len);
		Serial.println("--> "+Wheel_Conf.name[_wheel_Id]+" ");
		rtn = motor.Init(Modbus_Conf.id[_wheel_Id], Modbus_Conf.baud[_wheel_Id]);	// id and baud
		_first_enc_echo = false;
		_step_alive = false;
		_alive = motor.is_alive();
		return rtn;
	}
	
	/*********************************************************************
		Name: 
		Input: 
		Ouput:
		Return:
	*********************************************************************/
	void update_comm(const uint32_t &now)
	{
		comm_err_t update_res = motor.update(now);
		
		if( update_res != Comm_Return_Code::NORMAL)
		{
			//Serial.print(_wheel_Id);
			Serial.write(RosTxHeader.text, RosTxHeader.len);
			Serial.print(Wheel_Conf.name[_wheel_Id]+" ");
			Serial.println(Comm_Code_text.data[update_res]);
		}
	}
	
	
	/*********************************************************************
		Name: 
		Input: 
		Ouput:
		Return:
	*********************************************************************/
	void update(const int32_t &dt_us)
	{
		if(!_free_motor) {
			if(_last_free_motor == _free_motor) {
				motor.servo_drive( (int16_t)_current_rpm_cmd );
			}else {
				motor.servo_on();
				_last_free_motor = _free_motor;	// rising edge detection
			}
		}else{
			motor.servo_off();
			_last_free_motor = _free_motor;
		}
		
		// feedback
		update_encoder();
		update_velocity( dt_us );
		
		// rising edge detection for encoder liveness
		if(!_first_enc_echo) {
			if(_step_alive) {
				_first_enc_echo = true;
			}
			reset();
		}
	}
	
	/*********************************************************************
		Name: 
		Input: 
		Ouput:
		Return:
	*********************************************************************/
	void reset(void)
	{
		_current_vel_cmd = 0;     // in metric unit: m/s
		_current_rpm_cmd = 0;

		_ang_vel = 0;		// drive angular velocity
		_lin_vel = 0;		// drive linear velocity

		_cum_step = 0;
		_angle = 0;		// drive wheel current angle
		_dist = 0;
		
		_step_diff_since_last = 0;
		_step_diff = 0;
				 
		_last_step = _curr_step;
	}
	
	/*********************************************************************
		Name: set_vel
		Input: [vel] velocity in m/s
		Ouput:
		Return: 
		Note: the command will be sent to the motor by update()
	*********************************************************************/
	bool set_vel(double vel)
	{
		_current_vel_cmd = constrain(vel, -MAX_WHEEL_VEL, MAX_WHEEL_VEL);
		double rpm = get_cmd_rpm_inrange();
		rpm *= (double)Wheel_Conf.motor_dir[_wheel_Id];
		_current_vel_cmd = _current_vel_cmd;
		_current_rpm_cmd = rpm;
		return true;
	}
	
    void update_encoder(void)
    {
        if(motor.step_available()) {
			_curr_step = motor.get_step();
			_step_alive = true;
		}else {
			return;
		}
		
		int64_t d_step = (int64_t)_curr_step - (int64_t)_last_step;
		
		// Check if cross overflow point
		if( abs(d_step) > INT32_MAX )
		{
			if( d_step > 0)
			{
				d_step = UINT32_MAX - d_step;
			}else 
			{
				d_step = d_step - UINT32_MAX;
			}
		}
		_step_diff = (int32_t) d_step;
		
        _last_step  = _curr_step;
		_cum_step += _step_diff;
		_step_diff_since_last += _step_diff;
    }

    void update_velocity(const uint32_t &dt_us)
	{
		double __angle_diff = STEP2RAD * (double)_step_diff;
		_ang_vel = (__angle_diff / dt_us) * 1000000;
		_lin_vel = _ang_vel * WHEEL_RAD;
		
		//_angle = STEP2RAD * _cum_step;
		//_dist = STEP2RAD * (double)_cum_step * WHEEL_RAD;
	}
	
	int32_t get_cum_step()
	{
		return _cum_step;
	}
	
	double get_velocity()
	{
		return Wheel_Conf.step_dir[_wheel_Id]*_lin_vel;
	}
	   
    double get_cmd_rpm_inrange(void)
    {
        return constrain( get_cmd_rpm(), -MAX_MOTOR_RPM, MAX_MOTOR_RPM );
    }
    
    double get_cmd_rpm(void)
    {
        return (_current_vel_cmd * VEL_TO_MOTOR_RPM);
    }
	
	bool is_alive(void)
	{
		return _alive;
	}
	
	bool is_free(void)
	{
		return _free_motor;
	}
	
	void free_motor(void)
	{
		_free_motor = true;
	}
	
	void arm_motor(void)
	{
		_free_motor = false;
	}
	
    /*********************************************************************
		Name: get_lin_diff
		Input:
		Ouput:
		Return: linear distance since last call
		Note: dirve linear distance difference
	*********************************************************************/ 
	double get_lin_diff()
	{
		double dis_since_last = STEP2RAD * (double) _step_diff_since_last * WHEEL_RAD;
		_step_diff_since_last = 0;
		return Wheel_Conf.step_dir[_wheel_Id]*dis_since_last;
	}
		
	/*********************************************************************
		===DEBUG FUN===
		Name: set_vel_blocked
		Input: [vel] velocity in m/s
		Ouput:
		Return: false if error
	*********************************************************************/
	/*bool set_vel_blocked(double vel)
	{
		_current_vel_cmd = constrain(vel, -MAX_WHEEL_VEL, MAX_WHEEL_VEL);
		double rpm = get_cmd_rpm_inrange();
		Serial.print(Wheel_Conf.name[_wheel_Id]);
		rpm *= (double)Wheel_Conf.motor_dir[_wheel_Id];
		return motor.cmd_rpm_blocked( (int16_t)rpm );
	}*/
	
	
private:
    Wheel_Id _wheel_Id;
	
	double _current_vel_cmd;      // in metric unit: m/s
	double _current_rpm_cmd;
	
	/// Encoder related
	int32_t _curr_step;     // current encoder position
    int32_t _cum_step;		// cumulative encoder value
	int32_t _last_step;		// last encoder position 
	int32_t _step_diff;		// encoder position difference between updates

	/// Wheel related
   	double _ang_vel;	// wheel's angular velocity
	double _lin_vel;	// wheel's linear velocity

	double _angle;		// wheel's current angle
	double _dist;		// wheel's accumulative distance
		
	int32_t _step_diff_since_last;
	
	bool _first_enc_echo;
	bool _step_alive;
	bool _alive;		// liveliness
	bool _free_motor, _last_free_motor;
	
};

#endif