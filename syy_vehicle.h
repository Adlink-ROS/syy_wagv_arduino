#ifndef SYY_VEHICLE_
#define SYY_VEHICLE_
#include <Arduino.h>
#include "syy_wheel.h"
#include "syy_wagv_vehicle_config.h"
#include "ros_comm.h"

class SyyVehicle {
public:
	/// Constructor
    SyyVehicle(USARTClass &port_l, USARTClass &port_r, int ctl_ch_l, int ctl_ch_r, int tx_l, int tx_r): 
		wheel_l(Wheel_Id::LEFT, port_l, ctl_ch_l, tx_l),
		wheel_r(Wheel_Id::RIGHT, port_r, ctl_ch_r, tx_r)
	{
    }
    
	void Init()
	{
        bool resl, resr;
		resl = wheel_l.Init();
		resr = wheel_r.Init();
		reset();
		_init_flag = true;
		if(resl && resr)
		{
			Serial.write(RosTxHeader.text, RosTxHeader.len);
			Serial.println("Vehicle Start!");
			_alive = true;
		}else {
			_alive = false;
			if(!resl) {
				Serial.write(RosTxHeader.text, RosTxHeader.len);
				Serial.println("Left wheel error!");
			}
			if(!resr) {
				Serial.write(RosTxHeader.text, RosTxHeader.len);
				Serial.println("Right wheel error!");
			}
			Serial.write(RosTxHeader.text, RosTxHeader.len);
			Serial.println("Restart to reset");
		}
	}
	
	void reset()
	{
		_x = 0;
		_y = 0;
		_th = 0;
		_vx = 0;
		_wz = 0;
		_init_flag = false;
		return;
	}
	
	void Update_Comm(const uint32_t &now)
	{
		wheel_l.update_comm(now);
		wheel_r.update_comm(now);
	}
	
	void set_velocity(double vx, double wz, uint32_t &dt_us)
	{
		set_vel_tgt(vx, wz, _tgt_vx, _tgt_wz);
		calc_vel_cmd(_tgt_vx, _tgt_wz, dt_us);
		
		bool res1 = wheel_l.set_vel(_cmd_left_vel);
		bool res2 = wheel_r.set_vel(_cmd_right_vel);
		return; 
	}
	
	void update(uint32_t &dt_us)
	{
		if(_alive)
		{
			wheel_l.update(dt_us);
			wheel_r.update(dt_us);
			calc_odom(dt_us);
			if(_init_flag) reset();
		}
		return;
	}
	
	void get_velocity(double &vx, double & wz)
	{
		vx = _vx;
		wz = _wz;
		return;
	}
	void get_odom(double &x, double &y, double &th)
	{
		x = _x;
		y = _y;
		th = _th;
		return;
	}
	
	void get_wheel_vel(double &l, double &r)
	{
		l = wheel_l.get_velocity();
		r = wheel_r.get_velocity();
		return;
	}
		
	bool is_alive(void)
	{
		return _alive;
	}
	
	void set_free()
	{
		wheel_l.free_motor();
		wheel_r.free_motor();
	}
	
	void set_arm()
	{
		wheel_l.arm_motor();
		wheel_r.arm_motor();
	}
	
	/*********************************************************************
		Name: set_vel_tgt
		Input: [vx] velocity in m/s, [wz] rotational velocity in rad/s
		Ouput: [tgt_vx], [tgt_wx]
		Return: 
		Note:
	*********************************************************************/
	void set_vel_tgt(double vx, double wz, double &tgt_vx, double &tgt_wx)
	{
		vx = constrain(vx, -MAX_VX, MAX_VX);
		wz = constrain(wz, -MAX_OMEGA_Z, MAX_OMEGA_Z);
		
		double left_vel = vx - wz * WHEEL_SEP_DIV2;
		double right_vel = vx + wz * WHEEL_SEP_DIV2;
		
		// normalize and constrain the velocity
		float quicker = Max( abs(left_vel), abs(right_vel) );
		if( quicker > MAX_WHEEL_VEL) {
			//right_vel /= quicker;
			//left_vel /= quicker;
			vx /= quicker;
			wz /= quicker;
		}
		
		tgt_vx = vx;
		tgt_wx = wz;
		return;
	}
	
	void calc_vel_cmd(double tgt_vx, double tgt_wz, uint32_t &dt_us)
	{
		double alpha, alpha_compl;
		double cutoff_freq_rad;
		double dt = (double)dt_us*0.000001;
		
		//===== 1st order low pass filter for velocity =====
		// LPF: y(k) = a*x(k) + (1-a)*y(k-1)
		cutoff_freq_rad = 2*PI * CUTOFF_FREQ_V * dt;
		alpha = cutoff_freq_rad / (cutoff_freq_rad + 1);
		alpha_compl = 1 / (cutoff_freq_rad + 1);
		_cmd_vx = alpha * tgt_vx + alpha_compl * _cmd_vx;
		
		//===== 1st order low pass filter for omega =====
		cutoff_freq_rad = 2*PI * CUTOFF_FREQ_W * dt;
		alpha = cutoff_freq_rad / (cutoff_freq_rad + 1);
		alpha_compl = 1 / (cutoff_freq_rad + 1);
		_cmd_wz = alpha * tgt_wz + alpha_compl * _cmd_wz;
		
		//===== Calculate wheel command =====
		_cmd_left_vel = _cmd_vx - _cmd_wz * WHEEL_SEP_DIV2;
		_cmd_right_vel = _cmd_vx + _cmd_wz * WHEEL_SEP_DIV2;
		
		return;
	}
	
	void calc_odom(uint32_t &dt_us)
	{
		double ds_l = wheel_l.get_lin_diff();
		double ds_r = wheel_r.get_lin_diff();
		
		double dS = (ds_l + ds_r) / 2;
		double dth = (ds_r - ds_l) / WHEEL_SEP;
		double dx = dS * cos(_th + dth/2);
		double dy = dS * sin(_th + dth/2);
		
		_x += dx;
		_y += dy;
		_th += dth;
		
		_vx = dS / dt_us * 1000000;
		_wz = dth / dt_us * 1000000;
		return;
	}
	
	
private:
	SyyWheel wheel_l;
	SyyWheel wheel_r;
	double _tgt_vx, _cmd_vx;      // m/s: target vx, command velocity
	double _tgt_wz, _cmd_wz;
	double _cmd_left_vel, _cmd_right_vel;
	//double _curr_left_vel, _curr_right_vel;
	
	double _x, _y, _th;		// Coordinate in odom frame
	double _vx, _wz;
	
	bool _init_flag;
	bool _alive;
};

#endif