#ifndef ROS_COMM_H_
#define ROS_COMM_H_

#include <UARTClass.h>
#include "comm_utils.h"

#define ROS_TX_HEADER_LEN 2
#define ROS_RX_HEADER_LEN 2

enum Ros_Tx_Type {
	ROS_TX_IMU,
	ROS_TX_ODOM,
	ROS_TX_BATT,
	ROS_TX_TEXT,
	ROS_TX_COUNT
};
enum Ros_Rx_Type {
	ROS_RX_CMDVEL,
	ROS_RX_TEXT,
	ROS_RX_COUNT,		// above is count
	ROS_RX_UNKNOWN
};

struct Ros_Pkt_Length_t
{
	int tx[Ros_Tx_Type::ROS_TX_COUNT];
	int rx[Ros_Rx_Type::ROS_RX_COUNT];
};
static Ros_Pkt_Length_t _set_Ros_Pkt_Length(void)
{
	Ros_Pkt_Length_t temp;
	
	temp.tx[Ros_Tx_Type::ROS_TX_IMU] 		= 2+9*4;
	temp.tx[Ros_Tx_Type::ROS_TX_ODOM]		= 2+5*4+1;
	temp.tx[Ros_Tx_Type::ROS_TX_BATT]		= 2+16;
	temp.tx[Ros_Tx_Type::ROS_TX_TEXT] 		= -1;	
	
	temp.rx[Ros_Rx_Type::ROS_RX_CMDVEL] 	= 2+8;	
	temp.rx[Ros_Rx_Type::ROS_RX_TEXT]		= -1;	
	return temp;
};
Ros_Pkt_Length_t Ros_Pkt_Length = _set_Ros_Pkt_Length();

struct RosTxHeader_t 
{
	byte imu[2];
	byte odom[2];
	byte battery[2];
	byte text[2];
	int len;
};
static RosTxHeader_t _set_RosTxHeader_t(void)
{
	RosTxHeader_t temp;
	temp.imu[0] = 0xFF;
	temp.imu[1] = 0xFA;
	temp.odom[0] = 0xFF;
	temp.odom[1] = 0xFB;
	temp.battery[0] = 0xFF;
	temp.battery[1] = 0xFC;
	temp.text[0] = 0xFF;
	temp.text[1] = 0xFD;
	temp.len = ROS_TX_HEADER_LEN;
	return temp;
};
static RosTxHeader_t RosTxHeader = _set_RosTxHeader_t();

struct RosRxHeader_t 
{
	byte cmdvel[2];
	byte text[2];
	int len;
};
static RosRxHeader_t _set_RosRxHeader_t(void)
{
	RosRxHeader_t temp;
	temp.cmdvel[0] = 0xFF;
	temp.cmdvel[1] = 0xF9;
	temp.text[0] = 0xFF;
	temp.text[1] = 0xF8;
	temp.len = ROS_RX_HEADER_LEN;
	return temp;
};
static RosRxHeader_t RosRxHeader = _set_RosRxHeader_t();

class RosComm
{
public:
	RosComm(UARTClass &port) : _port(port)
	{
	};
	
	void Init(int baudrate)
	{
		_port.begin(baudrate);
		RX_Status _rx_state;
				
		_cmd_vx = 0;
		_cmd_wz = 0;
		_odom_cnt = 0;
		_cmd_rest = false;
		_cmd_free = false;
		_new_cmdtext = false;
		_new_cmdvel = false;
		_rx_state = RX_Status::RX_STDBY;
	}
	
	bool tx_imu( const float (&q)[4], const float (&accel)[3], 
	             const float (&gyro)[3], const float (&mag)[3], const unsigned long &data_time)
	{
		byte tx_buf[Ros_Pkt_Length.tx[Ros_Tx_Type::ROS_TX_IMU]];
		int len = 0;
		for(; len < RosTxHeader.len; len++) {
			tx_buf[len] = RosTxHeader.imu[len];
		}
		for(int i=0;i<3;i++) {
			len += serialize(&tx_buf[len], accel[i]);
		}
		for(int i=0;i<3;i++) {
			len += serialize(&tx_buf[len], gyro[i]);
		}
		for(int i=0;i<3;i++) {
			len += serialize(&tx_buf[len], mag[i]);
		}
		/*Serial.write(RosTxHeader.text, RosTxHeader.len);
		byte *ptr = (byte *)&accel[2];
		String L;
		L = String("0x: ");
		for(int k=0 ; k<4 ; k++) {
			L += String(ptr[k],HEX) + " ";
		}
		Serial.print(L);
		Serial.println(accel[2]);*/
		if(len != Ros_Pkt_Length.tx[Ros_Tx_Type::ROS_TX_IMU]) {
			return false;
		} else {
			return send_pkt(tx_buf, len);
		}
	}
	
	bool tx_odom(const float &x, const float &y, const float &th, 
	             const float &vx, const float &wz                    )
	{
		byte tx_buf[Ros_Pkt_Length.tx[Ros_Tx_Type::ROS_TX_ODOM]];
		int len = 0;
		for(; len < RosTxHeader.len; len++) {
			tx_buf[len] = RosTxHeader.odom[len];
		}
		len += serialize<float>(&tx_buf[len], x);
		len += serialize<float>(&tx_buf[len], y);
		len += serialize<float>(&tx_buf[len], th);
		len += serialize<float>(&tx_buf[len], vx);
		len += serialize<float>(&tx_buf[len], wz);
		tx_buf[len] = _odom_cnt;
		_odom_cnt++;
		len++;
		
		if(len != Ros_Pkt_Length.tx[Ros_Tx_Type::ROS_TX_ODOM]) {
			return false;
		} else {
			return send_pkt(tx_buf, len);
		}
	}
	
	bool tx_batt(int cV, int cA, int cmah, int cap)
	{
		byte tx_buf[Ros_Pkt_Length.tx[Ros_Tx_Type::ROS_TX_BATT]];
		int len = 0;
		for(; len < RosTxHeader.len; len++) {
			tx_buf[len] = RosTxHeader.battery[len];
		}
		len += serialize<int>(&tx_buf[len], cV);
		len += serialize<int>(&tx_buf[len], cA);
		len += serialize<int>(&tx_buf[len], cmah);
		len += serialize<int>(&tx_buf[len], cap);
		
		if(len != Ros_Pkt_Length.tx[Ros_Tx_Type::ROS_TX_BATT]) {
			return false;
		} else {
			return send_pkt(tx_buf, len);
		}
	}
	
	bool send_pkt(byte *data, int len)
	{
		if(len < _port.availableForWrite())
		{
			_port.write(data, len);
			return true;
		}else {
			return false;
		}
	}
	
	bool get_cmd(double &vel, double &omg, bool &reset, bool &free_car)
	{
		bool res = false;
		if(new_cmdvel()) {
			float vel_f, omg_f;
			get_new_cmdvel(vel_f, omg_f);
			vel = (double)vel_f;
			omg = (double)omg_f;
			res = true;
		}
		if(new_cmdtext()) {
			bool free_toggle;
			get_new_cmdtext(reset, free_toggle);
			if(free_toggle) {
				free_car = !free_car;
			}
		}
		return res;
	}
	
	void update_comm()
	{
		if( _rx_state == RX_Status::RX_STDBY)
		{
			if( _port.available() < RosRxHeader.len)
			{
				return;				
			} 
			else
			{
				for(int i=0; i<RosRxHeader.len; i++) {
					_cmd[i] = _port.read();
				}
				_curr_rx_type = parse_rx_type(_cmd, RosRxHeader.len);
				if(_curr_rx_type == Ros_Rx_Type::ROS_RX_UNKNOWN) {
					_rx_state = RX_Status::RX_SYNCING;
				} else {
					_rx_state = RX_Status::RX_RECEIVING;
				}
			}
		}
		
		if( _rx_state == RX_Status::RX_SYNCING)
		{
			while(_port.available() && _curr_rx_type==Ros_Rx_Type::ROS_RX_UNKNOWN)
			{
				for(int i=0; i<(RosRxHeader.len-1); i++) {	// push everything one step 
					_cmd[i] = _cmd[i+1];
				}
				_cmd[RosRxHeader.len-1] = _port.read();
				_curr_rx_type = parse_rx_type(_cmd, RosRxHeader.len);
				if(_curr_rx_type != Ros_Rx_Type::ROS_RX_UNKNOWN) {
					_rx_state = RX_Status::RX_RECEIVING;
				}
			}
		}
		
		if( _rx_state == RX_Status::RX_RECEIVING)
		{
			if(_curr_rx_type == Ros_Rx_Type::ROS_RX_TEXT) {
				
				while(_port.available()) {
					_rx_data[_rx_txt_len] = _port.read();
					_rx_txt_len++;
					
					// detect EOL
					if((_rx_data[_rx_txt_len-1] == '\r')||(_rx_txt_len>=50)) {
						parse_rx_data(_rx_data, _curr_rx_type, _rx_txt_len);
						_rx_state = RX_Status::RX_STDBY;
						_rx_txt_len = 0;
						return;
					}
				}
				return;	// nothing more to read, no EOL
			}
			
			if( _port.available() < Ros_Pkt_Length.rx[_curr_rx_type])
			{
				return;		// waiting for enough data to comein		
			} else
			{
				for(int i=0; i<Ros_Pkt_Length.rx[_curr_rx_type]; i++) {	// push everything one step 
					_rx_data[i] = _port.read();
				}
				parse_rx_data(_rx_data, _curr_rx_type, Ros_Pkt_Length.rx[_curr_rx_type]);
				_rx_state = RX_Status::RX_STDBY;
			}
		}
		return;
	}// update_comm()
	
	bool new_cmdvel()
	{
		return _new_cmdvel;
	}
	bool new_cmdtext()
	{
		return _new_cmdtext;
	}
	void get_new_cmdvel(float &vx, float &wz)
	{
		if( _new_cmdvel) {
			vx = _cmd_vx;
			wz = _cmd_wz;
			_new_cmdvel = false;
		}
		return;
	}
	void get_new_cmdtext(bool &cmd_rest, bool &cmd_free)
	{
		if(_new_cmdtext) {
			cmd_rest = _cmd_rest;
			cmd_free = _cmd_free;
			_cmd_rest = false;
			_cmd_free = false;
			_new_cmdtext = false;
		}
		return;
	}
	
private:
	UARTClass &_port;
	//Comm_Status _comm_state;
	RX_Status _rx_state;
	Ros_Rx_Type _curr_rx_type;
	int _rx_txt_len;
	
	byte _cmd[ROS_RX_HEADER_LEN];
	byte _rx_data[50];
	//byte _tx_buffer[128];
	byte _odom_cnt;
	
	float _cmd_vx, _cmd_wz;
	bool _cmd_rest, _cmd_free;
	bool _new_cmdtext, _new_cmdvel;
	
	template <typename T>
	int serialize(byte *bfr, T data)
	{
		int size = sizeof(data);
		memcpy(bfr, &data, size);
		/*unsigned int dd = *(int *)(&data);
		for(int i=0; i<size; i++) 
		{
			bfr[i] = (byte)( ( dd  >> ((size-1-i)*8) )&0xFF );
		}*/
		return size;
	}
	
	template <typename T>
	int deserialize(byte *bfr, T &data)
	{
		int size = sizeof(data);
		memcpy(&data, bfr, size);
		/*if(size != 4) return 0;
		unsigned int dd = 0;
		dd = (unsigned int)( bfr[0] << (size-1)*8 );	// so it'll clear all the bits
		for(int i=size-1; i>0; i--) 
		{
			dd |= bfr[size-i]<<((i-1)*8);
		}
		data = *(T *)(&dd);*/
		return size;
	}
	
	Ros_Rx_Type parse_rx_type(byte *cmd, int len)
	{
		if(len != RosRxHeader.len) {
			return Ros_Rx_Type::ROS_RX_UNKNOWN;
		}
		if(cmd[0] == RosRxHeader.cmdvel[0] && cmd[1] == RosRxHeader.cmdvel[1]) {
			return Ros_Rx_Type::ROS_RX_CMDVEL;
		}
		if(cmd[0] == RosRxHeader.text[0] && cmd[1] == RosRxHeader.text[1]) {
			_rx_txt_len = 0;
			return Ros_Rx_Type::ROS_RX_TEXT;
		}
		return Ros_Rx_Type::ROS_RX_UNKNOWN;
	}
	
	bool parse_rx_data(byte *rx_data, Ros_Rx_Type type, int len)
	{
		switch(type) {
			case (Ros_Rx_Type::ROS_RX_CMDVEL):
				_new_cmdvel = parse_cmdvel(rx_data, _cmd_vx, _cmd_wz);
				return _new_cmdvel;
				break;
			case (Ros_Rx_Type::ROS_RX_TEXT):
				_new_cmdtext = parse_cmdtext(rx_data, len, _cmd_rest, _cmd_free);
				return _new_cmdtext;
				break;
			default:
				return false;
				break;
		}
	}
	
	bool parse_cmdvel(byte *bfr, float &_cmd_vx, float &_cmd_wz)
	{
		deserialize<float>(&bfr[0], _cmd_vx);
		deserialize<float>(&bfr[4], _cmd_wz);
		return true;
	}
	
	bool parse_cmdtext(const byte *rx_data, const int len, bool &cmd_rest, bool &cmd_free)
	{
		Serial.write(RosTxHeader.text, RosTxHeader.len);
		Serial.print("Get txtcmd: ");
		Serial.write(rx_data, len-1);
		Serial.println(" ");
		
		char key = rx_data[0];
		switch(key) {
			case 'f':
				cmd_free = true;
				return true;
				break;
			case 'r':
				cmd_rest = true;
				return true;
				break;
			default:
				return false;
				break;
		}
		return true;
	}
	
};


#endif