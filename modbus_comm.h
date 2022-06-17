#ifndef MODBUS_COMM_H_
#define MODBUS_COMM_H_

#include <USARTClass.h>
#include <stdint.h>
//#include <String>

#include "syy_wagv_vehicle_config.h"
#include "comm_utils.h"
#include "modrtu_crc.h"
#include "ros_comm.h"

#define RS485Transmit    HIGH
#define RS485Receive     LOW

#define RTU_RX_TIMEOUT_MS 30
#define MAX_COMM_UPDATE_PERD_MS 5	// by p.38, controller will responde after 4.7ms

enum EV_Fun {
	DRIVE,				// JG
	SERVO_FREE,
	SERVO_ON,
	SERVO_OFF
};

enum Comm_Mode {
	MULTI_DRV,			// FC = 0x65
	READ_REG_MULTI,		// FC = 0x03
	WRITE_REG,			// FC = 0x06
	WRITE_REG_MULTI,	// FC = 0x10
	COMM_MODE_COUNT
};

struct Rtu_Pkt_Length_t
{
	int tx[Comm_Mode::COMM_MODE_COUNT];
	int rx[Comm_Mode::COMM_MODE_COUNT];
};

static Rtu_Pkt_Length_t _set_Rtu_Pkt_Length(void)
{
	Rtu_Pkt_Length_t temp;
	
	temp.tx[Comm_Mode::MULTI_DRV] 		= 11;	// p.31
	temp.tx[Comm_Mode::READ_REG_MULTI]	= 8;	// p.41
	temp.tx[Comm_Mode::WRITE_REG]		= 8;	// p.41
	temp.tx[Comm_Mode::WRITE_REG_MULTI] = 9;	// p.42 = (9+write_count*2)
	
	temp.rx[Comm_Mode::MULTI_DRV] 		= 8;	// p.31
	temp.rx[Comm_Mode::READ_REG_MULTI]	= 5;	// p.41 = (5+read_count*2)
	temp.rx[Comm_Mode::WRITE_REG]		= 8;	// p.41
	temp.rx[Comm_Mode::WRITE_REG_MULTI] = 8;	// p.42
	return temp;
};
Rtu_Pkt_Length_t Rtu_Pkt_Length = _set_Rtu_Pkt_Length();

// p.30
struct MultiDriveCMD_t 
{
	byte ISTOP;
	byte JG;
	byte FREE;
	byte SVON;
	byte SVOFF;
	byte IMR;
	byte MR;
	byte MA;
	byte CS;
	byte CMR;
	byte CMA;
	byte NUL;
};

static MultiDriveCMD_t _set_MultiDriveCMD(void)
{
	MultiDriveCMD_t temp;
	temp.ISTOP = 0x00;
	temp.JG = 0x0A;
	temp.FREE = 0x05;
	temp.SVON = 0x06;
	temp.SVOFF = 0x07;
	temp.IMR = 0x08;
	temp.MR = 0x0C;
	temp.MA = 0x0D;
	temp.CS = 0x0E;
	temp.CMR = 0x0F;
	temp.CMA = 0x10;
	temp.NUL = 0x63;
	return temp;
};

static MultiDriveCMD_t MultiDriveCMD = _set_MultiDriveCMD();

class ModbusComm : public Serial_Comm
{
public:
	ModbusComm(USARTClass &port, int ctl_ch, int tx_ch) : _port(port), _serial_tx_ctrl(ctl_ch), _tx_ch(tx_ch)
	{
		_comm_state = Comm_Status::STDBY;
		_rx_bfr_len = 0; 
		_new_step = false;
		_multi_drive_echo_err = false;
		_alive = false;
	};
	
	/*********************************************************************
		Name: Init()
		Input:    sub_id: Modbus RTU ID / baudrate
		Ouput:
		Return:   true
	*********************************************************************/
	bool Init(uint8_t sub_id, int baudrate){
		//=== Init pins ===
		pinMode(_serial_tx_ctrl, OUTPUT);  				// Transceiver control pin
		digitalWrite(_serial_tx_ctrl, RS485Receive);	// Tx mode off
		
		_baud = baudrate;
		_port.begin(_baud);
		//delay(10);
		_port.flush();
		delay(100);					// wait for port to begin
		while(_port.available()) _port.read(); // clear input buffer
		_modbus_id = sub_id;
		
		
		uint8_t rx_len;
		//=== servo off ===
		Serial.write(RosTxHeader.text, RosTxHeader.len);
		Serial.print("   servo off, ");
		servo_off();
		cmd_tx_flush();							// block until tx is finished
		delay(5); 								// takes about 7ms to respond according to p.39
		comm_err_t res = wait_rtu_echo(Comm_Mode::MULTI_DRV, RTU_RX_TIMEOUT_MS*2, rx_len);
		if(res == Comm_Return_Code::NORMAL) {
			decode_rtu(Comm_Mode::MULTI_DRV, _rx_buffer, rx_len);
		}
		print_dbg(res, _rx_buffer, rx_len);			// debug print

		delay(1000);
		
		//=== servo on ===
		Serial.write(RosTxHeader.text, RosTxHeader.len);
		Serial.print("   servo on, ");
		servo_on();
		cmd_tx_flush();
		delay(5); 								// takes about 7ms to respond according to p.39
		res = wait_rtu_echo(Comm_Mode::MULTI_DRV, RTU_RX_TIMEOUT_MS*2, rx_len);
		print_dbg(res, _rx_buffer, rx_len);			// debug print
		if(res == Comm_Return_Code::NORMAL){
			decode_rtu(Comm_Mode::MULTI_DRV, _rx_buffer, rx_len);
			_alive = true;
			return true;
		}else {
			Serial.write(RosTxHeader.text, RosTxHeader.len);
			Serial.println("Motor init failed, please restart");
			return false;
		}
	};
	
	bool servo_on(void)
	{
		return cmd_servo_state(EV_Fun::SERVO_ON);
	};
	
	bool servo_off(void)
	{
		return cmd_servo_state(EV_Fun::SERVO_OFF);
	};
	
	bool servo_free(void)
	{
		return cmd_servo_state(EV_Fun::SERVO_FREE);
	};
	
	bool servo_drive(int16_t rpm)
	{
		bool res = cmd_servo_rpm(EV_Fun::DRIVE, rpm);
		return res;
	}
	
	comm_err_t update(const uint32_t now)
	{
		return update_comm(now);
	}
	
	bool step_available()
	{
		return _new_step;
	}
	
	int32_t get_step()
	{
		_new_step = false;
		return _multi_drive_step;
	}
	
	bool is_alive()
	{
		return _alive;
	}
	
	comm_err_t update_comm(uint32_t now)
	{
		int32_t time_since_cmd = now - _rx_bfr_start_clk;
		comm_err_t res = Comm_Return_Code::NORMAL;
		
		if( _comm_state >= Comm_Status::COMM_STATUS_COUNT) {
			//Serial.println("[ERR] Comm state error!");
			return Comm_Return_Code::UNKOWN_STATUS;
		}
				
		if( _comm_state == Comm_Status::STDBY ) {
			// do nothing
			return Comm_Return_Code::NORMAL;
		}
		
		if( _comm_state == Comm_Status::TRANSMITTING ) {
			if(_port.availableForWrite() == SERIAL_BUFFER_SIZE - 1)
			{
				_port.flush();				// It seems that the last byte wont get sent if not blocked for a while
				digitalWrite(_serial_tx_ctrl, RS485Receive);	// tx mode off
				_comm_state = Comm_Status::WAITING_ECHO;
				_rx_bfr_start_clk = now;
			}
			if( time_since_cmd > MAX_COMM_UPDATE_PERD_MS)
			{
				//Serial.println(time_since_cmd);
				res = Comm_Return_Code::LOW_TX_FREQ;
			}
		}
		
		if( _comm_state == Comm_Status::WAITING_ECHO ) {
			// check for response time
			if( _port.available() )
			{
				_comm_state = Comm_Status::RECEIVING;
			}else if( time_since_cmd > RTU_RX_TIMEOUT_MS )
			{
				//Serial.println("[ERR] RTU No response timeout!");
				_comm_state = Comm_Status::STDBY;
				_rx_bfr_len = 0;
				return Comm_Return_Code::NO_RX_TIMEOUT;
			}
		}
		
		if( _comm_state == Comm_Status::RECEIVING ) {
			// rx timeout
			if( _port.available()==0 && time_since_cmd > RTU_RX_TIMEOUT_MS)
			{
				//Serial.print("[ERR] RTU echo timeout!");
				//print_bfr(_rx_buffer, _rx_bfr_len);
				_comm_state = Comm_Status::STDBY;
				_rx_bfr_len = 0;
				return Comm_Return_Code::ECHO_TIMEOUT;
			}
				
			// read in all bytes in the buffer
			while( _port.available() )
			{
				_rx_buffer[_rx_bfr_len] = _port.read();   // Read the byte
				_rx_bfr_len++;
			}
			
			// do something with the copied data
			if( _rx_bfr_len < Rtu_Pkt_Length.rx[_last_cmd_mode] ) 
			{
				return res;							// return if not all packets have arrived
			}
			else if( _rx_bfr_len > Rtu_Pkt_Length.rx[_last_cmd_mode] ) 
			{
				//Serial.print("[ERR] RTU echo too long: ");
				//print_bfr(_rx_buffer, _rx_bfr_len);
				_comm_state = Comm_Status::STDBY;
				_rx_bfr_len = 0;
				res = Comm_Return_Code::ECHO_TOO_LONG;
			}
			else if( _rx_bfr_len == Rtu_Pkt_Length.rx[_last_cmd_mode] ) 
			{
				// Get expected bfr length
				bool suc = decode_rtu(_last_cmd_mode, _rx_buffer, _rx_bfr_len);
				if(!suc)
				{
					//Serial.print("[ERR] RTU crc failed: ");
					Serial.write(RosTxHeader.text, RosTxHeader.len);
					print_bfr(_rx_buffer, _rx_bfr_len);// print buffer content
					_port.read();		// to prevent unsynced situation
					res = Comm_Return_Code::CRC_FAILED;
				}
				if(_multi_drive_echo_err) {
					//Serial.print("[ERR] Multi-Drive FC echo 67h: ");
					Serial.write(RosTxHeader.text, RosTxHeader.len);
					print_bfr(_rx_buffer, _rx_bfr_len);// print buffer content
					res = Comm_Return_Code::FUNCTION_ERROR;
				}
				_comm_state = Comm_Status::STDBY;
				_rx_bfr_len = 0;
			}
		}

		return res;
	}
	
	
	/*********************************************************************
	Name: Wait until tx is finished. At 57600 bps, multi-drive packet
			  (11 bytes) takes ~1.72 ms.
			  This function should be called before the driver starts to
			  respond, that is, at p.38 C3.5+Tb2 = 4.7 ms
	*********************************************************************/
	void cmd_tx_flush()
	{
		_port.flush();									// blocked until finish
		digitalWrite(_serial_tx_ctrl, RS485Receive);	// tx mode off
		_comm_state = Comm_Status::WAITING_ECHO;
	}
	
	/*********************************************************************
		===DEBUG FUN===
		Name: cmd_vel()
		Input: rpm: motor rpm
		Ouput:
		Return: true if success
	*********************************************************************/
	/*bool cmd_rpm_blocked(int16_t rpm) {
		uint8_t rx_len;
		rpm = constrain(rpm, -MAX_MOTOR_RPM, MAX_MOTOR_RPM);
		
		Serial.print(" move: ");
		Serial.print(rpm);

		servo_drive(rpm);
		cmd_tx_flush();
		
		delay(5); 		// takes about 5+1.7ms to respond according to p.38
		comm_err_t res = wait_rtu_echo(Comm_Mode::MULTI_DRV, 100, rx_len);

		if(res != Comm_Return_Code::NORMAL)
		{
			print_dbg(res, _rx_buffer, rx_len);
			return false;
		}
		int32_t step;
		bool success = multidrive_dec_pos(_rx_buffer, rx_len, step);
		_new_step = true;
		Serial.print(" motor step: ");
		Serial.println(step);
		return true;
	};*/
	
	
private:
	USARTClass &_port;		// Serial object
	int _baud;					// Serial baudrate
	const int _serial_tx_ctrl;	// 485 module tx/rx control pin
	const int _tx_ch;
	uint8_t _modbus_id;			// modbus ID of motor controller
	
	byte _tx_buffer[20];
	byte _rx_buffer[50];
	
	uint8_t _rx_bfr_len;		// Current buffer size
	uint32_t _rx_bfr_start_clk;	// Time the buffer starts;
	
	Comm_Status _comm_state;
	Comm_Mode _last_cmd_mode;	// to expect what echo we're waiting for
	
	int32_t _multi_drive_step;	// multi-drive echo: step
	bool _new_step;				// new step available
	bool _multi_drive_echo_err;
	bool _alive;				// liveliness of the motor
	
	//byte _svon[] = {0x00, 0x65, 0x01, 0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x55, 0xF6};
	//byte _svoff[] = {0x00, 0x65, 0x01, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x68, 0x36};
	//byte _temphead[] = {0x00, 0x65, 0x01, 0x01, 0x0A, 0x00, 0x00}; //RUN
	//byte _softack[] = {0x01, 0x03, 0x80, 0x00, 0x00, 0x05, 0xAC, 0x09}; //PCsoftware ACK
	
	/*********************************************************************
		Name: cmd_servo_state / cmd_servo_rpm
		Input: [state] / ([rpm]) / [_tx_buffer]
		Ouput: [_comm_state] [_last_cmd_mode] [_rx_bfr_start_clk]
		Return: true if success
		Note: Multi-drive function
		      These functions will not block.
			  See cmd_tx_flush() below for blocked transmission.
	*********************************************************************/
	bool cmd_servo_state(EV_Fun state)
	{
		if(_comm_state != Comm_Status::STDBY)
		{
			return false;
		}
		_last_cmd_mode = Comm_Mode::MULTI_DRV;
		int8_t tx_len = gen_multi_drive_rtu_pkt(state, 0, 0);	// update _tx_buffer
		
		digitalWrite(_serial_tx_ctrl, RS485Transmit);	// tx mode
		_port.write(_tx_buffer, tx_len);
		_rx_bfr_start_clk = millis();
		_comm_state = Comm_Status::TRANSMITTING;
		
		return true;
	};
	
	bool cmd_servo_rpm(EV_Fun state, int16_t rpm)
	{
		if(state != EV_Fun::DRIVE) return false;

		_last_cmd_mode = Comm_Mode::MULTI_DRV;
		rpm = constrain(rpm, -MAX_MOTOR_RPM, MAX_MOTOR_RPM);
		int8_t tx_len = gen_multi_drive_rtu_pkt(state, 0, (uint16_t)rpm);
		
		digitalWrite(_serial_tx_ctrl, RS485Transmit);	// tx mode on
		_port.write(_tx_buffer, tx_len);
		_rx_bfr_start_clk = millis();
		_comm_state = Comm_Status::TRANSMITTING;
		return true;
	};
	
	/*********************************************************************
		Name: gen_multi_drive_rtu_pkt
		Input: [state] / [state] + [rpm]
		Ouput: [_tx_buffer]
		Return: Length of the message, -1 if there's any error
		Note: Generate packet for Multi-Drive function of DEXMART
		      EV controller
	*********************************************************************/
	int8_t gen_multi_drive_rtu_pkt(EV_Fun motor_fun, uint16_t data1, uint16_t data2)
	{
		uint8_t len;
		_tx_buffer[0] = 0x00;		// broadcast
		_tx_buffer[1] = 0x65;		// multi-drive p.29
		_tx_buffer[2] = 0x01;		// SubID(slave) count
		_tx_buffer[3] = 0x01;		// SubID 1
		
		switch(motor_fun)
		{
			case EV_Fun::SERVO_ON:
				_tx_buffer[4] = MultiDriveCMD.SVON; // CMD code
				_tx_buffer[5] = 0x00;
				_tx_buffer[6] = 0x00;
				_tx_buffer[7] = 0x00;
				_tx_buffer[8] = 0x00;
				len = 9;
				break;
			case EV_Fun::SERVO_OFF:
				_tx_buffer[4] = MultiDriveCMD.SVOFF; // CMD code
				_tx_buffer[5] = 0x00;
				_tx_buffer[6] = 0x00;
				_tx_buffer[7] = 0x00;
				_tx_buffer[8] = 0x00;
				len = 9;
				break;
			case EV_Fun::DRIVE:
				_tx_buffer[4] = MultiDriveCMD.JG; // CMD code
				_tx_buffer[5] = 0x00;
				_tx_buffer[6] = 0x00;
				_tx_buffer[7] = (byte)(data2 >> 8);
				_tx_buffer[8] = (byte)(data2 & 0xFF);
				len = 9;
				break;
			case EV_Fun::SERVO_FREE:
				_tx_buffer[4] = MultiDriveCMD.FREE; // CMD code
				_tx_buffer[5] = 0x00;
				_tx_buffer[6] = 0x00;
				_tx_buffer[7] = 0x00;
				_tx_buffer[8] = 0x00;
				len = 9;
				break;	
			default:
				return -1;
		}
		uint16_t crc = ModRTU_CRC(_tx_buffer, 9);
		_tx_buffer[len] = (byte)(crc & 0xFF);	// lowbit first
		_tx_buffer[len+1] = (byte)(crc >> 8);
		len += 2;
		return len;
	}
	
	/*********************************************************************
		Name: wait_rtu_echo
		Input: [mode] RTU target mode /[timeout_ms]
		Ouput: [_rx_bfr_len] [_comm_state] [_rx_buffer] [len]
		Return: [n]        Length of the message
		        [-1]       Unknown comm_mode or incorrect _comm_state
		        [INT8_MIN] Timeout
		        [-n]       CRC check failed with length N message
		Note: Currently only decodes Multi-Drive packet
	*********************************************************************/
	comm_err_t wait_rtu_echo(Comm_Mode mode, uint32_t timeout_ms, uint8_t &len)
	{
		uint32_t clk_start = millis();
		
		
		// Get expected read length
		if( ( mode >= Comm_Mode::COMM_MODE_COUNT ) || (_comm_state != Comm_Status::WAITING_ECHO) )
		{
			_comm_state = Comm_Status::STDBY;
			_rx_bfr_len = 0;
			return Comm_Return_Code::UNKOWN_STATUS;
		}
		
		len = Rtu_Pkt_Length.rx[mode];
		comm_err_t ret;
		_comm_state = Comm_Status::RECEIVING;
		
		// Read with blocking
		for(int i=0 ; i<len ; i++)
		{
			// block until signal comes, and check for timeout
			while( !_port.available() ) {
				if((millis() - clk_start > timeout_ms)) {
					_comm_state = Comm_Status::STDBY;
					_rx_bfr_len = 0;
					return Comm_Return_Code::NO_RX_TIMEOUT;
				}
			}
			_rx_buffer[i] = _port.read();   // Read the byte
		}
		
		// check CRC
		if(!Check_ModRTU_CRC(_rx_buffer, len))
		{
			ret = Comm_Return_Code::CRC_FAILED;
		}else {
			ret = Comm_Return_Code::NORMAL;
		}

		_comm_state = Comm_Status::STDBY;
		_rx_bfr_len = 0;
		return ret;
	}
	
	/*********************************************************************
		Name: decode_rtu
		Input: [mode] RTU target mode /[bfr] buffer /[rx_len] msg length
		Ouput: [_multi_drive_step] [_new_step] [multi_drive_echo_err]
		Return: false if packet is unknown or CRC failed
		Note: Currently only decodes Multi-Drive packet
	*********************************************************************/
	bool decode_rtu(Comm_Mode mode, byte *bfr, uint8_t len)
	{
		// check CRC
		if( !Check_ModRTU_CRC(bfr, len)) {
			return false;
		}
		
		switch(mode) {
			case (Comm_Mode::MULTI_DRV):
				multidrive_dec_pos(bfr, len, _multi_drive_step);
				_new_step = true;
				if( bfr[1] == 0x67) {
					_multi_drive_echo_err = true;
				}else {
					_multi_drive_echo_err = false;
				}
				break;
			case (Comm_Mode::READ_REG_MULTI):		// FC = 0x03
				break;
			case (Comm_Mode::WRITE_REG):			// FC = 0x06
				break;
			case (Comm_Mode::WRITE_REG_MULTI):		// FC = 0x10
				break;
			case (Comm_Mode::COMM_MODE_COUNT):
				break;
			default:
				return false;
		}
		return true;
	};
	
	/*********************************************************************
		Name: multidrive_dec_pos
		Input: bfr: buffer / rx_len: length to print
		Ouput:
		Return: false if buffer length is insufficient
		Note: Decode motor position in multi-drive packet.
	*********************************************************************/
	bool multidrive_dec_pos(byte *bfr, int8_t len, int32_t &step)
	{
		if(len < 6) return false;
		step = ((int32_t)bfr[2]<<24) | ((int32_t)bfr[3]<<16);
		step |= ((int32_t)bfr[4]<<8) | ((int32_t)bfr[5]);
		return true;
	};
	
	
};

#endif
