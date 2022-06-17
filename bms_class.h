#ifndef BMS_CLASS_
#define BMS_CLASS_

#include <USARTClass.h>
#include "comm_utils.h"
#include "ros_comm.h"
#include <stdint.h>

#define BMS_RX_TIMEOUT_MS 1500
#define MAX_BMS_COMM_UPDATE_PERD_MS 15		// by p.38, controller will responde after 4.7ms

enum Bms_Fun {
	BASIC_INFO,				// JG
	CELL_VOLTAGE,
	HARDWARE_VER,
	BMS_FUN_COUNT
};
struct Bms_Pkt_t
{
	int tx_len[Bms_Fun::BMS_FUN_COUNT];
	int rx_len[Bms_Fun::BMS_FUN_COUNT];
	byte ctrl_byte[Bms_Fun::BMS_FUN_COUNT];
};
static Bms_Pkt_t _set_Bms_Pkt(void)
{
	Bms_Pkt_t temp;
	
	temp.tx_len[Bms_Fun::BASIC_INFO] 	= 7;
	temp.tx_len[Bms_Fun::CELL_VOLTAGE]	= 0;
	temp.tx_len[Bms_Fun::HARDWARE_VER]	= 0;
	
	temp.rx_len[Bms_Fun::BASIC_INFO] 	= 34;
	temp.rx_len[Bms_Fun::CELL_VOLTAGE]	= 0;
	temp.rx_len[Bms_Fun::HARDWARE_VER]	= 0;
	
	temp.ctrl_byte[Bms_Fun::BASIC_INFO] 	= 0x03;
	temp.ctrl_byte[Bms_Fun::CELL_VOLTAGE]	= 0x04;
	temp.ctrl_byte[Bms_Fun::HARDWARE_VER]	= 0x05;
	
	return temp;
};
const Bms_Pkt_t Bms_Pkt = _set_Bms_Pkt();

class BMS_Class : public Serial_Comm
{
public:
	BMS_Class(USARTClass &port) : _port(port)
	{
		_comm_state = Comm_Status::STDBY;
		_rx_bfr_len = 0; 
	};
	
	/*********************************************************************
		Name: Init()
		Input:    baudrate
		Ouput:
		Return:   true
	*********************************************************************/
	bool Init(int baudrate){
		_baud = baudrate;
		_port.begin(_baud);
		
		uint8_t rx_len;
		bool ret = true;
		
		delay(100);			// wait for port to begin
		//=== servo off ===
		Serial.write(RosTxHeader.text, RosTxHeader.len);
		Serial.print("--> Battery Check...");
		check_bms();
		tx_flush();			// block until tx is finished
		comm_err_t res = wait_bms_echo(_last_cmd_fun, 100, rx_len);
		if(res!=Comm_Return_Code::NORMAL){
			Serial.print("   comm error: ");
			Serial.println(Comm_Code_text.data[res]);
			ret = false;
		}

		if(!decode_bms(_last_cmd_fun, _rx_buffer, rx_len)){
			Serial.write(RosTxHeader.text, RosTxHeader.len);
			Serial.print("   decode failed");
			Serial.println(Comm_Code_text.data[res]);
			print_dbg(res, _rx_buffer, rx_len);
			ret = false;
		}
		if(ret) {
			Serial.print("success! ");
			Serial.print(_cap_percentage);
			Serial.println("%");
		}
			
			
		delay(700);
		return ret;
	};
	
	void update(const uint32_t now)
	{
	}
	
	bool _new_info_available()
	{
		bool temp = _new_bms_info;
		return temp;
	}
	
	void get_status(int &centi_v, int &centi_a, int &centi_mah, int &percent)
	{
		_new_bms_info = false;
		centi_v = (unsigned int) _c_volt;			// centi-voltage, 10mV
		centi_a = (int)_c_amp;			// centi-ampere, 10mA
		centi_mah = (unsigned int)_c_cap;			// centi-capacity, 10mAh
		percent = (int)_cap_percentage;	// capacity percentage
		return;
	}
	
	comm_err_t update_comm(const uint32_t now)
	{
		int32_t time_since_cmd = now - _rx_bfr_start_clk;
		comm_err_t res = Comm_Return_Code::NORMAL;
				
		if( _comm_state >= Comm_Status::COMM_STATUS_COUNT) {
			
			//Serial.print("[ERR] BMS Comm state error!");
			return Comm_Return_Code::UNKOWN_STATUS;
		}
				
		if( _comm_state == Comm_Status::STDBY) { 
			// do nothing
			return Comm_Return_Code::NORMAL;
		}
			
		if( _comm_state == Comm_Status::TRANSMITTING) {
			if(_port.availableForWrite() == SERIAL_BUFFER_SIZE - 1)
			{
				//_port.flush();				// It seems that the last byte wont get sent if not blocked for a while
				_comm_state = Comm_Status::WAITING_ECHO;
			}
			if( time_since_cmd > MAX_BMS_COMM_UPDATE_PERD_MS)
			{
				//Serial.print("[WARN] BMS Tx freq too low: ");
				//Serial.print(time_since_cmd);
				//Serial.print("ms, avail: ");
				//Serial.println(_port.availableForWrite());
				res = Comm_Return_Code::LOW_TX_FREQ;
			}
		}
			
		if( _comm_state == Comm_Status::WAITING_ECHO ) {
			// check for response time
			if( _port.available() )
			{
				_comm_state = Comm_Status::RECEIVING;
			}else if( time_since_cmd > BMS_RX_TIMEOUT_MS )
			{
				//Serial.println("[ERR] BMS No response timeout!");
				_comm_state = Comm_Status::STDBY;
				_rx_bfr_len = 0;
				return Comm_Return_Code::NO_RX_TIMEOUT;
			}
		}
			
		if( _comm_state == Comm_Status::RECEIVING) {
			// rx timeout
			if( _port.available()==0 && time_since_cmd > BMS_RX_TIMEOUT_MS)
			{
				//Serial.print("[ERR] BMS echo timeout!");
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
			
			if( _rx_bfr_len < Bms_Pkt.rx_len[_last_cmd_fun] ) 
			{
				return res;								// return if not all packets have arrived
			}
			else if( _rx_bfr_len > Bms_Pkt.rx_len[_last_cmd_fun] ) 
			{
				//Serial.print("[ERR] BMS echo too long: ");
				//print_bfr(_rx_buffer, _rx_bfr_len);
				_comm_state = Comm_Status::STDBY;
				_rx_bfr_len = 0;				
				res = Comm_Return_Code::ECHO_TOO_LONG;
			}
			else if( _rx_bfr_len == Bms_Pkt.rx_len[_last_cmd_fun] ) 
			{
				// Get expected bfr length
				bool suc = decode_bms(_last_cmd_fun, _rx_buffer, _rx_bfr_len);
				if(!suc)
				{
					Serial.write(RosTxHeader.text, RosTxHeader.len);
					Serial.print("[ERR] BMS crc failed: ");
					print_bfr(_rx_buffer, _rx_bfr_len);// print buffer content					
					res = Comm_Return_Code::CRC_FAILED;
				}
				if(bms_echo_err) {
					Serial.write(RosTxHeader.text, RosTxHeader.len);
					Serial.print("[ERR] BMS FC echo 0x80: ");
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
		Name: check_bms()
		Input:
		Ouput:
		Return: true if success
		Note: This function will not block.
			  See tx_flush() below for blocked transmission.
	*********************************************************************/
	bool check_bms()
	{
		int8_t tx_len = gen_bms_ask_pkt();	// update _tx_buffer
		_last_cmd_fun = Bms_Fun::BASIC_INFO;
		_port.write(_tx_buffer, tx_len);
		_rx_bfr_start_clk = millis();
		_comm_state = Comm_Status::TRANSMITTING;
		//Serial.print(" Checking, len:");
		//Serial.print(tx_len);
		//Serial.print(", avail: ");
		//Serial.println(_port.availableForWrite());
		
		return true;
	};

	/*********************************************************************
	Name: Wait until tx is finished. At 57600 bps, multi-drive packet
			  (11 bytes) takes ~1.72 ms.
			  This function should be called before the driver starts to
			  respond, that is, at p.38 C3.5+Tb2 = 4.7 ms
	*********************************************************************/
	void tx_flush()
	{
		_port.flush();									// blocked until finish
		_comm_state = Comm_Status::WAITING_ECHO;
	}
	
	
private:
	USARTClass &_port;		// Serial object
	int _baud;					// Serial baudrate
	
	byte _tx_buffer[20];
	byte _rx_buffer[50];
	
	uint8_t _rx_bfr_len;		// Current buffer size
	uint32_t _rx_bfr_start_clk;	// Time the buffer starts;
	
	Comm_Status _comm_state;
	Bms_Fun _last_cmd_fun;	// to expect what echo we're waiting for
	
	bool _new_bms_info;				// new step available
	bool bms_echo_err;
	uint16_t _c_volt;			// centi-voltage, 10mV
	int16_t _c_amp;				// centi-ampere, 10mA
	uint16_t _c_cap;			// centi-capacity, 10mAh
	int16_t _cap_percentage;	// capacity percentage

	/*********************************************************************
		Name: gen_bms_ask_pkt
		Input: 
		Ouput: data via member _tx_buffer
		Return: the length of the message, or -1 if there's any error
		Note: Generate packet for BMS asking function of ????? BMS
	*********************************************************************/
	int8_t gen_bms_ask_pkt()
	{
		uint8_t len;
		
		_tx_buffer[0]= 0xDD;	// header
		_tx_buffer[1]= 0xA5;	// read
		_tx_buffer[2]= Bms_Pkt.ctrl_byte[Bms_Fun::BASIC_INFO]; 	// 03讀取基本資訊及狀態 04讀取單體電壓 
		_tx_buffer[3]= 0x00;	// length
		_tx_buffer[4]= 0xFF;	// checksum H
		_tx_buffer[5]= 0xFD;	// checksum L
		_tx_buffer[6]= 0x77; 	// tailer
		len = Bms_Pkt.tx_len[Bms_Fun::BASIC_INFO];
		//Serial.print(len);
		
		/*uint16_t crc = ModRTU_CRC(_tx_buffer, 9);
		_tx_buffer[len] = (byte)(crc & 0xFF);	// lowbit first
		_tx_buffer[len+1] = (byte)(crc >> 8);
		len += 2;*/
		return (int8_t)len;
	}
	
	/*********************************************************************
		return: the length of the message
			 -1 if there's any error
			 INT8_MIN if timeout
			 -n if CRC check failed with length N message
	*********************************************************************/
	comm_err_t wait_bms_echo(Bms_Fun bms_fun, uint32_t timeout_ms, uint8_t &len)
	{
		uint32_t clk_start = millis();
		
		// Get expected read length
		if( ( bms_fun >= Bms_Fun::BMS_FUN_COUNT ) || (_comm_state != Comm_Status::WAITING_ECHO) )
		{
			_comm_state = Comm_Status::STDBY;
			_rx_bfr_len = 0;
			return Comm_Return_Code::UNKOWN_STATUS;
		}
		
		len = Bms_Pkt.rx_len[bms_fun];
		comm_err_t res;
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
		
		/* check CRC
		if(Check_ModRTU_CRC(_rx_buffer, len))
		{
			ret = len;
		}else {
			ret = -len;
		}*/
		res = Comm_Return_Code::NORMAL;
		_comm_state = Comm_Status::STDBY;
		_rx_bfr_len = 0;
		return res;
	}
	
	bool decode_bms(Bms_Fun mode, byte *bfr, uint8_t len)
	{
		// check CRC
		/*if( !Check_ModRTU_CRC(bfr, len)) {
			return false;
		}*/
		
		switch(mode) {
			case (Bms_Fun::BASIC_INFO):
				bms_dec_basic_info(bfr, len);
				_new_bms_info = true;
				if( bfr[2] == 0x80) {
					bms_echo_err = true;
				}
				break;
			case (Bms_Fun::CELL_VOLTAGE):			// FC = 0x06
				break;
			case (Bms_Fun::HARDWARE_VER):	// FC = 0x10
				break;
			default:
				return false;
		}
		return true;
	};
	
	/*********************************************************************
		Name: bms_dec_basic_info
		Input: bfr: buffer / rx_len: length to print
		Ouput:
		Return: false if buffer length is insufficient
		Note: Decode basic BMS info
	*********************************************************************/
	bool bms_dec_basic_info(byte *bfr, int8_t len)
	{
		if(len < 24) return false;
		
		_c_volt = ((uint16_t)bfr[4]<<8) | ((uint16_t)bfr[5]);// centi-voltage, 10mV
		_c_amp = ((int16_t)bfr[6]<<8) | ((int16_t)bfr[7]);// centi-ampere, 10mA
		_c_cap = ((uint16_t)bfr[8]<<8) | ((uint16_t)bfr[9]);// centi-capacity, 10mAh
		_cap_percentage = bfr[23];// capacity percentage
		
		return true;
	};
};

#endif