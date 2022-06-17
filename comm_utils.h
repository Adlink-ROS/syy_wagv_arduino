#ifndef COMM_UTILS_H_
#define COMM_UTILS_H_
//#include <String.h>
enum Comm_Status {
	STDBY,
	TRANSMITTING,
	WAITING_ECHO,
	RECEIVING,
	COMM_STATUS_COUNT
};

enum RX_Status {
	RX_STDBY,
	RX_SYNCING,
	RX_RECEIVING,
	RX_STATUS_COUNT
};

enum Comm_Return_Code {
	NORMAL,
	UNKOWN_STATUS,
	LOW_TX_FREQ,
	NO_RX_TIMEOUT,
	ECHO_TIMEOUT,
	ECHO_TOO_LONG,
	CRC_FAILED,
	FUNCTION_ERROR,
	Comm_Return_Code_COUNT
};
typedef Comm_Return_Code comm_err_t;

struct Comm_Return_Code_Text_t {
	String data[Comm_Return_Code_COUNT];
};
static Comm_Return_Code_Text_t _set_Comm_Return_Code_Text(void)
{
	Comm_Return_Code_Text_t temp;
	temp.data[Comm_Return_Code::NORMAL] = "Comm OK";
	temp.data[Comm_Return_Code::UNKOWN_STATUS] = "Status Error";
	temp.data[Comm_Return_Code::LOW_TX_FREQ] = "Low Tx freq";
	temp.data[Comm_Return_Code::NO_RX_TIMEOUT] = "Timeout: No rx";
	temp.data[Comm_Return_Code::ECHO_TIMEOUT] = "Timeout: echo short";
	temp.data[Comm_Return_Code::ECHO_TOO_LONG] = "Rx too long!";
	temp.data[Comm_Return_Code::CRC_FAILED] = "CRC failed";
	temp.data[Comm_Return_Code::FUNCTION_ERROR] = "Slave error code";
	return temp;
};
const static Comm_Return_Code_Text_t Comm_Code_text = _set_Comm_Return_Code_Text();
class Serial_Comm {
public:
	/*********************************************************************
		===DEBUG FUN===
		Name: print_dbg()
		          print hex through serial port
		Input: bfr: buffer / rx_len: length to print
		Ouput:
		Return:
	*********************************************************************/
	void print_bfr(const byte *bfr, const uint8_t len)
	{
		String L;
		L = String("len:") + len + " pkt[";
		for(int k=0 ; k<abs(len) ; k++) {
			L += String(bfr[k],HEX) + " ";
		}
		L += "] ";
		Serial.println(L);
	}
	
	/*********************************************************************
		===DEBUG FUN===
		Name: print_dbg()
		          print hex through serial port
		Input: bfr: buffer / rx_len: length to print
		Ouput:
		Return:
	*********************************************************************/
	void print_dbg(comm_err_t comm_st, byte *bfr, const int8_t rx_len)
	{
		Serial.print(Comm_Code_text.data[comm_st]+" ");
		
		if( (comm_st == Comm_Return_Code::NORMAL)||
		    (comm_st >= Comm_Return_Code::ECHO_TIMEOUT) )
		{
			print_bfr(bfr, rx_len);
		}
	}
private:
};

#endif