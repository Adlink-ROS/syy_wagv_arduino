#ifndef MODRTU_CRC_H_
#define MODRTU_CRC_H_
 
// Compute the MODBUS RTU CRC
uint16_t ModRTU_CRC(byte* buf, int len)
{
	uint16_t crc = 0xFFFF;

	for (int pos = 0; pos < len; pos++) {
	  crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc

	  for (int i = 8; i != 0; i--) {    // Loop over each bit
		if ((crc & 0x0001) != 0) {      // If the LSB is set
		  crc >>= 1;                    // Shift right and XOR 0xA001
		  crc ^= 0xA001;
		}
		else                            // Else LSB is not set
		  crc >>= 1;                    // Just shift right
	  }
	}
	// Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
	return crc;  
}

// Check last two byte for CRC
bool Check_ModRTU_CRC(byte* buf, int len)
{
	uint16_t rx_crc = (uint16_t)buf[len-1]<<8 | (buf[len-2]);
	if( rx_crc == ModRTU_CRC(buf, len-2) ) {
		return true;		// CRC check pass
	}else {
		return false;		// CRC check fail
	}
}

#endif