#ifndef SYY_GPIO_H_
#define SYY_GPIO_H_

class SyyGpio {
public:
	/// Constructor
    SyyGpio(int ch, uint16_t debounce): 
		_pin(ch),
		_debounce_cnt(debounce)
	{
    }
	
	void Init()
	{
		pinMode(_pin, INPUT_PULLUP);
		_last_state = HIGH;
		_counter = 0;
		_rising = 0;
		_falling = 0;
		return;
	}
	
	void update()
	{
		int pin_state = digitalRead(_pin);
		if( pin_state != _last_state )	// edge detection
		{
			_counter++;
			if(_counter >= _debounce_cnt)
			{
				_counter = 0;
				_last_state = pin_state;
				if(pin_state == HIGH)	{				
					_rising++;
				}else {
					_falling++;
				}
			}
		}
		return;
	}
	
	bool is_high()
	{
		if( _last_state == HIGH ) {
			return true;
		} else {
			return false;
		}
	}
	
	bool is_low()
	{
		if( _last_state == LOW ) {
			return true;
		} else {
			return false;
		}
	}
	
	bool is_rising()
	{
		if(_rising != 0) {
			_rising = 0;	// reset
			return true;
		}
		return false;
	}

	bool is_falling()
	{
		if(_falling != 0) {
			_falling = 0;	// reset
			return true;
		}
		return false;
	}	
private:
	const int _pin;
	const int _debounce_cnt;
	int _counter;
	int _last_state;
	unsigned int _rising, _falling;
};	
#endif