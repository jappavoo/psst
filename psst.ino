/* PSST */

#include <Wire.h>
#include "RTClib.h"
#include "Adafruit_DRV2605.h"
#include "Bounce2.h"
#include <bluefruit.h>

const int redLedPin = LED_RED;    // LED connected to digital pin 9
const int blueLedPin = LED_BLUE;

class  EventState {
  volatile bool button_;
  volatile bool rtcAlarm_;
public:
  EventState() : button_(false), rtcAlarm_(false) {}

  void buttonTriggered() {  button_ = true; }
  void ackButton() { button_ = false; }
  bool button() { return button_; }

  void RTCAlarmTriggered() { rtcAlarm_ = true; }
  void ackRTCAlarm() {  rtcAlarm_ = false; }
  bool RTCAlarm() { return rtcAlarm_; }

  void clearAll() { ackButton(); ackRTCAlarm(); }
  void print() { Serial.print(F("Events: "));
    if (button_) Serial.print(F("B "));
    if (rtcAlarm_) Serial.print(F("A "));
    Serial.println();
  }
};

EventState Events;

class ActivityState {
  uint8_t bits_;
  enum BITS { BUTTON=1<<0, HAPTIC=1<<1, RTC=1<<2, BLUETOOTH=1<<3, OTHER=1<<4 };
public:
  ActivityState() : bits_(0) {}
  void setButton() { bits_ |= BUTTON; }
  void clearButton() { bits_ &= ~BUTTON; }
  int isButtonActive() { return bits_ & BUTTON; }
  
  void setHaptic() { bits_ |= HAPTIC; }
  void clearHaptic() { bits_ &= ~HAPTIC; }
  int isHapticActive() { return bits_ & HAPTIC; }
  
  void setRTC()    { bits_ |= RTC; }
  void clearRTC()    { bits_ &= ~RTC; }
  int isRTCActive()  { return bits_ & RTC; }
  
  void setBlueTooth() { bits_ |= BLUETOOTH; }
  void clearBlueTooth() { bits_ &= ~BLUETOOTH; }
  
  void setOther() { bits_ |= OTHER; }  
  void clearOther() { bits_ &= ~OTHER; }

  void clearAll() {  bits_ = 0; }

  int isActive() { return bits_; }
  int isIdle()   { return bits_ == 0; }
};

ActivityState Activity;

// Each major hardware device has its own class

class ButtonDevice {
  static const int buttonPin = A4;
  void (*cb_)(void);
  Bounce debouncer_;
  int state_;
  unsigned long pressTimeStamp_;
  bool longPress_;

  static void disableInterrupt() { detachInterrupt(buttonPin); }
  
  static void CallBack(void) {
    disableInterrupt();
    Events.buttonTriggered();
  }

  void shortPressAction();
  void longPressAction();
  
public:
  ButtonDevice() { cb_ = NULL; debouncer_ = Bounce(); };
  int setup() {
    pinMode(buttonPin, INPUT_PULLUP);
    digitalWrite(buttonPin, HIGH);
    debouncer_.attach(buttonPin);
    debouncer_.interval(5);
    setButtonCallBack(CallBack);
    return 1;
  }
  
  void setButtonCallBack(void (*cb)(void)) {
    cb_ = cb;
  }
  
  void enableInterrupt() { attachInterrupt(buttonPin, cb_, FALLING); }
    
  int loopActivity();
  
};

const char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

class RTCDevice : public RTC_DS3231 {
  static const uint8_t  CLOCK_ADDRESS=0x68;
  static const int interruptPin = A2;
  void (*cb_)(void);

  static void disableInterrupt() { detachInterrupt(interruptPin); }
  
  static void CallBack(void) {
    // disableInterrupt();
    Events.RTCAlarmTriggered();
  }

public:
  int setup() {
    pinMode(interruptPin, INPUT_PULLUP);
    digitalWrite(interruptPin, HIGH);
    setAlarmCallBack(CallBack);
    // clear alarm interrupt flags
    int rc = RTC_DS3231::begin();
    writeSqwPinMode(DS3231_OFF);
    turnOffAlarm(1);
    turnOffAlarm(2);
    checkIfAlarm(1);
    checkIfAlarm(2);

    // Attach interrupt we do this once as we use device
    // Ack to avoid multiple interrupts.
    enableInterrupt();
   
    return rc;
  }

  void setAlarmCallBack(void (*cb)(void)) {
    cb_ = cb;
  }

  void enableInterrupt() { attachInterrupt(interruptPin, cb_, FALLING); }

  int loopActivity();

  // Alarm functions		
  void getA1Time(byte& A1Day, byte& A1Hour, byte& A1Minute, byte& A1Second, byte& AlarmBits, bool& A1Dy, bool& A1h12, bool& A1PM); 
  /* Retrieves everything you could want to know about alarm
   * one. 
   * A1Dy true makes the alarm go on A1Day = Day of Week,
   * A1Dy false makes the alarm go on A1Day = Date of month.
   *
   * byte AlarmBits sets the behavior of the alarms:
   *	Dy	A1M4	       A1M3	       A1M2	A1M1	Rate
   *	X	1		1		1		1		Once per second
   *	X	1		1		1		0		Alarm when seconds match
   *	X	1		1		0		0		Alarm when min, sec match
   *	X	1		0		0		0		Alarm when hour, min, sec match
   *	0	0		0		0		0		Alarm when date, h, m, s match
   *	1	0		0		0		0		Alarm when DoW, h, m, s match
   *
   *	Dy	A2M4	A2M3	A2M2	Rate
   *	X	1		1		1		Once per minute (at seconds = 00)
   *	X	1		1		0		Alarm when minutes match
   *	X	1		0		0		Alarm when hours and minutes match
   *	0	0		0		0		Alarm when date, hour, min match
   *	1	0		0		0		Alarm when DoW, hour, min match
   */
  void getA2Time(byte& A2Day, byte& A2Hour, byte& A2Minute, byte& AlarmBits, bool& A2Dy, bool& A2h12, bool& A2PM); 
  // Same as getA1Time();, but A2 only goes on seconds == 00.
  void setA1Time(byte A1Day, byte A1Hour, byte A1Minute, byte A1Second, byte AlarmBits, bool A1Dy, bool A1h12, bool A1PM); 
  // Set the details for Alarm 1
  void setA2Time(byte A2Day, byte A2Hour, byte A2Minute, byte AlarmBits, bool A2Dy, bool A2h12, bool A2PM); 
  // Set the details for Alarm 2
  void turnOnAlarm(byte Alarm); 
  // Enables alarm 1 or 2 and the external interrupt pin.
  // If Alarm != 1, it assumes Alarm == 2.
  void turnOffAlarm(byte Alarm); 
  // Disables alarm 1 or 2 (default is 2 if Alarm != 1);
  // and leaves the interrupt pin alone.
  bool checkAlarmEnabled(byte Alarm); 
  // Returns T/F to indicate whether the requested alarm is
  // enabled. Defaults to 2 if Alarm != 1.
  bool checkIfAlarm(byte Alarm); 
  // Checks whether the indicated alarm (1 or 2, 2 default);
  // has been activated.
  
  void printAlarmInfo() {
    byte ADay, AHour, AMinute, ASecond, ABits;
    bool ADy, A12h, Apm;
    
    Serial.print("Alarm 1: ");
    getA1Time(ADay, AHour, AMinute, ASecond, ABits, ADy, A12h, Apm);
    Serial.print(ADay, DEC);
    if (ADy) {
      Serial.print(" DoW");
    } else {
      Serial.print(" Date");
    }
    Serial.print(" ");
    Serial.print(AHour, DEC);
    Serial.print(" ");
    Serial.print(AMinute, DEC);
    Serial.print(" ");
    Serial.print(ASecond, DEC);
    Serial.print(" ");
    if (A12h) {
      if (Apm) {
	Serial.print("pm ");
      } else {
	Serial.print("am ");
      }
    }
    if (checkAlarmEnabled(1)) {
      Serial.print("enabled");
    }

    // Display Alarm 2 information
    Serial.print("\nAlarm 2: ");
    getA2Time(ADay, AHour, AMinute, ABits, ADy, A12h, Apm);
    Serial.print(ADay, DEC);

    if (ADy) Serial.print(" DoW "); else Serial.print(" Date ");

    Serial.print(AHour, HEX);
    Serial.print(" ");
    Serial.print(AMinute, DEC);
    Serial.print(" ");
    if (A12h) {
      if (Apm) {
	Serial.print("pm");
      } else {
	Serial.print("am");
      }
    }
    if (checkAlarmEnabled(2)) {
      Serial.print("enabled");
    }
    // display alarm bits
    Serial.print("\nAlarm bits: ");
    Serial.println(ABits, BIN);
  }
      
private:
  byte decToBcd(byte val); 
  // Convert normal decimal numbers to binary coded decimal
  byte bcdToDec(byte val); 
  // Convert binary coded decimal to normal decimal numbers
  byte readControlByte(bool which); 
  // Read selected control byte: (0); reads 0x0e, (1) reads 0x0f
  void writeControlByte(byte control, bool which); 
			// Write the selected control byte. 
			// which == false -> 0x0e, true->0x0f.
};

byte RTCDevice::decToBcd(byte val) {
// Convert normal decimal numbers to binary coded decimal
	return ( (val/10*16) + (val%10) );
}

byte RTCDevice::bcdToDec(byte val) {
// Convert binary coded decimal to normal decimal numbers
	return ( (val/16*10) + (val%16) );
}

byte RTCDevice::readControlByte(bool which) {
	// Read selected control byte
	// first byte (0) is 0x0e, second (1) is 0x0f
	Wire.beginTransmission(CLOCK_ADDRESS);
	if (which) {
		// second control byte
		Wire.write(0x0f);
	} else {
		// first control byte
		Wire.write(0x0e);
	}
	Wire.endTransmission();
	Wire.requestFrom(CLOCK_ADDRESS, 1);
	return Wire.read();	
}

void RTCDevice::writeControlByte(byte control, bool which) {
	// Write the selected control byte.
	// which=false -> 0x0e, true->0x0f.
	Wire.beginTransmission(CLOCK_ADDRESS);
	if (which) {
		Wire.write(0x0f);
	} else {
		Wire.write(0x0e);
	}
	Wire.write(control);
	Wire.endTransmission();
}



void RTCDevice::getA1Time(byte& A1Day, byte& A1Hour, byte& A1Minute, byte& A1Second, byte& AlarmBits, bool& A1Dy, bool& A1h12, bool& A1PM) {
	byte temp_buffer;
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x07);
	Wire.endTransmission();

	Wire.requestFrom(CLOCK_ADDRESS, 4);

	temp_buffer	= Wire.read();	// Get A1M1 and A1 Seconds
	A1Second	= bcdToDec(temp_buffer & 0b01111111);
	// put A1M1 bit in position 0 of DS3231_AlarmBits.
	AlarmBits	= AlarmBits | (temp_buffer & 0b10000000)>>7;

	temp_buffer		= Wire.read();	// Get A1M2 and A1 minutes
	A1Minute	= bcdToDec(temp_buffer & 0b01111111);
	// put A1M2 bit in position 1 of DS3231_AlarmBits.
	AlarmBits	= AlarmBits | (temp_buffer & 0b10000000)>>6;

	temp_buffer	= Wire.read();	// Get A1M3 and A1 Hour
	// put A1M3 bit in position 2 of DS3231_AlarmBits.
	AlarmBits	= AlarmBits | (temp_buffer & 0b10000000)>>5;
	// determine A1 12/24 mode
	A1h12		= temp_buffer & 0b01000000;
	if (A1h12) {
		A1PM	= temp_buffer & 0b00100000;			// determine am/pm
		A1Hour	= bcdToDec(temp_buffer & 0b00011111);	// 12-hour
	} else {
		A1Hour	= bcdToDec(temp_buffer & 0b00111111);	// 24-hour
	}

	temp_buffer	= Wire.read();	// Get A1M4 and A1 Day/Date
	// put A1M3 bit in position 3 of DS3231_AlarmBits.
	AlarmBits	= AlarmBits | (temp_buffer & 0b10000000)>>4;
	// determine A1 day or date flag
	A1Dy		= (temp_buffer & 0b01000000)>>6;
	if (A1Dy) {
		// alarm is by day of week, not date.
		A1Day	= bcdToDec(temp_buffer & 0b00001111);
	} else {
		// alarm is by date, not day of week.
		A1Day	= bcdToDec(temp_buffer & 0b00111111);
	}
}

void RTCDevice::getA2Time(byte& A2Day, byte& A2Hour, byte& A2Minute, byte& AlarmBits, bool& A2Dy, bool& A2h12, bool& A2PM) {
	byte temp_buffer;
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x0b);
	Wire.endTransmission();

	Wire.requestFrom(CLOCK_ADDRESS, 3); 
	temp_buffer	= Wire.read();	// Get A2M2 and A2 Minutes
	A2Minute	= bcdToDec(temp_buffer & 0b01111111);
	// put A2M2 bit in position 4 of DS3231_AlarmBits.
	AlarmBits	= AlarmBits | (temp_buffer & 0b10000000)>>3;

	temp_buffer	= Wire.read();	// Get A2M3 and A2 Hour
	// put A2M3 bit in position 5 of DS3231_AlarmBits.
	AlarmBits	= AlarmBits | (temp_buffer & 0b10000000)>>2;
	// determine A2 12/24 mode
	A2h12		= temp_buffer & 0b01000000;
	if (A2h12) {
		A2PM	= temp_buffer & 0b00100000;			// determine am/pm
		A2Hour	= bcdToDec(temp_buffer & 0b00011111);	// 12-hour
	} else {
		A2Hour	= bcdToDec(temp_buffer & 0b00111111);	// 24-hour
	}

	temp_buffer	= Wire.read();	// Get A2M4 and A1 Day/Date
	// put A2M4 bit in position 6 of DS3231_AlarmBits.
	AlarmBits	= AlarmBits | (temp_buffer & 0b10000000)>>1;
	// determine A2 day or date flag
	A2Dy		= (temp_buffer & 0b01000000)>>6;
	if (A2Dy) {
		// alarm is by day of week, not date.
		A2Day	= bcdToDec(temp_buffer & 0b00001111);
	} else {
		// alarm is by date, not day of week.
		A2Day	= bcdToDec(temp_buffer & 0b00111111);
	}
}

void RTCDevice::setA1Time(byte A1Day, byte A1Hour, byte A1Minute, byte A1Second, byte AlarmBits, bool A1Dy, bool A1h12, bool A1PM) {
	//	Sets the alarm-1 date and time on the DS3231, using A1* information
	byte temp_buffer;
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x07);	// A1 starts at 07h
	// Send A1 second and A1M1
	Wire.write(decToBcd(A1Second) | ((AlarmBits & 0b00000001) << 7));
	// Send A1 Minute and A1M2
	Wire.write(decToBcd(A1Minute) | ((AlarmBits & 0b00000010) << 6));
	// Figure out A1 hour 
	if (A1h12) {
		// Start by converting existing time to h12 if it was given in 24h.
		if (A1Hour > 12) {
			// well, then, this obviously isn't a h12 time, is it?
			A1Hour = A1Hour - 12;
			A1PM = true;
		}
		if (A1PM) {
			// Afternoon
			// Convert the hour to BCD and add appropriate flags.
			temp_buffer = decToBcd(A1Hour) | 0b01100000;
		} else {
			// Morning
			// Convert the hour to BCD and add appropriate flags.
			temp_buffer = decToBcd(A1Hour) | 0b01000000;
		}
	} else {
		// Now for 24h
		temp_buffer = decToBcd(A1Hour); 
	}
	temp_buffer = temp_buffer | ((AlarmBits & 0b00000100)<<5);
	// A1 hour is figured out, send it
	Wire.write(temp_buffer); 
	// Figure out A1 day/date and A1M4
	temp_buffer = ((AlarmBits & 0b00001000)<<4) | decToBcd(A1Day);
	if (A1Dy) {
		// Set A1 Day/Date flag (Otherwise it's zero)
		temp_buffer = temp_buffer | 0b01000000;
	}
	Wire.write(temp_buffer);
	// All done!
	Wire.endTransmission();
}

void RTCDevice::setA2Time(byte A2Day, byte A2Hour, byte A2Minute, byte AlarmBits, bool A2Dy, bool A2h12, bool A2PM) {
	//	Sets the alarm-2 date and time on the DS3231, using A2* information
	byte temp_buffer;
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x0b);	// A1 starts at 0bh
	// Send A2 Minute and A2M2
	Wire.write(decToBcd(A2Minute) | ((AlarmBits & 0b00010000) << 3));
	// Figure out A2 hour 
	if (A2h12) {
		// Start by converting existing time to h12 if it was given in 24h.
		if (A2Hour > 12) {
			// well, then, this obviously isn't a h12 time, is it?
			A2Hour = A2Hour - 12;
			A2PM = true;
		}
		if (A2PM) {
			// Afternoon
			// Convert the hour to BCD and add appropriate flags.
			temp_buffer = decToBcd(A2Hour) | 0b01100000;
		} else {
			// Morning
			// Convert the hour to BCD and add appropriate flags.
			temp_buffer = decToBcd(A2Hour) | 0b01000000;
		}
	} else {
		// Now for 24h
		temp_buffer = decToBcd(A2Hour); 
	}
	// add in A2M3 bit
	temp_buffer = temp_buffer | ((AlarmBits & 0b00100000)<<2);
	// A2 hour is figured out, send it
	Wire.write(temp_buffer); 
	// Figure out A2 day/date and A2M4
	temp_buffer = ((AlarmBits & 0b01000000)<<1) | decToBcd(A2Day);
	if (A2Dy) {
		// Set A2 Day/Date flag (Otherwise it's zero)
		temp_buffer = temp_buffer | 0b01000000;
	}
	Wire.write(temp_buffer);
	// All done!
	Wire.endTransmission();
}

void RTCDevice::turnOnAlarm(byte Alarm) {
	// turns on alarm number "Alarm". Defaults to 2 if Alarm is not 1.
	byte temp_buffer = readControlByte(0);
	// modify control byte
	if (Alarm == 1) {
		temp_buffer = temp_buffer | 0b00000101;
	} else {
		temp_buffer = temp_buffer | 0b00000110;
	}
	writeControlByte(temp_buffer, 0);
}

void RTCDevice::turnOffAlarm(byte Alarm) {
	// turns off alarm number "Alarm". Defaults to 2 if Alarm is not 1.
	// Leaves interrupt pin alone.
	byte temp_buffer = readControlByte(0);
	// modify control byte
	if (Alarm == 1) {
		temp_buffer = temp_buffer & 0b11111110;
	} else {
		temp_buffer = temp_buffer & 0b11111101;
	}
	writeControlByte(temp_buffer, 0);
}

bool RTCDevice::checkAlarmEnabled(byte Alarm) {
	// Checks whether the given alarm is enabled.
	byte result = 0x0;
	byte temp_buffer = readControlByte(0);
	if (Alarm == 1) {
		result = temp_buffer & 0b00000001;
	} else {
		result = temp_buffer & 0b00000010;
	}
	return result;
}

bool RTCDevice::checkIfAlarm(byte Alarm) {
	// Checks whether alarm 1 or alarm 2 flag is on, returns T/F accordingly.
	// Turns flag off, also.
	// defaults to checking alarm 2, unless Alarm == 1.
	byte result;
	byte temp_buffer = readControlByte(1);
	if (Alarm == 1) {
		// Did alarm 1 go off?
		result = temp_buffer & 0b00000001;
		// clear flag
		temp_buffer = temp_buffer & 0b11111110;
	} else {
		// Did alarm 2 go off?
		result = temp_buffer & 0b00000010;
		// clear flag
		temp_buffer = temp_buffer & 0b11111101;
	}
	writeControlByte(temp_buffer, 1);
	return result;
}

class HapticDevice : public Adafruit_DRV2605 {
  
  static const uint8_t DEFAULT_EFFECT = 1;
  static const long    POLL_RATE_MS = 20;
  static const uint8_t STBY_MODE_BIT = 6; // numbering from zero see 7.6.2 of device manual 
  uint8_t effect_;
  uint8_t goState_;
  long lastPoll_;
  bool stby_;
  
  void setStandByMode()   {
    if (stby_ == false ) {
      uint8_t mode = readRegister8(DRV2605_REG_MODE);
      mode |= 1<<STBY_MODE_BIT;
      setMode(mode);
      stby_ = true;
      Serial.println("Haptic in Standby");
    }
  }
  
  uint8_t clearStandByMode() {
    if (stby_ == true) {
      uint8_t mode = readRegister8(DRV2605_REG_MODE);
      mode &= ~(1<<STBY_MODE_BIT);
      setMode(mode);
      stby_ = false;
      Serial.println("Haptic Woken up");
    }
  }
  
public:
  int setup() {
    if (!begin() ) {
      Serial.println("Cound not find haptric driver");
      return 0;
    }

    // begin ensure that we are out of standby mode
    stby_ = false;
    
    selectLibrary(1);  
    // I2C trigger by sending 'go' command 
    // default, internal trigger when sending GO command
    setMode(DRV2605_MODE_INTTRIG);
    setEffect(DEFAULT_EFFECT);
    
    return 1;
  }

  void effectPrint(uint8_t effect) {
    switch(effect) {
    case   1:
      Serial.println(F("1 − Strong Click - 100%"));
      break;
    case   2:
      Serial.println(F("2 − Strong Click - 60%"));
      break;
    case   3:
      Serial.println(F("3 − Strong Click - 30%"));
      break;
    case   4:
      Serial.println(F("4 − Sharp Click - 100%"));
      break;
    case   5:
      Serial.println(F("5 − Sharp Click - 60%"));
      break;
    case   6:
      Serial.println(F("6 − Sharp Click - 30%"));
      break;
    case   7:
      Serial.println(F("7 − Soft Bump - 100%"));
      break;
    case   8:
      Serial.println(F("8 − Soft Bump - 60%"));
      break;
    case   9:
      Serial.println(F("9 − Soft Bump - 30%"));
      break;
    case  10:
      Serial.println(F("10 − Double Click - 100%"));
      break;
    case  11:
      Serial.println(F("11 − Double Click - 60%"));
      break;
    case  12:
      Serial.println(F("12 − Triple Click - 100%"));
      break;
    case  13:
      Serial.println(F("13 − Soft Fuzz - 60%"));
      break; 
    case  14:
      Serial.println(F("14 − Strong Buzz - 100%"));
      break;
    case  15:
      Serial.println(F("15 − 750 ms Alert 100%"));
      break;
    case  16:
      Serial.println(F("16 − 1000 ms Alert 100%"));
      break;
    case  17:
      Serial.println(F("17 − Strong Click 1 - 100%"));
      break;
    case  18:
      Serial.println(F("18 − Strong Click 2 - 80%"));
      break;
    case  19:
      Serial.println(F("19 − Strong Click 3 - 60%"));
      break;
    case  20:
      Serial.println(F("20 − Strong Click 4 - 30%"));
      break; 
    case  21:
      Serial.println(F("21 − Medium Click 1 - 100%"));
      break;
    case  22:
      Serial.println(F("22 − Medium Click 2 - 80%"));
      break;
    case  23:
      Serial.println(F("23 − Medium Click 3 - 60%"));
      break;
    case  24:
      Serial.println(F("24 − Sharp Tick 1 - 100%"));
      break;
    case  25:
      Serial.println(F("25 − Sharp Tick 2 - 80%"));
      break;
    case  26:
      Serial.println(F("26 − Sharp Tick 3 – 60%"));
      break;
    case  27:
      Serial.println(F("27 − Short Double Click Strong 1 – 100%"));
      break;
    case  28:
      Serial.println(F("28 − Short Double Click Strong 2 – 80%"));
      break;
    case  29:
      Serial.println(F("29 − Short Double Click Strong 3 – 60%"));
      break;
    case  30:
      Serial.println(F("30 − Short Double Click Strong 4 – 30%"));
      break;
    case  31:
      Serial.println(F("31 − Short Double Click Medium 1 – 100%"));
      break;
    case  32:
      Serial.println(F("32 − Short Double Click Medium 2 – 80%"));
      break;
    case  33:
      Serial.println(F("33 − Short Double Click Medium 3 – 60%"));
      break;
    case  34:
      Serial.println(F("34 − Short Double Sharp Tick 1 – 100%"));
      break;
    case  35:
      Serial.println(F("35 − Short Double Sharp Tick 2 – 80%"));
      break;
    case  36:
      Serial.println(F("36 − Short Double Sharp Tick 3 – 60%"));
      break;
    case  37:
      Serial.println(F("37 − Long Double Sharp Click Strong 1 – 100%"));
      break;
    case  38:
      Serial.println(F("38 − Long Double Sharp Click Strong 2 – 80%"));
      break;
    case  39:
      Serial.println(F("39 − Long Double Sharp Click Strong 3 – 60%"));
      break;
    case  40:
      Serial.println(F("40 − Long Double Sharp Click Strong 4 – 30%"));
      break;
    case  41:
      Serial.println(F("41 − Long Double Sharp Click Medium 1 – 100%"));
      break;
    case  42:
      Serial.println(F("42 − Long Double Sharp Click Medium 2 – 80%"));
      break;
    case  43:
      Serial.println(F("43 − Long Double Sharp Click Medium 3 – 60%"));
      break;
    case  44:
      Serial.println(F("44 − Long Double Sharp Tick 1 – 100%"));
      break;
    case  45:
      Serial.println(F("45 − Long Double Sharp Tick 2 – 80%"));
      break;
    case  46:
      Serial.println(F("46 − Long Double Sharp Tick 3 – 60%"));
      break;
    case  47:
      Serial.println(F("47 − Buzz 1 – 100%"));
      break;
    case  48:
      Serial.println(F("48 − Buzz 2 – 80%"));
      break;
    case  49:
      Serial.println(F("49 − Buzz 3 – 60%"));
      break;
    case  50:
      Serial.println(F("50 − Buzz 4 – 40%"));
      break;
    case  51:
      Serial.println(F("51 − Buzz 5 – 20%"));
      break;
    case  52:
      Serial.println(F("52 − Pulsing Strong 1 – 100%"));
      break;
    case  53:
      Serial.println(F("53 − Pulsing Strong 2 – 60%"));
      break;
    case  54:
      Serial.println(F("54 − Pulsing Medium 1 – 100%"));
      break;
    case  55:
      Serial.println(F("55 − Pulsing Medium 2 – 60%"));
      break;
    case  56:
      Serial.println(F("56 − Pulsing Sharp 1 – 100%"));
      break;
    case  57:
      Serial.println(F("57 − Pulsing Sharp 2 – 60%"));
      break;
    case  58:
      Serial.println(F("58 − Transition Click 1 – 100%"));
      break;
    case  59:
      Serial.println(F("59 − Transition Click 2 – 80%"));
      break;
    case  60:
      Serial.println(F("60 − Transition Click 3 – 60%"));
      break;
    case  61:
      Serial.println(F("61 − Transition Click 4 – 40%"));
      break;
    case  62:
      Serial.println(F("62 − Transition Click 5 – 20%"));
      break;
    case  63:
      Serial.println(F("63 − Transition Click 6 – 10%"));
      break;
    case  64:
      Serial.println(F("64 − Transition Hum 1 – 100%"));
      break;
    case  65:
      Serial.println(F("65 − Transition Hum 2 – 80%"));
      break;
    case  66:
      Serial.println(F("66 − Transition Hum 3 – 60%"));
      break;
    case  67:
      Serial.println(F("67 − Transition Hum 4 – 40%"));
      break;
    case  68:
      Serial.println(F("68 − Transition Hum 5 – 20%"));
      break;
    case  69:
      Serial.println(F("69 − Transition Hum 6 – 10%"));
      break;
    case  70:
      Serial.println(F("70 − Transition Ramp Down Long Smooth 1 – 100 to 0%"));
      break;
    case  71:
      Serial.println(F("71 − Transition Ramp Down Long Smooth 2 – 100 to 0%"));
      break;
    case  72:
      Serial.println(F("72 − Transition Ramp Down Medium Smooth 1 – 100 to 0%"));
      break;
    case  73:
      Serial.println(F("73 − Transition Ramp Down Medium Smooth 2 – 100 to 0%"));
      break;
    case  74:
      Serial.println(F("74 − Transition Ramp Down Short Smooth 1 – 100 to 0%"));
      break;
    case  75:
      Serial.println(F("75 − Transition Ramp Down Short Smooth 2 – 100 to 0%"));
      break;
    case  76:
      Serial.println(F("76 − Transition Ramp Down Long Sharp 1 – 100 to 0%"));
      break;
    case  77:
      Serial.println(F("77 − Transition Ramp Down Long Sharp 2 – 100 to 0%"));
      break;
    case  78:
      Serial.println(F("78 − Transition Ramp Down Medium Sharp 1 – 100 to 0%"));
      break;
    case  79:
      Serial.println(F("79 − Transition Ramp Down Medium Sharp 2 – 100 to 0%"));
      break;
    case  80:
      Serial.println(F("80 − Transition Ramp Down Short Sharp 1 – 100 to 0%"));
      break;
    case  81:
      Serial.println(F("81 − Transition Ramp Down Short Sharp 2 – 100 to 0%"));
      break;
    case  82:
      Serial.println(F("82 − Transition Ramp Up Long Smooth 1 – 0 to 100%"));
      break;
    case  83:
      Serial.println(F("83 − Transition Ramp Up Long Smooth 2 – 0 to 100%"));
      break;
    case  84:
      Serial.println(F("84 − Transition Ramp Up Medium Smooth 1 – 0 to 100%"));
      break;
    case  85:
      Serial.println(F("85 − Transition Ramp Up Medium Smooth 2 – 0 to 100%"));
      break;
    case  86:
      Serial.println(F("86 − Transition Ramp Up Short Smooth 1 – 0 to 100%"));
      break;
    case  87:
      Serial.println(F("87 − Transition Ramp Up Short Smooth 2 – 0 to 100%"));
      break;
    case  88:
      Serial.println(F("88 − Transition Ramp Up Long Sharp 1 – 0 to 100%"));
      break;
    case  89:
      Serial.println(F("89 − Transition Ramp Up Long Sharp 2 – 0 to 100%"));
      break;
    case  90:
      Serial.println(F("90 − Transition Ramp Up Medium Sharp 1 – 0 to 100%"));
      break;
    case  91:
      Serial.println(F("91 − Transition Ramp Up Medium Sharp 2 – 0 to 100%"));
      break;
    case  92:
      Serial.println(F("92 − Transition Ramp Up Short Sharp 1 – 0 to 100%"));
      break;
    case  93:
      Serial.println(F("93 − Transition Ramp Up Short Sharp 2 – 0 to 100%"));
      break;
    case  94:
      Serial.println(F("94 − Transition Ramp Down Long Smooth 1 – 50 to 0%"));
      break;
    case  95:
      Serial.println(F("95 − Transition Ramp Down Long Smooth 2 – 50 to 0%"));
      break;
    case  96:
      Serial.println(F("96 − Transition Ramp Down Medium Smooth 1 – 50 to 0%"));
      break;
    case  97:
      Serial.println(F("97 − Transition Ramp Down Medium Smooth 2 – 50 to 0%"));
      break;
    case  98:
      Serial.println(F("98 − Transition Ramp Down Short Smooth 1 – 50 to 0%"));
      break;
    case  99:
      Serial.println(F("99 − Transition Ramp Down Short Smooth 2 – 50 to 0%"));
      break;
    case  100:
      Serial.println(F("100 − Transition Ramp Down Long Sharp 1 – 50 to 0%"));
      break;
    case  101:
      Serial.println(F("101 − Transition Ramp Down Long Sharp 2 – 50 to 0%"));
      break;
    case  102:
      Serial.println(F("102 − Transition Ramp Down Medium Sharp 1 – 50 to 0%"));
      break;
    case  103:
      Serial.println(F("103 − Transition Ramp Down Medium Sharp 2 – 50 to 0%"));
      break;
    case  104:
      Serial.println(F("104 − Transition Ramp Down Short Sharp 1 – 50 to 0%"));
      break;
    case  105:
      Serial.println(F("105 − Transition Ramp Down Short Sharp 2 – 50 to 0%"));
      break;
    case  106:
      Serial.println(F("106 − Transition Ramp Up Long Smooth 1 – 0 to 50%"));
      break;
    case  107:
      Serial.println(F("107 − Transition Ramp Up Long Smooth 2 – 0 to 50%"));
      break;
    case  108:
      Serial.println(F("108 − Transition Ramp Up Medium Smooth 1 – 0 to 50%"));
      break;
    case  109:
      Serial.println(F("109 − Transition Ramp Up Medium Smooth 2 – 0 to 50%"));
      break;
    case  110:
      Serial.println(F("110 − Transition Ramp Up Short Smooth 1 – 0 to 50%"));
      break;
    case  111:
      Serial.println(F("111 − Transition Ramp Up Short Smooth 2 – 0 to 50%"));
      break;
    case  112:
      Serial.println(F("112 − Transition Ramp Up Long Sharp 1 – 0 to 50%"));
      break;
    case  113:
      Serial.println(F("113 − Transition Ramp Up Long Sharp 2 – 0 to 50%"));
      break;
    case  114:
      Serial.println(F("114 − Transition Ramp Up Medium Sharp 1 – 0 to 50%"));
      break;
    case  115:
      Serial.println(F("115 − Transition Ramp Up Medium Sharp 2 – 0 to 50%"));
      break;
    case  116:
      Serial.println(F("116 − Transition Ramp Up Short Sharp 1 – 0 to 50%"));
      break;
    case  117:
      Serial.println(F("117 − Transition Ramp Up Short Sharp 2 – 0 to 50%"));
      break;
    case  118:
      Serial.println(F("118 − Long buzz for programmatic stopping – 100%"));
      break;
    case  119:
      Serial.println(F("119 − Smooth Hum 1 (No kick or brake pulse) – 50%"));
      break;
    case  120:
      Serial.println(F("120 − Smooth Hum 2 (No kick or brake pulse) – 40%"));
      break;
    case  121:
      Serial.println(F("121 − Smooth Hum 3 (No kick or brake pulse) – 30%"));
      break;
    case  122:
      Serial.println(F("122 − Smooth Hum 4 (No kick or brake pulse) – 20%"));
      break;
    case  123:
      Serial.println(F("123 − Smooth Hum 5 (No kick or brake pulse) – 10%"));
      break;
    default:
      Serial.print(effect);
      Serial.println(F(" - Unknown effect")); 
    }
 
  }

  void setEffect(uint8_t effect) {
    // set the effect to play
    if (effect != effect_) {
      setWaveform(0, effect);  // play effect 
      setWaveform(1, 0);       // end waveform
      effect_ = effect;
    }
  }
  
  void playEffect(uint8_t effect, long delayms=0, boolean verbose=false) {
    if (verbose) effectPrint(effect_);

    clearStandByMode();
    
    setEffect(effect);
    
    // play the effect!
    go();

    goState_ = readRegister8(DRV2605_REG_GO);
    
    if (goState_) {
      lastPoll_ = millis();
      Activity.setHaptic();
    }
    
    // wait a bit
    if(delayms) delay(delayms);
  }
  
  void playRange(uint8_t min=1, uint8_t max=123, long delayms=1500, bool verbose=true) {
    uint8_t effect;
    for (effect = min; effect <= max; effect ++) {
      playEffect(effect, delayms, verbose);
    }
  }

  int loopActivity() {
    int rc;
    if (goState_) { 
      if (millis() - lastPoll_ > POLL_RATE_MS) {
	goState_ = readRegister8(DRV2605_REG_GO);
	lastPoll_ = millis();
      }
    }
    return goState_;
  }

  void sleep() {
    setStandByMode();
  }
};

class BLEDevice {
  BLEDfu  bledfu;  // OTA DFU service
  BLEDis  bledis;  // device information
  BLEUart bleuart; // uart over ble
  BLEBas  blebas;  // battery
public:
  void setup();
  int loopActivity();
};

/**
 * RTOS Idle callback is automatically invoked by FreeRTOS
 * when there are no active threads. E.g when loop() calls delay() and
 * there is no bluetooth or hw event. This is the ideal place to handle
 * background data.
 *
 * NOTE: FreeRTOS is configured as tickless idle mode. After this callback
 * is executed, if there is time, freeRTOS kernel will go into low power mode.
 * Therefore waitForEvent() should not be called in this callback.
 * http://www.freertos.org/low-power-tickless-rtos.html
 *
 * WARNING: This function MUST NOT call any blocking FreeRTOS API
 * such as delay(), xSemaphoreTake() etc ... for more information
 * http://www.freertos.org/a00016.html
 */
void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
}

// Global Instances
ButtonDevice Button;
HapticDevice Haptic;
RTCDevice    RTC;
BLEDevice    BLE;

#define ALRM1_MATCH_EVERY_SEC  0b1111  // once a second
#define ALRM1_MATCH_SEC        0b1110  // when seconds match
#define ALRM1_MATCH_MIN_SEC    0b1100  // when minutes and seconds match
#define ALRM1_MATCH_HR_MIN_SEC 0b1000  // when hours, minutes, and seconds match

#define ALRM2_ONCE_PER_MIN     0b111   // once per minute (00 seconds of every minute)
#define ALRM2_MATCH_MIN        0b110   // when minutes match
#define ALRM2_MATCH_HR_MIN     0b100   // when hours and minutes match

void SetAlarms(uint8_t second, uint8_t minute) {
   // This is the interesting part which sets the AlarmBits and configures, when the Alarm be triggered
  byte ALRM1_SET = ALRM1_MATCH_SEC; // trigger A1 when minute and second match
  byte ALRM2_SET = 0;

  // combine the AlarmBits
  int ALARM_BITS = ALRM2_SET;
  ALARM_BITS <<= 4;
  ALARM_BITS |= ALRM1_SET;

  second %= 60;
  
  // Trigger Alarm when Minute == 30 or 0
  // Clock.setA1Time(Day, Hour, Minute, Second, AlarmBits, DayOfWeek, 12 hour mode, PM)
  RTC.setA1Time(0, 0, 0, second, ALARM_BITS, false, false, false);
  //RTC.setA2Time(0, 0, minute, ALARM_BITS, false, false, false); 

  RTC.checkIfAlarm(1);
  // Turn on Alarm
  RTC.turnOnAlarm(1);
  //RTC.turnOnAlarm(2);

  RTC.printAlarmInfo();
}

//  Application logic is fundamentally in these following routines
void
ButtonDevice::shortPressAction(void)
{
  digitalWrite(blueLedPin, HIGH);
  Serial.println(F("Short Button Press."));
  DateTime now = RTC.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  SetAlarms(now.second()+5, now.minute() + 1);
  
  digitalWrite(blueLedPin, LOW);
}

void
ButtonDevice::longPressAction(void)
{
  Serial.println("Long Button Press Action.");
}


int
ButtonDevice::loopActivity()
{
  boolean changed = debouncer_.update();
  
  if ( changed ) {
    // Get the update value
    int value = debouncer_.read();
    if (value == HIGH) {
      state_ = 0;
      Serial.println("Button released (state 0)");
      if (!longPress_) shortPressAction(); else longPressAction();
      return 0;
    } else {
      state_ = 1;
      Serial.println("Button pressed (state 1)");
      pressTimeStamp_ = millis();
      longPress_ = false;
    }
  }
  
  if  ( state_ == 1 ) {
    if ( millis() - pressTimeStamp_ >= 500 && longPress_ == false) {
      Serial.println("Long Press Detected");
      Haptic.playEffect(8); // soft bump 100%
      longPress_ = true;
    }
  }
  return 1;
}

int 
RTCDevice::loopActivity()
{
  Serial.println("RTCAlarm Activity");
  //

  if (checkIfAlarm(1)) {
    RTC.turnOffAlarm(1);  // stop from reocurring
    Serial.println("Alarm 1 triggered");
  }

  if (checkIfAlarm(2)) {
    RTC.turnOffAlarm(2); // stop from reocurring
    Serial.println("Alarm 2 triggered");
  }

  DateTime now = RTC.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  
  RTC.printAlarmInfo();
  //  RTC.enableInterrupt();
  return 0;
}

void setup() {
  pinMode(redLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);

  digitalWrite(redLedPin, LOW);
  digitalWrite(blueLedPin, HIGH);

  
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  Serial.println(F("PSST"));

  // Lets get the clock going quickly incase we need to set the time  
  if (! RTC.setup()) {
    digitalWrite(redLedPin, HIGH);
    Serial.println(F("RTC::setup Failed\n"));
    while(1);
  }

  if (RTC.lostPower()) {
    Serial.println(F("RTC lost power, lets set the time!"));
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  
  Serial.println(F("RTC ready"));

  if (! Haptic.setup()) {
    digitalWrite(redLedPin, HIGH);
    Serial.println(F("HapticDriver::setup Failed\n"));
    while (1);
  }
  Serial.println(F("HapticDriver ready"));

  if (! Button.setup()) {
    digitalWrite(redLedPin, HIGH);
    Serial.println(F("Button::setup Failed\n"));
  }
  Serial.println(F("Button ready"));
  

  
  RTC.setA1Time(0, 0, 0, 0,
		0,
		false, false, false);  
  RTC.setA2Time(0, 0, 0,
		0,
		false, false, false);
  RTC.turnOffAlarm(1);
  RTC.turnOffAlarm(2);
  
  RTC.printAlarmInfo();

  Activity.clearAll();
  Events.clearAll();

  digitalWrite(LED_BLUE, LOW);
  
  Haptic.playEffect(86);
  Button.enableInterrupt();
}

void loop() {
  
  if (Activity.isIdle()) {
    // power things down and wait for interrupt triggered events
    Haptic.sleep();
    digitalWrite(redLedPin, LOW);
    sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    waitForEvent();
    //delay(DELAY_FOREVER);
  }
  
  // Events.print();
  // process Events
  if (Events.button()) {
    Serial.println("Button Event");
    digitalWrite(redLedPin, HIGH);
    Activity.setButton();
    Events.ackButton();
  }
  if (Events.RTCAlarm()) {
    Serial.println("RTCAlarm Event");
    digitalWrite(redLedPin, HIGH);
    Activity.setRTC();
    Events.ackRTCAlarm();
  }  
  
  if (Activity.isButtonActive()) {
    if (!Button.loopActivity()) {
      Serial.println("Button Activity Done");
      Activity.clearButton();
      Button.enableInterrupt();
    }
  }
  if (Activity.isHapticActive()) {
    if (!Haptic.loopActivity()) {
      Serial.println("Haptic Activity Done");
      Activity.clearHaptic();
    }
  }
  if (Activity.isRTCActive()) {
    if (!RTC.loopActivity()) {
      Serial.println("RTCAlarm Activity Done");
      Activity.clearRTC();
    }
  }
  //    !BlueTooth.loop() && Activity.clearBluetooth();
}
