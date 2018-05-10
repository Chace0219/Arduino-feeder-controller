/*

*/
#include <Wire.h>
#include "RTClib.h"

#include "FiniteStateMachine.h"
#include "FBD.h"

RTC_DS1307 rtc;
bool firstFlag = true;

void targetOver();

#define SWITCHDEBOUNCE 200;

TON switchEastTON(200), switchWestTON(200), switchSouthTON(200), switchNorthTON(200), pos2TON(200), pos3TON(200);
TON buttonEastTON(200), buttonWestTON(200), buttonSouthTON(200), buttonNorthTON(200);
Ftrg remotebutTrg;
Rtrg proxSensorTrg;

bool millActive = false;
// relay level
#define RELAYON   LOW
#define RELAYOFF  HIGH

const uint8_t MILLLEVEL = A0;
const uint8_t MILLMOTOR = 12;

const uint8_t EASTSWITCH = 41;
const uint8_t WESTSWITCH = 39;
const uint8_t SOUTHSWITCH = 40;
const uint8_t NORTHSWITCH = 38;

const uint8_t POSITION2 = 42;
const uint8_t POSITION3 = 43;

// const uint8_t PROXSENSOR = 5;
const uint8_t PROXSENSOR = 2;

// buttons
const uint8_t EASTBUTTON = 23;
const uint8_t WESTBUTTON = 25;
const uint8_t NORTHBUTTON = 27;
const uint8_t SOUTHBUTTON = 29;

// motor controller
const uint8_t CARRIERPWR = 22;
const uint8_t CARRIERFWD = 24;
const uint8_t CARRIERREV = 26;

const uint8_t FEEDERPWR = 28;
const uint8_t FEEDERFWD = 30;
const uint8_t FEEDERREV = 32;

const uint8_t SUPPLYPWR = 4;
const uint8_t SUPPLYACT = 5;


// time schedules settings
typedef struct
{
	uint8_t nHour;
	uint8_t nMin;
	uint8_t nSec;
}SettingTime;

/*
#define SCHEDULECOUNT 10
SettingTime schedules[SCHEDULECOUNT] = {
{ 15, 03, 0 },{ 16, 34, 0 },{ 7, 0, 0 },{ 9, 0, 0 },{ 11, 0, 0 },{ 15, 0, 0 },{ 17, 0, 0 },{ 19, 0, 0 },{ 20, 0, 0 },{ 21, 0, 0 }
};
*/

#define SCHEDULECOUNT 1
SettingTime schedules[SCHEDULECOUNT] = {
	{ 10, 55, 30 }
};

void initPorts()
{
	// Wireless button input using interrupt 
	pinMode(EASTSWITCH, INPUT_PULLUP);
	pinMode(WESTSWITCH, INPUT_PULLUP);
	pinMode(SOUTHSWITCH, INPUT_PULLUP);
	pinMode(NORTHSWITCH, INPUT_PULLUP);
	pinMode(POSITION2, INPUT_PULLUP);
	pinMode(POSITION3, INPUT_PULLUP);

	// prox sensor interrupt
	pinMode(PROXSENSOR, INPUT_PULLUP);
	attachInterrupt(0, targetOver, FALLING);

	// 
	pinMode(EASTBUTTON, INPUT_PULLUP);
	pinMode(WESTBUTTON, INPUT_PULLUP);
	pinMode(NORTHBUTTON, INPUT_PULLUP);
	pinMode(SOUTHBUTTON, INPUT_PULLUP);


	pinMode(CARRIERPWR, OUTPUT);
	digitalWrite(CARRIERPWR, RELAYOFF);
	pinMode(CARRIERFWD, OUTPUT);
	digitalWrite(CARRIERFWD, RELAYOFF);
	pinMode(CARRIERREV, OUTPUT);
	digitalWrite(CARRIERREV, RELAYOFF);

	pinMode(FEEDERPWR, OUTPUT);
	digitalWrite(FEEDERPWR, RELAYOFF);
	pinMode(FEEDERFWD, OUTPUT);
	digitalWrite(FEEDERFWD, RELAYOFF);
	pinMode(FEEDERREV, OUTPUT);
	digitalWrite(FEEDERREV, RELAYOFF);

	pinMode(SUPPLYPWR, OUTPUT);
	digitalWrite(SUPPLYPWR, RELAYOFF);
	pinMode(SUPPLYACT, OUTPUT);
	digitalWrite(SUPPLYACT, RELAYOFF);

	pinMode(MILLMOTOR, OUTPUT);
}

State Startup = State(NULL);
State carrierToHome = State(NULL);
State feederToHome = State(NULL);
State Standby = State(NULL);

State Pos4Supply = State(NULL);
State Pos4ToPos3 = State(NULL);
State Pos3Supply = State(NULL);
State Pos3ToPos2 = State(NULL);
State Pos2Supply = State(NULL);
State Pos2ToPos1 = State(NULL);
State Pos1Supply = State(NULL);
State Pos1ToPos4 = State(NULL);

State Manual = State(NULL);

// FSM instance
FSM controller = FSM(Startup);
void setup()
{
	//
	Serial.begin(115200);
	Serial.println("Program started!");

	// rtc initialize
	if (!rtc.begin())
	{
		Serial.println("Couldn't find RTC");
		while (1);
	}

	if (!rtc.isrunning())
	{
		Serial.println("RTC is NOT running!");
		// rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
		// following line sets the RTC to the date & time this sketch was compiled
		// rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
		// This line sets the RTC with an explicit date & time, for example to set
		// January 21, 2014 at 3am you would call:
		// rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
	}

	// init part
	initPorts();

	Serial.println("System Startup");
}

static uint32_t nRTCTime = millis();
const uint32_t RTCINTERVAL = 2000;

void carrierToWest()
{
	digitalWrite(CARRIERPWR, RELAYON);
	digitalWrite(CARRIERFWD, RELAYOFF);
	digitalWrite(CARRIERREV, RELAYON);
}

void carrierToEast()
{
	digitalWrite(CARRIERPWR, RELAYON);
	digitalWrite(CARRIERFWD, RELAYON);
	digitalWrite(CARRIERREV, RELAYOFF);
}

void carrierStop()
{
	digitalWrite(CARRIERPWR, RELAYOFF);
	digitalWrite(CARRIERFWD, RELAYOFF);
	digitalWrite(CARRIERREV, RELAYOFF);
}

void feederStop()
{
	digitalWrite(FEEDERPWR, RELAYOFF);
	digitalWrite(FEEDERFWD, RELAYOFF);
	digitalWrite(FEEDERREV, RELAYOFF);
}

void feederToNorth()
{
	digitalWrite(FEEDERPWR, RELAYON);
	digitalWrite(FEEDERFWD, RELAYOFF);
	digitalWrite(FEEDERREV, RELAYON);
}

void feederToSouth()
{
	digitalWrite(FEEDERPWR, RELAYON);
	digitalWrite(FEEDERFWD, RELAYON);
	digitalWrite(FEEDERREV, RELAYOFF);
}

void loop()
{
	{
		switchEastTON.IN = digitalRead(EASTSWITCH) == false;
		switchEastTON.update();

		switchWestTON.IN = digitalRead(WESTSWITCH) == false;
		switchWestTON.update();

		switchSouthTON.IN = digitalRead(SOUTHSWITCH) == false;
		switchSouthTON.update();

		switchNorthTON.IN = digitalRead(NORTHSWITCH) == false;
		switchNorthTON.update();

		buttonEastTON.IN = digitalRead(EASTBUTTON) == false;
		buttonEastTON.update();

		buttonWestTON.IN = digitalRead(WESTBUTTON) == false;
		buttonWestTON.update();

		buttonSouthTON.IN = digitalRead(SOUTHBUTTON) == false;
		buttonSouthTON.update();

		buttonNorthTON.IN = digitalRead(NORTHBUTTON) == false;
		buttonNorthTON.update();

		pos2TON.IN = digitalRead(POSITION2) == false;
		pos2TON.update();

		pos3TON.IN = digitalRead(POSITION3) == false;
		pos3TON.update();
	}

	// enter into manual
	if ((buttonEastTON.Q || buttonWestTON.Q) || (buttonSouthTON.Q || buttonNorthTON.Q))
	{
		if (!controller.isInState(Manual))
		{
			analogWrite(MILLMOTOR, 0);
			controller.transitionTo(Manual);
		}
	}

	remotebutTrg.IN = (buttonEastTON.Q || buttonWestTON.Q) || (buttonNorthTON.Q || buttonSouthTON.Q);
	remotebutTrg.update();

	// FSM instance update
	controller.update();

	proxSensorTrg.update();
	proxSensorTrg.IN = false;

	uint16_t millLevel = analogRead(MILLLEVEL);

	if (controller.isInState(Standby))
	{
		analogWrite(MILLMOTOR, 0);
		carrierStop();
		feederStop();

		DateTime now = rtc.now();
		uint32_t currtime = now.hour();
		currtime *= 100;
		currtime += now.minute();
		currtime *= 100;
		currtime += now.second();

		for (uint8_t idx = 0; idx < SCHEDULECOUNT; idx++)
		{
			uint32_t scheduletime = schedules[idx].nHour;
			scheduletime *= 100;
			scheduletime += schedules[idx].nMin;
			scheduletime *= 100;
			scheduletime += schedules[idx].nSec;

			if (currtime >= scheduletime && currtime <= (scheduletime + 15))
			{
				Serial.println("ready to supply");
				controller.transitionTo(carrierToHome);
				millActive = false;
				break;
			}
		}
	}
	else if (controller.isInState(Manual))
	{
		Serial.println("entered into manual Mode");

		if (buttonEastTON.Q)
		{
			if (switchEastTON.Q)
			{
				Serial.println(F("Manual mode, arrived to East"));
				carrierStop();
			}
			else
			{
				Serial.println(F("Manual mode, to East"));
				carrierToEast();
			}
		}
		else if (buttonWestTON.Q)
		{
			if (switchWestTON.Q)
			{
				Serial.println(F("Manual mode, arrived to west"));
				carrierStop();
			}
			else
			{
				Serial.println(F("Manual mode, to west"));
				carrierToWest();
			}
		}
		else
			carrierStop();

		if (buttonNorthTON.Q)
		{
			if (switchNorthTON.Q)
			{
				Serial.println(F("Manual mode, arrived to north"));
				feederStop();
			}
			else
			{
				Serial.println(F("Manual mode, to north"));
				feederToNorth();
			}
		}
		else if (buttonSouthTON.Q)
		{
			if (switchSouthTON.Q)
			{
				Serial.println(F("Manual mode, arrived to south"));
				feederStop();
			}
			else
			{
				Serial.println(F("Manual mode, to south"));
				feederToSouth();
			}
		}
		else
			feederStop();


		if (remotebutTrg.Q)
		{
			controller.transitionTo(Standby);
			Serial.println(F("All remote button are released, will be enter standby mode!"));
		}
	}
	else if (controller.isInState(Startup))
	{
		carrierStop();
		feederStop();
		analogWrite(MILLMOTOR, 0);

		if (controller.timeInCurrentState() > 2000)
		{
			Serial.println(F("carrier is moving to west edge!"));
			controller.transitionTo(carrierToHome);
		}
	}
	else if (controller.isInState(carrierToHome))
	{
		carrierToWest();
		feederStop();
		analogWrite(MILLMOTOR, 0);

		if (switchWestTON.Q)
		{
			Serial.println(F("feeder is moving to south edge!"));
			controller.transitionTo(feederToHome);
		}
	}
	else if (controller.isInState(feederToHome))
	{
		carrierStop();
		feederToSouth();

		if (switchSouthTON.Q)
		{
			feederStop();
			Serial.println("feeder is in south position");
			if (firstFlag)
			{
				firstFlag = false;
				Serial.println("enter to standby");
				controller.transitionTo(Standby);
			}
			else
			{
				Serial.println("enter to supply cycle");
				controller.transitionTo(Pos4Supply);
				Serial.println(F("carrier is moving to east in table4"));
			}
		}
	}
	else if (controller.isInState(Pos4Supply))
	{
		carrierToEast();
		feederStop();
		if (proxSensorTrg.Q) // trigger mill motor
		{
			millActive = true;
			Serial.println(F("mill motor will be started"));
			analogWrite(MILLMOTOR, map(millLevel, 0, 1024, 0, 255));
		}

		if (millActive)
		{
			Serial.print("mill level is ");
			Serial.println(millLevel);
			analogWrite(MILLMOTOR, map(millLevel, 0, 1024, 0, 255));
		}

		if (switchEastTON.Q)
		{
			controller.transitionTo(Pos4ToPos3);
			Serial.println(F("feeder is moving to position 3"));
		}
	}
	else if (controller.isInState(Pos4ToPos3))
	{
		carrierStop();
		feederToNorth();
		analogWrite(MILLMOTOR, 0);

		if (pos3TON.Q)
		{
			Serial.println(F("position3 detected!"));
			controller.transitionTo(Pos3Supply);
			Serial.println(F("Carrier is moving to west in table3"));
			Serial.println(F("mill motor will be started"));
		}
	}
	else if (controller.isInState(Pos3Supply))
	{
		feederStop();
		carrierToWest();
		analogWrite(MILLMOTOR, map(millLevel, 0, 1024, 0, 255));

		if (proxSensorTrg.Q)
		{
			Serial.println(F("mill motor stopped"));
			controller.transitionTo(Pos3ToPos2);
			Serial.println(F("feeder is moving to position 2"));
		}
	}
	else if (controller.isInState(Pos3ToPos2))
	{
		feederToNorth();
		carrierStop();
		analogWrite(MILLMOTOR, 0);
		if (pos2TON.Q)
		{
			Serial.println(F("position2 is detected!"));
			controller.transitionTo(Pos2Supply);
			Serial.println(F("Carrier is moving to east in table2"));
		}
	}
	else if (controller.isInState(Pos2Supply))
	{
		feederStop();
		carrierToEast();
		analogWrite(MILLMOTOR, map(millLevel, 0, 1024, 0, 255));
		if (switchEastTON.Q)
		{
			Serial.println(F("mill motor stopped"));
			controller.transitionTo(Pos2ToPos1);
			Serial.println(F("feeder is moving to position 2"));
		}
	}
	else if (controller.isInState(Pos2ToPos1))
	{
		feederToNorth();
		carrierStop();
		analogWrite(MILLMOTOR, 0);
		if (switchNorthTON.Q)
		{
			Serial.println(F("north switch detected"));
			controller.transitionTo(Pos1Supply);
			Serial.println(F("Carrier is moving to west in table1"));
			millActive = true;
		}
	}
	else if (controller.isInState(Pos1Supply))
	{
		feederStop();
		carrierToWest();
		if(millActive)
			analogWrite(MILLMOTOR, map(millLevel, 0, 1024, 0, 255));
		else
			analogWrite(MILLMOTOR, 0);

		if (proxSensorTrg.Q)
		{
			millActive = false;
			Serial.println(F("mill motor stopped"));
		}

		if (switchWestTON.Q)
		{
			controller.transitionTo(Pos1ToPos4);
			Serial.println(F("feeder is moving to south"));
		}

	}
	else if (controller.isInState(Pos1ToPos4))
	{
		feederToSouth();
		carrierStop();
		analogWrite(MILLMOTOR, 0);

		if (switchSouthTON.Q)
		{
			Serial.println(F("south switch detected"));
			controller.transitionTo(Standby);
			Serial.println(F("entered into standby status"));
		}
	}


	if ((millis() - nRTCTime) > RTCINTERVAL)
	{
		nRTCTime = millis();
		DateTime now = rtc.now();
		Serial.print(now.year(), DEC);
		Serial.print('/');
		Serial.print(now.month(), DEC);
		Serial.print('/');
		Serial.print(now.day(), DEC);
		Serial.print(" (");
		Serial.print(") ");
		Serial.print(now.hour(), DEC);
		Serial.print(':');
		Serial.print(now.minute(), DEC);
		Serial.print(':');
		Serial.print(now.second(), DEC);
		Serial.println();
	}
}

void targetOver()
{
	proxSensorTrg.IN = true;
}