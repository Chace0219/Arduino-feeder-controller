/*
	
*/
#include <Wire.h>
#include "RTClib.h"

#include "FiniteStateMachine.h"
#include "FBD.h"

RTC_DS1307 rtc;

bool firstFlag = true;
volatile bool proxTriggered = false;

void targetOver();

TON switchEastTON, switchWestTON, switchSouthTON, switchNorthTON;

TON buttonEastTON, buttonWestTON, buttonNorthTON, buttonSouthTON;
Ftrg remotebutTrg;


const uint16_t SWITCHDEBOUNCE = 200;

extern State Startup;
extern State Standby;
extern State Forward;
extern State Backward;

// relay level
#define RELAYON   LOW
#define RELAYOFF  HIGH

// pin definitions
const uint8_t EASTSWITCH = 5;
const uint8_t WESTSWITCH = 3;
const uint8_t SOUTHSWITCH = 4;
const uint8_t NORTHSWITCH = 13;
const uint8_t PROXSENSOR = 2;

// buttons
const uint8_t EASTBUTTON = 6;
const uint8_t WESTBUTTON = 7;
const uint8_t NORTHBUTTON = 8;
const uint8_t SOUTHBUTTON = 9;

// motor controller
const uint8_t CARRIERPWR = A0;
const uint8_t CARRIERFWD = A1;
const uint8_t CARRIERREV = A2;

const uint8_t FEEDERPWR = A3;
const uint8_t FEEDERFWD = 10;
const uint8_t FEEDERREV = 11;

const uint8_t SUPPLYPWR = 12;
const uint8_t SUPPLYACT = 13;

// time schedules settings
typedef struct
{
	uint8_t nHour;
	uint8_t nMin;
	uint8_t nSec;
}SettingTime;

#define SCHEDULECOUNT 10
SettingTime schedules[SCHEDULECOUNT] = {
	{ 15, 03, 0 },{ 16, 34, 0 },{ 7, 0, 0 },{ 9, 0, 0 },{ 11, 0, 0 },{ 15, 0, 0 },{ 17, 0, 0 },{ 19, 0, 0 },{ 20, 0, 0 },{ 21, 0, 0 }
};

void initPorts()
{
	pinMode(EASTSWITCH, INPUT_PULLUP);
	pinMode(WESTSWITCH, INPUT_PULLUP);
	pinMode(SOUTHSWITCH, INPUT_PULLUP);
	pinMode(NORTHSWITCH, INPUT_PULLUP);

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
}

void initFBDs()
{
	
	switchEastTON.IN = false;
	switchEastTON.Q = false;
	switchEastTON.PRE = false;
	switchEastTON.IN = false;
	switchEastTON.ET = false;
	switchEastTON.PT = SWITCHDEBOUNCE;

	switchWestTON.IN = false;
	switchWestTON.Q = false;
	switchWestTON.PRE = false;
	switchWestTON.IN = false;
	switchWestTON.ET = false;
	switchWestTON.PT = SWITCHDEBOUNCE;

	switchSouthTON.IN = false;
	switchSouthTON.Q = false;
	switchSouthTON.PRE = false;
	switchSouthTON.IN = false;
	switchSouthTON.ET = false;
	switchSouthTON.PT = SWITCHDEBOUNCE;

	switchNorthTON.IN = false;
	switchNorthTON.Q = false;
	switchNorthTON.PRE = false;
	switchNorthTON.IN = false;
	switchNorthTON.ET = false;
	switchNorthTON.PT = SWITCHDEBOUNCE;

	buttonEastTON.IN = false;
	buttonEastTON.Q = false;
	buttonEastTON.PRE = false;
	buttonEastTON.IN = false;
	buttonEastTON.ET = false;
	buttonEastTON.PT = SWITCHDEBOUNCE;

	buttonWestTON.IN = false;
	buttonWestTON.Q = false;
	buttonWestTON.PRE = false;
	buttonWestTON.IN = false;
	buttonWestTON.ET = false;
	buttonWestTON.PT = SWITCHDEBOUNCE;

	buttonNorthTON.IN = false;
	buttonNorthTON.Q = false;
	buttonNorthTON.PRE = false;
	buttonNorthTON.IN = false;
	buttonNorthTON.ET = false;
	buttonNorthTON.PT = SWITCHDEBOUNCE;

	buttonSouthTON.IN = false;
	buttonSouthTON.Q = false;
	buttonSouthTON.PRE = false;
	buttonSouthTON.IN = false;
	buttonSouthTON.ET = false;
	buttonSouthTON.PT = SWITCHDEBOUNCE;

	remotebutTrg.IN = false;
	remotebutTrg.PRE = false;
	remotebutTrg.Q = false;

}

extern FSM controller;

void carrierToHomeEnter();
void carrierToHomeUpdate();
void feederToHomeEnter();
void feederToHomeUpdate();
void feederToHomeExit();

void standbyEnter();
void standbyUpdate();

void supplyPos1Enter();
void supplyPos1Update();

void movePos1_2Enter();
void movePos1_2Update();

void supplyPos2Enter();
void supplyPos2Update();

void movePos2_3Enter();
void movePos2_3Update();

void supplyPos3Enter();
void supplyPos3Update();

void movePos3_4Enter();
void movePos3_4Update();

void supplyPos4Enter();
void supplyPos4Update();

void manualEnter();
void manualUpdate();

void startupUpdate();

State Startup = State(NULL, startupUpdate, NULL);

State CarrierToHome = State(carrierToHomeEnter, carrierToHomeUpdate, feederToHomeExit);
State FeederToHome = State(feederToHomeEnter, feederToHomeUpdate, NULL);
State Standby = State(standbyEnter, standbyUpdate, NULL);

State Pos1Supply = State(supplyPos1Enter, supplyPos1Update, NULL);
State Pos12Pos2 = State(movePos1_2Enter, movePos1_2Update, NULL);

State Pos2Supply = State(supplyPos2Enter, supplyPos2Update, NULL);
State Pos22Pos3 = State(movePos2_3Enter, movePos2_3Update, NULL);

State Pos3Supply = State(supplyPos3Enter, supplyPos3Update, NULL);
State Pos32Pos4 = State(movePos3_4Enter, movePos3_4Update, NULL);

State Pos4Supply = State(supplyPos4Enter, supplyPos4Update, NULL);

State Manual = State(manualEnter, manualUpdate, NULL);

// FSM instance
FSM controller = FSM(Startup);

void startupUpdate()
{
	if (controller.timeInCurrentState() > 2000)
	{
		controller.transitionTo(CarrierToHome);
	}
	digitalWrite(SUPPLYPWR, RELAYOFF);
	digitalWrite(SUPPLYACT, RELAYOFF);

}

void carrierToHomeEnter()
{
	if (switchEastTON.Q)
	{
		Serial.println(F("Carrier is in home position."));
		controller.immediateTransitionTo(FeederToHome);
	}
	else
	{
		digitalWrite(CARRIERPWR, RELAYON);
		digitalWrite(CARRIERFWD, RELAYOFF);
		digitalWrite(CARRIERREV, RELAYON);

		Serial.println(F("Carrier is moving to east edge!"));
	}
}

void carrierToHomeUpdate()
{
	if (switchEastTON.Q)
	{
		// stop 
		digitalWrite(CARRIERPWR, RELAYOFF);
		digitalWrite(CARRIERFWD, RELAYOFF);
		digitalWrite(CARRIERREV, RELAYOFF);
		controller.transitionTo(FeederToHome);
	}
	else
	{
	}
}

void feederToHomeEnter()
{
	if (switchSouthTON.Q)
	{
		Serial.println(F("Feeder is in home position."));
		controller.immediateTransitionTo(Standby);
	}
	else
	{
		digitalWrite(FEEDERPWR, RELAYON);
		digitalWrite(FEEDERFWD, RELAYOFF);
		digitalWrite(FEEDERREV, RELAYON);
		Serial.println(F("Feeder is moving to home edge!"));
	}
}

void feederToHomeUpdate()
{
	if (switchSouthTON.Q)
	{
		// stop 
		digitalWrite(FEEDERPWR, RELAYOFF);
		digitalWrite(FEEDERFWD, RELAYOFF);
		digitalWrite(FEEDERREV, RELAYOFF);

		Serial.println("Feeder is in home");
		if (firstFlag)
		{
			Serial.println("enter to standby");
			firstFlag = false;
			controller.transitionTo(Standby);
		}
		else
		{
			Serial.println("enter to supply cycle");
			controller.transitionTo(Pos1Supply);
		}
	}
	else
	{

	}
}

void feederToHomeExit()
{

}

void supplyPos1Enter()
{
	Serial.println(F("Carrier is moving to west in position 1."));

	digitalWrite(CARRIERPWR, RELAYON);
	digitalWrite(CARRIERFWD, RELAYON);
	digitalWrite(CARRIERREV, RELAYOFF);
	digitalWrite(SUPPLYPWR, RELAYON);
	digitalWrite(SUPPLYACT, RELAYON);
}

void supplyPos1Update()
{
	if (switchWestTON.Q)
	{
		// stop 
		digitalWrite(CARRIERPWR, RELAYOFF);
		digitalWrite(CARRIERFWD, RELAYOFF);
		digitalWrite(CARRIERREV, RELAYOFF);
		digitalWrite(SUPPLYPWR, RELAYOFF);
		digitalWrite(SUPPLYACT, RELAYOFF);
		controller.transitionTo(Pos12Pos2);
	}
}

void movePos1_2Enter()
{
	Serial.println(F("Feeder is moving to position 2."));
	digitalWrite(FEEDERPWR, RELAYON);
	digitalWrite(FEEDERFWD, RELAYON);
	digitalWrite(FEEDERREV, RELAYOFF);
	proxTriggered = false;
}

void movePos1_2Update()
{
	if (proxTriggered)
	{
		Serial.println(F("position2 is detected!"));

		digitalWrite(FEEDERPWR, RELAYOFF);
		digitalWrite(FEEDERFWD, RELAYOFF);
		digitalWrite(FEEDERREV, RELAYOFF);
		controller.transitionTo(Pos2Supply);
	}
}

void supplyPos2Enter()
{
	Serial.println(F("Carrier is moving to east in position 2."));

	digitalWrite(CARRIERPWR, RELAYON);
	digitalWrite(CARRIERFWD, RELAYOFF);
	digitalWrite(CARRIERREV, RELAYON);

	digitalWrite(SUPPLYPWR, RELAYON);
	digitalWrite(SUPPLYACT, RELAYON);
}

void supplyPos2Update()
{
	if (switchEastTON.Q)
	{
		// stop 
		digitalWrite(CARRIERPWR, RELAYOFF);
		digitalWrite(CARRIERFWD, RELAYOFF);
		digitalWrite(CARRIERREV, RELAYOFF);

		digitalWrite(SUPPLYPWR, RELAYOFF);
		digitalWrite(SUPPLYACT, RELAYOFF);

		controller.transitionTo(Pos22Pos3);
	}
}

void movePos2_3Enter()
{
	Serial.println(F("Feeder is moving to position 3."));

	digitalWrite(FEEDERPWR, RELAYON);
	digitalWrite(FEEDERFWD, RELAYON);
	digitalWrite(FEEDERREV, RELAYOFF);

	proxTriggered = false;
}

void movePos2_3Update()
{
	if (proxTriggered)
	{
		Serial.println(F("position3 is detected!"));

		digitalWrite(FEEDERPWR, RELAYOFF);
		digitalWrite(FEEDERFWD, RELAYOFF);
		digitalWrite(FEEDERREV, RELAYOFF);
		controller.transitionTo(Pos3Supply);
	}
}

void supplyPos3Enter()
{
	Serial.println(F("Carrier is moving to west in position 3."));

	digitalWrite(CARRIERPWR, RELAYON);
	digitalWrite(CARRIERFWD, RELAYON);
	digitalWrite(CARRIERREV, RELAYOFF);

	digitalWrite(SUPPLYPWR, RELAYON);
	digitalWrite(SUPPLYACT, RELAYON);
}

void supplyPos3Update()
{
	if (switchWestTON.Q)
	{
		// stop 
		digitalWrite(CARRIERPWR, RELAYOFF);
		digitalWrite(CARRIERFWD, RELAYOFF);
		digitalWrite(CARRIERREV, RELAYOFF);

		digitalWrite(SUPPLYPWR, RELAYOFF);
		digitalWrite(SUPPLYACT, RELAYOFF);

		controller.transitionTo(Pos32Pos4);
	}
}

void movePos3_4Enter()
{
	Serial.println(F("Feeder is moving to position 4."));

	digitalWrite(FEEDERPWR, RELAYON);
	digitalWrite(FEEDERFWD, RELAYON);
	digitalWrite(FEEDERREV, RELAYOFF);

	proxTriggered = false;
}

void movePos3_4Update()
{
	if (proxTriggered)
	{
		Serial.println(F("position4 is detected!"));

		digitalWrite(FEEDERPWR, RELAYOFF);
		digitalWrite(FEEDERFWD, RELAYOFF);
		digitalWrite(FEEDERREV, RELAYOFF);
		controller.transitionTo(Pos4Supply);
	}
}

void supplyPos4Enter()
{
	Serial.println(F("Carrier is moving to east in position 4."));

	digitalWrite(CARRIERPWR, RELAYON);
	digitalWrite(CARRIERFWD, RELAYOFF);
	digitalWrite(CARRIERREV, RELAYON);

	digitalWrite(SUPPLYPWR, RELAYON);
	digitalWrite(SUPPLYACT, RELAYON);
}

void supplyPos4Update()
{
	if (switchEastTON.Q)
	{
		// stop 
		digitalWrite(CARRIERPWR, RELAYOFF);
		digitalWrite(CARRIERFWD, RELAYOFF);
		digitalWrite(CARRIERREV, RELAYOFF);

		digitalWrite(SUPPLYPWR, RELAYOFF);
		digitalWrite(SUPPLYACT, RELAYOFF);

		controller.transitionTo(Standby);
	}
}

void standbyEnter()
{
	digitalWrite(CARRIERPWR, RELAYOFF);
	digitalWrite(CARRIERFWD, RELAYOFF);
	digitalWrite(CARRIERREV, RELAYOFF);

	digitalWrite(FEEDERPWR, RELAYOFF);
	digitalWrite(FEEDERFWD, RELAYOFF);
	digitalWrite(FEEDERREV, RELAYOFF);

	digitalWrite(SUPPLYPWR, RELAYOFF);
	digitalWrite(SUPPLYACT, RELAYOFF);

	Serial.println(F("Standby entered!"));
}

void standbyUpdate()
{
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
			controller.transitionTo(CarrierToHome);
			break;
		}
	}
}

void manualEnter()
{
	Serial.println("entered into manual Mode");
	
	digitalWrite(CARRIERPWR, RELAYOFF);
	digitalWrite(CARRIERFWD, RELAYOFF);
	digitalWrite(CARRIERREV, RELAYOFF);

	digitalWrite(FEEDERPWR, RELAYOFF);
	digitalWrite(FEEDERFWD, RELAYOFF);
	digitalWrite(FEEDERREV, RELAYOFF);

	digitalWrite(SUPPLYPWR, RELAYOFF);
	digitalWrite(SUPPLYACT, RELAYOFF);

	remotebutTrg.IN = false;
	remotebutTrg.PRE = false;
	remotebutTrg.Q = false;
}

void manualUpdate()
{
	if (buttonEastTON.Q)
	{

		if (switchEastTON.Q)
		{
			Serial.println(F("Manual mode, to East"));
			digitalWrite(CARRIERPWR, RELAYOFF);
			digitalWrite(CARRIERFWD, RELAYOFF);
			digitalWrite(CARRIERREV, RELAYOFF);
		}
		else
		{
			Serial.println(F("Manual mode, arrived to East"));
			digitalWrite(CARRIERPWR, RELAYON);
			digitalWrite(CARRIERFWD, RELAYOFF);
			digitalWrite(CARRIERREV, RELAYON);
		}

	}
	else if (buttonWestTON.Q)
	{
		if (switchWestTON.Q)
		{
			Serial.println(F("Manual mode, to west"));
			digitalWrite(CARRIERPWR, RELAYOFF);
			digitalWrite(CARRIERFWD, RELAYOFF);
			digitalWrite(CARRIERREV, RELAYOFF);
		}
		else
		{
			Serial.println(F("Manual mode, arrived to west"));
			digitalWrite(CARRIERPWR, RELAYON);
			digitalWrite(CARRIERFWD, RELAYON);
			digitalWrite(CARRIERREV, RELAYOFF);
		}
	}
	else
	{
		digitalWrite(CARRIERPWR, RELAYOFF);
		digitalWrite(CARRIERFWD, RELAYOFF);
		digitalWrite(CARRIERREV, RELAYOFF);
	}

	if (buttonNorthTON.Q)
	{
		if (switchNorthTON.Q)
		{
			Serial.println(F("Manual mode, arrived to north"));
			digitalWrite(FEEDERPWR, RELAYOFF);
			digitalWrite(FEEDERFWD, RELAYOFF);
			digitalWrite(FEEDERREV, RELAYOFF);
		}
		else
		{
			Serial.println(F("Manual mode, to north"));
			digitalWrite(FEEDERPWR, RELAYON);
			digitalWrite(FEEDERFWD, RELAYON);
			digitalWrite(FEEDERREV, RELAYOFF);
		}
	}
	else if (buttonSouthTON.Q)
	{
		if (switchSouthTON.Q)
		{
			Serial.println(F("Manual mode, to south"));
			digitalWrite(FEEDERPWR, RELAYOFF);
			digitalWrite(FEEDERFWD, RELAYOFF);
			digitalWrite(FEEDERREV, RELAYOFF);
		}
		else
		{
			Serial.println(F("Manual mode, arrived to south"));
			digitalWrite(FEEDERPWR, RELAYON);
			digitalWrite(FEEDERFWD, RELAYOFF);
			digitalWrite(FEEDERREV, RELAYON);
		}
	}
	else
	{
		digitalWrite(FEEDERPWR, RELAYOFF);
		digitalWrite(FEEDERFWD, RELAYOFF);
		digitalWrite(FEEDERREV, RELAYOFF);
	}

	remotebutTrg.IN = (buttonEastTON.Q || buttonWestTON.Q) || (buttonNorthTON.Q || buttonSouthTON.Q);
	FTrgFunc(&remotebutTrg);
	if (remotebutTrg.Q)
	{
		controller.transitionTo(Standby);
		Serial.println(F("All remote button are released, will be enter standby mode!"));
	}
}

// 
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
	initFBDs();
	Serial.println("System Startup");
}

static uint32_t nRTCTime = millis();
const uint32_t RTCINTERVAL = 2000;
void loop()
{

	switchEastTON.IN = (digitalRead(EASTSWITCH) == false);
	TONFunc(&switchEastTON);

	switchWestTON.IN = (digitalRead(WESTSWITCH) == false);
	TONFunc(&switchWestTON);

	switchSouthTON.IN = (digitalRead(SOUTHSWITCH) == false);
	TONFunc(&switchSouthTON);

	switchNorthTON.IN = (digitalRead(NORTHSWITCH) == false);
	TONFunc(&switchSouthTON);

	buttonEastTON.IN = digitalRead(EASTBUTTON) == false;
	TONFunc(&buttonEastTON);

	buttonWestTON.IN = digitalRead(WESTBUTTON) == false;
	TONFunc(&buttonWestTON);

	buttonSouthTON.IN = digitalRead(SOUTHBUTTON) == false;
	TONFunc(&buttonSouthTON);

	buttonNorthTON.IN = digitalRead(NORTHBUTTON) == false;
	TONFunc(&buttonNorthTON);

	// enter into manual
	if ((buttonEastTON.Q || buttonWestTON.Q) || (buttonSouthTON.Q || buttonNorthTON.Q))
	{
		if(!controller.isInState(Manual))
			controller.transitionTo(Manual);
	}

	// FSM instance update
	controller.update();

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
	proxTriggered = true;
}