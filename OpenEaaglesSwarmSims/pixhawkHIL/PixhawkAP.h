#pragma once

//------------------------------------------------------------------------------
// Class: SimAP
//------------------------------------------------------------------------------
#ifndef __Eaagles_Swarms_PixhawkAP_H__
#define __Eaagles_Swarms_PixhawkAP_H__

#include "openeaagles/dynamics/JSBSimModel.h"
#include "AutoPilot.h"

#include "checksum.h"
#include "mavlink_types.h"
#include "common\mavlink.h"
#include "pixhawk\mavlink.h"
#include "Serial.h"
#include "windows.h"
#include "openeaagles/basic/Thread.h"
#include <fstream>  // used to write to file
#include <chrono>   // used for runtime calculation in usec precision
#include <mutex>

using namespace std;

namespace Eaagles {
	namespace Basic { class Number; }
	namespace Simulation { class Vehicle; }

namespace Swarms {

class PixhawkAP : public AutoPilot
{
	DECLARE_SUBCLASS(PixhawkAP, AutoPilot)

	enum State { SEND_COUNT, AWAIT_REQ, SEND_ITEM, AWAIT_ACK, START_NAV, END_MISSION };

public:
	PixhawkAP();

	virtual int getPortNum() const { return portNum; }
	virtual uint64_t sinceSystemBoot() const;
	virtual const char* getMode() const;
	virtual const char* getTextMsging() const;
	virtual bool setPortNum(const int p);
	virtual bool setMode(const Basic::String* const m);
	virtual bool setTextMsging(const Basic::String* const m);
	virtual void updateData(const LCreal dt = 0.0);
	virtual bool sendMessage(mavlink_message_t* msg);

	bool isInitialized();
	bool isReceiving() { return receiving; }
	
	CSerial serial;
	mutex sendMutex;
	ofstream outputfile;        // DELETE ME... USED FOR TESTING ONLY!!!
	int wpTimeoutCount = 0;
	bool reqRcvd = false;
	bool ackRcvd = false;
	void setAileron(double input) {aileron = input;}
	void setElevator(double input) {elevator = input;}
	void setRudder(double input) {rudder = input;}
	void setThrottle(double input) {throttle = input;}

protected:
	bool setSlotPortNum(const Basic::Number* const msg);
	bool setSlotMode(const Basic::String* const msg);
	bool setSlotTextMsging(const Basic::String* const msg);
	
	void flyUav();
	void sendSimStateToPX4();

private:
	bool createReceivingThread();
	bool connectToPixhawk();
	Basic::safe_ptr<Basic::Thread> rcvThread;

	int portNum;
	bool receiving;
	chrono::system_clock::time_point start_time;
	chrono::system_clock::time_point timeout_start;
	State missionState = SEND_COUNT;
	int itemSendCount = 0;
	double aileron  = 0;
	double elevator = 0;
	double rudder   = 0;
	double throttle = 0;
	uint8_t sid = 255;
	uint8_t cid = 0;

	Eaagles::Basic::safe_ptr<const Basic::String> mode;
	Eaagles::Basic::safe_ptr<const Basic::String> textMsging;
};

}
}
#endif