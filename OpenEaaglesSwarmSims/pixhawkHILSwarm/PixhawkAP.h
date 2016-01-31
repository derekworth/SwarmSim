#pragma once

//------------------------------------------------------------------------------
// Class: SimAP
//------------------------------------------------------------------------------
#ifndef __Eaagles_Swarms_PixhawkAP_H__
#define __Eaagles_Swarms_PixhawkAP_H__

#include "openeaagles/dynamics/JSBSimModel.h"
#include "SwarmAutopilot.h"

#include "checksum.h"
#include "mavlink_types.h"
#include "common\mavlink.h"
#include "pixhawk\mavlink.h"
#include "Serial.h"
#include "windows.h"
#include <fstream>  // used to write to file
#include <chrono>   // used for runtime calculation in usec precision
#include <mutex>

using namespace std;

namespace Eaagles {
	namespace Basic { class Number; }
	namespace Simulation { class Vehicle; }

namespace Swarms {

class PixhawkAP : public SwarmAutopilot
{
	DECLARE_SUBCLASS(PixhawkAP, SwarmAutopilot)

	enum MsnState { SEND_COUNT, AWAIT_REQ, SEND_ITEM, AWAIT_ACK, INTERMISSION };

public:
	PixhawkAP();

	// getters
	virtual int getPortNum() const { return portNum; }
	virtual const char* getMode() const;
	virtual const char* getStatustexts() const;
	uint8_t  getHbSid()            const { return hbSid; }
	uint8_t  getHbCid()            const { return hbCid; }
	uint8_t  getHbType()           const { return hbType; }
	uint8_t  getHbAutopilot()      const { return hbAutopilot; }
	uint8_t  getHbBaseMode()       const { return hbBaseMode; }
	uint32_t getHbCustomMode()     const { return hbCustomMode; }
	uint8_t  getHbSystemStatus()   const { return hbSystemStatus; }
	uint8_t  getHbMavlinkVersion() const { return hbMavlinkVersion; }
	double   getMiLat()            const { return miLat; }
	double   getMiLon()            const { return miLon; }
	double   getMiAlt()            const { return miAlt; }
	uint16_t getMcCnt()            const { return mcCnt; }
	double   getHcRollCtrl()       const { return hcRollCtrl; }
	double   getHcPitchCtrl()      const { return hcPitchCtrl; }
	double   getHcYawCtrl()        const { return hcYawCtrl; }
	double   getHcThrottleCtrl()   const { return hcThrottleCtrl; }
	uint8_t  getHcSysMode()        const { return hcSysMode; }
	uint8_t  getHcNavMode()        const { return hcNavMode; }
	bool     getMsnCntSent()       const { return msnCntSent; }
	bool     getMsnReqRcvd()       const { return msnReqRcvd; }
	bool     getMsnItmSent()       const { return msnItmSent; }
	bool     getMsnAckRcvd()       const { return msnAckRcvd; }
	int      getMsnTimeout()       const { return msnTimeout; }
	int      getSerialDataWaiting() { return serial.ReadDataWaiting(); }
	bool     isSerialOpen() { return serial.IsOpened(); }

	// setters
	virtual void setWaypoint(const osg::Vec3& posNED, const LCreal altMeters);
	void setSerialReadData(void* lpBuffer, int bufferSize) { serial.ReadData(lpBuffer, bufferSize); }
	virtual void setPortNum(const int p) { portNum = p; }
	virtual void setMode(const Basic::String* const m) { mode = m; }
	virtual void setStatustexts(const Basic::String* const m) { statustexts = m; }
	void setHbSid(const uint8_t sid)            { hbSid = sid; }
	void setHbCid(const uint8_t cid)            { hbCid = cid; }
	void setHbType(const uint8_t type)          { hbType = type; }
	void setHbAutopilot(const uint8_t ap)       { hbAutopilot = ap; }
	void setHbBaseMode(const uint8_t bmode)     { hbBaseMode = bmode; }
	void setHbCustomMode(const uint8_t cmode)   { hbCustomMode = cmode; }
	void setHbSystemStatus(const uint8_t stat)  { hbSystemStatus = stat; }
	void setHbMavlinkVersion(const uint8_t ver) { hbMavlinkVersion = ver; }
	void setMiLat(const double lat)             { miLat = lat; }
	void setMiLon(const double lon)             { miLon = lon; }
	void setMiAlt(const double alt)             { miAlt = alt; }
	void setMcCnt(const uint16_t cnt)           { mcCnt = cnt; }
	void setHcRollCtrl(const double rctrl)      { hcRollCtrl = rctrl; }
	void setHcPitchCtrl(const double pctrl)     { hcPitchCtrl = pctrl; }
	void setHcYawCtrl(const double yctrl)       { hcYawCtrl = yctrl; }
	void setHcThrottleCtrl(const double tctrl)  { hcThrottleCtrl = tctrl; }
	void setHcSysMode(const uint8_t smode)      { hcSysMode = smode; }
	void setHcNavMode(const uint8_t nmode)      { hcNavMode = nmode; }
	void setMsnCntSent(const bool mcs)          { msnCntSent = mcs; }
	void setMsnReqRcvd(const bool mrr)          { msnReqRcvd = mrr; }
	void setMsnItmSent(const bool mis)          { msnItmSent = mis; }
	void setMsnAckRcvd(const bool mar)          { msnAckRcvd = mar; }
	void setMsnTimeout(const int  mto)          { msnTimeout = mto; }

	// utility methods
	virtual bool sendMessage(mavlink_message_t* msg);
	virtual bool sendBytes(char* msg);
	virtual uint64_t sinceSystemBoot();
	bool isInitialized();
	double* rollPitchYaw(double x, double y, double z, bool inDegrees, bool reverse, double phi, double theta, double psi);
	void recordMessage(uint8_t msgid, bool sending, int byteCnt);
	
	void setRollControl(double input)     { hcRollCtrl     = input; }
	void setPitchControl(double input)    { hcPitchCtrl    = input; }
	void setYawControl(double input)      { hcYawCtrl      = input; }
	void setThrottleControl(double input) { hcThrottleCtrl = input; }

	// update method (time critical)
	virtual void dynamics(const LCreal dt) override;

protected:
	bool setSlotPortNum(const Basic::Number* const msg);
	bool setSlotMode(const Basic::String* const msg);
	bool setSlotStatustexts(const Basic::String* const msg);
	
	void updatePX4();
	void sendHeartbeat();
	void sendSetMode();
	void sendHilGps();
	void sendHilSensor();
	void sendDynamicWaypoint();
	void updateMagValues();
	void receive();

private:
	bool connectToPixhawk();

	// communications management attributes
	CSerial serial;
	mutex sendMutex;
	mutex recordMutex;
	chrono::system_clock::time_point startTime;
	bool startTimeSet;
	mavlink_message_t msg1; // heartbeat
	mavlink_message_t msg2; // set_mode, hil_sensor, hil_gps 
	mavlink_message_t msg3; // mission_<item/request/request_list/count/ack>
	// receiving attributes
	mavlink_message_t rcvMsg;
	mavlink_status_t mavStatus;
	char text[MAVLINK_MSG_ID_STATUSTEXT_LEN + 1];
	int bufferSize;
	int bytesRead;
	char* lpBuffer;

	bool sentInitCmd;
	// used for synchronizing 'send' frequencies
	uint64_t hil_gps_time;
	uint64_t hil_sensor_time;
	uint64_t heartbeat_time;
	uint64_t set_mode_time;
	uint64_t mag_time;

	float xmag, ymag, zmag;

	uint8_t sid;
	uint8_t cid;

	//-------------------------
	// data received from PX4
	//-------------------------
	// HEARTBEAT (0)
	uint8_t  hbSid;
	uint8_t  hbCid;
	uint8_t  hbType;
	uint8_t  hbAutopilot;
	uint8_t  hbBaseMode;
	uint32_t hbCustomMode;
	uint8_t  hbSystemStatus;
	uint8_t  hbMavlinkVersion;
	// MISSION_ITEM (39)	    
	double miLat;
	double miLon;
	double miAlt;			  
	// MISSION_COUNT (44)	   
	uint16_t mcCnt;
	// HIL_CONTROLS (91)	   
	double  hcRollCtrl;
	double  hcPitchCtrl;
	double  hcYawCtrl;
	double  hcThrottleCtrl;
	uint8_t hcSysMode;
	uint8_t hcNavMode;

	// dynamic waypoint attributes
	double dwLat;
	double dwLon;
	double dwAlt;
	osg::Vec3 dwLLA;

	// waypoint handshake attributes
	bool msnCntSent;
	bool msnReqRcvd;
	bool msnItmSent;
	bool msnAckRcvd;
	uint64_t  msnTimeout;
	int msnTimeoutCount;
	MsnState currState;

	// mavlink message tracking
	int     messageTSs[10000]; // timestamps
	uint8_t messageIDs[10000]; // IDs
	bool    messageDRs[10000]; // directions (i.e. sending/receiving)
	int     messageBCs[10000]; // byte count
	int messageIDsIndex;
	bool printedMessageIDs;
	double messageIDStartTime; // in seconds

	//-------------------------
	// initial configurations
	//-------------------------
	int portNum;
	Eaagles::Basic::safe_ptr<const Basic::String> mode;
	Eaagles::Basic::safe_ptr<const Basic::String> statustexts;
};

}
}
#endif