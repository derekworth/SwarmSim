#include "PixhawkAP.h"
#include "UAV.h"

#include "openeaagles/basic/Number.h"
#include "openeaagles/basic/Thread.h"
#include "openeaagles/basic/Nav.h"
#include "openeaagles/basic/osg/Vec3"
#include "openeaagles/basic/EarthModel.h"
#include "openeaagles/simulation/Player.h"
#include "openeaagles/simulation/Navigation.h"
#include "openeaagles/simulation/Steerpoint.h"
#include "openeaagles/simulation/Route.h"
#include "openeaagles/simulation/Simulation.h"
#include "openeaagles/dynamics/JSBSimModel.h"
#include "JSBSim/FGFDMExec.h"
#include "JSBSim/models/FGFCS.h"
#include "JSBSim/models/FGAtmosphere.h"
#include "JSBSim/models/FGAuxiliary.h"

// used for testing
#include <iostream>
#include <iomanip>
#include <thread>
#include <conio.h>

#define PI 3.1415926535897932384626433832795

namespace Eaagles {
namespace Swarms {

// =============================================================================
// class: PixhawkAP
// =============================================================================

IMPLEMENT_SUBCLASS(PixhawkAP,"PixhawkAP")

BEGIN_SLOTTABLE(PixhawkAP)
	"portNum",
	"mode",
	"statustexts",
END_SLOTTABLE(PixhawkAP)

BEGIN_SLOT_MAP(PixhawkAP)
	ON_SLOT(1, setSlotPortNum,     Basic::Number) // 1) USB COM port assigned to specified PX4
	ON_SLOT(2, setSlotMode,        Basic::String) // 2) modes: "nav" and "swarm" (autonomous flight)
	                                              //    default = "nav"
	ON_SLOT(3, setSlotStatustexts, Basic::String) // 3) Statustexts refers to printing to console the contents
	                                              //    of mavlink STATUSTEXT (#253) message received from PX4
												  //    default = "off"
END_SLOT_MAP()                                                                        

Eaagles::Basic::Object* PixhawkAP::getSlotByIndex(const int si) {                                                                                     
	return BaseClass::getSlotByIndex(si);
}

EMPTY_SERIALIZER(PixhawkAP)

//------------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------------
PixhawkAP::PixhawkAP() 
 : rcvThread(nullptr) {
	STANDARD_CONSTRUCTOR()

	receiving        = false;
	sentInitCmd      = false;
    gpsCount         = rand() % 6;	// start randomly between 0 - 5
    hbCount          = rand() % 60;	// start randomly between 0 - 59
	smCount          = 0;
	sid              = 255;
	cid              = 0;
	hbSid			 = 0;
	hbCid			 = 0;
	hbType			 = 0;
	hbAutopilot		 = 0;
	hbBaseMode		 = 0;
	hbCustomMode	 = 0;
	hbSystemStatus	 = 0;
	hbMavlinkVersion = 0;
	miLat            = 0;
	miLon            = 0;
	miAlt            = 0;
	mcCnt            = 0;
	hcRollCtrl	     = 0.0;
	hcPitchCtrl	     = 0.0;
	hcYawCtrl	     = 0.0;
	hcThrottleCtrl   = 0.8;
	hcSysMode    	 = 0;
	hcNavMode    	 = 0;
	dwLat            = 0;
	dwLon            = 0;
	dwAlt            = 0;
	dwLLA.set(0, 0, 0);
	msnCntSent       = false;
	msnReqRcvd       = false;
	msnItmSent       = false;
	msnAckRcvd       = false;
	msnTimeout       = 0;
	currState        = SEND_COUNT;
	portNum          = 0;
	mode = nullptr;
	setMode(new Basic::String("nav"));
	statustexts = nullptr;
	setStatustexts(new Basic::String("off"));
}

//------------------------------------------------------------------------------------
// copyData() - copies one object to another
//------------------------------------------------------------------------------------
void PixhawkAP::copyData(const PixhawkAP& org, const bool cc) {
	BaseClass::copyData(org);

	receiving        = org.receiving;
	sentInitCmd      = org.sentInitCmd;
    gpsCount         = org.gpsCount;
    hbCount          = org.hbCount;
	smCount          = org.smCount;
	sid              = org.sid;
	cid              = org.cid;
	hbSid			 = org.hbSid;
	hbCid			 = org.hbCid;
	hbType			 = org.hbType;
	hbAutopilot		 = org.hbAutopilot;
	hbBaseMode		 = org.hbBaseMode;
	hbCustomMode	 = org.hbCustomMode;
	hbSystemStatus	 = org.hbSystemStatus;
	hbMavlinkVersion = org.hbMavlinkVersion;
	miLat            = org.miLat;
	miLon            = org.miLon;
	miAlt            = org.miAlt;
	mcCnt            = org.mcCnt;
	hcRollCtrl	     = org.hcRollCtrl;
	hcPitchCtrl	     = org.hcPitchCtrl;
	hcYawCtrl	     = org.hcYawCtrl;
	hcThrottleCtrl   = org.hcThrottleCtrl;
	hcSysMode    	 = org.hcSysMode;
	hcNavMode    	 = org.hcNavMode;
	portNum          = org.portNum;
	dwLat            = org.dwLat;
	dwLon            = org.dwLon;
	dwAlt            = org.dwAlt;
	dwLLA            = org.dwLLA;
	msnCntSent       = org.msnCntSent;
	msnReqRcvd       = org.msnReqRcvd;
	msnItmSent       = org.msnItmSent;
	msnAckRcvd       = org.msnAckRcvd;
	msnTimeout       = org.msnTimeout;
	currState        = SEND_COUNT;

	if(cc) {
		mode = nullptr;
		statustexts = nullptr;
	}

	Basic::String* m = nullptr;
	if (org.mode != nullptr) m = org.mode->clone();
	mode = m;
	if (m != 0) m->unref();

	Basic::String* s = nullptr;
	if (org.statustexts != nullptr) m = org.statustexts->clone();
	statustexts = s;
	if (s != 0) s->unref();

	// We need to init this ourselves, so ...
	if (rcvThread != nullptr) {
		rcvThread->terminate();
		rcvThread = nullptr;
	}
}

//------------------------------------------------------------------------------------
// deleteData() -- delete this object's data
//------------------------------------------------------------------------------------
void PixhawkAP::deleteData() {
	receiving = false;
	serial.Close();
	mode = nullptr;
	statustexts = nullptr;
	if (rcvThread != nullptr) {
		rcvThread->terminate();
		rcvThread = nullptr;
	}
}

//------------------------------------------------------------------------------------
// Rotation - rotate a coordinate around the origin by roll(phi), pitch(theta), and yaw(psi) angles
//------------------------------------------------------------------------------------

double* PixhawkAP::rollPitchYaw(double x, double y, double z, bool inDegrees, bool reverse, double phi, double theta, double psi) {
	// convert degrees to radians
	if (inDegrees) {
		phi   *= PI / 180;
		theta *= PI / 180;
		psi   *= PI / 180;
	}

	// reverse the Euler angles
	if (reverse) {
		phi   = -phi;
		theta = -theta;
		psi   = -psi;
	}

	double Rx[3][3] = { { 1, 0, 0 }, { 0, cos(phi), -sin(phi) }, { 0, sin(phi), cos(phi) } };
	double Ry[3][3] = { { cos(theta), 0, sin(theta) }, { 0, 1, 0 }, { -sin(theta), 0, cos(theta) } };
	double Rz[3][3] = { { cos(psi), -sin(psi), 0 }, { sin(psi), cos(psi), 0 }, { 0, 0, 1 } };

	double S0[3] = { x, y, z };

	double S1[3] = { 0, 0, 0 };
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			if (reverse)
				S1[i] += Rz[i][j] * S0[j]; // Yaw
			else
				S1[i] += Rx[i][j] * S0[j]; // Roll

	double S2[3] = { 0, 0, 0 };
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			S2[i] += Ry[i][j] * S1[j]; // Pitch

	double S3[3] = { 0, 0, 0 };
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			if (reverse)
				S3[i] += Rx[i][j] * S2[j]; // Roll
			else
				S3[i] += Rz[i][j] * S2[j]; // Yaw

	return S3;
}

//------------------------------------------------------------------------------
// Getter methods
//------------------------------------------------------------------------------

const char* PixhawkAP::getMode() const {
	const char* p = 0;
	if (mode != 0) p = *mode;
	return p;
}

const char* PixhawkAP::getStatustexts() const {
	const char* p = 0;
	if (statustexts != 0) p = *statustexts;
	return p;
}

//------------------------------------------------------------------------------------
// Setter methods
//------------------------------------------------------------------------------------

// used by OCA to update Dynamic Waypoint
void PixhawkAP::setWaypoint(const osg::Vec3& posNED, const LCreal altMeters) {
	if (strcmp(*mode, "swarm") != 0) return;
	dwLLA.set(posNED.x(), posNED.y(), posNED.z());

	Simulation::Simulation* s = getOwnship()->getSimulation();
	const double maxRefRange = s->getMaxRefRange();
	const Basic::EarthModel* em = s->getEarthModel();

	// Compute & set the lat/lon/alt position
	const double refLat = s->getRefLatitude();
	const double refLon = s->getRefLongitude();
	const double cosRlat = s->getCosRefLat();

	if (s->isGamingAreaUsingEarthModel()) {
		const double sinRlat = s->getSinRefLat();
		Basic::Nav::convertPosVec2llE(refLat, refLon, sinRlat, cosRlat, dwLLA, &dwLat, &dwLon, &dwAlt, em);
	} else {
		Basic::Nav::convertPosVec2llS(refLat, refLon, cosRlat, dwLLA, &dwLat, &dwLon, &dwAlt);
	}

	dwAlt = altMeters;
}

//------------------------------------------------------------------------------------
// Slot methods
//------------------------------------------------------------------------------------

bool PixhawkAP::setSlotPortNum(const Basic::Number* const msg) {
	bool ok = (msg != nullptr);
	if (ok) setPortNum(msg->getInt());
	// establish connection to Pixhawk over given port and startup receiver thread
	return ok;
}

bool PixhawkAP::setSlotMode(const Basic::String* const msg) {
	bool ok = false;
	if (msg != nullptr) {
		if (strcmp(msg->getString(), "nav") == 0) { setMode(msg); }
		else setMode(new Basic::String("swarm"));
		ok = true;
	}
	return ok;
}

bool PixhawkAP::setSlotStatustexts(const Basic::String* const msg) {
	bool ok = false;
	if (msg != nullptr) {
		if (strcmp(msg->getString(), "on") == 0) { setStatustexts(msg); }
		else setStatustexts(new Basic::String("off"));
		ok = true;
	}
	return ok;
}

//------------------------------------------------------------------------------
// Tracks how long this program has been running
//------------------------------------------------------------------------------

// time (in usec) since program started
uint64_t PixhawkAP::sinceSystemBoot() const {
	return chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - startTime).count();
}

//------------------------------------------------------------------------------
// Send UAV simulation state (i.e. UAV position/attitude/velocity/etc.) to PX4
//------------------------------------------------------------------------------

// HEARTBEAT (0)
void PixhawkAP::sendHeartbeat() {
	if (++hbCount >= 60) hbCount = 0; else return; // control heartbeat refresh rate

	if (hbSid == 0 && !sentInitCmd) {
		// ask pixhawk to start communicating over mavlink
		sendMutex.lock();
		// "...sh /etc/init.d/rc.usb....." used to initiate communications
		char* buff = "\x0d\x0d\x0d\x73\x68\x20\x2f\x65\x74\x63\x2f\x69\x6e\x69\x74\x2e\x64\x2f\x72\x63\x2e\x75\x73\x62\x0a\x0d\x0d\x0d\x00";
		int bytesSent = serial.SendData(buff, 29);
		sendMutex.unlock();
		sentInitCmd = true;
	} else if (hbSid != 0) {
		mavlink_msg_heartbeat_pack(sid, cid, &msg1, 6, 8, 192, 0, 4);
		sendMessage(&msg1);
		// set mode to HIL

		if (++smCount >= 5) smCount = 0; else return; // control set_mode refresh rate
		if (hbBaseMode != 189) {                      // set mode to HIL if not already
			mavlink_msg_set_mode_pack(sid, cid, &msg2, 1, 189, 262144);
			sendMessage(&msg2);
		}
	}
}

// HIL_SENSOR (107)
void PixhawkAP::sendHilSensor() {
	// get UAV
	UAV* uav = dynamic_cast<UAV*>(getOwnship());
	if (uav == nullptr) return;
	// get UAV's dynamics model
	Eaagles::Dynamics::JSBSimModel* dm = dynamic_cast<Eaagles::Dynamics::JSBSimModel*>(uav->getDynamicsModel());
	if (dm == nullptr) return;

	float xacc = uav->getAcceleration().x();
	float yacc = uav->getAcceleration().y();
	float zacc = uav->getAcceleration().z() - 9.68235;
	//cout << "\racc:" << xacc << "\t" << yacc << "\t" << zacc << "                          ";

	float xgyro = uav->getAngularVelocities().x();
	float ygyro = uav->getAngularVelocities().y();
	float zgyro = uav->getAngularVelocities().z();
	//cout << "\rgyro:" << xgyro << "\t" << ygyro << "\t" << zgyro << "                          ";

	double phiR   = uav->getRollR();
	double thetaR = uav->getPitchR();
	double psiR   = uav->getHeadingR();
	//cout << "roll: " << phiR << " | pitch: " << thetaR << " | yaw: " << psiR << endl;

	// Static north pointing unit vector of magnetic field in the Inertial (NED) Frame 15K ft MSL
	// over USAFA Airfield on 12/29/2015 (dec/inc of 8.3085/65.7404 degs respectively)
	float xmag = 0.902124413;
	float ymag = 0.131742403;
	float zmag = 0.410871614;

	// Rotate vector
	double* reverseRotation = rollPitchYaw(xmag, ymag, zmag, false, true, phiR, thetaR, psiR);
	xmag = reverseRotation[0];
	ymag = reverseRotation[1];
	zmag = reverseRotation[2];
	//cout << "\rrollCtrl: " << hcRollCtrl << " | pitchCtrl: " << hcPitchCtrl << " | yawCtrl: " << hcYawCtrl << "                                  "; //###
	//cout << "\rmag:" << xmag << "\t" << ymag << "\t" << zmag << "                          ";

	JSBSim::FGFDMExec* fdmex = dm->getJSBSim();                if (fdmex == nullptr) return;      // get JSBSim
	JSBSim::FGAtmosphere* Atmosphere = fdmex->GetAtmosphere(); if (Atmosphere == nullptr) return; // get atmosphere from JSBSim
	JSBSim::FGAuxiliary* Auxiliary = fdmex->GetAuxiliary();    if (Auxiliary == nullptr) return;  // get auxiliary from JSBSim

	// get absolute pressure (in millibars)
	float abs_pressure = static_cast<LCreal>(Atmosphere->GetPressure()) * 68.9475728 / 144;

	// get pressure altitude
	double altFt = static_cast<LCreal>(Atmosphere->GetPressureAltitude()); // in feet
	float pressure_alt = altFt / 3.28083989502;                            // convert to meters

	// get differential pressure (in millibars)
	double density = Atmosphere->GetDensity(altFt) * 515.379;             // convert from slug/ft^3 to kg/m^3
	double velocity = Auxiliary->GetVtrueKTS() * 0.514444444;             // convert from Kts to m/s
	float diff_pressure = ((density * velocity * velocity * 0.5) * 0.01); // convert from pascal to millibar

	// get temperature (in Celsius)
	float temperature = (static_cast<LCreal>(Atmosphere->GetTemperature()) - 491.67) / 1.8; // convert from Rankine to Celsius using 

	//cout << "\rabs_pressure: " << abs_pressure << "\tdiff_pressure: " << diff_pressure << "\tpressure_alt: " << pressure_alt << "\ttemperature: " << temperature << "                          ";

	// send HIL_SENSOR and HIL_GPS messages
	mavlink_msg_hil_sensor_pack(sid, cid, &msg2, sinceSystemBoot(),
		xacc,
		yacc,
		zacc,
		xgyro,
		ygyro,
		zgyro,
		xmag,
		ymag,
		zmag,
		abs_pressure,
		diff_pressure,
		pressure_alt,
		temperature,
		4095);

	sendMessage(&msg2);
}

// HIL_GPS (113)
void PixhawkAP::sendHilGps() {
	if (++gpsCount >= 6) gpsCount = 0; else return; // control refresh rate

	UAV* uav = dynamic_cast<UAV*>(getOwnship());
	if (uav != nullptr) {
		Eaagles::Dynamics::JSBSimModel* dm = dynamic_cast<Eaagles::Dynamics::JSBSimModel*>(uav->getDynamicsModel());
		if (dm == nullptr) return;

		uint8_t  fix_type = 3;
		double dlat, dlon, dalt;
		uav->getPositionLLA(&dlat, &dlon, &dalt);
		int32_t  lat = (int32_t)(dlat * 10000000);
		int32_t  lon = (int32_t)(dlon * 10000000);
		int32_t  alt = (int32_t)(dalt * 1000);
		uint16_t eph = 30;
		uint16_t epv = 60;
		uint16_t vel = uav->getGroundSpeed() * 100;
		int16_t  vn = uav->getVelocity().x() * 100;
		int16_t  ve = uav->getVelocity().y() * 100;
		int16_t  vd = uav->getVelocity().z() * 100;
		
		double   cog_signed = uav->getGroundTrackD();
		if (cog_signed < 0) cog_signed = 360 + cog_signed; // convert from -180 thru 180 to 0 thru 360
		uint16_t cog = cog_signed * 100;

		uint8_t  satellites_visible = 8;

		mavlink_msg_hil_gps_pack(sid, cid, &msg2, sinceSystemBoot(),
			fix_type,
			lat,
			lon,
			alt,
			eph,
			epv,
			vel,
			vn,
			ve,
			vd,
			cog,
			satellites_visible);

		sendMessage(&msg2);
	}
}

// Dynamic Waypoint Following (DWF)
void PixhawkAP::sendDynamicWaypoint() {
	/*
	 ___________________________________________________
	| Sequence Diagram for waypoint updates:            |
	|___________________________________________________|
	|           SIM          |          PX4             |
	|________________________|__________________________|
	|                        |                          |
	| MISSION_COUNT (44) --> |                          |
	|                        |                          |
	|                        | <-- MISSION_REQUEST (40) |
	|                        |                          |
	|  MISSION_ITEM (39) --> |                          |
	|                        |                          |
	|                        | <-- MISSION_ACK (47)     |
	|________________________|__________________________|

	*/
	switch (currState) {
	case SEND_COUNT:
		currState = AWAIT_REQ;
		msnReqRcvd = false;
		msnTimeout = 0;
		// send MISSION_COUNT (44)
		mavlink_msg_mission_count_pack(sid, cid, &msg3, 1, 190, 1);
		sendMessage(&msg3);
		break;
	case AWAIT_REQ:
		if (msnReqRcvd)
			currState = SEND_ITEM;
		else if (++msnTimeout >= 25) // wait some arbitrary time before re-sending message
			currState = SEND_COUNT;
		break;
	case SEND_ITEM:
		currState = AWAIT_ACK;
		msnAckRcvd = false;
		msnTimeout = 0;
		// check for RTL mode
		if (hbCustomMode == 84148224) {
			Swarms::UAV* uav = dynamic_cast<Swarms::UAV*>(this->getOwnship());
			if (uav == 0) return;
			uav->getPositionLLA(&dwLat, &dwLon, &dwAlt);	
		}
		// send MISSION_ITEM (39)
		//cout << "\rlat: " << dwLat << "\tlon: " << dwLon << "\talt: " << dwAlt << "                                            ";
		mavlink_msg_mission_item_pack(sid, cid, &msg3, 1, 190, 0, 0, 16, 1, 1, 0, 25, 0, 0, dwLat, dwLon, dwAlt);
		sendMessage(&msg3);
		break;
	case AWAIT_ACK:
		if (msnAckRcvd) {
			currState = INTERMISSION;
			msnTimeout = 0;
		} else if (++msnTimeout >= 25) // wait before re-sending
			currState = SEND_ITEM;
		break;
	case INTERMISSION:
		if (++msnTimeout >= 300) // wait a few seconds between dynamic waypoint updates
			currState = SEND_COUNT;
		break;
	}
}

void PixhawkAP::updatePX4() {
	if (!isInitialized()) return;

	sendHeartbeat();
	sendHilSensor();
	sendHilGps();
	sendDynamicWaypoint();
}

//------------------------------------------------------------------------------
// dynamics() is called by updateTC() and therefore a time-critical method.
// Autopilot updates are time-critical because control inputs must sync with
// the FDM (i.e. JSBSim) which is also a time-critical process.
//------------------------------------------------------------------------------

void PixhawkAP::dynamics(const LCreal dt) {
	// initialize HIL if not done so already
	if (!serial.IsOpened()) { // open serial connection if not already
		if (!connectToPixhawk()) { // attempt to open serial port to PX4
			cout << "ERROR: failed to establish connection to PX4." << endl;
			_getch();
			exit(0);
		} else {
			startTime = std::chrono::high_resolution_clock::now(); // set start time of program execution
		}
	}
	// start receiving thread if not already
	if (rcvThread == nullptr) {
		if (!createReceivingThread()) { // attempt to start listening for mavlink msgs
			cout << "ERROR: failed to start receiving thread." << endl;
			_getch();
			exit(0);
		}
	}

	// allow manual flight
	if (mode == 0) return;

	// get UAV
	Swarms::UAV* uav = dynamic_cast<Swarms::UAV*>(this->getOwnship());
	if (uav == 0) return;

	// get UAV's dynamics model (i.e. JSBSim)
	Eaagles::Dynamics::JSBSimModel* dm = dynamic_cast<Eaagles::Dynamics::JSBSimModel*>(uav->getDynamicsModel());
	if (dm == 0) return;

	// provide control inputs to JSBSim
	dm->setControlStickPitchInput(hcPitchCtrl);
	dm->setControlStickRollInput(hcRollCtrl);
	dm->setRudderPedalInput(hcYawCtrl);
	dm->setThrottles(&hcThrottleCtrl, 1);

	// push UAV attitude/position/etc to PX4
	updatePX4();

	BaseClass::updateData(dt);
}

//------------------------------------------------------------------------------
// USB connection management
//------------------------------------------------------------------------------

bool PixhawkAP::connectToPixhawk() {
	if (!serial.Open(portNum, 9600)) {
		cout << "Failed to open port (" << portNum << ") :(" << endl;
		return false;
	}
	cout << "Connected to PX4 over COM port " << portNum << "." << endl;
	return true;
}

bool PixhawkAP::sendMessage(mavlink_message_t* msg) {
	sendMutex.lock();
	uint8_t buff[265];
	int buffLen = mavlink_msg_to_send_buffer(buff, msg);
	int bytesSent = serial.SendData((char*)buff, buffLen);
	sendMutex.unlock();

	return bytesSent > 0;
}

bool PixhawkAP::isInitialized() {
	if (rcvThread == nullptr) {
		cout << "ERROR: receive thread is NULL" << endl;
		return false;
	}
	if (!serial.IsOpened()) {
		cout << "ERROR: serial connection not open" << endl;
		return false;
	}
	return true;
}

//==============================================================================
// Thread that continuously receives messages from the PX4
//==============================================================================
class ReceiveThread : public Basic::ThreadSingleTask {
	DECLARE_SUBCLASS(ReceiveThread, Basic::ThreadSingleTask)
public:
	ReceiveThread(Basic::Component* const parent, const LCreal priority);

private:
	virtual unsigned long userFunc();
};

IMPLEMENT_SUBCLASS(ReceiveThread, "ReceiveThread")
EMPTY_SLOTTABLE(ReceiveThread)
EMPTY_COPYDATA(ReceiveThread)
EMPTY_DELETEDATA(ReceiveThread)
EMPTY_SERIALIZER(ReceiveThread)

ReceiveThread::ReceiveThread(Basic::Component* const parent, const LCreal priority)
: Basic::ThreadSingleTask(parent, priority)
{
	STANDARD_CONSTRUCTOR()
}

unsigned long ReceiveThread::userFunc()
{
	PixhawkAP* parent = dynamic_cast<PixhawkAP*>(getParent());
	if (parent != nullptr) {
		mavlink_message_t sndMsg;
		mavlink_message_t rcvMsg;
		mavlink_status_t mavStatus;
		uint8_t base_mode;
		bool pixhawkArmed = false;
		int bufferSize = 64;
		int bytesRead = 0;
		char* lpBuffer = new char[bufferSize];
		char text[MAVLINK_MSG_ID_STATUSTEXT_LEN + 1];

		// continuously receive serial data
		while (parent->isReceiving()) {
			if (parent->isSerialOpen()) {
				if (parent->getSerialDataWaiting() > bufferSize) { // wait until we have enough data to fill buffer
					parent->setSerialReadData(lpBuffer, bufferSize); // refill buffer
					while (bytesRead < bufferSize) {
						if (mavlink_parse_char(1, lpBuffer[bytesRead++], &rcvMsg, &mavStatus)) { // build msg
							// Process mavlink messages
							switch (rcvMsg.msgid) {
							case MAVLINK_MSG_ID_HEARTBEAT:
								//cout << "PX4 Heartbeat = type: " << (int)mavlink_msg_heartbeat_get_type(&rcvMsg) <<
								//	" | autopilot: " << (int)mavlink_msg_heartbeat_get_autopilot(&rcvMsg) <<
								//	" | base_mode: " << (int)mavlink_msg_heartbeat_get_base_mode(&rcvMsg) <<
								//	" | custom_mode: " << (int)mavlink_msg_heartbeat_get_custom_mode(&rcvMsg) <<
								//	" | system_status: " << (int)mavlink_msg_heartbeat_get_system_status(&rcvMsg) <<
								//	" | mavlink_version: " << (int)mavlink_msg_heartbeat_get_mavlink_version(&rcvMsg) << endl;
								parent->setHbSid(rcvMsg.sysid);
								parent->setHbCid(rcvMsg.compid);
								parent->setHbType(mavlink_msg_heartbeat_get_type(&rcvMsg));
								parent->setHbAutopilot(mavlink_msg_heartbeat_get_autopilot(&rcvMsg));
								parent->setHbBaseMode(mavlink_msg_heartbeat_get_base_mode(&rcvMsg));
								parent->setHbCustomMode(mavlink_msg_heartbeat_get_custom_mode(&rcvMsg));
								parent->setHbSystemStatus(mavlink_msg_heartbeat_get_system_status(&rcvMsg));
								parent->setHbMavlinkVersion(mavlink_msg_heartbeat_get_mavlink_version(&rcvMsg));
								break;
							case MAVLINK_MSG_ID_MISSION_ITEM:
								parent->setMiLat(mavlink_msg_mission_item_get_x(&rcvMsg));
								parent->setMiLon(mavlink_msg_mission_item_get_y(&rcvMsg));
								parent->setMiAlt(mavlink_msg_mission_item_get_z(&rcvMsg));
								break;
							case MAVLINK_MSG_ID_MISSION_REQUEST:
								if (mavlink_msg_mission_request_get_seq(&rcvMsg)              == 0 &&
								    mavlink_msg_mission_request_get_target_system(&rcvMsg)    == 255 &&
									mavlink_msg_mission_request_get_target_component(&rcvMsg) == 0) {
									parent->setMsnReqRcvd(true);
								}
								break;
							case MAVLINK_MSG_ID_MISSION_COUNT:
								parent->setMcCnt(mavlink_msg_mission_count_get_count(&rcvMsg));
								break;
							case MAVLINK_MSG_ID_MISSION_ACK:
								if (mavlink_msg_mission_ack_get_type(&rcvMsg)             == 0 &&
								    mavlink_msg_mission_ack_get_target_system(&rcvMsg)    == 255 &&
									mavlink_msg_mission_ack_get_target_component(&rcvMsg) == 0) {
									parent->setMsnAckRcvd(true);
								}
								break;
							case MAVLINK_MSG_ID_HIL_CONTROLS:
								parent->setHcRollCtrl(mavlink_msg_hil_controls_get_roll_ailerons(&rcvMsg));
								parent->setHcPitchCtrl(-mavlink_msg_hil_controls_get_pitch_elevator(&rcvMsg));
								parent->setHcYawCtrl(mavlink_msg_hil_controls_get_yaw_rudder(&rcvMsg));
								parent->setHcThrottleCtrl(mavlink_msg_hil_controls_get_throttle(&rcvMsg));
								parent->setHcSysMode(mavlink_msg_hil_controls_get_mode(&rcvMsg));
								parent->setHcNavMode(mavlink_msg_hil_controls_get_nav_mode(&rcvMsg));
								break;
							case MAVLINK_MSG_ID_STATUSTEXT:
								if (strcmp(parent->getStatustexts(), "on") == 0) {
									mavlink_msg_statustext_get_text(&rcvMsg, (char*)&text);
									cout << "\nStatus Text (UAV ID = " << parent->getOwnship()->getID() << "): " << text << endl;
								}
								break;
							} // end switch
						} // end if
					} // end while true
					bytesRead = 0;
				} else {
					std::this_thread::yield(); // allows other threads to run if serial buffer is empty
				}
			} else {
				cout << "ERROR: serial port not connected, failure to connect to Pixhawk" << endl;
				break;
			}
		} // stop receiving only when program exits
		delete[] lpBuffer;
	}
	return 0;
}

bool PixhawkAP::createReceivingThread() {
	receiving = true; // allows the receiving thread to execute (controls its while loop)
	if (rcvThread == nullptr) {
		rcvThread = new ReceiveThread(this, 0.6);
		rcvThread->unref(); // thread is a safe_ptr<>

		bool ok = rcvThread->create();
		if (!ok) {
			rcvThread = nullptr;
			if (isMessageEnabled(MSG_ERROR)) {
				cerr << "PixhawkAP::createReceivingProcess(): ERROR, failed to create the receive thread!" << endl;
			}
		}
	}
	receiving = (rcvThread != nullptr);

	return receiving;
}

} // end Swarms namespace
} // end Eaagles namespace