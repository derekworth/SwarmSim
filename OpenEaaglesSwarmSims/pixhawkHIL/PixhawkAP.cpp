#include "PixhawkAP.h"
#include "UAV.h"
#include "SimOCA.h"

#include "openeaagles/basic/Number.h"
#include "openeaagles/basic/Thread.h"
#include "openeaagles/simulation/Player.h"
#include "openeaagles/simulation/Navigation.h"
#include "openeaagles/simulation/Steerpoint.h"
#include "openeaagles/simulation/Route.h"
#include "openeaagles/dynamics/JSBSimModel.h"
#include "JSBSim/FGFDMExec.h"
#include "JSBSim/models/FGFCS.h"
#include "JSBSim/models/FGAerodynamics.h"

// used for testing
#include <iostream>
#include <iomanip>
#include <thread>

namespace Eaagles {
namespace Swarms {

// =============================================================================
// class: PixhawkAP
// =============================================================================

IMPLEMENT_SUBCLASS(PixhawkAP,"PixhawkAP")

BEGIN_SLOTTABLE(PixhawkAP)
	"portNum",
	"mode",
	"textMsging"
END_SLOTTABLE(PixhawkAP)

BEGIN_SLOT_MAP(PixhawkAP)
	ON_SLOT(1, setSlotPortNum,    Basic::Number) // 1) USB COM port assigned to specified PX4
	ON_SLOT(2, setSlotMode,       Basic::String) // 2) modes: "nav" and "swarm" (autonomous flight)
	                                             //    default = "nav"
	ON_SLOT(3, setSlotTextMsging, Basic::String) // 3) Text messaging refers to printing to console the contents
	                                             //    of mavlink STATUSTEXT (#253) message received from PX4
												 //    default = "off"
END_SLOT_MAP()                                                                        

Eaagles::Basic::Object* PixhawkAP::getSlotByIndex(const int si)
{                                                                                     
	return BaseClass::getSlotByIndex(si);
}

EMPTY_SERIALIZER(PixhawkAP)

//------------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------------
PixhawkAP::PixhawkAP() 
 : rcvThread(nullptr)
{
	STANDARD_CONSTRUCTOR()

	portNum = 0;
	receiving = false;

	mode = nullptr;
	setMode(new Basic::String("nav"));

	textMsging = nullptr;
	setTextMsging(new Basic::String("off"));

	outputfile.open("pixhawkCommOutput.txt", ios::app); // DELETE ME... USED FOR TESTING ONLY!!!
}

//------------------------------------------------------------------------------------
// copyData() - copies one object to another
//------------------------------------------------------------------------------------
void PixhawkAP::copyData(const PixhawkAP& org, const bool cc) 
{
	BaseClass::copyData(org);

	if(cc) {
		mode = nullptr;
	}
	
	portNum = org.portNum;
	receiving = false;

	Basic::String* m = nullptr;
	if (org.mode != nullptr) m = org.mode->clone();
	mode = m;
	if(m != 0) m->unref();

	// We need to init this ourselves, so ...
	if (rcvThread != nullptr) {
		rcvThread->terminate();
		rcvThread = nullptr;
	}
}

//------------------------------------------------------------------------------------
// deleteData() -- delete this object's data
//------------------------------------------------------------------------------------
void PixhawkAP::deleteData()
{
	receiving = false;
	serial.Close();
	mode = nullptr;
	textMsging = nullptr;
	if (rcvThread != nullptr) {
		rcvThread->terminate();
		rcvThread = nullptr;
	}
	outputfile.close();        // DELETE ME... USED FOR TESTING ONLY!!!
}

//------------------------------------------------------------------------------------
// Rotation - rotate a coordinate around the origin by roll(phi), pitch(theta), and yaw(psi) angles
//------------------------------------------------------------------------------------

double* rollPitchYaw(double x, double y, double z, bool inDegrees, bool reverse, double phi, double theta, double psi) {
	// convert degrees to radians
	if (inDegrees) {
		double PI = 3.14159265359;
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

//------------------------------------------------------------------------------------
// Setter methods
//------------------------------------------------------------------------------------

bool PixhawkAP::setPortNum(const int p)
{
	portNum = p;
	return true;
}

bool PixhawkAP::setMode(const Basic::String* const m)
{
	mode = m;
	return true;
}

bool PixhawkAP::setTextMsging(const Basic::String* const m)
{
	textMsging = m;
	return true;
}

//------------------------------------------------------------------------------------
// Slot methods
//------------------------------------------------------------------------------------

bool PixhawkAP::setSlotPortNum(const Basic::Number* const msg)
{
	bool ok = (msg != nullptr);
	if (ok) ok = setPortNum(msg->getInt());
	// establish connection to Pixhawk over given port and startup receiver thread
	if (!isInitialized()) {
		connectToPixhawk();										// open serial port to PX4
		createReceivingThread();								// start listening for messages from PX4
		start_time = std::chrono::high_resolution_clock::now();
	}
	return ok;
}

bool PixhawkAP::setSlotMode(const Basic::String* const msg)
{
	bool ok = false;
	if (msg != nullptr) {
		if (strcmp(msg->getString(), "nav") == 0) { setMode(msg); }
		else setMode(new Basic::String("swarm"));
		ok = true;
	}
	return ok;
}

bool PixhawkAP::setSlotTextMsging(const Basic::String* const msg)
{
	bool ok = false;
	if (msg != nullptr) {
		if (strcmp(msg->getString(), "on") == 0) { setTextMsging(msg); }
		else setTextMsging(new Basic::String("off"));
		ok = true;
	}
	return ok;
}

//------------------------------------------------------------------------------
// Data access function
//------------------------------------------------------------------------------

const char* PixhawkAP::getMode() const
{
	const char* p = 0;
	if (mode != 0) p = *mode;
	return p;
}

const char* PixhawkAP::getTextMsging() const
{
	const char* p = 0;
	if (textMsging != 0) p = *textMsging;
	return p;
}

//------------------------------------------------------------------------------
// Tracks how long this program has been running
//------------------------------------------------------------------------------

uint64_t PixhawkAP::sinceSystemBoot() const
{
	return chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
}

//------------------------------------------------------------------------------
// Send UAV autopilot control (i.e. stick/rudder/throttle) to JSBSim
//------------------------------------------------------------------------------

void PixhawkAP::flyUav() {
	if (mode == 0) return; // allow manual flight

	Swarms::UAV* uav = dynamic_cast<Swarms::UAV*>(this->getOwnship()); // get UAV
	if (uav == 0) return;

	Eaagles::Dynamics::JSBSimModel* dm = dynamic_cast<Eaagles::Dynamics::JSBSimModel*>(uav->getDynamicsModel()); // get UAV's dynamics model
	if (dm == 0) return;

	// provide control inputs
	dm->setControlStickPitchInput(elevator);
	dm->setControlStickRollInput(aileron);
	dm->setRudderPedalInput(rudder);
	dm->setThrottles(&throttle, 1);
}

//------------------------------------------------------------------------------
// Send UAV simulation state (i.e. UAV position/attitude/velocity/etc.) to PX4
//------------------------------------------------------------------------------
// MAVLink HIL requires the following to be satisfied
//   1) enable HIL
//   2) diable safety
//   3) arm system
//   4) supply valid measurement data

void PixhawkAP::sendSimStateToPX4() {
	// Send a sim state message during each non-time critical update (i.e. 50 Hz)
	UAV* uav = dynamic_cast<UAV*>(getOwnship());
	if (uav != nullptr) {
		Eaagles::Dynamics::JSBSimModel* dm = dynamic_cast<Eaagles::Dynamics::JSBSimModel*>(uav->getDynamicsModel()); // get UAV's dynamics model
		if (dm == nullptr) return;

		//-----------------------------------------
		// HIL_SENSOR (107)
		//-----------------------------------------
		float xacc = uav->getAcceleration().x();
		float yacc = uav->getAcceleration().y();
		float zacc = uav->getAcceleration().z() - 9.68235;
		//cout << "\racc:" << xacc << "\t" << yacc << "\t" << zacc << "                          ";

		float xgyro = uav->getAngularVelocities().x();
		float ygyro = uav->getAngularVelocities().y();
		float zgyro = uav->getAngularVelocities().z();
		//cout << "\rgyro:" << xgyro << "\t" << ygyro << "\t" << zgyro << "                          ";

		double phi   = uav->getRollR();
		double theta = uav->getPitchR();
		double psi   = uav->getHeadingR();

		// Static north pointing unit vector of magnetic field in the Inertial (NED) Frame
		// over San Francisco International Airport on 11/17/2015 (declination/inclination of 13.6627/61.0930 degrees respectively)
		float xmag = 0.850634213;
		float ymag = 0.206775684;
		float zmag = 0.483389338;
		// Rotate vector
		double* reverseRotation = rollPitchYaw(xmag, ymag, zmag, false, true, phi, theta, psi);
		xmag = reverseRotation[0];
		ymag = reverseRotation[1];
		zmag = reverseRotation[2];
		//cout << "\rmag:" << xmag << "\t" << ymag << "\t" << zmag << "                          ";
		
		float pressure_alt = dm->getPressureAltitude() / 3.2808399;
		float abs_pressure = -0.1092*pressure_alt+1010.7;
		float diff_pressure = dm->getDifferentialPressure(); //uav->getCalibratedAirspeed();//(uav->getCalibratedAirspeed()*uav->getCalibratedAirspeed() * 1.225f) / 2.0f;
		//cout << "pressure_alt:" << pressure_alt << endl;
		//cout << "abs_pressure:" << abs_pressure << endl;
		//cout << "diff_pressure:" << diff_pressure << endl;

		float temperature = dm->getTemperature();

		uint32_t fields_updated = 0xffff; // assume all fields always updated

		mavlink_message_t msg;

		//cout << 
		//	xacc		   << " " << 
		//	yacc		   << " " << 
		//	zacc		   << " " << 
		//	xgyro		   << " " << 
		//	ygyro		   << " " << 
		//	zgyro		   << " " << 
		//	xmag		   << " " << 
		//	ymag		   << " " << 
		//	zmag		   << " " << 
		//	abs_pressure   << " " << 
		//	diff_pressure  << " " << 
		//	pressure_alt   << " " << 
		//	temperature	   << " " << 
		//	fields_updated << " " << endl;

		// send HIL_SENSOR and HIL_GPS messages
		mavlink_msg_hil_sensor_pack(sid, cid, &msg, sinceSystemBoot(),
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
			fields_updated);

		sendMessage(&msg);

		//-----------------------------------------
		// HIL_GPS (113)
		//-----------------------------------------
		uint8_t  fix_type           = 3;
		double dlat, dlon, dalt;
		uav->getPositionLLA(&dlat, &dlon, &dalt);
		int32_t  lat                =  (int32_t)(dlat*10000000);
		int32_t  lon                =  (int32_t)(dlon*10000000);
		int32_t  alt                = -(int32_t)(dalt*1000);
		uint16_t eph                = 30;
		uint16_t epv                = 60; 
		uint16_t vel                = uav->getGroundSpeed();
		int16_t  vn                 = uav->getVelocity().x()*100;
		int16_t  ve                 = uav->getVelocity().y()*100;
		int16_t  vd                 = uav->getVelocity().z()*100;
		uint16_t cog                = uav->getGroundTrackD()*100;
		uint8_t  satellites_visible = 8;
		
		mavlink_msg_hil_gps_pack(sid, cid, &msg, sinceSystemBoot(),
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

		sendMessage(&msg);

		if (0)
		// MISSION UPDATE state machine
		switch (missionState) {
		case SEND_COUNT:
			ackRcvd = reqRcvd = false;
			mavlink_msg_mission_count_pack(254, 1, &msg, 1, 1, 1);
			sendMessage(&msg);
			cout << "Mission Count message sent" << endl;
			// start timer
			timeout_start = std::chrono::high_resolution_clock::now();
			missionState = AWAIT_REQ;
			break;
		case AWAIT_REQ:
			// check timer (checked by the receiving thread)
			if (reqRcvd) {
				cout << "Mission Request received" << endl;
				missionState = SEND_ITEM;
			} else if (chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - timeout_start).count() > 1000000) { // wait 1s before trying again
				cout << "Timeout: no Mission Request received" << endl;
				missionState = SEND_COUNT; // try again
			}
			break;
		case SEND_ITEM:
			mavlink_msg_mission_item_pack(254, 1, &msg, 1, 1, 0, 0, 16, 2, 0, 0, 2, 0, 0, 37.6311589, -122.3646069, 2500);
			sendMessage(&msg);
			cout << "Mission Item message sent" << endl;
			// start timer
			timeout_start = std::chrono::high_resolution_clock::now();
			missionState = AWAIT_ACK;
			break;
		case AWAIT_ACK:
			// check timer (checked by the receiving thread)
			if (ackRcvd) {
				cout << "Mission Ack received" << endl;
				missionState = START_NAV;
			} else if (chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - timeout_start).count() > 1000000) { // wait 1s before trying again
				cout << "Timeout(" << itemSendCount << "): no Mission Ack received" << endl;
				if (++itemSendCount < 10) {
					missionState = SEND_ITEM; // try again
				} else {
					missionState = SEND_COUNT; // start over from beginning
				}
			}
			break;
		case START_NAV:
			//mavlink_msg_command_long_pack(254, 1, &msg, 1, 1, 300, 1, 0, 0, 0, 0, 0, 0, 0);
			//mavlink_msg_set_mode_pack(254, 1, &msg, 1, 220, 0);
			//sendMessage(&msg);
			//cout << "Mission underway..." << endl;
			//missionState = END_MISSION;
			break;
		case END_MISSION:
			break;
		}

		// set current mission item to the updated one
		//mavlink_msg_mission_set_current_pack(254, 1, &msg, 1, 1, 0);
		//sendMessage(&msg);

		// get list
		//mavlink_msg_mission_request_list_pack(254, 1, &msg, 1, 1);
		//sendMessage(&msg);
	}
}

//------------------------------------------------------------------------------
// Simulation driven update method (non-time critical, ~50Hz)
//------------------------------------------------------------------------------

void PixhawkAP::updateData(const LCreal dt)
{
	flyUav();            // push control to JSBSim
	sendSimStateToPX4(); // push UAV attitude/position/etc to PX4
	BaseClass::updateData(dt);
}

//------------------------------------------------------------------------------
// Establish serial (USB) communications with the Pixhawk PX4
//------------------------------------------------------------------------------

bool PixhawkAP::connectToPixhawk()
{
	if (!serial.Open(portNum, 9600)) {
		cout << "Failed to open port (" << portNum << ") :(" << endl;
		return false;
	}
	return true;
}

bool PixhawkAP::sendMessage(mavlink_message_t* msg)
{
	sendMutex.lock();
	uint8_t buff[265];
	int buffLen = mavlink_msg_to_send_buffer(buff, msg);
	int bytesSent = serial.SendData((char*)buff, buffLen);
	sendMutex.unlock();

	return bytesSent > 0;
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
		int bufferSize = 50;
		char* lpBuffer = new char[bufferSize];
		char text[MAVLINK_MSG_ID_STATUSTEXT_LEN + 1];

		// continuously receive serial data
		cout << "Currently receving data from PX4..." << endl;
		while (parent->isReceiving()) {
			if (parent->serial.IsOpened()) {
				if (parent->serial.ReadDataWaiting() > bufferSize) {
					parent->serial.ReadData(lpBuffer, bufferSize);
					for (int i = 0; i < bufferSize; i++) {
						//--------------------------------------
						// Builds messages w/ rcv'd bytes
						//--------------------------------------
						if (mavlink_parse_char(1, lpBuffer[i], &rcvMsg, &mavStatus)) {

							// ===== SAVE TO FILE ===========================
							//parent->outputfile << dec << (int)rcvMsg.magic << "\t" << (int)rcvMsg.len << "\t" << (int)rcvMsg.seq << "\t" << (int)rcvMsg.sysid << "\t" << (int)rcvMsg.compid << "\t" << (int)rcvMsg.msgid << "\t" << (int)rcvMsg.checksum;
							//uint8_t* payload = (uint8_t *)&(rcvMsg.payload64[0]);
							//for (int i = 0; i < rcvMsg.len; i++) {
							//	parent->outputfile << hex << "\t" << (int)payload[i];
							//}
							//parent->outputfile << endl;
							// ===== END SAVE TO FILE =======================
							//cout << ".";

							switch (rcvMsg.msgid) {
							case MAVLINK_MSG_ID_HEARTBEAT:
								base_mode = mavlink_msg_heartbeat_get_base_mode(&rcvMsg);
								if (!(base_mode & MAV_MODE_FLAG_HIL_ENABLED)) { // initialize HIL in not already done so
									cout << "Attempting to establish HIL, sending SET_MODE message w/ base_mode = MAV_MODE_FLAG_HIL_ENABLED." << endl;
									//mavlink_msg_set_mode_pack(1, 1, &sndMsg, 1, MAV_MODE_FLAG_HIL_ENABLED, 0);
									mavlink_msg_set_mode_pack(1, 1, &sndMsg, 1, 189, 0);
									parent->sendMessage(&sndMsg);
								}
								//cout << "\n===================================================================" << (int)base_mode << endl;
								break;
							case MAVLINK_MSG_ID_HIL_CONTROLS:
								pixhawkArmed = true;
								mavlink_hil_controls_t h;
								mavlink_msg_hil_controls_decode(&rcvMsg, &h);
								if (h.pitch_elevator != 0 || h.roll_ailerons != 0 || h.yaw_rudder != 0 || h.throttle != 0) {
									//cout << "\r" << h.throttle << "\t" << h.pitch_elevator << "\t" << h.roll_ailerons << "\t" << h.yaw_rudder; // << "\t" << h.aux1 << "\t" << h.aux2 << "\t" << h.aux3 << "\t" << h.aux4 << "\t" << h.throttle << "\t" << h.time_usec << endl;
									//parent->outputfile << (int)h.mode << "\t" << (int)h.nav_mode << "\t" << h.pitch_elevator << "\t" << h.roll_ailerons << "\t" << h.yaw_rudder << "\t" << h.aux1 << "\t" << h.aux2 << "\t" << h.aux3 << "\t" << h.aux4 << "\t" << h.throttle << "\t" << h.time_usec << endl;
								}
								parent->setElevator(h.pitch_elevator);
								parent->setAileron(h.roll_ailerons);
								parent->setRudder(h.yaw_rudder);
								parent->setThrottle(h.throttle);
								break;
							case MAVLINK_MSG_ID_STATUSTEXT:
								mavlink_msg_statustext_get_text(&rcvMsg, (char*)&text);
								cout << "\nStatus Text: " << text << endl;
								break;
							//case MAVLINK_MSG_ID_MISSION_REQUEST:
							//	if (!parent->reqRcvd && mavlink_msg_mission_request_get_seq(&rcvMsg) == 0) {
							//		parent->reqRcvd = true;
							//	}
							//	break;
							case MAVLINK_MSG_ID_MISSION_ACK:
								if (!parent->ackRcvd) {
									if (mavlink_msg_mission_ack_get_type(&rcvMsg) == 0) {
										cout << "Mission accepted OK!" << endl;
										parent->ackRcvd = true;
									}
									else {
										cout << "Mission error: " << mavlink_msg_mission_ack_get_type(&rcvMsg) << endl;
									}
								}
								break;
							default:
								if (!pixhawkArmed) { // arm system
									cout << "Attempting to arm the system, sending COMMAND_LONG message." << endl;
									mavlink_msg_command_long_pack(254, 1, &sndMsg, 1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0);
									parent->sendMessage(&sndMsg);
								}
							} // end switch
						} // end if
					} // end for
				} // end if
				else {
					std::this_thread::yield(); // allows other threads to run if serial buffer is empty
				} // end else
			} // end if
			else {
				cout << "ERROR: serial port not connected, failure to connect to Pixhawk" << endl;
				break;
			} // end else
		} // stop receiving only when program exits

		delete[] lpBuffer;
	}
	return 0;
}

//------------------------------------------------------------------------------
// Create threat that asyncronously receives data from PX4
//------------------------------------------------------------------------------
bool PixhawkAP::createReceivingThread()
{
	receiving = true;
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

//------------------------------------------------------------------------------
// True if receiving thread is running and serial port to PX4 is open
//------------------------------------------------------------------------------
bool PixhawkAP::isInitialized()
{
	return (rcvThread != nullptr) && serial.IsOpened();
}

} // end Swarms namespace
} // end Eaagles namespace