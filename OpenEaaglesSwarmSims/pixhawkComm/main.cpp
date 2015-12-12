//#pragma once
//
//#ifndef  __Eaagles_Swarms_PixhawkAP_H__
//#define  __Eaagles_Swarms_PixhawkAP_H__
//#endif // ! __Eaagles_Swarms_PixhawkAP_H__

#include <iostream> // used for std::cout
#include <fstream>  // used to write to file
#include "conio.h"  // used for _getch();
#include <stdint.h> // used for int8_t, int16_t, etc.
#include <windows.h>
#include <thread>
#include "Serial.h"  // used for serial port communication

#include "coremag.hxx"

#include <chrono>

#include "mavlink\checksum.h"
#include "mavlink\mavlink_types.h"
#include "mavlink\common\mavlink.h"
#include "mavlink\pixhawk\mavlink.h"
//#include "mavlink\common\mavlink_msg_hil_controls.h"
//#include "mavlink\common\mavlink_msg_hil_state_quaternion.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif

typedef std::chrono::time_point<std::chrono::system_clock, std::chrono::system_clock::duration> time_pt;
typedef std::chrono::duration<std::chrono::system_clock::rep, std::chrono::system_clock::period> time_dur;

using namespace std;

bool receiving, sending, sendOnce;
HANDLE rtHandle, stHandle;
CSerial serial;
ofstream outputfile;
int packetsWritten = 0;
mavlink_message_t* message = new mavlink_message_t;
mavlink_status_t* status = new mavlink_status_t;
chrono::system_clock::time_point start_time = std::chrono::high_resolution_clock::now();
uint64_t boot_time = 0;
uint8_t mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;
int sendType = -1;
uint32_t pingSeq = 0;

uint64_t timeSinceBoot() {
	time_dur duration = std::chrono::high_resolution_clock::now() - start_time;
	return chrono::duration_cast<std::chrono::microseconds>(duration).count() + boot_time * 1000;
}

void printMessage(mavlink_message_t* msg, bool saveToFileAsWell, bool includePayload) {
	if (msg->magic == 254) {
		//cout << dec << (int)msg->magic << " " << (int)msg->len << " " << (int)msg->seq << " " << (int)msg->sysid << " " << (int)msg->compid << " " << (int)msg->msgid << " " << (int)msg->checksum;
		if (saveToFileAsWell) {
			outputfile << dec << (int)msg->magic << "\t" << (int)msg->len << "\t" << (int)msg->seq << "\t" << (int)msg->sysid << "\t" << (int)msg->compid << "\t" << (int)msg->msgid << "\t" << (int)msg->checksum;
		}
		if (includePayload) {
			int index = 0;
			for (int i = 0; i < sizeof(msg->payload64) && i < (msg->len / 8) + 1; i++) {
				uint8_t bytes[8];
				bytes[0] = (msg->payload64[i] & 0x00000000000000FF) >> 0;
				bytes[1] = (msg->payload64[i] & 0x000000000000FF00) >> 8;
				bytes[2] = (msg->payload64[i] & 0x0000000000FF0000) >> 16;
				bytes[3] = (msg->payload64[i] & 0x00000000FF000000) >> 24;
				bytes[4] = (msg->payload64[i] & 0x000000FF00000000) >> 32;
				bytes[5] = (msg->payload64[i] & 0x0000FF0000000000) >> 40;
				bytes[6] = (msg->payload64[i] & 0x00FF000000000000) >> 48;
				bytes[7] = (msg->payload64[i] & 0xFF00000000000000) >> 56;
				for (int j = 0; j < sizeof(bytes); j++) {
					if (index < msg->len) {
						if (msg->msgid == MAVLINK_MSG_ID_STATUSTEXT) {
							cout << " " << (char)bytes[j];
						}
						
						if (saveToFileAsWell) {
							outputfile << hex << "\t" << (int)bytes[j];
						}
						index++;
					}
				}
			}
		}
		if (msg->msgid == MAVLINK_MSG_ID_STATUSTEXT) {
			cout << endl;
		}
		if (saveToFileAsWell) {
			outputfile << endl;
			packetsWritten++;
		}
		
	}
}

bool openSerialPort(int portNum) {
	if (!serial.Open(portNum, 9600)) {
		cout << "Failed to open port (" << portNum << ") :(" << endl;
		return false;
	}
	return true;
}

DWORD WINAPI rcvThread(LPVOID lpParameter) {
	int bufferSize = 5;
	char* lpBuffer = new char[bufferSize];

	// continuously receive serial data
	cout << "Currently receving data from PX4..." << endl;
	while (receiving) {
		if (serial.ReadDataWaiting() > bufferSize) {
			serial.ReadData(lpBuffer, bufferSize);
			for (int i = 0; i < bufferSize; i++) {
				if (mavlink_parse_char(1, lpBuffer[i], message, status)) {
					printMessage(message, true, true);
					// TODO: look for HIL related messages and handle them appropriately
					// -- Specifically, look for HIL_CONTROLS (91) message to send to JSBSim
					if (message->msgid == 2) {
						if (boot_time == 0) {
							start_time = std::chrono::high_resolution_clock::now();
							boot_time = message->payload64[1] & 0xFFFFFFFF;
							cout << dec << "Boot time: " << boot_time << " ms" << endl;
						}
					}
				}
			}
		}
		else {
			std::this_thread::yield();
		}
	}

	delete[] lpBuffer;

	return 0;
}

DWORD WINAPI sndThread(LPVOID lpParameter) {

	mavlink_message_t* msg = new mavlink_message_t;
	float attitude_quaternion[4] = {1,0,0,0};

	while (sending) {
		if (sendOnce) {
			switch (sendType) {
			case 4:    // PING
				mavlink_msg_ping_pack(1, 1, msg, timeSinceBoot(), pingSeq++, 0, 0);
				break;
			case 11:   // SET_MODE
				mavlink_msg_set_mode_pack(1, 1, msg, 1,
					MAV_MODE_FLAG_HIL_ENABLED,
					0);
				break;
			case 12:   // SET_MODE
				mavlink_msg_set_mode_pack(1, 1, msg, 1,
					MAV_MODE_FLAG_STABILIZE_ENABLED,
					0);
				break;
			case 13:   // SET_MODE
				mavlink_msg_set_mode_pack(1, 1, msg, 1,
					MAV_MODE_FLAG_GUIDED_ENABLED,
					0);
				break;
			case 14:   // SET_MODE
				mavlink_msg_set_mode_pack(1, 1, msg, 1,
					MAV_MODE_FLAG_AUTO_ENABLED,
					0);
				break;
			case 21:   // PARAM_REQUEST_LIST
				mavlink_msg_param_request_list_pack(1, 1, msg, 1, 1);
				break;
			case 39:
				mavlink_msg_mission_item_pack(254, 1, msg, 1, 1, 0, MAV_FRAME_MISSION, 300, 1, 0, 0, 0, 0, 0, 0, 0, 0);
				break;
			case 43:
				mavlink_msg_mission_request_list_pack(254, 1, msg, 1, 1);
			case 76:
				mavlink_msg_command_long_pack(254, 1, msg, 1, 1, 22, 0, 5, 0, 0, 0, 37, 117, 500);
			case 115:  // HIL_STATE_QUATERNION
				// TODO: get sim state data from JSBSim and pack into this msg
				mavlink_msg_hil_state_quaternion_pack(1, 1, msg, timeSinceBoot(), attitude_quaternion, 0.001, 0.000, 0.000, 0x98ab02f4, 0x43e3eba4, 0x0000f42c, 0x0036, 0x0045, 0x0000, 0x03f5, 0x0390, 0x000f, 0x000c, 0x0002);
				break;
			default:
				sendType = -1;
			}

			if (sendType != -1){
				// convert msg to buffered data
				uint8_t buff[265];
				int buffLen = mavlink_msg_to_send_buffer(buff, msg);

				// send buffered data to PX4
				cout << "Sent " << serial.SendData((char*)buff, buffLen) << " bytes to PX4..." << endl;
			}
			// reset sending parameters
			sendOnce = false;
			sendType = -1;
		}
	}
	// MAV_MODE_FLAG_DECODE_POSITION_HIL;
	// MAV_MODE_FLAG_HIL_ENABLED;

	delete[] msg;

	return 0;
}

void startSndRcvThreads() {
	LPTHREAD_START_ROUTINE s = rcvThread;
	if (!receiving) {
		receiving = true;
		rtHandle = CreateThread(0, 0, rcvThread, 0, 0, 0);
	}
	if (!sending) {
		sending = true;
		stHandle = CreateThread(0, 0, sndThread, 0, 0, 0);
	}
}

void stopSndRcvThreads() {
	if (receiving) {
		receiving = false;
		CloseHandle(rtHandle);
	}
	if (sending) {
		sending = false;
		CloseHandle(stHandle);
	}
}

void msgTesting() {
	// open output file
	outputfile.open("pixhawkCommOutput.txt", ios::app);

	// create a generic mavlink message
	mavlink_message_t* msg = new mavlink_message_t;

	/****************************************/
	/*   0 | HEARTBEAT
	/****************************************/
	//mavlink_msg_heartbeat_pack(1, 1, msg, 11, 12, 13, 10, 14);

	/****************************************/
	/*  11 | SET_MODE
	/****************************************/
	//uint8_t mode = MAV_MODE_FLAG_HIL_ENABLED &
	//			   MAV_MODE_FLAG_STABILIZE_ENABLED &
	//			   MAV_MODE_FLAG_GUIDED_ENABLED &
	//			   MAV_MODE_FLAG_AUTO_ENABLED;
	//mavlink_msg_set_mode_pack(1, 1, msg, 1, mode, 0);

	/****************************************/
	/*  21 | PARAM_REQUEST_LIST
	/****************************************/
	//mavlink_msg_param_request_list_pack(1, 1, msg, 10, 11);

	/****************************************/
	/*  22 | PARAM_VALUE
	/****************************************/
	//mavlink_msg_param_value_pack(1,1,msg,"abcdefghijklmnop",2,3,4,5);

	/****************************************/
	/*  30 | ATTITUDE
	/****************************************/
	//mavlink_msg_attitude_pack(1, 1, msg, 1, 2, 3, 4, 5, 6, 7);

	/****************************************/
	/*  32 | LOCAL_POSITION_NED
	/****************************************/
	//mavlink_msg_local_position_ned_pack(1, 1, msg, 1, 2, 3, 4, 5, 6, 7);

	/****************************************/
	/*  74 | VFR_HUD
	/****************************************/
	//mavlink_msg_vfr_hud_pack(1, 1, msg, 1, 2, 3, 4, 5, 6);

	/****************************************/
	/*  76 | COMMAND_LONG
	/****************************************/
	//mavlink_msg_command_long_pack(1, 1, msg, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11);

	/****************************************/
	/* 105 | HIGHRES_IMU
	/****************************************/
	//mavlink_msg_highres_imu_pack(1, 1, msg, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);

	/****************************************/
	/* 115 | HIL_STATE_QUATERNION
	/****************************************/
	//float attitude_quaternion[4] = { 1, 0, 0, 0 };
	//mavlink_msg_hil_state_quaternion_pack(1, 1, msg, 0, attitude_quaternion, 0x00000001, 0x00000002, 0x00000003, 0x00000004, 0x00000005, 0x00000006, 0x0007, 0x0008, 0x0009, 0x000a, 0x000b, 0x000c, 0x000d, 0x000e);
	////                                                                       0x3f800000, 0x40000000, 0x40400000
	
	/****************************************/
	/*  39 | MISSION_ITEM
	/****************************************/
	mavlink_msg_mission_item_pack(254, 1, msg, 13, 14, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,12);


	// convert message to bytes and print to console/file
	uint8_t buff[265];
	int buffLen = mavlink_msg_to_send_buffer(buff, msg);
	for (int i = 0; i < buffLen; i++) {
		if (mavlink_parse_char(1, buff[i], message, status)) {
			printMessage(message, true, true);
		}
	}
	delete[] msg;

	outputfile.close();
}

void startCommunications() {
	int portNum = 5;
	int inputIndex = 0;
	char inputs[3];

	if (openSerialPort(portNum)) {
		char input = ' ';
		cout << "Port (" << portNum << ") open!" << endl;
		outputfile.open("pixhawkCommOutput.txt", ios::app);
		cout << "File open for saving packets..." << endl;
		startSndRcvThreads();
		while (input != 'q') { // quit
			input = _getch();
			if (input == 'c') {
				cout << dec << "Packets written: " << packetsWritten << endl;
			}
			else if (input == 't') {
				cout << dec << "Current time: " << timeSinceBoot() << " us" << endl;
			}
			else if (input == 's') {
				inputs[0] = _getch();
				inputs[1] = _getch();
				inputs[2] = _getch();
				sendType = atoi(inputs);
				if (sendType > 0 && sendType < 256) {
					sendOnce = true;
				}
			} // end of if (input == 's')
		} // end of while (input != 'q')

		stopSndRcvThreads();
		serial.Close();
		outputfile.close();
	}
	else {
		_getch();
	}
}

void processFiles() {

	uint16_t b;
	mavlink_message_t rcvMsg;
	mavlink_status_t stat;

	ofstream output1;
	output1.open("C:\\Users\\Derek\\Desktop\\snd_out.txt", ios::app);
	ifstream input1("C:\\Users\\Derek\\Desktop\\snd.txt");
	while (input1 >> hex >> b) {
		if (mavlink_parse_char(1, (char)b, &rcvMsg, &stat)) {
			cout << ".";
			// ===== SAVE TO FILE ===========================
			output1 << dec << (int)rcvMsg.magic << "\t" << (int)rcvMsg.len << "\t" << (int)rcvMsg.seq << "\t" << (int)rcvMsg.sysid << "\t" << (int)rcvMsg.compid << "\t" << (int)rcvMsg.msgid << "\t" << (int)rcvMsg.checksum;
			uint8_t* payload = (uint8_t *)&(rcvMsg.payload64[0]);
			for (int i = 0; i < rcvMsg.len; i++) {
				output1 << hex << "\t" << (int)payload[i];
			}
			output1 << endl;
			// ===== END SAVE TO FILE =======================
		}
	}
	output1.close();
	input1.close();



	ofstream output2;
	output2.open("C:\\Users\\Derek\\Desktop\\rcv_out.txt", ios::app);
	ifstream input2("C:\\Users\\Derek\\Desktop\\rcv.txt");
	while (input2 >> hex >> b) {
		if (mavlink_parse_char(1, (char)b, &rcvMsg, &stat)) {
			cout << "-";
			// ===== SAVE TO FILE ===========================
			output2 << dec << (int)rcvMsg.magic << "\t" << (int)rcvMsg.len << "\t" << (int)rcvMsg.seq << "\t" << (int)rcvMsg.sysid << "\t" << (int)rcvMsg.compid << "\t" << (int)rcvMsg.msgid << "\t" << (int)rcvMsg.checksum;
			uint8_t* payload = (uint8_t *)&(rcvMsg.payload64[0]);
			for (int i = 0; i < rcvMsg.len; i++) {
				output2 << hex << "\t" << (int)payload[i];
			}
			output2 << endl;
			// ===== END SAVE TO FILE =======================
		}
	}
	output2.close();
	input2.close();
}

void testMagXYZ() {
	double field[6];
	double usedLat, usedLon, usedAlt;
	usedLat = 37.621313 * PI / 180; // radians
	usedLon = 302.378955 * PI / 180; // radians
	usedAlt = 0.815068;             // km
	cout << usedLat << " " << usedLon << " " << usedAlt << " " << yymmdd_to_julian_days(15, 10, 7) << endl;
	cout << calc_magvar(
		usedLat,
		usedLon,
		usedAlt,
		yymmdd_to_julian_days(2015, 9, 30),
		field) << endl;
	cout << "B_r   : " << field[0] << endl;
	cout << "B_th  : " << field[1] << endl;
	cout << "B_phi : " << field[2] << endl;
	cout << "B_x   : " << field[3] << endl;
	cout << "B_y   : " << field[4] << endl;
	cout << "B_z   : " << field[5] << endl;
	double m, x, y, z;
	x = field[3];
	y = field[4];
	z = field[5];
	m = sqrt(x*x + y*y + z*z);
	x = x / m;
	y = y / m;
	z = z / m;
	cout << x << " " << y << " " << z << endl;


	_getch();
}

void main(int argc, char* argv[]) {
	//startCommunications();
	//msgTesting();
	processFiles();
	//testMagXYZ();

	delete message;
	delete status;
}