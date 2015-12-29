//#pragma once
//
//#ifndef  __Eaagles_Swarms_PixhawkAP_H__
//#define  __Eaagles_Swarms_PixhawkAP_H__
//#endif // ! __Eaagles_Swarms_PixhawkAP_H__

#define PI 3.1415926535897932384626433832795

#include <iostream> // used for std::cout
#include <fstream>  // used to write to file
#include <string>
#include "conio.h"  // used for _getch();
#include <stdint.h> // used for int8_t, int16_t, etc.
#include <windows.h>
#include "Serial.h"  // used for serial port communication

#include "mavlink\mavlink_types.h"
#include "mavlink\common\mavlink.h"
#include "mavlink\pixhawk\mavlink.h"

using namespace std;

uint8_t isAsciiHex(char letter) {
	if ((letter >= '0' && letter <= '9') ||
		(letter >= 'a' && letter <= 'f') ||
		(letter >= 'A' && letter <= 'F')) {
		return true;
	} else {
		return false;
	}
}

uint8_t convertAsciiToHex(char letter) {
	uint8_t hex = 0;
	if (letter >= '0' && letter <= '9') {
		hex = letter - 48;
	} else if (letter >= 'a' && letter <= 'f') {
		hex = letter - 87;
	} else if (letter >= 'A' && letter <= 'F') {
		hex = letter - 55;
	} else {
		return 0;
	}
	return hex;
}

void preprocessFile(char* filename) {
	char str[160];
	strcpy(str, "C:\\Users\\Derek\\Desktop\\HIL Experimentation\\serialcaptures\\");
	strcat(str, filename);

	string line;
	bool success = true;

	// "read from" file
	ifstream input(str);

	// "append to" file
	ofstream output;
	output.open("C:\\Users\\Derek\\Desktop\\HIL Experimentation\\serialcaptures\\hex_output.txt", ios::trunc);

	if (output.is_open()) {
		if (input.is_open()) {
			while (getline(input, line)) {
				if (line.find("    ") == 0)
					output << line.substr(4, 47) << "\n";
				else if (line.find("Read data") < 100)
					output << "R\n";
				else if (line.find("Written data") < 100)
					output << "W\n";
			}
			input.close();
		} else {
			std::cout << "Unable to open input file.";
			success = false;
		}
		output.close();
	} else {
		std::cout << "Unable to open output files.";
		success = false;
	}
	if (success) std::cout << "Preprocessing complete." << endl;
}

void processFile() {
	char b;
	//cout.setf(ios::hex, ios::basefield);

	mavlink_message_t* msg;
	mavlink_message_t rMsg;
	mavlink_message_t wMsg;
	mavlink_status_t rStat;
	mavlink_status_t wStat;

	char param_id[17];
	float controls[8];
	uint8_t fcv[8], mcv[8], ocv[8];
	char text[51];

	// "read from" file
	ifstream input("C:\\Users\\Derek\\Desktop\\HIL Experimentation\\serialcaptures\\hex_output.txt");

	// "append to" files
	ofstream output;
	output.open("C:\\Users\\Derek\\Desktop\\HIL Experimentation\\serialcaptures\\mavlink_output.txt", ios::trunc);

	bool isWriting = false;
	bool isReading = false;
	bool msgComplete = false;

	// Process Send data
	int i = 0;
	bool first = true;
	uint8_t hexValue = 0;
	bool hexValid = false;
	//while (input >> hex >> b) {

	while (input.get(b)) {

		if (isAsciiHex(b)) {
			if (first) {
				hexValue = convertAsciiToHex(b) << 4;
				first = false;
				hexValid = false;
			} else {
				hexValue = convertAsciiToHex(b) | hexValue;
				first = true;
				hexValid = true;
			}
		} else {
			hexValue = 0;
			first = true;
			hexValid = false;
			if (b == 'W') {
				isWriting = true;
				isReading = false;
			} else if (b == 'R') {
				isWriting = false;
				isReading = true;
			}
		}

		if (hexValid && isWriting && !isReading) {
			msgComplete = mavlink_parse_char(1, hexValue, &wMsg, &wStat);
			msg = &wMsg;
		} else if (hexValid && isReading && !isWriting) {
			msgComplete = mavlink_parse_char(2, hexValue, &rMsg, &rStat);
			msg = &rMsg;
		}

		if (msgComplete) {
			msgComplete = false;
			std::cout << ".";
			// ===== SAVE TO FILE ===========================
			output << dec << (int)msg->magic << "\t" << (int)msg->len << "\t" << (int)msg->seq << "\t" << (int)msg->sysid << "\t" << (int)msg->compid << "\t" << (int)msg->msgid << "\t" << (int)msg->checksum << "\t";

			switch (msg->msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT:
				output << "type: " << (int)mavlink_msg_heartbeat_get_type(msg) <<
					" | autopilot: " << (int)mavlink_msg_heartbeat_get_autopilot(msg) <<
					" | base_mode: " << (int)mavlink_msg_heartbeat_get_base_mode(msg) <<
					" | custom_mode: " << (int)mavlink_msg_heartbeat_get_custom_mode(msg) <<
					" | system_status: " << (int)mavlink_msg_heartbeat_get_system_status(msg) <<
					" | mavlink_version: " << (int)mavlink_msg_heartbeat_get_mavlink_version(msg);
				break;
			case MAVLINK_MSG_ID_SET_MODE:
				output << "target_system: " << (int)mavlink_msg_set_mode_get_target_system(msg) <<
					" | base_mode: " << (int)mavlink_msg_set_mode_get_base_mode(msg) <<
					" | custom_mode: " << (int)mavlink_msg_set_mode_get_custom_mode(msg);
				break;
			case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
				mavlink_msg_param_request_read_get_param_id(msg, param_id);
				output << "target_system: " << (int)mavlink_msg_param_request_read_get_target_system(msg) <<
					" | target_component: " << (int)mavlink_msg_param_request_read_get_target_component(msg) <<
					" | param_id: " << param_id <<
					" | param_index: " << (int)mavlink_msg_param_request_read_get_param_index(msg);
				break;
			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
				output << "target_system: " << (int)mavlink_msg_param_request_list_get_target_system(msg) <<
					" | target_component: " << (int)mavlink_msg_param_request_list_get_target_component(msg);
				break;
			case MAVLINK_MSG_ID_PARAM_VALUE:
				mavlink_msg_param_value_get_param_id(msg, param_id);
				param_id[16] = '\0';
				output << "param_id: " << param_id <<
					" | param_value: " << mavlink_msg_param_value_get_param_value(msg) <<
					" | param_type: " << (int)mavlink_msg_param_value_get_param_type(msg) <<
					" | param_count: " << (int)mavlink_msg_param_value_get_param_count(msg) <<
					" | param_index: " << (int)mavlink_msg_param_value_get_param_index(msg);
				break;
			case MAVLINK_MSG_ID_GPS_RAW_INT:
				output << "time_usec: " << mavlink_msg_gps_raw_int_get_time_usec(msg) <<
					" | fix_type: " << (int)mavlink_msg_gps_raw_int_get_fix_type(msg) <<
					" | lat: " << (int)mavlink_msg_gps_raw_int_get_lat(msg) <<
					" | lon: " << (int)mavlink_msg_gps_raw_int_get_lon(msg) <<
					" | alt: " << (int)mavlink_msg_gps_raw_int_get_alt(msg) <<
					" | eph: " << (int)mavlink_msg_gps_raw_int_get_eph(msg) <<
					" | epv: " << (int)mavlink_msg_gps_raw_int_get_epv(msg) <<
					" | vel: " << (int)mavlink_msg_gps_raw_int_get_vel(msg) <<
					" | cog: " << (int)mavlink_msg_gps_raw_int_get_cog(msg) <<
					" | satellites_visible: " << (int)mavlink_msg_gps_raw_int_get_satellites_visible(msg);
				break;
			case MAVLINK_MSG_ID_ATTITUDE:
				output << "time_boot_ms: " << (int)mavlink_msg_attitude_get_time_boot_ms(msg) <<
					" | roll: " << mavlink_msg_attitude_get_roll(msg) <<
					" | pitch: " << mavlink_msg_attitude_get_pitch(msg) <<
					" | yaw: " << mavlink_msg_attitude_get_yaw(msg) <<
					" | rollspeed: " << mavlink_msg_attitude_get_rollspeed(msg) <<
					" | pitchspeed: " << mavlink_msg_attitude_get_pitchspeed(msg) <<
					" | yawspeed: " << mavlink_msg_attitude_get_yawspeed(msg);
				break;
			case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				output << "time_boot_ms: " << (int)mavlink_msg_local_position_ned_get_time_boot_ms(msg) <<
					" | x: " << mavlink_msg_local_position_ned_get_x(msg) <<
					" | y: " << mavlink_msg_local_position_ned_get_y(msg) <<
					" | z: " << mavlink_msg_local_position_ned_get_z(msg) <<
					" | vx: " << mavlink_msg_local_position_ned_get_vx(msg) <<
					" | vy: " << mavlink_msg_local_position_ned_get_vy(msg) <<
					" | vz: " << mavlink_msg_local_position_ned_get_vz(msg);
				break;
			case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
				output << "time_usec: " << (int)mavlink_msg_servo_output_raw_get_time_usec(msg) <<
					" | port: " << (int)mavlink_msg_servo_output_raw_get_port(msg) <<
					" | servo1_raw: " << (int)mavlink_msg_servo_output_raw_get_servo1_raw(msg) <<
					" | servo2_raw: " << (int)mavlink_msg_servo_output_raw_get_servo2_raw(msg) <<
					" | servo3_raw: " << (int)mavlink_msg_servo_output_raw_get_servo3_raw(msg) <<
					" | servo4_raw: " << (int)mavlink_msg_servo_output_raw_get_servo4_raw(msg) <<
					" | servo5_raw: " << (int)mavlink_msg_servo_output_raw_get_servo5_raw(msg) <<
					" | servo6_raw: " << (int)mavlink_msg_servo_output_raw_get_servo6_raw(msg) <<
					" | servo7_raw: " << (int)mavlink_msg_servo_output_raw_get_servo7_raw(msg) <<
					" | servo8_raw: " << (int)mavlink_msg_servo_output_raw_get_servo8_raw(msg);
				break;
			case MAVLINK_MSG_ID_MISSION_ITEM:
				output << "target_system: " << (int)mavlink_msg_mission_item_get_target_system(msg) <<
					" | target_component: " << (int)mavlink_msg_mission_item_get_target_component(msg) <<
					" | seq: " << (int)mavlink_msg_mission_item_get_seq(msg) <<
					" | frame: " << (int)mavlink_msg_mission_item_get_frame(msg) <<
					" | command: " << (int)mavlink_msg_mission_item_get_command(msg) <<
					" | current: " << (int)mavlink_msg_mission_item_get_current(msg) <<
					" | autocontinue: " << (int)mavlink_msg_mission_item_get_autocontinue(msg) <<
					" | param1: " << mavlink_msg_mission_item_get_param1(msg) <<
					" | param2: " << mavlink_msg_mission_item_get_param2(msg) <<
					" | param3: " << mavlink_msg_mission_item_get_param3(msg) <<
					" | param4: " << mavlink_msg_mission_item_get_param4(msg) <<
					" | x: " << mavlink_msg_mission_item_get_x(msg) <<
					" | y: " << mavlink_msg_mission_item_get_y(msg) <<
					" | z: " << mavlink_msg_mission_item_get_z(msg);
				break;
			case MAVLINK_MSG_ID_MISSION_REQUEST:
				output << "target_system: " << (int)mavlink_msg_mission_request_get_target_system(msg) <<
					" | target_component: " << (int)mavlink_msg_mission_request_get_target_component(msg) <<
					" | seq: " << (int)mavlink_msg_mission_request_get_seq(msg);
				break;
			case MAVLINK_MSG_ID_MISSION_CURRENT:
				output << "seq: " << (int)mavlink_msg_mission_current_get_seq(msg);
				break;
			case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
				output << "target_system: " << (int)mavlink_msg_mission_request_list_get_target_system(msg) <<
					" | target_component: " << (int)mavlink_msg_mission_request_list_get_target_component(msg);
				break;
			case MAVLINK_MSG_ID_MISSION_COUNT:
				output << "target_system: " << (int)mavlink_msg_mission_count_get_target_system(msg) <<
					" | target_component: " << (int)mavlink_msg_mission_count_get_target_component(msg) <<
					" | count: " << (int)mavlink_msg_mission_count_get_count(msg);
				break;
			case MAVLINK_MSG_ID_MISSION_ACK:
				output << "target_system: " << (int)mavlink_msg_mission_ack_get_target_system(msg) <<
					" | target_component: " << (int)mavlink_msg_mission_ack_get_target_component(msg) <<
					" | type: " << (int)mavlink_msg_mission_ack_get_type(msg);
				break;
			case MAVLINK_MSG_ID_VFR_HUD:
				output << "airspeed: " << mavlink_msg_vfr_hud_get_airspeed(msg) <<
					" | groundspeed: " << mavlink_msg_vfr_hud_get_groundspeed(msg) <<
					" | heading: " << (int)mavlink_msg_vfr_hud_get_heading(msg) <<
					" | throttle: " << (int)mavlink_msg_vfr_hud_get_throttle(msg) <<
					" | alt: " << mavlink_msg_vfr_hud_get_alt(msg) <<
					" | climb: " << mavlink_msg_vfr_hud_get_climb(msg);
				break;
			case MAVLINK_MSG_ID_COMMAND_LONG:
				output << "target_system: " << (int)mavlink_msg_command_long_get_target_system(msg) <<
					" | target_component: " << (int)mavlink_msg_command_long_get_target_component(msg) <<
					" | command: " << (int)mavlink_msg_command_long_get_command(msg) <<
					" | confirmation: " << (int)mavlink_msg_command_long_get_confirmation(msg) <<
					" | param1: " << mavlink_msg_command_long_get_param1(msg) <<
					" | param2: " << mavlink_msg_command_long_get_param2(msg) <<
					" | param3: " << mavlink_msg_command_long_get_param3(msg) <<
					" | param4: " << mavlink_msg_command_long_get_param4(msg) <<
					" | param5: " << mavlink_msg_command_long_get_param5(msg) <<
					" | param6: " << mavlink_msg_command_long_get_param6(msg) <<
					" | param7: " << mavlink_msg_command_long_get_param7(msg);
				break;
			case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				output << "time_boot_ms: " << (int)mavlink_msg_position_target_global_int_get_time_boot_ms(msg) <<
					" | coordinate_frame: " << (int)mavlink_msg_position_target_global_int_get_coordinate_frame(msg) <<
					" | type_mask: " << (int)mavlink_msg_position_target_global_int_get_type_mask(msg) <<
					" | lat_int: " << (int)mavlink_msg_position_target_global_int_get_lat_int(msg) <<
					" | lon_int: " << (int)mavlink_msg_position_target_global_int_get_lon_int(msg) <<
					" | alt: " << mavlink_msg_position_target_global_int_get_alt(msg) <<
					" | vx: " << mavlink_msg_position_target_global_int_get_vx(msg) <<
					" | vy: " << mavlink_msg_position_target_global_int_get_vy(msg) <<
					" | vz: " << mavlink_msg_position_target_global_int_get_vz(msg) <<
					" | afx: " << mavlink_msg_position_target_global_int_get_afx(msg) <<
					" | afy: " << mavlink_msg_position_target_global_int_get_afy(msg) <<
					" | afz: " << mavlink_msg_position_target_global_int_get_afz(msg) <<
					" | yaw: " << mavlink_msg_position_target_global_int_get_yaw(msg) <<
					" | yaw_rate: " << mavlink_msg_position_target_global_int_get_yaw_rate(msg);
				break;
			case MAVLINK_MSG_ID_HIL_CONTROLS:
				output << "time_usec: " << mavlink_msg_hil_controls_get_time_usec(msg) <<
					" | roll_ailerons: " << mavlink_msg_hil_controls_get_roll_ailerons(msg) <<
					" | pitch_elevator: " << mavlink_msg_hil_controls_get_pitch_elevator(msg) <<
					" | yaw_rudder: " << mavlink_msg_hil_controls_get_yaw_rudder(msg) <<
					" | throttle: " << mavlink_msg_hil_controls_get_throttle(msg) <<
					" | aux1: " << mavlink_msg_hil_controls_get_aux1(msg) <<
					" | aux2: " << mavlink_msg_hil_controls_get_aux2(msg) <<
					" | aux3: " << mavlink_msg_hil_controls_get_aux3(msg) <<
					" | aux4: " << mavlink_msg_hil_controls_get_aux4(msg) <<
					" | mode: " << (int)mavlink_msg_hil_controls_get_mode(msg) <<
					" | nav_mode: " << (int)mavlink_msg_hil_controls_get_nav_mode(msg);
				break;
			case MAVLINK_MSG_ID_HIGHRES_IMU:
				output << "time_usec: " << mavlink_msg_highres_imu_get_time_usec(msg) <<
					" | xacc: " << mavlink_msg_highres_imu_get_xacc(msg) <<
					" | yacc: " << mavlink_msg_highres_imu_get_yacc(msg) <<
					" | zacc: " << mavlink_msg_highres_imu_get_zacc(msg) <<
					" | xgyro: " << mavlink_msg_highres_imu_get_xgyro(msg) <<
					" | ygyro: " << mavlink_msg_highres_imu_get_ygyro(msg) <<
					" | zgyro: " << mavlink_msg_highres_imu_get_zgyro(msg) <<
					" | xmag: " << mavlink_msg_highres_imu_get_xmag(msg) <<
					" | ymag: " << mavlink_msg_highres_imu_get_ymag(msg) <<
					" | zmag: " << mavlink_msg_highres_imu_get_zmag(msg) <<
					" | abs_pressure: " << mavlink_msg_highres_imu_get_abs_pressure(msg) <<
					" | diff_pressure: " << mavlink_msg_highres_imu_get_diff_pressure(msg) <<
					" | pressure_alt: " << mavlink_msg_highres_imu_get_pressure_alt(msg) <<
					" | temperature: " << mavlink_msg_highres_imu_get_temperature(msg) <<
					" | fields_updated: " << (int)mavlink_msg_highres_imu_get_fields_updated(msg);
				break;
			case MAVLINK_MSG_ID_HIL_SENSOR:
				output << "time_usec: " << mavlink_msg_hil_sensor_get_time_usec(msg) <<
					" | xacc: " << mavlink_msg_hil_sensor_get_xacc(msg) <<
					" | yacc: " << mavlink_msg_hil_sensor_get_yacc(msg) <<
					" | zacc: " << mavlink_msg_hil_sensor_get_zacc(msg) <<
					" | xgyro: " << mavlink_msg_hil_sensor_get_xgyro(msg) <<
					" | ygyro: " << mavlink_msg_hil_sensor_get_ygyro(msg) <<
					" | zgyro: " << mavlink_msg_hil_sensor_get_zgyro(msg) <<
					" | xmag: " << mavlink_msg_hil_sensor_get_xmag(msg) <<
					" | ymag: " << mavlink_msg_hil_sensor_get_ymag(msg) <<
					" | zmag: " << mavlink_msg_hil_sensor_get_zmag(msg) <<
					" | abs_pressure: " << mavlink_msg_hil_sensor_get_abs_pressure(msg) <<
					" | diff_pressure: " << mavlink_msg_hil_sensor_get_diff_pressure(msg) <<
					" | pressure_alt: " << mavlink_msg_hil_sensor_get_pressure_alt(msg) <<
					" | temperature: " << mavlink_msg_hil_sensor_get_temperature(msg) <<
					" | fields_updated: " << (int)mavlink_msg_hil_sensor_get_fields_updated(msg);
				break;
			case MAVLINK_MSG_ID_HIL_GPS:
				output << "time_usec: " << mavlink_msg_hil_gps_get_time_usec(msg) <<
					" | fix_type: " << (int)mavlink_msg_hil_gps_get_fix_type(msg) <<
					" | lat: " << (int)mavlink_msg_hil_gps_get_lat(msg) <<
					" | lon: " << (int)mavlink_msg_hil_gps_get_lon(msg) <<
					" | alt: " << (int)mavlink_msg_hil_gps_get_alt(msg) <<
					" | eph: " << (int)mavlink_msg_hil_gps_get_eph(msg) <<
					" | epv: " << (int)mavlink_msg_hil_gps_get_epv(msg) <<
					" | vel: " << (int)mavlink_msg_hil_gps_get_vel(msg) <<
					" | vn: " << (int)mavlink_msg_hil_gps_get_vn(msg) <<
					" | ve: " << (int)mavlink_msg_hil_gps_get_ve(msg) <<
					" | vd: " << (int)mavlink_msg_hil_gps_get_vd(msg) <<
					" | cog: " << (int)mavlink_msg_hil_gps_get_cog(msg) <<
					" | satellites_visible: " << (int)mavlink_msg_hil_gps_get_satellites_visible(msg);
				break;
			case MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET:
				mavlink_msg_actuator_control_target_get_controls(msg, controls);
				output << "time_usec: " << mavlink_msg_actuator_control_target_get_time_usec(msg) <<
					" | group_mlx: " << (int)mavlink_msg_actuator_control_target_get_group_mlx(msg) <<
					" | controls: [" << controls[0]
					<< "," << controls[1]
					<< "," << controls[2]
					<< "," << controls[3]
					<< "," << controls[4]
					<< "," << controls[5]
					<< "," << controls[6]
					<< "," << controls[7] << "]";
				break;
			case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
				mavlink_msg_autopilot_version_get_flight_custom_version(msg, fcv);
				mavlink_msg_autopilot_version_get_middleware_custom_version(msg, mcv);
				mavlink_msg_autopilot_version_get_os_custom_version(msg, ocv);
				output << "capabilities: " << mavlink_msg_autopilot_version_get_capabilities(msg) <<
					" | flight_sw_version: " << (int)mavlink_msg_autopilot_version_get_flight_sw_version(msg) <<
					" | middleware_sw_version: " << (int)mavlink_msg_autopilot_version_get_middleware_sw_version(msg) <<
					" | os_sw_version: " << (int)mavlink_msg_autopilot_version_get_os_sw_version(msg) <<
					" | board_version: " << (int)mavlink_msg_autopilot_version_get_board_version(msg) <<
					" | flight_custom_version: [" << (int)fcv[0] <<
					"," << (int)fcv[1] <<
					"," << (int)fcv[2] <<
					"," << (int)fcv[3] <<
					"," << (int)fcv[4] <<
					"," << (int)fcv[5] <<
					"," << (int)fcv[6] <<
					"," << (int)fcv[7] << "]" <<
					" | middleware_custom_version: [" << (int)mcv[0] <<
					"," << (int)mcv[1] <<
					"," << (int)mcv[2] <<
					"," << (int)mcv[3] <<
					"," << (int)mcv[4] <<
					"," << (int)mcv[5] <<
					"," << (int)mcv[6] <<
					"," << (int)mcv[7] << "]" <<
					" | os_custom_version: [" << (int)ocv[0] <<
					"," << (int)ocv[1] <<
					"," << (int)ocv[2] <<
					"," << (int)ocv[3] <<
					"," << (int)ocv[4] <<
					"," << (int)ocv[5] <<
					"," << (int)ocv[6] <<
					"," << (int)ocv[7] << "]" <<
					" | vendor_id: " << (int)mavlink_msg_autopilot_version_get_vendor_id(msg) <<
					" | product_id: " << (int)mavlink_msg_autopilot_version_get_product_id(msg) <<
					" | uid: " << mavlink_msg_autopilot_version_get_uid(msg);
				break;
			case MAVLINK_MSG_ID_STATUSTEXT:
				mavlink_msg_statustext_get_text(msg, text);
				output << "severity: " << (int)mavlink_msg_statustext_get_severity(msg) <<
					" | text: " << text;
				break;
			default:
				output << "========= NOT TRANSLATED =========";
			}

			output << endl;
			// ===== END SAVE TO FILE =======================
		}
		//}
	}

	// close IO streams
	output.close();
	input.close();

	std::cout << "Mavlink messages have been generated. Press any key to exit." << endl;
}

double* rollPitchYaw(double x, double y, double z, bool inDegrees, bool reverse, double phi, double theta, double psi) {
	// convert degrees to radians
	if (inDegrees) {
		phi *= PI / 180;
		theta *= PI / 180;
		psi *= PI / 180;
	}

	// reverse the Euler angles
	if (reverse) {
		phi = -phi;
		theta = -theta;
		psi = -psi;
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

void main(int argc, char* argv[]) {
	preprocessFile("data.txt");
	processFile();

	//double rollD  = 78;
	//double pitchD = 17;
	//double yawD   = 315;
	//
	//double rollR  = rollD  * PI / 180;
	//double pitchR = pitchD * PI / 180;
	//double yawR   = yawD   * PI / 180;
	//
	//float xmag = 0.406549620;
	//float ymag = 0.059383093;
	//float zmag = 0.911696800;
	//
	//double* result = rollPitchYaw(xmag, ymag, zmag, false, true, rollR, pitchR, yawR);
	//
	//xmag = result[0];
	//ymag = result[1];
	//zmag = result[2];
	//
	//cout << "xmag: " << xmag << " | ymag: " << ymag << " | zmag: " << zmag << endl;
	//
	//_getch();
}