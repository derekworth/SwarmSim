// Class: SwarmSimulation

#include "SwarmSimulation.h"
#include "openeaagles/basic/Statistic.h"
#include <fstream>
#include "conio.h"

#include "openeaagles/basic/Number.h"
#include "openeaagles/simulation/Player.h"
#include "openeaagles/simulation/Simulation.h"
#include "openeaagles/basic/Pair.h"
#include "openeaagles/basic/PairStream.h"

using namespace std;

namespace Eaagles {
namespace Swarms {

IMPLEMENT_SUBCLASS(SwarmSimulation, "SwarmSimulation")
EMPTY_SLOTTABLE(SwarmSimulation)
EMPTY_CONSTRUCTOR(SwarmSimulation)
EMPTY_COPYDATA(SwarmSimulation)
EMPTY_DELETEDATA(SwarmSimulation)
EMPTY_SERIALIZER(SwarmSimulation)

void SwarmSimulation::printTimingStats() {
	if (count >= 30000) return;

	if (!initialized) {
		initialized = true;
		for (int i = 0; i < 22; i++) {
			timeSlots[i] = 0;
		}
	}
	const Basic::Statistic* ts = getTimingStats();
	//int c = cycle();
	//int f = frame() - 1;
	//if (f < 0) {
	//	c--;
	//	f = 15;
	//}

	// values rounded down to nearest millisecond
	int dt  = (int)ts->value();
	int avg = (int)ts->mean();
	int max = (int)ts->maxValue();

	if (dt > 19) {
		timeSlots[20]++; // greater than or equal to 20 ms
	} else {
		timeSlots[dt]++; // less than 20 ms
	}
	count++;

	// save performance data to file
	if (count >= 30000) {
		cout << "\nSimulation successfully performed 50K updates." << std::endl;
		
		while (true) {
			// "append to" file
			ofstream output;
			output.open("sim-update-durations.csv", ios::trunc);

			if (output.is_open()) {
				for (int i = 0; i < 21; i++) {
					output << i + 1 << "," << timeSlots[i] << "\n";
				}
				output.close();
				cout << "Results successfully written to file 'sim-update-durations.csv'." << endl;
				break;
			} else {
				cout << "ERROR: results not written to file, 'sim-update-durations.csv' could not be opened." << endl;
				cout << "Please check that file is not already open. Would you like to try again (y/n)?" << endl;
				int input = _getch();
				if (input != 'y' && input != 'Y') {
					cout << "Simulation results not written to file." << endl;
					break;
				}
			}
		}
	}

	//std::cout << "simulation(" << c << "," << f << "): dt=" << ts->value() << ", ave=" << ts->mean() << ", max=" << ts->maxValue() << std::endl;
	double time = (30000 - count) * 0.02 / 60;
	int min = (int) time;
	double sec = (time - min) * 60;
	if (sec < 10)
		cout << "\r" << getPlayers()->entries() - 13 << " UAVs | time remaining: " << min << ":0" << (int)sec << "                                                       ";
	else
		cout << "\r" << getPlayers()->entries() - 13 << " UAVs | time remaining: " << min << ":" << (int)sec << "                                                       ";
	
}

} // End Swarms namespace
} // End Eaagles namespace
