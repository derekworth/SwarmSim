//#include "UAV.h"

#include "Factory.h"
#include "SimAP.h"
#include "UAV.h"

#include "openeaagles/basic/Parser.h"
#include "openeaagles/simulation/Player.h"
#include "openeaagles/basic/Pair.h"
#include "openeaagles/basic/PairStream.h"
#include "openeaagles/basic/Timers.h"
#include "openeaagles/simulation/Station.h"
// extra for printing:
#include "openeaagles/simulation/Simulation.h"
#include "openeaagles/simulation/Autopilot.h"
#include "openeaagles/dynamics/JSBSimModel.h"

// class factories
#include "openeaagles/simulation/Factory.h"
#include "openeaagles/dynamics/Factory.h"
#include "openeaagles/sensors/Factory.h"
#include "openeaagles/dis/Factory.h"
#include "openeaagles/basic/Factory.h"
#include "openeaagles/ioDevice/Factory.h"

#include <cstring>
#include <cstdlib>

// used for testing
#include <iostream>
#include "conio.h"
#include <iomanip>
#include <fstream>  // used to write to file

using namespace std;

namespace Eaagles {
	namespace Swarms {

		// Test file	
		const char* configFile = "swarm.edl";

		// Background frame rate (Hz)
		const int bgRate = 50;

		// System descriptions
		static Simulation::Station* station = 0;
		
		//-----------------------------------------------------------------------------
		// Eaagles::Swarms::builder() -- builds simulation tree
		//-----------------------------------------------------------------------------
		static void builder()
		{
			cout << "Reading file : " << configFile << endl;

			// Read the description file
			int errors = 0;
			Basic::Object* q1 = Basic::lcParser(configFile, Factory::createObj, &errors);
			if (errors > 0) {
				cerr << "File: " << configFile << ", errors: " << errors << endl;
				exit(1);
			}

			station = 0;
			if (q1 != 0) {
				// When we were given a Basic::Pair, get the pointer to its object.
				Basic::Pair* pp = dynamic_cast<Basic::Pair*>(q1);
				if (pp != 0) {
					q1 = pp->object();
				}

				// What we should have here is the Station object
				station = dynamic_cast<Simulation::Station*>(q1);
			}

			// Make sure we did get a valid Station object (we must have one!)
			if (station == 0) {
				cout << "Invalid description file!" << endl;
				exit(EXIT_FAILURE);
			}
		}

		//-----------------------------------------------------------------------------
		// Eaagles::Swarms::main() -- Main routine
		//-----------------------------------------------------------------------------
		void exec(int argc, char* argv[])
		{
			// parse arguments
			for (int i = 1; i < argc; i++) {
				if (strcmp(argv[i],"-f") == 0) {
					configFile = argv[++i];
				}
			}
			// build a Station
			builder();

			// Reset the Simulation
			station->event(Basic::Component::RESET_EVENT);
			// Set timer for the background tasks
			station->tcFrame( static_cast<LCreal>(1.0/static_cast<double>(station->getTimeCriticalRate())) );
			
			// Create the Time Critical Thread
			station->createTimeCriticalProcess();
			// short pause to allow os to startup thread
			lcSleep(2000);
			
			// Calc delta time for background thread
			double dt = 1.0/static_cast<double>(bgRate);
			
			// System Time of Day 
			double simTime = 0.0;                 // Simulator time reference
			double startTime = getComputerTime(); // Time of day (sec) run started

			//cout << "Simulation running..." << endl;
			
			while (true) { // runs for 10 minutes
				// Update background thread
				station->updateData(static_cast<LCreal>(dt));
			
				simTime += dt;                      // time of next frame
				double timeNow = getComputerTime(); // time now
			
				double elapsedTime = timeNow - startTime;
				double nextFrameStart = simTime - elapsedTime;
				int sleepTime = static_cast<int>(nextFrameStart*1000.0);

				// wait for the next frame
				if (sleepTime > 0)
					lcSleep(sleepTime);
			}
		}
	} // End Swarms namespace
} // End Eaagles namespace


//-----------------------------------------------------------------------------
// main() -- Main routine
//-----------------------------------------------------------------------------
void main(int argc, char* argv[])
{
	Eaagles::Swarms::exec(argc, argv);
}