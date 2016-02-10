//------------------------------------------------------------------------------
// Class: SwarmSimulation
//------------------------------------------------------------------------------
#ifndef __Eaagles_Swarms_SwarmSimulation_H_
#define __Eaagles_Swarms_SwarmSimulation_H__

#include "openeaagles/simulation/Simulation.h"

namespace Eaagles {
namespace Swarms {

class SwarmSimulation : public Simulation::Simulation
{
	DECLARE_SUBCLASS(SwarmSimulation, Simulation)

public:
	SwarmSimulation();
protected:
	void printTimingStats() override;
private:
	bool initialized = false;
	int count = 0;
	int timeSlots[21];
};

} // End Swarms namespace
} // End Eaagles namespace

#endif
