//------------------------------------------------------------------------------
// Class: UAV
//
// Description: Simulation player
//------------------------------------------------------------------------------
#ifndef __Eaagles_Swarms_UAV_H_
#define __Eaagles_Swarms_UAV_H__

#include "OnboardControlAgent.h"
#include "openeaagles/simulation/AirVehicle.h"

namespace Eaagles {

namespace Swarms {

class UAV : public Simulation::UnmannedAirVehicle
{
	DECLARE_SUBCLASS(UAV, Simulation::UnmannedAirVehicle)

public:
	UAV();

	// Basic::Component interface
	virtual void reset();
	virtual bool setOnboardControlAgent(Basic::Pair* const agent); // Sets our Onboard Control Agent (for swarming)
	virtual void updateTC(const LCreal dt = 0.0);
	
	OnboardControlAgent* getOCA();               
	const OnboardControlAgent* getOCA() const; 

protected:
	virtual void updateOCA();
private:
	Basic::Pair* oca;
	bool loadOCA;
};

} // End Swarms namespace
} // End Eaagles namespace

#endif
