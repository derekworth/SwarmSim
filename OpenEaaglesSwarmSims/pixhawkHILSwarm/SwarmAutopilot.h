#pragma once

//------------------------------------------------------------------------------
// Class: SwarmAutopilot
//   Provides an autopilot interface that enforces Dynamic Waypoint
//   Following (DWF) for swarm behavior modeling.
//------------------------------------------------------------------------------
#ifndef __Eaagles_Simulation_SwarmAutopilot_H__
#define __Eaagles_Simulation_SwarmAutopilot_H__

#include "openeaagles/simulation/Pilot.h"

namespace Eaagles {
namespace Swarms {

//------------------------------------------------------------------------------
// Class: SwarmAutopilot
// Description: Base swarm autopilot model ...
//
//    Base class for all swarm autopilot decision logic models for
//    any Eaagles UAV type.
//
//    This class is one of the "top level" systems attached to a Player
//    class (see Player.h).
// Factory name: SwarmAutopilot
//------------------------------------------------------------------------------
class SwarmAutopilot : public Simulation::Pilot
{
	DECLARE_SUBCLASS(SwarmAutopilot, Simulation::Pilot)

public:
	SwarmAutopilot();
	
	// modes include navigation, swarming, and manual
	virtual const char* getMode() const;
	virtual void setMode(const Basic::String* const m);

	// Dynamic Waypoint Following
	virtual void setWaypoint(const osg::Vec3& posMetersNED, const LCreal altMeters);
};

}
}
#endif