#pragma once

//------------------------------------------------------------------------------
// Class: AutoPilot
//------------------------------------------------------------------------------
#ifndef __Eaagles_Swarms_AutoPilot_H__
#define __Eaagles_Swarms_AutoPilot_H__

#include "openeaagles/simulation/Pilot.h"
#include "openeaagles/simulation/Steerpoint.h"
#include "openeaagles/simulation/Route.h"

using namespace std;

namespace Eaagles {
namespace Swarms {

class AutoPilot : public Simulation::Pilot
{
	DECLARE_SUBCLASS(AutoPilot, Simulation::Pilot)

public:
	AutoPilot();
		
	virtual const char* getMode() const;
	virtual int getWaypointCount() const { return priRoute->getNumberOfSteerpoints(); }
	virtual bool addWaypoint(Simulation::Steerpoint* wp);
	virtual bool removeWaypoint(int index);
	virtual bool updateCurrentWaypoint(double lat, double lon, double alt);
	virtual const Simulation::Steerpoint* getWaypoint(int index);
	virtual bool setCurrentWaypoint(int index);
	virtual bool setMode(const Basic::String* const m);

protected:
	bool setSlotMode(const Basic::String* const msg);
	virtual bool setSlotRoute(const Simulation::Route* const msg);

private:
	Eaagles::Basic::safe_ptr<const Basic::String> mode;
	Eaagles::Basic::safe_ptr<Simulation::Route> priRoute;
};

}
}
#endif