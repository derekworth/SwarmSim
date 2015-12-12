#include "AutoPilot.h"
#include "UAV.h"

#include "openeaagles/basic/Number.h"
#include "openeaagles/simulation/Player.h"
#include "openeaagles/simulation/Navigation.h"
#include "openeaagles/simulation/Steerpoint.h"
#include "openeaagles/simulation/Route.h"

// used for testing
#include <iostream>
#include <iomanip>

namespace Eaagles {
namespace Swarms {

// =============================================================================
// class: AutoPilot
// =============================================================================

IMPLEMENT_SUBCLASS(AutoPilot,"AutoPilot")

BEGIN_SLOTTABLE(AutoPilot)
	"mode",
	"route,"
END_SLOTTABLE(AutoPilot)

BEGIN_SLOT_MAP(AutoPilot)
	ON_SLOT(1, setSlotMode, Basic::String) // modes: "nav" and "swarm"; default = "swarm"
	ON_SLOT(2, setSlotRoute, Simulation::Route)
END_SLOT_MAP()                                                                        

Eaagles::Basic::Object* AutoPilot::getSlotByIndex(const int si)                         
{                                                                                     
	return BaseClass::getSlotByIndex(si);                                             
}

EMPTY_SERIALIZER(AutoPilot)

//------------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------------
AutoPilot::AutoPilot() 
{
	STANDARD_CONSTRUCTOR()

	priRoute = nullptr;
	mode = nullptr;
	setMode(new Basic::String("swarm"));
}

//------------------------------------------------------------------------------------
// copyData() - copies one object to another
//------------------------------------------------------------------------------------
void AutoPilot::copyData(const AutoPilot& org, const bool cc) 
{
	BaseClass::copyData(org);

	if(cc) {
		mode = nullptr;
	}

	if (org.priRoute != nullptr) {
		Simulation::Route* p = org.priRoute->clone();
		priRoute = p;
		p->container(this);
		p->unref();  // safe_ptr<> has it
	}
	else priRoute = nullptr;

	if (org.mode != nullptr) {
		Basic::String* m = org.mode->clone();
		mode = m;
		m->unref();
	}
	else mode = nullptr;
}

//------------------------------------------------------------------------------------
// deleteData() -- delete this object's data
//------------------------------------------------------------------------------------
void AutoPilot::deleteData()
{
	priRoute = nullptr;
	mode = nullptr;
}

//------------------------------------------------------------------------------------
// Getter methods
//------------------------------------------------------------------------------------

const Simulation::Steerpoint* AutoPilot::getWaypoint(int index)
{
	return nullptr;
}

const char* AutoPilot::getMode() const
{
	const char* p = 0;
	if (mode != 0) p = *mode;
	return p;
}

//------------------------------------------------------------------------------------
// Setter methods
//------------------------------------------------------------------------------------

bool AutoPilot::setMode(const Basic::String* const m)
{
   mode = m;
   return true;
}

bool AutoPilot::setCurrentWaypoint(int index)
{
	return false;
}

//------------------------------------------------------------------------------------
// Misc methods
//------------------------------------------------------------------------------------

bool AutoPilot::addWaypoint(Simulation::Steerpoint* wp)
{
	return false;
}

bool AutoPilot::removeWaypoint(int index)
{
	return false;
}

bool AutoPilot::updateCurrentWaypoint(double lat, double lon, double alt)
{
	return false;
}

//------------------------------------------------------------------------------------
// Slot methods
//------------------------------------------------------------------------------------

bool AutoPilot::setSlotRoute(const Simulation::Route* const msg)
{
	priRoute = msg->clone();

	if (priRoute != nullptr) {
		priRoute->container(this);
	}
	return true;
}

bool AutoPilot::setSlotMode(const Basic::String* const msg)
{
	bool ok = false;
	if (msg != 0) {
		if( strcmp(msg->getString(), "nav") == 0) { setMode(msg); }
		else setMode(new Basic::String("swarm"));
		ok = true;
	}
	return ok;
}

} // end Swarms namespace
} // end Eaagles namespace