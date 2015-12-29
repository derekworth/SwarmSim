#include "SwarmAutopilot.h"
#include "OnboardControlAgent.h"
#include "UAV.h"

#include "openeaagles/simulation/Navigation.h"
#include "openeaagles/simulation/Route.h"
#include "openeaagles/simulation/Steerpoint.h"
#include "openeaagles/dynamics/JSBSimModel.h"

namespace Eaagles {
namespace Swarms {

IMPLEMENT_EMPTY_SLOTTABLE_SUBCLASS(SwarmAutopilot, "SwarmAutopilot")
EMPTY_SERIALIZER(SwarmAutopilot)

//------------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------------
SwarmAutopilot::SwarmAutopilot() 
{
	STANDARD_CONSTRUCTOR()
}

//------------------------------------------------------------------------------------
// copyData() - copies one object to another
//------------------------------------------------------------------------------------
void SwarmAutopilot::copyData(const SwarmAutopilot& org, const bool cc) 
{
	BaseClass::copyData(org);
}

//------------------------------------------------------------------------------------
// deleteData() -- delete this object's data
//------------------------------------------------------------------------------------
void SwarmAutopilot::deleteData()
{
}

//------------------------------------------------------------------------------------
// Getter and Setter methods
//------------------------------------------------------------------------------------

const char* SwarmAutopilot::getMode() const {
	return nullptr;
}

void SwarmAutopilot::setMode(const Basic::String* const m) {

}

void SwarmAutopilot::setWaypoint(const osg::Vec3& posMetersNED, const LCreal altMeters) {

}

} // end Swarms namespace
} // end Eaagles namespace