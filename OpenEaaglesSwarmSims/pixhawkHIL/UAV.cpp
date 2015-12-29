
#include "UAV.h"
#include "OnboardControlAgent.h"

#include "openeaagles/simulation/RfSensor.h"
#include "openeaagles/simulation/StoresMgr.h"
#include "openeaagles/simulation/DynamicsModel.h"
#include "openeaagles/simulation/Pilot.h"
#include "openeaagles/sensors/Tws.h"
#include "openeaagles/sensors/Gmti.h"
#include "openeaagles/basic/List.h"
#include "openeaagles/basic/osg/Matrix"
#include "openeaagles/basic/units/Angles.h"

namespace Eaagles {
namespace Swarms {

IMPLEMENT_SUBCLASS(UAV,"UAV")

//------------------------------------------------------------------------------
// Slot table for this form type
//------------------------------------------------------------------------------
BEGIN_SLOTTABLE(UAV)
	"xxx", // dummy slot
END_SLOTTABLE(UAV)

//------------------------------------------------------------------------------
//  Map slot table to handlers
//------------------------------------------------------------------------------
BEGIN_SLOT_MAP(UAV)
	//ON_SLOT(1, setOnboardControlAgent, Basic::Component)
END_SLOT_MAP()


//------------------------------------------------------------------------------
// Constructor(s)
//------------------------------------------------------------------------------
UAV::UAV()
{
	STANDARD_CONSTRUCTOR()
	
	static Basic::String generic("UAV");
	setType(&generic);

	oca = 0;
	loadOCA = true;
}

//------------------------------------------------------------------------------
// copyData(), deleteData() -- copy (delete) member data
//------------------------------------------------------------------------------
void UAV::copyData(const UAV& org, const bool)
{
	BaseClass::copyData(org);
	oca = 0;
	loadOCA = true;
}

void UAV::deleteData()
{
	setOnboardControlAgent(0);
}

//------------------------------------------------------------------------------
// reset() -- 
//------------------------------------------------------------------------------
void UAV::reset()
{
	updateOCA();
	loadOCA = false;
	BaseClass::reset();
}

//------------------------------------------------------------------------------
// getSlotByIndex() for BasicGL::Graphic
//------------------------------------------------------------------------------
Basic::Object* UAV::getSlotByIndex(const int si)
{
	return BaseClass::getSlotByIndex(si);
}

//------------------------------------------------------------------------------
// serialize
//------------------------------------------------------------------------------
std::ostream& UAV::serialize(std::ostream& sout, const int i, const bool slotsOnly) const
{
	using namespace std;

	int j = 0;
	if ( !slotsOnly ) {
		//indent(sout,i);
		sout << "( " << getFactoryName() << endl;
		j = 4;
	}

	//indent(sout,i+j);
	//sout << "masterMode: " << findMasterModeName(getMasterMode()) << endl;


	BaseClass::serialize(sout,i+j,true);

	if ( !slotsOnly ) {
		indent(sout,i);
		sout << ")" << endl;
	}

	return sout;
}

bool UAV::setOnboardControlAgent(Basic::Pair* const agent)
{
	bool ok = false;
	if (agent == 0) {
		if (oca != 0) oca->unref();
		oca = 0;
		ok = true;
	}
	else if ( agent->object()->isClassType(typeid(OnboardControlAgent)) ) {
		if (oca != 0) oca->unref();
		oca = agent;
		oca->ref();
		ok = true;
	}
	return ok;
}

void UAV::updateOCA()
{
	loadOCA = false;
	setOnboardControlAgent( findByType(typeid(Swarms::OnboardControlAgent)) );
}

//------------------------------------------------------------------------------
// updateTC() -- update time critical stuff here
//------------------------------------------------------------------------------
void UAV::updateTC(const LCreal dt)
{
   // Make sure we've loaded our OCA pointer
   if (loadOCA) {
	  updateOCA();
	  loadOCA = false;
   }
   BaseClass::updateTC(dt);
}

OnboardControlAgent* UAV::getOCA()
{
	return (oca != 0) ? (static_cast<OnboardControlAgent*>(oca->object())) : 0;
}

const OnboardControlAgent* UAV::getOCA() const
{
	return (oca != 0) ? (static_cast<OnboardControlAgent*>(oca->object())) : 0;
}

} // End Swarms namespace
} // End Eaagles namespace
