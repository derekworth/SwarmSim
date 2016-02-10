#include "OnboardControlAgent.h"
#include "UAV.h"
#include "SimAP.h"
#include "PixhawkAP.h"

#include "openeaagles/basic/Number.h"
#include "openeaagles/simulation/Player.h"
#include "openeaagles/simulation/Simulation.h"
#include "openeaagles/basic/Pair.h"
#include "openeaagles/basic/PairStream.h"

namespace Eaagles {
namespace Swarms {

// =============================================================================
// class: OnboardControlAgent
// =============================================================================

IMPLEMENT_SUBCLASS(OnboardControlAgent,"OnboardControlAgent")

BEGIN_SLOTTABLE(OnboardControlAgent)
	"sFactor",
	"aFactor",
	"cFactor",
	"commDistance",
	"desiredSeparation",
END_SLOTTABLE(OnboardControlAgent)

BEGIN_SLOT_MAP(OnboardControlAgent)
	ON_SLOT( 1, setSlotSeparationFactor,  Basic::Number)   // Default = 0.5
	ON_SLOT( 2, setSlotAlignmentFactor,   Basic::Number)   // Default = 10
	ON_SLOT( 3, setSlotCohesionFactor,    Basic::Number)   // Default = 1
	ON_SLOT( 4, setSlotCommDistance,      Basic::Distance) // Distance in nautical miles, default = 15 NM
	ON_SLOT( 4, setSlotCommDistance,      Basic::Number)   // Distance in meters
	ON_SLOT( 5, setSlotDesiredSeparation, Basic::Distance) // Distance in nautical miles
	ON_SLOT( 5, setSlotDesiredSeparation, Basic::Number)   // Distance in meters, default = 1000 meters
END_SLOT_MAP()                                                                        

Eaagles::Basic::Object* OnboardControlAgent::getSlotByIndex(const int si)                         
{                                                                                     
	return BaseClass::getSlotByIndex(si);
}

EMPTY_SERIALIZER(OnboardControlAgent)

//------------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------------
OnboardControlAgent::OnboardControlAgent() 
{
	STANDARD_CONSTRUCTOR()

	startTime  = 0;
	sFactor	   = 0.5;
	aFactor    = 10.0;
	cFactor    = 1.0;
	commDist   = 27780; // measured in meters (15 Nautical Miles)
	desiredSep = 1000;  // in meters
}

//------------------------------------------------------------------------------------
// copyData() - copies one object to another
//------------------------------------------------------------------------------------
void OnboardControlAgent::copyData(const OnboardControlAgent& org, const bool) 
{
	BaseClass::copyData(org);

	startTime  = org.startTime;
	sFactor    = org.sFactor;
	aFactor    = org.aFactor;
	cFactor    = org.cFactor;
	commDist   = org.commDist;
	desiredSep = org.desiredSep;
}

//------------------------------------------------------------------------------------
// deleteData() -- delete this object's data
//------------------------------------------------------------------------------------
void OnboardControlAgent::deleteData() {

}

//------------------------------------------------------------------------------------
// Getter methods
//------------------------------------------------------------------------------------
	
Eaagles::osg::Vec3d OnboardControlAgent::getSeparationVector() {
	Swarms::UAV* owner = dynamic_cast<Swarms::UAV*>(getOwnship());
	Eaagles::osg::Vec3d pos1 = owner->getPosition();
	Basic::PairStream* players = owner->getSimulation()->getPlayers();
	Eaagles::osg::Vec3d sum(0,0,0); // used to sum the separation vectors of neighboring UAVs
	int i = 1;
	int count = 0;

	while(true) {
		Basic::Pair* player = players->getPosition(i);
		if(player != 0) {
			UAV* uav = dynamic_cast<UAV*>(player->object());
			if(uav != 0 && owner->getID() != uav->getID()) {
				Eaagles::osg::Vec3d pos2 = uav->getPosition();
				double dist = getDistance(pos1, pos2); // calc distance
				if(dist > 0 && dist < getDesiredSeparation()) { // determine if UAVs are within range to communicate
					osg::Vec3d v = pos1-pos2;
					osg::Vec3d s = v*pow((desiredSep / v.length()), 2);
					sum += s;
					count++;
				}
			}
		} else break;
		i++;
	}

	if(count > 0)
		return (sum/count) * sFactor;
	else
		return sum;
}

Eaagles::osg::Vec3d OnboardControlAgent::getAlignmentVector() {
	Swarms::UAV* owner = dynamic_cast<Swarms::UAV*>(getOwnship());
	Eaagles::osg::Vec3d pos = owner->getPosition();
	Basic::PairStream* players = owner->getSimulation()->getPlayers();
	Eaagles::osg::Vec3d sum(0,0,0); // used to sum the velocity vectors of neighboring UAVs
	int i = 1;
	int count = 0;

	while(true) {
		Basic::Pair* player = players->getPosition(i);
		if(player != 0) {
			UAV* uav = dynamic_cast<UAV*>(player->object());
			if(uav != 0 && owner->getID() != uav->getID()) {
				// calc distance
				double dist = getDistance(pos, uav->getPosition());
				if(dist > 0 && dist < getCommDistance()) {
					sum += uav->getVelocity();
					count++;
				}
			}
		} else break;
		i++;
	}

	if(count > 0)
		return (sum/count) * aFactor;
	else
		return sum;
}

Eaagles::osg::Vec3d OnboardControlAgent::getCohesionVector() {
	Swarms::UAV* owner = dynamic_cast<Swarms::UAV*>(getOwnship());
	Eaagles::osg::Vec3d pos1 = owner->getPosition();
	Basic::PairStream* players = owner->getSimulation()->getPlayers();
	Eaagles::osg::Vec3d sum(0,0,0); // used to sum the position vectors of neighboring UAVs
	int i = 1;
	int count = 0;

	while(true) {
		Basic::Pair* player = players->getPosition(i);
		if(player != 0) {
			UAV* uav = dynamic_cast<UAV*>(player->object());
			if(uav != 0 && owner->getID() != uav->getID()) {
				Eaagles::osg::Vec3d pos2 = uav->getPosition();
				double dist = getDistance(pos1, pos2); // calc distance
				if(dist > 0 && dist < getCommDistance()) { // determine if UAVs are within range to communicate
					sum += pos2;
					count++;
				}
			}
		} else break;
		i++;
	}

	if(count > 0)
		return (sum/count - pos1) * cFactor;
	else
		return sum;
}

//------------------------------------------------------------------------------------
// Setter methods
//------------------------------------------------------------------------------------

bool OnboardControlAgent::setSeparationFactor(const double factr)
{
   sFactor = factr;
   return true;
}

bool OnboardControlAgent::setAlignmentFactor(const double factr)
{
   aFactor = factr;
   return true;
}

bool OnboardControlAgent::setCohesionFactor(const double factr)
{
   cFactor = factr;
   return true;
}

bool OnboardControlAgent::setCommDistance(const double dist)
{
   commDist = dist;
   return true;
}

bool OnboardControlAgent::setDesiredSeparation(const double sep)
{
   desiredSep = sep;
   return true;
}

//------------------------------------------------------------------------------------
// Slot methods
//------------------------------------------------------------------------------------

bool OnboardControlAgent::setSlotSeparationFactor(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setSeparationFactor(msg->getDouble());
   return ok;
}

bool OnboardControlAgent::setSlotAlignmentFactor(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setAlignmentFactor(msg->getDouble());
   return ok;
}

bool OnboardControlAgent::setSlotCohesionFactor(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setCohesionFactor(msg->getDouble());
   return ok;
}

bool OnboardControlAgent::setSlotCommDistance(const Basic::Number* const msg)
{
	bool ok = false;
	if(msg != 0) {
		double dist = msg->getDouble();
		ok = setCommDistance(dist);
	}
	return ok;
}

bool OnboardControlAgent::setSlotCommDistance(const Basic::Distance* const msg)
{
	bool ok = false;
	if(msg != 0) {
		double dist = Basic::Meters::convertStatic(*msg);
		ok = setCommDistance(dist);
	}
	return ok;
}

bool OnboardControlAgent::setSlotDesiredSeparation(const Basic::Number* const msg)
{
	bool ok = false;
	if(msg != 0) {
		double dist = msg->getDouble();
		ok = setDesiredSeparation(dist);
	}
	return ok;
}

bool OnboardControlAgent::setSlotDesiredSeparation(const Basic::Distance* const msg)
{
	bool ok = false;
	if(msg != 0) {
		double dist = Basic::Meters::convertStatic(*msg);
		ok = setDesiredSeparation(dist);
	}
	return ok;
}

//------------------------------------------------------------------------------------
// DWF updates are not time-critical
//------------------------------------------------------------------------------------

void OnboardControlAgent::updateData(const LCreal dt) {
	if (startTime > getComputerTime()) return;
	startTime = getComputerTime() + 5; // control the refresh rate here (in seconds / update); set to +5 sec during HIL
	
	Swarms::UAV* uav = dynamic_cast<Swarms::UAV*>(getOwnship());
	if (uav == nullptr) return;
	Swarms::SwarmAutopilot* ap = dynamic_cast<Swarms::SwarmAutopilot*>(uav->getPilot());
	if (ap == nullptr) return;

	// Reynolds Flocking rules:
	Eaagles::osg::Vec3d aVec = getAlignmentVector();
	Eaagles::osg::Vec3d sVec = getSeparationVector();
	Eaagles::osg::Vec3d cVec = getCohesionVector();

	Eaagles::osg::Vec3d nextWaypoint = aVec + sVec + cVec; // inertial frame (NED with UAV origin)
	Eaagles::osg::Vec3d uavPosition  = uav->getPosition();

	if(nextWaypoint.length() == 0) {
		// set waypoint for straight ahead (same altitude)
		ap->setWaypoint(uavPosition + uav->getVelocity() * 500, uav->getAltitude());
	} else {
		ap->setWaypoint(uavPosition + nextWaypoint, -(uavPosition + nextWaypoint).z());

		//========================================================================================
		// DRAW OCA CREATED WAYPOINTS															  
		//========================================================================================
		if (false) { // set to true only while simulating one swarming UAV with Reynolds vectors
			Swarms::UAV* owner = dynamic_cast<Swarms::UAV*>(getOwnship());
			Basic::PairStream* players = owner->getSimulation()->getPlayers();

			int i = 1;
			while (true) {
				Basic::Pair* player = players->getPosition(i);
				if (player != 0) {
					Simulation::Player* p = dynamic_cast<Simulation::Player*>(player->object());
					switch (p->getID()) {
					case 14: // vect_A
						p->setPosition(uavPosition + aVec);
						break;
					case 15: // vect_S
						p->setPosition(uavPosition + sVec);
						break;
					case 16: // vect_C
						p->setPosition(uavPosition + cVec);
						break;
					case 17: // vect_X
						p->setPosition(uavPosition + nextWaypoint);
						break;
					}
				} else break;
				i++;
			}
		}
		//========================================================================================
	}
	
	BaseClass::updateData(dt);
}

} // end Swarms namespace
} // end Eaagles namespace