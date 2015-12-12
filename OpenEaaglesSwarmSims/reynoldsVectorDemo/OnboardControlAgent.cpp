#include "OnboardControlAgent.h"
#include "UAV.h"
#include "SwarmAP.h"

#include "openeaagles/basic/Number.h"
#include "openeaagles/simulation/Player.h"
#include "openeaagles/simulation/Navigation.h"
#include "openeaagles/simulation/Steerpoint.h"
#include "openeaagles/simulation/Route.h"
#include "openeaagles/simulation/Simulation.h"
#include "openeaagles/basic/Pair.h"
#include "openeaagles/basic/PairStream.h"

#include "openeaagles/basic/EarthModel.h"
#include "openeaagles/basic/osg/Vec3"
#include "openeaagles/basic/osg/Vec4"
#include "openeaagles/basic/osg/Matrix"
#include "openeaagles/basic/units/Angles.h"
#include "openeaagles/basic/units/Distances.h"

#include "math.h"

// used for testing
#include <iostream>
#include "conio.h" // _getch() used for pausing execution
#include <iomanip>

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
	ON_SLOT( 1, setSlotSeparationFactor,  Basic::Number)
	ON_SLOT( 2, setSlotAlignmentFactor,   Basic::Number)
	ON_SLOT( 3, setSlotCohesionFactor,    Basic::Number)
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

	sFactor    = 1.0;
	aFactor    = 1.0;
	cFactor    = 1.0;
	commDist   = 27780; // measured in meters (15 Nautical Miles)
	desiredSep = 1000;  // in meters
	wp = 0;
}

//------------------------------------------------------------------------------------
// copyData() - copies one object to another
//------------------------------------------------------------------------------------
void OnboardControlAgent::copyData(const OnboardControlAgent& org, const bool) 
{
	BaseClass::copyData(org);

	sFactor    = org.sFactor;
	aFactor    = org.aFactor;
	cFactor    = org.cFactor;
	commDist   = org.commDist;
	desiredSep = org.desiredSep;
	wp = 0;
}

//------------------------------------------------------------------------------------
// deleteData() -- delete this object's data
//------------------------------------------------------------------------------------
void OnboardControlAgent::deleteData()
{
	if(wp != 0) {
		// remove waypoint from player's navigation route (if exists)
		Swarms::UAV* uav = dynamic_cast<Swarms::UAV*>(getOwnship());
		Eaagles::Simulation::Navigation* nav = uav->getNavigation();
		if(nav != 0) {
			Eaagles::Simulation::Route* route = nav->getPriRoute();
			if(route != 0) {
				route->deleteSteerpoint(wp);
			}
		}
		// delete waypoint
		wp->unref();
	}
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
					osg::Vec3d s = v*pow((1000/v.length()), 2);
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
// Update non-time critical stuff here
//------------------------------------------------------------------------------------

void OnboardControlAgent::updateData(const LCreal dt)
{
	Swarms::UAV* uav = dynamic_cast<Swarms::UAV*>(getOwnship());
	const char* mode = dynamic_cast<SwarmAP*>(uav->getPilot())->getMode();

	if( strcmp(mode, "swarm") != 0 ) return;

	Simulation::Navigation* nav = uav->getNavigation();
	if(nav == 0) {
		nav = new Simulation::Navigation();
		Basic::Pair* navPair = new Basic::Pair("Navigation", nav);
		uav->addComponent(navPair);
	}
	Simulation::Route* route = nav->getPriRoute();
	if(route == 0) {
		route = new Simulation::Route();
		nav->setRoute(route);
	}
	
	// Update next waypoint based on flocking algorithm/parameters
	if(wp == 0) {
		wp = new Simulation::Steerpoint();
		route->insertSteerpoint(wp);
	}
	Eaagles::osg::Vec3d aVec = getAlignmentVector();
	Eaagles::osg::Vec3d sVec = getSeparationVector();
	Eaagles::osg::Vec3d cVec = getCohesionVector();

	Eaagles::osg::Vec3d nextWaypoint = aVec + sVec + cVec;

	Eaagles::osg::Vec3d uavPosition = uav->getPosition();

	if(nextWaypoint.length() == 0) {
		// set waypoint for straight ahead (same altitude)
		wp->setPosition( uavPosition + uav->getVelocity()*5000 );
		wp->setCmdAltitude( uav->getAltitude() );
	} else {
		wp->setPosition( uavPosition + nextWaypoint );
		wp->setCmdAltitude( -(uavPosition + nextWaypoint).z() );
		//================================================================================================================
		// DRAW OCA CREATED WAYPOINT
		Swarms::UAV* owner = dynamic_cast<Swarms::UAV*>(getOwnship());
		Basic::PairStream* players = owner->getSimulation()->getPlayers();

		int i = 1;
		while(true) {
			Basic::Pair* player = players->getPosition(i);
			if(player != 0) {
				Simulation::Player* p = dynamic_cast<Simulation::Player*>(player->object());
				if(p->getID() == 15) {
					p->setPosition(wp->getPosition());
				}
				if(p->getID() == 16) {
					p->setPosition(uavPosition + aVec);
				}
				if(p->getID() == 17) {
					p->setPosition(uavPosition + sVec);
				}
				if(p->getID() == 18) {
					p->setPosition(uavPosition + cVec);
				}
			} else break;
			i++;
		}
		//================================================================================================================
	}
	route->directTo(wp);
	BaseClass::updateData(dt);
}

} // end Swarms namespace
} // end Eaagles namespace