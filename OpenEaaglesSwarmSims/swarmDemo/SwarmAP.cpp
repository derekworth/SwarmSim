#include "SwarmAP.h"
#include "UAV.h"
#include "OnboardControlAgent.h"

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
// class: SwarmAP
// =============================================================================

IMPLEMENT_SUBCLASS(SwarmAP,"SwarmAP")

BEGIN_SLOTTABLE(SwarmAP)
	"rollKp",
	"rollKi",
	"rollKd",
	"pitchKp",
	"pitchKi",
	"pitchKd",
	"yawKp",
	"yawKi",
	"yawKd",
	"altKp",
	"altKi",
	"altKd",
	"hdgKp",
	"hdgKi",
	"hdgKd",
	"minPitch",
	"maxPitch",
	"maxRoll",
	"mode",
END_SLOTTABLE(SwarmAP)

BEGIN_SLOT_MAP(SwarmAP)
	ON_SLOT(  1, setSlotRollKp,   Basic::Number)
	ON_SLOT(  2, setSlotRollKi,   Basic::Number)
	ON_SLOT(  3, setSlotRollKd,   Basic::Number)
	ON_SLOT(  4, setSlotPitchKp,  Basic::Number)
	ON_SLOT(  5, setSlotPitchKi,  Basic::Number)
	ON_SLOT(  6, setSlotPitchKd,  Basic::Number)
	ON_SLOT(  7, setSlotYawKp,    Basic::Number)
	ON_SLOT(  8, setSlotYawKi,    Basic::Number)
	ON_SLOT(  9, setSlotYawKd,    Basic::Number)
	ON_SLOT( 10, setSlotAltKp,    Basic::Number)
	ON_SLOT( 11, setSlotAltKi,    Basic::Number)
	ON_SLOT( 12, setSlotAltKd,    Basic::Number)
	ON_SLOT( 13, setSlotHdgKp,    Basic::Number)
	ON_SLOT( 14, setSlotHdgKi,    Basic::Number)
	ON_SLOT( 15, setSlotHdgKd,    Basic::Number)
	ON_SLOT( 16, setSlotMinPitch, Basic::Number)
	ON_SLOT( 17, setSlotMaxPitch, Basic::Number)
	ON_SLOT( 18, setSlotMaxRoll,  Basic::Number)
	ON_SLOT( 19, setSlotMode,     Basic::String) // modes: "nav", "swarm" (autonomous flight), and "manual"; default = "nav"
END_SLOT_MAP()                                                                        

Eaagles::Basic::Object* SwarmAP::getSlotByIndex(const int si)                         
{                                                                                     
	return BaseClass::getSlotByIndex(si);                                             
}

EMPTY_SERIALIZER(SwarmAP)

//------------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------------
SwarmAP::SwarmAP() 
{
	STANDARD_CONSTRUCTOR()

	rollKp   = 0.0;
	rollKi   = 0.0;
	rollKd   = 0.0;
			
	pitchKp  = 0.0;
	pitchKi  = 0.0;
	pitchKd  = 0.0;
			
	yawKp    = 0.0;
	yawKi    = 0.0;
	yawKd    = 0.0;
			
	altKp    = 0.0;
	altKi    = 0.0;
	altKd    = 0.0;
			
	hdgKp    = 0.0;
	hdgKi    = 0.0;
	hdgKd    = 0.0;

	minPitch = -25.0;
	maxPitch = 25.0;
	maxRoll  = 45.0;

	integRollError = 0.0;
	derivRollError = 0.0;
	rollSetPoint   = 0.0;
	sideStickSP    = 0.0;

	integPitchError = 0.0;
	derivPitchError = 0.0;
	pitchSetPoint   = 0.0;

	integYawError = 0.0;
	derivYawError = 0.0;
	rudderSP      = 0.0;

	integAltError = 0.0;
	derivAltError = 0.0;

	integHdgError = 0.0;
	derivHdgError = 0.0;

	mode = 0;
	setMode(new Basic::String("nav"));
}

//------------------------------------------------------------------------------------
// copyData() - copies one object to another
//------------------------------------------------------------------------------------
void SwarmAP::copyData(const SwarmAP& org, const bool cc) 
{
	BaseClass::copyData(org);

	if(cc) {
		mode = 0;
	}

	rollKp  = org.rollKp;
	rollKi  = org.rollKi;
	rollKd  = org.rollKd;
			
	pitchKp = org.pitchKp;
	pitchKi = org.pitchKi;
	pitchKd = org.pitchKd;
			
	yawKp   = org.yawKp;
	yawKi   = org.yawKi;
	yawKd   = org.yawKd;
			
	altKp   = org.altKp;
	altKi   = org.altKi;
	altKd   = org.altKd;
			
	hdgKp   = org.altKp;
	hdgKi   = org.altKi;
	hdgKd   = org.altKd;

	minPitch = org.minPitch;
	maxPitch = org.maxPitch;
	maxRoll  = org.maxRoll;

	integRollError  = org.integRollError;
	derivRollError  = org.derivRollError;
	rollSetPoint    = org.rollSetPoint;
	sideStickSP     = org.sideStickSP;

	integPitchError = org.integPitchError;
	derivPitchError = org.derivPitchError;
	pitchSetPoint   = org.pitchSetPoint;

	integYawError   = org.integYawError;
	derivYawError   = org.derivYawError;
	rudderSP        = org.rudderSP;

	integAltError   = org.integAltError;
	derivAltError   = org.derivAltError;

	integHdgError   = org.integHdgError;
	derivHdgError   = org.derivHdgError;

	Basic::String* m = 0;
	if(org.mode != 0) m = org.mode->clone();
	mode = m;
	if(m != 0) m->unref();
}

//------------------------------------------------------------------------------------
// deleteData() -- delete this object's data
//------------------------------------------------------------------------------------
void SwarmAP::deleteData()
{
	mode = 0;
}

//------------------------------------------------------------------------------------
// Setter methods
//------------------------------------------------------------------------------------

bool SwarmAP::setRollKp(const double kp)
{
   rollKp = kp;
   return true;
}

bool SwarmAP::setRollKi(const double ki)
{
   rollKi = ki;
   return true;
}

bool SwarmAP::setRollKd(const double kd)
{
   rollKd = kd;
   return true;
}

bool SwarmAP::setPitchKp(const double kp)
{
   pitchKp = kp;
   return true;
}

bool SwarmAP::setPitchKi(const double ki)
{
   pitchKi = ki;
   return true;
}

bool SwarmAP::setPitchKd(const double kd)
{
   pitchKd = kd;
   return true;
}

bool SwarmAP::setYawKp(const double kp)
{
   yawKp = kp;
   return true;
}

bool SwarmAP::setYawKi(const double ki)
{
   yawKi = ki;
   return true;
}

bool SwarmAP::setYawKd(const double kd)
{
   yawKd = kd;
   return true;
}

bool SwarmAP::setAltKp(const double kp)
{
   altKp = kp;
   return true;
}

bool SwarmAP::setAltKi(const double ki)
{
   altKi = ki;
   return true;
}

bool SwarmAP::setAltKd(const double kd)
{
   altKd = kd;
   return true;
}

bool SwarmAP::setHdgKp(const double kp)
{
   hdgKp = kp;
   return true;
}

bool SwarmAP::setHdgKi(const double ki)
{
   hdgKi = ki;
   return true;
}

bool SwarmAP::setHdgKd(const double kd)
{
   hdgKd = kd;
   return true;
}

bool SwarmAP::setMinPitch(const double min)
{
   minPitch = min;
   return true;
}

bool SwarmAP::setMaxPitch(const double max)
{
   maxPitch = max;
   return true;
}

bool SwarmAP::setMaxRoll(const double max)
{
   maxRoll = max;
   return true;
}

bool SwarmAP::setMode(const Basic::String* const m)
{
   mode = m;
   return true;
}

//------------------------------------------------------------------------------------
// Slot methods
//------------------------------------------------------------------------------------

bool SwarmAP::setSlotRollKp(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setRollKp(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotRollKi(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setRollKi(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotRollKd(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setRollKd(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotPitchKp(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setPitchKp(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotPitchKi(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setPitchKi(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotPitchKd(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setPitchKd(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotYawKp(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setYawKp(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotYawKi(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setYawKi(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotYawKd(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setYawKd(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotAltKp(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setAltKp(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotAltKi(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setAltKi(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotAltKd(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setAltKd(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotHdgKp(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setHdgKp(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotHdgKi(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setHdgKi(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotHdgKd(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setHdgKd(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotMinPitch(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setMinPitch(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotMaxPitch(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setMaxPitch(msg->getDouble());
   return ok;
}

bool SwarmAP::setSlotMaxRoll(const Basic::Number* const msg)
{
	bool ok = (msg != 0);
	if (ok) ok = setMaxRoll(msg->getDouble());
	return ok;
}

bool SwarmAP::setSlotMode(const Basic::String* const msg)
{
	bool ok = false;
	if (msg != 0) {
		if( strcmp(msg->getString(), "nav") == 0 || strcmp(msg->getString(), "swarm") == 0) { setMode(msg); }
		else setMode(new Basic::String("manual"));
		ok = true;
	}
	return ok;
}

//------------------------------------------------------------------------------
// Data access function
//------------------------------------------------------------------------------

const char* SwarmAP::getMode() const
{
	const char* p = 0;
	if(mode != 0) p = *mode;
	return p;
}
		
//------------------------------------------------------------------------------
// ELEVATOR control for flight toward destination altitude
//------------------------------------------------------------------------------
double SwarmAP::getFwdStick() {
	Swarms::UAV* uav = dynamic_cast<Swarms::UAV*>(this->getOwnship());
	if(uav == 0) return 0;

	double processValue = uav->getPitchD();
	double error = pitchSetPoint - processValue;

	integPitchError += pitchKi * error;
	double output = pitchKp * error + integPitchError + pitchKd * (error - derivPitchError);
	derivPitchError = error;

	return output;
}

//------------------------------------------------------------------------------
// AILERON control for flight toward destination heading
//------------------------------------------------------------------------------
double SwarmAP::getSideStick() {
	Swarms::UAV* uav = dynamic_cast<Swarms::UAV*>(this->getOwnship());
	if(uav == 0) return 0;

	double diff = rollSetPoint - sideStickSP;
	if(diff > 0.3 || diff < -0.3) {
		if(rollSetPoint < sideStickSP)
			sideStickSP -= 0.2;
		else if(rollSetPoint > sideStickSP)
			sideStickSP += 0.2;
	} else {
		sideStickSP = rollSetPoint;
	}
	
	double processValue = uav->getRollD();
	double error = sideStickSP - processValue;

	integRollError += rollKi * error;

	double output = rollKp * error + integRollError + rollKd * (error - derivRollError);

	derivRollError = error;

	return output;
}

//------------------------------------------------------------------------------
// ALTITUDE control
//------------------------------------------------------------------------------
int c = 0;
void SwarmAP::setPitchSetPoint() {
	const Swarms::UAV* uav = dynamic_cast<Swarms::UAV*>(this->getOwnship());
	if(uav == 0) return;

	// Do we have valid NAV steering data?
	const Eaagles::Simulation::Navigation* nav = uav->getNavigation();
	if(nav == 0 || !nav->isNavSteeringValid()) return;

	// Do we have NAV commanded altitude?
	const Eaagles::Simulation::Route* route = nav->getPriRoute();
	if (route == 0) return;

	// Do we have a Steerpoint
	const Eaagles::Simulation::Steerpoint* sp = route->getSteerpoint();

	if (sp == 0 || !sp->isCmdAltValid()) return;

	double setPoint = sp->getCmdAltitudeFt();

	double processValue = uav->getAltitudeFt();
	double error = setPoint - processValue;

	integAltError += altKi * error;

	pitchSetPoint = altKp * error + integAltError + altKd * (error - derivAltError);

	derivAltError = error;
	if(pitchSetPoint > maxPitch) pitchSetPoint = maxPitch;
	else if(pitchSetPoint < minPitch) pitchSetPoint = minPitch;
}

//------------------------------------------------------------------------------
// HEADING control
//------------------------------------------------------------------------------
void SwarmAP::setRollSetPoint() {
	Swarms::UAV* uav = dynamic_cast<Swarms::UAV*>(this->getOwnship());
	if(uav == 0) return;

	// Do we have valid NAV steering data?
	const Eaagles::Simulation::Navigation* nav = uav->getNavigation();
	if(nav == 0 || !nav->isNavSteeringValid()) return;

	double setPoint = nav->getTrueBrgDeg();

	double processValue = uav->getHeadingD();
	double error = setPoint - processValue;
	if(error < -180) error += 360; // normalizes error between -180 (turn left) to 180 (turn right)

	integHdgError += hdgKi * error;

	rollSetPoint = hdgKp * error + integHdgError + hdgKd * (error - derivHdgError);
	if(rollSetPoint < 2 && rollSetPoint > -2) rollSetPoint = 0;

	if(rollSetPoint > maxRoll) rollSetPoint = maxRoll;
	else if(rollSetPoint < -maxRoll) rollSetPoint = -maxRoll;

	derivHdgError = error;
}

//------------------------------------------------------------------------------
// RUDDER control for flight toward destination heading
//------------------------------------------------------------------------------
double SwarmAP::getRudder() {
	Swarms::UAV* uav = dynamic_cast<Swarms::UAV*>(this->getOwnship());
	if(uav == 0) return 0;
	
	// Do we have valid NAV steering data?
	const Eaagles::Simulation::Navigation* nav = uav->getNavigation();
	if(nav == 0 || !nav->isNavSteeringValid()) return 0;
	double setPoint = nav->getTrueBrgDeg();
	double processValue = uav->getHeadingD();
	double error = setPoint - processValue;
	if(error < -180) error += 360;
	
	if(error > 5 || error < -5) {
		integYawError = 0;
		return 0;
	}

	integYawError += yawKi * error;

	double output = yawKp * error + integYawError + yawKd * (error - derivYawError);

	derivYawError = error;
	return output;
}

//------------------------------------------------------------------------------
// UAV autopilot control
//------------------------------------------------------------------------------
void SwarmAP::flyUav() {
	if(mode == 0) return; // allow manual flight
	
	Swarms::UAV* uav = dynamic_cast<Swarms::UAV*>(this->getOwnship()); // get UAV
	if (uav == 0) return;
	
	Eaagles::Dynamics::JSBSimModel* dm = dynamic_cast<Eaagles::Dynamics::JSBSimModel*>(uav->getDynamicsModel()); // get UAV's dynamics model
	if (dm == 0) return;
	
	// set pitch and roll set points based on commanded altitude and bearing
	setPitchSetPoint();
	setRollSetPoint();

	// provide control inputs
	dm->setControlStickPitchInput(  getFwdStick() );
	dm->setControlStickRollInput( getSideStick() );
	dm->setRudderPedalInput( getRudder() );
}

void SwarmAP::updateData(const LCreal dt)
{
	flyUav();
	BaseClass::updateData(dt);
}

} // end Swarms namespace
} // end Eaagles namespace