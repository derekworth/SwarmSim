#include "SimAP.h"
#include "OnboardControlAgent.h"
#include "UAV.h"

#include "openeaagles/simulation/Navigation.h"
#include "openeaagles/simulation/Route.h"
#include "openeaagles/simulation/Steerpoint.h"
#include "openeaagles/dynamics/JSBSimModel.h"

namespace Eaagles {
namespace Swarms {

// =============================================================================
// class: SimAP
// =============================================================================

IMPLEMENT_SUBCLASS(SimAP,"SimAP")

BEGIN_SLOTTABLE(SimAP)
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
END_SLOTTABLE(SimAP)

BEGIN_SLOT_MAP(SimAP)
	ON_SLOT(  1, setSlotRollKp,   Basic::Number) // default = 0.05
	ON_SLOT(  2, setSlotRollKi,   Basic::Number) // default = 0.002
	ON_SLOT(  3, setSlotRollKd,   Basic::Number) // default = 0.001
	ON_SLOT(  4, setSlotPitchKp,  Basic::Number) // default = 0.05
	ON_SLOT(  5, setSlotPitchKi,  Basic::Number) // default = 0.002
	ON_SLOT(  6, setSlotPitchKd,  Basic::Number) // default = 0.001
	ON_SLOT(  7, setSlotYawKp,    Basic::Number) // default = 0.02
	ON_SLOT(  8, setSlotYawKi,    Basic::Number) // default = 0.001
	ON_SLOT(  9, setSlotYawKd,    Basic::Number) // default = 0.0005
	ON_SLOT( 10, setSlotAltKp,    Basic::Number) // default = 0.05
	ON_SLOT( 11, setSlotAltKi,    Basic::Number) // default = 0.0
	ON_SLOT( 12, setSlotAltKd,    Basic::Number) // default = 0.001
	ON_SLOT( 13, setSlotHdgKp,    Basic::Number) // default = 0.7
	ON_SLOT( 14, setSlotHdgKi,    Basic::Number) // default = 0.0
	ON_SLOT( 15, setSlotHdgKd,    Basic::Number) // default = 0.01
	ON_SLOT( 16, setSlotMinPitch, Basic::Number) // default = -10
	ON_SLOT( 17, setSlotMaxPitch, Basic::Number) // default = 15
	ON_SLOT( 18, setSlotMaxRoll,  Basic::Number) // default = 30
	ON_SLOT( 19, setSlotMode,     Basic::String) // modes: "nav", "swarm" (autonomous flight), and "manual"; default = "nav"
END_SLOT_MAP()                                                                        

Eaagles::Basic::Object* SimAP::getSlotByIndex(const int si)                         
{                                                                                     
	return BaseClass::getSlotByIndex(si);                                             
}

EMPTY_SERIALIZER(SimAP)

//------------------------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------------------------
SimAP::SimAP() 
{
	STANDARD_CONSTRUCTOR()

	rollKp   = 0.05;
	rollKi   = 0.002;
	rollKd   = 0.001;
			
	pitchKp  = 0.05;
	pitchKi  = 0.002;
	pitchKd  = 0.001;
			
	yawKp    = 0.02;
	yawKi    = 0.001;
	yawKd    = 0.0005;
			
	altKp    = 0.05;
	altKi    = 0.0;
	altKd    = 0.001;
			
	hdgKp    = 0.7;
	hdgKi    = 0.0;
	hdgKd    = 0.01;

	minPitch = -10.0;
	maxPitch = 15.0;
	maxRoll  = 30.0;

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

	mode = nullptr;
	setMode(new Basic::String("nav"));

	uav   = nullptr;
	nav   = nullptr;
	route = nullptr;
	wp    = nullptr;
	fdm   = nullptr;
}

//------------------------------------------------------------------------------------
// copyData() - copies one object to another
//------------------------------------------------------------------------------------
void SimAP::copyData(const SimAP& org, const bool cc) 
{
	BaseClass::copyData(org);

	if (cc) {
		mode = nullptr;
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

	Basic::String* m = nullptr;
	if(org.mode != nullptr) m = org.mode->clone();
	mode = m;
	if (m != nullptr) m->unref();

	uav   = nullptr;
	nav   = nullptr;
	route = nullptr;
	wp    = nullptr;
	fdm   = nullptr;
}

//------------------------------------------------------------------------------------
// deleteData() -- delete this object's data
//------------------------------------------------------------------------------------
void SimAP::deleteData()
{
	if (wp != nullptr) {
		// remove waypoint from player's navigation route (if exists)
		route->deleteSteerpoint(wp);
		// delete waypoint
		wp->unref();
		route->unref();
		nav->unref();
		uav->unref();
		fdm->unref();
	}
	mode = nullptr;
}

//------------------------------------------------------------------------------------
// Setter methods
//------------------------------------------------------------------------------------

bool SimAP::setRollKp(const double kp)
{
   rollKp = kp;
   return true;
}

bool SimAP::setRollKi(const double ki)
{
   rollKi = ki;
   return true;
}

bool SimAP::setRollKd(const double kd)
{
   rollKd = kd;
   return true;
}

bool SimAP::setPitchKp(const double kp)
{
   pitchKp = kp;
   return true;
}

bool SimAP::setPitchKi(const double ki)
{
   pitchKi = ki;
   return true;
}

bool SimAP::setPitchKd(const double kd)
{
   pitchKd = kd;
   return true;
}

bool SimAP::setYawKp(const double kp)
{
   yawKp = kp;
   return true;
}

bool SimAP::setYawKi(const double ki)
{
   yawKi = ki;
   return true;
}

bool SimAP::setYawKd(const double kd)
{
   yawKd = kd;
   return true;
}

bool SimAP::setAltKp(const double kp)
{
   altKp = kp;
   return true;
}

bool SimAP::setAltKi(const double ki)
{
   altKi = ki;
   return true;
}

bool SimAP::setAltKd(const double kd)
{
   altKd = kd;
   return true;
}

bool SimAP::setHdgKp(const double kp)
{
   hdgKp = kp;
   return true;
}

bool SimAP::setHdgKi(const double ki)
{
   hdgKi = ki;
   return true;
}

bool SimAP::setHdgKd(const double kd)
{
   hdgKd = kd;
   return true;
}

bool SimAP::setMinPitch(const double min)
{
   minPitch = min;
   return true;
}

bool SimAP::setMaxPitch(const double max)
{
   maxPitch = max;
   return true;
}

bool SimAP::setMaxRoll(const double max)
{
   maxRoll = max;
   return true;
}

void SimAP::setMode(const Basic::String* const m)
{
   mode = m;
}

//------------------------------------------------------------------------------------
// Slot methods
//------------------------------------------------------------------------------------

bool SimAP::setSlotRollKp(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setRollKp(msg->getDouble());
   return ok;
}

bool SimAP::setSlotRollKi(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setRollKi(msg->getDouble());
   return ok;
}

bool SimAP::setSlotRollKd(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setRollKd(msg->getDouble());
   return ok;
}

bool SimAP::setSlotPitchKp(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setPitchKp(msg->getDouble());
   return ok;
}

bool SimAP::setSlotPitchKi(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setPitchKi(msg->getDouble());
   return ok;
}

bool SimAP::setSlotPitchKd(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setPitchKd(msg->getDouble());
   return ok;
}

bool SimAP::setSlotYawKp(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setYawKp(msg->getDouble());
   return ok;
}

bool SimAP::setSlotYawKi(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setYawKi(msg->getDouble());
   return ok;
}

bool SimAP::setSlotYawKd(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setYawKd(msg->getDouble());
   return ok;
}

bool SimAP::setSlotAltKp(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setAltKp(msg->getDouble());
   return ok;
}

bool SimAP::setSlotAltKi(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setAltKi(msg->getDouble());
   return ok;
}

bool SimAP::setSlotAltKd(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setAltKd(msg->getDouble());
   return ok;
}

bool SimAP::setSlotHdgKp(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setHdgKp(msg->getDouble());
   return ok;
}

bool SimAP::setSlotHdgKi(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setHdgKi(msg->getDouble());
   return ok;
}

bool SimAP::setSlotHdgKd(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setHdgKd(msg->getDouble());
   return ok;
}

bool SimAP::setSlotMinPitch(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setMinPitch(msg->getDouble());
   return ok;
}

bool SimAP::setSlotMaxPitch(const Basic::Number* const msg)
{
   bool ok = (msg != 0);
   if (ok) ok = setMaxPitch(msg->getDouble());
   return ok;
}

bool SimAP::setSlotMaxRoll(const Basic::Number* const msg)
{
	bool ok = (msg != 0);
	if (ok) ok = setMaxRoll(msg->getDouble());
	return ok;
}

bool SimAP::setSlotMode(const Basic::String* const msg)
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

const char* SimAP::getMode() const
{
	const char* p = 0;
	if(mode != 0) p = *mode;
	return p;
}
		
//------------------------------------------------------------------------------
// ELEVATOR control for flight toward destination altitude
//------------------------------------------------------------------------------

double SimAP::getFwdStick() {
	if (uav == nullptr) {
		uav = dynamic_cast<Swarms::UAV*>(this->getOwnship());
		if (uav == nullptr) return 0;
	}

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

double SimAP::getSideStick() {
	if (uav == nullptr) {
		uav = dynamic_cast<Swarms::UAV*>(this->getOwnship());
		if (uav == nullptr) return 0;
	}

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

void SimAP::setPitchSetPoint() {
	if (uav == nullptr) {
		uav = dynamic_cast<Swarms::UAV*>(this->getOwnship());
		if (uav == nullptr) return;
	}

	// Do we have valid NAV steering data?
	if (nav == nullptr) {
		nav = uav->getNavigation();
		if (nav == nullptr || !nav->isNavSteeringValid()) return;
	}

	// Do we have a route?
	if (route == nullptr) {
		route = nav->getPriRoute();
		if (route == 0) return;
	}

	// Do we have a Steerpoint
	const Eaagles::Simulation::Steerpoint* sp = route->getSteerpoint();
	if (sp == nullptr || !sp->isCmdAltValid()) return;


	double setPoint = sp->getCmdAltitudeFt();   // desired alt
	double processValue = uav->getAltitudeFt(); // current alt
	double error = setPoint - processValue;     // diff

	integAltError += altKi * error;
	pitchSetPoint = altKp * error + integAltError + altKd * (error - derivAltError);
	derivAltError = error;
	if (pitchSetPoint > maxPitch) pitchSetPoint = maxPitch;
	else if (pitchSetPoint < minPitch) pitchSetPoint = minPitch;
}

//------------------------------------------------------------------------------
// HEADING control
//------------------------------------------------------------------------------

void SimAP::setRollSetPoint() {
	if (uav == nullptr) {
		uav = dynamic_cast<Swarms::UAV*>(this->getOwnship());
		if (uav == nullptr) return;
	}

	// Do we have valid NAV steering data?
	if (nav == nullptr) {
		nav = uav->getNavigation();
		if (nav == nullptr || !nav->isNavSteeringValid()) return;
	}

	double setPoint = nav->getTrueBrgDeg();	  // desired hdg
	double processValue = uav->getHeadingD(); // current hdg
	double error = setPoint - processValue;	  // diff

	if (error < -180) error += 360; // normalizes error between -180 (turn left) to 180 (turn right)

	integHdgError += hdgKi * error;

	rollSetPoint = hdgKp * error + integHdgError + hdgKd * (error - derivHdgError);
	if (rollSetPoint < 2 && rollSetPoint > -2) rollSetPoint = 0;

	if (rollSetPoint > maxRoll) rollSetPoint = maxRoll;
	else if (rollSetPoint < -maxRoll) rollSetPoint = -maxRoll;

	derivHdgError = error;
}

//------------------------------------------------------------------------------
// RUDDER control for flight toward destination heading
//------------------------------------------------------------------------------

double SimAP::getRudder() {
	if (uav == nullptr) {
		uav = dynamic_cast<Swarms::UAV*>(this->getOwnship());
		if (uav == nullptr) return 0;
	}

	// Do we have valid NAV steering data?
	if (nav == nullptr) {
		nav = uav->getNavigation();
		if (nav == nullptr || !nav->isNavSteeringValid()) return 0;
	}

	double setPoint = nav->getTrueBrgDeg();	  // desired hdg
	double processValue = uav->getHeadingD(); // current hdg
	double error = setPoint - processValue;	  // diff

	if (error < -180) error += 360;

	if (error > 5 || error < -5) {
		integYawError = 0;
		return 0;
	}

	integYawError += yawKi * error;

	double output = yawKp * error + integYawError + yawKd * (error - derivYawError);

	derivYawError = error;
	return output;
}

//------------------------------------------------------------------------------
// UAV autopilot control - used by OCA for Dynamic Waypoint Following (DWF)
//------------------------------------------------------------------------------

void SimAP::setWaypoint(const osg::Vec3& pos, const LCreal altM) {
	// do nothing if NOT in swarming mode (i.e. Dynamic Waypoint Following not applicable)
	if (strcmp(*mode, "swarm") != 0) return;

	if (wp == nullptr) { // add waypoint if not present
		if (route == nullptr) { // add route if not present
			if (nav == nullptr) { // add navigation system if not present
				if (uav == nullptr) { // add uav if not present
					uav = dynamic_cast<Swarms::UAV*>(getOwnship());
					if (uav == nullptr) return; // do nothing if ownship is not a UAV
				}
				nav = uav->getNavigation();
				if (nav == nullptr) { // add new instance since UAV does not already have a nav system
					nav = new Simulation::Navigation();
					Basic::Pair* navPair = new Basic::Pair("Navigation", nav);
					uav->addComponent(navPair);
				}
			}
			route = nav->getPriRoute();
			if (route == nullptr) { // add new instance since nav system does not already have a primary route
				route = new Simulation::Route();
				nav->setRoute(route);
			}
		}
		wp = new Simulation::Steerpoint();
		route->insertSteerpoint(wp);
	}
	// set Dynamic Waypoint (DW)
	wp->setPosition(pos);     // Lat/Lon
	wp->setCmdAltitude(altM); // Alt
	route->directTo(wp);      // Fly to DW
}

//------------------------------------------------------------------------------
// dynamics() is called by updateTC() and therefore a time-critical method.
// Autopilot updates are time-critical because control inputs must sync with
// the FDM (i.e. JSBSim) which is also a time-critical process.
//------------------------------------------------------------------------------

void SimAP::dynamics(const LCreal dt) {
	if (mode == 0) return; // allow manual flight

	uav = dynamic_cast<Swarms::UAV*>(getOwnship()); // get UAV
	if (uav == 0) return;

	if (fdm == nullptr) {
		fdm = dynamic_cast<Eaagles::Dynamics::JSBSimModel*>(uav->getDynamicsModel()); // get UAV's flight dynamics model
		if (fdm == nullptr) return;
	}

	// set pitch and roll set points based on commanded altitude and bearing
	setPitchSetPoint();
	setRollSetPoint();

	// provide control inputs
	fdm->setControlStickPitchInput(getFwdStick());
	fdm->setControlStickRollInput(getSideStick());
	fdm->setRudderPedalInput(getRudder());
	fdm->setThrottles(&throttle, 1);

	BaseClass::dynamics(dt);
}

} // end Swarms namespace
} // end Eaagles namespace