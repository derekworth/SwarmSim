#pragma once

//------------------------------------------------------------------------------
// Class: SimAP
//   Provides a simulated autopilot designed to fly a small fixed-wing UAV
//   with traditional control surfaces (ailerions, rudder, and elevator).
//------------------------------------------------------------------------------
#ifndef __Eaagles_Swarms_SimAP_H__
#define __Eaagles_Swarms_SimAP_H__

#include "SwarmAutopilot.h"

namespace Eaagles {
	namespace Basic { class Number; }
	namespace Simulation {
		class Navigation;
		class Route; 
		class Steerpoint; 
		class Vehicle;
	}
	namespace Dynamics { class JSBSimModel; }
	namespace Swarms { class UAV; }

namespace Swarms {

class SimAP : public SwarmAutopilot
{
	DECLARE_SUBCLASS(SimAP, SwarmAutopilot)

public:
	SimAP();
	
	virtual double getRollKp() const { return rollKp; }
	virtual double getRollKi() const { return rollKi; }
	virtual double getRollKd() const { return rollKd; }
	
	virtual double getPitchKp() const { return pitchKp; }
	virtual double getPitchKi() const { return pitchKi; }
	virtual double getPitchKd() const { return pitchKd; }
	
	virtual double getYawKp() const { return yawKp; }
	virtual double getYawKi() const { return yawKi; }
	virtual double getYawKd() const { return yawKd; }
	
	virtual double getAltKp() const { return altKp; }
	virtual double getAltKi() const { return altKi; }
	virtual double getAltKd() const { return altKd; }
	
	virtual double getHdgKp() const { return hdgKp; }
	virtual double getHdgKi() const { return hdgKi; }
	virtual double getHdgKd() const { return hdgKd; }
	
	virtual double getMinPitch() const { return minPitch; }
	virtual double getMaxPitch() const { return maxPitch; }
	virtual double getMaxRoll()  const { return maxRoll; }
	
	virtual const char* getMode() const;
	
	virtual bool setRollKp(const double kp);
	virtual bool setRollKi(const double ki);
	virtual bool setRollKd(const double kd);
	
	virtual bool setPitchKp(const double kp);
	virtual bool setPitchKi(const double ki);
	virtual bool setPitchKd(const double kd);
	
	virtual bool setYawKp(const double kp);
	virtual bool setYawKi(const double ki);
	virtual bool setYawKd(const double kd);
	
	virtual bool setAltKp(const double kp);
	virtual bool setAltKi(const double ki);
	virtual bool setAltKd(const double kd);
	
	virtual bool setHdgKp(const double kp);
	virtual bool setHdgKi(const double ki);
	virtual bool setHdgKd(const double kd);
	
	virtual bool setMinPitch(const double min);
	virtual bool setMaxPitch(const double max);
	virtual bool setMaxRoll(const double max);
	
	virtual void setMode(const Basic::String* const m);
	virtual void setWaypoint(const osg::Vec3& pos, const LCreal altM);

	virtual void dynamics(const LCreal dt) override;

protected:
	bool setSlotRollKp(const Basic::Number* const msg);
	bool setSlotRollKi(const Basic::Number* const msg);
	bool setSlotRollKd(const Basic::Number* const msg);

	bool setSlotPitchKp(const Basic::Number* const msg);
	bool setSlotPitchKi(const Basic::Number* const msg);
	bool setSlotPitchKd(const Basic::Number* const msg);

	bool setSlotYawKp(const Basic::Number* const msg);
	bool setSlotYawKi(const Basic::Number* const msg);
	bool setSlotYawKd(const Basic::Number* const msg);

	bool setSlotAltKp(const Basic::Number* const msg);
	bool setSlotAltKi(const Basic::Number* const msg);
	bool setSlotAltKd(const Basic::Number* const msg);

	bool setSlotHdgKp(const Basic::Number* const msg);
	bool setSlotHdgKi(const Basic::Number* const msg);
	bool setSlotHdgKd(const Basic::Number* const msg);

	bool setSlotMinPitch(const Basic::Number* const msg);
	bool setSlotMaxPitch(const Basic::Number* const msg);
	bool setSlotMaxRoll(const Basic::Number* const msg);

	bool setSlotMode(const Basic::String* const msg);
	
	bool getUav();
	
	double getFwdStick();
	double getSideStick();
	double getRudder();
	void setPitchSetPoint();
	void setRollSetPoint();
	
	void flyUav();

private:
	double rollKp;
	double rollKi;
	double rollKd;
	
	double pitchKp;
	double pitchKi;
	double pitchKd;
	
	double yawKp;
	double yawKi;
	double yawKd;
	
	double altKp;
	double altKi;
	double altKd;
	
	double hdgKp;
	double hdgKi;
	double hdgKd;
	
	double minPitch;
	double maxPitch;
	double maxRoll;

	double integRollError;
	double derivRollError;
	double rollSetPoint;
	double sideStickSP;

	double integPitchError;
	double derivPitchError;
	double pitchSetPoint;

	double integYawError;
	double derivYawError;
	double rudderSP;

	double integAltError;
	double derivAltError;

	double integHdgError;
	double derivHdgError;

	double throttle = 1.0;

	Eaagles::Basic::safe_ptr<const Basic::String> mode;

	Swarms::UAV* uav;
	Simulation::Navigation* nav;
	Simulation::Route* route;
	Simulation::Steerpoint* wp;
	Dynamics::JSBSimModel* fdm;
};

}
}
#endif