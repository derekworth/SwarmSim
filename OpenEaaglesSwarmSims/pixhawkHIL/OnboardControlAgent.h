#pragma once

//------------------------------------------------------------------------------
// Class: OnboardControlAgent
//------------------------------------------------------------------------------
#ifndef __Eaagles_Swarms_OnboardControlAgent_H__
#define __Eaagles_Swarms_OnboardControlAgent_H__

#include "openeaagles/simulation/System.h"
#include "openeaagles/basic/units/Distances.h"

using namespace std;

namespace Eaagles {
	namespace Basic { class Number; }
	namespace Simulation { class System; }

namespace Swarms {

class OnboardControlAgent : public Simulation::System
{
	DECLARE_SUBCLASS(OnboardControlAgent, Simulation::System)

public:
	OnboardControlAgent();

	virtual Eaagles::osg::Vec3d getSeparationVector();
	virtual Eaagles::osg::Vec3d getAlignmentVector();
	virtual Eaagles::osg::Vec3d getCohesionVector();
	
	virtual double getSeparationFactor() const { return sFactor; }
	virtual double getAlignmentFactor() const { return aFactor; }
	virtual double getCohesionFactor() const { return cFactor; }
	virtual double getCommDistance() const { return commDist; }
	virtual double getDesiredSeparation() const { return desiredSep; }
	
	virtual double getDistance(Eaagles::osg::Vec3d v1, Eaagles::osg::Vec3d v2) { return sqrt(pow(v1.x()-v2.x(),2) + pow(v1.y()-v2.y(),2) + pow(v1.z()-v2.z(),2)); }
	
	virtual bool setSeparationFactor(const double factr);
	virtual bool setAlignmentFactor(const double factr);
	virtual bool setCohesionFactor(const double factr);
	virtual bool setCommDistance(const double dist);     // dist = distance (in meters)
	virtual bool setDesiredSeparation(const double sep); // sep = separation (in meters)

	virtual void updateData(const LCreal dt = 0.0);
	
	virtual bool setSlotSeparationFactor(const Basic::Number* const msg);
	virtual bool setSlotAlignmentFactor(const Basic::Number* const msg);
	virtual bool setSlotCohesionFactor(const Basic::Number* const msg);
	virtual bool setSlotCommDistance(const Basic::Distance* const msg);
	virtual bool setSlotCommDistance(const Basic::Number* const msg);
	virtual bool setSlotDesiredSeparation(const Basic::Distance* const msg);
	virtual bool setSlotDesiredSeparation(const Basic::Number* const msg);

private:
	double startTime;
	double sFactor;
	double aFactor;
	double cFactor;
	double commDist;   // in meters
	double desiredSep; // in meters
};

}
}
#endif