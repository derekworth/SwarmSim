#pragma once

//------------------------------------------------------------------------------
// Class: SimOCA
//------------------------------------------------------------------------------
#ifndef __Eaagles_Swarms_SimOCA_H__
#define __Eaagles_Swarms_SimOCA_H__

#include "OnboardControlAgent.h"
#include "openeaagles/basic/units/Distances.h"

using namespace std;

namespace Eaagles {
	namespace Basic { class Number; }
	namespace Simulation { class Steerpoint; class System; }

namespace Swarms {

class SimOCA : public OnboardControlAgent
{
	DECLARE_SUBCLASS(SimOCA, OnboardControlAgent)

public:
	SimOCA();

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

	virtual Simulation::Steerpoint* getWaypoint() const { return wp; }

	virtual void updateData(const LCreal dt = 0.0);
	
	virtual bool setSlotSeparationFactor(const Basic::Number* const msg);
	virtual bool setSlotAlignmentFactor(const Basic::Number* const msg);
	virtual bool setSlotCohesionFactor(const Basic::Number* const msg);
	virtual bool setSlotCommDistance(const Basic::Distance* const msg);
	virtual bool setSlotCommDistance(const Basic::Number* const msg);
	virtual bool setSlotDesiredSeparation(const Basic::Distance* const msg);
	virtual bool setSlotDesiredSeparation(const Basic::Number* const msg);

private:
	double sFactor;
	double aFactor;
	double cFactor;
	double commDist;   // in meters
	double desiredSep; // in meters
	Simulation::Steerpoint* wp;
};

}
}
#endif