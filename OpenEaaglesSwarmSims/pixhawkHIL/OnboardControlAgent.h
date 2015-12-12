#pragma once

//------------------------------------------------------------------------------
// Class: OnboardControlAgent
//------------------------------------------------------------------------------
#ifndef __Eaagles_Swarms_OnboardControlAgent_H__
#define __Eaagles_Swarms_OnboardControlAgent_H__

#include "openeaagles/simulation/System.h"

namespace Eaagles {
namespace Swarms {

class OnboardControlAgent : public Simulation::System
{
	DECLARE_SUBCLASS(OnboardControlAgent, Simulation::System)

public:
	OnboardControlAgent();
};

} // End Swarms namespace
} // End Eaagles namespace
#endif