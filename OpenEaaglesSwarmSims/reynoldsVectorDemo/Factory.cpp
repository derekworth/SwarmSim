//------------------------------------------------------------------------------
// Class: Factory
//------------------------------------------------------------------------------
#include "Factory.h"

#include "openeaagles/basic/Object.h"

#include "UAV.h"
#include "SwarmAP.h"
#include "OnboardControlAgent.h"

// class factories
#include "openeaagles/basic/Factory.h"
#include "openeaagles/dis/Factory.h"
#include "openeaagles/instruments/Factory.h"
#include "openeaagles/ioDevice/Factory.h"
#include "openeaagles/sensors/Factory.h"
#include "openeaagles/simulation/Factory.h"
#include "openeaagles/dynamics/Factory.h"
#include "openeaagles/ioDevice/Factory.h"

#include <cstring>

namespace Eaagles {
	namespace Swarms {

		Factory::Factory() {}

		Basic::Object* Factory::createObj(const char* name)
		{
			Basic::Object* obj = 0;
			
			if ( std::strcmp(name, UAV::getFactoryName()) == 0 ) {
				obj = new UAV();
			}
			if ( std::strcmp(name, SwarmAP::getFactoryName()) == 0 ) {
				obj = new SwarmAP();
			}
			if ( std::strcmp(name, OnboardControlAgent::getFactoryName()) == 0 ) {
				obj = new OnboardControlAgent();
			}

			// Framework libraries
			if (obj == 0) obj = Simulation::Factory::createObj(name);			  
			if (obj == 0) obj = Sensor::Factory::createObj(name);				  
			if (obj == 0) obj = Network::Dis::Factory::createObj(name);			  
			if (obj == 0) obj = Dynamics::Factory::createObj(name);				  
			if (obj == 0) obj = IoDevice::Factory::createObj(name);					  
			if (obj == 0) obj = Basic::Factory::createObj(name);

			return obj;
		}

	}  // end namespace Example
}  // end namespace Eaagles