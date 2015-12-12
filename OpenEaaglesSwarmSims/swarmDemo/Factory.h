//------------------------------------------------------------------------------
// Class: Factory
//
// Description: Class factory
//------------------------------------------------------------------------------
#ifndef __Eaagles_Swarms_Factory_H__
#define __Eaagles_Swarms_Factory_H__

namespace Eaagles {

	namespace Basic { class Object; }

	namespace Swarms {

		class Factory
		{
		public:
			static Basic::Object* createObj(const char* name);

		protected:
			Factory();   // prevent object creation
		};

	}  // end namespace Example
}  // end namespace Eaagles

#endif
