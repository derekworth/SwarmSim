//------------------------------------------------------------------------------
// Class: RealBeamRadar
//------------------------------------------------------------------------------
#include "RealBeamRadar.h"

#include "openeaagles/simulation/Antenna.h"
#include "openeaagles/simulation/Player.h"
#include "openeaagles/simulation/Simulation.h"

#include "openeaagles/basic/Color.h"
#include "openeaagles/basic/Rgb.h"
#include "openeaagles/basic/Hsva.h"
#include "openeaagles/basic/Color.h"
#include "openeaagles/basic/Nav.h"
#include "openeaagles/basic/Number.h"
#include "openeaagles/basic/String.h"
#include "openeaagles/basic/Pair.h"
#include "openeaagles/basic/PairStream.h"
#include "openeaagles/basic/Terrain.h"
#include "openeaagles/basic/units/Angles.h"
#include "openeaagles/basic/units/Distances.h"

namespace Eaagles {
namespace Example {

const int IMG_WIDTH = 1024;
const int IMG_HEIGHT = 1024;

IMPLEMENT_SUBCLASS(RealBeamRadar,"RealBeamRadar")
EMPTY_SERIALIZER(RealBeamRadar)

BEGIN_SLOTTABLE(RealBeamRadar)
   "interpolate",    //  1) Interpolate flag (default: false)
END_SLOTTABLE(RealBeamRadar)

BEGIN_SLOT_MAP(RealBeamRadar)
   ON_SLOT( 1, setSlotInterpolate,   Basic::Number)
END_SLOT_MAP()

//------------------------------------------------------------------------------
// Constructors, destructor, copy operator & clone()
//------------------------------------------------------------------------------
RealBeamRadar::RealBeamRadar()
{
   STANDARD_CONSTRUCTOR()

   terrain = nullptr;

   altitude = 15000.0 * Basic::Distance::FT2M;
   antAzAngle = 0.0;
   antElAngle = 0.0;
   ray0 = 0;
   beamWidth = 180.0;
   interpolate = false;
   fpass = true;

   // the image
   image = nullptr;
   imgWidth = 0;
   imgHeight = 0;

   // working storage
   elevations = new LCreal[IMG_WIDTH];
   validFlgs = new bool[IMG_WIDTH];
   aacData = new LCreal[IMG_WIDTH];
   maskFlgs = new bool[IMG_WIDTH];

   // create the image memory
   initImageMemory(IMG_WIDTH, IMG_HEIGHT);
}

//------------------------------------------------------------------------------
// copyData(), deleteData() -- copy (delete) member data
//------------------------------------------------------------------------------
void RealBeamRadar::copyData(const RealBeamRadar& org, const bool cc)
{
   BaseClass::copyData(org);

   if (cc) {
      terrain = nullptr;
      image = nullptr;
      elevations = nullptr;
      validFlgs = nullptr;
      aacData = nullptr;
      maskFlgs = nullptr;
      initImageMemory(IMG_WIDTH, IMG_HEIGHT);
   }

   setTerrain( nullptr );

   altitude = org.altitude;
   antAzAngle = org.antAzAngle;
   antElAngle = org.antElAngle;
   beamWidth = org.beamWidth;

   interpolate = org.interpolate;
   ray0 = 0;
   fpass = true;

   copyImageMemory(org);
}

//------------------------------------------------------------------------------
// deleteData() -- delete member data
//------------------------------------------------------------------------------
void RealBeamRadar::deleteData()
{
   setTerrain( nullptr );
   freeImageMemory();
}


//------------------------------------------------------------------------------
// transmit() -- send radar emissions
//------------------------------------------------------------------------------
void RealBeamRadar::transmit(const LCreal dt)
{
   BaseClass::transmit(dt);

   // Test parameters
   double latitude  = 0;
   double longitude = 0;
   beamWidth = 7.0;

   //
   const Simulation::Player* own = getOwnship();
   if (own != nullptr) {
      // Get our ownship parameters
      altitude = static_cast<LCreal>(own->getAltitude());
      latitude = own->getLatitude();
      longitude = own->getLongitude();

      // Locate the terrain elevation database
      if (terrain == nullptr) {

         const Simulation::Simulation* sim = own->getSimulation();
         if (sim != nullptr) {
            setTerrain( sim->getTerrain() );
         }
      }
   }

   // Transmitting, scanning
   const Simulation::Antenna* ant = getAntenna();
   if (isTransmitting() && ant != nullptr && image != nullptr && terrain != nullptr && terrain->isDataLoaded()) {

      // Compute max range (NM)
      LCreal maxRngNM = getRange();

      // Compute ground range
      LCreal groundRange[IMG_HEIGHT];
      computeGroundRanges(groundRange, IMG_HEIGHT, maxRngNM);

      // Compute slant range
      LCreal slantRange2[IMG_HEIGHT];
      computeSlantRanges2(slantRange2, IMG_HEIGHT, groundRange, altitude);

      // Compute the loss from range
      LCreal rangeLoss[IMG_HEIGHT];
      computeRangeLoss(rangeLoss, IMG_HEIGHT, slantRange2);

      // Compute the earth's curvature effect
      LCreal curvature[IMG_HEIGHT];
      computeEarthCurvature(curvature, IMG_HEIGHT, maxRngNM, static_cast<LCreal>(Basic::Nav::ERAD60));

      LCreal hue = 120.0;      // see Hsv
      LCreal saturation = 0.0; // see Hsv
      const Basic::Hsva* grayTable[19];
      grayTable[0]  = new Basic::Hsva(  hue,  saturation,  0.0f,     1.0f );
      grayTable[1]  = new Basic::Hsva(  hue,  saturation,  0.0872f,  1.0f );
      grayTable[2]  = new Basic::Hsva(  hue,  saturation,  0.1736f,  1.0f );
      grayTable[3]  = new Basic::Hsva(  hue,  saturation,  0.2588f,  1.0f );
      grayTable[4]  = new Basic::Hsva(  hue,  saturation,  0.3420f,  1.0f );
      grayTable[5]  = new Basic::Hsva(  hue,  saturation,  0.4226f,  1.0f );
      grayTable[6]  = new Basic::Hsva(  hue,  saturation,  0.5000f,  1.0f );
      grayTable[7]  = new Basic::Hsva(  hue,  saturation,  0.5736f,  1.0f );
      grayTable[8]  = new Basic::Hsva(  hue,  saturation,  0.6428f,  1.0f );
      grayTable[9]  = new Basic::Hsva(  hue,  saturation,  0.7071f,  1.0f );
      grayTable[10] = new Basic::Hsva(  hue,  saturation,  0.7660f,  1.0f );
      grayTable[11] = new Basic::Hsva(  hue,  saturation,  0.8192f,  1.0f );
      grayTable[12] = new Basic::Hsva(  hue,  saturation,  0.8660f,  1.0f );
      grayTable[13] = new Basic::Hsva(  hue,  saturation,  0.9063f,  1.0f );
      grayTable[14] = new Basic::Hsva(  hue,  saturation,  0.9397f,  1.0f );
      grayTable[15] = new Basic::Hsva(  hue,  saturation,  0.9659f,  1.0f );
      grayTable[16] = new Basic::Hsva(  hue,  saturation,  0.9848f,  1.0f );
      grayTable[17] = new Basic::Hsva(  hue,  saturation,  0.9962f,  1.0f );
      grayTable[18] = new Basic::Hsva(  hue,  saturation,  1.0f,     1.0f );

      // Get antenna look angles
      antAzAngle = static_cast<LCreal>(ant->getAzimuthD());
      antElAngle = static_cast<LCreal>(ant->getElevationD());

      // Which ray are we on?
      LCreal halfRay = static_cast<LCreal>(IMG_WIDTH/2.0f);
      int ray = static_cast<int>(((antAzAngle/45.0f) * halfRay) + halfRay);
      if (ray < 0) ray = 0;
      if (ray > (IMG_WIDTH-1)) ray = (IMG_WIDTH-1);
      if (fpass) { ray0 = ray; fpass = false; }

      // ---
      // For all rays from ray0 to our current ray
      // ---
      int icol = ray0;
      while (icol != ray) {

         for (int irow = 0; irow < IMG_HEIGHT; irow++) {
            elevations[irow] = 0;
            aacData[irow] = 1.0;
            validFlgs[irow] = false;
            maskFlgs[irow] = false;
         }

         // Direction
         int xx = icol - (IMG_WIDTH/2);
         LCreal direction = 45.0 * static_cast<LCreal>(xx) / static_cast<LCreal>(IMG_WIDTH/2);

         // get a strip of elevations from south to north
         unsigned int num = terrain->getElevations(elevations, validFlgs, IMG_HEIGHT, latitude, longitude, direction, groundRange[IMG_HEIGHT-1], interpolate);

         // Apply earth curvature effects to terrain elevations
         for (int irow = 0; irow < IMG_HEIGHT; irow++) {
            elevations[irow] -= curvature[irow];
         }

         // Generate Masks
         Basic::Terrain::vbwShadowChecker(maskFlgs, elevations, validFlgs, IMG_HEIGHT, groundRange[IMG_HEIGHT-1], altitude, antElAngle, beamWidth);

         // Compute AAC data
         Basic::Terrain::aac(aacData, elevations, maskFlgs, IMG_HEIGHT, groundRange[IMG_HEIGHT-1], altitude);

         // Draw a line along the Y points (moving from south to north along the latitude lines)
         for (int irow = 0; irow < IMG_HEIGHT; irow++) {

            LCreal sn = aacData[irow];

            // convert to a color (or gray) value
            osg::Vec3 color(0,0,0);
            if (validFlgs[irow] && !maskFlgs[irow]) {
               Basic::Terrain::getElevationColor(sn, 0.0, 1.0, grayTable, 19, color);
            }

            // store this color
            int idx = irow*imgWidth*PIXEL_SIZE + icol*PIXEL_SIZE;
            image[idx+0] = static_cast<unsigned char>( 255.0 * color[0] );
            image[idx+1] = static_cast<unsigned char>( 255.0 * color[1] );
            image[idx+2] = static_cast<unsigned char>( 255.0 * color[2] );
         }

         if (icol < ray) icol++;
         else if (icol > ray) icol--;
      }

      ray0 = ray;

   }
}

//------------------------------------------------------------------------------
// Compute the ground ranges to each point
//------------------------------------------------------------------------------
bool RealBeamRadar::computeGroundRanges(LCreal* const groundRange, const unsigned int n, const LCreal maxRngNM)
{
   bool ok = false;
   if (groundRange != nullptr && n > 0 && maxRngNM > 0) {

      // Max range (m)
      LCreal maxRng = maxRngNM * Basic::Distance::NM2M;

      // Delta range between points (m)
      LCreal deltaRng = maxRng/static_cast<LCreal>(n);

      // Step of the ground ranges (m)
      LCreal curRng = 0;
      for (unsigned int idx = 0; idx < n; idx++) {
         groundRange[idx] = curRng;
         curRng += deltaRng;
      }

      ok = true;
   }
   return ok;
}

//------------------------------------------------------------------------------
// Compute the square of the slant ranges to each point
//------------------------------------------------------------------------------
bool RealBeamRadar::computeSlantRanges2(LCreal* const slantRange2, const unsigned int n, const LCreal* const gndRng, const LCreal altitude)
{
   bool ok = false;
   if (slantRange2 != nullptr && n > 0 && gndRng != nullptr) {

      // Altitude squared
      LCreal alt2 = altitude * altitude;

      // Slant range squared (m^2) is altitude squared plus ground range squared.
      for (unsigned int idx = 0; idx < n; idx++) {
         slantRange2[idx] = alt2 + gndRng[idx]*gndRng[idx];
      }

      ok = true;
   }
   return ok;
}

//------------------------------------------------------------------------------
// Compute the loss from range
//------------------------------------------------------------------------------
bool RealBeamRadar::computeRangeLoss(LCreal* const rangeLoss, const unsigned int n, const LCreal* const slantRange2)
{
   bool ok = false;
   if (rangeLoss != nullptr && n > 0 && slantRange2 != nullptr) {

      for (unsigned int idx = 0; idx < n; idx++) {
         if (slantRange2[idx] > 0)
            rangeLoss[idx] = 1.0f / ( slantRange2[idx]*slantRange2[idx] );
         else
            rangeLoss[idx] = 1.0;
      }

      ok = true;
   }
   return ok;
}

//------------------------------------------------------------------------------
// Effects of earth curvature
//------------------------------------------------------------------------------
bool RealBeamRadar::computeEarthCurvature(LCreal* const curvature, const unsigned int n, const LCreal maxRngNM, const LCreal radiusNM)
{
   bool ok = false;
   if (curvature != nullptr && n > 0 && maxRngNM > 0 && radiusNM > 0) {

      LCreal radius = radiusNM * Basic::Distance::NM2M;
      LCreal maxRng = maxRngNM * Basic::Distance::NM2M;
      for (unsigned int idx = 0; idx < n; idx++) {
         LCreal curRng = maxRng * static_cast<LCreal>(idx)/static_cast<LCreal>(n);
         LCreal arc = curRng / radius;
         LCreal cs = 1.0;
         LCreal c0 = lcCos(arc);
         if (c0 != 0) cs = 1.0 / c0;
         curvature[idx] = radius * (cs  - 1.0f);
      }

      ok = true;
   }
   return ok;
}

//------------------------------------------------------------------------------
// set functions
//------------------------------------------------------------------------------

bool RealBeamRadar::setTerrain(const Basic::Terrain* const msg)
{
   if (msg != terrain) {
      if (terrain != nullptr) terrain->unref();
      terrain = msg;
      if (terrain != nullptr) terrain->ref();
   }
   return true;
}

//------------------------------------------------------------------------------
// Slot functions
//------------------------------------------------------------------------------

// Set interpolate flag
bool RealBeamRadar::setSlotInterpolate(const Basic::Number* const msg)
{
   bool ok = false;
   if (msg != nullptr) {
      interpolate = msg->getBoolean();
      ok = true;
   }
   return ok;
}

//------------------------------------------------------------------------------
// copy the image memory from another object
//------------------------------------------------------------------------------
bool RealBeamRadar::copyImageMemory(const RealBeamRadar& org)
{
   // First free our old memory
   freeImageMemory();

   // Now allocate the new memory (if needed)
   bool ok = initImageMemory(org.imgWidth, org.imgHeight);
   if (ok) {

      // and copy the data
      for (int irow = 0; irow < imgHeight; irow++) {

         elevations[irow] = org.elevations[irow];
         validFlgs[irow] = org.validFlgs[irow];
         aacData[irow] = org.aacData[irow];
         maskFlgs[irow] = org.maskFlgs[irow];

         for (int icol = 0; icol < imgWidth; icol++) {
            int idx = (irow * imgWidth * PIXEL_SIZE) + (icol * PIXEL_SIZE);
            image[idx+0] = org.image[idx+0];
            image[idx+1] = org.image[idx+1];
            image[idx+2] = org.image[idx+2];
         }
      }
   }
   return true;
}

//------------------------------------------------------------------------------
// allocate and init image memory
//------------------------------------------------------------------------------
bool RealBeamRadar::initImageMemory(const int width, const int height)
{
   bool ok = false;
   if (width > 0  && width <= MAX_IMAGE_WIDTH &&
      height > 0 && height <= MAX_IMAGE_HEIGHT) {

      // allocate space for the image
      unsigned char* tmpImage = new unsigned char[width * height * PIXEL_SIZE];
      if (tmpImage != nullptr) {

         // clear the memory
         unsigned char* p = tmpImage;
         unsigned char* q = (tmpImage + width*height * PIXEL_SIZE);
//         unsigned int n = width*height*PIXEL_SIZE;
         while (p < q) {
            *p++ = 0;
         }

         // set our member variables
         image = tmpImage;
         imgWidth = width;
         imgHeight = height;

         // allocate the working storage
         elevations = new LCreal[height];
         validFlgs = new bool[height];
         aacData = new LCreal[height];
         maskFlgs = new bool[height];

         ok = true;

      }
   }
   return ok;
}

//------------------------------------------------------------------------------
// free the image memory
//------------------------------------------------------------------------------
void RealBeamRadar::freeImageMemory()
{
   // temp pointer
   unsigned char* tmpImage = image;

   // Clear the member variables
   image = nullptr;
   imgWidth = 0;
   imgHeight = 0;

   // Free the memory
   if (tmpImage != nullptr)   { delete[] tmpImage; }
   if (elevations != nullptr) { delete[] elevations; elevations = nullptr; }
   if (validFlgs != nullptr)  { delete[] validFlgs;  validFlgs = nullptr; }
   if (aacData != nullptr)    { delete[] aacData;    aacData = nullptr; }
   if (maskFlgs != nullptr)   { delete[] maskFlgs;   maskFlgs = nullptr; }
}

//------------------------------------------------------------------------------
// getSlotByIndex()
//------------------------------------------------------------------------------
Basic::Object* RealBeamRadar::getSlotByIndex(const int si)
{
    return BaseClass::getSlotByIndex(si);
}


} // end Example namespace
} // end Eaagles namespace

