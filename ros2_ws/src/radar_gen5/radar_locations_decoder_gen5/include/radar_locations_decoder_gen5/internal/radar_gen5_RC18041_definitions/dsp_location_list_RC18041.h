/******************************************************************************/
/*! \file  dsp_location_list.h

\brief Header for Location List

\author CC-DA/ECR3 Knut Schumacher and Oliver Perkuhn

*/
/******************************************************************************/
#ifndef DSP_LOCATION_LIST_H_INCLUDED
#define DSP_LOCATION_LIST_H_INCLUDED

#ifdef _MSC_VER
#pragma warning(push,0)
#endif
#include <vfc/core/vfc_types.hpp>
#include <vfc/container/vfc_fifo.hpp>
#include <vfc/core/vfc_float16_storage.hpp>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

namespace RC18041
{

#define MAXNUMLOCATIONS 256

// Norms
#define DSP_LOC_QUALI_NORM (vfc::uint8_t)255
#define dspLocQualiDeNorm(x)    x*0.003921568627451f  // 1/255
#define dspLocDegToRad(x)       x*0.017453292519943f  // pi/180
#define dspLocDegVarToRadVar(x) x*(0.017453292519943f*0.017453292519943f) // (pi/180)^2

//! bit position of extended measured status of location
enum LocMeasStatus
{
   LOC_MEASURED=0,
   LOC_MULTITARGETAZI,
   LOC_MULTITARGETELEV,
   LOC_STANDING
};

/**
 * \brief external location items structure
 *       Norm: []
 * \verbatim
 *       phys. unit: -
 * \endverbatim
*/
struct DSP_LOCATION_ITEM_ST
{

public:
   // Constructor
   DSP_LOCATION_ITEM_ST() : d(0.0f), dVar(0.0f), v(0.0f), vVar(0.0f), dvCov(0.0f), dvQly(0),
      theta(0.0f), phi(0.0f), thetaQly(0), phiQly(0), thetaVar(0.0f), phiVar(0.0f),
      RCS(0.0f), RSSI(0.0f), dSprd(0.0f), vSprd(0.0f), dvSprdOrntn(0.0f), measStatus(0)
   { };

   /* Access Functions */
   /*! \brief getter function for radial distance measured, phys. unit: m */
   vfc::float32_t getRadialDistance()                 const { return d; };
   /*! \brief getter function for radial distance variance measured, phys. unit: m^2 */
   vfc::float32_t getRadialDistanceVar()              const { return (vfc::float32_t) dVar; };
   /*! \brief getter function for radial relative velocity measured, phys. unit: m/s */
   vfc::float32_t getRelativeRadialVelocity()         const { return v; };
   /*! \brief getter function for radial relative velocity variance measured, phys. unit: (m/s)^2 */
   vfc::float32_t getRelativeRadialVelocityVar()      const { return (vfc::float32_t) vVar; };
   /*! \brief getter function for covariance of radial distance and velocity measured, phys. unit: m^2/s */
   vfc::float32_t getRadialDistanceVelocityCov()      const { return (vfc::float32_t) dvCov; };
   /*! \brief getter function for quality of velocity resolution processing, value range [0 1] */
   vfc::float32_t getRadialDistanceVelocityQuality()  const { return (vfc::float32_t) dspLocQualiDeNorm(dvQly); };


   /*! \brief getter function for azimuth angles, phys. unit: rad */
   vfc::float32_t getAzimuthAngle()                   const { return (vfc::float32_t) dspLocDegToRad(theta); };
   /*! \brief getter function for azimuth angles, phys. unit: deg */
   vfc::float32_t getAzimuthAngleDeg()                const { return (vfc::float32_t) theta; };
   /*! \brief getter function for variance of azimuth angle, phys. unit: rad^2 */
   vfc::float32_t getAzimuthAngleVar()                const { return (vfc::float32_t) dspLocDegVarToRadVar(thetaVar); };
   /*! \brief getter function for variance of azimuth angle, phys. unit: deg^2 */
   vfc::float32_t getAzimuthAngleVarDeg()             const { return (vfc::float32_t) thetaVar; };
   /*! \brief getter function for quality of probability for correct signal model (azimuth angle), value range [0 1] */
   vfc::float32_t getAzimuthAngleQuality()            const { return (vfc::float32_t) dspLocQualiDeNorm(thetaQly); };
   /*! \brief getter function for elevation angles, unit: rad */
   vfc::float32_t getElevationAngle()                 const { return (vfc::float32_t) dspLocDegToRad(phi); };
   /*! \brief getter function for elevation angles, unit: deg */
   vfc::float32_t getElevationAngleDeg()              const { return (vfc::float32_t) phi; };
   /*! \brief getter function for variance of elevation angle, phys. unit: rad^2 */
   vfc::float32_t getElevationAngleVar()              const { return (vfc::float32_t) dspLocDegVarToRadVar(phiVar); };
   /*! \brief getter function for variance of elevation angle, phys. unit: deg^2 */
   vfc::float32_t getElevationAngleVarDeg()           const { return (vfc::float32_t) phiVar; };
   /*! \brief getter function for quality of probability for correct signal model (elevation angle), value range [0 1] */
   vfc::float32_t getElevationAngleQuality()          const { return (vfc::float32_t) dspLocQualiDeNorm(phiQly); };

   /*! \brief getter function for RCS estimation */
   vfc::float32_t getRcs()                            const { return (vfc::float32_t) RCS; };
   /*! \brief getter function for Received Signal Strength Indication (RSSI) */
   vfc::float32_t getRssi()                           const { return (vfc::float32_t) RSSI; };
   /*! \brief getter function for spectral spread in distance */
   vfc::float32_t getRadialDistanceSpread()               const { return (vfc::float32_t) dSprd; };
   /*! \brief getter function for spectral spread in velocity */
   vfc::float32_t getRelativeRadialVelocitySpread()       const { return (vfc::float32_t) vSprd; };
   /*! \brief getter function for orientation of spectral spread in distance and velocity  */
   vfc::float32_t getDistanceVelocitySpreadOrientation()  const { return (vfc::float32_t) dvSprdOrntn; };

   /*! \brief getter function for extended measured status location measured, values [0 , 1] */
   bool isValid()                                     const { return ((measStatus & (1 << LOC_MEASURED) ) > 0); };
   /*! \brief getter function for extended measured status multitarged location azimuth angle, values [0 , 1] */
   bool isMultiTargetAzimuthActive()                  const { return ((measStatus & (1 << LOC_MULTITARGETAZI) ) > 0); };
   /*! \brief getter function for extended measured status multitarged location elevation angle, values [0 , 1] */
   bool isMultiTargetElevationActive()                const { return ((measStatus & (1 << LOC_MULTITARGETELEV) ) > 0); };
   /*! \brief getter function for extended measured status location standing, values [0 , 1] */
   bool isStanding()                                  const { return ((measStatus & (1 << LOC_STANDING) ) > 0); };


   /**
    * \brief radial distance measured
    *       Norm: []
    * \verbatim
    *       phys. unit: m
    * \endverbatim
   */
   vfc::float32_t d;
   /**
    * \brief radial distance variance measured
    *       Norm: []
    * \verbatim
    *       phys. unit: m^2
    * \endverbatim
   */
   vfc::float16_storage_t dVar;
   /**
    * \brief radial relative velocity measured
    *       Norm: []
    * \verbatim
    *       phys. unit: m/s
    * \endverbatim
   */
   vfc::float32_t v;
   /**
    * \brief radial relative velocity variance measured
    *       Norm: []
    * \verbatim
    *       phys. unit: (m/s)^2
    * \endverbatim
   */
   vfc::float16_storage_t vVar;
   /**
    * \brief covariance of radial distance and velocity measured
    *       Norm: []
    * \verbatim
    *       phys. unit: m^2/s

    * \endverbatim
   */
   vfc::float16_storage_t dvCov;
   /**
    * \brief quality of velocity resolution processing
    *       value range: [0 255]
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t dvQly;
   /**
    * \brief azimuth angle
    *       Norm: []
    * \verbatim
    *       phys. unit: deg
    * \endverbatim
   */
   vfc::float16_storage_t theta;
   /**
    * \brief elevation angles
    *       Norm: []
    * \verbatim
    *       phys. unit: deg
    * \endverbatim
   */
   vfc::float16_storage_t phi;
   /**
    * \brief probability for correct signal model (azimuth angle)
    *       value range: [0 255]
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t thetaQly;
   /**
    * \brief probability for correct signal model (elevation angle)
    *       value range: [0 255]
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t phiQly;
   /**
    * \brief variance of azimuth angle
    *       Norm: []
    * \verbatim
    *       phys. unit: deg^2
    * \endverbatim
   */
   vfc::float16_storage_t thetaVar;
   /**
    * \brief variance of elevation angles
    *       Norm: []
    * \verbatim
    *       phys. unit: deg^2
    * \endverbatim
   */
   vfc::float16_storage_t phiVar;
   /**
    * \brief RCS estimation
    *       Norm: []
    * \verbatim
    *       phys. unit: dBm2
    * \endverbatim
   */
   vfc::float16_storage_t RCS;
   /**
    * \brief Received Signal Strength Indication (RSSI)
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t RSSI;
   /**
    * \brief spectral spread in distance
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t dSprd;
   /**
    * \brief spectral spread in velocity
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t vSprd;
   /**
    * \brief orientation of spectral spread in distance and velocity
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t dvSprdOrntn;
   /**
    * \brief extended measured status
    *        Bit field, Bit 0: measured, Bit 1: Two target estimator azimuth,
    *                   Bit 2: Two target estimator elevation, Bit3: Location standing
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t measStatus;
};


/*! \brief external location list structure */
struct DSP_LOCATION_LIST_ST
{

public:

   // Constructor
   DSP_LOCATION_LIST_ST() : NumLocs(0), tAbsMeasTimeStamp(0) { };

   /* Access Functions  */
   /*! \brief getter function for radial distance measured, phys. unit: m */
   vfc::float32_t getRadialDistance(vfc::uint16_t locIndex)                 const { return (Item[locIndex].d); };
   /*! \brief getter function for radial distance variance measured, phys. unit: m^2 */
   vfc::float32_t getRadialDistanceVar(vfc::uint16_t locIndex)              const { return (vfc::float32_t) (Item[locIndex].dVar); };
   /*! \brief getter function for radial relative velocity measured, phys. unit: m/s */
   vfc::float32_t getRelativeRadialVelocity(vfc::uint16_t locIndex)         const { return (Item[locIndex].v); };
   /*! \brief getter function for radial relative velocity variance measured, phys. unit: (m/s)^2 */
   vfc::float32_t getRelativeRadialVelocityVar(vfc::uint16_t locIndex)      const { return vfc::float32_t (Item[locIndex].vVar); };
   /*! \brief getter function for covariance of radial distance and velocity measured, phys. unit: m^2/s */
   vfc::float32_t getRadialDistanceVelocityCov(vfc::uint16_t locIndex)      const { return (vfc::float32_t) (Item[locIndex].dvCov); };
   /*! \brief getter function for quality of velocity resolution processing, value range [0 1]  */
   vfc::float32_t getRadialDistanceVelocityQuality(vfc::uint16_t locIndex)  const { return (vfc::float32_t) (dspLocQualiDeNorm(Item[locIndex].dvQly)); };

   /*! \brief getter function for azimuth angles, phys. unit: rad */
   vfc::float32_t getAzimuthAngle(vfc::uint16_t locIndex)                   const { return (vfc::float32_t) dspLocDegToRad(Item[locIndex].theta); };
   /*! \brief getter function for azimuth angles, phys. unit: deg */
   vfc::float32_t getAzimuthAngleDeg(vfc::uint16_t locIndex)                const { return (vfc::float32_t) Item[locIndex].theta; };
   /*! \brief getter function for variance of azimuth angle, phys. unit: rad^2 */
   vfc::float32_t getAzimuthAngleVar(vfc::uint16_t locIndex)                const { return (vfc::float32_t) dspLocDegVarToRadVar(Item[locIndex].thetaVar); };
   /*! \brief getter function for variance of azimuth angle, phys. unit: deg^2 */
   vfc::float32_t getAzimuthAngleVarDeg(vfc::uint16_t locIndex)             const { return (vfc::float32_t) Item[locIndex].thetaVar; };
   /*! \brief getter function for quality of probability for correct signal model (azimuth angle), value range [0 1] */
   vfc::float32_t getAzimuthAngleQuality(vfc::uint16_t locIndex)            const { return (vfc::float32_t) dspLocQualiDeNorm(Item[locIndex].thetaQly); };
   /*! \brief getter function for elevation angles, unit: rad */
   vfc::float32_t getElevationAngle(vfc::uint16_t locIndex)                 const { return (vfc::float32_t) dspLocDegToRad(Item[locIndex].phi); };
   /*! \brief getter function for elevation angles, unit: deg */
   vfc::float32_t getElevationAngleDeg(vfc::uint16_t locIndex)              const { return (vfc::float32_t) Item[locIndex].phi; };
   /*! \brief getter function for variance of elevation angle, phys. unit: rad^2 */
   vfc::float32_t getElevationAngleVar(vfc::uint16_t locIndex)              const { return (vfc::float32_t) dspLocDegVarToRadVar(Item[locIndex].phiVar); };
   /*! \brief getter function for variance of elevation angle, phys. unit: deg^2 */
   vfc::float32_t getElevationAngleVarDeg(vfc::uint16_t locIndex)           const { return (vfc::float32_t) Item[locIndex].phiVar; };
   /*! \brief getter function for quality of probability for correct signal model (elevation angle), value range [0 1] */
   vfc::float32_t getElevationAngleQuality(vfc::uint16_t locIndex)          const { return (vfc::float32_t) dspLocQualiDeNorm(Item[locIndex].phiQly); };

   /*! \brief getter function for RCS estimation */
   vfc::float32_t getRcs(vfc::uint16_t locIndex)                            const { return (vfc::float32_t) (Item[locIndex].RCS); };
   /*! \brief getter function for Received Signal Strength Indication (RSSI) */
   vfc::float32_t getRssi(vfc::uint16_t locIndex)                           const { return (vfc::float32_t) (Item[locIndex].RSSI); };
   /*! \brief getter function for spectral spread in distance */
   vfc::float32_t getRadialDistanceSpread(vfc::uint16_t locIndex)           const { return (vfc::float32_t) (Item[locIndex].dSprd); };
   /*! \brief getter function for spectral spread in velocity */
   vfc::float32_t getRelativeRadialVelocitySpread(vfc::uint16_t locIndex)   const { return (vfc::float32_t) (Item[locIndex].vSprd); };
   /*! \brief getter function for orientation of spectral spread in distance and velocity */
   vfc::float32_t getDistanceVelocitySpreadOrientation(vfc::uint16_t locIndex)  const { return (vfc::float32_t) (Item[locIndex].dvSprdOrntn); };

   /*! \brief getter function for extended measured status location measured, values [0 , 1] */
   bool isValid(vfc::uint16_t locIndex)                                     const { return ((Item[locIndex].measStatus & (1 << LOC_MEASURED) ) > 0); };
   /*! \brief getter function for extended measured status multitarged location azimuth angle, values [0 , 1] */
   bool isMultiTargetElevationActive(vfc::uint16_t locIndex)                const { return ((Item[locIndex].measStatus & (1 << LOC_MULTITARGETELEV) ) > 0); };
   /*! \brief getter function for extended measured status multitarged location elevation angle, values [0 , 1] */
   bool isMultiTargetAzimuthActive(vfc::uint16_t locIndex)                  const { return ((Item[locIndex].measStatus & (1 << LOC_MULTITARGETAZI) ) > 0); };
   /*! \brief getter function for extended measured status location standing, values [0 , 1] */
   bool isStanding(vfc::uint16_t locIndex)                                  const { return ((Item[locIndex].measStatus & (1 << LOC_STANDING) ) > 0); };

   /*! \brief getter function for number of available locations in external location list */
   vfc::uint16_t getNumberOfLocations(void)                                const { return (NumLocs); };
   /*! \brief getter function for absolute time stamp (ECU) of measurement, phys. unit: s */
   vfc::uint32_t getAbsMeasTime(void)                                      const { return (tAbsMeasTimeStamp); };

   /**
    * \brief external location items structure
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   struct DSP_LOCATION_ITEM_ST Item[MAXNUMLOCATIONS];

   /**
    * \brief number of available locations in external location list
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
    vfc::uint16_t NumLocs;
   /**
    * \brief absolute time stamp (ECU) of measurement
    *       Norm: [N_t65536UL_ul = 65536]
    * \verbatim
    *       phys. unit: s
    * \endverbatim
   */
    vfc::uint32_t tAbsMeasTimeStamp;
};

}

#endif
