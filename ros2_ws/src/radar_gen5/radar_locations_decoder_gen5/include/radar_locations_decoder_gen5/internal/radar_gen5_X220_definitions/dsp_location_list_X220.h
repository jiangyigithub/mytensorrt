/******************************************************************************/
/*! \file  dsp_location_list.h

\brief Header for Location List

\author CC-DA/ECR3 Knut Schumacher and Oliver Perkuhn

*/
/******************************************************************************/
#ifndef DSP_LOCATION_LIST_H_INCLUDED
#define DSP_LOCATION_LIST_H_INCLUDED

// 1020: Avoid macros.
//   ---> macros are allowed
// PRQA S 1020 EOF

// 1021: This macro is replaced with a literal.
// --> DSP coding rules allow macros.
//PRQA S 1021 EOF

// 1026: Macro may be used as a constant expression.
// --> DSP coding rules allow macros.
//PRQA S 1026 EOF

// QAC Rule 2000, 2300, 2400: The function 'func' is in the global scope. The object ‘name’ is in the global scope. The type name 'name' is in the global scope.
// Global namespace is explicitly allowed within DSP project context. See DSP handbook. Naming conflicts are prevented by design.
// PRQA S 2000, 2300, 2400  EOF


#ifdef _MSC_VER
#pragma warning(push,0)
#endif
#include <vfc/core/vfc_types.hpp>
#include <vfc/container/vfc_fifo.hpp>
#include <vfc/core/vfc_float16_storage.hpp>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

namespace X220
{

// copied from dsp_location_list_cfg.h
#define MAXNUMLOCATIONS 342

// Norms
#define DSP_LOC_QUALI_NORM      ((vfc::uint8_t)255)
#define dspLocQualiDeNorm(x)    ((x)*0.003921568627451F)  // 1/255
#define dspLocDegToRad(x)       ((x)*0.017453292519943F)  // pi/180
#define dspLocDegVarToRadVar(x) ((x)*(0.017453292519943F*0.017453292519943F)) // (pi/180)^2

// maximum value for AbsMeasTimeStamp
#define DSP_ABSMEASTIMESTAMP_MAX    (281474976)

//! bit position of extended measured status of location
enum LocMeasStatus
{
   LOC_MEASURED=0,
   LOC_MULTITARGETAZI,
   LOC_MULTITARGETELEV,
   LOC_STANDING,
   LOC_MULTITARGETVAR,
   LOC_LOWPOWER,
   LOC_AZIMUTH_AMBIGUITY
};

/**
 * \brief external location items structure
 *       Norm: []
 * \verbatim
 *       phys. unit: -
 * \endverbatim
*/
// 2171: The struct 'class_name' has access specifiers.
// 2173: The struct 'class_name' has constructors and/or destructor.
// 2175: This struct is not a POD type.
// --> in the DSP architecture when there are only member variable with the
// set/get methods and the initialization list the struct name can be used.
// PRQA S 2171, 2173, 2175 1
struct DSP_LOCATION_ITEM_ST
{

public:
   // Constructor
   DSP_LOCATION_ITEM_ST() : d(0.0F), v(0.0F), dVar(0.0F), vVar(0.0F), dvCov(0.0F),
      theta(0.0F), phi(0.0F), thetaQly(0U), phiQly(0U), thetaVar(0.0F), phiVar(0.0F),
      RCS(0.0F), RSSI(0.0F), dvQly(0U), measStatus(0U)
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


   /*! \brief getter function for azimuth angles (cone angle: [x,y,z] = [r*sqrt(1-sin^2(theta)-sin^2(phi)), r*sin(theta), r*sin(phi)]), phys. unit: rad */
   vfc::float32_t getAzimuthAngle()                   const { return (vfc::float32_t) dspLocDegToRad(theta); };
   /*! \brief getter function for azimuth angles (cone angle), phys. unit: deg */
   vfc::float32_t getAzimuthAngleDeg()                const { return (vfc::float32_t) theta; };
   /*! \brief getter function for variance of azimuth angle (cone angle), phys. unit: rad^2 */
   vfc::float32_t getAzimuthAngleVar()                const { return (vfc::float32_t) dspLocDegVarToRadVar(thetaVar); };
   /*! \brief getter function for variance of azimuth angle (cone angle), phys. unit: deg^2 */
   vfc::float32_t getAzimuthAngleVarDeg()             const { return (vfc::float32_t) thetaVar; };
   /*! \brief getter function for quality of probability for correct signal model (azimuth angle), value range [0 1] */
   vfc::float32_t getAzimuthAngleQuality()            const { return (vfc::float32_t) dspLocQualiDeNorm(thetaQly); };
   /*! \brief getter function for elevation angles (cone angle), unit: rad */
   vfc::float32_t getElevationAngle()                 const { return (vfc::float32_t) dspLocDegToRad(phi); };
   /*! \brief getter function for elevation angles (cone angle), unit: deg */
   vfc::float32_t getElevationAngleDeg()              const { return (vfc::float32_t) phi; };
   /*! \brief getter function for variance of elevation angle (cone angle), phys. unit: rad^2 */
   vfc::float32_t getElevationAngleVar()              const { return (vfc::float32_t) dspLocDegVarToRadVar(phiVar); };
   /*! \brief getter function for variance of elevation angle (cone angle), phys. unit: deg^2 */
   vfc::float32_t getElevationAngleVarDeg()           const { return (vfc::float32_t) phiVar; };
   /*! \brief getter function for quality of probability for correct signal model (elevation angle), value range [0 1] */
   vfc::float32_t getElevationAngleQuality()          const { return (vfc::float32_t) dspLocQualiDeNorm(phiQly); };

   /*! \brief getter function for RCS estimation */
   vfc::float32_t getRcs()                            const { return (vfc::float32_t) RCS; };
   /*! \brief getter function for Received Signal Strength Indication (RSSI), value range [0 100] */
   vfc::float32_t getRssi()                           const { return (vfc::float32_t) RSSI; };

   /*! \brief getter function for extended measured status location measured and range check passed, values [0 , 1] */
   bool isValid()                                     const { return ((measStatus & (1 << LOC_MEASURED) ) > 0); };
   /*! \brief getter function for extended measured status multitarged location azimuth angle, values [0 , 1] */
   bool isMultiTargetAzimuthActive()                  const { return ((measStatus & (1 << LOC_MULTITARGETAZI) ) > 0); };
   /*! \brief getter function for extended measured status multitarged location elevation angle, values [0 , 1] */
   bool isMultiTargetElevationActive()                const { return ((measStatus & (1 << LOC_MULTITARGETELEV) ) > 0); };
   /*! \brief getter function for extended measured status location standing, values [0 , 1] */
   bool isStanding()                                  const { return ((measStatus & (1 << LOC_STANDING) ) > 0); };
   /*! \brief getter function for extended measured status low power location, if low power location, getter return false, values [0 , 1] */
   bool isHighPower()                                 const { return (!((measStatus & (1 << LOC_LOWPOWER)) > 0)); };
   /*! \brief getter function for extended measured status azimuth ambiguity detected, values [0 , 1] */
   bool isAzimuthAmbiguityDetected()                  const { return ((measStatus & (1 << LOC_AZIMUTH_AMBIGUITY)) > 0); };

   /*! \brief getter function for index of location which is the azimuth ambiguity peer, UINT16_MAX if there is no ambiguity peer */
   vfc::uint16_t getIdxAzimuthAmbiguityPeer()         const { return idxAzimuthAmbiguityPeer; };


   /**
    * \brief radial distance measured
    *       Norm: []
    * \verbatim
    *       phys. unit: m
    * \endverbatim
   */
   vfc::float32_t d;    // PRQA S 2100       // 2100: object_name' is public
   /**
    * \brief radial relative velocity measured
    *       Norm: []
    * \verbatim
    *       phys. unit: m/s
    * \endverbatim
   */
   vfc::float32_t v;    // PRQA S 2100       // 2100: object_name' is public
   /**
   * \brief radial distance variance measured
   *       Norm: []
   * \verbatim
   *       phys. unit: m^2
   * \endverbatim
   */
   vfc::float16_storage_t dVar;    // PRQA S 2100       // 2100: object_name' is public
   /**
    * \brief radial relative velocity variance measured
    *       Norm: []
    * \verbatim
    *       phys. unit: (m/s)^2
    * \endverbatim
   */
   vfc::float16_storage_t vVar;    // PRQA S 2100       // 2100: object_name' is public
   /**
    * \brief covariance of radial distance and velocity measured
    *       Norm: []
    * \verbatim
    *       phys. unit: m^2/s

    * \endverbatim
   */
   vfc::float16_storage_t dvCov;    // PRQA S 2100       // 2100: object_name' is public
   /**
    * \brief azimuth angle (cone angle: [x,y,z] = [r*sqrt(1-sin^2(theta)-sin^2(phi)), r*sin(theta), r*sin(phi)])
    *       Norm: []
    * \verbatim
    *       phys. unit: deg
    * \endverbatim
   */
   vfc::float16_storage_t theta;    // PRQA S 2100       // 2100: object_name' is public
   /**
    * \brief elevation angles (cone angle)
    *       Norm: []
    * \verbatim
    *       phys. unit: deg
    * \endverbatim
   */
   vfc::float16_storage_t phi;    // PRQA S 2100       // 2100: object_name' is public
   /**
    * \brief probability for correct signal model (azimuth angle)
    *       value range: [0 255]
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t thetaQly;    // PRQA S 2100       // 2100: object_name' is public
   /**
    * \brief probability for correct signal model (elevation angle)
    *       value range: [0 255]
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t phiQly;    // PRQA S 2100       // 2100: object_name' is public
   /**
    * \brief variance of azimuth angle (cone angle)
    *       Norm: []
    * \verbatim
    *       phys. unit: deg^2
    * \endverbatim
   */
   vfc::float16_storage_t thetaVar;    // PRQA S 2100       // 2100: object_name' is public
   /**
    * \brief variance of elevation angles (cone angle)
    *       Norm: []
    * \verbatim
    *       phys. unit: deg^2
    * \endverbatim
   */
   vfc::float16_storage_t phiVar;    // PRQA S 2100       // 2100: object_name' is public
   /**
    * \brief RCS estimation
    *       Norm: []
    * \verbatim
    *       phys. unit: dBm2
    * \endverbatim
   */
   vfc::float16_storage_t RCS;    // PRQA S 2100       // 2100: object_name' is public
   /**
    * \brief Received Signal Strength Indication (RSSI)
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t RSSI;    // PRQA S 2100       // 2100: object_name' is public
   /**
    * \brief quality of velocity resolution processing
    *       value range: [0 255]
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t dvQly;    // PRQA S 2100       // 2100: object_name' is public
   /**
    *\brief extended measured status
    *        Bit field, Bit 0: measured and range check passed
    *                   Bit 1 : Two target estimator azimuth,
    *                   Bit 2 : Two target estimator elevation
    *                   Bit 3 : Location standing
    *                   Bit 4 : Two target estimator VAR/velocity
    *                   Bit 5 : Low power location
    *                   Bit 6 : Azimuth ambiguity location
    *       Norm : []
    * \verbatim
    *       phys.unit: -
    *\endverbatim
   */
   vfc::uint8_t measStatus;    // PRQA S 2100       // 2100: object_name' is public
   /**
    * \brief index of location peer of azimuth ambiguity
    *
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint16_t idxAzimuthAmbiguityPeer;    // PRQA S 2100       // 2100: object_name' is public

};


/*! \brief external location list structure */
// 2171: The struct 'class_name' has access specifiers.
// 2173: The struct 'class_name' has constructors and/or destructor.
// 2175: This struct is not a POD type.
// --> in the DSP architecture when there are only member variable with the
// set/get methods and the initialization list the struct name can be used.
// 2400: The type name 'name' is in the global scope.
// --> the struct name starts with the module name, no problem
// PRQA S 2171, 2173, 2175, 2400 1
struct DSP_LOCATION_LIST_ST
{

public:

   // Constructor
   DSP_LOCATION_LIST_ST() : NumLocs(0U), tAbsMeasTimeStamp(0UL) { };

   /* Access Functions  */
   // 2844: Possible: Dereference of an invalid pointer value.
   //    --> per design ensured, that there are no invalid pointer value
   // PRQA S 2844 ++
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
   vfc::float32_t getRadialDistanceVelocityQuality(vfc::uint16_t locIndex)  const { return (vfc::float32_t) dspLocQualiDeNorm(Item[locIndex].dvQly); };

   /*! \brief getter function for azimuth angles (cone angle: [x,y,z] = [r*sqrt(1-sin^2(theta)-sin^2(phi)), r*sin(theta), r*sin(phi)]), phys. unit: rad */
   vfc::float32_t getAzimuthAngle(vfc::uint16_t locIndex)                   const { return (vfc::float32_t) dspLocDegToRad(Item[locIndex].theta); };
   /*! \brief getter function for azimuth angles (cone angle), phys. unit: deg */
   vfc::float32_t getAzimuthAngleDeg(vfc::uint16_t locIndex)                const { return (vfc::float32_t) Item[locIndex].theta; };
   /*! \brief getter function for variance of azimuth angle (cone angle), phys. unit: rad^2 */
   vfc::float32_t getAzimuthAngleVar(vfc::uint16_t locIndex)                const { return (vfc::float32_t) dspLocDegVarToRadVar(Item[locIndex].thetaVar); };
   /*! \brief getter function for variance of azimuth angle (cone angle), phys. unit: deg^2 */
   vfc::float32_t getAzimuthAngleVarDeg(vfc::uint16_t locIndex)             const { return (vfc::float32_t) Item[locIndex].thetaVar; };
   /*! \brief getter function for quality of probability for correct signal model (azimuth angle), value range [0 1] */
   vfc::float32_t getAzimuthAngleQuality(vfc::uint16_t locIndex)            const { return (vfc::float32_t) dspLocQualiDeNorm(Item[locIndex].thetaQly); };
   /*! \brief getter function for elevation angles (cone angle), unit: rad */
   vfc::float32_t getElevationAngle(vfc::uint16_t locIndex)                 const { return (vfc::float32_t) dspLocDegToRad(Item[locIndex].phi); };
   /*! \brief getter function for elevation angles (cone angle), unit: deg */
   vfc::float32_t getElevationAngleDeg(vfc::uint16_t locIndex)              const { return (vfc::float32_t) Item[locIndex].phi; };
   /*! \brief getter function for variance of elevation angle (cone angle), phys. unit: rad^2 */
   vfc::float32_t getElevationAngleVar(vfc::uint16_t locIndex)              const { return (vfc::float32_t) dspLocDegVarToRadVar(Item[locIndex].phiVar); };
   /*! \brief getter function for variance of elevation angle (cone angle), phys. unit: deg^2 */
   vfc::float32_t getElevationAngleVarDeg(vfc::uint16_t locIndex)           const { return (vfc::float32_t) Item[locIndex].phiVar; };
   /*! \brief getter function for quality of probability for correct signal model (elevation angle), value range [0 1] */
   vfc::float32_t getElevationAngleQuality(vfc::uint16_t locIndex)          const { return (vfc::float32_t) dspLocQualiDeNorm(Item[locIndex].phiQly); };

   /*! \brief getter function for RCS estimation */
   vfc::float32_t getRcs(vfc::uint16_t locIndex)                            const { return (vfc::float32_t) (Item[locIndex].RCS); };
   /*! \brief getter function for Received Signal Strength Indication (RSSI) */
   vfc::float32_t getRssi(vfc::uint16_t locIndex)                           const { return (vfc::float32_t) (Item[locIndex].RSSI); };

   /*! \brief getter function for extended measured status location measured and range check passed, values [0 , 1] */
   bool isValid(vfc::uint16_t locIndex)                                     const { return ((Item[locIndex].measStatus & (1 << LOC_MEASURED) ) > 0); };
   /*! \brief getter function for extended measured status multitarged location azimuth angle, values [0 , 1] */
   bool isMultiTargetElevationActive(vfc::uint16_t locIndex)                const { return ((Item[locIndex].measStatus & (1 << LOC_MULTITARGETELEV) ) > 0); };
   /*! \brief getter function for extended measured status multitarged location elevation angle, values [0 , 1] */
   bool isMultiTargetAzimuthActive(vfc::uint16_t locIndex)                  const { return ((Item[locIndex].measStatus & (1 << LOC_MULTITARGETAZI) ) > 0); };
   /*! \brief getter function for extended measured status location standing, values [0 , 1] */
   bool isStanding(vfc::uint16_t locIndex)                                  const { return ((Item[locIndex].measStatus & (1 << LOC_STANDING) ) > 0); };
   /*! \brief getter function for extended measured status low power location, if low power location, getter return false, values [0 , 1] */
   bool isHighPower(vfc::uint16_t locIndex)                                 const { return (!((Item[locIndex].measStatus & (1 << LOC_LOWPOWER)) > 0)); };
   /*! \brief getter function for extended measured status azimuth ambiguity detected, values [0 , 1] */
   bool isAzimuthAmbiguityDetected(vfc::uint16_t locIndex)                  const { return ((Item[locIndex].measStatus & (1 << LOC_AZIMUTH_AMBIGUITY)) > 0); };

   /*! \brief getter function for index of location which is the azimuth ambiguity peer, UINT16_MAX if there is no ambiguity peer */
   vfc::uint16_t getIdxAzimuthAmbiguityPeer(vfc::uint16_t locIndex)         const { return Item[locIndex].idxAzimuthAmbiguityPeer; };

   // PRQA S 2844 --
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
   struct DSP_LOCATION_ITEM_ST Item[MAXNUMLOCATIONS];     // PRQA S 2100       // 2100: object_name' is public

   /**
    * \brief number of available locations in external location list
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
    vfc::uint16_t NumLocs;    // PRQA S 2100       // 2100: object_name' is public
   /**
    * \brief absolute time stamp (ECU) of measurement
    *       Norm: [N_t65536UL_ul = 65536]
    * \verbatim
    *       phys. unit: s
    * \endverbatim
   */
    vfc::uint32_t tAbsMeasTimeStamp;    // PRQA S 2100       // 2100: object_name' is public
};
}
#endif
