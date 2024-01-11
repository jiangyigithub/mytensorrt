/******************************************************************************/
/*! \file  dsp_sensingstatus_data_packet.h
\brief Header for Sensing Status

\author CC-DA/ECR3 Knut Schumacher

*/
/******************************************************************************/
#ifndef DSP_SENSINGSTATUS_H_INCLUDED
#define DSP_SENSINGSTATUS_H_INCLUDED

// 1020: Avoid macros
// --> DSP coding rules allow macros.
//PRQA S 1020 EOF

// QAC Rule 1021: This macro is replaced with a literal.
// The DSP coding rules allow macros. See DSP handbook.
// PRQA S 1021EOF

// QAC Rule 2000, 2300, 2400: The function 'func' is in the global scope. The object ‘name’ is in the global scope. The type name 'name' is in the global scope.
// Global namespace is explicitly allowed within DSP project context. See DSP handbook. Naming conflicts are prevented by design.
// PRQA S 2000, 2300, 2400  EOF

#ifdef _MSC_VER
#pragma warning(push,0)
#endif
#include <vfc/core/vfc_types.hpp>
#include <vfc/container/vfc_fifo.hpp>
#include <vfc/core/vfc_math.hpp>
#include <vfc/core/vfc_float16_storage.hpp>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

namespace X170
{

// Norms
/** macro rad to deg x*180/pi */
#define dspSensRadToDeg(x)      ((x)*57.295779513F)
// FoV
//! Array size of FoV in azimuth (size >1)
#define MAXNUMTHETAFOV (25U)
//! Azimuth span for FoV calculation [-THETASPAN THETASPAN]
#define THETASPAN     (60.0F)
//! RCS reference in m^2 for FoV
#define FOV_RCSREF    (10.0F)

//! Array size of FoV factor in elevation >1
#define MAXNUMPHIFOV  (11U)
//! Elevation span for FoV calculation [-PHISPAN PHISPAN]
#define PHISPAN       (15.0F)

/*! \brief Modulation Performance Structure */
struct DSP_MODULATIONPERFORMANCE_ST
{

public:
   // Constructor

   DSP_MODULATIONPERFORMANCE_ST() : DmpID(0U), ModID(0U), facDMaxModDur(0.0F), dSprblty(0.0F), vSprblty(0.0F),
      dPrec(0.0F), vPrec(0.0F), dvCov(0.0F), dmin(0.0F), dmax(0.0F), vmin(0.0F), vmax(0.0F)
   { };

   /**
    * \brief active detection and measurement program ID
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t DmpID;
   /**
    * \brief active modulation ID
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint16_t ModID;
   /**
    * \brief distance range scaling factor for FoV from modulation
    *        (distance degradation due to thermal degradation, noise increase)
    *        1: full range
    *        0.93: range reduction if 75% of ramps are used
    *        0.84: range reduction if 50% of ramps are used
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t facDMaxModDur;

   /*!  \brief separability in distance
   Norm: []
   \verbatim
   phys. unit: m
   \endverbatim
   */
   vfc::float16_storage_t dSprblty;
   /**
    * \brief separability in relative velocity
    *       Norm: []
    * \verbatim
    *       phys. unit: m/s
    * \endverbatim
   */

   vfc::float16_storage_t vSprblty;
   /**
    * \brief precision in distance
    *       Norm: []
    * \verbatim
    *       phys. unit: m
    * \endverbatim
   */
   vfc::float16_storage_t dPrec;
   /**
    * \brief precision in relative velocity
    *       Norm: []
    * \verbatim
    *       phys. unit: m/s
    * \endverbatim
   */
   vfc::float16_storage_t vPrec;
   /**
    * \brief covariance of distance and relative velocity
    *       Norm: []
    * \verbatim
    *       phys. unit: m^2/s
    * \endverbatim
   */
   vfc::float16_storage_t dvCov;
   /**
    * \brief minimal measurable distance
    *       Norm: []
    * \verbatim
    *       phys. unit: m
    * \endverbatim
   */
   vfc::float16_storage_t dmin;
   /**
    * \brief maximal measurable distance
    *       Norm: []
    * \verbatim
    *       phys. unit: m
    * \endverbatim
   */
   vfc::float16_storage_t dmax;
   /**
    * \brief minimal measurable relative velocity
    *       Norm: []
    * \verbatim
    *       phys. unit: m/s
    * \endverbatim
   */
   vfc::float16_storage_t vmin;
   /**
    * \brief maximal measurable relative velocity
    *       Norm: []
    * \verbatim
    *       phys. unit: m/s
    * \endverbatim
   */
   vfc::float16_storage_t vmax;
};

/*! \brief Misalignment Structure */
struct DSP_MISALIGNMENT_ST
{

public:
   // Constructor
   DSP_MISALIGNMENT_ST() : thetaMalAng(0.0F), thetaMalAngVar(0.0F),
         phiMalAng(0.0F), phiMalAngVar(0.0F), malStatus(0U), malStatusEme(0U), phiMalAngEme(0.0F), phiMalAngEmeVar(0.0F), percNegativeTheta(0.0F)
   { };

   /**
    * \brief estimated misalignment angle in azimuth (spherical coordinates)
    *       Norm: []
    * \verbatim
    *       phys. unit: rad
    * \endverbatim
   */
   vfc::float16_storage_t thetaMalAng;
   /**
    * \brief variance of estimated misalignment angle in azimuth (spherical coordinates)
    *       Norm: []
    * \verbatim
    *       phys. unit: rad^2
    * \endverbatim
   */
   vfc::float16_storage_t thetaMalAngVar;
   /**
    * \brief estimated misalignment angle in elevation (spherical coordinates)
    *       Norm: []
    * \verbatim
    *       phys. unit: rad
    * \endverbatim
   */
   vfc::float16_storage_t phiMalAng;
   /**
    * \brief variance of estimated misalignment angle in elevation (spherical coordinates)
    *       Norm: []
    * \verbatim
    *       phys. unit: rad^2
    * \endverbatim
   */
   vfc::float16_storage_t phiMalAngVar;
   /**
    * \brief status of estimation ( 0 == valid >0 not valid)
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t malStatus;
   /**
    * \brief status of eme based estimation ( 0 == valid >0 not valid)
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t malStatusEme;
   /**
    * \brief estimated misalignment angle in elevation (EME) (spherical coordinates)
    *       Norm: []
    * \verbatim
    *       phys. unit: rad
    * \endverbatim
   */
   vfc::float16_storage_t phiMalAngEme;
   /**
    * \brief variance of estimated misalignment angle in elevation (EME) (spherical coordinates)
    *       Norm: []
    * \verbatim
    *       phys. unit: rad^2
    * \endverbatim
   */
   vfc::float16_storage_t phiMalAngEmeVar;
   /**
   * \brief percentage of standing locations selected by MAL on one side of the radar axis (<0° in azimuth)
   *       Norm: []
   * \verbatim
   *       phys. unit: -
   * \endverbatim
   */
   vfc::float16_storage_t percNegativeTheta;
};

/*! \brief Blindness Structure and states */
enum ValidFlgState
{
   //! radar coating indicator status = invalid
   RCI_INDCR_INVALID = 0,
   //! radar coating indicator status = valid
   RCI_INDCR_VALID = 255
};

struct DSP_BLINDNESS_ST
{

public:
   // Constructor
   DSP_BLINDNESS_ST() :
      mdThetaIndcrSO(0.F),
      mdThetaIndcrMO(0.F),
	  mdThetaIndcr(0.F),
      mdPhiIndcr(0.F),
      mdRCSIndcrMO(0.F),
      nBinsOverDetThdIndcr(0.F),
      nRefIndcr(0.F),
      mdCrossCorrIndcrSO(0.F),
      mdThetaIndcrSOVldFlg(0U),
      mdThetaIndcrMOVldFlg(0U),
	  mdThetaIndcrVldFlg(0U),
      mdPhiIndcrVldFlg(0U),
      mdRCSIndcrMOVldFlg(0U),
      nBinsOverDetThdIndcrVldFlg(0U),
      nRefIndcrVldFlg(0U),
      mdCrossCorrIndcrSOVldFlg(0U)
   { };

   /**
    * \brief azimuth angle fit one target model deviation for SO's
    *       value range: [0 1]
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t mdThetaIndcrSO;
   /**
    * \brief Azimuth angle fit one target model deviation for MO's
    *       value range: [0 1]
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t mdThetaIndcrMO;
   /**
   * \brief Azimuth angle fit one target model deviation
   *       value range: [0 1]
   * \verbatim
   *       phys. unit: -
   * \endverbatim
   */
   vfc::float16_storage_t mdThetaIndcr;
   /**
    * \brief  Elevation angle fit one target model deviation indicator
    *       value range: [0 1]
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t mdPhiIndcr;
   /**
    * \brief Radar Cross Section Model Deviation Indicator for MO's
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t mdRCSIndcrMO;
   /**
    * \brief  Number of bins over detection threshold
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t nBinsOverDetThdIndcr;
   /**
    * \brief Number of valid reflexions indicator
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t nRefIndcr;
   /**
    * \brief Standing object model deviation indicator
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t mdCrossCorrIndcrSO;
   /**
    * \brief validity flag for mdThetaIndcrSO
    *       0   = indicator invalid
	*		255 = indicator valid
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t mdThetaIndcrSOVldFlg;
   /**
    * \brief validity flag for mdThetaIndcrMO
    *       0   = indicator invalid
	*		255 = indicator valid
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t mdThetaIndcrMOVldFlg;
   /**
   * \brief validity flag for mdThetaIndcr
   *       0   = indicator invalid
   *		255 = indicator valid
   * \verbatim
   *       phys. unit: -
   * \endverbatim
   */
   vfc::uint8_t mdThetaIndcrVldFlg;
   /**
    * \brief validity flag for mdPhiIndcr
    *       0   = indicator invalid
    *     255 = indicator valid
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t mdPhiIndcrVldFlg;
   /**
    * \brief validity flag for mdRCSIndcrMO
    *       0   = indicator invalid
    *     255 = indicator valid
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t mdRCSIndcrMOVldFlg;
   /**
    * \brief validity flag for nBinsOverDetThdIndcr
    *       0   = indicator invalid
    *     255 = indicator valid
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t nBinsOverDetThdIndcrVldFlg;
      /**
    * \brief validity flag for nRefIndcr
    *       0   = indicator invalid
    *    255 = indicator valid
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t nRefIndcrVldFlg;
   /**
    * \brief validity flag for mdCrossCorrIndcrSO
    *        0   = indicator invalid
    *     255 = indicator valid
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t mdCrossCorrIndcrSOVldFlg;

   /*! \brief getter function for mdThetaIndcrSO indicator */
   vfc::float32_t getMdThetaIndcrSO() const { return (vfc::float32_t)mdThetaIndcrSO; }
   /*! \brief getter function for mdThetaIndcrMO indicator */
   vfc::float32_t getMdThetaIndcrMO() const { return (vfc::float32_t)mdThetaIndcrMO; }
   /*! \brief getter function for mdThetaIndcrMO indicator */
   vfc::float32_t getMdThetaIndcr() const { return (vfc::float32_t)mdThetaIndcr; }
   /*! \brief getter function for mdPhiIndcr indicator */
   vfc::float32_t getMdPhiIndcr() const { return (vfc::float32_t)mdPhiIndcr; }
   /*! \brief getter function for mdRCSIndcrMO indicator */
   vfc::float32_t getMdRCSIndcrMO() const { return (vfc::float32_t)mdRCSIndcrMO; }
   /*! \brief getter function for nBinsOverDetThdIndcr indicator */
   vfc::float32_t getNBinsOverDetThdIndcr() const { return (vfc::float32_t)nBinsOverDetThdIndcr; }
   /*! \brief getter function for nRefIndcr indicator */
   vfc::float32_t getNRefIndcr() const { return (vfc::float32_t)nRefIndcr; }
   /*! \brief getter function for mdCrossCorrIndcrSO indicator */
   vfc::float32_t getMdCrossCorrIndcrSO() const { return (vfc::float32_t)mdCrossCorrIndcrSO; }

   /*! \brief getter function for mdThetaIndcrSO indicator validity flag */
   bool isMdThetaIndcrSOVld() const { return (mdThetaIndcrSOVldFlg == (vfc::uint8_t) RCI_INDCR_VALID); }
   /*! \brief getter function for mdThetaIndcrMO indicator validity flag */
   bool isMdThetaIndcrMOVld() const { return (mdThetaIndcrMOVldFlg == (vfc::uint8_t) RCI_INDCR_VALID); }
   /*! \brief getter function for mdThetaIndcrMO indicator validity flag */
   bool isMdThetaIndcrVld() const { return (mdThetaIndcrVldFlg == (vfc::uint8_t) RCI_INDCR_VALID); }
   /*! \brief getter function for mdPhiIndcr indicator validity flag */
   bool isMdPhiIndcrVld() const { return (mdPhiIndcrVldFlg == (vfc::uint8_t) RCI_INDCR_VALID); }
   /*! \brief getter function for mdRCSIndcrMO indicator validity flag */
   bool isMdRCSIndcrMOVld() const { return (mdRCSIndcrMOVldFlg == (vfc::uint8_t) RCI_INDCR_VALID); }
   /*! \brief getter function for nBinsOverDetThdIndcr indicator validity flag */
   bool isNBinsOverDetThdIndcrVld() const { return (nBinsOverDetThdIndcrVldFlg == (vfc::uint8_t) RCI_INDCR_VALID); }
   /*! \brief getter function for nRefIndcr indicator validity flag */
   bool isNRefIndcrVld() const { return (nRefIndcrVldFlg == (vfc::uint8_t) RCI_INDCR_VALID); }
   /*! \brief getter function for mdCrossCorrIndcrSO indicator validity flag */
   bool isMdCrossCorrIndcrSOVld() const { return (mdCrossCorrIndcrSOVldFlg == (vfc::uint8_t) RCI_INDCR_VALID); }

};

/*! \brief Interference Structure */
struct DSP_INTERFERENCE_ST
{

public:
   // Constructor
   DSP_INTERFERENCE_ST() : IntfrIndcr(0.0F), IntfrIndcrStatus(0U)
   { };

   /**
    * \brief FoV reduction due to interference
    *        1.0: no interference found, no degradation
    *        0.0: completely blind due to interference
    *        Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t IntfrIndcr;
   /**
    * \brief status of interference indicator
    *        0 = invalid, 1 = valid and no interference found,
    *        2 = valid and interference found
    *        Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t IntfrIndcrStatus;
};

/*! \brief Field of View Structure */
struct DSP_FIELDOFVIEW_ST
{

public:
   // Constructor
   DSP_FIELDOFVIEW_ST() : dMaxThetaView(), thetaViewAry(),
         facDMaxPhiView(), phiViewAry()
   { };
   /**
    * \brief FoV range for 10m^2 target at azimuth angle (cone angle) thetaViewAry[x], elevation angle = 0
    *       Norm: []
    * \verbatim
    *       phys. unit: m
    * \endverbatim
   */
   vfc::float16_storage_t dMaxThetaView[MAXNUMTHETAFOV];
   /**
    * \brief azimuth angle array (cone angle) for FoV
    *       Norm: []
    * \verbatim
    *       phys. unit: deg
    * \endverbatim
   */
   vfc::float16_storage_t thetaViewAry[MAXNUMTHETAFOV];
   /**
    * \brief Range scaling for elevation angle (cone angle) phiViewAry[x], azimuth angle = 0
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t facDMaxPhiView[MAXNUMPHIFOV];
   /**
    * \brief elevation angle array (cone angle) for FoV
    *       Norm: []
    * \verbatim
    *       phys. unit: deg
    * \endverbatim
   */
   vfc::float16_storage_t phiViewAry[MAXNUMPHIFOV];
};

/*! \brief sensing status structure */
struct DSP_SENSINGSTATE_DATA_PACKET_ST
{

public:

   // Constructor
   DSP_SENSINGSTATE_DATA_PACKET_ST() : OpMode(0U), DataMeasured(0U), facThermalDeg(0.0F)
   { };

   /*! \brief getter function for DSP operation mode */
   vfc::uint8_t getOpMode(void)  const { return (OpMode); };
   /*! \brief getter function for measurement state of cycle */
   bool isDataMeasured(void)  const { return (DataMeasured!=0U); };
   /** \brief getter function for factor for degradation of average
    *         duty cycle due to thermal degradation
    *         (skip cycles and/or reduce ramps) */
   vfc::float16_storage_t getfacThermalDeg(void) const { return (facThermalDeg); };

   /**
   \brief getter function for FoV range depending on azimuth angle, elevation angle (in cone angles)
   assumptions: constant noise in d, input angles in sensor coordinates (cone angle:
   [x,y,z] = [r*sqrt(1-sin^2(theta)-sin^2(phi)), r*sin(theta), r*sin(phi)]) without misaglinment

   \param angleAzimuth           in: azimuth angle (cone angle), phys. unit: rad
   \param angleElevation         in: elevation angle (cone angle), phys. unit: rad

   \return FoV for reference object, phys. unit: m


   */
   vfc::float32_t getViewDistance(vfc::float32_t angleAzimuth, vfc::float32_t angleElevation) const
   {
      vfc::float32_t aziMinDist = std::numeric_limits<vfc::float32_t>::max();
      vfc::float32_t elevMinDist = std::numeric_limits<vfc::float32_t>::max();
      const vfc::float32_t aziDeg = dspSensRadToDeg(angleAzimuth);
      const vfc::float32_t elevDeg = dspSensRadToDeg(angleElevation);
      vfc::float32_t aziDist = 0.0F;
      vfc::float32_t elevDist = 0.0F;
      // QAC Rule 3803: The return value of this function call is not used.
      // This is a wrong QAC warning. The return value of numeric_limits<vfc::uint8_t>::min() is used and assigned.
      // PRQA S 3803 2
      vfc::uint8_t idxMinDistAzi = std::numeric_limits<vfc::uint8_t>::min();
      vfc::uint8_t idxMinDistElev = std::numeric_limits<vfc::uint8_t>::min();

      /* search for nearest angle in azimuth */
      vfc::float32_t dMaxAziFoV = 0.0F;
      if (aziDeg < (vfc::float32_t) FieldOfView_st.thetaViewAry[0])
      {
         dMaxAziFoV = 0.0F;
      }
      else if (aziDeg >(vfc::float32_t) FieldOfView_st.thetaViewAry[static_cast<vfc::uint8_t>(MAXNUMTHETAFOV - 1)])
      {
         dMaxAziFoV = 0.0F;
      }
      else
      {
         for (vfc::uint8_t idxAzi = 0U; idxAzi < MAXNUMTHETAFOV; idxAzi++)
         {
            aziDist = vfc::abs(aziDeg - (vfc::float32_t) FieldOfView_st.thetaViewAry[idxAzi]);
            if (aziDist < aziMinDist)
            {
               aziMinDist = aziDist;
               idxMinDistAzi = idxAzi;
            }
         }
         dMaxAziFoV = (vfc::float32_t) FieldOfView_st.dMaxThetaView[idxMinDistAzi];
      }
      /* search for nearest angle in elevation */
      vfc::float32_t facSDMaxElevFoV = 0.0F;
      if (elevDeg < (vfc::float32_t) FieldOfView_st.phiViewAry[0])
      {
         facSDMaxElevFoV = 0.0F;
      }
      else if (elevDeg >(vfc::float32_t) FieldOfView_st.phiViewAry[static_cast<vfc::uint8_t>(MAXNUMPHIFOV - 1)])
      {
         facSDMaxElevFoV = 0.0F;
      }
      else
      {
         for (vfc::uint8_t idxElev = 0U; idxElev < MAXNUMPHIFOV; idxElev++)
         {
            elevDist = vfc::abs(elevDeg - (vfc::float32_t) FieldOfView_st.phiViewAry[idxElev]);
            if (elevDist < elevMinDist)
            {
               elevMinDist = elevDist;
               idxMinDistElev = idxElev;
            }
         }
         facSDMaxElevFoV = (vfc::float32_t)FieldOfView_st.facDMaxPhiView[idxMinDistElev];
      }
      /* calculate antenna FoV without modulation limitation*/
      vfc::float32_t dMaxFoV = (dMaxAziFoV * facSDMaxElevFoV);

      return((vfc::float32_t) std::min((vfc::float32_t) ModulationPerformance_st.dmax, (vfc::float32_t) dMaxFoV));
   };

   /**
   \brief getter function for FoV range depending on azimuth angle, elevation angle (in cone angles) and RCS
          assumtions: constant noise in d, input angles in sensor coordinates (cone angle:
          [x,y,z] = [r*sqrt(1-sin^2(theta)-sin^2(phi)), r*sin(theta), r*sin(phi)]) without misaglinment

   \param angleAzimuth           in: azimuth angle (cone angle), phys. unit: rad
   \param angleElevation         in: elevation angle (cone angle), phys. unit: rad
   \param RCSObj                 in: object RCS, phys. unit: m^2

   \return FoV of object, phys. unit: m
   */
   vfc::float32_t getViewDistance(vfc::float32_t angleAzimuth, vfc::float32_t angleElevation, vfc::float32_t RCSObj) const
   {
      vfc::float32_t aziMinDist = std::numeric_limits<vfc::float32_t>::max();
      vfc::float32_t elevMinDist = std::numeric_limits<vfc::float32_t>::max();
      const vfc::float32_t aziDeg = dspSensRadToDeg(angleAzimuth);
      const vfc::float32_t elevDeg = dspSensRadToDeg(angleElevation);
      vfc::float32_t aziDist= 0.0F;
      vfc::float32_t elevDist = 0.0F;
      // QAC Rule 3803: The return value of this function call is not used.
      // This is a wrong QAC warning. The return value of numeric_limits<vfc::uint8_t>::min() is used and assigned.
      // PRQA S 3803 2
      vfc::uint8_t idxMinDistAzi = std::numeric_limits<vfc::uint8_t>::min();
      vfc::uint8_t idxMinDistElev = std::numeric_limits<vfc::uint8_t>::min();

      /* scaling factor RCS */
      vfc::float32_t facDistRCSScale = vfc::sqrt(vfc::sqrt(RCSObj/FOV_RCSREF));
      /* search for nearest angle in azimuth */
      vfc::float32_t dMaxAziFoV = 0.0F;
      if (aziDeg < (vfc::float32_t) FieldOfView_st.thetaViewAry[0])
      {
         dMaxAziFoV = 0.0F;
      }
      else if (aziDeg >(vfc::float32_t) FieldOfView_st.thetaViewAry[static_cast<vfc::uint8_t>(MAXNUMTHETAFOV - 1)])
      {
         dMaxAziFoV = 0.0F;
      }
      else
      {
         for (vfc::uint8_t idxAzi = 0U; idxAzi < MAXNUMTHETAFOV; idxAzi++)
         {
            aziDist = vfc::abs(aziDeg - (vfc::float32_t) FieldOfView_st.thetaViewAry[idxAzi]);
            if (aziDist < aziMinDist)
            {
               aziMinDist = aziDist;
               idxMinDistAzi = idxAzi;
            }
         }
         dMaxAziFoV = (vfc::float32_t) FieldOfView_st.dMaxThetaView[idxMinDistAzi];
      }
      /* search for nearest angle in elevation */
      vfc::float32_t facSDMaxElevFoV = 0.0F;
      if (elevDeg < (vfc::float32_t) FieldOfView_st.phiViewAry[0])
      {
         facSDMaxElevFoV = 0.0F;
      }
      else if (elevDeg >(vfc::float32_t) FieldOfView_st.phiViewAry[static_cast<vfc::uint8_t>(MAXNUMPHIFOV - 1)])
      {
         facSDMaxElevFoV = 0.0F;
      }
      else
      {
         for (vfc::uint8_t idxElev = 0U; idxElev < MAXNUMPHIFOV; idxElev++)
         {
            elevDist = vfc::abs(elevDeg - (vfc::float32_t) FieldOfView_st.phiViewAry[idxElev]);
            if (elevDist < elevMinDist)
            {
               elevMinDist = elevDist;
               idxMinDistElev = idxElev;
            }
         }
         facSDMaxElevFoV = (vfc::float32_t)FieldOfView_st.facDMaxPhiView[idxMinDistElev];
      }
      /* calculate antenna FoV without modulation limitation*/
      vfc::float32_t dMaxFoV = ((dMaxAziFoV * facSDMaxElevFoV) * facDistRCSScale);

      return((vfc::float32_t) std::min((vfc::float32_t) ModulationPerformance_st.dmax,(vfc::float32_t) dMaxFoV));
   };

   /**  \brief DSP operation mode
     *  \verbatim
     *       phys. unit: -
     *  \endverbatim
   */
   vfc::uint8_t OpMode;
   /*! \brief measurement state of cycle
     *        0 : values of measurement in the current cycle invalid
     *        1 : values of measurement in the current cycle valid
     * \verbatim
     *       phys. unit: -
     * \endverbatim
   */
   vfc::uint8_t DataMeasured;
   /*! \brief factor for degradation of average
    *         duty cycle due to thermal degradation
    *         (skip cycles and/or reduce ramps)
    *         1: no degradation
    *         0.5: On-time of modulation and thus MMIC thermal load reduction of 50%
    *         typical value range: [0.25 1]
    * \verbatim
    *    phys. unit: -
    * \endverbatim
   */
   vfc::float16_storage_t facThermalDeg;

   struct DSP_MODULATIONPERFORMANCE_ST ModulationPerformance_st;
   struct DSP_MISALIGNMENT_ST Misalignment_st;
   struct DSP_BLINDNESS_ST Blindness_st;
   struct DSP_INTERFERENCE_ST Interference_st;
   struct DSP_FIELDOFVIEW_ST FieldOfView_st;
};

}

#endif
