/******************************************************************************/
/*! \file  dsp_sensingstatus_data_packet.h
\brief Header for Sensing Status

\author CC-DA/ECR3 Knut Schumacher

*/
/******************************************************************************/
#ifndef DSP_SENSINGSTATUS_H_INCLUDED
#define DSP_SENSINGSTATUS_H_INCLUDED

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

// Norms
/** macro rad to deg x*180/pi */
#define dspSensRadToDeg(x)       x*57.295779513f
// FoV
//! Array size of FoV in azimuth >1
#define MAXNUMTHETAFOV  25
//! Azimuth span for FoV calculation [-THETASPAN THETASPAN]
#define THETASPAN 60
//! RCS reference in m^2 for FoV
#define FOV_RCSREF    (10.0f)

//! Array size of FoV factor in elevation >1
#define MAXNUMPHIFOV  11
//! Elevation span for FoV calculation [-PHISPAN PHISPAN]
#define PHISPAN 15

//! radar coating indicator status = invalid
#define RCI_INDCR_INVALID NULL_UB
//! radar coating indicator status = init state
#define RCI_INDCR_INIT 1
//! radar coating indicator status = valid
#define RCI_INDCR_VALID UINT8_MAX

namespace X123
{

/*! \brief Modulation Performance Structure */
struct DSP_MODULATIONPERFORMANCE_ST
{

public:
	// Constructor
	DSP_MODULATIONPERFORMANCE_ST() : DmpID(0), ModID(0), facDMaxModDur(0.0f), dSprblty(0.0f), vSprblty(0.0f),
		dPrec(0.0f), vPrec(0.0f), dvCov(0.0f), dmin(0.0f), dmax(0.0f), vmin(0.0f), vmax(0.0f)
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
	DSP_MISALIGNMENT_ST() : thetaMalAng(0.0f), thetaMalAngVar(0.0f),
	      phiMalAng(0.0f), phiMalAngVar(0.0f), malStatus(0)
	{ };

   /**
    * \brief estimated misalignment angle in azimuth
    *       Norm: []
    * \verbatim
    *       phys. unit: rad
    * \endverbatim
   */
		vfc::float16_storage_t thetaMalAng;
   /**
    * \brief variance of estimated misalignment angle in azimuth
    *       Norm: []
    * \verbatim
    *       phys. unit: rad^2
    * \endverbatim
   */
	vfc::float16_storage_t thetaMalAngVar;
   /**
    * \brief estimated misalignment angle in elevation
    *       Norm: []
    * \verbatim
    *       phys. unit: rad
    * \endverbatim
   */
	vfc::float16_storage_t phiMalAng;
   /**
    * \brief variance of estimated misalignment angle in elevation
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
};

/*! \brief Blindness Structure */
struct DSP_BLINDNESS_ST
{

public:
	// Constructor
	DSP_BLINDNESS_ST() : mdThetaIndcrSO(0), mdThetaIndcrMO(0), mdPhiIndcr(0), mdRCSIndcrMO(0),
	      nBinsOverDetThdIndcr(0), nRefIndcr(0), maxCrossCorrIndcr(0), mdThetaIndcrSOVldFlg(0), mdThetaIndcrMOVldFlg(0),
	      mdPhiIndcrVldFlg(0), mdRCSIndcrMOVldFlg(0), nBinsOverDetThdIndcrVldFlg(0), nRefIndcrVldFlg(0), maxCrossCorrIndcrVldFlg(0)
	{ };

   /**
    * \brief azimuth angle fit one target model deviation for SO's
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
	vfc::uint16_t mdThetaIndcrSO;
   /**
    * \brief Azimuth angle fit one target model deviation for MO's
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint16_t mdThetaIndcrMO;
   /**
    * \brief  Elevation angle fit one target model deviation indicator
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint16_t mdPhiIndcr;
   /**
    * \brief Radar Cross Section Model Deviation Indicator for MO's
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint16_t mdRCSIndcrMO;
   /**
    * \brief  Number of bins over detection threshold
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint16_t nBinsOverDetThdIndcr;
   /**
    * \brief Number of valid reflexions indicator
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint16_t nRefIndcr;
   /**
    * \brief Standing object model deviation indicator
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */   vfc::uint16_t maxCrossCorrIndcr;
   /**
    * \brief validity flag for mdThetaIndcrSO
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t mdThetaIndcrSOVldFlg;
   /**
    * \brief validity flag for mdThetaIndcrMO
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
	vfc::uint8_t mdThetaIndcrMOVldFlg;
   /**
    * \brief validity flag for mdPhiIndcr
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint8_t mdPhiIndcrVldFlg;
   /**
    * \brief validity flag for mdRCSIndcrMO
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
	vfc::uint8_t mdRCSIndcrMOVldFlg;
   /**
    * \brief validity flag for nBinsOverDetThdIndcr
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
	vfc::uint8_t nBinsOverDetThdIndcrVldFlg;
      /**
    * \brief validity flag for nRefIndcr
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
   vfc::uint16_t nRefIndcrVldFlg;
   /**
    * \brief validity flag for maxCrossCorrIndcr
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */   vfc::uint16_t maxCrossCorrIndcrVldFlg;
};

/*! \brief Interference Structure */
struct DSP_INTERFERENCE_ST
{

public:
	// Constructor
	DSP_INTERFERENCE_ST() : IntfrIndcr(0.0f), IntfrIndcrStatus(0)
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
    * \brief FoV range for 10m^2 target at azimuth angle thetaViewAry[x], elevation angle = 0
    *       Norm: []
    * \verbatim
    *       phys. unit: m
    * \endverbatim
   */
	vfc::float16_storage_t dMaxThetaView[MAXNUMTHETAFOV];
   /**
    * \brief azimuth angle array for FoV
    *       Norm: []
    * \verbatim
    *       phys. unit: deg
    * \endverbatim
   */
	vfc::float16_storage_t thetaViewAry[MAXNUMTHETAFOV];
   /**
    * \brief Range scaling for elevation angle phiViewAry[x], azimut angle = 0
    *       Norm: []
    * \verbatim
    *       phys. unit: -
    * \endverbatim
   */
	vfc::float16_storage_t facDMaxPhiView[MAXNUMPHIFOV];
   /**
    * \brief elevation angle array for FoV
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
	DSP_SENSINGSTATE_DATA_PACKET_ST() : OpMode(0), DataMeasured(0), facThermalDeg(0.0f)
   { };

	/* Access Functions  */
   //#ifdef _MSC_VER
	//#pragma warning(disable: 4100)
   //#endif
 
   /*! \brief getter function for DSP operation mode */
   vfc::uint8_t getOpMode(void)  const { return (OpMode); };
   /*! \brief getter function for measurement state of cycle */
   bool isDataMeasured(void)  const { return (DataMeasured!=0); };
   /** \brief getter function for factor for degradation of average
    *         duty cycle due to thermal degradation
    *         (skip cycles and/or reduce ramps) */
   vfc::float16_storage_t getfacThermalDeg(void) const { return (facThermalDeg); };

   /**
   \brief getter function for FoV range depending on azimuth angle, elevation angle
          assumtions: constant noise in d, input angles in sensor coordinates without misaglinment
         
   \param angleAzimuth           in: azimuth angle, phys. unit: rad
   \param angleElevation         in: elevation angle, phys. unit: rad

   \return FoV of object, phys. unit: m
   */
   vfc::float32_t getViewDistance(vfc::float32_t angleAzimuth, vfc::float32_t angleElevation) const
   {
      vfc::float32_t aziMinDist = std::numeric_limits<vfc::float32_t>::max();
      vfc::float32_t elevMinDist = std::numeric_limits<vfc::float32_t>::max();
      vfc::float32_t aziDist, elevDist;
      vfc::float32_t m_dMaxFoV = 0.0f;
      vfc::uint8_t idxMinDistAzi = std::numeric_limits<vfc::uint8_t>::min();
      vfc::uint8_t idxMinDistElev = std::numeric_limits<vfc::uint8_t>::min();

      /* search for nearest angle in azimuth */
      for(vfc::uint8_t idxAzi = 0; idxAzi<MAXNUMTHETAFOV; idxAzi++)
      {
        aziDist = vfc::abs(dspSensRadToDeg(angleAzimuth)-FieldOfView_st.thetaViewAry[idxAzi]);
        if (aziDist<aziMinDist)
           {
              aziMinDist = aziDist;
              idxMinDistAzi = idxAzi;
           }
      }
      /* search for nearest angle in elevation */
      for(vfc::uint8_t idxElev = 0; idxElev<MAXNUMPHIFOV; idxElev++)
      {
        elevDist = vfc::abs(dspSensRadToDeg(angleElevation)-FieldOfView_st.phiViewAry[idxElev]);
        if (elevDist<elevMinDist)
           {
              elevMinDist = elevDist;
              idxMinDistElev = idxElev;
           }
      }
      /* calculate unlimited FoV */
      m_dMaxFoV = ((vfc::float32_t)FieldOfView_st.dMaxThetaView[idxMinDistAzi] * (vfc::float32_t)FieldOfView_st.facDMaxPhiView[idxMinDistElev]);

      return((vfc::float32_t) std::min((vfc::float32_t) ModulationPerformance_st.dmax,(vfc::float32_t) m_dMaxFoV)); 
   };

   /**
   \brief getter function for FoV range depending on azimuth angle, elevation angle and RCS
          assumtions: constant noise in d, input angles in sensor coordinates without misaglinment
         
   \param angleAzimuth           in: azimuth angle, phys. unit: rad
   \param angleElevation         in: elevation angle, phys. unit: rad
   \param RCSObj                 in: object RCS, phys. unit: m^2

   \return FoV of object, phys. unit: m
   */
   vfc::float32_t getViewDistance(vfc::float32_t angleAzimuth, vfc::float32_t angleElevation, vfc::float32_t RCSObj) const
   {
      vfc::float32_t aziMinDist = std::numeric_limits<vfc::float32_t>::max();
      vfc::float32_t elevMinDist = std::numeric_limits<vfc::float32_t>::max();
      vfc::float32_t aziDist, elevDist;
      vfc::float32_t m_dMaxFoV = 0.0f;
      vfc::uint8_t idxMinDistAzi = std::numeric_limits<vfc::uint8_t>::min();
      vfc::uint8_t idxMinDistElev = std::numeric_limits<vfc::uint8_t>::min();

      /* scaling factor RCS */
      vfc::float32_t m_facDistRCSScale = vfc::sqrt(vfc::sqrt(RCSObj/FOV_RCSREF));
      /* search for nearest angle in azimuth */
      for(vfc::uint8_t idxAzi = 0; idxAzi<MAXNUMTHETAFOV; idxAzi++)
      {
        aziDist = vfc::abs(dspSensRadToDeg(angleAzimuth)-FieldOfView_st.thetaViewAry[idxAzi]);
        if (aziDist<aziMinDist)
           {
              aziMinDist = aziDist;
              idxMinDistAzi = idxAzi;
           }
      }
      /* search for nearest angle in elevation */
      for(vfc::uint8_t idxElev = 0; idxElev<MAXNUMPHIFOV; idxElev++)
      {
        elevDist = vfc::abs(dspSensRadToDeg(angleElevation)-FieldOfView_st.phiViewAry[idxElev]);
        if (elevDist<elevMinDist)
           {
              elevMinDist = elevDist;
              idxMinDistElev = idxElev;
           }
      }
      /* calculate unlimited FoV */
      m_dMaxFoV = (((vfc::float32_t)FieldOfView_st.dMaxThetaView[idxMinDistAzi] * (vfc::float32_t)FieldOfView_st.facDMaxPhiView[idxMinDistElev]) * (vfc::float32_t) m_facDistRCSScale);

      return((vfc::float32_t) std::min((vfc::float32_t) ModulationPerformance_st.dmax,(vfc::float32_t) m_dMaxFoV)); 
   };
   //#ifdef _MSC_VER
	//#pragma warning(default: 4100)
   //#endif

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
