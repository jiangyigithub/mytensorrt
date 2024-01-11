//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2008 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy or use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  P R O J E C T   I N F O R M A T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc
//  Target system(s):
//       Compiler(s): c++ std conformal
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: ves3kor
//  Department:
//=============================================================================
//  F I L E   C O N T E N T S   A N D   R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @brief This is a brief description.
/// @par Synopsis:
///     This is the detailed description.
///
/// @par Revision History:
///     $Source: vfc_siunits_types.hpp $
///     $Revision: 1.10 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/09/19 15:56:27MESZ $
///     $Locker:  $
///     $Name:  $
///     $State: in_work $
///
/// @par Review Information:
/// - Reviewed version:
/// - Type (use 'X' to mark):
///     - [ ] Formal Review
///     - [ ] Walkthrough
///     - [ ] Inspection
/// - State including date (DD.MM.YYYY)
///     - [--.--.----] Preparation
///     - [--.--.----] Review audit
///     - [--.--.----] Integration of findings
///     - [--.--.----] Test
///     - [--.--.----] Verification of integration of findings
///     - [--.--.----] Review release
/// - Responsible:
/// - Review-Document:
//=============================================================================

#ifndef VFC_SIUNITS_TYPES_HPP_INCLUDED
#define VFC_SIUNITS_TYPES_HPP_INCLUDED

#include "vfc/core/vfc_siunits.hpp"

namespace vfc
{

    //=============================================================================
    //Base quantity length
    //=============================================================================
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::NANO, 1,             //Length
        vfc::CMassType, vfc::BASE, 0,               //Mass
        vfc::CTimeType, vfc::BASE, 0,               //Time
        vfc::CCurrentType, vfc::BASE, 0,            //Current
        vfc::CTemperatureType, vfc::BASE, 0,        //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,  //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,  //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,              //Angle
        vfc::CBasicType>                            info_nano_metre_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::MICRO, 1,                //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_micro_metre_t;


    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::MILLI, 1,                //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_milli_metre_t;


    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::CENTI, 1,                //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_centi_metre_t;


    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::DECI, 1,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_deci_metre_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 1,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_metre_t;


    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::KILO, 1,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_kilo_metre_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::MEGA, 1,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_mega_metre_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::GIGA, 1,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_giga_metre_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::TERA, 1,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_tera_metre_t;


    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, -1,                //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_per_metre_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, -1,                //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, -1,                  //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_per_metre_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, -2,                //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_per_square_metre_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, -2,                //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, -1,                  //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_per_square_metre_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 2,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, -2,                  //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_square_metre_per_square_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 2,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, -4,                  //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_square_metre_per_quartic_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 1,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 1,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_metre_second_t;

    struct CMileType
    {
        typedef CLength si_base_unit_type;
        enum
        {
            CONVERSION_RATIONAL = 0,
            FLOATING_OFFSET     = 0,
            NUM = 201168,
            DEN = 125,
            OFFSET = 0
        };

        template<class ValueType>
        inline static void doMultiply(ValueType& f_value_r)
        {
            f_value_r *= static_cast<ValueType>(1609.344);
        }

        template<class ValueType>
        inline static void doDivide(ValueType& f_value_r)
        {
            f_value_r /= static_cast<ValueType>(1609.344);
        }
    };

    typedef vfc::TUnitInfoType<
        vfc::CMileType, vfc::BASE, 1,                   //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_mile_t;


    //=============================================================================
    //Base quantity area
    //=============================================================================
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 2,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_square_metre_t;

    //=============================================================================
    //Base quantity volume
    //=============================================================================
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 3,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_cubic_metre_t;


    // KiliMtr 2
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::KILO, 2,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_square_kilo_metre_t;


    // KiliMtr 3
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::KILO, 3,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_cubic_kilo_metre_t;

    //cubic decimetre = litre
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::DECI, 3,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_cubic_deci_metre_t;
    
    // litre = dm ^ 3
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::DECI, 3,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_litre_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::DECI, 3,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::HOUR, -1,          //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_litre_per_hour_t;

    //=============================================================================
    //Base quantity time
    //=============================================================================

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::NANO_SECOND, 1,    //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_nano_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::MICRO_SECOND, 1,   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_micro_second_t;


    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::MILLI_SECOND, 1,   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_milli_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, 1,         //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, 2,         //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_square_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::MINUTE, 1,         //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_minute_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::HOUR, 1,           //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_hour_t;

    
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::DAY, 1,            //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_day_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::WEEK, 1,            //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_week_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::MONTH, 1,           //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_month_t;

    
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::YEAR, 1,           //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_year_t;


    //=============================================================================
    //Base quantity 1/time - hertz
    //=============================================================================

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::NANO_SECOND, -1,   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_per_nano_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::MICRO_SECOND, -1,  //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_per_micro_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::MILLI_SECOND, -1,  //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_per_milli_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, -1,        //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_per_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, -2,        //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_per_square_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::MINUTE, -1,        //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_per_minute_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::HOUR, -1,          //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_per_hour_t;


    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::HOUR, -2,          //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_per_square_hour_t;

    //=============================================================================
    //Base quantity angle
    //=============================================================================
    struct CSIDegreeType
    {
        typedef CAngle  si_base_unit_type;

        enum
        {
            CONVERSION_RATIONAL = 0,
            FLOATING_OFFSET = 0
        };

        // Good numerical approximation for Pi. The 355/113 approximation gives 7digits of precision which covers the float32 type (3.141593).
        // For float64 16digits are required, the value should be 3.141592653589793 (245850922 / 78256779)
        // Unfortunately 78256779 * 180 overflows, the smallest value possible is then 1725033 which gives only 13digits of precision (5419351 / 1725033)
        static const vfc::int32_t NUM = 355;
        static const vfc::int32_t DEN = (113 * 180);
        static const vfc::int32_t OFFSET = 0;
        
        template<class ValueType>
        inline static void doMultiply(ValueType& f_value_r)
        {
            // degree to radian conversion of non float types is not supported due to extreme rounding errors
            VFC_STATIC_ASSERT(TIsFloating<ValueType>::value);
            f_value_r *= static_cast<ValueType>(G_DEG2RAD);
        }

        template<class ValueType>
        inline static void doDivide(ValueType& f_value_r)
        {
            // radian to degree conversion of non float types is not supported due to extreme rounding errors
            VFC_STATIC_ASSERT(TIsFloating<ValueType>::value);
            f_value_r /= static_cast<ValueType>(G_DEG2RAD);
        }
    };

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 1,                  //Angle
        vfc::CBasicType>                                info_radian_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CSIDegreeType, vfc::BASE, 1,               //Angle
        vfc::CBasicType>                                info_degree_t;

    
   typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, -1,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CSIDegreeType, vfc::BASE, 1,               //Angle
        vfc::CBasicType>                                info_degree_per_second_t;

   typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, -2,                  //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CSIDegreeType, vfc::BASE, 2,               //Angle
        vfc::CBasicType>                                info_degree_square_per_second_square_t;


    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 2,                  //Angle
        vfc::CBasicType>                                info_square_radian_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CSIDegreeType, vfc::BASE, 2,               //Angle
        vfc::CBasicType>                                info_square_degree_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, -1,                  //Angle
        vfc::CBasicType>                                info_per_radian_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, -2,                  //Angle
        vfc::CBasicType>                                info_per_square_radian_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, -1,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 1,                  //Angle
        vfc::CBasicType>                                info_radian_per_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, -2,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 1,                  //Angle
        vfc::CBasicType>                                info_radian_per_square_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, -2,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 2,                  //Angle
        vfc::CBasicType>                                info_square_radian_per_square_second_t;

    //=============================================================================
    //Base quantity mass
    //=============================================================================
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 1,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_gram_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::KILO, 1,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_kilo_gram_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::MEGA, 1,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_ton_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::MILLI, 1,                  //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_milli_gram_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::MICRO, 1,                  //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_micro_gram_t;

    //=============================================================================
    //Base quantity temperature
    //=============================================================================
    struct CCelsiusType
    {
        typedef CTemperature  si_base_unit_type;

        enum
        {
            CONVERSION_RATIONAL = 1,
            FLOATING_OFFSET = 0
        };

        static const vfc::int32_t NUM = 1;
        static const vfc::int32_t DEN = 1;
        static const vfc::int32_t OFFSET = 273;

   };

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 1,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_kelvin_t;


    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CCelsiusType, vfc::BASE, 1,                //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_celsius_t;

    //=============================================================================
    //Base quantity Ampere
    //=============================================================================

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::NANO, 1,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_nano_ampere_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::MICRO, 1,               //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_micro_ampere_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::MILLI, 1,               //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_milli_ampere_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::CENTI, 1,               //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_centi_ampere_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::DECI, 1,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_deci_ampere_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 1,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_ampere_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::KILO, 1,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_kilo_ampere_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::MEGA, 1,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_mega_ampere_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::GIGA, 1,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_giga_ampere_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::TERA, 1,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_tera_ampere_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, -1,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_per_ampere_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 2,                 //Length
        vfc::CMassType, vfc::BASE, 1,                   //Mass
        vfc::CTimeType, vfc::BASE, -3,                  //Time
        vfc::CCurrentType, vfc::BASE, -1,               //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_volt_t;

//    typedef vfc::TUnitInfoType<
//        vfc::CLengthType, vfc::BASE, 2,                 //Length
//        vfc::CMassType, vfc::BASE, 1,                   //Mass
//        vfc::CTimeType, vfc::BASE, -3,                  //Time
//        vfc::CCurrentType, vfc::BASE, -1,               //Current
//        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
//        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
//        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
//        vfc::CAngleType, vfc::BASE, 0,                  //Angle
//        vfc::CBasicType>                                info_milli_volt_t;


    //=============================================================================
    //Base quantity Candela ( cd )
    //=============================================================================

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::NANO, 1,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_nano_candela_t;


    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::MICRO, 1,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_micro_candela_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::MILLI, 1,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_milli_candela_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::CENTI, 1,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_centi_candela_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::DECI, 1,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_deci_candela_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 1,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_candela_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::KILO, 1,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_kilo_candela_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::MEGA, 1,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_mega_candela_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::GIGA, 1,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_giga_candela_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::TERA, 1,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_tera_candela_t;




    // 1 / cd
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, -1,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_per_candela_t;

    //Newton = mass * acceleration
    //Newton = Kg * m/s2
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 1,                 //Length
        vfc::CMassType, vfc::KILO, 1,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, -2,        //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_kilo_gram_metre_per_square_second_t;

    //Newton = mass * acceleration
    //Newton = Kg * m/s2
   typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 1,                 //Length
        vfc::CMassType, vfc::KILO, 1,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, -2,        //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_newton_t;
   
   // same as newton
   typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 1,                 //Length
        vfc::CMassType, vfc::KILO, 1,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, -2,        //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_force_of_gravity_t;

    
    // Killo Newton = 10^3 Kg m/s2  = Mg.m/s2
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 1,                 //Length
        vfc::CMassType, vfc::MEGA, 1,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, -2,        //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_kilo_newton_t;
    
    // joule ( KG * (m2/s2) ) == N M
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 2,                 //Length
        vfc::CMassType, vfc::KILO, 1,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, -2,        //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_joule_t;
    // J = Nm
    typedef info_joule_t                                info_newton_metre_t;

    // watt = Joule/second = N m / s
    // watt = Kg * m2 / s3
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 2,                 //Length
        vfc::CMassType, vfc::KILO, 1,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, -3,        //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_watt_t;

    // kilowatt = kilo* Kg * m2 / s3
    // kilowatt = Mg * m2 / s3
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 2,                 //Length
        vfc::CMassType, vfc::MEGA, 1,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, -3,        //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_kilo_watt_t;

     
    //pascal=force/area= kg / (m s2) 
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, -1,                 //Length
        vfc::CMassType, vfc::KILO, 1,                    //Mass
        vfc::CTimeType, vfc::BASE, -2,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_pascal_t;

    //10^5Kg/(m.s^2) = 10^9g/(dm.s^2) = Bar
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::DECA, -1,                 //Length
        vfc::CMassType, vfc::GIGA, 1,                    //Mass
        vfc::CTimeType, vfc::BASE, -2,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_bar_t;

    // bar = 10^9g/(dm.s^2)
    // milli bar = 10^-3*bar = 10^6g/(dm.s^2)
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::DECA, -1,                 //Length
        vfc::CMassType, vfc::MEGA, 1,                    //Mass
        vfc::CTimeType, vfc::BASE, -2,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_milli_bar_t;


    // 1 lx = 1 lm·m-2 = 1 cd· sr·m–2
    // sr = steradian
    // The steradian is dimensionless because 1 sr = m2·m-2 = 1
    // assumed steradian = 1
    // http://en.wikipedia.org/wiki/Lux
    // http://en.wikipedia.org/wiki/Steradian

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, -2,                //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 1,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_lux_t;


    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 1,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, -1,        //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_metre_per_second_t;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::KILO, 1,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::HOUR, -1,          //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_kilometre_per_hour_t;

    typedef vfc::TUnitInfoType<
        vfc::CMileType, vfc::BASE, 1,                   //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::HOUR, -1,          //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_miles_per_hour_t;


    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 1,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, -2,        //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_metre_per_square_second_t;

    // gravity "g" = 9.81m/s^-2
    struct CGraviAccelScaleType
    {
        typedef CLength  si_base_unit_type;

        enum
        {
            CONVERSION_RATIONAL = 1,
            FLOATING_OFFSET = 0
        };

        static const vfc::int32_t NUM = 981;
        static const vfc::int32_t DEN = 100;
        static const vfc::int32_t OFFSET = 0;

    };

 typedef vfc::TUnitInfoType<
        vfc::CGraviAccelScaleType, vfc::BASE, 1,        //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, -2,        //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_gravitational_accel_t;

   typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::KILO, 1,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::HOUR, -2,          //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_kilometre_per_square_hour_t;

   typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 1,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::sitime::SECOND, -3,        //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_metre_per_cubic_second_t;

    //=============================================================================
    //Base quantity slope
    //=============================================================================
    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,                 //Length
        vfc::CMassType, vfc::BASE, 0,                   //Mass
        vfc::CTimeType, vfc::BASE, 0,                   //Time
        vfc::CCurrentType, vfc::BASE, 0,                //Current
        vfc::CTemperatureType, vfc::BASE, 0,            //Temperature
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,      //AmountOfSubstance
        vfc::CLuminousIntensityType, vfc::BASE, 0,      //LuminousIntensity
        vfc::CAngleType, vfc::BASE, 0,                  //Angle
        vfc::CBasicType>                                info_slope_t;

    //=============================================================================
    //Base quantity pixel
    //=============================================================================
    typedef vfc::TRBInfoType<
        vfc::CPixelType, vfc::BASE, 1,             //Pixel
        vfc::CPercentageType, vfc::BASE, 0,         //Percentage
        vfc::CBasicType>                            info_pixel_t;

    typedef vfc::TRBInfoType<
        vfc::CPixelType, vfc::BASE, 2,             //Pixel
        vfc::CPercentageType, vfc::BASE, 0,         //Percentage
        vfc::CBasicType>                            info_square_pixel_t;

    typedef vfc::TRBInfoType<
        vfc::CPixelType, vfc::BASE, -1,             //Pixel
        vfc::CPercentageType, vfc::BASE, 0,         //Percentage
        vfc::CBasicType>                            info_per_pixel_t;

    typedef vfc::TRBInfoType<
        vfc::CPixelType, vfc::BASE, -2,             //Pixel
        vfc::CPercentageType, vfc::BASE, 0,         //Percentage
        vfc::CBasicType>                            info_per_square_pixel_t;

    typedef vfc::TRBInfoType<
        vfc::CPixelType, vfc::BASE, 0,             //Pixel
        vfc::CPercentageType, vfc::BASE, 1,        //Percentage
        vfc::CBasicType>                            info_percentage_t;


}


namespace vfc
{   // open namespace vfc

    //============================================================
    //SIUnits common typedefs
    //============================================================

    template<class ValueType, class UserType = vfc::CDefaultType,
                                    template<class, class, class> class ConvertPolicyType = vfc::TConvert>
    struct TSIUnitType
    {

        ///////////////////////////////////////////////
        ///////////////// length /////////////////////
        ///////////////////////////////////////////////

        // power = 1

        typedef vfc::TSIUnits<ValueType, info_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_metre_t;


        typedef vfc::TSIUnits<ValueType, info_kilo_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_kilo_metre_t;

        typedef vfc::TSIUnits<ValueType, info_mega_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_mega_metre_t;

        typedef vfc::TSIUnits<ValueType, info_giga_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_giga_metre_t;

        typedef vfc::TSIUnits<ValueType, info_tera_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_tera_metre_t;

        typedef vfc::TSIUnits<ValueType, info_deci_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_deci_metre_t;

        typedef vfc::TSIUnits<ValueType, info_centi_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_centi_metre_t;

        typedef vfc::TSIUnits<ValueType, info_milli_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_milli_metre_t;

        typedef vfc::TSIUnits<ValueType, info_micro_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_micro_metre_t;

        typedef vfc::TSIUnits<ValueType, info_nano_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_nano_metre_t;

        typedef vfc::TSIUnits<ValueType, info_metre_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_metre_second_t;

        typedef vfc::TSIUnits<ValueType, info_mile_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_mile_t;

        // power = 2

        typedef vfc::TSIUnits<ValueType, info_square_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_square_metre_t;

        typedef vfc::TSIUnits<ValueType, info_square_kilo_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_square_kilo_metre_t;

        typedef vfc::TSIUnits<ValueType, info_cubic_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_cubic_metre_t;

        typedef vfc::TSIUnits<ValueType, info_cubic_kilo_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_cubic_kilo_metre_t;

        typedef vfc::TSIUnits<ValueType, info_cubic_deci_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_cubic_deci_metre_t;
        
        //litre = cubic decimetre
        typedef vfc::TSIUnits<ValueType, info_litre_t, info_rbnil_type,
                UserType, ConvertPolicyType>                                           si_litre_t;

        typedef vfc::TSIUnits<ValueType, info_litre_per_hour_t, info_rbnil_type,
                UserType, ConvertPolicyType>                                           si_litre_per_hour_t;

        typedef vfc::TSIUnits<ValueType, info_square_metre_per_square_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_square_metre_per_square_second_t;

        typedef vfc::TSIUnits<ValueType, info_square_metre_per_quartic_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_square_metre_per_quartic_second_t;

        typedef vfc::TSIUnits<ValueType, info_per_metre_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_per_metre_second_t;

        typedef vfc::TSIUnits<ValueType, info_per_square_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_per_square_metre_t;

        typedef vfc::TSIUnits<ValueType, info_per_square_metre_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_per_square_metre_second_t;

        typedef vfc::TSIUnits<ValueType, info_per_metre_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_per_metre_t;

        ///////////////////////////////////////////////
        ///////////////// time /////////////////////
        ///////////////////////////////////////////////

        // power = 1

        typedef vfc::TSIUnits<ValueType, info_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_second_t;

        typedef vfc::TSIUnits<ValueType, info_minute_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_minute_t;

        typedef vfc::TSIUnits<ValueType, info_hour_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_hour_t;

        typedef vfc::TSIUnits<ValueType, info_milli_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_milli_second_t;

        typedef vfc::TSIUnits<ValueType, info_micro_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_micro_second_t;
        
        typedef vfc::TSIUnits<ValueType, info_nano_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_nano_second_t;

        // 1 / s
        typedef vfc::TSIUnits<ValueType, info_per_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_per_second_t;

        //  1 / hr
        typedef vfc::TSIUnits<ValueType, info_per_hour_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_per_hour_t;


        //  1 / s2
        typedef vfc::TSIUnits<ValueType, info_per_square_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_per_square_second_t;

        //  s2
        typedef vfc::TSIUnits<ValueType, info_square_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_square_second_t;

        //  1 / hr2
        typedef vfc::TSIUnits<ValueType, info_per_square_hour_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_per_square_hour_t;

        typedef vfc::TSIUnits<ValueType, info_day_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_day_t;

        typedef vfc::TSIUnits<ValueType, info_week_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_week_t;


        typedef vfc::TSIUnits<ValueType, info_month_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_month_t;

        typedef vfc::TSIUnits<ValueType, info_year_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_year_t;

        typedef vfc::TSIUnits<ValueType, info_per_minute_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_per_minute_t;



        ///////////////////////////////////////////////
        ///////////////// Degree /////////////////////
        ///////////////////////////////////////////////

        typedef vfc::TSIUnits<ValueType, info_degree_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                         si_degree_t;
        
        typedef vfc::TSIUnits<ValueType, info_degree_per_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                         si_degree_per_second_t;

        typedef vfc::TSIUnits<ValueType, info_degree_square_per_second_square_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                         si_degree_square_per_second_square_t;

        typedef vfc::TSIUnits<ValueType, info_radian_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                         si_radian_t;

        typedef vfc::TSIUnits<ValueType, info_square_radian_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                         si_square_radian_t;

        typedef vfc::TSIUnits<ValueType, info_square_degree_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                         si_square_degree_t;

        typedef vfc::TSIUnits<ValueType, info_per_square_radian_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                         si_per_square_radian_t;

        typedef vfc::TSIUnits<ValueType, info_per_radian_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                         si_per_radian_t;

        typedef vfc::TSIUnits<ValueType, info_radian_per_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                         si_radian_per_second_t;

        typedef vfc::TSIUnits<ValueType, info_radian_per_square_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                         si_radian_per_square_second_t;

        typedef vfc::TSIUnits<ValueType, info_square_radian_per_square_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                         si_square_radian_per_square_second_t;


        ///////////////////////////////////////////////
        ///////////////// Mass /////////////////////
        ///////////////////////////////////////////////

        typedef vfc::TSIUnits<ValueType, info_gram_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_gram_t;

        typedef vfc::TSIUnits<ValueType, info_kilo_gram_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_kilo_gram_t;

        ///////////////////////////////////////////////
        ///////////////// temperature /////////////////////
        ///////////////////////////////////////////////

        //  temperature based
        typedef vfc::TSIUnits<ValueType, info_kelvin_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_kelvin_t;

        typedef vfc::TSIUnits<ValueType, info_celsius_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_celsius_t;

        ///////////////////////////////////////////////
        ///////////////// ampere /////////////////////
        ///////////////////////////////////////////////

        typedef vfc::TSIUnits<ValueType, info_ampere_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_ampere_t;

        typedef vfc::TSIUnits<ValueType, info_kilo_ampere_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_kilo_ampere_t;

        typedef vfc::TSIUnits<ValueType, info_mega_ampere_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_mega_ampere_t;

        typedef vfc::TSIUnits<ValueType, info_giga_ampere_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_giga_ampere_t;

        typedef vfc::TSIUnits<ValueType, info_tera_ampere_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_tera_ampere_t;

        typedef vfc::TSIUnits<ValueType, info_deci_ampere_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_deci_ampere_t;

        typedef vfc::TSIUnits<ValueType, info_centi_ampere_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_centi_ampere_t;

        typedef vfc::TSIUnits<ValueType, info_milli_ampere_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_milli_ampere_t;

        typedef vfc::TSIUnits<ValueType, info_micro_ampere_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_micro_ampere_t;

        typedef vfc::TSIUnits<ValueType, info_nano_ampere_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_nano_ampere_t;

        // 1 / A
        typedef vfc::TSIUnits<ValueType, info_per_ampere_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_per_ampere_t;

        // V (Volt)
        typedef vfc::TSIUnits<ValueType, info_volt_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_volt_t;
            
//        typedef vfc::TSIUnits<ValueType, info_milli_volt_t, info_rbnil_type,
//            UserType, ConvertPolicyType>                                           si_milli_volt_t;


        ///////////////////////////////////////////////
        ///////////////// candela /////////////////////
        ///////////////////////////////////////////////

        typedef vfc::TSIUnits<ValueType, info_candela_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_candela_t;

        typedef vfc::TSIUnits<ValueType, info_kilo_candela_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_kilo_candela_t;

        typedef vfc::TSIUnits<ValueType, info_mega_candela_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_mega_candela_t;

        typedef vfc::TSIUnits<ValueType, info_giga_candela_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_giga_candela_t;

        typedef vfc::TSIUnits<ValueType, info_tera_candela_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_tera_candela_t;

        typedef vfc::TSIUnits<ValueType, info_deci_candela_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_deci_candela_t;

        typedef vfc::TSIUnits<ValueType, info_centi_candela_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_centi_candela_t;

        typedef vfc::TSIUnits<ValueType, info_milli_candela_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_milli_candela_t;

        typedef vfc::TSIUnits<ValueType, info_micro_candela_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_micro_candela_t;

        typedef vfc::TSIUnits<ValueType, info_nano_candela_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_nano_candela_t;

        // 1 / cd
        typedef vfc::TSIUnits<ValueType, info_per_candela_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_per_candela_t;


        ///////////////////////////////////////////////////////////////////////////
        //////////////////// Commonly used types //////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////

        // Hz
        typedef     si_per_second_t                             si_hertz_t ;

        
        typedef vfc::TSIUnits<ValueType, info_kilo_gram_metre_per_square_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                   si_kilo_gram_metre_per_square_second_t;
        
        typedef vfc::TSIUnits<ValueType, info_newton_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_newton_t;
        
        typedef vfc::TSIUnits<ValueType, info_force_of_gravity_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_force_of_gravity_t;

        typedef vfc::TSIUnits<ValueType, info_kilo_newton_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_kilo_newton_t;

        typedef vfc::TSIUnits<ValueType, info_joule_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_joule_t;

        typedef vfc::TSIUnits<ValueType, info_watt_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_watt_t;

        typedef vfc::TSIUnits<ValueType, info_kilo_watt_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_kilo_watt_t;

        typedef vfc::TSIUnits<ValueType, info_bar_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_bar_t;

        
        typedef vfc::TSIUnits<ValueType, info_milli_bar_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_milli_bar_t;

        typedef vfc::TSIUnits<ValueType, info_pascal_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_pascal_t;

        typedef vfc::TSIUnits<ValueType, info_lux_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_lux_t;
  
        // newton metre
        typedef     si_joule_t      si_newton_metre_t ;

        //  velocity
        typedef vfc::TSIUnits<ValueType, info_metre_per_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_metre_per_second_t;


        typedef vfc::TSIUnits<ValueType, info_kilometre_per_hour_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_kilometre_per_hour_t;

        typedef vfc::TSIUnits<ValueType, info_miles_per_hour_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_miles_per_hour_t;

        //  acceleration
        typedef vfc::TSIUnits<ValueType, info_metre_per_square_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_metre_per_square_second_t;

        typedef vfc::TSIUnits<ValueType, info_gravitational_accel_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_gravitational_accel_t;

        typedef vfc::TSIUnits<ValueType, info_kilometre_per_square_hour_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_kilometre_per_square_hour_t;

        //metre_per_cubic_second
        typedef vfc::TSIUnits<ValueType, info_metre_per_cubic_second_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_metre_per_cubic_second_t;

        // wave number
         typedef vfc::TSIUnits<ValueType, info_per_metre_t, info_rbnil_type,
             UserType, ConvertPolicyType>                                          si_wavenumber_t;


        ///////////////////////////////////////////////
        ///////////////// slope /////////////////////
        ///////////////////////////////////////////////

        typedef vfc::TSIUnits<ValueType, info_slope_t, info_rbnil_type,
            UserType, ConvertPolicyType>                                           si_slope_t;

        ///////////////////////////////////////////////
        ///////////////// pixel /////////////////////
        ///////////////////////////////////////////////

        typedef vfc::TSIUnits<ValueType, info_nil_type, info_pixel_t,
            UserType, ConvertPolicyType>                                     si_pixel_t;

        typedef vfc::TSIUnits<ValueType, info_nil_type, info_square_pixel_t,
            UserType, ConvertPolicyType>                                     si_square_pixel_t;

        typedef vfc::TSIUnits<ValueType, info_nil_type, info_per_pixel_t,
            UserType, ConvertPolicyType>                                     si_per_pixel_t;

        typedef vfc::TSIUnits<ValueType, info_nil_type, info_per_square_pixel_t,
            UserType, ConvertPolicyType>                                     si_per_square_pixel_t;

        typedef vfc::TSIUnits<ValueType, vfc::info_per_radian_t, vfc::info_pixel_t,
            UserType, ConvertPolicyType>                                      si_pixel_per_radian_t;
        
        typedef vfc::TSIUnits<ValueType, info_nil_type, info_percentage_t,
            UserType, ConvertPolicyType>                                      si_percent_t;

        typedef vfc::TSIUnits<ValueType, info_per_second_t, info_percentage_t,
            UserType, ConvertPolicyType>                                      si_percent_per_second_t;

        typedef vfc::TSIUnits<ValueType, info_force_of_gravity_t, info_percentage_t,
            UserType, ConvertPolicyType>                                      si_percent_of_force_of_gravity_t;

        // nil type
        typedef vfc::TSIUnits<ValueType, vfc::info_nil_type, info_rbnil_type,
            UserType, ConvertPolicyType>                                          si_nil_t;


    };

    template < class UserType = vfc::CDefaultType,
                                    template<class , class, class> class ConvertPolicyType = vfc::TConvert>
    class TSIUnitType_i32 : public TSIUnitType<vfc::int32_t, UserType, ConvertPolicyType>
    {

    };

    template <class UserType = vfc::CDefaultType,
                                    template<class , class, class> class ConvertPolicyType = vfc::TConvert>
    class TSIUnitType_f32 : public TSIUnitType<vfc::float32_t, UserType, ConvertPolicyType>
    {

    };

    template <class UserType = vfc::CDefaultType,
                                    template<class , class, class> class ConvertPolicyType = vfc::TConvert>
    class TSIUnitType_f64 : public TSIUnitType<vfc::float64_t, UserType, ConvertPolicyType>
    {

    };

    //=========================================================================
    // TRectAreaTypeTraits
    //-------------------------------------------------------------------------
    //! Specialization of trait to identify the type promotion for SIunits.
    //! @author gaj2kor
    //! @ingroup vfc_group_core_types
    //=========================================================================
    template <  class ValueType,
                class UnitInfoType,
                class RBInfoType,     
                class UserType,
                template<class, class, class> class ConvertPolicyType>
    struct TRectAreaTypeTraits<
    vfc::TSIUnits<ValueType,UnitInfoType,RBInfoType,UserType,ConvertPolicyType>
    >
    {
        VFC_STATIC_ASSERT(!(vfc::TIsUnsignedArithmetic<ValueType>::value));
    
        typedef typename vfc::TSIUnits<
                                ValueType,UnitInfoType,
                                RBInfoType,UserType,ConvertPolicyType>::sqr_promoted_type area_type;
   
    };

}// closed namespace vfc

#endif //VFC_SIUNITS_TYPES_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_siunits_types.hpp  $
//  Revision 1.11 2016/07/06 07:29:29MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Add SI units for miles and miles per hour (mantis0005173)
//  Revision 1.10 2014/09/19 15:56:27MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_siunits: correct the Degree/Radian handling (mantis0004533)
//  Revision 1.9 2014/01/28 11:50:29MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_siunits: replace tabs by 4 spaces (mantis0004394)
//  Revision 1.8 2013/11/08 14:19:21MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  -  Provide SIUnits convenience types in more than just float32 (mantis4186)
//  Revision 1.7 2013/05/15 09:55:26MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - add two more si unit conventient types metre_pixel, radian_per_square_second (mantis4174)
//  Revision 1.6 2013/05/15 08:41:11MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - add further si unit types per_square_metre_second, per_metre_second (mantis4249)
//  Revision 1.5 2012/12/18 08:27:32MEZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.4 2012/10/24 15:52:11IST Sudhakar Nannapaneni (RBEI/ESD1) (SNU5KOR) 
//  -Added support for the new module TRect under vfc::core (mantis3349)
//  Revision 1.3 2012/08/17 11:53:09IST Gaurav Jain (RBEI/ESD1) (gaj2kor) 
//  - Added info_square_second_t in vfc_siunits_types.hpp. (mantis 4162)
//  Revision 1.2 2010/09/13 20:13:53IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - si units: add support for units used in Volkswagen AG DBC files (mantis3370)
//  Revision 1.1 2010/08/11 21:27:08IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/core/core.pj
//  Revision 1.11 2010/06/02 13:34:06MESZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Introduction of RB Types as new template parameter. (Mantis :0003259)
//  Revision 1.10 2010/03/12 19:17:24GMT+05:30 Jaeger Thomas (CC/ESV2) (JAT2HI)
//  - fixed compiler warning (mantis3253)
//  Revision 1.9 2010/03/11 06:23:23CET Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Inclusion of new SIUnits UserType (like "pixel", "coord system").
//  Revision 1.8 2009/07/31 18:41:50GMT+05:30 Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Support for different offsets in the siunits implementation, for units like temperature Kelvin and °C. (mantis 2975)
//  Revision 1.7 2009/05/29 16:07:46GMT+05:30 Dhananjay N (RBEI/EAC1) (dhn1kor)
//  - change all spelling  meter to metre.(matis:2747)
//  Revision 1.6 2009/01/23 16:01:22IST Dilip Krishna (RBEI/EAC1) (dkn2kor)
//  - interface fixed with default template parameters
//  Revision 1.5 2008/12/30 09:14:26IST Dilip Krishna (RBEI/EAC1) (dkn2kor)
//  - compilation problem due to policy type in gcc corrected
//  Revision 1.4 2008/12/11 17:01:12IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  - major redesign to accomodate the conversion between different types
//  Revision 1.3 2008/09/22 18:51:08IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  - added the convenience typedefs with "open" user type(Mantis :- 0002352)
//  Revision 1.2 2008/09/17 11:17:44IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  -add convenience typedefs for siunits_types (mantis:- 0002322)
//  -add support for electric current (A) (mantis:- 2328)
//  -add support for Candela (mantis:- 2340)
//  Revision 1.1 2008/08/20 18:24:53IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_siunits/vfc_siunits.pj
//=============================================================================

