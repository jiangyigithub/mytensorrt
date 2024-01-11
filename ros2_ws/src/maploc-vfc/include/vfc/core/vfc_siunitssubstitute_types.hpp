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
///     $Source: vfc_siunitssubstitute_types.hpp $
///     $Revision: 1.4 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2012/12/18 08:27:32MEZ $
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
    struct CSubstituteInfoType
    {
        typedef CSubstituteInfoType self_unit_type;
        typedef CSubstituteInfoType base_unit_type;

        struct CSubstitute
        {
        };

        typedef vfc::TSIType<CSubstitute>            CDummyUnitType;


        enum
        {
            SI_UNIT = 0
        };

        enum
        {
            SI_FUNDAMENTAL = 1
        };

        typedef CDummyUnitType                  length_unit_type;
        typedef CDummyUnitType                  mass_unit_type;
        typedef CDummyUnitType                  time_unit_type;
        typedef CDummyUnitType                  current_unit_type;
        typedef CDummyUnitType                  temperature_unit_type;
        typedef CDummyUnitType                  amountofsubstance_unit_type;
        typedef CDummyUnitType                  luminousintensity_unit_type;
        typedef CDummyUnitType                  angle_unit_type;
        typedef vfc::CBasicType                 unit_type;

        static const vfc::int32_t LENGTH_PREFIX_VALUE = vfc::BASE;
        static const vfc::int32_t LENGTH_POWER_VALUE = 0;

        static const vfc::int32_t MASS_PREFIX_VALUE = vfc::BASE;
        static const vfc::int32_t MASS_POWER_VALUE = 0;

        static const vfc::int32_t TIME_PREFIX_VALUE = vfc::BASE;
        static const vfc::int32_t TIME_POWER_VALUE = 0;

        static const vfc::int32_t CURRENT_PREFIX_VALUE = vfc::BASE;
        static const vfc::int32_t CURRENT_POWER_VALUE = 0;

        static const vfc::int32_t TEMPERATURE_PREFIX_VALUE = vfc::BASE;
        static const vfc::int32_t TEMPERATURE_POWER_VALUE = 0;

        static const vfc::int32_t AMOUNTOFSUBSTANCE_PREFIX_VALUE = vfc::BASE;
        static const vfc::int32_t AMOUNTOFSUBSTANCE_POWER_VALUE = 0;

        static const vfc::int32_t LUMINOUSINTENSITY_PREFIX_VALUE = vfc::BASE;
        static const vfc::int32_t LUMINOUSINTENSITY_POWER_VALUE = 0;

        static const vfc::int32_t ANGLE_PREFIX_VALUE = vfc::BASE;
        static const vfc::int32_t ANGLE_POWER_VALUE = 0;
    };

    struct SSubstituteRBInfoType
    {

        typedef SSubstituteRBInfoType     self_unit_type;
        typedef SSubstituteRBInfoType     base_unit_type;

        struct CSubstitutePixel
        {
        };

        struct CSubstitutePercentage
        {
        };

        typedef vfc::TSIType<CSubstitutePixel>            CDummyUnitPixelType;
        typedef vfc::TSIType<CSubstitutePercentage>       CDummyUnitPercentageType;

        enum
        {
            SI_UNIT = 0
        };

        enum
        {
            SI_FUNDAMENTAL = 1
        };

        typedef CDummyUnitPixelType              pixel_unit_type;
        typedef CDummyUnitPercentageType         percentage_unit_type;
        typedef vfc::CBasicType                  unit_type;

        static const vfc::int32_t PIXEL_PREFIX_VALUE = vfc::BASE;
        static const vfc::int32_t PIXEL_POWER_VALUE = 0;
        static const vfc::int32_t PERCENTAGE_PREFIX_VALUE = vfc::BASE;
        static const vfc::int32_t PERCENTAGE_POWER_VALUE = 0;

    };

    template<>
    struct TSIUnitMuliplicationPromote<CSubstituteInfoType, CSubstituteInfoType>
    {
        typedef CSubstituteInfoType unit_info_type;
    };

    template<>
    struct TSIUnitRBMuliplicationPromote<SSubstituteRBInfoType, SSubstituteRBInfoType>
    {
        typedef SSubstituteRBInfoType unit_info_type;
    };


    template<class ValueType, class UserType = vfc::CDefaultType,
                                template<class, class, class> class ConvertPolicyType = vfc::TNoConvertConvert>
    struct TSIUnitType
    {
        ///////////////////////////////////////////////
        ///////////////// length /////////////////////
        ///////////////////////////////////////////////

        // power = 1

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_metre_t;


        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_kilo_metre_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_mega_metre_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_giga_metre_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_tera_metre_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_deci_metre_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_centi_metre_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_milli_metre_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_micro_metre_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_nano_metre_t;

        // power = 2

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_square_metre_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                 si_square_kilo_metre_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                       si_cubic_metre_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                  si_cubic_kilo_metre_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, 
            UserType, ConvertPolicyType>                                  si_metre_per_cubic_second_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                  si_metre_second_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                  si_cubic_deci_metre_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                  si_litre_t;



        ///////////////////////////////////////////////
        ///////////////// time /////////////////////
        ///////////////////////////////////////////////

        // power = 1

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_second_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_minute_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_hour_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_milli_second_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_micro_second_t;

        // 1 / s
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_per_second_t;

        //  1 / hr
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_per_hour_t;


        //  1 / s2
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                 si_per_square_second_t;

        //  1 / hr2
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                 si_per_square_hour_t;
        
        
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                 si_day_t;

        
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                 si_week_t;

        
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                 si_month_t;

        
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                 si_year_t;
        ///////////////////////////////////////////////
        ///////////////// Degree /////////////////////
        ///////////////////////////////////////////////

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_degree_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, 
            UserType, ConvertPolicyType>                                      si_degree_per_second_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_radian_t;


        ///////////////////////////////////////////////
        ///////////////// Mass /////////////////////
        ///////////////////////////////////////////////

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_gram_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_kilo_gram_t;

        ///////////////////////////////////////////////
        ///////////////// temperature /////////////////////
        ///////////////////////////////////////////////

        //  temperature based
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_kelvin_t;

        ///////////////////////////////////////////////
        ///////////////// ampere /////////////////////
        ///////////////////////////////////////////////

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_ampere_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_kilo_ampere_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_mega_ampere_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_giga_ampere_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_tera_ampere_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_deci_ampere_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_centi_ampere_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_milli_ampere_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_micro_ampere_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_nano_ampere_t;

        // 1 / A
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_per_ampere_t;


        ///////////////////////////////////////////////
        ///////////////// candela /////////////////////
        ///////////////////////////////////////////////

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_candela_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_kilo_candela_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_mega_candela_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_giga_candela_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_tera_candela_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_deci_candela_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_centi_candela_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_milli_candela_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_micro_candela_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_nano_candela_t;

        // 1 / cd
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_per_candela_t;


        ///////////////////////////////////////////////////////////////////////////
        //////////////////// Commonly used types //////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////

        // Hz
        typedef     si_per_second_t                             si_hertz_t ;


        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_joule_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_watt_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_kilo_watt_t;

          
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_bar_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_milli_bar_t;

        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, 
            UserType, ConvertPolicyType>                                      si_force_of_gravity_t;


        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_lux_t;
        
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, 
            UserType, ConvertPolicyType>                                      si_kilo_gram_metre_per_square_second_t;

        // newton metre
        typedef     si_joule_t      si_newton_metre_t ;

        
        //Newton = mass * acceleration
        //Newton = Kg * m/s2
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_newton_t;

           //KiloNewton  = Kilo*Kilo gram*metre/s2
        //KiloNewton  = Mega gram*metre/s2
        //KiloNewton  = Mg * m/s2

         typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_kilo_newton_t;


        //  velocity
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                 si_metre_per_second_t;


        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                               si_kilometre_per_hour_t;

        //  acceleration
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                          si_metre_per_square_second_t;


        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                        si_kilometre_per_square_hour_t;


        // wave number
         typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
             SSubstituteRBInfoType, UserType, ConvertPolicyType>                                     si_wavenumber_t;

        // nil type
        typedef vfc::TSIUnits<ValueType, CSubstituteInfoType,
            SSubstituteRBInfoType, UserType, ConvertPolicyType>                                      si_nil_t;


    };

    template<class UserType = vfc::CDefaultType,
                                        template<class, class, class> class ConvertPolicyType = vfc::TNoConvertConvert>
    class TSIUnitType_i32 : public TSIUnitType<vfc::int32_t, UserType, ConvertPolicyType>
    {

    };

    template<class UserType = vfc::CDefaultType,
                                        template<class, class, class> class ConvertPolicyType = vfc::TNoConvertConvert>
    class TSIUnitType_f32 : public TSIUnitType<vfc::float32_t, UserType, ConvertPolicyType>
    {

    };

    template<class UserType = vfc::CDefaultType,
                                        template<class, class, class> class ConvertPolicyType = vfc::TNoConvertConvert>
    class TSIUnitType_f64 : public TSIUnitType<vfc::float64_t, UserType, ConvertPolicyType>
    {

    };

}

#endif //VFC_SIUNITSSUBSTITUTE_TYPES_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_siunitssubstitute_types.hpp  $
//  Revision 1.4 2012/12/18 08:27:32MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.3 2012/02/22 10:33:40MEZ Gaurav Jain (RBEI/ESD1) (gaj2kor) 
//  - Include guard is replaced to avoid the compilation error by the inclusion of vfc_siunits_types.hpp and vfc_siunitssubstitute_types.hpp in vfc_core_all. (mantis : 3489)
//  Revision 1.2 2010/09/13 20:13:51IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - si units: add support for units used in Volkswagen AG DBC files (mantis3370)
//  Revision 1.1 2010/08/11 21:27:08IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/core/core.pj
//  Revision 1.3 2010/06/02 13:34:08MESZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Introduction of RB Types as new template parameter. (Mantis :0003259)
//  Revision 1.2 2009/05/29 16:07:43GMT+05:30 Dhananjay N (RBEI/EAC1) (dhn1kor)
//  - change all spelling  meter to metre.(matis:2747)
//  Revision 1.1 2009/01/23 16:03:11IST Dilip Krishna (RBEI/EAC1) (dkn2kor)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_siunits/vfc_siunits.pj
//=============================================================================

