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
//       Projectname:
//  Target system(s):
//       Compiler(s): VS8.0
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: gaj2kor
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
///     $Source: vfc_siunits_convenienttypes.hpp $
///     $Revision: 1.10 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/09/25 09:53:25MESZ $
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

#ifndef VFC_SIUNITS_CONVENIENTTYPES_HPP_INCLUDED
#define VFC_SIUNITS_CONVENIENTTYPES_HPP_INCLUDED

#include "vfc/core/vfc_siunits.hpp"
#include "vfc/core/vfc_siunits_types.hpp"
#include "vfc/core/vfc_preprocessor.hpp"

#define CONVENIENCE_TYPE(UNIT, RBINFO, NAME) \
    typedef vfc::TSIUnits<vfc::int32_t,   vfc::info_##UNIT, vfc::info_##RBINFO, vfc::CDefaultType, vfc::TConvert> si_##NAME##_i32_t;  \
    typedef vfc::TSIUnits<vfc::uint32_t,  vfc::info_##UNIT, vfc::info_##RBINFO, vfc::CDefaultType, vfc::TConvert> si_##NAME##_ui32_t; \
    typedef vfc::TSIUnits<vfc::float32_t, vfc::info_##UNIT, vfc::info_##RBINFO, vfc::CDefaultType, vfc::TConvert> si_##NAME##_f32_t;  \
    typedef vfc::TSIUnits<vfc::float64_t, vfc::info_##UNIT, vfc::info_##RBINFO, vfc::CDefaultType, vfc::TConvert> si_##NAME##_f64_t

namespace vfc
{
    //============================================================
    //SIUnits common typedefs
    //============================================================

    namespace CSI
    {
        ///////////////////////////////////////////////
        ///////////////// slope ///////////////////////
        ///////////////////////////////////////////////
        CONVENIENCE_TYPE(slope_t, rbnil_type, slope);

        ///////////////////////////////////////////////
        ///////////////// pixel ///////////////////////
        ///////////////////////////////////////////////
        CONVENIENCE_TYPE(nil_type,     pixel_t,            pixel);
        CONVENIENCE_TYPE(nil_type,     square_pixel_t,     square_pixel);
        CONVENIENCE_TYPE(nil_type,     per_pixel_t,        per_pixel);
        CONVENIENCE_TYPE(nil_type,     per_square_pixel_t, per_square_pixel);
        CONVENIENCE_TYPE(per_radian_t, pixel_t,            pixel_per_radian);
        CONVENIENCE_TYPE(metre_t,      pixel_t,            metre_pixel);

        ///////////////////////////////////////////////
        ///////////////// length //////////////////////
        ///////////////////////////////////////////////
        // power = 1
        CONVENIENCE_TYPE(metre_t,              rbnil_type, metre);
        CONVENIENCE_TYPE(kilo_metre_t,         rbnil_type, kilo_metre);
        CONVENIENCE_TYPE(mega_metre_t,         rbnil_type, mega_metre);
        CONVENIENCE_TYPE(giga_metre_t,         rbnil_type, giga_metre);
        CONVENIENCE_TYPE(tera_metre_t,         rbnil_type, tera_metre);
        CONVENIENCE_TYPE(deci_metre_t,         rbnil_type, deci_metre);
        CONVENIENCE_TYPE(centi_metre_t,        rbnil_type, centi_metre);
        CONVENIENCE_TYPE(milli_metre_t,        rbnil_type, milli_metre);
        CONVENIENCE_TYPE(micro_metre_t,        rbnil_type, micro_metre);
        CONVENIENCE_TYPE(nano_metre_t,         rbnil_type, nano_metre);
        // power = 2
        CONVENIENCE_TYPE(square_metre_t,       rbnil_type, square_metre);
        CONVENIENCE_TYPE(square_kilo_metre_t,  rbnil_type, square_kilo_metre);
        // power = 3
        CONVENIENCE_TYPE(cubic_metre_t,        rbnil_type, cubic_metre);
        CONVENIENCE_TYPE(cubic_kilo_metre_t,   rbnil_type, cubic_kilo_metre);
        // power = -1
        CONVENIENCE_TYPE(per_metre_t,          rbnil_type, per_metre);
        // power = -2
        CONVENIENCE_TYPE(per_square_metre_t,   rbnil_type, per_square_metre);

        // imperial
        CONVENIENCE_TYPE(mile_t,               rbnil_type, mile);

        ///////////////////////////////////////////////
        ///////////////// time ////////////////////////
        ///////////////////////////////////////////////
        // power = 1
        CONVENIENCE_TYPE(second_t,            rbnil_type, second);
        CONVENIENCE_TYPE(minute_t,            rbnil_type, minute);
        CONVENIENCE_TYPE(hour_t,              rbnil_type, hour);
        CONVENIENCE_TYPE(milli_second_t,      rbnil_type, milli_second);
        CONVENIENCE_TYPE(micro_second_t,      rbnil_type, micro_second);
        // power = -1
        CONVENIENCE_TYPE(per_second_t,        rbnil_type, per_second);
        CONVENIENCE_TYPE(per_minute_t,        rbnil_type, per_minute);
        CONVENIENCE_TYPE(per_hour_t,          rbnil_type, per_hour);
        CONVENIENCE_TYPE(per_milli_second_t,  rbnil_type, per_milli_second);
        CONVENIENCE_TYPE(per_micro_second_t,  rbnil_type, per_micro_second);
        // power = -2
        CONVENIENCE_TYPE(per_square_second_t, rbnil_type, per_square_second);
        CONVENIENCE_TYPE(per_square_hour_t,   rbnil_type, per_square_hour);
        // power = 2
        CONVENIENCE_TYPE(square_second_t,     rbnil_type, square_second);

        ///////////////////////////////////////////////
        ///////////////// angles //////////////////////
        ///////////////////////////////////////////////
        CONVENIENCE_TYPE(degree_t,                          rbnil_type, degree);
        CONVENIENCE_TYPE(radian_t,                          rbnil_type, radian);
        CONVENIENCE_TYPE(square_radian_t,                   rbnil_type, square_radian);
        CONVENIENCE_TYPE(square_degree_t,                   rbnil_type, square_degree);
        CONVENIENCE_TYPE(per_square_radian_t,               rbnil_type, per_square_radian);
        CONVENIENCE_TYPE(per_radian_t,                      rbnil_type, per_radian);
        CONVENIENCE_TYPE(degree_per_second_t,               rbnil_type, degree_per_second);
        CONVENIENCE_TYPE(degree_square_per_second_square_t, rbnil_type, degree_square_per_second_square);
        CONVENIENCE_TYPE(radian_per_second_t,               rbnil_type, radian_per_second);
        CONVENIENCE_TYPE(radian_per_square_second_t,        rbnil_type, radian_per_square_second);
        CONVENIENCE_TYPE(square_radian_per_square_second_t, rbnil_type, square_radian_per_square_second);

        ///////////////////////////////////////////////
        ///////////////// mass ////////////////////////
        ///////////////////////////////////////////////
        CONVENIENCE_TYPE(gram_t,       rbnil_type, gram);
        CONVENIENCE_TYPE(kilo_gram_t,  rbnil_type, kilo_gram);
        CONVENIENCE_TYPE(ton_t,        rbnil_type, ton);
        CONVENIENCE_TYPE(milli_gram_t, rbnil_type, milli_gram);
        CONVENIENCE_TYPE(micro_gram_t, rbnil_type, micro_gram);

        ///////////////////////////////////////////////
        ///////////////// temperature /////////////////
        ///////////////////////////////////////////////
        CONVENIENCE_TYPE(kelvin_t,  rbnil_type, kelvin);
        CONVENIENCE_TYPE(celsius_t, rbnil_type, celsius);

        ///////////////////////////////////////////////
        ///////////////// current /////////////////////
        ///////////////////////////////////////////////
        CONVENIENCE_TYPE(ampere_t,       rbnil_type, ampere);
        CONVENIENCE_TYPE(deci_ampere_t,  rbnil_type, deci_ampere);
        CONVENIENCE_TYPE(centi_ampere_t, rbnil_type, centi_ampere);
        CONVENIENCE_TYPE(kilo_ampere_t,  rbnil_type, kilo_ampere);
        CONVENIENCE_TYPE(mega_ampere_t,  rbnil_type, mega_ampere);
        CONVENIENCE_TYPE(giga_ampere_t,  rbnil_type, giga_ampere);
        CONVENIENCE_TYPE(tera_ampere_t,  rbnil_type, tera_ampere);
        CONVENIENCE_TYPE(milli_ampere_t, rbnil_type, milli_ampere);
        CONVENIENCE_TYPE(micro_ampere_t, rbnil_type, micro_ampere);
        CONVENIENCE_TYPE(nano_ampere_t,  rbnil_type, nano_ampere);
        CONVENIENCE_TYPE(per_ampere_t,   rbnil_type, per_ampere);

        ///////////////////////////////////////////////
        //////////// luminous intensity ///////////////
        ///////////////////////////////////////////////
        CONVENIENCE_TYPE(candela_t,       rbnil_type, candela);
        CONVENIENCE_TYPE(deci_candela_t,  rbnil_type, deci_candela);
        CONVENIENCE_TYPE(centi_candela_t, rbnil_type, centi_candela);
        CONVENIENCE_TYPE(kilo_candela_t,  rbnil_type, kilo_candela);
        CONVENIENCE_TYPE(mega_candela_t,  rbnil_type, mega_candela);
        CONVENIENCE_TYPE(giga_candela_t,  rbnil_type, giga_candela);
        CONVENIENCE_TYPE(tera_candela_t,  rbnil_type, tera_candela);
        CONVENIENCE_TYPE(milli_candela_t, rbnil_type, milli_candela);
        CONVENIENCE_TYPE(micro_candela_t, rbnil_type, micro_candela);
        CONVENIENCE_TYPE(nano_candela_t,  rbnil_type, nano_candela);
        CONVENIENCE_TYPE(per_candela_t,   rbnil_type, per_candela);

        ///////////////////////////////////////////////////////////////////////////
        //////////////////// Commonly used types //////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////

        CONVENIENCE_TYPE(joule_t,                     rbnil_type,   joule);
        CONVENIENCE_TYPE(lux_t,                       rbnil_type,   lux);
        CONVENIENCE_TYPE(metre_per_second_t,          rbnil_type,   metre_per_second);
        CONVENIENCE_TYPE(miles_per_hour_t,            rbnil_type,   miles_per_hour);
        CONVENIENCE_TYPE(per_metre_second_t,          rbnil_type,   per_metre_second);
        CONVENIENCE_TYPE(metre_per_cubic_second_t,    rbnil_type,   metre_per_cubic_second);
        CONVENIENCE_TYPE(kilometre_per_hour_t,        rbnil_type,   kilometre_per_hour);
        CONVENIENCE_TYPE(metre_per_square_second_t,   rbnil_type,   metre_per_square_second);
        CONVENIENCE_TYPE(per_square_metre_second_t,   rbnil_type,   per_square_metre_second);
        CONVENIENCE_TYPE(kilometre_per_square_hour_t, rbnil_type,   kilometre_per_square_hour);
        CONVENIENCE_TYPE(per_second_t,                rbnil_type,   hertz);
        CONVENIENCE_TYPE(joule_t,                     rbnil_type,   newton_metre);
        CONVENIENCE_TYPE(per_metre_t,                 rbnil_type,   wavenumber);
        CONVENIENCE_TYPE(nil_type,                    percentage_t, percent);
        CONVENIENCE_TYPE(per_second_t,                percentage_t, percent_per_second);
        CONVENIENCE_TYPE(force_of_gravity_t,          percentage_t, percent_of_force_of_gravity);
        CONVENIENCE_TYPE(square_metre_per_square_second_t, rbnil_type, square_metre_per_square_second);
        CONVENIENCE_TYPE(square_metre_per_quartic_second_t, rbnil_type, square_metre_per_quartic_second);
        CONVENIENCE_TYPE(gravitational_accel_t,       rbnil_type,   gravitational_accel);

        // TODO: who needs this special/non conformant type here?
        typedef si_pixel_per_radian_f32_t si_pix_per_rad;

    } // namespace CSI

}

#endif //VFC_SIUNITS_CONVENIENTTYPES_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_siunits_convenienttypes.hpp  $
//  Revision 1.12 2016/11/28 09:12:16MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Change SI-Units convenience scope "CSI" to namespace (mantis0005277)
//  Revision 1.11 2016/07/06 07:29:29MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Add SI units for miles and miles per hour (mantis0005173)
//  Revision 1.10 2014/09/25 09:53:25MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - additional SI convenient types (mantis0004215)
//  Revision 1.9 2014/01/28 11:50:28MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_siunits: replace tabs by 4 spaces (mantis0004394)
//  Revision 1.8 2013/11/08 14:19:15MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  -  Provide SIUnits convenience types in more than just float32 (mantis4186)
//  Revision 1.2 2010/09/13 16:43:49MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - si units: add support for units used in Volkswagen AG DBC files (mantis3370)
//  Revision 1.1 2010/08/11 21:27:07IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/core/core.pj
//  Revision 1.2 2010/06/02 13:34:02MESZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Introduction of RB Types as new template parameter. (Mantis :0003259)
//  Revision 1.1 2010/03/12 20:26:06GMT+05:30 Jaeger Thomas (CC/ESV2) (JAT2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/vfc_siunits.pj
//=============================================================================
