//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2006 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy, use or 
//  distribution is an offensive act against international law and may be 
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/core
//          Synopsis: 
//  Target system(s): 
//       Compiler(s): VS7.1, VS8.0, CW9.3, CWembedded8.5
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes: 
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: voh2hi
//  Department: 
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: include/vfc/core/vfc_fixedpoint_types.hpp $
///     $Revision: 1.12 $
///     $Author: Renner Christian (CC/ESV2) (rec1lr) $
///     $Date: 2008/02/25 13:25:37MEZ $
///     $Locker:  $
///     $Name: 0032 RC1 KW07  $
///     $State: InProcess $
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

#ifndef VFC_FIXEDPOINT_TYPES_HPP_INCLUDED
#define VFC_FIXEDPOINT_TYPES_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

#ifndef USE_FLOATING_POINT
    #include "vfc/core/vfc_fixedpoint.hpp"
#else
    #include "vfc/core/vfc_fixedpoint_substitute.hpp"
#endif


namespace vfc
{

#ifndef USE_FLOATING_POINT
    
    typedef vfc::TFixedPoint<2,  vfc::int32_t>       fp32p2_t;
    typedef vfc::TFixedPoint<3,  vfc::int32_t>       fp32p3_t;
    typedef vfc::TFixedPoint<4,  vfc::int32_t>       fp32p4_t;
    typedef vfc::TFixedPoint<5,  vfc::int32_t>       fp32p5_t;
    typedef vfc::TFixedPoint<6,  vfc::int32_t>       fp32p6_t;
    typedef vfc::TFixedPoint<7,  vfc::int32_t>       fp32p7_t;
    typedef vfc::TFixedPoint<8,  vfc::int32_t>       fp32p8_t;
    typedef vfc::TFixedPoint<9,  vfc::int32_t>       fp32p9_t;
    typedef vfc::TFixedPoint<10, vfc::int32_t>       fp32p10_t;
    typedef vfc::TFixedPoint<11, vfc::int32_t>       fp32p11_t;
    typedef vfc::TFixedPoint<12, vfc::int32_t>       fp32p12_t;
    typedef vfc::TFixedPoint<13, vfc::int32_t>       fp32p13_t;
    typedef vfc::TFixedPoint<14, vfc::int32_t>       fp32p14_t;
    typedef vfc::TFixedPoint<15, vfc::int32_t>       fp32p15_t;
    typedef vfc::TFixedPoint<16, vfc::int32_t>       fp32p16_t;
    typedef vfc::TFixedPoint<17, vfc::int32_t>       fp32p17_t;
    typedef vfc::TFixedPoint<18, vfc::int32_t>       fp32p18_t;
    typedef vfc::TFixedPoint<19, vfc::int32_t>       fp32p19_t;
    typedef vfc::TFixedPoint<20, vfc::int32_t>       fp32p20_t;
    typedef vfc::TFixedPoint<21, vfc::int32_t>       fp32p21_t;
    typedef vfc::TFixedPoint<22, vfc::int32_t>       fp32p22_t;
    typedef vfc::TFixedPoint<23, vfc::int32_t>       fp32p23_t;
    typedef vfc::TFixedPoint<24, vfc::int32_t>       fp32p24_t;
    typedef vfc::TFixedPoint<25, vfc::int32_t>       fp32p25_t;
    typedef vfc::TFixedPoint<26, vfc::int32_t>       fp32p26_t;
    typedef vfc::TFixedPoint<27, vfc::int32_t>       fp32p27_t;
    typedef vfc::TFixedPoint<28, vfc::int32_t>       fp32p28_t;
    typedef vfc::TFixedPoint<29, vfc::int32_t>       fp32p29_t;
    typedef vfc::TFixedPoint<30, vfc::int32_t>       fp32p30_t;


    typedef vfc::TFixedPoint<2,  vfc::int64_t >  fp64p2_t;
    typedef vfc::TFixedPoint<3,  vfc::int64_t >  fp64p3_t;
    typedef vfc::TFixedPoint<4,  vfc::int64_t >  fp64p4_t;
    typedef vfc::TFixedPoint<5,  vfc::int64_t >  fp64p5_t;
    typedef vfc::TFixedPoint<6,  vfc::int64_t >  fp64p6_t;
    typedef vfc::TFixedPoint<7,  vfc::int64_t >  fp64p7_t;
    typedef vfc::TFixedPoint<8,  vfc::int64_t >  fp64p8_t;
    typedef vfc::TFixedPoint<9,  vfc::int64_t >  fp64p9_t;
    typedef vfc::TFixedPoint<10, vfc::int64_t >  fp64p10_t;
    typedef vfc::TFixedPoint<11, vfc::int64_t >  fp64p11_t;
    typedef vfc::TFixedPoint<12, vfc::int64_t >  fp64p12_t;
    typedef vfc::TFixedPoint<13, vfc::int64_t >  fp64p13_t;
    typedef vfc::TFixedPoint<14, vfc::int64_t >  fp64p14_t;
    typedef vfc::TFixedPoint<15, vfc::int64_t >  fp64p15_t;
    typedef vfc::TFixedPoint<16, vfc::int64_t >  fp64p16_t;
    typedef vfc::TFixedPoint<17, vfc::int64_t >  fp64p17_t;
    typedef vfc::TFixedPoint<18, vfc::int64_t >  fp64p18_t;
    typedef vfc::TFixedPoint<19, vfc::int64_t >  fp64p19_t;
    typedef vfc::TFixedPoint<20, vfc::int64_t >  fp64p20_t;
    typedef vfc::TFixedPoint<21, vfc::int64_t >  fp64p21_t;
    typedef vfc::TFixedPoint<22, vfc::int64_t >  fp64p22_t;
    typedef vfc::TFixedPoint<23, vfc::int64_t >  fp64p23_t;
    typedef vfc::TFixedPoint<24, vfc::int64_t >  fp64p24_t;
    typedef vfc::TFixedPoint<25, vfc::int64_t >  fp64p25_t;
    typedef vfc::TFixedPoint<26, vfc::int64_t >  fp64p26_t;
    typedef vfc::TFixedPoint<27, vfc::int64_t >  fp64p27_t;
    typedef vfc::TFixedPoint<28, vfc::int64_t >  fp64p28_t;
    typedef vfc::TFixedPoint<29, vfc::int64_t >  fp64p29_t;
    typedef vfc::TFixedPoint<30, vfc::int64_t >  fp64p30_t;
    typedef vfc::TFixedPoint<31, vfc::int64_t >  fp64p31_t;
    typedef vfc::TFixedPoint<32, vfc::int64_t >  fp64p32_t;
    typedef vfc::TFixedPoint<33, vfc::int64_t >  fp64p33_t;
    typedef vfc::TFixedPoint<34, vfc::int64_t >  fp64p34_t;
    typedef vfc::TFixedPoint<35, vfc::int64_t >  fp64p35_t;
    typedef vfc::TFixedPoint<36, vfc::int64_t >  fp64p36_t;
    typedef vfc::TFixedPoint<37, vfc::int64_t >  fp64p37_t;
    typedef vfc::TFixedPoint<38, vfc::int64_t >  fp64p38_t;
    typedef vfc::TFixedPoint<39, vfc::int64_t >  fp64p39_t;
    typedef vfc::TFixedPoint<40, vfc::int64_t >  fp64p40_t;
    typedef vfc::TFixedPoint<41, vfc::int64_t >  fp64p41_t;
    typedef vfc::TFixedPoint<42, vfc::int64_t >  fp64p42_t;
    typedef vfc::TFixedPoint<43, vfc::int64_t >  fp64p43_t;
    typedef vfc::TFixedPoint<44, vfc::int64_t >  fp64p44_t;
    typedef vfc::TFixedPoint<45, vfc::int64_t >  fp64p45_t;
    typedef vfc::TFixedPoint<46, vfc::int64_t >  fp64p46_t;
    typedef vfc::TFixedPoint<47, vfc::int64_t >  fp64p47_t;
    typedef vfc::TFixedPoint<48, vfc::int64_t >  fp64p48_t;
    typedef vfc::TFixedPoint<49, vfc::int64_t >  fp64p49_t;
    typedef vfc::TFixedPoint<50, vfc::int64_t >  fp64p50_t;
    typedef vfc::TFixedPoint<51, vfc::int64_t >  fp64p51_t;
    typedef vfc::TFixedPoint<52, vfc::int64_t >  fp64p52_t;
    typedef vfc::TFixedPoint<53, vfc::int64_t >  fp64p53_t;
    typedef vfc::TFixedPoint<54, vfc::int64_t >  fp64p54_t;
    typedef vfc::TFixedPoint<55, vfc::int64_t >  fp64p55_t;
    typedef vfc::TFixedPoint<56, vfc::int64_t >  fp64p56_t;
    typedef vfc::TFixedPoint<57, vfc::int64_t >  fp64p57_t;
    typedef vfc::TFixedPoint<58, vfc::int64_t >  fp64p58_t;
    typedef vfc::TFixedPoint<59, vfc::int64_t >  fp64p59_t;
    typedef vfc::TFixedPoint<60, vfc::int64_t >  fp64p60_t;
    typedef vfc::TFixedPoint<61, vfc::int64_t >  fp64p61_t;
    typedef vfc::TFixedPoint<62, vfc::int64_t >  fp64p62_t;

#else  // USE_FLOATING_POINT
    
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32,  2>  fp32p2_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32,  3>  fp32p3_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32,  4>  fp32p4_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32,  5>  fp32p5_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32,  6>  fp32p6_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32,  7>  fp32p7_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32,  8>  fp32p8_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32,  9>  fp32p9_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 10>  fp32p10_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 11>  fp32p11_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 12>  fp32p12_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 13>  fp32p13_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 14>  fp32p14_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 15>  fp32p15_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 16>  fp32p16_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 17>  fp32p17_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 18>  fp32p18_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 19>  fp32p19_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 20>  fp32p20_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 21>  fp32p21_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 22>  fp32p22_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 23>  fp32p23_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 24>  fp32p24_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 25>  fp32p25_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 26>  fp32p26_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 27>  fp32p27_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 28>  fp32p28_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 29>  fp32p29_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32, 30>  fp32p30_t;


    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64,  2>  fp64p2_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64,  3>  fp64p3_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64,  4>  fp64p4_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64,  5>  fp64p5_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64,  6>  fp64p6_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64,  7>  fp64p7_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64,  8>  fp64p8_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64,  9>  fp64p9_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 10>  fp64p10_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 11>  fp64p11_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 12>  fp64p12_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 13>  fp64p13_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 14>  fp64p14_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 15>  fp64p15_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 16>  fp64p16_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 17>  fp64p17_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 18>  fp64p18_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 19>  fp64p19_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 20>  fp64p20_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 21>  fp64p21_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 22>  fp64p22_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 23>  fp64p23_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 24>  fp64p24_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 25>  fp64p25_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 26>  fp64p26_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 27>  fp64p27_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 28>  fp64p28_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 29>  fp64p29_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 30>  fp64p30_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 31>  fp64p31_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 32>  fp64p32_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 33>  fp64p33_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 34>  fp64p34_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 35>  fp64p35_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 36>  fp64p36_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 37>  fp64p37_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 38>  fp64p38_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 39>  fp64p39_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 40>  fp64p40_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 41>  fp64p41_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 42>  fp64p42_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 43>  fp64p43_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 44>  fp64p44_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 45>  fp64p45_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 46>  fp64p46_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 47>  fp64p47_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 48>  fp64p48_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 49>  fp64p49_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 50>  fp64p50_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 51>  fp64p51_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 52>  fp64p52_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 53>  fp64p53_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 54>  fp64p54_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 55>  fp64p55_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 56>  fp64p56_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 57>  fp64p57_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 58>  fp64p58_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 59>  fp64p59_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 60>  fp64p60_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 61>  fp64p61_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 62>  fp64p62_t;
    typedef vfc::TFixedPointSubstitute<vfc::float64_t, 64, 63>  fp64p63_t;


#endif // #ifndef USE_FLOATING_POINT

} // end namespace vfc

#endif // VFC_FIXEDPOINT_TYPES_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: include/vfc/core/vfc_fixedpoint_types.hpp  $
//  Revision 1.12 2008/02/25 13:25:37MEZ Renner Christian (CC/ESV2) (rec1lr) 
//  added missing fp64 types
//  Revision 1.11 2008/02/20 06:40:50CET Vinaykumar Setty (RBIN/EAE6) (vmr1kor) 
//  Removal od OpearationPolicy from TFixedPoint class
//  Revision 1.10 2008/01/17 15:50:19IST Voelz Henning (AE-DA/ESV1) (voh2hi) 
//  fixedpoint substitute bugfix
//  Revision 1.9 2008/01/17 10:16:59CET Voelz Henning (AE-DA/ESV1) (voh2hi) 
//  remove fixedpoint size typedefs, add missing precsion typedefs
//  Revision 1.8 2007/10/22 08:21:19CEST Voelz Henning (AE-DA/ESV1) (voh2hi) 
//  new typedefs added for substitute class
//  Revision 1.7 2007/10/19 13:17:12CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr) 
//  Added 32bit missing typedefs.
//  Revision 1.6 2007/09/25 16:52:00CEST Voelz Henning (AE-DA/ESV1) (voh2hi) 
//  changes for msvc
//  Revision 1.5 2007/09/24 15:40:00CEST Voelz Henning (AE-DA/ESA3) (voh2hi) 
//  template argument added at substitute class
//  Revision 1.4 2007/09/21 11:05:44CEST Voelz Henning (AE-DA/ESA3) (voh2hi) 
//  bugfix namespace
//  Revision 1.3 2007/09/21 09:36:16CEST Voelz Henning (AE-DA/ESA3) (voh2hi) 
//  typedefs for fixedpoint_substitute added
//  Revision 1.2 2007/09/20 16:38:30CEST Renner Christian (AE-DA/ESA3) (rec1lr) 
//  cosmetics
//  Revision 1.1 2007/09/17 10:15:22CEST Voelz Henning (AE-DA/ESA3) (voh2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_fixedpoint/vfc_fixedpoint.pj
//=============================================================================


