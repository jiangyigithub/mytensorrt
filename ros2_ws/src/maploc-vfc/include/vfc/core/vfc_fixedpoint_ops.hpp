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
//        Name: zvh2hi
//  Department:
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: include/vfc/core/vfc_fixedpoint_ops.hpp $
///     $Revision: 1.23 $
///     $Author: Gaurav Jain (RBEI/ESD1) (gaj2kor) $
///     $Date: 2009/10/01 12:21:02MESZ $
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

#ifndef VFC_FIXEDPOINT_OPS_HPP_INCLUDED
#define VFC_FIXEDPOINT_OPS_HPP_INCLUDED

#include "vfc/core/vfc_fixedpoint.hpp"

namespace vfc
{
    ///  the nint function returns the nearest integer of specified value
    template <vfc::int32_t FracBitsValue, class ValueType>
    inline int32_t nint    (const TFixedPoint<FracBitsValue, ValueType>& f_value);

    /// the floor function returns an integer value representing the largest integer
    /// that is less than or equal to specified value.
    template <vfc::int32_t FracBitsValue, class ValueType >
    inline int32_t floor   (const TFixedPoint<FracBitsValue, ValueType>& f_value);

    /// the ceil function an integer value representing the smallest integer
    /// that is greater than or equal to specified value.
    template <vfc::int32_t FracBitsValue, class ValueType>
    inline int32_t ceil    (const TFixedPoint<FracBitsValue, ValueType>& f_value);


    template <class ResultType, vfc::int32_t FracBitsValue, class ValueType>
    inline ResultType  numeric_cast(const TFixedPoint<FracBitsValue, ValueType>& f_value);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline TFixedPoint<FracBitsValue, ValueType> operator-(const TFixedPoint<FracBitsValue, ValueType>& f_value);

    template <vfc::int32_t FracBitsValue, class ValueType >
    inline TFixedPoint<FracBitsValue, ValueType>  sqr(const TFixedPoint<FracBitsValue, ValueType>& f_value);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline TFixedPoint<FracBitsValue, ValueType>  sqrt(const TFixedPoint<FracBitsValue, ValueType>& f_value);

    // not the right place for this function, has to be moved to vfc_core
    // no fixedpoint dependancy
    inline vfc::uint32_t int_sqrt(const vfc::uint32_t x);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline TFixedPoint<FracBitsValue, ValueType>  abs(const TFixedPoint<FracBitsValue, ValueType>& f_value);

#ifndef    VFC_NO_FLOAT
    //! \warning pow(a, b) = exp(b * log(a)) requires b * log(a) to fit into a fp32p28
    template <vfc::int32_t FracBitsValue, class ValueType>
    inline TFixedPoint<FracBitsValue, ValueType> pow(const TFixedPoint<FracBitsValue, ValueType>& f_base,
        const TFixedPoint<FracBitsValue, ValueType>& f_exponent);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline vfc::TFixedPoint<FracBitsValue, ValueType>  exp(const TFixedPoint<FracBitsValue, ValueType>& f_value);
#endif

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline vfc::TFixedPoint<FracBitsValue, ValueType> log(const TFixedPoint<FracBitsValue, ValueType>& f_value);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline TFixedPoint<FracBitsValue, ValueType>  sin(const TFixedPoint<FracBitsValue, ValueType>& f_value);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline TFixedPoint<FracBitsValue, ValueType>  cos(const TFixedPoint<FracBitsValue, ValueType>& f_value);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline TFixedPoint<FracBitsValue, ValueType>  tan(const TFixedPoint<FracBitsValue, ValueType>& f_value);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline TFixedPoint<FracBitsValue, ValueType>  fmod(const TFixedPoint<FracBitsValue, ValueType>& f_x,
                                                       const TFixedPoint<FracBitsValue, ValueType>& f_y);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline vfc::TFixedPoint<FracBitsValue, ValueType> atan(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline TFixedPoint<FracBitsValue, ValueType>  asin(const TFixedPoint<FracBitsValue, ValueType>& f_value);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline TFixedPoint<FracBitsValue, ValueType>  acos(const TFixedPoint<FracBitsValue, ValueType>& f_value);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline vfc::TFixedPoint<FracBitsValue, ValueType> atan2(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_dy,
        const vfc::TFixedPoint<FracBitsValue, ValueType>& f_dx);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline TFixedPoint<FracBitsValue, ValueType>  min(const TFixedPoint<FracBitsValue, ValueType>& f_lhs,
                                                                const TFixedPoint<FracBitsValue, ValueType>& f_rhs);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline TFixedPoint<FracBitsValue, ValueType>  max(const TFixedPoint<FracBitsValue, ValueType>& f_lhs,
                                                                const TFixedPoint<FracBitsValue, ValueType>& f_rhs);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline bool  isEqual(const TFixedPoint<FracBitsValue, ValueType>& f_value1,
                  const TFixedPoint<FracBitsValue, ValueType>& f_value2,
                  const TFixedPoint<FracBitsValue, ValueType>& f_delta);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline bool  isPositive(const TFixedPoint<FracBitsValue, ValueType>& f_value);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline bool  isNegative(const TFixedPoint<FracBitsValue, ValueType>& f_value);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline bool  isZero(const TFixedPoint<FracBitsValue, ValueType>& f_value);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline bool  notZero(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value);

    template <vfc::int32_t FracBitsValue, class ValueType>
    inline TFixedPoint<FracBitsValue, ValueType>  signum(const TFixedPoint<FracBitsValue, ValueType>& f_value);


}   // namespace vfc closed

#include "vfc/core/vfc_fixedpoint_ops.inl"

#endif //VFC_FIXEDPOINT_OPS_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: include/vfc/core/vfc_fixedpoint_ops.hpp  $
//  Revision 1.23 2009/10/01 12:21:02MESZ Gaurav Jain (RBEI/ESD1) (gaj2kor) 
//  - Clean separation of fpu-code and fpu-free code using VFC_NO_FLOAT.
//  Revision 1.22 2009/03/25 20:29:45GMT+05:30 Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  - Linebreak at 120 in all fixedpoint files. (mantis 0002692)
//  Revision 1.21 2008/04/09 14:48:10IST Voelz Henning (CC-DA/ESV4) (VOH2HI)
//  variable names changed
//  Revision 1.20 2008/02/20 06:43:17CET Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  Removal of OpearationPolicy
//  Revision 1.19 2008/01/17 15:46:29IST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  internal namespace changed to intern;
//  gcc warnings removed
//  gcc error (typename) removed
//  Revision 1.18 2008/01/16 17:27:07CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Moved atan from vfc_fixedpoint_trig.hpp to vfc_fixedpoint_lut_math.inl.
//  Removed all trig solutions that are not used in the ldw project. vfc_fixedpoint_lut_math.hpp is now the only header, which contains trig algorithm.
//  Removed all little template helpers from vfc_fixedpoint_trig_helper.hpp.
//  Revision 1.17 2008/01/16 15:57:39CET Voelz Henning (AE-DA/ESV1) (voh2hi)
//  methods redirected to LUT implementations
//  Revision 1.16 2007/12/20 13:06:44CET Voelz Henning (AE-DA/ESV1) (voh2hi)
//  use lut implementation for sin/cos/tan
//  Revision 1.15 2007/12/19 15:42:49CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Exposed lut based trig functions to vfc:: namespace.
//  Revision 1.14 2007/12/19 14:50:36CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Added log to ops.
//  Revision 1.13 2007/12/04 16:17:39CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Changed static const intergral to enum type -> Needed cast for constructor. Moved all definitions and helping calculations from vfc_fixedpoint_trig to vfc_fixedpoint_trig_helper.hpp.
//  Revision 1.12 2007/11/26 09:31:27CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Better usage for pow(a, b). Exp now has a template computed return type.
//  Revision 1.11 2007/11/19 16:26:29CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Made vfc::exp vfc::pow for fixedpoint more usable.
//  vfc::pow has some restrictions about the maximal value that is allowed. (pow(a, b) == exp(b * log(a)) : b * log(a) must fit into a fp32p28).
//  CppUnit test now testing nearly all function. Still some test need a rewrite.
//  Revision 1.10 2007/11/12 17:35:45CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Modified fixedpoint pow. Now the user have to take care about the return type and the maximum value.
//  Revision 1.9 2007/11/12 11:33:38CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Made atan more usable (but if using it with too high values it becomes more inaccurate!). fixedpoint ops now work with intern method.
//  Revision 1.8 2007/10/30 14:11:59CET Renner Christian (AE-DA/ESV1) (rec1lr)
//  temporary check in for int_sqrt function (has to be moved o vfc_core, because no fixedpoint dependency)
//  Revision 1.7 2007/10/23 10:08:09CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  vfc::signum() added
//  Revision 1.6 2007/10/22 11:04:02CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  notZero added
//  Revision 1.5 2007/10/22 10:02:13CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  sin / cos / tan with template taylor summands
//  Revision 1.4 2007/10/22 08:52:18CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  inlining
//  Revision 1.2 2007/09/24 10:38:53CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  isZero, isPositive, isNegative, isEqual added
//  Revision 1.1 2007/09/24 10:17:35CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_fixedpoint/vfc_fixedpoint.pj
//=============================================================================
