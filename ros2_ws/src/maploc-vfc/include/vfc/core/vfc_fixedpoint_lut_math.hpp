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
///     $Source: include/vfc/core/vfc_fixedpoint_lut_math.hpp $
///     $Revision: 1.13 $
///     $Author: Gaurav Jain (RBEI/ESD1) (gaj2kor) $
///     $Date: 2009/01/27 11:13:57MEZ $
///     $Locker:  $
///     $Name: 0032 RC1  $
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

#ifndef VFC_FIXEDPOINT_LUT_MATH_HPP
#define VFC_FIXEDPOINT_LUT_MATH_HPP

namespace vfc
{

    namespace intern
    {

        template <typename ValueType>
        inline ValueType fixedlut_sin(const ValueType& f_value);

        template <typename ValueType>
        inline ValueType fixedlut_cos(const ValueType& f_value);

        template <typename ValueType>
        inline ValueType fixedlut_tan(const ValueType& f_value);

        template <typename ValueType>
        inline ValueType fixedlut_exp(const ValueType& f_value);

        template <typename ValueType>
        inline ValueType fixedlut_log(const ValueType& f_value);

        template <typename ValueType>
        inline ValueType fixedlut_pow(const ValueType& f_base, const ValueType& f_exponent);

        template <typename ValueType>
        inline ValueType fixedlut_asin(const ValueType& f_value);

        template <typename ValueType>
        inline ValueType fixedlut_acos(const ValueType& f_value);

        template <typename ValueType>
        inline ValueType atan(const ValueType& f_arg);

    } // end intern

} // end vfc

#include "vfc/core/vfc_fixedpoint_lut_math.inl"

#endif // VFC_FIXEDPOINT_LUT_MATH_HPP

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: include/vfc/core/vfc_fixedpoint_lut_math.hpp  $
//  Revision 1.13 2009/01/27 11:13:57MEZ Gaurav Jain (RBEI/ESD1) (gaj2kor) 
//  -Inclusion of footer and proper indendation. 
//  (Matnis : 0002540)
//=============================================================================

