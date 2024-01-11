//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2006 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorised copy, use or 
//  distribution is an offensive act against international law and may be 
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: 
//          Synopsis: 
//  Target system(s): 
//       Compiler(s): VS7.1
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes: 
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: dkn2kor
//  Department: 
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_linalg_exp2d.inl $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (jat2hi) $
///     $Date: 2007/05/09 10:21:19MESZ $
///     $Locker:  $
///     $Name: 0032 RC1 Hello KW07  $
///     $State: In_Development $
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

template<class ValueType, class Expression1Type, class Expression2Type>
inline 
ValueType
vfc::linalg::intern::TExp2DMulOpImpl<ValueType, Expression1Type, 
Expression2Type, vfc::linalg::CDynamicRectangle>::eval(
    const Expression1Type& f_operand1_r, 
    const Expression2Type& f_operand2_r, 
    size_type f_row, size_type f_col)
{
    value_type value = static_cast<value_type>(0);
    for (vfc::int32_t i = 0; i < f_operand1_r.getNbColumns(); ++i)
    {
        value += f_operand1_r(f_row, i) * f_operand2_r(i, f_col);
    }
    return value;
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_exp2d.inl  $
//  Revision 1.1 2007/05/09 10:21:19MESZ Jaeger Thomas (CC-DA/EPV2) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
