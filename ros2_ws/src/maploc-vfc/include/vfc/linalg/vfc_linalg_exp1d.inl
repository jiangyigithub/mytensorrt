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
///     $Source: vfc_linalg_exp1d.inl $
///     $Revision: 1.3 $
///     $Author: Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) $
///     $Date: 2009/05/28 09:19:03MESZ $
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

//Evaluates the dot product of a dynamic 1D expression/vector
template<class ValueType, class Expression1Type, class Expression2Type>
inline
ValueType vfc::linalg::intern::TExp1DDotProductImpl<ValueType, Expression1Type, Expression2Type, vfc::linalg::CDynamicRectangle> ::eval (
    const Expression1Type& f_operand1_r, 
    const Expression2Type& f_operand2_r)
{
    VFC_REQUIRE(f_operand1_r.getDim() == f_operand2_r.getDim());
    ValueType value = static_cast<ValueType>(0);
    for (vfc::int32_t i=0; i < f_operand1_r.getDim(); ++i)
    {
        value += f_operand1_r[i] * f_operand2_r[i];
    }
    return value;
}


//Evaluates the sum of a dynamic 1D expression/vector
template<class ValueType, class ExpressionType>
inline
ValueType vfc::linalg::intern::TExp1DSumImpl<ValueType, ExpressionType, vfc::linalg::CDynamicRectangle> ::eval (
    const ExpressionType& f_operand_r)
{
    ValueType value = static_cast<ValueType>(0);
    for (vfc::int32_t i = 0; i < f_operand_r.getDim(); i++)
    {
        value += f_operand_r[i];
    }
    return value;
}

//Evaluates the max of a dynamic 1D expression/vector
template<class ValueType, class ExpressionType>
inline
ValueType vfc::linalg::intern::TExp1DNormLinfOpImpl<ValueType, ExpressionType, vfc::linalg::CDynamicRectangle> ::eval (
    const ExpressionType& f_operand_r)
{
    ValueType value = static_cast<ValueType>(0);
    for (vfc::int32_t i = 0; i < f_operand_r.getDim(); i++)
    {
        value = (stlalias::max)(value, vfc::abs(f_operand_r[i]));
    }
    return value;
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_exp1d.inl  $
//  Revision 1.3 2009/05/28 09:19:03MESZ Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) 
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.2 2008/07/10 14:07:18CEST Dilip Krishna (RBEI/EAC1) (dkn2kor) 
//  - fixed min/max errors due to macro definitions of min & max (mantis2168)
//  Revision 1.1 2007/05/09 13:51:18IST Jaeger Thomas (CC-DA/ESV1) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
