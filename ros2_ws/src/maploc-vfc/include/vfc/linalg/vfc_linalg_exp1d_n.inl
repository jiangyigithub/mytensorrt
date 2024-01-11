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
///     $Source: vfc_linalg_exp1d_n.inl $
///     $Revision: 1.4 $
///     $Author: Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) $
///     $Date: 2009/05/28 09:19:04MESZ $
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
#include "vfc/linalg/vfc_linalg_algorithm1d.hpp"
#include "vfc/core/vfc_math.hpp"

namespace vfc
{   // namespace vfc opened

    namespace linalg
    {   // namespace linalg opened

        //-------------------------------------------------------------------------
        // conditional doxygen documentation
        //! @cond VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

        namespace intern
        {   // namespace intern opened

            /// Unrolls a 1D Expression to evaluate sum
            template<class ValueType, vfc::int32_t RowValue>
            class TUnroll1DOp
            {
                public:
                /// Unrolls the 1D Expression
                /// Returns the sum
                template<class ExpressionType>
                static
                inline
                ValueType evalSum (const ExpressionType& f_operand_r)
                {
                    return f_operand_r[RowValue-1] +
                        TUnroll1DOp<ValueType, RowValue-1>::evalSum(f_operand_r);
                }

                /// Unrolls the 1D Expression
                /// Returns the max
                template<class ExpressionType>
                static
                inline
                ValueType evalNormLinf (const ExpressionType& f_operand_r)
                {
                    return (stlalias::max)(vfc::abs(f_operand_r[RowValue-1]),
                        TUnroll1DOp<ValueType, RowValue-1>::evalNormLinf(f_operand_r));
                }
            };

            template<class ValueType>
            class TUnroll1DOp<ValueType, 1>
            {
                public:
                /// Unrolls the 1D Expression
                /// Returns the sum
                template<class ExpressionType>
                static
                inline
                ValueType evalSum (const ExpressionType& f_operand_r)
                {
                    return f_operand_r[0];
                }

                /// Unrolls the 1D Expression
                /// Returns the max
                template<class ExpressionType>
                static
                inline
                ValueType evalNormLinf (const ExpressionType& f_operand_r)
                {
                    return vfc::abs(f_operand_r[0]);
                }
            };


        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed
template<class ValueType, vfc::int32_t RowValue>
template<class Expression1Type, class Expression2Type>
inline ValueType vfc::linalg::intern::TUnroll1DDotProductOp<ValueType, RowValue>::eval (
    const Expression1Type& f_operand1_r,
    const Expression2Type& f_operand2_r)
{
    return (f_operand1_r[RowValue-1 ] * f_operand2_r[RowValue-1])
            +
            intern::TUnroll1DDotProductOp<ValueType, RowValue-1 >::eval(
            f_operand1_r, f_operand2_r);
}

template<class ValueType, class Expression1Type, class Expression2Type, vfc::int32_t RowValue>
inline ValueType vfc::linalg::intern::TExp1DDotProductImpl<ValueType, Expression1Type, Expression2Type, vfc::linalg::TStaticRectangle<RowValue, 1> >::eval (
    const Expression1Type& f_operand1_r,
    const Expression2Type& f_operand2_r)
{
    return eval(f_operand1_r, f_operand2_r, typename intern::TMetaprogUnroll<RowValue, 1>::type());
}

template<class ValueType, class Expression1Type, class Expression2Type, vfc::int32_t RowValue>
inline
ValueType vfc::linalg::intern::TExp1DDotProductImpl<ValueType, Expression1Type, Expression2Type, vfc::linalg::TStaticRectangle<RowValue, 1> >::eval (
    const Expression1Type& f_operand1_r,
    const Expression2Type& f_operand2_r,
    true_t)
{
    return  intern::TUnroll1DDotProductOp<ValueType, RowValue>::eval(
            f_operand1_r, f_operand2_r);
}

template<class ValueType, class Expression1Type, class Expression2Type, vfc::int32_t RowValue>
inline
ValueType vfc::linalg::intern::TExp1DDotProductImpl<ValueType, Expression1Type, Expression2Type, vfc::linalg::TStaticRectangle<RowValue, 1> >::eval (
    const Expression1Type& f_operand1_r,
    const Expression2Type& f_operand2_r,
    false_t)
{
    ValueType value = static_cast<ValueType>(0);
    for (vfc::int32_t i=0; i < f_operand1_r.getDim(); ++i)
    {
        value += f_operand1_r[i] * f_operand2_r[i];
    }
    return value;
}

template<class ValueType, class ExpressionType, vfc::int32_t RowValue>
inline
ValueType
vfc::linalg::intern::TExp1DSumImpl<ValueType, ExpressionType,
    vfc::linalg::TStaticRectangle<RowValue, 1> >::eval (
    const ExpressionType& f_operand_r)
{
    return eval(f_operand_r, typename intern::TMetaprogUnroll<RowValue, 1>::type());
}

template<class ValueType, class ExpressionType, vfc::int32_t RowValue>
inline
ValueType
vfc::linalg::intern::TExp1DSumImpl<ValueType, ExpressionType,
    vfc::linalg::TStaticRectangle<RowValue, 1> >::eval (
    const ExpressionType& f_operand_r, true_t)
{
    return intern::TUnroll1DOp<ValueType, RowValue>::evalSum(f_operand_r);
}

template<class ValueType, class ExpressionType, vfc::int32_t RowValue>
inline
ValueType
vfc::linalg::intern::TExp1DSumImpl<ValueType, ExpressionType,
    vfc::linalg::TStaticRectangle<RowValue, 1> >::eval (
    const ExpressionType& f_operand_r, false_t)
{
    ValueType value = static_cast<ValueType>(0);
    for (vfc::int32_t i = 0; i < f_operand_r.getDim(); i++)
    {
        value += f_operand_r[i];
    }
    return value;
}

template<class ValueType, class ExpressionType, vfc::int32_t RowValue>
inline
ValueType
vfc::linalg::intern::TExp1DNormLinfOpImpl<ValueType, ExpressionType,
    vfc::linalg::TStaticRectangle<RowValue, 1> >::eval (
    const ExpressionType& f_operand_r)
{
    //In this implementation a meta program unrolled implementation is slower than
    //for loop implementation. So using the for loop implementation.
    //Needs evaluation in target compilers.
    return eval(f_operand_r, false_t());
    //return eval(f_operand_r, typename intern::TMetaprogUnroll<RowValue, 1>::type());
}

template<class ValueType, class ExpressionType, vfc::int32_t RowValue>
inline
ValueType
vfc::linalg::intern::TExp1DNormLinfOpImpl<ValueType, ExpressionType,
    vfc::linalg::TStaticRectangle<RowValue, 1> >::eval (
    const ExpressionType& f_operand_r, true_t)
{
    return intern::TUnroll1DOp<ValueType, RowValue>::evalNormLinf(f_operand_r);
}

template<class ValueType, class ExpressionType, vfc::int32_t RowValue>
inline
ValueType
vfc::linalg::intern::TExp1DNormLinfOpImpl<ValueType, ExpressionType,
    vfc::linalg::TStaticRectangle<RowValue, 1> >::eval (
    const ExpressionType& f_operand_r, false_t)
{
    ValueType value = static_cast<ValueType>(0);
    for (vfc::int32_t i = 0; i < RowValue; i++)
    {
        value = (stlalias::max)(value, vfc::abs(f_operand_r[i]));
    }
    return value;
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_exp1d_n.inl  $
//  Revision 1.4 2009/05/28 09:19:04MESZ Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) 
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.3 2009/03/25 14:48:14CET Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.2 2008/07/10 17:37:24IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  - fixed min/max errors due to macro definitions of min & max (mantis2168)
//  Revision 1.1 2007/05/09 13:51:19IST Jaeger Thomas (CC-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
