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
///     $Source: vfc_linalg_exp2d_mn.inl $
///     $Revision: 1.2 $
///     $Author: gaj2kor $
///     $Date: 2009/03/25 14:48:20MEZ $
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

#include "vfc/linalg/vfc_linalg_algorithm2d.hpp"

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

            template <class ValueType, vfc::int32_t Arg1Size2Value>
            struct TUnroll2DMul
            {
                //Used when the unrolling is required
                template <class Arg1Type, class Arg2Type>
                static ValueType eval (
                    const Arg1Type& f_arg1,
                    const Arg2Type& f_arg2,
                    vfc::int32_t f_row, vfc::int32_t f_col, true_t)
                {
                    return (f_arg1(f_row, Arg1Size2Value) *
                            f_arg2(Arg1Size2Value, f_col)) +
                        intern::TUnroll2DMul<ValueType, Arg1Size2Value-1>::eval(
                        f_arg1, f_arg2, f_row, f_col, true_t());
                }

                //Used when the unrolling is not required
                template <class Arg1Type, class Arg2Type>
                static ValueType eval (
                    const Arg1Type& f_arg1,
                    const Arg2Type& f_arg2,
                    vfc::int32_t f_row, vfc::int32_t f_col, false_t)
                {
                    ValueType value = static_cast<ValueType>(0);
                    for (vfc::int32_t i=0; i < f_arg1.getNbColumns(); ++i)
                    {
                        value += f_arg1(f_row, i) * f_arg2(i, f_col);
                    }
                    return value;
                }
            };

            template <class ValueType>
            struct TUnroll2DMul<ValueType, 0>
            {
                template <class Arg1Type, class Arg2Type>
                static ValueType eval (
                    const Arg1Type& f_arg1,
                    const Arg2Type& f_arg2,
                    vfc::int32_t f_row, vfc::int32_t f_col, true_t)
                {
                    return f_arg1(f_row, 0)*f_arg2(0, f_col);
                }
            };


        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed

template<class ValueType, class Expression1Type, class Expression2Type,
    class ShapeType>
inline
ValueType
vfc::linalg::intern::TExp2DMulOpImpl<ValueType, Expression1Type,
Expression2Type, ShapeType>::eval(
    const Expression1Type& f_operand1_r,
    const Expression2Type& f_operand2_r,
    size_type f_row, size_type f_col)
{
    return intern::TUnroll2DMul<value_type,
        Expression1Type::NB_COLUMNS-1>::eval(f_operand1_r,
            f_operand2_r, f_row, f_col,
            typename vfc::linalg::intern::TMetaprogUnroll<ShapeType::NB_ROWS,
                ShapeType::NB_COLUMNS>::type());
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_exp2d_mn.inl  $
//  Revision 1.2 2009/03/25 14:48:20MEZ gaj2kor 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.1 2007/05/09 13:51:20IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
