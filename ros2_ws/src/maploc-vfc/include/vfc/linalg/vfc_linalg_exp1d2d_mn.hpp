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
///     $Source: vfc_linalg_exp1d2d_mn.hpp $
///     $Revision: 1.4 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2011/01/21 12:53:34MEZ $
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

#ifndef VFC_LINALG_EXP1D2D_MN_HPP_INCLUDED
#define VFC_LINALG_EXP1D2D_MN_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

#include "vfc/linalg/vfc_linalg_exp1d2d_common.hpp"
#include "vfc/linalg/vfc_linalg_shapepromotion.hpp"

namespace vfc
{   // namespace vfc open

    namespace linalg
    {   // namespace linalg open

        //-------------------------------------------------------------------------
        // conditional doxygen documentation
        //! @cond VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

        namespace intern
        {   // namespace intern open

            //=====================================================================
            // TUnroll1D2DMulOp
            //---------------------------------------------------------------------
            //! Template unrolling for 2D * 1D operation
            //! @param ValueType          Data Type
            //! @param Arg1Size2Value
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp1d2d_mn.hpp $
            //=====================================================================
            template<class ValueType, vfc::int32_t Arg1Size2Value>
            class TUnroll1D2DMulOp
            {
            public:

                //---------------------------------------------------------------------
                //! Evaluates for 2D * 1D operation
                //! @param f_row            Number of rows
                //! @param f_operand1_r     Expression of type Expression1Type
                //! @param f_operand2_r     Expression of type Expression2Type
                //! @return  Returns for 2D * 1D operation.
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d_mn.hpp $
                //---------------------------------------------------------------------
                template<class Expression1Type, class Expression2Type>
                inline
                static ValueType eval (
                    const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r,
                    vfc::int32_t f_row)
                {
                    return (f_operand1_r(f_row, Arg1Size2Value-1) * f_operand2_r[Arg1Size2Value-1])
                            +
                            TUnroll1D2DMulOp<ValueType, Arg1Size2Value-1 >::eval(
                            f_operand1_r, f_operand2_r, f_row);
                }
            };

            //=====================================================================
            // TUnroll1D2DMulOp
            //---------------------------------------------------------------------
            //! Template unrolling for 2D * 1D operation (Template Specialization)
            //! @param ValueType          Data Type
            //! @param Arg1Size2Value     1
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp1d2d_mn.hpp $
            //=====================================================================
            template <class ValueType>
            class TUnroll1D2DMulOp<ValueType, 1>
            {
            public:
                //---------------------------------------------------------------------
                //! Evaluates for 2D * 1D operation
                //! @param f_operand1_r     Expression of type Expression1Type
                //! @param f_operand2_r     Expression of type Expression2Type
                //! @param f_row            Number of rows
                //! @return  Returns for 2D * 1D operation.
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d_mn.hpp $
                //---------------------------------------------------------------------
                template <class Expression1Type, class Expression2Type>
                inline
                static ValueType eval (
                    const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r,
                    vfc::int32_t f_row)
                {
                    return f_operand1_r(f_row, 0) * f_operand2_r[0];
                }
            };

            //=====================================================================
            // TExp2D1DMulOpImpl
            //---------------------------------------------------------------------
            //! 2D * 1D implementation for a static expression
            //! @param ValueType          Data Type
            //! @param Expression1Type    Data Type
            //! @param Expression2Type    Data Type
            //! @param ShapeType          Data Type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp1d2d_mn.hpp $
            //=====================================================================
            template<class ValueType, class Expression1Type, class Expression2Type, class ShapeType>
            class TExp2D1DMulOpImpl
            {
            public:

                //---------------------------------------------------------------------
                //! Evaluates 2D * 1D, Switches between with/without template unrolling
                //! @param f_row            Number of rows
                //! @param f_operand1_r     Expression of type Expression1Type
                //! @param f_operand2_r     Expression of type Expression2Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d_mn.hpp $
                //---------------------------------------------------------------------
                inline
                static ValueType eval (
                    const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r,
                    vfc::int32_t f_row)
                {
                    return eval(f_operand1_r, f_operand2_r, f_row, typename intern::TMetaprogUnroll<Expression1Type::NB_ROWS, Expression1Type::NB_COLUMNS>::type());
                }
            private:

                //---------------------------------------------------------------------
                //! Evaluates 2D * 1D, With unrolling
                //! @param f_row            Number of rows
                //! @param f_operand1_r     Expression of type Expression1Type
                //! @param f_operand2_r     Expression of type Expression2Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d_mn.hpp $
                //---------------------------------------------------------------------
                inline
                static ValueType eval (
                    const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r,
                    vfc::int32_t f_row,
                    true_t)
                {
                    return  TUnroll1D2DMulOp<ValueType, Expression1Type::NB_COLUMNS>::eval(
                            f_operand1_r, f_operand2_r, f_row);
                }

                //---------------------------------------------------------------------
                //! Evaluates 2D * 1D, Without unrolling
                //! @param f_row            Number of rows
                //! @param f_operand1_r     Expression of type Expression1Type
                //! @param f_operand2_r     Expression of type Expression2Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d_mn.hpp $
                //---------------------------------------------------------------------
                inline
                static ValueType eval (
                    const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r,
                    vfc::int32_t f_row,
                    false_t)
                {
                    ValueType value = static_cast<ValueType>(0);
                    for (vfc::int32_t i=0; i < f_operand2_r.getDim(); ++i)
                    {
                        value += f_operand1_r(f_row, i) * f_operand2_r[i];
                    }
                    return value;
                }
            };

            //=====================================================================
            // TExp1DSubViewFrom2DOp
            //---------------------------------------------------------------------
            //! SubView operator for fixed size 2D static Expressions
            //! @param ValueType          Data Type
            //! @param ExpressionType     Data Type
            //! @param RowValue           Number of rows
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp1d2d_mn.hpp $
            //=====================================================================
            template <class ValueType, class ExpressionType, vfc::int32_t RowValue>
            class TExp1DSubViewFrom2DOp<ValueType, ExpressionType,
                vfc::linalg::TStaticRectangle<RowValue, 1> >
            {
            public:
                /// Value Type
                typedef ValueType                                           value_type;
                /// Shape Type
                typedef TStaticRectangle<RowValue, 1>                       shape_type;

                /// Size Type
                typedef vfc::int32_t                                        size_type;

                //---------------------------------------------------------------------
                //! Saves references to the operands
                //! Asserts if the inputs do not have valid dimensions
                //! @param f_operand_r      Expression data of type ExpressionType
                //! @param f_row0           Row Number
                //! @param f_column0        Column Number
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d_mn.hpp $
                //---------------------------------------------------------------------
                inline
                TExp1DSubViewFrom2DOp(const ExpressionType& f_operand_r,
                    size_type f_row0, size_type f_column0) :
                    m_operand(f_operand_r),
                    m_nbStartPos(f_row0*ExpressionType::NB_COLUMNS+f_column0)
                {
                    VFC_REQUIRE(0 <= f_row0 && 0 <= f_column0);

                     //check if the max size required is acceptanle
                    VFC_REQUIRE(shape_type::NB_ROWS <= ((ExpressionType::NB_ROWS *
                        ExpressionType::NB_COLUMNS) - m_nbStartPos));
                }

                //---------------------------------------------------------------------
                //! Returns a single element at the given position in the subview
                //! @param f_pos            Position of single element
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d_mn.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_pos) const
                {
                    VFC_REQUIRE(f_pos < shape_type::NB_ROWS && 0 <= f_pos);
                    size_type row = static_cast<size_type>((m_nbStartPos + f_pos)/
                            ExpressionType::NB_COLUMNS);
                    size_type col = (m_nbStartPos + f_pos)%ExpressionType::NB_COLUMNS;
                    return m_operand(row, col);
                }

                //---------------------------------------------------------------------
                //! Returns the dimension of the subview expression
                //! @param void
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d_mn.hpp $
                //---------------------------------------------------------------------
                inline size_type getDim(void) const   { return shape_type::NB_ROWS; }

            private:
                TExp1DSubViewFrom2DOp& operator=(
                    const TExp1DSubViewFrom2DOp<ValueType, ExpressionType,
                        TStaticRectangle<RowValue, 1> >&);

                /// Operand
                const ExpressionType m_operand;     //!< reference to expression
                size_type m_nbStartPos;             //!< start position in the expression
            };

        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed

#endif //VFC_LINALG_EXP1D2D_MN_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_exp1d2d_mn.hpp  $
//  Revision 1.4 2011/01/21 12:53:34MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - sometimes .hpp or namespace qualifiers are missing (mantis3599)
//  Revision 1.3 2009/03/25 14:48:17MEZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.2 2008/07/31 14:08:17IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:19IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
