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
///     $Source: vfc_linalg_exp1d_n.hpp $
///     $Revision: 1.5 $
///     $Author: gaj2kor $
///     $Date: 2009/03/25 14:48:13MEZ $
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

#ifndef VFC_LINALG_EXP1D_N_HPP_INCLUDED
#define VFC_LINALG_EXP1D_N_HPP_INCLUDED

#include "vfc/linalg/vfc_linalg_exp1d_common.hpp"
#include "vfc/linalg/vfc_linalg_shapepromotion.hpp"

namespace vfc
{   // namespace vfc opened

    namespace linalg
    {   // namespace linalg opened

        //-------------------------------------------------------------------------
        // conditional doxygen documentation
        //! @cond VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

        namespace intern
        {   // namespace intern  opened

            //=============================================================================
            // TExp1D
            //-----------------------------------------------------------------------------
            //! Static 1D Expression
            //! @param ValueType          Data Type
            //! @param OperatorType       Data Type
            //! @param RowValue           Number of Rows
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_n.hpp $
            //=============================================================================
            template <class ValueType, class OperatorType,
                    vfc::int32_t RowValue>
            class TExp1D<ValueType, OperatorType,
                TStaticRectangle<RowValue, 1> >
            {
            public:
                /// Value Type
                typedef ValueType                               value_type;
                /// Shape Type
                typedef TStaticRectangle<RowValue, 1>           shape_type;
                /// Size Type
                typedef vfc::int32_t                            size_type;

                enum { NB_ROWS = shape_type::NB_ROWS };

                //---------------------------------------------------------------------
                //! Constructor to construct an expression from an operator
                //! Saves reference to the operator
                //! @param f_operand_r      Expression Data of type ExpressionType
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                inline
                TExp1D(const OperatorType& f_operator_r)
                    : m_operator(f_operator_r) { }

                //---------------------------------------------------------------------
                //! Evaluates operator [] for single element
                //! @param f_row      Row number
                //! @return  Returns a single element at the given position
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                inline value_type operator[](size_type f_row) const
                {
                    return m_operator.eval(f_row);
                }

                //---------------------------------------------------------------------
                //! Returns dimension of the 1D Expression
                //! @param void
                //! @return  Returns dimension of the 1D Expression
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                inline size_type getDim() const { return NB_ROWS; }

            private:
                TExp1D& operator=(const TExp1D<ValueType, OperatorType, TStaticRectangle<RowValue, 1> >&);
                // Operator
                const OperatorType m_operator;

                //Dummy function for suppression of QAC++ msg 2141
                //Msg 2141 : The subscript 'operator []' is not available in a non-const version.
                value_type operator[](size_type f_row);

            };

            //=============================================================================
            // TUnroll1DDotProductOp
            //-----------------------------------------------------------------------------
            //! Unrolls a 1D Expression to evaluate Dot Product
            //! @param ValueType          Data Type
            //! @param RowValue           Number of Rows
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_n.hpp $
            //=============================================================================
            template<class ValueType, vfc::int32_t RowValue>
            class TUnroll1DDotProductOp
            {
            public:

                //---------------------------------------------------------------------
                //! Unrolls the 1D Expression
                //! Returns the Dot Product
                //! @param f_operand1_r         Expression of type Expression1Type
                //! @param f_operand2_r         Expression of type Expression2Type
                //! @return  Returns the Dot Product
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                template<class Expression1Type, class Expression2Type>
                static
                ValueType eval (
                    const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r);
            };

            //=============================================================================
            // TUnroll1DDotProductOp
            //-----------------------------------------------------------------------------
            //! Unrolls a 1D Expression to evaluate Dot Product. Template Specialization
            //! @param ValueType          Data Type
            //! @param RowValue           1
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_n.hpp $
            //=============================================================================
            template <class ValueType>
            class TUnroll1DDotProductOp<ValueType, 1>
            {
            public:
                //---------------------------------------------------------------------
                //! Returns the Scalar Product
                //! @param f_arg1_r         Expression of type Expression1Type
                //! @param f_arg2_r         Expression of type Expression2Type
                //! @return  Returns the Scalar Product
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                template <class Expression1Type, class Expression2Type>
                inline
                static ValueType eval (
                    const Expression1Type& f_arg1_r,
                    const Expression2Type& f_arg2_r)
                {
                    return f_arg1_r[0]*f_arg2_r[0];
                }
            };

            //=============================================================================
            // TExp1DDotProductImpl
            //-----------------------------------------------------------------------------
            //! Dot product implementation for a static Vector/1D Expression
            //! @param ValueType          Data Type
            //! @param Expression1Type    Data Type
            //! @param Expression2Type    Data Type
            //! @param RowValue           Number of Rows
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_n.hpp $
            //=============================================================================
            template<class ValueType, class Expression1Type, class Expression2Type, vfc::int32_t RowValue>
            class TExp1DDotProductImpl<ValueType, Expression1Type, Expression2Type, TStaticRectangle<RowValue, 1> >
            {
            public:

                /// Value Type
                typedef ValueType                       value_type;
                /// Shape Type
                typedef TStaticRectangle<RowValue, 1>   shape_type;
                /// Size Type
                typedef vfc::int32_t                    size_type;

                //---------------------------------------------------------------------
                //! Evaluates the dot product of a static vector/1D expression
                //! Switches between with/without unrolling
                //! @param f_operand1_r         Expression of type Expression1Type
                //! @param f_operand2_r         Expression of type Expression2Type
                //! @return  Returns the dot Product
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                static
                ValueType eval (
                    const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r);
            private:

                //---------------------------------------------------------------------
                //! Returns the dot product using template unrolling
                //! @param f_operand1_r         Expression of type Expression1Type
                //! @param f_operand2_r         Expression of type Expression2Type
                //! @return  Returns the dot product using template unrolling
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                static ValueType eval (
                    const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r,
                    true_t);

                //---------------------------------------------------------------------
                //! Returns the dot product without using template unrolling
                //! @param f_operand1_r         Expression of type Expression1Type
                //! @param f_operand2_r         Expression of type Expression2Type
                //! @return  Returns the dot product without using template unrolling
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                static ValueType eval (
                    const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r,
                    false_t);
            };


            //=============================================================================
            // TExp1DSumImpl
            //---------------------------------------------------------------------
            //! Sum implementation for a static Vector/1D Expression
            //! @param ValueType          Data Type
            //! @param ExpressionType     Data Type
            //! @param RowValue           Number of Rows
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_n.hpp $
            //=============================================================================
            template<class ValueType, class ExpressionType, vfc::int32_t RowValue>
            class TExp1DSumImpl<ValueType, ExpressionType, TStaticRectangle<RowValue, 1> >
            {
            public:

                /// Value Type
                typedef ValueType                       value_type;
                /// Shape Type
                typedef TStaticRectangle<RowValue, 1>   shape_type;
                /// Size Type
                typedef vfc::int32_t                    size_type;

                //---------------------------------------------------------------------
                //! Evaluates the dot product of a static vector/1D expression
                //! Switches between with/without unrolling
                //! @param f_operand_r         Expression of type ExpressionType
                //! @return  Returns the dot Product
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                static
                ValueType eval (const ExpressionType& f_operand_r);
            private:
                //---------------------------------------------------------------------
                //! Returns the sum using template unrolling
                //! @param f_operand_r         Expression of type ExpressionType
                //! @return  Returns the sum using template unrolling
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                static ValueType eval (
                    const ExpressionType& f_operand_r,
                    true_t);

                //---------------------------------------------------------------------
                //! Returns the sum without using template unrolling
                //! @param f_operand_r         Expression of type ExpressionType
                //! @return  Returns the sum without using template unrolling
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                static ValueType eval (
                    const ExpressionType& f_operand_r,
                    false_t);
            };

            //=============================================================================
            // TExp1DNormLinfOpImpl
            //---------------------------------------------------------------------
            //! Norm implementation for a static Vector/1D Expression
            //! @param ValueType          Data Type
            //! @param ExpressionType     Data Type
            //! @param RowValue           Number of Rows
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_n.hpp $
            //=============================================================================
            template<class ValueType, class ExpressionType, vfc::int32_t RowValue>
            class TExp1DNormLinfOpImpl<ValueType, ExpressionType, TStaticRectangle<RowValue, 1> >
            {
            public:

                /// Value Type
                typedef ValueType                       value_type;
                /// Shape Type
                typedef TStaticRectangle<RowValue, 1>   shape_type;
                /// Size Type
                typedef vfc::int32_t                    size_type;

                //---------------------------------------------------------------------
                //! Evaluates the dot product of a static vector/1D expression
                //! Switches between with/without unrolling
                //! @param f_operand_r         Expression of type ExpressionType
                //! @return  Returns the dot Product
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                static
                ValueType eval (const ExpressionType& f_operand_r);
            private:
                //---------------------------------------------------------------------
                //! Returns the norm using template unrolling
                //! @param f_operand_r         Expression of type ExpressionType
                //! @param true_t
                //! @return  Returns the norm using template unrolling
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                static ValueType eval (
                    const ExpressionType& f_operand_r,
                    true_t);

                //---------------------------------------------------------------------
                //! Returns the norm without using template unrolling
                //! @param f_operand_r         Expression of type ExpressionType
                //! @param false_t
                //! @return  Returns the norm without using template unrolling
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                static ValueType eval (
                    const ExpressionType& f_operand_r,
                    false_t);
            };

            //=============================================================================
            // TExp1DSubViewFrom1DOp
            //---------------------------------------------------------------------
            //! SubView operator for fixed size 1D Expressions
            //! @param ValueType          Data Type
            //! @param ExpressionType     Data Type
            //! @param RowValue           Number of Rows
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_n.hpp $
            //=============================================================================
            template <class ValueType, class ExpressionType,
                vfc::int32_t RowValue>
            class TExp1DSubViewFrom1DOp<ValueType, ExpressionType,
                TStaticRectangle<RowValue, 1> >
            {
            public:

                /// Value Type
                typedef ValueType                                           value_type;
                /// Shape Type
                typedef TStaticRectangle<RowValue, 1>                       shape_type;

                /// Size Type
                typedef vfc::int32_t                                        size_type;

                //---------------------------------------------------------------------
                //! Constructor
                //! Saves references to the operands
                //! Asserts if the inputs do not have valid dimensions
                //! @param f_operand_r      Expression Data of type ExpressionType
                //! @param f_pos0           Index Position
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                inline
                TExp1DSubViewFrom1DOp(const ExpressionType& f_operand_r,
                    size_type f_pos0) :
                    m_operand(f_operand_r), m_nbStartPos(f_pos0)
                {
                    VFC_REQUIRE(0 <= f_pos0);
                    //check if the max size required is acceptanle
                    VFC_REQUIRE(shape_type::NB_ROWS <= (ExpressionType::NB_ROWS - f_pos0));
                }

                //---------------------------------------------------------------------
                //! Returns a single element at the given position in the subview
                //! @param f_pos         Index Position
                //! @return  Returns a single element at the given position in the subview
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_pos) const
                {
                    VFC_REQUIRE(f_pos < shape_type::NB_ROWS && 0 <= f_pos);
                    return m_operand[m_nbStartPos+f_pos];
                }

                //---------------------------------------------------------------------
                //! Returns the dimension of the expression
                //! @param void
                //! @return  Returns the dimension of the expression
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_n.hpp $
                //---------------------------------------------------------------------
                inline size_type getDim(void) const   { return shape_type::NB_ROWS; }

            private:
                TExp1DSubViewFrom1DOp& operator=(
                    const TExp1DSubViewFrom1DOp<ValueType, ExpressionType,
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

#include "vfc/linalg/vfc_linalg_exp1d_n.inl"

#endif //VFC_LINALG_EXP1D_N_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_exp1d_n.hpp  $
//  Revision 1.5 2009/03/25 14:48:13MEZ gaj2kor 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.4 2009/01/07 10:19:27IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  QAC++ Warning (2141) Removal.
//  (Mantis : 0002479)
//  Revision 1.3 2008/07/31 14:08:14IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.2 2007/06/22 18:40:00IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  - corrected redeclared function parameter names (mantis 1692)
//  Revision 1.1 2007/05/09 10:21:19CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
