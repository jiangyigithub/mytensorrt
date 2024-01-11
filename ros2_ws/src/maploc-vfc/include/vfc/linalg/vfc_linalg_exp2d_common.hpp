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
///     $Source: vfc_linalg_exp2d_common.hpp $
///     $Revision: 1.5 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2011/01/21 12:53:28MEZ $
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

#ifndef VFC_LINALG_EXP2D_COMMON_HPP_INCLUDED
#define VFC_LINALG_EXP2D_COMMON_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_assert.hpp"
#include "vfc/core/vfc_type_traits.hpp"

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
        {   // namespace intern opened

            //=====================================================================
            // TExp2D
            //---------------------------------------------------------------------
            //! Template container for 2d expression, wraps 2D operators
            //! @param ValueType           Data Type
            //! @param OperatorType        Data Type
            //! @param ShapeType           Data Type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d_common.hpp $
            //=====================================================================
            template <class ValueType, class OperatorType, class ShapeType>
            class TExp2D
            {
            public:
                /// Value Type
                typedef ValueType       value_type;
                /// Shape type
                typedef ShapeType       shape_type;
                /// Size type
                typedef vfc::int32_t     size_type;

                enum { NB_ROWS = ShapeType::NB_ROWS };
                enum { NB_COLUMNS = ShapeType::NB_COLUMNS };

                //---------------------------------------------------------------------
                //! Constructs expression from given operator
                //! @param f_operator_r        2D Expression
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline
                TExp2D(const OperatorType& f_operator_r) : m_operator(f_operator_r) { }

                //---------------------------------------------------------------------
                //! Evaluates operator() for single element
                //! @param f_row        Number of rows
                //! @param f_col        Number of column
                //! @returns Returns value of expression at specific row and column
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type operator()(size_type f_row, size_type f_col) const
                {
                    return m_operator.eval(f_row, f_col);
                }

                //---------------------------------------------------------------------
                //! Returns column dimension 2D expression
                //! @param void
                //! @returns Returns number of column
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbColumns() const { return m_operator.getNbColumns(); }

                //---------------------------------------------------------------------
                //! Returns row dimension 2D expression
                //! @param void
                //! @returns Returns number of row
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbRows() const { return m_operator.getNbRows(); }
            private:
                const OperatorType m_operator; //!< operator wrapped in the expression to be evaluated
            };

            //=====================================================================
            // TExp2DAddOp
            //---------------------------------------------------------------------
            //! 2D expression add operator
            //! @param ValueType            Data Type
            //! @param Expression1Type      Data Type
            //! @param Expression2Type      Data Type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d_common.hpp $
            //=====================================================================
            template <class ValueType, class Expression1Type, class Expression2Type>
            class TExp2DAddOp
            {
            public:

                /// Value Type
                typedef ValueType                                   value_type;
                /// Shape type after shape promotion
                typedef typename vfc::linalg::TBinaryShapePromotion<
                    typename Expression1Type::shape_type,
                    typename Expression2Type::shape_type>::type     shape_type;
                /// Size type
                typedef vfc::int32_t                                 size_type;

                //---------------------------------------------------------------------
                //! Saves refernces to both operands, asserts if operands dimensions does not fit.
                //! @param f_operand1_r     2D Expresstion of type Expression1Type
                //! @param f_operand2_r     2D Expresstion of type Expression2Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline TExp2DAddOp(const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r) :
                        m_operand1(f_operand1_r), m_operand2(f_operand2_r)
                {
                    VFC_REQUIRE( f_operand1_r.getNbRows() == f_operand2_r.getNbRows() && \
                        f_operand1_r.getNbColumns() == f_operand2_r.getNbColumns() );
                }

                //---------------------------------------------------------------------
                //! Evaluates addition operation for single element
                //! @param f_row     Number of rows
                //! @param f_col     Number of column
                //! @returns Returns addition of (single element) m_operand1 with m_operand2
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval (size_type f_row, size_type f_col) const
                {
                    return m_operand1(f_row, f_col) + m_operand2(f_row, f_col);
                }

                //---------------------------------------------------------------------
                //! Returns column dimension of resulting 2D expression
                //! @param void
                //! @returns Returns number of columns
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbColumns() const { return m_operand1.getNbColumns(); }

                //---------------------------------------------------------------------
                //! Returns row dimension of resulting 2D expression
                //! @param void
                //! @returns Returns number of row
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbRows() const { return m_operand1.getNbRows(); }
            private:
                TExp2DAddOp& operator=(
                    const TExp2DAddOp<ValueType, Expression1Type, Expression2Type>&);

                const Expression1Type m_operand1;    //!< first operand
                const Expression2Type m_operand2;    //!< second operand
            };

            //=====================================================================
            // TExp2DSubOp
            //---------------------------------------------------------------------
            //! 2D expression subtraction operator
            //! @param ValueType            Data Type
            //! @param Expression1Type      Data Type
            //! @param Expression2Type      Data Type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d_common.hpp $
            //=====================================================================
            template <class ValueType, class Expression1Type, class Expression2Type>
            class TExp2DSubOp
            {
            public:
                /// Value Type
                typedef ValueType                                   value_type;
                /// Shape type after shape promotion
                typedef typename TBinaryShapePromotion<
                    typename Expression1Type::shape_type,
                    typename Expression2Type::shape_type>::type     shape_type;
                /// Size type
                typedef vfc::int32_t                                 size_type;

                //---------------------------------------------------------------------
                //! Saves refernces to both operands, asserts if operands dimensions does not fit.
                //! @param f_operand1_r     2D Expresstion of type Expression1Type
                //! @param f_operand2_r     2D Expresstion of type Expression2Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline TExp2DSubOp(const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r) :
                        m_operand1(f_operand1_r), m_operand2(f_operand2_r)
                {
                    VFC_REQUIRE( f_operand1_r.getNbRows() == f_operand2_r.getNbRows() && \
                        f_operand1_r.getNbColumns() == f_operand2_r.getNbColumns() );
                }

                //---------------------------------------------------------------------
                //! Evaluates subtraction operation for single element
                //! @param f_row     Number of rows
                //! @param f_col     Number of column
                //! @returns Returns subtraction of (single element) m_operand1 with m_operand2
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval (size_type f_row, size_type f_col) const
                {
                    return m_operand1(f_row, f_col) - m_operand2(f_row, f_col);
                }

                //---------------------------------------------------------------------
                //! Returns column dimension of resulting 2D expression
                //! @param void
                //! @returns Returns number of columns
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbColumns() const { return m_operand1.getNbColumns(); }

                //---------------------------------------------------------------------
                //! Returns row dimension of resulting 2D expression
                //! @param void
                //! @returns Returns number of row
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                size_type getNbRows() const { return m_operand1.getNbRows(); }
            private:
                TExp2DSubOp& operator=(const TExp2DSubOp<ValueType, Expression1Type,
                    Expression2Type>&);

                const Expression1Type m_operand1;    //!< first operand
                const Expression2Type m_operand2;    //!< second operand
            };

            //=====================================================================
            // TExp2DNegateOp
            //---------------------------------------------------------------------
            //! 2D expression negate operator
            //! @param ValueType            Data Type
            //! @param ExpressionType       Data Type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d_common.hpp $
            //=====================================================================
            template <class ValueType, class ExpressionType>
            class TExp2DNegateOp
            {
            public:
                /// Value Type
                typedef ValueType                               value_type;
                /// Shape type
                typedef typename ExpressionType::shape_type     shape_type;
                /// Size type
                typedef vfc::int32_t                             size_type;
                /// constructor, saves the refernce to operand
                inline TExp2DNegateOp(const ExpressionType& f_operand_r)
                    : m_operand(f_operand_r) { }

                //---------------------------------------------------------------------
                //! Evaluates negate operation for single element
                //! @param f_row     Number of rows
                //! @param f_col     Number of column
                //! @returns Returns negate of single element
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_row, size_type f_col) const
                {
                    return -(m_operand(f_row, f_col));
                }

                //---------------------------------------------------------------------
                //! Returns row dimension of resulting 2D expression
                //! @param void
                //! @returns Returns number of rows
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbRows(void) const   { return m_operand.getNbRows(); }

                //---------------------------------------------------------------------
                //! Returns column dimension of resulting 2D expression
                //! @param void
                //! @returns Returns number of column
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type  getNbColumns(void) const   { return m_operand.getNbColumns(); }
            private:
                TExp2DNegateOp& operator=(const TExp2DNegateOp<ValueType,
                    ExpressionType>&);
                const ExpressionType m_operand;    //!< operand
            };

            //=====================================================================
            // TExp2DTransposeOp
            //---------------------------------------------------------------------
            //! 2D expression transpose operator
            //! @param ValueType            Data Type
            //! @param ExpressionType       Data Type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d_common.hpp $
            //=====================================================================
            template <class ValueType, class ExpressionType>
            class TExp2DTransposeOp
            {
            public:
                /// Value Type
                typedef ValueType                                   value_type;
                /// Shape type of the transposed matrix
                typedef typename
                     TTransposeMatrixShapePromotion<typename
                        ExpressionType::shape_type>::shape_type     shape_type;
                /// Size type
                typedef vfc::int32_t                                size_type;

                //---------------------------------------------------------------------
                //! Saves refernces to operand
                //! @param f_operand_r     2D Expresstion of type ExpressionType
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline TExp2DTransposeOp(const ExpressionType& f_operand_r)
                    : m_operand(f_operand_r)     { }

                //---------------------------------------------------------------------
                //! Evaluates transpose operation for single element
                //! @param f_row     Number of rows
                //! @param f_col     Number of column
                //! @returns Returns value at specific row and column
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_row, size_type f_col) const
                {
                    return (m_operand(f_col, f_row));
                }

                //---------------------------------------------------------------------
                //! Returns row dimension of resulting 2D expression
                //! @param void
                //! @returns Returns number of row
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbRows(void) const { return m_operand.getNbColumns(); }

                //---------------------------------------------------------------------
                //! Returns column dimension of resulting 2D expression
                //! @param void
                //! @returns Returns number of column
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbColumns(void) const { return m_operand.getNbRows(); }
            private:
                TExp2DTransposeOp& operator=(const TExp2DTransposeOp<ValueType,
                    ExpressionType>&);
                const ExpressionType m_operand;    //!< first operator
            };

            //=====================================================================
            // TExp2DScalarOp
            //---------------------------------------------------------------------
            //! 2D expression scalar operator used by scalar multiplication and division
            //! @param ValueType            Data Type
            //! @param ExpressionType       Data Type
            //! @param OperationFunctorType Data Type for function pointer
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d_common.hpp $
            //=====================================================================
            template <class ValueType, class ExpressionType, class OperationFunctorType>
            class TExp2DScalarOp
            {
            public:
                /// Value Type
                typedef ValueType                               value_type;
                /// Shape type
                typedef typename ExpressionType::shape_type     shape_type;
                /// Size type
                typedef vfc::int32_t                             size_type;

                //---------------------------------------------------------------------
                //! Saves refernces to both operands
                //! @param f_operand1_r     2D Expresstion of type Expression1Type
                //! @param f_operand2_r     2D Expresstion of type Expression2Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline TExp2DScalarOp(const ExpressionType& f_operand1_r,
                    const ValueType& f_operand2_r)
                    : m_operand1(f_operand1_r), m_operand2(f_operand2_r) { }

                //---------------------------------------------------------------------
                //! Evaluates static operator func for single element
                //! @param f_row     Number of rows
                //! @param f_col     Number of column
                //! @returns Returns static operator func for single element
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_row, size_type f_col) const
                {
                    return OperationFunctorType()(m_operand1(f_row, f_col),
                        m_operand2);
                }

                //---------------------------------------------------------------------
                //! Returns row dimension of resulting 2D expression
                //! @param void
                //! @returns Returns number of rows
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbRows(void) const
                {
                    return m_operand1.getNbRows();
                }

                //---------------------------------------------------------------------
                //! Returns column dimension of resulting 2D expression
                //! @param void
                //! @returns Returns number of column
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbColumns(void) const
                {
                    return m_operand1.getNbColumns();
                }
            private:
                TExp2DScalarOp& operator=(const TExp2DScalarOp<ValueType,
                    ExpressionType, OperationFunctorType>&);
                const ExpressionType m_operand1;    //!< first operand
                value_type            m_operand2;    //!< second operand
            };

            //=====================================================================
            // TExp2DDivScalarOp
            //---------------------------------------------------------------------
            //! 2D expression scalar operator used by scalar multiplication and division
            //! @param ValueType            Data Type
            //! @param ExpressionType       Data Type
            //! @param OperationFunctorType Data Type for function pointer
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d_common.hpp $
            //=====================================================================
            template <class ValueType, class ExpressionType, class OperationFunctorType>
            class TExp2DDivScalarOp
            {
            public:
                /// Value Type
                typedef ValueType                               value_type;
                /// Shape type
                typedef typename ExpressionType::shape_type     shape_type;
                /// Size type
                typedef vfc::int32_t                            size_type;

                //---------------------------------------------------------------------
                //! Saves refernces to both operands
                //! @param f_operand1_r     2D Expresstion of type Expression1Type
                //! @param f_operand2_r     2D Expresstion of type Expression2Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline TExp2DDivScalarOp(const ExpressionType& f_operand1_r,
                    const ValueType& f_operand2_r)
                    : m_operand1(f_operand1_r)
                {
                    setScalarIntern(f_operand2_r, typename vfc::TInt2Boolean<vfc::TIsFloating<value_type>::value>::type());
                }

                //---------------------------------------------------------------------
                //! Evaluates static operator func for single element
                //! @param f_row     Number of rows
                //! @param f_col     Number of column
                //! @returns Returns static operator func for single element
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_row, size_type f_col) const
                {
                    //return OperationFunctorType()(m_operand1(f_row, f_col),
                    //    m_operand2);
                    return evalIntern(f_row, f_col, typename vfc::TInt2Boolean<vfc::TIsFloating<value_type>::value>::type());
                }

                //---------------------------------------------------------------------
                //! Returns row dimension of resulting 2D expression
                //! @param void
                //! @returns Returns number of row
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbRows(void) const
                {
                    return m_operand1.getNbRows();
                }

                //---------------------------------------------------------------------
                //! Returns column dimension of resulting 2D expression
                //! @param void
                //! @returns Returns number of column
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbColumns(void) const
                {
                    return m_operand1.getNbColumns();
                }
            private:
                TExp2DDivScalarOp& operator=(const TExp2DDivScalarOp<ValueType,
                    ExpressionType, OperationFunctorType>&);

                //---------------------------------------------------------------------
                //! Set m_operand2 with inverse of f_scalar
                //! @param f_scalar     Scalar value
                //! @param true_t
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline
                void    setScalarIntern(const value_type& f_scalar, true_t)
                {
                    m_operand2 = static_cast<value_type>(1)/f_scalar;
                }

                //---------------------------------------------------------------------
                //! Set m_operand2 with f_scalar (Overloaded function).
                //! @param f_scalar     Scalar value
                //! @param false_t
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline
                void    setScalarIntern(const value_type& f_scalar, false_t)
                {
                    m_operand2 = f_scalar;
                }

                //---------------------------------------------------------------------
                //! Returns product of m_operand1 with m_operand2.
                //! @param f_row     Number of rows
                //! @param f_col     Number of column
                //! @param true_t
                //! @returns Returns product of m_operand1 with m_operand2.
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline
                value_type  evalIntern (size_type f_row, size_type f_col, true_t) const
                {
                    return m_operand1(f_row,f_col)*m_operand2;
                }

                //---------------------------------------------------------------------
                //! Returns Division of m_operand1 with m_operand2.
                //! @param f_row     Number of rows
                //! @param f_col     Number of column
                //! @param false_t
                //! @returns Returns Division of m_operand1 with m_operand2.
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline
                value_type  evalIntern (size_type f_row, size_type f_col, false_t) const
                {
                    return m_operand1(f_row,f_col)/m_operand2;
                }

                const ExpressionType m_operand1;    //!< first operand
                value_type           m_operand2;    //!< second operand
            };


            //=====================================================================
            // TExp2DMulOpImpl
            //---------------------------------------------------------------------
            //! Forward declaration, implementation present
            //! in dynmic or static. The Shape defines the specialization that
            //! would be used
            //! @param ValueType            Data Type
            //! @param Expression1Type      Data Type
            //! @param Expression2Type      Data Type
            //! @param ShapeType            Data Type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d_common.hpp $
            //=====================================================================
            template<typename ValueType, class Expression1Type, class Expression2Type,
                class ShapeType>
            class TExp2DMulOpImpl;

            //=====================================================================
            // TExp2DMulOp
            //---------------------------------------------------------------------
            //! 2D expression matrix multiplication operator.
            //! @param ValueType            Data Type
            //! @param Expression1Type      Data Type
            //! @param Expression2Type      Data Type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d_common.hpp $
            //=====================================================================
            template <class ValueType, class Expression1Type, class Expression2Type>
            class TExp2DMulOp
            {
            public:
                /// Value Type
                typedef ValueType                                   value_type;
                /// Shape type
                typedef typename TMulShapePromotion <
                    typename Expression1Type::shape_type,
                    typename Expression2Type::shape_type >::type    shape_type;
                /// Size type
                typedef vfc::int32_t                                 size_type;

                //---------------------------------------------------------------------
                //! Saves refernces to both operands, Asserts if operands dimensions does not fit
                //! @param f_operand1_r     2D Expresstion of type Expression1Type
                //! @param f_operand2_r     2D Expresstion of type Expression2Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline TExp2DMulOp(const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r)
                    : m_operand1(f_operand1_r), m_operand2(f_operand2_r)
                {
                    VFC_REQUIRE(f_operand1_r.getNbColumns() == f_operand2_r.getNbRows());
                }

                //---------------------------------------------------------------------
                //! Evaluates multiplication operator func for single element
                //! @param f_row     Number of rows
                //! @param f_col     Number of column
                //! @returns Returns multiplication of m_operand1 with m_operand2.
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval (size_type f_row, size_type f_col) const
                {
                    return TExp2DMulOpImpl<ValueType,
                        Expression1Type, Expression2Type,
                        typename TMulShapePromotion <
                            typename Expression1Type::shape_type,
                            typename Expression2Type::shape_type>::type >::eval(
                                m_operand1, m_operand2, f_row, f_col);
                }

                //---------------------------------------------------------------------
                //! Returns column dimension of resulting 2D expression
                //! @param void
                //! @returns Returns column dimension of resulting 2D expression
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbColumns() const { return m_operand2.getNbColumns();}

                //---------------------------------------------------------------------
                //! Returns row dimension of resulting 2D expression
                //! @param void
                //! @return Returns row dimension of resulting 2D expression
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbRows() const { return m_operand1.getNbRows();}

            private:
                TExp2DMulOp& operator=(const TExp2DMulOp<ValueType, Expression1Type, Expression2Type>&);

                const Expression1Type m_operand1;   //!< first operand
                const Expression2Type m_operand2;   //!< second operand
            };

            //=====================================================================
            // TExp2DSubMatrixOp
            //---------------------------------------------------------------------
            //! Forward declaration of the 2D expression submatrix operator.
            //! Implementation is dynamic/static specific and located in
            //! the corresponding files.
            //! @param ValueType            Data Type
            //! @param Expression1Type      Data Type
            //! @param Expression2Type      Data Type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d_common.hpp $
            //=====================================================================
            template <class ValueType, class ExpressionType, class ShapeType>
            class TExp2DSubMatrixOp;

        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed

#endif //VFC_LINALG_EXP2D_COMMON_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_exp2d_common.hpp  $
//  Revision 1.5 2011/01/21 12:53:28MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - sometimes .hpp or namespace qualifiers are missing (mantis3599)
//  Revision 1.4 2009/03/25 14:48:19MEZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.3 2008/07/31 14:08:20IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.2 2007/07/02 16:25:23IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  - moved the definition of the dynamic TExp2DSubMatrixOp to the dynamic specific file vfc_linalg_exp2d.hpp (mantis 1725)
//  Revision 1.1 2007/05/09 10:21:19CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
