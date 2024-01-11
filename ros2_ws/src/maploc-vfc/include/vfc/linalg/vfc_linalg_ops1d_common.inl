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
///     $Source: vfc_linalg_ops1d_common.inl $
///     $Revision: 1.4 $
///     $Author: Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) $
///     $Date: 2009/05/28 09:19:05MESZ $
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

#include "vfc/core/vfc_assert.hpp"
#include "vfc/core/vfc_math.hpp"
#include "vfc/linalg/vfc_linalg_shapepromotion.hpp"
#include "vfc/linalg/vfc_linalg_vectorbase.hpp"

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

            /// Dot Product operator
            template <class ValueType, class Expression1Type, class Expression2Type>
            class TExp1DDotProductOp
            {
            public:
                /// Constructor
                /// Saves references to the operands
                /// Asserts if the inputs do not have valid dimensions
                inline TExp1DDotProductOp(
                    const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r)
                    : m_operand1(f_operand1_r), m_operand2(f_operand2_r)
                {
                    VFC_REQUIRE(m_operand1.getDim() == m_operand2.getDim());
                }

                /// Returns the dot product of inputs
                inline ValueType eval() const
                {
                    return intern::TExp1DDotProductImpl<ValueType,
                        Expression1Type,
                        Expression2Type,
                        typename TBinaryShapePromotion
                            <typename Expression1Type::shape_type,
                            typename Expression2Type::shape_type>::type>
                        ::eval(m_operand1, m_operand2);
                }

                /// Returns the dimension of the operands
                inline ::vfc::int32_t getDim()
                {
                    return m_operand1.getDim();
                }
            private:

                TExp1DDotProductOp& operator=(
                    const TExp1DDotProductOp<ValueType, Expression1Type, Expression2Type>&);

                /// First Operand
                const Expression1Type m_operand1;

                /// Second Operand
                const Expression2Type m_operand2;
            };

            /// Sum operator
            template <class ValueType, class ExpressionType>
            class TExp1DSumOp
            {
            public:
                /// Constructor
                /// Saves references to the operands
                /// Asserts if the inputs do not have valid dimensions
                inline TExp1DSumOp(
                    const ExpressionType& f_operand_r)
                    : m_operand(f_operand_r)
                {

                }

                /// Returns the sum of input 1d expression
                inline ValueType eval() const
                {
                    //return 0;
                    return intern::TExp1DSumImpl<ValueType,
                        ExpressionType,
                        typename ExpressionType::shape_type>::eval(m_operand);
                }

                /// Returns the dimension of the operand
                inline ::vfc::int32_t getDim()
                {
                    return m_operand.getDim();
                }
            private:

                TExp1DSumOp& operator=(
                    const TExp1DSumOp<ValueType, ExpressionType>&);

                /// First Operand
                const ExpressionType m_operand;

            };


            /// Max operator
            template <class ValueType, class ExpressionType>
            class TExp1DNormLinfOp
            {
            public:
                /// Constructor
                /// Saves references to the operands
                /// Asserts if the inputs do not have valid dimensions
                inline TExp1DNormLinfOp(
                    const ExpressionType& f_operand_r)
                    : m_operand(f_operand_r)
                {

                }

                /// Returns the sum of input 1d expression
                inline ValueType eval() const
                {
                    return intern::TExp1DNormLinfOpImpl<ValueType,
                        ExpressionType,
                        typename ExpressionType::shape_type>::eval(m_operand);
                }

                /// Returns the dimension of the operand
                inline ::vfc::int32_t getDim()
                {
                    return m_operand.getDim();
                }
            private:

                TExp1DNormLinfOp& operator=(
                    const TExp1DNormLinfOp<ValueType, ExpressionType>&);

                /// First Operand
                const ExpressionType m_operand;

            };

        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed

template <class ValueType, class Vector1Type, class Vector2Type,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DAddOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
        vfc::linalg::TVectorConstRef<ValueType, Vector2Type, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator+ (
    const vfc::linalg::TVectorBase<ValueType, Vector1Type, Shape1Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, Vector2Type, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DAddOp<ValueType,
            TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
            TVectorConstRef<ValueType, Vector2Type, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DAddOp<ValueType,
            TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
            TVectorConstRef<ValueType, Vector2Type, Shape2Type> >(
                f_op1_r, f_op2_r));
}

template <class ValueType, class VectorType, class OperatorType,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DAddOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape1Type>,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator+ (
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DAddOp<ValueType,
            TVectorConstRef<ValueType, VectorType, Shape1Type>,
            intern::TExp1D<ValueType, OperatorType, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DAddOp<ValueType,
            TVectorConstRef<ValueType, VectorType, Shape1Type>,
            intern::TExp1D<ValueType, OperatorType, Shape2Type> >(f_op1_r, f_op2_r));
}

template <class ValueType, class VectorType, class OperatorType,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DAddOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape1Type>,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator+ (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape1Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DAddOp<ValueType,
            intern::TExp1D<ValueType, OperatorType, Shape1Type>,
            TVectorConstRef<ValueType, VectorType, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DAddOp<ValueType,
            intern::TExp1D<ValueType, OperatorType, Shape1Type>,
            TVectorConstRef<ValueType, VectorType, Shape2Type> >(
                f_op1_r, f_op2_r));
}

template <class ValueType, class Operator1Type, class Operator2Type,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DAddOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
        vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type>
vfc::linalg::operator+ (
    const vfc::linalg::intern::TExp1D<ValueType, Operator1Type, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DAddOp<ValueType,
            intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
            intern::TExp1D<ValueType, Operator2Type, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DAddOp<ValueType,
            intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
            intern::TExp1D<ValueType, Operator2Type, Shape2Type> >(f_op1_r, f_op2_r));
}
template <class ValueType, class Vector1Type, class Vector2Type,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DSubOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
        vfc::linalg::TVectorConstRef<ValueType, Vector2Type, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator- (
    const vfc::linalg::TVectorBase<ValueType, Vector1Type, Shape1Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, Vector2Type, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DSubOp<ValueType,
            TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
            TVectorConstRef<ValueType, Vector2Type, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DSubOp<ValueType,
            TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
            TVectorConstRef<ValueType, Vector2Type, Shape2Type> >(
                f_op1_r, f_op2_r));
}

template <class ValueType, class VectorType, class OperatorType,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DSubOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape1Type>,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator- (
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DSubOp<ValueType,
            TVectorConstRef<ValueType, VectorType, Shape1Type>,
            intern::TExp1D<ValueType, OperatorType, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DSubOp<ValueType,
            TVectorConstRef<ValueType, VectorType, Shape1Type>,
            intern::TExp1D<ValueType, OperatorType, Shape2Type> >(f_op1_r, f_op2_r));
}

template <class ValueType, class VectorType, class OperatorType,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DSubOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape1Type>,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator- (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape1Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DSubOp<ValueType,
            intern::TExp1D<ValueType, OperatorType, Shape1Type>,
            TVectorConstRef<ValueType, VectorType, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DSubOp<ValueType,
            intern::TExp1D<ValueType, OperatorType, Shape1Type>,
            TVectorConstRef<ValueType, VectorType, Shape2Type> >(
                f_op1_r, f_op2_r));
}

template <class ValueType, class Operator1Type, class Operator2Type,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DSubOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
        vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type>
vfc::linalg::operator- (
    const vfc::linalg::intern::TExp1D<ValueType, Operator1Type, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DSubOp<ValueType,
            intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
            intern::TExp1D<ValueType, Operator2Type, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DSubOp<ValueType,
            intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
            intern::TExp1D<ValueType, Operator2Type, Shape2Type> >(f_op1_r, f_op2_r));
}

template <class ValueType, class VectorType, class ShapeType>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DNegateOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, ShapeType> >,
    typename VectorType::shape_type>
vfc::linalg::operator- (
    const vfc::linalg::TVectorBase<ValueType, VectorType, ShapeType>& f_operand_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DNegateOp<ValueType,
            TVectorConstRef<ValueType, VectorType, ShapeType> > ,
        typename VectorType::shape_type>(
            intern::TExp1DNegateOp<ValueType,
                TVectorConstRef<ValueType, VectorType, ShapeType> >(f_operand_r));
}

template <class ValueType, class OperatorType, class ShapeType>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DNegateOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType> >,
    ShapeType>
vfc::linalg::operator- (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_operand_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DNegateOp<ValueType,
            intern::TExp1D<ValueType, OperatorType, ShapeType> >,
        ShapeType>(
            intern::TExp1DNegateOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, ShapeType> >(f_operand_r));
}

template <class ValueType, class VectorType, class ShapeType>
inline
const
vfc::linalg::intern::TExp1D <ValueType,
    vfc::linalg::intern::TExp1DScalarOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, ShapeType>,
        stlalias::multiplies<ValueType> > , 
    typename VectorType::shape_type>
vfc::linalg::operator* (
    const vfc::linalg::TVectorBase<ValueType, VectorType,ShapeType>& f_operand1_r,
    const ValueType& f_operand2_r)
{
    return intern::TExp1D <ValueType,
    intern::TExp1DScalarOp<ValueType,
        TVectorConstRef<ValueType, VectorType, ShapeType>,
        stlalias::multiplies<ValueType> > , 
    typename VectorType::shape_type>(
        intern::TExp1DScalarOp<ValueType,
        TVectorConstRef<ValueType, VectorType, ShapeType>,
        stlalias::multiplies<ValueType> >(f_operand1_r, f_operand2_r));

}

template <class ValueType, class VectorType, class ShapeType>
inline
const
vfc::linalg::intern::TExp1D <ValueType,
    vfc::linalg::intern::TExp1DScalarOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, ShapeType>,
        stlalias::multiplies<ValueType> > , 
    typename VectorType::shape_type>
vfc::linalg::operator* (
    const ValueType& f_operand1_r,
    const vfc::linalg::TVectorBase<ValueType, VectorType,ShapeType>& f_operand2_r)
{
    return intern::TExp1D <ValueType,
    intern::TExp1DScalarOp<ValueType,
        TVectorConstRef<ValueType, VectorType, ShapeType>,
        stlalias::multiplies<ValueType> > , 
    typename VectorType::shape_type>(
        intern::TExp1DScalarOp<ValueType,
        TVectorConstRef<ValueType, VectorType, ShapeType>,
        stlalias::multiplies<ValueType> >(f_operand2_r, f_operand1_r));

}

template <class ValueType, class OperatorType, class ShapeType>
inline
const
vfc::linalg::intern::TExp1D <ValueType,
    vfc::linalg::intern::TExp1DScalarOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>,
        stlalias::multiplies<ValueType> > , 
    ShapeType>
vfc::linalg::operator* (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_operand1_r,
    const ValueType& f_operand2_r)
{
    return intern::TExp1D <ValueType,
    intern::TExp1DScalarOp<ValueType,
        intern::TExp1D<ValueType, OperatorType, ShapeType>,
        stlalias::multiplies<ValueType> > , 
    ShapeType>(
        intern::TExp1DScalarOp<ValueType,
        intern::TExp1D<ValueType, OperatorType, ShapeType>,
        stlalias::multiplies<ValueType> >(f_operand1_r, f_operand2_r));

}

template <class ValueType, class OperatorType, class ShapeType>
inline
const
vfc::linalg::intern::TExp1D <ValueType,
    vfc::linalg::intern::TExp1DScalarOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>,
        stlalias::multiplies<ValueType> > , 
    ShapeType>
vfc::linalg::operator* (
    const ValueType& f_operand1_r,
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_operand2_r)
{
    return intern::TExp1D <ValueType,
    intern::TExp1DScalarOp<ValueType,
        intern::TExp1D<ValueType, OperatorType, ShapeType>,
        stlalias::multiplies<ValueType> > , 
    ShapeType>(
        intern::TExp1DScalarOp<ValueType,
        intern::TExp1D<ValueType, OperatorType, ShapeType>,
        stlalias::multiplies<ValueType> >(f_operand2_r, f_operand1_r));

}

template <class ValueType, class VectorType, class ShapeType>
inline
const
vfc::linalg::intern::TExp1D <ValueType,
    vfc::linalg::intern::TExp1DScalarOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, ShapeType>,
        stlalias::divides<ValueType> > , 
    typename VectorType::shape_type>
vfc::linalg::operator/ (
    const vfc::linalg::TVectorBase<ValueType, VectorType,ShapeType>& f_operand1_r,
    const ValueType& f_operand2_r)
{
    VFC_REQUIRE( !vfc::isZero(f_operand2_r) );
    return intern::TExp1D <ValueType,
    intern::TExp1DScalarOp<ValueType,
        TVectorConstRef<ValueType, VectorType, ShapeType>,
        stlalias::divides<ValueType> > , 
    typename VectorType::shape_type>(
        intern::TExp1DScalarOp<ValueType,
        TVectorConstRef<ValueType, VectorType, ShapeType>,
        stlalias::divides<ValueType> >(f_operand1_r, f_operand2_r));

}

template <class ValueType, class OperatorType, class ShapeType>
inline
const
vfc::linalg::intern::TExp1D <ValueType,
    vfc::linalg::intern::TExp1DScalarOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>,
        stlalias::divides<ValueType> > , 
    ShapeType>
vfc::linalg::operator/ (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_operand1_r,
    const ValueType& f_operand2_r)
{
    VFC_REQUIRE( !vfc::isZero(f_operand2_r) );
    return intern::TExp1D <ValueType,
    intern::TExp1DScalarOp<ValueType,
        intern::TExp1D<ValueType, OperatorType, ShapeType>,
        stlalias::divides<ValueType> > , 
    ShapeType>(
        intern::TExp1DScalarOp<ValueType,
        intern::TExp1D<ValueType, OperatorType, ShapeType>,
        stlalias::divides<ValueType> >(f_operand1_r, f_operand2_r));

}

template <class ValueType, class Vector1Type, class Vector2Type,
    class Shape1Type, class Shape2Type>
inline
ValueType
vfc::linalg::dot (
    const vfc::linalg::TVectorBase<ValueType, Vector1Type, Shape1Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, Vector2Type, Shape2Type>& f_op2_r)
{
    return  intern::TExp1DDotProductOp<ValueType,
            TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
            TVectorConstRef<ValueType, Vector2Type, Shape2Type> >(
            f_op1_r, f_op2_r).eval();
}

template <class ValueType, class VectorType, class OperatorType,
    class Shape1Type, class Shape2Type>
inline
ValueType
vfc::linalg::dot (
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r)
{
    return  intern::TExp1DDotProductOp<ValueType,
            TVectorConstRef<ValueType, VectorType, Shape1Type>,
            intern::TExp1D<ValueType, OperatorType, Shape2Type> >(f_op1_r, f_op2_r).eval();
}

template <class ValueType, class VectorType, class OperatorType,
    class Shape1Type, class Shape2Type>
inline
ValueType
vfc::linalg::dot (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape1Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r)
{
    return  intern::TExp1DDotProductOp<ValueType,
            intern::TExp1D<ValueType, OperatorType, Shape1Type>,
            TVectorConstRef<ValueType, VectorType, Shape2Type> >(
            f_op1_r, f_op2_r).eval();
}

template <class ValueType, class Operator1Type, class Operator2Type,
    class Shape1Type, class Shape2Type>
inline
ValueType
vfc::linalg::dot (
    const vfc::linalg::intern::TExp1D<ValueType, Operator1Type, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r)
{
    return  intern::TExp1DDotProductOp<ValueType,
            intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
            intern::TExp1D<ValueType, Operator2Type, Shape2Type> >(f_op1_r, f_op2_r).eval();
}

template <class ValueType, class VectorType, class ShapeType>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DAbsOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, ShapeType> >,
    typename VectorType::shape_type>
vfc::linalg::abs (
    const vfc::linalg::TVectorBase<ValueType, VectorType, ShapeType>& f_operand_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DAbsOp<ValueType,
            TVectorConstRef<ValueType, VectorType, ShapeType> > ,
        typename VectorType::shape_type>(
            intern::TExp1DAbsOp<ValueType,
                TVectorConstRef<ValueType, VectorType, ShapeType> >(f_operand_r));
}

template <class ValueType, class OperatorType, class ShapeType>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DAbsOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType> >,
    ShapeType>
vfc::linalg::abs (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_operand_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DAbsOp<ValueType,
            intern::TExp1D<ValueType, OperatorType, ShapeType> >,
        ShapeType>(
            intern::TExp1DAbsOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, ShapeType> >(f_operand_r));
}

template <class ValueType, class Vector1Type, class Vector2Type,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DMinOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
        vfc::linalg::TVectorConstRef<ValueType, Vector2Type, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
(vfc::linalg::min) (
    const vfc::linalg::TVectorBase<ValueType, Vector1Type, Shape1Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, Vector2Type, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DMinOp<ValueType,
            TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
            TVectorConstRef<ValueType, Vector2Type, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DMinOp<ValueType,
            TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
            TVectorConstRef<ValueType, Vector2Type, Shape2Type> >(
                f_op1_r, f_op2_r));
}

template <class ValueType, class VectorType, class OperatorType,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DMinOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape1Type>,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
(vfc::linalg::min) (
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DMinOp<ValueType,
            TVectorConstRef<ValueType, VectorType, Shape1Type>,
            intern::TExp1D<ValueType, OperatorType, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DMinOp<ValueType,
            TVectorConstRef<ValueType, VectorType, Shape1Type>,
            intern::TExp1D<ValueType, OperatorType, Shape2Type> >(f_op1_r, f_op2_r));
}

template <class ValueType, class VectorType, class OperatorType,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DMinOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape1Type>,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
(vfc::linalg::min) (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape1Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DMinOp<ValueType,
            intern::TExp1D<ValueType, OperatorType, Shape1Type>,
            TVectorConstRef<ValueType, VectorType, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DMinOp<ValueType,
            intern::TExp1D<ValueType, OperatorType, Shape1Type>,
            TVectorConstRef<ValueType, VectorType, Shape2Type> >(
                f_op1_r, f_op2_r));
}

template <class ValueType, class Operator1Type, class Operator2Type,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DMinOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
        vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type>
(vfc::linalg::min) (
    const vfc::linalg::intern::TExp1D<ValueType, Operator1Type, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DMinOp<ValueType,
            intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
            intern::TExp1D<ValueType, Operator2Type, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DMinOp<ValueType,
            intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
            intern::TExp1D<ValueType, Operator2Type, Shape2Type> >(f_op1_r, f_op2_r));
}


template <class ValueType, class Vector1Type, class Vector2Type,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DMaxOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
        vfc::linalg::TVectorConstRef<ValueType, Vector2Type, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
(vfc::linalg::max) (
    const vfc::linalg::TVectorBase<ValueType, Vector1Type, Shape1Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, Vector2Type, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DMaxOp<ValueType,
            TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
            TVectorConstRef<ValueType, Vector2Type, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DMaxOp<ValueType,
            TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
            TVectorConstRef<ValueType, Vector2Type, Shape2Type> >(
                f_op1_r, f_op2_r));
}

template <class ValueType, class VectorType, class OperatorType,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DMaxOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape1Type>,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
(vfc::linalg::max) (
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DMaxOp<ValueType,
            TVectorConstRef<ValueType, VectorType, Shape1Type>,
            intern::TExp1D<ValueType, OperatorType, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DMaxOp<ValueType,
            TVectorConstRef<ValueType, VectorType, Shape1Type>,
            intern::TExp1D<ValueType, OperatorType, Shape2Type> >(f_op1_r, f_op2_r));
}

template <class ValueType, class VectorType, class OperatorType,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DMaxOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape1Type>,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
(vfc::linalg::max) (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape1Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DMaxOp<ValueType,
            intern::TExp1D<ValueType, OperatorType, Shape1Type>,
            TVectorConstRef<ValueType, VectorType, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DMaxOp<ValueType,
            intern::TExp1D<ValueType, OperatorType, Shape1Type>,
            TVectorConstRef<ValueType, VectorType, Shape2Type> >(
                f_op1_r, f_op2_r));
}

template <class ValueType, class Operator1Type, class Operator2Type,
    class Shape1Type, class Shape2Type>
inline
const
vfc::linalg::intern::TExp1D<ValueType,
    vfc::linalg::intern::TExp1DMaxOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
        vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type> >,
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type>
(vfc::linalg::max) (
    const vfc::linalg::intern::TExp1D<ValueType, Operator1Type, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r)
{
    return intern::TExp1D<ValueType,
        intern::TExp1DMaxOp<ValueType,
            intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
            intern::TExp1D<ValueType, Operator2Type, Shape2Type> >,
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp1DMaxOp<ValueType,
            intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
            intern::TExp1D<ValueType, Operator2Type, Shape2Type> >(f_op1_r, f_op2_r));
}


template <class ValueType, class VectorType, class ShapeType>
inline
ValueType
vfc::linalg::sum (
    const vfc::linalg::TVectorBase<ValueType, VectorType, ShapeType>& f_op_r)
{
    return intern::TExp1DSumOp<ValueType,
                TVectorConstRef<ValueType, VectorType, ShapeType> >(f_op_r).eval();
}

template <class ValueType, class OperatorType, class ShapeType>
inline
ValueType
vfc::linalg::sum (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_op_r)
{
    return intern::TExp1DSumOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, ShapeType> >(f_op_r).eval();
}

template <class ValueType, class VectorType, class ShapeType>
inline
ValueType
vfc::linalg::norm_L1 (
    const vfc::linalg::TVectorBase<ValueType, VectorType, ShapeType>& f_op_r)
{
    return sum(abs(f_op_r));
}

template <class ValueType, class OperatorType, class ShapeType>
inline
ValueType
vfc::linalg::norm_L1 (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_op_r)
{
    return sum(abs(f_op_r));
}

template <class ValueType, class VectorType, class ShapeType>
inline
ValueType
vfc::linalg::norm_L2 (
    const vfc::linalg::TVectorBase<ValueType, VectorType, ShapeType>& f_op_r)
{
    return static_cast<ValueType>(vfc::sqrt(dot(f_op_r, f_op_r)));
}

template <class ValueType, class OperatorType, class ShapeType>
inline
ValueType
vfc::linalg::norm_L2 (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_op_r)
{
    return static_cast<ValueType>(vfc::sqrt(dot(f_op_r, f_op_r)));
}

template <class ValueType, class VectorType, class ShapeType>
inline
ValueType
vfc::linalg::norm_Linf (
    const vfc::linalg::TVectorBase<ValueType, VectorType, ShapeType>& f_op_r)
{
    return intern::TExp1DNormLinfOp<ValueType,
                TVectorConstRef<ValueType, VectorType, ShapeType> >(f_op_r).eval();
}

template <class ValueType, class OperatorType, class ShapeType>
inline
ValueType
vfc::linalg::norm_Linf (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_op_r)
{
    return intern::TExp1DNormLinfOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, ShapeType> >(f_op_r).eval();
}

template <class ValueType, class VectorType, class ShapeType>
inline
ValueType
vfc::linalg::length (
    const vfc::linalg::TVectorBase<ValueType, VectorType, ShapeType>& f_op_r)
{
    return norm_L2(f_op_r);
}

template <class ValueType, class OperatorType, class ShapeType>
inline
ValueType
vfc::linalg::length (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_op_r)
{
    return norm_L2(f_op_r);
}


template <class ValueType, class VectorType, class ShapeType>
inline
const
vfc::linalg::intern::TExp1D <ValueType,
    vfc::linalg::intern::TExp1DScalarOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, ShapeType>,
        stlalias::divides<ValueType> > , 
    typename VectorType::shape_type>
vfc::linalg::normalized (
    const vfc::linalg::TVectorBase<ValueType, VectorType,ShapeType>& f_operand_r)
{
    const ValueType len = length(f_operand_r);
    VFC_REQUIRE( !vfc::isZero(len) );
    return intern::TExp1D <ValueType,
    intern::TExp1DScalarOp<ValueType,
        TVectorConstRef<ValueType, VectorType, ShapeType>,
        stlalias::divides<ValueType> > , 
    typename VectorType::shape_type>(
        intern::TExp1DScalarOp<ValueType,
        TVectorConstRef<ValueType, VectorType, ShapeType>,
        stlalias::divides<ValueType> >(f_operand_r, len));
}

template <class ValueType, class OperatorType, class ShapeType>
inline
const
vfc::linalg::intern::TExp1D <ValueType,
    vfc::linalg::intern::TExp1DScalarOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>,
        stlalias::divides<ValueType> > , 
    ShapeType>
vfc::linalg::normalized (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_operand_r)
{
    const ValueType len = length(f_operand_r);
    VFC_REQUIRE( !vfc::isZero(len) );
    return intern::TExp1D <ValueType,
    intern::TExp1DScalarOp<ValueType,
        intern::TExp1D<ValueType, OperatorType, ShapeType>,
        stlalias::divides<ValueType> > , 
    ShapeType>(
        intern::TExp1DScalarOp<ValueType,
        intern::TExp1D<ValueType, OperatorType, ShapeType>,
        stlalias::divides<ValueType> >(f_operand_r, len));
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_ops1d_common.inl  $
//  Revision 1.4 2009/05/28 09:19:05MESZ Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) 
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.3 2009/03/25 14:48:24CET Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.2 2008/07/10 17:37:26IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  - fixed min/max errors due to macro definitions of min & max (mantis2168)
//  Revision 1.1 2007/05/09 13:51:24IST Jaeger Thomas (CC-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
