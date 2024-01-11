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
///     $Source: vfc_linalg_ops1d_common.hpp $
///     $Revision: 1.7 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/03/11 16:41:09MEZ $
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

#ifndef VFC_LINALG_OPS1D_COMMON_HPP_INCLUDED
#define VFC_LINALG_OPS1D_COMMON_HPP_INCLUDED

#include <functional>

#include "vfc/linalg/vfc_linalg_exp1d_common.hpp"

// forward declarations
namespace vfc
{
    namespace linalg
    {
        template<class ValueType, class DerivedType, class ShapeType>
        class TVectorBase;

        template <class ValueType, class DerivedType, class ShapeType>
        class TVectorConstRef;

    }
}

namespace vfc
{// namespace vfc opened

    namespace linalg
    {// namespace linalg opened

        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg BEGIN
        //-------------------------------------------------------------------------
        /// @addtogroup vfc_group_linalg
        /// @{
        //-------------------------------------------------------------------------

        //---------------------------------------------------------------------
        //!  Operator overloading for Vector & Vector addition
        //! @param f_op1_r     Object of TVectorBase
        //! @param f_op2_r     Object of TVectorBase
        //! @return  Returns Vector & Vector addition
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Vector1Type, class Vector2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DAddOp<ValueType,
                TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
                TVectorConstRef<ValueType, Vector2Type, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator+ (const TVectorBase<ValueType, Vector1Type, Shape1Type>& f_op1_r,
            const TVectorBase<ValueType, Vector2Type, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for Vector & 1D Expression addition
        //! @param f_op1_r     Object of TVectorBase
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns Vector & 1D Expression addition
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DAddOp<ValueType,
                TVectorConstRef<ValueType, VectorType, Shape1Type>,
                intern::TExp1D<ValueType, OperatorType, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator+ (const TVectorBase<ValueType, VectorType, Shape1Type>& f_op1_r,
            const intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for 1D Expression & Vector addition
        //! @param f_op1_r     Object of TExp1D
        //! @param f_op2_r     Object of TVectorBase
        //! @return  Returns 1D Expression & Vector addition
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DAddOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, Shape1Type>,
                TVectorConstRef<ValueType, VectorType, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator+ (const intern::TExp1D<ValueType, OperatorType, Shape1Type>& f_op1_r,
            const TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for 1D Expression & 1D Expression addition
        //! @param f_op1_r     Object of TExp1D
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns 1D Expression & 1D Expression addition
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Operator1Type, class Operator2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DAddOp<ValueType,
                intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
                intern::TExp1D<ValueType, Operator2Type, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator+ (const intern::TExp1D<ValueType, Operator1Type, Shape1Type>& f_op1_r,
            const intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for Vector & Vector subraction
        //! @param f_op1_r     Object of TVectorBase
        //! @param f_op2_r     Object of TVectorBase
        //! @return  Returns Vector & Vector subraction
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Vector1Type, class Vector2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DSubOp<ValueType,
                TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
                TVectorConstRef<ValueType, Vector2Type, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator- (const TVectorBase<ValueType, Vector1Type, Shape1Type>& f_op1_r,
            const TVectorBase<ValueType, Vector2Type, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for Vector & 1D Expression subraction
        //! @param f_op1_r     Object of TVectorBase
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns Vector & 1D Expression subraction
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class OperatorType, class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DSubOp<ValueType,
                TVectorConstRef<ValueType, VectorType, Shape1Type>,
                intern::TExp1D<ValueType, OperatorType, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator- (const TVectorBase<ValueType, VectorType, Shape1Type>& f_op1_r,
            const intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for 1D Expression & Vector subraction
        //! @param f_op1_r     Object of TExp1D
        //! @param f_op2_r     Object of TVectorBase
        //! @return  Returns 1D Expression & Vector subraction
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DSubOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, Shape1Type>,
                TVectorConstRef<ValueType, VectorType, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator- (const intern::TExp1D<ValueType, OperatorType, Shape1Type>& f_op1_r,
            const TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for 1D Expression & 1D Expression subraction
        //! @param f_op1_r     Object of TExp1D
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns 1D Expression & 1D Expression subraction
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Operator1Type, class Operator2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DSubOp<ValueType,
                intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
                intern::TExp1D<ValueType, Operator2Type, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator- (const intern::TExp1D<ValueType, Operator1Type, Shape1Type>& f_op1_r,
            const intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for Vector Negation
        //! @param f_operand_r     Object of TVectorBase
        //! @return  Returns Vector Negation
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class ShapeType>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DNegateOp<ValueType,
                TVectorConstRef<ValueType, VectorType, ShapeType> >,
            typename VectorType::shape_type>
        operator- (const TVectorBase<ValueType, VectorType, ShapeType>& f_operand_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for 1D Expression Negation
        //! @param f_operand_r     Object of TExp1D
        //! @return  Returns 1D Expression Negation
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DNegateOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, ShapeType> >,
            ShapeType>
        operator- (const intern::TExp1D<ValueType, OperatorType, ShapeType>& f_operand_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for Vector & Value Multiplication
        //! @param f_operand1_r     Object of TVectorBase
        //! @param f_operand2_r     Scalar value
        //! @return  Returns Vector & Value Multiplication
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class ShapeType>
        const
        intern::TExp1D <ValueType,
            intern::TExp1DScalarOp<ValueType,
                TVectorConstRef<ValueType, VectorType, ShapeType>,
                stlalias::multiplies<ValueType> > ,
            typename VectorType::shape_type>
        operator* (const TVectorBase<ValueType, VectorType,ShapeType>& f_operand1_r,
            const ValueType& f_operand2_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for Value & Vector Multiplication
        //! @param f_operand1_r     Scalar value
        //! @param f_operand2_r     Object of TVectorBase
        //! @return  Returns Value & Vector Multiplication
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class ShapeType>
        const
        intern::TExp1D <ValueType,
            intern::TExp1DScalarOp<ValueType,
                TVectorConstRef<ValueType, VectorType, ShapeType>,
                stlalias::multiplies<ValueType> > ,
            typename VectorType::shape_type>
        operator* (const ValueType& f_operand1_r,
            const TVectorBase<ValueType, VectorType,ShapeType>& f_operand2_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for 1D Expression & Value Multiplication
        //! @param f_operand1_r     Object of TExp1D
        //! @param f_operand2_r     Scalar value
        //! @return  Returns 1D Expression & Value Multiplication
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        const
        intern::TExp1D <ValueType,
            intern::TExp1DScalarOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, ShapeType>,
                stlalias::multiplies<ValueType> > ,
            ShapeType>
        operator* (const intern::TExp1D<ValueType, OperatorType, ShapeType>& f_operand1_r,
            const ValueType& f_operand2_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for Value & 1D Expression Multiplication
        //! @param f_operand1_r     Scalar value
        //! @param f_operand2_r     Object of TExp1D
        //! @return  Returns Value & 1D Expression Multiplication
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        const
        intern::TExp1D <ValueType,
            intern::TExp1DScalarOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, ShapeType>,
                stlalias::multiplies<ValueType> > ,
            ShapeType>
        operator* (const ValueType& f_operand1_r,
            const intern::TExp1D<ValueType, OperatorType, ShapeType>& f_operand2_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for Vector & Value Division
        //! @param f_operand1_r     Object of TVectorBase
        //! @param f_operand2_r     Scalar value
        //! @return  Returns Vector & Value Division
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class ShapeType>
        const
        intern::TExp1D <ValueType,
            intern::TExp1DScalarOp<ValueType,
                TVectorConstRef<ValueType, VectorType, ShapeType>,
                stlalias::divides<ValueType> > ,
            typename VectorType::shape_type>
        operator/ (
            const TVectorBase<ValueType, VectorType,ShapeType>& f_operand1_r,
            const ValueType& f_operand2_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for 1D Expression & Value Division
        //! @param f_operand1_r     Object of TExp1D
        //! @param f_operand2_r     Scalar value
        //! @return  Returns 1D Expression & Value Division
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        const
        intern::TExp1D <ValueType,
            intern::TExp1DScalarOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, ShapeType>,
                stlalias::divides<ValueType> > ,
            ShapeType>
        operator/ (
            const intern::TExp1D<ValueType, OperatorType, ShapeType>& f_operand1_r,
            const ValueType& f_operand2_r);

        //---------------------------------------------------------------------
        //!  dot function overload:
        //!        Calculates dot product of two vectors and returns a single value of
        //!        type ValueType
        //! @param f_op1_r     Object of TVectorBase
        //! @param f_op2_r     Object of TVectorBase
        //! @return  Returns dot product of two vectors of type ValueType
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Vector1Type, class Vector2Type,
            class Shape1Type, class Shape2Type>
        ValueType
        dot (
            const TVectorBase<ValueType, Vector1Type, Shape1Type>& f_op1_r,
            const TVectorBase<ValueType, Vector2Type, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  dot function overload:
        //!        Calculates dot product of vector and 1D expression
        //!        and returns a single value of type ValueType
        //! @param f_op1_r     Object of TVectorBase
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns dot product of vector and 1D expression of type ValueType
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
        ValueType
        dot (
            const TVectorBase<ValueType, VectorType, Shape1Type>& f_op1_r,
            const intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  dot function overload:
        //!        Calculates dot product of 1D expression and vector
        //!        and returns a single value of type ValueType
        //! @param f_op1_r     Object of TVectorBase
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns dot product of 1D expression and vector of type ValueType
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
        ValueType
        dot (
            const intern::TExp1D<ValueType, OperatorType, Shape1Type>& f_op1_r,
            const TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  dot function overload:
        //!        Calculates dot product of 1D expression and 1D expression
        //!        and returns a single value of type ValueType
        //! @param f_op1_r     Object of TExp1D
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns dot product of 1D expression and 1D expression of type ValueType
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Operator1Type, class Operator2Type,
            class Shape1Type, class Shape2Type>
        ValueType
        dot (
            const intern::TExp1D<ValueType, Operator1Type, Shape1Type>& f_op1_r,
            const intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  abs function overload for vectors.
        //! @param f_operand_r     Object of TVectorBase
        //! @return  Returns abs of vectors.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class ShapeType>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DAbsOp<ValueType,
                TVectorConstRef<ValueType, VectorType, ShapeType> >,
            typename VectorType::shape_type>
        abs (const TVectorBase<ValueType, VectorType, ShapeType>& f_operand_r);

        //---------------------------------------------------------------------
        //!  abs function overload for vectors 1D Expressions
        //! @param f_operand_r     Object of TExp1D
        //! @return  Returns abs of vector and 1D Expressions.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DAbsOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, ShapeType> >,
            ShapeType>
        abs (const intern::TExp1D<ValueType, OperatorType, ShapeType>& f_operand_r);

        //---------------------------------------------------------------------
        //!  Min operator overloading for Vector & Vector operations
        //! @param f_op1_r     Object of TVectorBase
        //! @param f_op2_r     Object of TVectorBase
        //! @return  Returns Min of vectors.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Vector1Type, class Vector2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DMinOp<ValueType,
                TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
                TVectorConstRef<ValueType, Vector2Type, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        min (const TVectorBase<ValueType, Vector1Type, Shape1Type>& f_op1_r,
            const TVectorBase<ValueType, Vector2Type, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Min operator overloading for Vector & 1D expression operations
        //! @param f_op1_r     Object of TVectorBase
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns Min of Vector & 1D expression operations.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DMinOp<ValueType,
                TVectorConstRef<ValueType, VectorType, Shape1Type>,
                intern::TExp1D<ValueType, OperatorType, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        min (const TVectorBase<ValueType, VectorType, Shape1Type>& f_op1_r,
            const intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Min operator overloading for 1D expression & Vector operations
        //! @param f_op1_r     Object of TExp1D
        //! @param f_op2_r     Object of TVectorBase
        //! @return  Returns Min of 1D expression & Vector operations.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DMinOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, Shape1Type>,
                TVectorConstRef<ValueType, VectorType, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        min (const intern::TExp1D<ValueType, OperatorType, Shape1Type>& f_op1_r,
            const TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Min operator overloading for 1D expression & 1D expression operations
        //! @param f_op1_r     Object of TExp1D
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns Min of 1D expression & 1D expression operations.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Operator1Type, class Operator2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DMinOp<ValueType,
                intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
                intern::TExp1D<ValueType, Operator2Type, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        min (const intern::TExp1D<ValueType, Operator1Type, Shape1Type>& f_op1_r,
            const intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r);


        //---------------------------------------------------------------------
        //!  Max operator overloading for Vector & Vector operations
        //! @param f_op1_r     Object of TVectorBase
        //! @param f_op2_r     Object of TVectorBase
        //! @return  Returns Max of Vector & Vector operations.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Vector1Type, class Vector2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DMaxOp<ValueType,
                TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
                TVectorConstRef<ValueType, Vector2Type, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        max (const TVectorBase<ValueType, Vector1Type, Shape1Type>& f_op1_r,
            const TVectorBase<ValueType, Vector2Type, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Max operator overloading for Vector & 1D expression operations
        //! @param f_op1_r     Object of TVectorBase
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns Max of Vector & 1D expression operations.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DMaxOp<ValueType,
                TVectorConstRef<ValueType, VectorType, Shape1Type>,
                intern::TExp1D<ValueType, OperatorType, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        max (const TVectorBase<ValueType, VectorType, Shape1Type>& f_op1_r,
            const intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Max operator overloading for 1D expression & Vector operations
        //! @param f_op1_r     Object of TExp1D
        //! @param f_op2_r     Object of TVectorBase
        //! @return  Returns Max of 1D expression & Vector operations.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DMaxOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, Shape1Type>,
                TVectorConstRef<ValueType, VectorType, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        max (const intern::TExp1D<ValueType, OperatorType, Shape1Type>& f_op1_r,
            const TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Max operator overloading for 1D expression & 1D expression operations
        //! @param f_op1_r     Object of TExp1D
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns Max of 1D expression & 1D expression operations.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Operator1Type, class Operator2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DMaxOp<ValueType,
                intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
                intern::TExp1D<ValueType, Operator2Type, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        max (const intern::TExp1D<ValueType, Operator1Type, Shape1Type>& f_op1_r,
            const intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //!  Sum function overload
        //! @param f_op_r     Object of TVectorBase
        //! @return  Returns Sum of vectors.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class ShapeType>
        ValueType
        sum (const TVectorBase<ValueType, VectorType, ShapeType>& f_op_r);

        //---------------------------------------------------------------------
        //!  Sum function overload
        //! @param f_op_r     Object of TExp1D
        //! @return  Returns Sum of 1D expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        ValueType
        sum (const intern::TExp1D<ValueType, OperatorType, ShapeType>& f_op_r);

        //---------------------------------------------------------------------
        //!  norm_L1 function overload
        //! @param f_op_r     Object of TVectorBase
        //! @return  Returns normalized value of vectors.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class ShapeType>
        ValueType
        norm_L1 (const TVectorBase<ValueType, VectorType, ShapeType>& f_op_r);

        //---------------------------------------------------------------------
        //!  norm_L1 function overload
        //! @param f_op_r     Object of TExp1D
        //! @return  Returns normalized value of 1D expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        ValueType
        norm_L1 (const intern::TExp1D<ValueType, OperatorType, ShapeType>& f_op_r);

        //---------------------------------------------------------------------
        //!  norm_L2 function overload
        //! @param f_op_r     Object of TVectorBase
        //! @return  Returns normalized value of vectors.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class ShapeType>
        ValueType
        norm_L2 (const TVectorBase<ValueType, VectorType, ShapeType>& f_op_r);

        //---------------------------------------------------------------------
        //!  norm_L2 function overload
        //! @param f_op_r     Object of TExp1D
        //! @return  Returns normalized value of 1D expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        ValueType
        norm_L2 (const intern::TExp1D<ValueType, OperatorType, ShapeType>& f_op_r);

        //---------------------------------------------------------------------
        //!  norm_Linf function overload
        //! @param f_op_r     Object of TVectorBase
        //! @return  Returns norm_Linf of vectors.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class ShapeType>
        ValueType
        norm_Linf (const TVectorBase<ValueType, VectorType, ShapeType>& f_op_r);

        //---------------------------------------------------------------------
        //!  norm_Linf function overload
        //! @param f_op_r     Object of TExp1D
        //! @return  Returns norm_Linf of 1D expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        ValueType
        norm_Linf (const intern::TExp1D<ValueType, OperatorType, ShapeType>& f_op_r);


        //---------------------------------------------------------------------
        //!  Length function overload
        //! @param f_op_r     Object of TVectorBase
        //! @return  Returns Length of vectors.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class ShapeType>
        ValueType
        length (const TVectorBase<ValueType, VectorType, ShapeType>& f_op_r);

        //---------------------------------------------------------------------
        //!  Length function overload
        //! @param f_op_r     Object of TExp1D
        //! @return  Returns Length of 1D expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        ValueType
        length (const intern::TExp1D<ValueType, OperatorType, ShapeType>& f_op_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for vector normalization.
        //!        Returns the normalized vector, input vector is unchanged.
        //! @param f_operand_r     Object of TVectorBase
        //! @return  Returns normalized of vectors.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class ShapeType>
        const
        intern::TExp1D <ValueType,
            intern::TExp1DScalarOp<ValueType,
                TVectorConstRef<ValueType, VectorType, ShapeType>,
                stlalias::divides<ValueType> > ,
            typename VectorType::shape_type>
        normalized (
            const TVectorBase<ValueType, VectorType,ShapeType>& f_operand_r);

        //---------------------------------------------------------------------
        //!  Operator overloading for 1D expression normalization.
        //!        Returns the normalized vector, input expression is unchanged.
        //! @param f_operand_r     Object of TExp1D
        //! @return  Returns normalized of 1D expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        const
        intern::TExp1D <ValueType,
            intern::TExp1DScalarOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, ShapeType>,
                stlalias::divides<ValueType> > ,
            ShapeType>
        normalized (
            const intern::TExp1D<ValueType, OperatorType, ShapeType>& f_operand_r);

        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg END
        //-------------------------------------------------------------------------
        /// @}
        //-------------------------------------------------------------------------

    }   // namespace linalg closed


}   // namespace vfc closed

#include "vfc/linalg/vfc_linalg_ops1d_common.inl"

#endif //VFC_LINALG_OPS1D_COMMON_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_ops1d_common.hpp  $
//  Revision 1.7 2014/03/11 16:41:09MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - The linalg _ops headers lack the <functional> include (mantis0004401)
//  Revision 1.6 2011/01/21 12:53:35MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - sometimes .hpp or namespace qualifiers are missing (mantis3599)
//  Revision 1.5 2009/05/28 09:19:06MESZ Muehlmann Karsten (CC/ESV2) (MUK2LR) 
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.4 2008/08/29 15:04:55CEST Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.3 2008/07/31 14:08:36IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:24IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
