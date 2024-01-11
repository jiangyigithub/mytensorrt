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
///     $Source: vfc_linalg_ops2d_common.inl $
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

#include "vfc/core/vfc_math.hpp"
#include "vfc/linalg/vfc_linalg_matrixbase.hpp"

//=============================================================================
//Matrix - Matrix Addition
//=============================================================================
template <class ValueType, class Matrix1Type, class Matrix2Type, 
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DAddOp<ValueType, 
        vfc::linalg::TMatrixConstRef<ValueType, Matrix1Type, Shape1Type>, 
        vfc::linalg::TMatrixConstRef<ValueType, Matrix2Type, Shape2Type> >, 
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator+ (
    const vfc::linalg::TMatrixBase<ValueType, Matrix1Type, Shape1Type>& f_operand1_r, 
    const vfc::linalg::TMatrixBase<ValueType, Matrix2Type, Shape2Type>& f_operand2_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DAddOp<ValueType, 
            TMatrixConstRef<ValueType, Matrix1Type, Shape1Type>, 
            TMatrixConstRef<ValueType, Matrix2Type, Shape2Type> >, 
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp2DAddOp<ValueType, 
            TMatrixConstRef<ValueType, Matrix1Type, Shape1Type>, 
            TMatrixConstRef<ValueType, Matrix2Type, Shape2Type> >(
                f_operand1_r, f_operand2_r));
}

template <class ValueType, class MatrixType, class OperatorType, 
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DAddOp<ValueType, 
        vfc::linalg::TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
        vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape2Type> >, 
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator+ (
    const vfc::linalg::TMatrixBase<ValueType, MatrixType, Shape1Type>& f_operand1_r, 
    const vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape2Type>& f_operand2_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DAddOp<ValueType, 
            TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
            intern::TExp2D<ValueType, OperatorType, Shape2Type> >, 
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp2DAddOp<ValueType, 
            TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
            intern::TExp2D<ValueType, OperatorType, Shape2Type> >(f_operand1_r, f_operand2_r));
}

template <class ValueType, class MatrixType, class OperatorType, 
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DAddOp<ValueType, 
        vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape1Type>, 
        vfc::linalg::TMatrixConstRef<ValueType, MatrixType, Shape2Type> >, 
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator+ (
    const vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape1Type>& f_operand1_r, 
    const vfc::linalg::TMatrixBase<ValueType, MatrixType, Shape2Type>& f_operand2_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DAddOp<ValueType, 
            intern::TExp2D<ValueType, OperatorType, Shape1Type>, 
            TMatrixConstRef<ValueType, MatrixType, Shape2Type> >, 
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp2DAddOp<ValueType, 
            intern::TExp2D<ValueType, OperatorType, Shape1Type>, 
            TMatrixConstRef<ValueType, MatrixType, Shape2Type> >(
                f_operand1_r, f_operand2_r));
}

template <class ValueType, class Operator1Type, class Operator2Type,
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DAddOp<ValueType, 
        vfc::linalg::intern::TExp2D<ValueType, Operator1Type, Shape1Type>, 
        vfc::linalg::intern::TExp2D<ValueType, Operator2Type, Shape2Type> >, 
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type>
vfc::linalg::operator+ (
    const vfc::linalg::intern::TExp2D<ValueType, Operator1Type, Shape1Type>& f_operand1_r, 
    const vfc::linalg::intern::TExp2D<ValueType, Operator2Type, Shape2Type>& f_operand2_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DAddOp<ValueType, 
            intern::TExp2D<ValueType, Operator1Type, Shape1Type>, 
            intern::TExp2D<ValueType, Operator2Type, Shape2Type> >, 
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp2DAddOp<ValueType, 
            intern::TExp2D<ValueType, Operator1Type, Shape1Type>, 
            intern::TExp2D<ValueType, Operator2Type, Shape2Type> >(f_operand1_r, f_operand2_r));
}
//=============================================================================
//Matrix - Matrix Subtraction
//=============================================================================
template <class ValueType, class Matrix1Type, class Matrix2Type, 
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DSubOp<ValueType, 
        vfc::linalg::TMatrixConstRef<ValueType, Matrix1Type, Shape1Type>, 
        vfc::linalg::TMatrixConstRef<ValueType, Matrix2Type, Shape2Type> >, 
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator- (
    const vfc::linalg::TMatrixBase<ValueType, Matrix1Type, Shape1Type>& f_operand1_r, 
    const vfc::linalg::TMatrixBase<ValueType, Matrix2Type, Shape2Type>& f_operand2_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DSubOp<ValueType, 
            TMatrixConstRef<ValueType, Matrix1Type, Shape1Type>, 
            TMatrixConstRef<ValueType, Matrix2Type, Shape2Type> >, 
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp2DSubOp<ValueType, 
            TMatrixConstRef<ValueType, Matrix1Type, Shape1Type>, 
            TMatrixConstRef<ValueType, Matrix2Type, Shape2Type> >(
                f_operand1_r, f_operand2_r));
}

template <class ValueType, class MatrixType, class OperatorType, 
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DSubOp<ValueType, 
        vfc::linalg::TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
        vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape2Type> >, 
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator- (
    const vfc::linalg::TMatrixBase<ValueType, MatrixType, Shape1Type>& f_operand1_r, 
    const vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape2Type>& f_operand2_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DSubOp<ValueType, 
            TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
            intern::TExp2D<ValueType, OperatorType, Shape2Type> >, 
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp2DSubOp<ValueType, 
            TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
            intern::TExp2D<ValueType, OperatorType, Shape2Type> >(f_operand1_r, f_operand2_r));
}

template <class ValueType, class MatrixType, class OperatorType, 
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DSubOp<ValueType, 
        vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape1Type>, 
        vfc::linalg::TMatrixConstRef<ValueType, MatrixType, Shape2Type> >, 
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator- (
    const vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape1Type>& f_operand1_r, 
    const vfc::linalg::TMatrixBase<ValueType, MatrixType, Shape2Type>& f_operand2_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DSubOp<ValueType, 
            intern::TExp2D<ValueType, OperatorType, Shape1Type>, 
            TMatrixConstRef<ValueType, MatrixType, Shape2Type> >, 
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp2DSubOp<ValueType, 
            intern::TExp2D<ValueType, OperatorType, Shape1Type>, 
            TMatrixConstRef<ValueType, MatrixType, Shape2Type> >(
                f_operand1_r, f_operand2_r));
}

template <class ValueType, class Operator1Type, class Operator2Type, 
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DSubOp<ValueType, 
        vfc::linalg::intern::TExp2D<ValueType, Operator1Type, Shape1Type>, 
        vfc::linalg::intern::TExp2D<ValueType, Operator2Type, Shape2Type> >, 
    typename vfc::linalg::TBinaryShapePromotion<Shape1Type, Shape2Type>::type>
vfc::linalg::operator- (
    const vfc::linalg::intern::TExp2D<ValueType, Operator1Type, Shape1Type>& f_operand1_r, 
    const vfc::linalg::intern::TExp2D<ValueType, Operator2Type, Shape2Type>& f_operand2_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DSubOp<ValueType, 
            intern::TExp2D<ValueType, Operator1Type, Shape1Type>, 
            intern::TExp2D<ValueType, Operator2Type, Shape2Type> >, 
        typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp2DSubOp<ValueType, 
            intern::TExp2D<ValueType, Operator1Type, Shape1Type>, 
            intern::TExp2D<ValueType, Operator2Type, Shape2Type> >(f_operand1_r, f_operand2_r));
}

//=============================================================================
//Matrix Negate
//=============================================================================
template <class ValueType, class MatrixType, class ShapeType>
inline      
const
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DNegateOp<ValueType, 
        vfc::linalg::TMatrixConstRef<ValueType, MatrixType, ShapeType> >, 
    typename MatrixType::shape_type> 
vfc::linalg::operator- (
    const vfc::linalg::TMatrixBase<ValueType, MatrixType, ShapeType>& f_operand_r)
{
    return intern::TExp2D<ValueType,
        intern::TExp2DNegateOp<ValueType, 
            TMatrixConstRef<ValueType, MatrixType, ShapeType> > , 
        typename MatrixType::shape_type>(
            intern::TExp2DNegateOp<ValueType, 
                TMatrixConstRef<ValueType, MatrixType, ShapeType> >(f_operand_r));
}

template <class ValueType, class OperatorType, class ShapeType>
inline      
const
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DNegateOp<ValueType, 
        vfc::linalg::intern::TExp2D<ValueType, OperatorType, ShapeType> >, 
    ShapeType> 
vfc::linalg::operator- (
    const vfc::linalg::intern::TExp2D<ValueType, OperatorType, ShapeType>& f_operand_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DNegateOp<ValueType, 
            intern::TExp2D<ValueType, OperatorType, ShapeType> >, 
        ShapeType>(
            intern::TExp2DNegateOp<ValueType, 
                intern::TExp2D<ValueType, OperatorType, ShapeType> >(f_operand_r));
}
        
//=============================================================================
//Matrix Transpose
//=============================================================================
template <class ValueType, class MatrixType, class ShapeType>
inline     
const
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DTransposeOp<ValueType, 
        vfc::linalg::TMatrixConstRef<ValueType, MatrixType, ShapeType> >, 
    typename vfc::linalg::TTransposeMatrixShapePromotion<
        typename MatrixType::shape_type>::shape_type>
vfc::linalg::transpose (
    const vfc::linalg::TMatrixBase<ValueType, MatrixType, ShapeType>& f_operand_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DTransposeOp<ValueType, 
            TMatrixConstRef<ValueType, MatrixType, ShapeType> >, 
        typename TTransposeMatrixShapePromotion<
            typename MatrixType::shape_type>::shape_type>(
            intern::TExp2DTransposeOp<ValueType, 
                TMatrixConstRef<ValueType, MatrixType, ShapeType> >(f_operand_r));
}

template <class ValueType, class OperatorType, class ShapeType>
inline     
const
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DTransposeOp<ValueType, 
        vfc::linalg::intern::TExp2D<ValueType, OperatorType, ShapeType> >, 
    typename vfc::linalg::TTransposeMatrixShapePromotion<ShapeType>::shape_type>
vfc::linalg::transpose (
    const intern::TExp2D<ValueType, OperatorType, ShapeType>& f_operand_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DTransposeOp<ValueType, 
            intern::TExp2D<ValueType, OperatorType, ShapeType> >, 
        typename TTransposeMatrixShapePromotion<ShapeType>::shape_type>(
            intern::TExp2DTransposeOp<ValueType, 
                intern::TExp2D<ValueType, OperatorType, ShapeType> >(f_operand_r));
}

//=============================================================================
//Matrix - Scalar Multiplication
//=============================================================================
template <class ValueType, class MatrixType, class ShapeType>
inline      
const
vfc::linalg::intern::TExp2D <ValueType, 
    vfc::linalg::intern::TExp2DScalarOp<ValueType, 
        vfc::linalg::TMatrixConstRef<ValueType, MatrixType, ShapeType>, 
        stlalias::multiplies<ValueType> > , 
    typename MatrixType::shape_type> 
vfc::linalg::operator* (
    const vfc::linalg::TMatrixBase<ValueType, MatrixType,ShapeType>& f_operand1_r, 
    const ValueType& f_operand2_r)
{
    return intern::TExp2D <ValueType, 
    intern::TExp2DScalarOp<ValueType, 
        TMatrixConstRef<ValueType, MatrixType, ShapeType>, 
        stlalias::multiplies<ValueType> > , 
    typename MatrixType::shape_type>(
        intern::TExp2DScalarOp<ValueType, 
        TMatrixConstRef<ValueType, MatrixType, ShapeType>, 
        stlalias::multiplies<ValueType> >(f_operand1_r, f_operand2_r));

}

template <class ValueType, class MatrixType, class ShapeType>
inline      
const
vfc::linalg::intern::TExp2D <ValueType, 
    vfc::linalg::intern::TExp2DScalarOp<ValueType, 
        vfc::linalg::TMatrixConstRef<ValueType, MatrixType, ShapeType>, 
        stlalias::multiplies<ValueType> > , 
    typename MatrixType::shape_type> 
vfc::linalg::operator* (
    const ValueType& f_operand1_r,
    const vfc::linalg::TMatrixBase<ValueType, MatrixType,ShapeType>& f_operand2_r)
{
    return intern::TExp2D <ValueType, 
    intern::TExp2DScalarOp<ValueType, 
        TMatrixConstRef<ValueType, MatrixType, ShapeType>, 
        stlalias::multiplies<ValueType> > , 
    typename MatrixType::shape_type>(
        intern::TExp2DScalarOp<ValueType, 
        TMatrixConstRef<ValueType, MatrixType, ShapeType>, 
        stlalias::multiplies<ValueType> >(f_operand2_r, f_operand1_r));

}

template <class ValueType, class OperatorType, class ShapeType>
inline      
const
vfc::linalg::intern::TExp2D <ValueType, 
    vfc::linalg::intern::TExp2DScalarOp<ValueType, 
        vfc::linalg::intern::TExp2D<ValueType, OperatorType, ShapeType>, 
        stlalias::multiplies<ValueType> > , 
    ShapeType> 
vfc::linalg::operator* (
    const vfc::linalg::intern::TExp2D<ValueType, OperatorType, ShapeType>& f_operand1_r,
    const ValueType& f_operand2_r)
{
    return intern::TExp2D <ValueType, 
    intern::TExp2DScalarOp<ValueType, 
        intern::TExp2D<ValueType, OperatorType, ShapeType>, 
        stlalias::multiplies<ValueType> > , 
    ShapeType>(
        intern::TExp2DScalarOp<ValueType, 
        intern::TExp2D<ValueType, OperatorType, ShapeType>, 
        stlalias::multiplies<ValueType> >(f_operand1_r, f_operand2_r));

}

template <class ValueType, class OperatorType, class ShapeType>
inline      
const
vfc::linalg::intern::TExp2D <ValueType, 
    vfc::linalg::intern::TExp2DScalarOp<ValueType, 
        vfc::linalg::intern::TExp2D<ValueType, OperatorType, ShapeType>, 
        stlalias::multiplies<ValueType> > , 
    ShapeType> 
vfc::linalg::operator* (
    const ValueType& f_operand1_r,
    const vfc::linalg::intern::TExp2D<ValueType, OperatorType, ShapeType>& f_operand2_r)
{
    return intern::TExp2D <ValueType, 
    intern::TExp2DScalarOp<ValueType, 
        intern::TExp2D<ValueType, OperatorType, ShapeType>, 
        stlalias::multiplies<ValueType> > , 
    ShapeType>(
        intern::TExp2DScalarOp<ValueType, 
        intern::TExp2D<ValueType, OperatorType, ShapeType>, 
        stlalias::multiplies<ValueType> >(f_operand2_r, f_operand1_r));

}
//=============================================================================
//Matrix - Scalar Division
//=============================================================================
template <class ValueType, class MatrixType, class ShapeType>
inline      
const
vfc::linalg::intern::TExp2D <ValueType, 
    vfc::linalg::intern::TExp2DDivScalarOp<ValueType, 
        vfc::linalg::TMatrixConstRef<ValueType, MatrixType, ShapeType>, 
        stlalias::divides<ValueType> > , 
    typename MatrixType::shape_type> 
vfc::linalg::operator/ (
    const vfc::linalg::TMatrixBase<ValueType, MatrixType,ShapeType>& f_operand1_r, 
    const ValueType& f_operand2_r)
{
    VFC_REQUIRE( !vfc::isZero(f_operand2_r) );
    return intern::TExp2D <ValueType, 
    intern::TExp2DDivScalarOp<ValueType, 
        TMatrixConstRef<ValueType, MatrixType, ShapeType>, 
        stlalias::divides<ValueType> > , 
    typename MatrixType::shape_type>(
        intern::TExp2DDivScalarOp<ValueType, 
        TMatrixConstRef<ValueType, MatrixType, ShapeType>, 
        stlalias::divides<ValueType> >(f_operand1_r, f_operand2_r));

}

template <class ValueType, class OperatorType, class ShapeType>
inline      
const
vfc::linalg::intern::TExp2D <ValueType, 
    vfc::linalg::intern::TExp2DDivScalarOp<ValueType, 
        vfc::linalg::intern::TExp2D<ValueType, OperatorType, ShapeType>, 
        stlalias::divides<ValueType> > , 
    ShapeType> 
vfc::linalg::operator/ (
    const vfc::linalg::intern::TExp2D<ValueType, OperatorType, ShapeType>& f_operand1_r,
    const ValueType& f_operand2_r)
{
    VFC_REQUIRE( !vfc::isZero(f_operand2_r) );
    return intern::TExp2D <ValueType, 
    intern::TExp2DDivScalarOp<ValueType, 
        intern::TExp2D<ValueType, OperatorType, ShapeType>, 
        stlalias::divides<ValueType> > , 
    ShapeType>(
        intern::TExp2DDivScalarOp<ValueType, 
        intern::TExp2D<ValueType, OperatorType, ShapeType>, 
        stlalias::divides<ValueType> >(f_operand1_r, f_operand2_r));

}
//=============================================================================
//Matrix - Matrix Multiplication
//=============================================================================
template <class ValueType, class Matrix1Type, class Matrix2Type, 
    class Shape1Type, class Shape2Type>
inline    
const
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DMulOp<ValueType, 
        vfc::linalg::TMatrixConstRef<ValueType, Matrix1Type, Shape1Type>, 
        vfc::linalg::TMatrixConstRef<ValueType, Matrix2Type, Shape2Type> >, 
    typename vfc::linalg::TMulShapePromotion<Shape1Type, Shape2Type>::type>
vfc::linalg::operator* (
    const vfc::linalg::TMatrixBase<ValueType, Matrix1Type, Shape1Type>& f_operand1_r, 
    const vfc::linalg::TMatrixBase<ValueType, Matrix2Type, Shape2Type>& f_operand2_r)
{
    VFC_REQUIRE(f_operand1_r.getNbColumns() == f_operand2_r.getNbRows());
    return intern::TExp2D<ValueType, 
        intern::TExp2DMulOp<ValueType, 
            TMatrixConstRef<ValueType, Matrix1Type, Shape1Type>, 
            TMatrixConstRef<ValueType, Matrix2Type, Shape2Type> >, 
        typename TMulShapePromotion<Shape1Type, Shape2Type>::type>(
            intern::TExp2DMulOp<ValueType, 
            TMatrixConstRef<ValueType, Matrix1Type, Shape1Type>, 
            TMatrixConstRef<ValueType, Matrix2Type, Shape2Type> >(
                f_operand1_r, f_operand2_r));
}

template <class ValueType, class MatrixType, class OperatorType, 
    class Shape1Type, class Shape2Type>
inline    
const
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DMulOp<ValueType, 
        vfc::linalg::TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
        vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape2Type> >, 
    typename vfc::linalg::TMulShapePromotion<Shape1Type, Shape2Type>::type>
vfc::linalg::operator* (
    const vfc::linalg::TMatrixBase<ValueType, MatrixType, Shape1Type>& f_operand1_r, 
    const vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape2Type>& f_operand2_r)
{
    VFC_REQUIRE(f_operand1_r.getNbColumns() == f_operand2_r.getNbRows());
    return intern::TExp2D<ValueType, 
        intern::TExp2DMulOp<ValueType, 
            TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
            intern::TExp2D<ValueType, OperatorType, Shape2Type> >, 
        typename TMulShapePromotion<Shape1Type, Shape2Type>::type>(
            intern::TExp2DMulOp<ValueType, 
            TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
            intern::TExp2D<ValueType, OperatorType, Shape2Type> >(
                f_operand1_r, f_operand2_r));
}

template <class ValueType, class MatrixType, class OperatorType, 
    class Shape1Type, class Shape2Type>
inline    
const
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DMulOp<ValueType, 
        vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape1Type>, 
        vfc::linalg::TMatrixConstRef<ValueType, MatrixType, Shape2Type> >, 
    typename vfc::linalg::TMulShapePromotion<Shape1Type, Shape2Type>::type>
vfc::linalg::operator* (
    const vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape1Type>& f_operand1_r, 
    const vfc::linalg::TMatrixBase<ValueType, MatrixType, Shape2Type>& f_operand2_r)
{
    VFC_REQUIRE(f_operand1_r.getNbColumns() == f_operand2_r.getNbRows());
    return intern::TExp2D<ValueType, 
        intern::TExp2DMulOp<ValueType, 
            intern::TExp2D<ValueType, OperatorType, Shape1Type>, 
            TMatrixConstRef<ValueType, MatrixType, Shape2Type> >, 
        typename TMulShapePromotion<Shape1Type, Shape2Type>::type>(
            intern::TExp2DMulOp<ValueType, 
            intern::TExp2D<ValueType, OperatorType, Shape1Type>, 
            TMatrixConstRef<ValueType, MatrixType, Shape2Type> >(
                f_operand1_r, f_operand2_r));
}

template <class ValueType, class Operator1Type, class Operator2Type, 
    class Shape1Type, class Shape2Type>
inline    
const
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DMulOp<ValueType, 
        vfc::linalg::intern::TExp2D<ValueType, Operator1Type, Shape1Type>, 
        vfc::linalg::intern::TExp2D<ValueType, Operator2Type, Shape2Type> >, 
    typename vfc::linalg::TMulShapePromotion<Shape1Type, Shape2Type>::type>
vfc::linalg::operator* (
    const vfc::linalg::intern::TExp2D<ValueType, Operator1Type, Shape1Type>& f_operand1_r, 
    const vfc::linalg::intern::TExp2D<ValueType, Operator2Type, Shape2Type>& f_operand2_r)
{
    VFC_REQUIRE(f_operand1_r.getNbColumns() == f_operand2_r.getNbRows());
    return intern::TExp2D<ValueType, 
        intern::TExp2DMulOp<ValueType, 
            intern::TExp2D<ValueType, Operator1Type, Shape1Type>, 
            intern::TExp2D<ValueType, Operator2Type, Shape2Type> >, 
        typename TMulShapePromotion<Shape1Type, Shape2Type>::type>(
            intern::TExp2DMulOp<ValueType, 
            intern::TExp2D<ValueType, Operator1Type, Shape1Type>, 
            intern::TExp2D<ValueType, Operator2Type, Shape2Type> >(
                f_operand1_r, f_operand2_r));
}



//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_ops2d_common.inl  $
//  Revision 1.3 2009/05/28 09:19:03MESZ Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) 
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.2 2007/07/02 13:13:22CEST Jaeger Thomas (CC-DA/ESV2) (jat2hi) 
//  - moved definition of the dynamic submatrix() op from the common files to the dynamic specific files (mantis 1726)
//  Revision 1.1 2007/05/09 10:21:25CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
