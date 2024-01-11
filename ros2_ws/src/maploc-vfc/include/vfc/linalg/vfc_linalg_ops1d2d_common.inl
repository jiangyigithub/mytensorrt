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
///     $Source: vfc_linalg_ops1d2d_common.inl $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (jat2hi) $
///     $Date: 2007/05/09 10:21:23MESZ $
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

#include "vfc/linalg/vfc_linalg_vectorbase.hpp"
#include "vfc/linalg/vfc_linalg_matrixbase.hpp"

template <class ValueType, class MatrixType, class VectorType, 
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp1D<ValueType, 
    vfc::linalg::intern::TExp2D1DMulOp<ValueType, 
        vfc::linalg::TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
        vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape2Type> >, 
    typename vfc::linalg::TExp2D1DMulShapePromotion<Shape1Type, Shape2Type>::type >
    vfc::linalg::operator* (const vfc::linalg::TMatrixBase<ValueType, MatrixType, Shape1Type>& f_op1_r, 
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r)
{
    return
        vfc::linalg::intern::TExp1D<ValueType, 
            vfc::linalg::intern::TExp2D1DMulOp<ValueType, 
                vfc::linalg::TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
                vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape2Type> >, 
            typename vfc::linalg::TExp2D1DMulShapePromotion<Shape1Type, Shape2Type>::type >(
                vfc::linalg::intern::TExp2D1DMulOp<ValueType, 
                vfc::linalg::TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
                vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape2Type> >(
                    f_op1_r, f_op2_r));
}

template <class ValueType, class MatrixType, class OperatorType, 
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp1D<ValueType, 
    vfc::linalg::intern::TExp2D1DMulOp<ValueType, 
        vfc::linalg::TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type> >, 
    typename vfc::linalg::TExp2D1DMulShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator* (const vfc::linalg::TMatrixBase<ValueType, MatrixType, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r)
{
    return vfc::linalg::intern::TExp1D<ValueType, 
            vfc::linalg::intern::TExp2D1DMulOp<ValueType, 
                vfc::linalg::TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
                vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type> >, 
            typename vfc::linalg::TExp2D1DMulShapePromotion<Shape1Type, Shape2Type>::type >(
                vfc::linalg::intern::TExp2D1DMulOp<ValueType, 
                    vfc::linalg::TMatrixConstRef<ValueType, MatrixType, Shape1Type>, 
                    vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type> >(
                        f_op1_r, f_op2_r));

}

template <class ValueType, class VectorType, class OperatorType, 
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp1D<ValueType, 
    vfc::linalg::intern::TExp2D1DMulOp<ValueType, 
        vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape1Type>, 
        vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape2Type> >, 
    typename vfc::linalg::TExp2D1DMulShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator* (const vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape1Type>& f_op1_r, 
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r)
{
    return vfc::linalg::intern::TExp1D<ValueType, 
            vfc::linalg::intern::TExp2D1DMulOp<ValueType, 
                vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape1Type>, 
                vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape2Type> >, 
            typename vfc::linalg::TExp2D1DMulShapePromotion<Shape1Type, Shape2Type>::type >(
                vfc::linalg::intern::TExp2D1DMulOp<ValueType, 
                vfc::linalg::intern::TExp2D<ValueType, OperatorType, Shape1Type>, 
                vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape2Type> >(
                    f_op1_r, f_op2_r));
}

template <class ValueType, class Operator1Type, class Operator2Type, 
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp1D<ValueType, 
    vfc::linalg::intern::TExp2D1DMulOp<ValueType, 
        vfc::linalg::intern::TExp2D<ValueType, Operator1Type, Shape1Type>, 
        vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type> >, 
    typename vfc::linalg::TExp2D1DMulShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::operator* (const vfc::linalg::intern::TExp2D<ValueType, Operator1Type, Shape1Type>& f_op1_r, 
    const vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r)
{
    return vfc::linalg::intern::TExp1D<ValueType, 
            vfc::linalg::intern::TExp2D1DMulOp<ValueType, 
                vfc::linalg::intern::TExp2D<ValueType, Operator1Type, Shape1Type>, 
                vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type> >, 
            typename vfc::linalg::TExp2D1DMulShapePromotion<Shape1Type, Shape2Type>::type >(
                vfc::linalg::intern::TExp2D1DMulOp<ValueType, 
                vfc::linalg::intern::TExp2D<ValueType, Operator1Type, Shape1Type>, 
                vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type> >(
                    f_op1_r, f_op2_r));
}

template <class ValueType, class Vector1Type, class Vector2Type, 
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DDyadProductOp<ValueType, 
        vfc::linalg::TVectorConstRef<ValueType, Vector1Type, Shape1Type>, 
        vfc::linalg::TVectorConstRef<ValueType, Vector2Type, Shape2Type> >, 
    typename vfc::linalg::TVectorDyadProductShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::dyad (
    const vfc::linalg::TVectorBase<ValueType, Vector1Type, Shape1Type>& f_op1_r, 
    const vfc::linalg::TVectorBase<ValueType, Vector2Type, Shape2Type>& f_op2_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DDyadProductOp<ValueType, 
            TVectorConstRef<ValueType, Vector1Type, Shape1Type>, 
            TVectorConstRef<ValueType, Vector2Type, Shape2Type> >, 
        typename TVectorDyadProductShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp2DDyadProductOp<ValueType, 
            TVectorConstRef<ValueType, Vector1Type, Shape1Type>, 
            TVectorConstRef<ValueType, Vector2Type, Shape2Type> >(
                f_op1_r, f_op2_r));
}



template <class ValueType, class VectorType, class OperatorType,  
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DDyadProductOp<ValueType, 
        vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape1Type>, 
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type> >, 
    typename vfc::linalg::TVectorDyadProductShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::dyad (
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape1Type>& f_op1_r, 
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DDyadProductOp<ValueType, 
            TVectorConstRef<ValueType, VectorType, Shape1Type>, 
            intern::TExp1D<ValueType, OperatorType, Shape2Type> >, 
        typename TVectorDyadProductShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp2DDyadProductOp<ValueType, 
            TVectorConstRef<ValueType, VectorType, Shape1Type>, 
            intern::TExp1D<ValueType, OperatorType, Shape2Type> >(
                f_op1_r, f_op2_r));
}

template <class ValueType, class VectorType, class OperatorType,  
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DDyadProductOp<ValueType, 
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape1Type>,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, Shape2Type> >, 
    typename vfc::linalg::TVectorDyadProductShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::dyad (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape1Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DDyadProductOp<ValueType, 
            intern::TExp1D<ValueType, OperatorType, Shape1Type>,
            TVectorConstRef<ValueType, VectorType, Shape2Type> >, 
        typename TVectorDyadProductShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp2DDyadProductOp<ValueType, 
            intern::TExp1D<ValueType, OperatorType, Shape1Type>,
            TVectorConstRef<ValueType, VectorType, Shape2Type> >(
                f_op1_r, f_op2_r));
}

template <class ValueType, class Operator1Type, class Operator2Type, 
    class Shape1Type, class Shape2Type>
inline
const 
vfc::linalg::intern::TExp2D<ValueType, 
    vfc::linalg::intern::TExp2DDyadProductOp<ValueType, 
        vfc::linalg::intern::TExp1D<ValueType, Operator1Type, Shape1Type>, 
        vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type> >, 
    typename vfc::linalg::TVectorDyadProductShapePromotion<Shape1Type, Shape2Type>::type >
vfc::linalg::dyad (
    const vfc::linalg::intern::TExp1D<ValueType, Operator1Type, Shape1Type>& f_op1_r, 
    const vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r)
{
    return intern::TExp2D<ValueType, 
        intern::TExp2DDyadProductOp<ValueType, 
            intern::TExp1D<ValueType, Operator1Type, Shape1Type>, 
            intern::TExp1D<ValueType, Operator2Type, Shape2Type> >, 
        typename TVectorDyadProductShapePromotion<Shape1Type, Shape2Type>::type >(
            intern::TExp2DDyadProductOp<ValueType, 
            intern::TExp1D<ValueType, Operator1Type, Shape1Type>, 
            intern::TExp1D<ValueType, Operator2Type, Shape2Type> >(
                f_op1_r, f_op2_r));
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_ops1d2d_common.inl  $
//  Revision 1.1 2007/05/09 10:21:23MESZ Jaeger Thomas (CC-DA/EPV2) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
