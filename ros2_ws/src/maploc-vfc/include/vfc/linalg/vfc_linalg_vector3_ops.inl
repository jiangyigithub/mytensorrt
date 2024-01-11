//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2007 by Robert Bosch GmbH. All rights reserved.
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
///     $Source: vfc_linalg_vector3_ops.inl $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (jat2hi) $
///     $Date: 2007/05/09 10:21:28MESZ $
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


//works only for vectors of size = 3, static_asserts for other values
template<class ValueType, vfc::int32_t RowValue, class Vector1Type, class Vector2Type>
inline
vfc::linalg::TVectorN<ValueType, RowValue> 
vfc::linalg::cross (
    const vfc::linalg::TVectorBase<ValueType, Vector1Type, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op1_r, 
    const vfc::linalg::TVectorBase<ValueType, Vector2Type, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op2_r)

{
    VFC_STATIC_ASSERT((3 == RowValue));
    TVectorN<ValueType, RowValue> crossVec;
    crossVec[0] = f_op1_r[1]*f_op2_r[2] - f_op2_r[1]*f_op1_r[2];
    crossVec[1] = f_op2_r[0]*f_op1_r[2] - f_op1_r[0]*f_op2_r[2];
    crossVec[2] = f_op1_r[0]*f_op2_r[1] - f_op2_r[0]*f_op1_r[1];
    return crossVec;
}

template<class ValueType, vfc::int32_t RowValue, class VectorType, class OperatorType>
inline
vfc::linalg::TVectorN<ValueType, RowValue> 
vfc::linalg::cross (
    const vfc::linalg::TVectorBase<ValueType, VectorType, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op1_r, 
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op2_r)
{
    VFC_STATIC_ASSERT((3 == RowValue));
    //Copy the expressions to a local variable so that it does not get calaculated again
    ValueType op2_0 = f_op2_r[0], op2_1 = f_op2_r[1], op2_2 = f_op2_r[2];
    TVectorN<ValueType, RowValue> crossVec;
    crossVec[0] = f_op1_r[1]*op2_2 - op2_1*f_op1_r[2];
    crossVec[1] = op2_0*f_op1_r[2] - f_op1_r[0]*op2_2;
    crossVec[2] = f_op1_r[0]*op2_1 - op2_0*f_op1_r[1];
    return crossVec;
}

template<class ValueType, vfc::int32_t RowValue, class VectorType, class OperatorType>
inline
vfc::linalg::TVectorN<ValueType, RowValue> 
vfc::linalg::cross (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op1_r, 
    const vfc::linalg::TVectorBase<ValueType, VectorType, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op2_r)
{
    VFC_STATIC_ASSERT((3 == RowValue));
    //Copy the expressions to a local variable so that it does not get calaculated again
    ValueType op1_0 = f_op1_r[0], op1_1 = f_op1_r[1], op1_2 = f_op1_r[2];
    TVectorN<ValueType, RowValue> crossVec;
    crossVec[0] = op1_1*f_op2_r[2] - f_op2_r[1]*op1_2;
    crossVec[1] = f_op2_r[0]*op1_2 - op1_0*f_op2_r[2];
    crossVec[2] = op1_0*f_op2_r[1] - f_op2_r[0]*op1_1;
    return crossVec;
}

template<class ValueType, vfc::int32_t RowValue, class Operator1Type, class Operator2Type>
vfc::linalg::TVectorN<ValueType, RowValue> 
vfc::linalg::cross (
    const vfc::linalg::intern::TExp1D<ValueType, Operator1Type, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op1_r, 
    const vfc::linalg::intern::TExp1D<ValueType, Operator2Type, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op2_r)
{
    VFC_STATIC_ASSERT((3 == RowValue));
    //Copy the expressions to a local variable so that it does not get calaculated again
    ValueType op1_0 = f_op1_r[0], op1_1 = f_op1_r[1], op1_2 = f_op1_r[2];
    ValueType op2_0 = f_op2_r[0], op2_1 = f_op2_r[1], op2_2 = f_op2_r[2];
    TVectorN<ValueType, RowValue> crossVec;
    crossVec[0] = op1_1*op2_2 - op2_1*op1_2;
    crossVec[1] = op2_0*op1_2 - op1_0*op2_2;
    crossVec[2] = op1_0*op2_1 - op2_0*op1_1;
    return crossVec;
}




template<class ValueType, vfc::int32_t RowValue, class Vector1Type, class Vector2Type>
inline
ValueType  
vfc::linalg::area (
    const vfc::linalg::TVectorBase<ValueType, Vector1Type, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op1_r, 
    const vfc::linalg::TVectorBase<ValueType, Vector2Type, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op2_r)

{
    return length(cross(f_op1_r, f_op2_r));
}

template<class ValueType, vfc::int32_t RowValue, class VectorType, class OperatorType>
inline
ValueType  
vfc::linalg::area (
    const vfc::linalg::TVectorBase<ValueType, VectorType, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op1_r, 
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op2_r)
{
    return length(cross(f_op1_r, f_op2_r));
}

template<class ValueType, vfc::int32_t RowValue, class VectorType, class OperatorType>
inline
ValueType  
vfc::linalg::area (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op1_r, 
    const vfc::linalg::TVectorBase<ValueType, VectorType, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op2_r)
{
    return length(cross(f_op1_r, f_op2_r));
}

template<class ValueType, vfc::int32_t RowValue, class Operator1Type, class Operator2Type>
ValueType 
vfc::linalg::area (
    const vfc::linalg::intern::TExp1D<ValueType, Operator1Type, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op1_r, 
    const vfc::linalg::intern::TExp1D<ValueType, Operator2Type, 
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_op2_r)
{
    return length(cross(f_op1_r, f_op2_r));
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_vector3_ops.inl  $
//  Revision 1.1 2007/05/09 10:21:28MESZ Jaeger Thomas (CC-DA/EPV2) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
