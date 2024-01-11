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
///     $Source: vfc_linalg_vector_ops.inl $
///     $Revision: 1.3 $
///     $Author: gaj2kor $
///     $Date: 2009/01/08 10:04:13MEZ $
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

#include "vfc/linalg/vfc_linalg_exp1d.hpp"
#include "vfc/linalg/vfc_linalg_ops1d.hpp"
#include "vfc/core/vfc_assert.hpp"

template <class ValueType, class Vector1Type, class Vector2Type,
            class Shape1Type, class Shape2Type>
                vfc::linalg::TVector<ValueType>
vfc::linalg::cross (
    const vfc::linalg::TVectorBase<ValueType, Vector1Type, Shape1Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, Vector2Type, Shape2Type>& f_op2_r)
{
    VFC_REQUIRE((f_op1_r.getDim() == 3) && (f_op2_r.getDim() == 3));
    TVector<ValueType> crossVec(3);
    crossVec[0] = f_op1_r[1]*f_op2_r[2] - f_op2_r[1]*f_op1_r[2];
    crossVec[1] = f_op2_r[0]*f_op1_r[2] - f_op1_r[0]*f_op2_r[2];
    crossVec[2] = f_op1_r[0]*f_op2_r[1] - f_op2_r[0]*f_op1_r[1];
    return crossVec;
}

template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
vfc::linalg::TVector<ValueType>
vfc::linalg::cross (
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r)
{
    VFC_REQUIRE((f_op1_r.getDim() == 3) && (f_op2_r.getDim() == 3));
    //Copy the expressions to a local variable so that it does not get calaculated again
    ValueType op2_0 = f_op2_r[0], op2_1 = f_op2_r[1], op2_2 = f_op2_r[2];
    TVector<ValueType> crossVec(3);
    crossVec[0] = f_op1_r[1]*op2_2 - op2_1*f_op1_r[2];
    crossVec[1] = op2_0*f_op1_r[2] - f_op1_r[0]*op2_2;
    crossVec[2] = f_op1_r[0]*op2_1 - op2_0*f_op1_r[1];
    return crossVec;
}

template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
vfc::linalg::TVector<ValueType>
vfc::linalg::cross (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape1Type>& f_op2_r)
{
    VFC_REQUIRE((f_op1_r.getDim() == 3) && (f_op2_r.getDim() == 3));
    //Copy the expressions to a local variable so that it does not get calaculated again
    ValueType op1_0 = f_op1_r[0], op1_1 = f_op1_r[1], op1_2 = f_op1_r[2];
    TVector<ValueType> crossVec(3);
    crossVec[0] = op1_1*f_op2_r[2] - f_op2_r[1]*op1_2;
    crossVec[1] = f_op2_r[0]*op1_2 - op1_0*f_op2_r[2];
    crossVec[2] = op1_0*f_op2_r[1] - f_op2_r[0]*op1_1;
    return crossVec;
}

template <class ValueType, class Operator1Type, class Operator2Type,
            class Shape1Type, class Shape2Type>
vfc::linalg::TVector<ValueType>
vfc::linalg::cross (
    const vfc::linalg::intern::TExp1D<ValueType, Operator1Type, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r)
{
    VFC_REQUIRE((f_op1_r.getDim() == 3) && (f_op2_r.getDim() == 3));
    //Copy the expressions to a local variable so that it does not get calaculated again
    ValueType op1_0 = f_op1_r[0], op1_1 = f_op1_r[1], op1_2 = f_op1_r[2];
    ValueType op2_0 = f_op2_r[0], op2_1 = f_op2_r[1], op2_2 = f_op2_r[2];
    TVector<ValueType> crossVec(3);
    crossVec[0] = op1_1*op2_2 - op2_1*op1_2;
    crossVec[1] = op2_0*op1_2 - op1_0*op2_2;
    crossVec[2] = op1_0*op2_1 - op2_0*op1_1;
    return crossVec;
}


template <class ValueType, class Vector1Type, class Vector2Type,
            class Shape1Type, class Shape2Type>
ValueType
vfc::linalg::area (
    const vfc::linalg::TVectorBase<ValueType, Vector1Type, Shape1Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, Vector2Type, Shape2Type>& f_op2_r)
{
    VFC_REQUIRE((f_op1_r.getDim() == 2 || f_op1_r.getDim() == 3) \
        && (f_op1_r.getDim() == 2 || f_op2_r.getDim() == 3));

    ValueType l_area = static_cast<ValueType>(0);
    if(2 == f_op1_r.getDim())
    {

        l_area = f_op1_r[0]*f_op2_r[1] - f_op1_r[1]*f_op2_r[0];
    }
    else if(3 == f_op1_r.getDim())
    {
        l_area = length(cross(f_op1_r, f_op2_r));
    }

    return l_area;
}

template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
ValueType
vfc::linalg::area (
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r)
{
    VFC_REQUIRE((f_op1_r.getDim() == 2 || f_op1_r.getDim() == 3) \
        && (f_op1_r.getDim() == 2 || f_op2_r.getDim() == 3));

    ValueType l_area = static_cast<ValueType>(0);
    if(2 == f_op1_r.getDim())
    {
        l_area = f_op1_r[0]*f_op2_r[1] - f_op1_r[1]*f_op2_r[0];
    }
    else if(3 == f_op1_r.getDim())
    {
        l_area = length(cross(f_op1_r, f_op2_r));
    }

    return l_area;
}

template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
ValueType
vfc::linalg::area (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op1_r,
    const vfc::linalg::TVectorBase<ValueType, VectorType, Shape1Type>& f_op2_r)
{
    VFC_REQUIRE((f_op1_r.getDim() == 2 || f_op1_r.getDim() == 3) \
        && (f_op1_r.getDim() == 2 || f_op2_r.getDim() == 3));

    ValueType l_area = static_cast<ValueType>(0);
    if(2 == f_op1_r.getDim())
    {
        l_area = f_op1_r[0]*f_op2_r[1] - f_op1_r[1]*f_op2_r[0];
    }
    else if(3 == f_op1_r.getDim())
    {
        l_area = length(cross(f_op1_r, f_op2_r));
    }

    return l_area;
}

template <class ValueType, class Operator1Type, class Operator2Type,
            class Shape1Type, class Shape2Type>
ValueType
vfc::linalg::area (
    const vfc::linalg::intern::TExp1D<ValueType, Operator1Type, Shape1Type>& f_op1_r,
    const vfc::linalg::intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r)
{
    VFC_REQUIRE((f_op1_r.getDim() == 2 || f_op1_r.getDim() == 3) \
        && (f_op1_r.getDim() == 2 || f_op2_r.getDim() == 3));

    ValueType l_area = static_cast<ValueType>(0);
    if(2 == f_op1_r.getDim())
    {
        l_area = f_op1_r[0]*f_op2_r[1] - f_op1_r[1]*f_op2_r[0];
    }
    else if(3 == f_op1_r.getDim())
    {
        l_area = length(cross(f_op1_r, f_op2_r));
    }

    return l_area;
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_vector_ops.inl  $
//  Revision 1.3 2009/01/08 10:04:13MEZ gaj2kor 
//  Removal of comments.
//  (Mantis : 0002479)
//  Revision 1.2 2009/01/07 10:20:30IST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  QAC++ Warning (2504) Removal.
//  (Mantis : 0002479)
//  Revision 1.1 2007/05/09 13:51:29IST Jaeger Thomas (CC-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
