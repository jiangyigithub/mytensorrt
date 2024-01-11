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
///     $Source: vfc_linalg_matrixmn.inl $
///     $Revision: 1.4 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/08/18 16:24:16MESZ $
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

#include "vfc/core/vfc_math.hpp"
#include "vfc/linalg/vfc_linalg_algorithm2d.hpp"
#include "vfc/linalg/vfc_linalg_shapepromotion.hpp"
#include "vfc/core/vfc_assert.hpp"  // used for VFC_ASSERT(), VFC_REQUIRE(), VFC_ENSURE()
#include "vfc/core/vfc_types.hpp"   // used for vfc::int32_t
#include <functional>

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
inline
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::TMatrixMN(
    const TMatrixMN<ValueType, RowValue, ColValue>& f_param_r)
    : TMatrixBase<ValueType, TMatrixMN<ValueType, RowValue, ColValue>, TStaticRectangle<RowValue, ColValue> >()
{
    operator=(f_param_r);
}


template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
inline
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::TMatrixMN(
    const ValueType& f_value_r)
{
    intern::copy_matrix_fixed_mn<RowValue, ColValue>(
        intern::TScalarWrapper<ValueType>(f_value_r), *this);
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
inline
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::TMatrixMN(
    vfc::int32_t VFC_USE_VAR_ONLY_IN_ASSERTION(f_rows), vfc::int32_t VFC_USE_VAR_ONLY_IN_ASSERTION(f_cols),
        const ValueType& f_value_r)
{
    VFC_ASSERT( (NB_ROWS == f_rows) && (NB_COLUMNS == f_cols) );
    intern::copy_matrix_fixed_mn<RowValue, ColValue>(
        intern::TScalarWrapper<ValueType>(f_value_r), *this);
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
template<class OperatorType>
inline
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::TMatrixMN(
    const intern::TExp2D<ValueType, OperatorType,
        TStaticRectangle<RowValue, ColValue> >& f_param_r)
{
    operator=(f_param_r);
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
inline
const vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>&
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::operator=(
    const TMatrixMN<ValueType, RowValue, ColValue>& f_rhs_r)
{
    //ensure self assignment
    if (this != &f_rhs_r)
    {
        intern::copy_matrix_fixed_mn<RowValue, ColValue>(f_rhs_r, *this);
    }
    return *this;
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
template <class OperatorType>
inline
const vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>&
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::operator=(
    const intern::TExp2D<ValueType, OperatorType,
        TStaticRectangle<RowValue, ColValue> >& f_rhs_r)
{
    intern::copy_matrix_fixed_mn<RowValue, ColValue>(f_rhs_r, *this);
    return *this;
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
inline
void vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::assign(
    const vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>& f_rhs_r)
{
    TMatrixMN<ValueType, RowValue, ColValue> temp(f_rhs_r);
    intern::copy_matrix_fixed_mn<RowValue,ColValue>(temp, *this);
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
template <class OperatorType>
inline
void vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::assign(
    const vfc::linalg::intern::TExp2D<ValueType, OperatorType,
        vfc::linalg::TStaticRectangle<RowValue, ColValue> >& f_rhs_r)
{
    TMatrixMN<ValueType, RowValue, ColValue> temp(f_rhs_r);
    intern::copy_matrix_fixed_mn<RowValue,ColValue>(temp, *this);
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
inline
void vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::assign_to(
    TMatrixMN<ValueType, RowValue, ColValue>& f_rhs_r) const
{
    intern::copy_matrix_fixed_mn<RowValue,ColValue>(*this, f_rhs_r);
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
template <class OperationFunctor>
inline
void vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::opassign(
    const vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>& f_rhs_r,
    const OperationFunctor& f_functor_r)
{
    TMatrixMN<ValueType, RowValue, ColValue> temp(f_rhs_r);
    intern::do_operation_matrix_fixed_mn<RowValue, ColValue>(*this, temp, *this,
        f_functor_r);
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
template <class OperatorType, class OperationFunctor>
inline
void vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::opassign(
    const intern::TExp2D<ValueType, OperatorType,
        TStaticRectangle<RowValue, ColValue> >& f_rhs_r,
    const OperationFunctor& f_functor_r)
{
    TMatrixMN<ValueType, RowValue, ColValue> temp(f_rhs_r);
    intern::do_operation_matrix_fixed_mn<RowValue, ColValue>(*this, temp, *this,
        f_functor_r);
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
template <class OperationFunctor>
inline
void vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::unaryOpassign(
    const ValueType& f_rhs_r,
    const OperationFunctor& f_functor_r)
{
    intern::do_operation_matrix_fixed_mn<RowValue, ColValue>(*this,
        intern::TScalarWrapper<ValueType>(f_rhs_r), *this,
        f_functor_r);
}


template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
inline
void vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::identity()
{
    intern::copy_matrix_fixed_mn<RowValue, ColValue>(
        intern::TIdentityFunctor<ValueType>(), *this);
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
inline
void vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::zero()
{
    intern::copy_matrix_fixed_mn<RowValue, ColValue>(
        intern::TScalarWrapper<ValueType>(static_cast<ValueType>(0)),
        *this);
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
template<class T>
inline
void vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::set(T f_value)
{
    intern::copy_matrix_fixed_mn<RowValue, ColValue>(
        intern::TScalarWrapper<ValueType>(static_cast<ValueType>(f_value)),
        *this);
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
inline
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>&
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::operator+= (
    const TMatrixMN<ValueType, RowValue, ColValue>& f_rhs_r)
{
    intern::do_operation_matrix_fixed_mn<RowValue, ColValue>(*this, f_rhs_r, *this,
        stlalias::plus<ValueType>());
    return *this;
}


template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
template<class OperatorType>
inline
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>&
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::operator+= (
    const intern::TExp2D<ValueType, OperatorType,
        TStaticRectangle<RowValue, ColValue> >& f_rhs_r)
{
    intern::do_operation_matrix_fixed_mn<RowValue, ColValue>(*this, f_rhs_r, *this,
        stlalias::plus<ValueType>());

    return *this;
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
inline
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>&
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::operator-= (
    const TMatrixMN<ValueType, RowValue, ColValue>& f_rhs_r)
{
    intern::do_operation_matrix_fixed_mn<RowValue, ColValue>(*this, f_rhs_r, *this,
        stlalias::minus<ValueType>());
    return *this;
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
template<class OperatorType>
inline
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>&
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::operator-= (
    const intern::TExp2D<ValueType, OperatorType,
        TStaticRectangle<RowValue, ColValue> >& f_rhs_r)
{
    intern::do_operation_matrix_fixed_mn<RowValue, ColValue>(*this, f_rhs_r, *this,
        stlalias::minus<ValueType>());
    return *this;
}


template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
inline
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>&
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::operator*= (
    const ValueType& f_value_r)
{
    intern::do_operation_matrix_fixed_mn<RowValue, ColValue>(*this,
        intern::TScalarWrapper<ValueType>(f_value_r), *this,
        stlalias::multiplies<ValueType>());
    return *this;
}

template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
inline
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>&
vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::operator/= (
    const ValueType& f_value_r)
{
    VFC_REQUIRE( !vfc::isZero(f_value_r) );
    intern::do_operation_matrix_fixed_mn<RowValue, ColValue>(*this,
        intern::TScalarWrapper<ValueType>(f_value_r), *this,
        stlalias::divides<ValueType>());
    return *this;
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_matrixmn.inl  $
//  Revision 1.4 2014/08/18 16:24:16MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - ...should be explicitly initialized in the copy constructor (mantis0004594)
//  Revision 1.3 2009/05/28 09:19:03MESZ Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) 
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.2 2009/05/11 07:38:44CEST Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  -Introduced new macro for the suppression of compiler warning.(mantis:2832)
//  Revision 1.1 2007/05/09 13:51:22IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
