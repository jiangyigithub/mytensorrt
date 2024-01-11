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
///     $Source: vfc_linalg_vector.inl $
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

#include "vfc/core/vfc_math.hpp"
#include "vfc/linalg/vfc_linalg_vectorn.hpp"
#include "vfc/linalg/vfc_linalg_algorithm1d.hpp"
#include <functional>

template <class ValueType, class StorageType>
inline
vfc::linalg::TVector<ValueType, StorageType>::TVector(
    const vfc::linalg::TVector<ValueType, StorageType>& f_param_r)
{
    operator=(f_param_r);
}


template <class ValueType, class StorageType>
template<class OperatorType, class ShapeType>
inline
vfc::linalg::TVector<ValueType, StorageType>::TVector(
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_param_r)
{
    operator=(f_param_r);
}

template <class ValueType, class StorageType>
template<vfc::int32_t RowValue>
inline
vfc::linalg::TVector<ValueType, StorageType>::TVector(
    const vfc::linalg::TVectorN<ValueType, RowValue>& f_param_r)
{
    operator=(f_param_r);
}

template <class ValueType, class StorageType>
inline
void vfc::linalg::TVector<ValueType, StorageType>::resize (size_type f_nbRows)
{
    m_data.resize(f_nbRows, ValueType());
    m_nbRows = f_nbRows;
}

template <class ValueType, class StorageType>
inline
void vfc::linalg::TVector<ValueType, StorageType>::resize (size_type f_nbRows,
    const value_type& f_val_r)
{
    m_data.resize(f_nbRows, f_val_r);
    m_nbRows = f_nbRows;
}

template <class ValueType, class StorageType>
inline
const vfc::linalg::TVector<ValueType, StorageType>&
vfc::linalg::TVector<ValueType, StorageType>::operator=(
    const vfc::linalg::TVector<ValueType, StorageType>& f_rhs_r)
{
    //ensure self assignment
    if (this != &f_rhs_r)
    {
        resize(f_rhs_r.getDim());
        intern::copy_vector_n(f_rhs_r, *this, m_nbRows);
    }
    return *this;
}

template <class ValueType, class StorageType>
template<vfc::int32_t RowValue>
inline
const vfc::linalg::TVector<ValueType, StorageType>&
vfc::linalg::TVector<ValueType, StorageType>::operator=(
    const vfc::linalg::TVectorN<ValueType, RowValue>& f_rhs_r)
{
    resize(f_rhs_r.getDim());
    intern::copy_vector_n(f_rhs_r, *this, m_nbRows);
    return *this;
}


template <class ValueType, class StorageType>
template <class OperatorType, class ShapeType>
inline
const vfc::linalg::TVector<ValueType, StorageType>&
vfc::linalg::TVector<ValueType, StorageType>::operator=(
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_rhs_r)
{
    resize(f_rhs_r.getDim());
    intern::copy_vector_n(f_rhs_r, *this, m_nbRows);
    return *this;
}


template <class ValueType, class StorageType>
template <vfc::int32_t RowValue>
inline
void vfc::linalg::TVector<ValueType, StorageType>::assign(
    const vfc::linalg::TVectorN<ValueType, RowValue>& f_rhs_r)
{
    TVector<ValueType, StorageType> temp(f_rhs_r);
    resize(temp.getDim());
    intern::copy_vector_n(temp, *this, m_nbRows);
}

template <class ValueType, class StorageType>
inline
void vfc::linalg::TVector<ValueType, StorageType>::assign(
    const vfc::linalg::TVector<ValueType, StorageType>& f_rhs_r)
{
    TVector<ValueType, StorageType> temp(f_rhs_r);
    resize(temp.getDim());
    intern::copy_vector_n(temp, *this, m_nbRows);
}

template <class ValueType, class StorageType>
template <class OperatorType, class ShapeType>
inline
void vfc::linalg::TVector<ValueType, StorageType>::assign(
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType,
        ShapeType>& f_rhs_r)
{
    TVector<ValueType, StorageType> temp(f_rhs_r);
    resize(temp.getDim());
    intern::copy_vector_n(temp, *this, m_nbRows);
}

template <class ValueType, class StorageType>
inline
void vfc::linalg::TVector<ValueType, StorageType>::assign_to(
    vfc::linalg::TVector<ValueType, StorageType>& f_rhs_r) const
{
    f_rhs_r.resize(getDim());
    intern::copy_vector_n(*this, f_rhs_r, m_nbRows);
}

template <class ValueType, class StorageType>
template <class OperationFunctor>
inline
void vfc::linalg::TVector<ValueType, StorageType>::opassign(
    const vfc::linalg::TVector<ValueType, StorageType>& f_rhs_r,
    const OperationFunctor& f_functor_r)
{
    TVector<ValueType, StorageType> temp(f_rhs_r);
    resize(temp.getDim());
    intern::do_operation_vector_n(*this, temp, *this, f_functor_r, m_nbRows);
}

template <class ValueType, class StorageType>
template <vfc::int32_t RowValue, class OperationFunctor>
inline
void vfc::linalg::TVector<ValueType, StorageType>::opassign(
    const vfc::linalg::TVectorN<ValueType, RowValue>& f_rhs_r,
    const OperationFunctor& f_functor_r)
{
    TVector<ValueType, StorageType> temp(f_rhs_r);
    resize(temp.getDim());
    intern::do_operation_vector_n(*this, temp, *this, f_functor_r, m_nbRows);
}

template <class ValueType, class StorageType>
template <class OperatorType, class ShapeType, class OperationFunctor>
inline
void vfc::linalg::TVector<ValueType, StorageType>::opassign(
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType,
        ShapeType>& f_rhs_r,
    const OperationFunctor& f_functor_r)
{
    TVector<ValueType, StorageType> temp(f_rhs_r);
    resize(temp.getDim());
    intern::do_operation_vector_n(*this, temp, *this, f_functor_r, m_nbRows);
}


template <class ValueType, class StorageType>
template <class OperationFunctor>
inline
void vfc::linalg::TVector<ValueType, StorageType>::unaryOpassign(
    const ValueType& f_rhs_r,
    const OperationFunctor& f_functor_r)
{
    intern::do_operation_vector_n(*this,
        intern::TScalarWrapper<ValueType>(f_rhs_r), *this,
        f_functor_r, m_nbRows);
}


template <class ValueType, class StorageType>
inline
void vfc::linalg::TVector<ValueType, StorageType>::normalize()
{
    operator/=(getLength());
}

template <class ValueType, class StorageType>
inline
void vfc::linalg::TVector<ValueType, StorageType>::zero()
{
    intern::copy_vector_n(
        intern::TScalarWrapper<ValueType>(static_cast<ValueType>(0)),
        *this, m_nbRows);
}

template <class ValueType, class StorageType>
inline
void vfc::linalg::TVector<ValueType, StorageType>::set(const ValueType& f_value_r)
{
    intern::copy_vector_n(
        intern::TScalarWrapper<ValueType>(f_value_r),
        *this, m_nbRows);
}

template <class ValueType, class StorageType>
template<class VectorType, class ShapeType>
inline
vfc::linalg::TVector<ValueType, StorageType>&
vfc::linalg::TVector<ValueType, StorageType>::operator+= (
    const vfc::linalg::TVectorBase<ValueType, VectorType, ShapeType>& f_rhs_r)
{
    VFC_REQUIRE( f_rhs_r.getDim() == m_nbRows);
    intern::do_operation_vector_n(
        *this, f_rhs_r, *this, stlalias::plus<ValueType>(), m_nbRows);
    return *this;
}


template <class ValueType, class StorageType>
template<class OperatorType, class ShapeType>
inline
vfc::linalg::TVector<ValueType, StorageType>&
vfc::linalg::TVector<ValueType, StorageType>::operator+= (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_rhs_r)
{
    VFC_REQUIRE( f_rhs_r.getDim() == m_nbRows );
    intern::do_operation_vector_n(
        *this, f_rhs_r, *this, stlalias::plus<ValueType>(), m_nbRows);
    return *this;
}

template <class ValueType, class StorageType>
template<class VectorType, class ShapeType>
inline
vfc::linalg::TVector<ValueType, StorageType>&
vfc::linalg::TVector<ValueType, StorageType>::operator-= (
    const vfc::linalg::TVectorBase<ValueType, VectorType, ShapeType>& f_rhs_r)
{
    VFC_REQUIRE( f_rhs_r.getDim() == m_nbRows );
    intern::do_operation_vector_n(
        *this, f_rhs_r, *this, stlalias::minus<ValueType>(), m_nbRows);
    return *this;
}


template <class ValueType, class StorageType>
template<class OperatorType, class ShapeType>
inline
vfc::linalg::TVector<ValueType, StorageType>&
vfc::linalg::TVector<ValueType, StorageType>::operator-= (
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType, ShapeType>& f_rhs_r)
{
    VFC_REQUIRE( f_rhs_r.getDim() == m_nbRows );
    intern::do_operation_vector_n(
        *this, f_rhs_r, *this, stlalias::minus<ValueType>(), m_nbRows);
    return *this;
}


template <class ValueType, class StorageType>
inline
vfc::linalg::TVector<ValueType, StorageType>&
vfc::linalg::TVector<ValueType, StorageType>::operator*= (
    const ValueType& f_value_r)
{
    intern::do_operation_vector_n(*this,
        intern::TScalarWrapper<ValueType>(f_value_r),
        *this,
        stlalias::multiplies<ValueType>(), m_nbRows);
    return *this;
}

template <class ValueType, class StorageType>
inline
vfc::linalg::TVector<ValueType, StorageType>&
vfc::linalg::TVector<ValueType, StorageType>::operator/= (
    const ValueType& f_value_r)
{
    VFC_REQUIRE( !vfc::isZero(f_value_r) );
    intern::do_operation_vector_n(*this,
        intern::TScalarWrapper<ValueType>(f_value_r),
        *this,
        stlalias::divides<ValueType>(), m_nbRows);
    return *this;
}

template <class ValueType, class StorageType>
inline
ValueType vfc::linalg::TVector<ValueType, StorageType>::getLength() const
{
    ValueType value = static_cast<value_type>(0);
    for (vfc::int32_t count=0; count<m_nbRows; ++count)
    {
        value += m_data[count] * m_data[count];
    }
    value = static_cast<value_type>(vfc::sqrt(static_cast<vfc::float64_t>(value)));
    return value;
}
//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_vector.inl  $
//  Revision 1.4 2009/05/28 09:19:05MESZ Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) 
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.3 2008/08/29 15:02:22CEST Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  Removal of template parameter from set functionality (Mantis :2176)
//  Revision 1.2 2008/07/21 11:06:24IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  set() function added ( Mantis : 2176 )
//  Revision 1.1 2007/05/09 13:51:26IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
