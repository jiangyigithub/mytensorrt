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
///     $Source: vfc_linalg_vectorn.inl $
///     $Revision: 1.3 $
///     $Author: Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) $
///     $Date: 2009/05/28 09:19:02MESZ $
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
#include "vfc/linalg/vfc_linalg_algorithm1d.hpp"
#include "vfc/core/vfc_math.hpp"
#include "vfc/core/vfc_assert.hpp"  // used for VFC_ASSERT(), VFC_REQUIRE(), VFC_ENSURE()
#include "vfc/core/vfc_types.hpp"   // used for vfc::int32_t
#include <functional>

namespace vfc
{   //namespace vfc opened

    namespace linalg
    {   //namespace linalg opened

        /// Unrolls static vector for evaluation of the vector length
        template <class ValueType, class VectorType, vfc::int32_t RowValue>
        class TUnroll1DLength
        {
            public:
            /// Value Type
            typedef ValueType                       value_type;
            /// Shape Type
            typedef typename VectorType::shape_type shape_type;
            /// Size Type
            typedef vfc::int32_t                    size_type;

            /// Evaluates the vector length
            inline
            static
            value_type eval(const VectorType& f_operand_r)
            {
                return f_operand_r[RowValue-1] *
                    f_operand_r[RowValue-1] +
                    TUnroll1DLength<ValueType, VectorType,
                        RowValue - 1>:: eval(f_operand_r);
            }

        private:
            TUnroll1DLength& operator=(const TUnroll1DLength<ValueType,
                VectorType, RowValue>&);
        };

        /// TUnroll1DLength specialization
        template <class ValueType, class VectorType>
        class TUnroll1DLength<ValueType, VectorType, 1>
        {
        public:
            /// Value Type
            typedef ValueType                       value_type;
            /// Shape Type
            typedef typename VectorType::shape_type shape_type;
            /// Size Type
            typedef vfc::int32_t                    size_type;

            inline
            static value_type eval(const VectorType& f_operand_r)
            {
                return f_operand_r[0] * f_operand_r[0];
            }
        };

    }   //namespace linalg closed

}   //namespace vfc closed



template <class ValueType, vfc::int32_t RowValue>
inline
vfc::linalg::TVectorN<ValueType, RowValue>::TVectorN(size_type VFC_USE_VAR_ONLY_IN_ASSERTION(f_nbRows),
                                                     const ValueType& f_default)
{
    VFC_REQUIRE(RowValue == f_nbRows);
    intern::copy_vector_fixed_n<RowValue>(
    intern::TScalarWrapper<ValueType>(f_default), *this);
}


template <class ValueType, vfc::int32_t RowValue>
template<class OperatorType>
inline
vfc::linalg::TVectorN<ValueType, RowValue>::TVectorN(
    const intern::TExp1D<ValueType,OperatorType,
        TStaticRectangle<RowValue, 1 > >& f_param_r)
{
    operator=(f_param_r);
}

template <class ValueType, vfc::int32_t RowValue>
template <class OperatorType>
inline
const vfc::linalg::TVectorN<ValueType, RowValue>&
vfc::linalg::TVectorN<ValueType, RowValue>::operator=(
    const intern::TExp1D<ValueType, OperatorType,
        TStaticRectangle< RowValue, 1> >& f_rhs_r)
{
    intern::copy_vector_fixed_n<NB_ROWS>(f_rhs_r, *this);
    return *this;
}

template <class ValueType, vfc::int32_t RowValue>
inline
const vfc::linalg::TVectorN<ValueType, RowValue>&
vfc::linalg::TVectorN<ValueType, RowValue>::operator=(
    const TVectorN<ValueType, RowValue>& f_rhs_r)
{
    if (this != &f_rhs_r)
    {
        intern::copy_vector_fixed_n<NB_ROWS>(f_rhs_r, *this);
    }
    return *this;
}

template <class ValueType, vfc::int32_t RowValue>
inline
void vfc::linalg::TVectorN<ValueType, RowValue>::assign(
    const vfc::linalg::TVectorN<ValueType, RowValue>& f_rhs_r)
{
    TVectorN<ValueType, RowValue> temp(f_rhs_r);
    intern::copy_vector_fixed_n<RowValue>(temp, *this);
}

template <class ValueType, vfc::int32_t RowValue>
template <class OperatorType>
inline
void vfc::linalg::TVectorN<ValueType, RowValue>::assign(
    const vfc::linalg::intern::TExp1D<ValueType, OperatorType,
        vfc::linalg::TStaticRectangle<RowValue, 1> >& f_rhs_r)
{
    TVectorN<ValueType, RowValue> temp(f_rhs_r);
    intern::copy_vector_fixed_n<RowValue>(temp, *this);
}



template <class ValueType, vfc::int32_t RowValue>
inline
void vfc::linalg::TVectorN<ValueType, RowValue>::assign_to(
    vfc::linalg::TVectorN<typename vfc::linalg::TVectorN<ValueType, RowValue>::value_type,
                          RowValue >& f_rhs_r) const
{
    intern::copy_vector_fixed_n<NB_ROWS>(*this, f_rhs_r);
}

template <class ValueType, vfc::int32_t RowValue>
template <class OperationFunctor>
inline
void vfc::linalg::TVectorN<ValueType, RowValue>::opassign(
    const TVectorN<ValueType, RowValue >& f_rhs_r,
    const OperationFunctor& f_func_r)
{
    TVectorN<ValueType, RowValue> temp(f_rhs_r);
    intern::do_operation_vector_fixed_n<RowValue>(*this, temp, *this, f_func_r);
}


template <class ValueType, vfc::int32_t RowValue>
template <class OperatorType, class OperationFunctor>
inline
void vfc::linalg::TVectorN<ValueType, RowValue>::opassign(
    const intern::TExp1D<ValueType, OperatorType,
        TStaticRectangle<RowValue, 1> >& f_rhs_r,
    const OperationFunctor& f_func_r)
{
    TVectorN<ValueType, RowValue> temp(f_rhs_r);
    intern::do_operation_vector_fixed_n<RowValue>(*this, temp, *this, f_func_r);
}

template <class ValueType, vfc::int32_t RowValue>
template <class OperationFunctor>
inline
void vfc::linalg::TVectorN<ValueType, RowValue>::unaryOpassign(
    const ValueType& f_rhs_r,
    const OperationFunctor& f_func_r)
{
    intern::do_operation_vector_fixed_n<RowValue>(*this,
        intern::TScalarWrapper<ValueType>(f_rhs_r), *this,
        f_func_r);
}

template <class ValueType, vfc::int32_t RowValue>
inline
const vfc::linalg::TVectorN<ValueType, RowValue>&
vfc::linalg::TVectorN<ValueType, RowValue>::operator+=(
    const TVectorN<ValueType,RowValue>& f_rhs_r)
{
    intern::do_operation_vector_fixed_n<RowValue>(
        *this, f_rhs_r, *this, stlalias::plus<ValueType>());
    return *this;
}

template <class ValueType, vfc::int32_t RowValue>
template<class OperatorType>
inline
vfc::linalg::TVectorN<ValueType, RowValue>&
vfc::linalg::TVectorN<ValueType, RowValue>::operator+= (
    const intern::TExp1D<ValueType, OperatorType,
    TStaticRectangle<RowValue, 1> >& f_rhs_r)
{
    intern::do_operation_vector_fixed_n<RowValue>(
        *this, f_rhs_r, *this, stlalias::plus<ValueType>());
    return *this;
}

template <class ValueType, vfc::int32_t RowValue>
inline
const vfc::linalg::TVectorN<ValueType, RowValue>&
vfc::linalg::TVectorN<ValueType, RowValue>::operator-=(
    const TVectorN<ValueType, RowValue>& f_rhs_r)
{
    intern::do_operation_vector_fixed_n<RowValue>(
        *this, f_rhs_r, *this, stlalias::minus<ValueType>());
    return *this;
}

template <class ValueType, vfc::int32_t RowValue>
template<class OperatorType>
inline
vfc::linalg::TVectorN<ValueType, RowValue>&
vfc::linalg::TVectorN<ValueType, RowValue>::operator-= (
    const intern::TExp1D<ValueType, OperatorType,
        TStaticRectangle<RowValue, 1> >& f_rhs_r)
{
    intern::do_operation_vector_fixed_n<RowValue>(
        *this, f_rhs_r, *this, stlalias::minus<ValueType>());
    return *this;
}

template <class ValueType, vfc::int32_t RowValue>
inline
vfc::linalg::TVectorN<ValueType, RowValue>&
vfc::linalg::TVectorN<ValueType, RowValue>::operator*= (
    const ValueType& f_value_r)
{
    intern::do_operation_vector_fixed_n<RowValue>(*this,
        intern::TScalarWrapper<ValueType>(f_value_r),
        *this,
        stlalias::multiplies<ValueType>());
    return *this;
}

template <class ValueType, vfc::int32_t RowValue>
inline
vfc::linalg::TVectorN<ValueType, RowValue>&
vfc::linalg::TVectorN<ValueType, RowValue>::operator/= (
    const ValueType& f_value_r)
{
    VFC_REQUIRE( !vfc::isZero(f_value_r) );
    intern::do_operation_vector_fixed_n<RowValue>(*this,
        intern::TScalarWrapper<ValueType>(f_value_r),
        *this,
        stlalias::divides<ValueType>());
    return *this;
}

template <class ValueType, vfc::int32_t RowValue>
inline
typename vfc::linalg::TVectorN<ValueType, RowValue>::value_type
vfc::linalg::TVectorN<ValueType, RowValue>::getLength() const
{
    return static_cast<ValueType>(
        vfc::sqrt(
        static_cast<vfc::float64_t>(
        TUnroll1DLength<ValueType, TVectorN<ValueType, RowValue>,
            RowValue >::eval(*this))));
}

template <class ValueType, vfc::int32_t RowValue>
inline
void vfc::linalg::TVectorN<ValueType, RowValue>::normalize()
{
    operator/=(getLength());
}

template <class ValueType, vfc::int32_t RowValue>
inline
void vfc::linalg::TVectorN<ValueType, RowValue>::zero()
{
    copy_vector_n(
        intern::TScalarWrapper<ValueType>(static_cast<ValueType>(0)),
        *this, RowValue);
}

template <class ValueType, vfc::int32_t RowValue>
template<class T>
inline
void vfc::linalg::TVectorN<ValueType, RowValue>::set(const T& f_value)
{
    copy_vector_n(
        intern::TScalarWrapper<ValueType>(static_cast<ValueType>(f_value)),
        *this, RowValue);
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_vectorn.inl  $
//  Revision 1.3 2009/05/28 09:19:02MESZ Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) 
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.2 2009/05/11 07:38:45CEST Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  -Introduced new macro for the suppression of compiler warning.(mantis:2832)
//  Revision 1.1 2007/05/09 13:51:30IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
