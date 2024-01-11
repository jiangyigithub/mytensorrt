//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2006 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy, use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/core
//          Synopsis:
//  Target system(s):
//       Compiler(s): VS7.1, VS8.0, CW9.3, CWembedded8.3
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: zvh2hi
//  Department:
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: include/vfc/core/vfc_fixedpoint.inl $
///     $Revision: 1.58 $
///     $Author: Muehlmann Karsten (CC/PJ-FA1) (MUK2LR) $
///     $Date: 2011/05/18 17:38:56MESZ $
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
#include "vfc/core/vfc_type_traits.hpp"     //used for TIsUnsignedArithmetic and TIsIntegral
#include <limits>

namespace vfc
{
    namespace intern
    {
        template <class ResultType, vfc::int32_t FracBitsValue, class ValueType>
        inline ResultType  numeric_cast(const TFixedPoint<FracBitsValue, ValueType>& f_value, vfc::true_t)
        {
            return static_cast<ResultType>(f_value.to_nint());
        }

        template <class ResultType, vfc::int32_t FracBitsValue, class ValueType>
        inline ResultType  internal_cast(const TFixedPoint<FracBitsValue, ValueType>& f_value, vfc::false_t)
        {
            return static_cast<ResultType>(f_value.to_int());
        }

        template <class ResultType, vfc::int32_t FracBitsValue, class ValueType>
        inline ResultType  internal_cast(const TFixedPoint<FracBitsValue, ValueType>& f_value, vfc::true_t)
        {
            return TFixedPoint<FracBitsValue, ValueType>(f_value);
        }

        template <class ResultType, vfc::int32_t FracBitsValue, class ValueType>
        inline ResultType  numeric_cast(const TFixedPoint<FracBitsValue, ValueType>& f_value, vfc::false_t)
        {
            return internal_cast<ResultType>(f_value,
                typename TInt2Boolean<TIsSameType<ResultType, TFixedPoint<FracBitsValue, ValueType> >::value>::type());
        }

        template <bool IsLeftShiftValue, vfc::int32_t ShiftValue>
        template <class ValueType>
        inline ValueType TDirectionalShift<IsLeftShiftValue, ShiftValue>::shift (const ValueType& f_value)
        {
            VFC_REQUIRE( vfc::abs(f_value) <
                            (stlalias::numeric_limits<ValueType>::max() >> ShiftValue) );
            return f_value << ShiftValue;
        }

        template <vfc::int32_t ShiftValue>
        template <class ValueType>
        inline ValueType TDirectionalShift<false, ShiftValue>::shift (const ValueType& f_value)
        {
            return f_value >> ShiftValue;
        }

        template <vfc::int32_t SrcFracBits, vfc::int32_t DestFracBits>
        template <class ValueType>
        inline ValueType TConvertFracBits<SrcFracBits, DestFracBits>::shift (const ValueType& f_value)
        {
            return shift_type::shift(f_value);
        }

    } // end namespace intern

} // end namespace vfc



#ifndef VFC_NO_FLOAT

template <vfc::int32_t FracBitsValue, class ValueType>
const vfc::float32_t
vfc::TFixedPoint<FracBitsValue, ValueType>::int2floatMultiplier_f32 = 1.f / static_cast<vfc::float32_t>(1 << FracBitsValue);

template <vfc::int32_t FracBitsValue, class ValueType>
const vfc::float32_t
vfc::TFixedPoint<FracBitsValue, ValueType>::float2intMultiplier_f32 = static_cast<vfc::float32_t>(1 << FracBitsValue);

template <vfc::int32_t FracBitsValue, class ValueType>
const vfc::float64_t
vfc::TFixedPoint<FracBitsValue, ValueType>::int2floatMultiplier_f64 = 1.f / static_cast<vfc::float64_t>(1 << FracBitsValue);

template <vfc::int32_t FracBitsValue, class ValueType>
const vfc::float64_t
vfc::TFixedPoint<FracBitsValue, ValueType>::float2intMultiplier_f64 = static_cast<vfc::float64_t>(1 << FracBitsValue);
#endif




template <vfc::int32_t FracBitsValue, class ValueType>
const vfc::TFixedPoint<FracBitsValue, ValueType>
vfc::TFixedPoint<FracBitsValue, ValueType>::zero = vfc::TFixedPoint<FracBitsValue, ValueType>(0);


template <vfc::int32_t FracBitsValue, class ValueType>
const vfc::TFixedPoint<FracBitsValue, ValueType>
vfc::TFixedPoint<FracBitsValue, ValueType>::one = vfc::TFixedPoint<FracBitsValue, ValueType>(1);


// minimum fixedpoint resolution
template <vfc::int32_t FracBitsValue, class ValueType>
const vfc::TFixedPoint<FracBitsValue, ValueType>
vfc::TFixedPoint<FracBitsValue, ValueType>::epsilon = vfc::TFixedPoint<FracBitsValue, ValueType>
(1, typename vfc::TFixedPoint<FracBitsValue, ValueType>::CNoShift());


template <vfc::int32_t FracBitsValue, class ValueType>
const vfc::TFixedPoint<FracBitsValue, ValueType>
vfc::TFixedPoint<FracBitsValue, ValueType>::max =
                vfc::TFixedPoint<FracBitsValue, ValueType>
                        (stlalias::numeric_limits<ValueType>::max(),
                        typename vfc::TFixedPoint<FracBitsValue, ValueType>::CNoShift());


#ifndef    VFC_NO_FLOAT
template <vfc::int32_t FracBitsValue, class ValueType>
inline void vfc::TFixedPoint<FracBitsValue, ValueType>::init(const vfc::float32_t& f_value_f32, vfc::false_t)
{
	m_fpvalue = static_cast<value_type>(f_value_f32 * float2intMultiplier_f32);
}

// (Msg Disable 474: This literal is of the non standard type 'long long'.)
// (Msg Disable 476: This literal is of the non standard type 'unsigned long long'.)
// PRQA S 474, 476  ++
template <vfc::int32_t FracBitsValue, class ValueType>
inline void vfc::TFixedPoint<FracBitsValue, ValueType>::init(const vfc::float64_t& f_value_f64, vfc::false_t)
{
	m_fpvalue = static_cast<value_type>(f_value_f64 * float2intMultiplier_f64);
}
#else

//dummy function to avoid compilation error
template <vfc::int32_t FracBitsValue, class ValueType>
template <typename InputType>
inline void vfc::TFixedPoint<FracBitsValue, ValueType>::init(const InputType& f_value, vfc::false_t)
{
    //intentionally left blank
}
#endif

template <vfc::int32_t FracBitsValue, class ValueType>
template <vfc::int32_t OtherFracBitsValue, class OtherTypeValue>
inline vfc::TFixedPoint<FracBitsValue, ValueType>::TFixedPoint (const TFixedPoint<OtherFracBitsValue, OtherTypeValue>&
                                                                                                              f_value)
:m_fpvalue(0)
{
    init(f_value);
}

//! init for fixedpoint type with another fracbits count
template <vfc::int32_t FracBitsValue, class ValueType>
template <vfc::int32_t OtherFracBitsValue, class OtherTypeValue>
inline void vfc::TFixedPoint<FracBitsValue, ValueType>::init(const TFixedPoint<OtherFracBitsValue, OtherTypeValue>&
                                                                                                            f_value)
{
    m_fpvalue = static_cast<ValueType>
        (vfc::intern::TConvertFracBits<OtherFracBitsValue, FracBitsValue>::shift
        (static_cast<typename vfc::TIf<(sizeof(ValueType) >= sizeof(OtherTypeValue)), ValueType, OtherTypeValue>::type>
        (f_value.fixedpoint())));

    VFC_REQUIRE(    ((vfc::isZero(f_value.fixedpoint()))  &&
                     (vfc::isZero((*this).fixedpoint() + f_value.fixedpoint()))) ||
                    ((!vfc::isZero(f_value.fixedpoint()))  &&
                     (!vfc::isZero((*this).fixedpoint() + f_value.fixedpoint()))));
}


template <vfc::int32_t FracBitsValue, class ValueType>
inline void vfc::TFixedPoint<FracBitsValue, ValueType>::setFixedpoint(const value_type& f_value)
{
     m_fpvalue = f_value;
}

//! deprecated, use vfc::fixedPointSumOverflowed instead (mind the reversed logic though!)
template <vfc::int32_t FracBitsValue, class ValueType>
inline bool vfc::TFixedPoint<FracBitsValue, ValueType >::isAdditionSafe(
                                                    const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs) const
{
    vfc::TFixedPoint<FracBitsValue, ValueType> l_sum;
    return !vfc::fixedPointSumOverflowed(*this, f_rhs, l_sum);
}


template <vfc::int32_t FracBitsValue, class ValueType>
inline bool vfc::TFixedPoint<FracBitsValue, ValueType>::isMultiplicationSafe(
                                                        const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs) const
{
    const vfc::int64_t res64_i64 = (static_cast<mult_op_type>(f_rhs.fixedpoint()) *
                                    static_cast<mult_op_type>(this->fixedpoint()))
                                            >> static_cast<vfc::int32_t>(SHIFT);

    if ((static_cast<mult_op_type>(stlalias::numeric_limits<value_type>::max()) < res64_i64) ||
        (static_cast<mult_op_type>(stlalias::numeric_limits<value_type>::max()) < -res64_i64))
    {
        return false;
    }
    return true;
}


template <vfc::int32_t FracBitsValue, class ValueType>
template <vfc::int32_t otherFracBitsValue, class otherValueType>
inline bool vfc::TFixedPoint<FracBitsValue, ValueType>::isDivisionSafe(
                                               const vfc::TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs) const
{
    if (vfc::isZero(f_rhs))
    {
        return false;
    }

    const div_op_type tempRhs  = static_cast<div_op_type>
       (vfc::intern::TConvertFracBits<otherFracBitsValue, SHIFT>::shift(static_cast<div_op_type>(f_rhs.fixedpoint())));
    const div_op_type tempLhs  = static_cast<div_op_type>(static_cast<div_op_type>
                                    ((*this).m_fpvalue)) << static_cast<vfc::int32_t>(SHIFT);
    if (vfc::isZero(tempRhs))
    {
        return false;
    }
    const div_op_type tempResult = tempLhs / tempRhs;

    return ((static_cast<div_op_type>((stlalias::numeric_limits<value_type>::max())) >= tempResult) &&
            (static_cast<div_op_type>((stlalias::numeric_limits<value_type>::max())) >= -tempResult));
}

#ifndef    VFC_NO_FLOAT
/// casts fixpoint to single precision floating point, for ieee floating point definition see
// http://www.psc.edu/general/software/packages/ieee/ieee.html
// http://en.wikipedia.org/wiki/Floating_point
// http://www.h-schmidt.net/FloatApplet/IEEE754de.html
// S EEEEEEEE FFFFFFFFFFFFFFFFFFFFFFF   ( S: sign, E: exponent bits, F: fraction )
// 0 1      8 9                    31
template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::float32_t  vfc::TFixedPoint<FracBitsValue, ValueType>::to_float32    (void) const
{
    // **** TBD **** replace by bit operations without floatingpoint division
    static const vfc::int32_t FLOAT32MANTISSABITS  = 23L;
    static const vfc::int32_t FLOAT32SIGNBIT       = 31L;
    static const vfc::int32_t FLOAT32EXPSUBTRAHENT = 127L;

    vfc::int32_t sign_i32 = ((m_fpvalue & static_cast<vfc::int32_t>(0x80000000L)) >> FLOAT32SIGNBIT) & 0x1L;
    vfc::int32_t exponent_i32 = 0;
    vfc::int32_t mantissa_i32 = 0;

    ValueType m_fpvalueTemp = (m_fpvalue ) ; //<< SHIFT);
    if((stlalias::numeric_limits<ValueType>::min()) == m_fpvalueTemp)
    {
            return (static_cast<vfc::float32_t>(m_fpvalueTemp) /
                    static_cast<vfc::float32_t>(FAC));
    }

    vfc::int32_t temp_i32 = vfc::abs(m_fpvalueTemp);

    if(temp_i32 == 0)
    {
        exponent_i32 = -127;
    }
    else if((temp_i32 =  (vfc::abs(m_fpvalueTemp) >> static_cast<vfc::int32_t>(SHIFT)))  > 0)
    {
        while((temp_i32 = temp_i32 >> 1) != 0)
        {
            if(exponent_i32 > 32)
            {
                break;
            }
            exponent_i32++;
        }
    }
    else
    {
        vfc::int32_t MASK = -1;
        MASK ^= ((MASK & static_cast<vfc::int32_t>(0x80000000L)) >> (31 - SHIFT));
        temp_i32 = vfc::abs(m_fpvalueTemp) & MASK;
        while((temp_i32 = temp_i32 >> 1) != 0)
        {
            if(exponent_i32 > 32)
            {
                break;
            }
            exponent_i32++;
        }
        exponent_i32 = exponent_i32 - SHIFT;
    }

    vfc::int32_t shiftValue_i32 = (FLOAT32MANTISSABITS - SHIFT - exponent_i32);
    vfc::float32_t f_value_f32 = 0.0;
    if(vfc::isPositive(shiftValue_i32))
    {
        mantissa_i32 = vfc::abs(m_fpvalueTemp) << shiftValue_i32;
    }
    else
    {
        mantissa_i32 = vfc::abs(m_fpvalueTemp) >> (-shiftValue_i32);
    }

    (*(vfc::int32_t*)&f_value_f32) |= ((sign_i32 << FLOAT32SIGNBIT) & static_cast<vfc::int32_t>(0x80000000L));
    (*(vfc::int32_t*)&f_value_f32) |= (((exponent_i32 + FLOAT32EXPSUBTRAHENT) << FLOAT32MANTISSABITS) &
                                                                               static_cast<vfc::int32_t>(0x7F800000L));
    (*(vfc::int32_t*)&f_value_f32) |= (mantissa_i32 & (0x007FFFFFL | (0x00800000L * (
                                                                        (FLOAT32EXPSUBTRAHENT + exponent_i32) == 0))));

    vfc::int32_t checkValue_i32 = (((1 << (VALIDBITS - SHIFT)) - 1) * ((sign_i32==0)?1:-1)) << SHIFT;
    if(m_fpvalueTemp == checkValue_i32)
    {
        f_value_f32 = static_cast<vfc::float32_t>(checkValue_i32 >> SHIFT);
    }

    return f_value_f32;
}


/// casts fixpoint to double precision floating point
// S EEEEEEEEEEE FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF ( S: sign, E: exponent bits, F: fraction )
// 0 1        11 12                                                63
template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::float64_t  vfc::TFixedPoint<FracBitsValue, ValueType>::to_float64    (void) const
{
    // **** TBD **** replace by bit operations without floatingpoint division
    static const vfc::int64_t FLOAT64MANTISSABITS  = 52LL;
    static const vfc::int64_t FLOAT64SIGNBIT       = 63LL;
    static const vfc::int64_t FLOAT64EXPSUBTRAHENT = 1023LL;

    vfc::int64_t sign_i64 = ((m_fpvalue & static_cast<vfc::int64_t>(0x8000000000000000LL)) >> FLOAT64SIGNBIT) & 0x1LL;
    vfc::int64_t exponent_i64 = 0;
    vfc::int64_t mantissa_i64 = 0;

    ValueType m_fpvalueTemp = m_fpvalue ; // = (m_fpvalue << SHIFT);
    if((stlalias::numeric_limits<ValueType>::min()) == m_fpvalueTemp)
    {
            return (static_cast<vfc::float64_t>(m_fpvalueTemp) /
                    static_cast<vfc::float64_t>(FAC));
    }

    vfc::int64_t temp_i64 = 0LL;

    if(m_fpvalueTemp < 0LL)
    {
        temp_i64 = (-1LL * m_fpvalueTemp);
    }
    else
    {
        temp_i64 = m_fpvalueTemp;
    }

    if(temp_i64 == 0)
    {
        exponent_i64 = -1023;
    }
    else if((temp_i64 = ((m_fpvalueTemp<0)?(m_fpvalueTemp * -1LL):m_fpvalueTemp) >> SHIFT)  > 0)
    {
        while((temp_i64 = temp_i64 >> 1) != 0)
        {
            if(exponent_i64 > 64)
            {
                break;
            }
            exponent_i64++;
        }
    }
    else
    {
        vfc::int64_t MASK = -1LL;
        MASK ^= ((MASK & static_cast<vfc::int64_t>(0x8000000000000000LL)) >> (63LL - SHIFT));
        temp_i64 = ((m_fpvalueTemp<0)?(m_fpvalueTemp * -1LL):m_fpvalueTemp) & MASK;
        while((temp_i64 = temp_i64 >> 1 ) != 0)
        {
            if(exponent_i64 > 64)
            {
                break;
            }
            exponent_i64++;
        }
        exponent_i64 = exponent_i64 - SHIFT;
    }

    vfc::int64_t shiftValue_i64 = (FLOAT64MANTISSABITS - SHIFT - exponent_i64);
    vfc::float64_t f_value_f64 = 0.0f;

    if(vfc::isPositive(shiftValue_i64))
    {
        mantissa_i64 = ((m_fpvalueTemp<0)?(m_fpvalueTemp*-1LL):m_fpvalueTemp) << shiftValue_i64;
    }
    else
    {
        mantissa_i64 = ((m_fpvalueTemp<0)?(m_fpvalueTemp*-1LL):m_fpvalueTemp) >> (-shiftValue_i64);
    }

    (*(vfc::int64_t*)&f_value_f64) |= ((sign_i64 << FLOAT64SIGNBIT) & static_cast<vfc::int64_t>(0x8000000000000000LL));
    (*(vfc::int64_t*)&f_value_f64) |= (((exponent_i64 + FLOAT64EXPSUBTRAHENT) << FLOAT64MANTISSABITS) &
                                                                      static_cast<vfc::int64_t>(0x7FF0000000000000LL));
    (*(vfc::int64_t*)&f_value_f64) |= (mantissa_i64 & (0x000FFFFFFFFFFFFFLL | (static_cast<vfc::int64_t>
                                              (0x8000000000000000LL) * ((FLOAT64EXPSUBTRAHENT + exponent_i64) == 0))));

    //vfc::int64_t checkValue = (((1 << (VALIDBITS - SHIFT)) - 1) * (sign_i64==0?1:-1)) << SHIFT;
    //if(m_fpvalueTemp == checkValue)
    //{
    //  f_value_f64 = (vfc::float64_t)(checkValue >> SHIFT);
    //}

    return f_value_f64;
}
#endif

// PRQA S 474, 476  --
// (Msg Enable 474: This literal is of the non standard type 'long long'.)
// (Msg Enable 476: This literal is of the non standard type 'unsigned long long'.)

////////###

template <vfc::int32_t FracBitsValue, class ValueType>
inline bool vfc::TFixedPoint<FracBitsValue, ValueType>::isLeftShiftSafe(const vfc::int32_t f_shift_i32) const
{
        return(((stlalias::numeric_limits<ValueType>::min()) != m_fpvalue) &&
               (vfc::abs(m_fpvalue) < (stlalias::numeric_limits<value_type>::max() >> f_shift_i32) ));
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::int32_t vfc::TFixedPoint<FracBitsValue, ValueType>::to_int(void) const
{
    return ((m_fpvalue >= static_cast<value_type>(0)) || ((stlalias::numeric_limits<ValueType>::min()) == m_fpvalue))  ?
                                                          ( m_fpvalue  >> static_cast<vfc::int32_t>(SHIFT)) :
                                                        -((-m_fpvalue) >> static_cast<vfc::int32_t>(SHIFT));
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::int32_t vfc::TFixedPoint<FracBitsValue, ValueType>::to_floor(void) const
{
    return ((m_fpvalue >= static_cast<value_type>(0))) ?
                             static_cast<vfc::int32_t>(m_fpvalue >>static_cast<vfc::int32_t>(SHIFT)) :
                            ((stlalias::numeric_limits<ValueType>::min()) != m_fpvalue) ?
                                static_cast<vfc::int32_t>(-((-m_fpvalue - 1) >> static_cast<vfc::int32_t>(SHIFT)) - 1):
                                static_cast<vfc::int32_t>(((m_fpvalue + 1) >> static_cast<vfc::int32_t>(SHIFT)) - 1);
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::int32_t vfc::TFixedPoint<FracBitsValue, ValueType>::to_ceil(void) const
{
    return ((stlalias::numeric_limits<ValueType>::min()) != m_fpvalue) ?
                    ((m_fpvalue - 1) >> static_cast<vfc::int32_t>(SHIFT)) +1 :
                     (m_fpvalue >> static_cast<vfc::int32_t>(SHIFT)) ;

}


template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::int32_t vfc::TFixedPoint<FracBitsValue, ValueType>::to_nint(void) const
{

    // +ROUND is equivalent to +0.5 in decimal
    return ((m_fpvalue >= static_cast<value_type>(0)) ?
                        //in case of positive number
                        (((m_fpvalue <= stlalias::numeric_limits<ValueType>::max() - ROUND )) ?
                                     //in case number is less then max
                                     ((m_fpvalue + ROUND) >> static_cast<vfc::int32_t>(SHIFT)) :
                                     //in case number is greater then or equal to max
                                     (MAXINTVAL+1)):
                        //in case of negative number
                        ((m_fpvalue >= stlalias::numeric_limits<ValueType>::min() + ROUND ) ?
                                     //in case number is greater then min
                                     (-((-m_fpvalue + ROUND) >> static_cast<vfc::int32_t>(SHIFT))):
                                     //in case number is less then or equal to min
                                     (-MAXINTVAL-1)));

}

template <vfc::int32_t FracBitsValue, class ValueType>
inline ValueType vfc::TFixedPoint<FracBitsValue, ValueType>::fixedpoint(void) const
{
    return m_fpvalue;
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>&
            vfc::TFixedPoint<FracBitsValue, ValueType>::operator=(const TFixedPoint& f_rhs)
{
    if(this != &f_rhs)
    {
        m_fpvalue = f_rhs.m_fpvalue;
    }
    return *this;
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>&
            vfc::TFixedPoint<FracBitsValue, ValueType>::operator+=(const TFixedPoint& f_rhs)
{
    VFC_REQUIRE(this->isAdditionSafe(f_rhs));
    m_fpvalue += f_rhs.m_fpvalue;
    return *this;
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>&
            vfc::TFixedPoint<FracBitsValue, ValueType>::operator-=(const TFixedPoint& f_rhs)
{
    VFC_REQUIRE(this->isAdditionSafe(-f_rhs));
    m_fpvalue -= f_rhs.m_fpvalue;
    return *this;
}


template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>&
            vfc::TFixedPoint<FracBitsValue, ValueType>::operator/=
                                (const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs)
{
    div_op_type f_op1 = static_cast<div_op_type>(m_fpvalue);
    div_op_type f_op2 = static_cast<div_op_type>(f_rhs.m_fpvalue);

    VFC_REQUIRE(this->isDivisionSafe(f_rhs));
    m_fpvalue = static_cast<ValueType>(((f_op1 << static_cast<vfc::int32_t>(SHIFT)) / f_op2)) ;
    return *this ;
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>&
            vfc::TFixedPoint<FracBitsValue, ValueType>::operator*=
                                (const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs)
{

    mult_op_type f_op1 =    static_cast<mult_op_type>(m_fpvalue);
    mult_op_type f_op2 =    static_cast<mult_op_type>(f_rhs.m_fpvalue);

    VFC_REQUIRE(this->isMultiplicationSafe(f_rhs));
    m_fpvalue = static_cast<value_type>((f_op1 * f_op2) >> static_cast<vfc::int32_t>(SHIFT));

    return *this;
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>
            vfc::TFixedPoint<FracBitsValue, ValueType>::operator-   (void) const
{
    return TFixedPoint(-m_fpvalue, CNoShift());
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline
const
vfc::TFixedPoint<FracBitsValue, ValueType>
            vfc::TFixedPoint<FracBitsValue, ValueType>::operator>>  (const vfc::int32_t f_shift_i32) const
{
    return TFixedPoint((m_fpvalue >> f_shift_i32), CNoShift());
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline
const
vfc::TFixedPoint<FracBitsValue, ValueType>
            vfc::TFixedPoint<FracBitsValue, ValueType>::operator<<  (const vfc::int32_t f_shift_i32) const
{
    VFC_REQUIRE(this->isLeftShiftSafe(f_shift_i32));
    return TFixedPoint((m_fpvalue << f_shift_i32), CNoShift());
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>&
            vfc::TFixedPoint<FracBitsValue, ValueType>::operator++  (void)
{
    VFC_REQUIRE(to_floor() < MAXINTVAL);
    m_fpvalue += FAC;
    return *this;
}

// (Msg Disable 2427 : Direct use of fundamental type.)
// PRQA S 2427 ++
template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>
            vfc::TFixedPoint<FracBitsValue, ValueType>::operator++  (int)
{
    VFC_REQUIRE(to_floor() < MAXINTVAL);
    TFixedPoint temp(*this);
    this->operator++();
    return temp;
}
// PRQA S 2427 --
// (Msg Enable 2427 : Direct use of fundamental type.)

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>&
            vfc::TFixedPoint<FracBitsValue, ValueType>::operator--  (void)
{
    VFC_REQUIRE(to_ceil() > -MAXINTVAL);
    m_fpvalue -= FAC;
    return *this;
}

// (Msg Disable 2427 : Direct use of fundamental type.)
// PRQA S 2427 ++
template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>
        vfc::TFixedPoint<FracBitsValue, ValueType>::operator--  (int)
{
    VFC_REQUIRE(to_ceil() > -MAXINTVAL);
    TFixedPoint temp(*this);
    this->operator--();
    return temp;
}
// PRQA S 2427 --
// (Msg Enable 2427 : Direct use of fundamental type.)

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>
            vfc::TFixedPoint<FracBitsValue, ValueType>::add(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_op1,
                                                            const vfc::TFixedPoint<FracBitsValue, ValueType>& f_op2)
{
    VFC_REQUIRE(f_op1.isAdditionSafe(f_op2));
    return TFixedPoint((f_op1.m_fpvalue + f_op2.m_fpvalue), CNoShift());
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>
            vfc::TFixedPoint<FracBitsValue, ValueType>::subtract (const TFixedPoint<FracBitsValue, ValueType>& f_op1,
                                                                  const TFixedPoint<FracBitsValue, ValueType>& f_op2)
{
    VFC_REQUIRE(f_op1.isAdditionSafe(-f_op2));
    return TFixedPoint((f_op1.m_fpvalue - f_op2.m_fpvalue),  CNoShift());
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>
            vfc::TFixedPoint<FracBitsValue, ValueType>::switch_multiply(
                                                        const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs,
                                                        vfc::true_t) const
{
    mult_op_type f_op1=  static_cast<mult_op_type>((*this).m_fpvalue);
    mult_op_type f_op2=  static_cast<mult_op_type>(f_rhs.fixedpoint());

    VFC_REQUIRE(this->isMultiplicationSafe(f_rhs));
    return TFixedPoint(static_cast<value_type>((f_op1 * f_op2) >> static_cast<vfc::int32_t>(SHIFT)),
            CNoShift());
}

template <vfc::int32_t FracBitsValue, class ValueType>
template <vfc::int32_t otherFracBitsValue, class otherValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>
            vfc::TFixedPoint<FracBitsValue, ValueType>::switch_multiply(
                                                        const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs,
                                                        vfc::false_t) const
{
    const mult_op_type temp = vfc::intern::TConvertFracBits<otherFracBitsValue, SHIFT>::shift(f_rhs.fixedpoint());

    mult_op_type f_op1= static_cast<mult_op_type>((*this).m_fpvalue);
    mult_op_type f_op2= temp;

    VFC_REQUIRE(this->isMultiplicationSafe(vfc::TFixedPoint<FracBitsValue, ValueType>(f_rhs)));
    // correction of scaling
    return TFixedPoint(static_cast<value_type>((f_op1 * f_op2) >> static_cast<vfc::int32_t>(SHIFT)),CNoShift());
}

template <vfc::int32_t FracBitsValue, class ValueType>
template <vfc::int32_t otherFracBitsValue, class otherValueType>
inline
const vfc::TFixedPoint<FracBitsValue, ValueType>
vfc::TFixedPoint<FracBitsValue, ValueType>::operator* (
                                                    const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs) const
{
    return switch_multiply(f_rhs, typename vfc::TInt2Boolean< vfc::TIsSameType< TFixedPoint<FracBitsValue, ValueType>,
                                                                 TFixedPoint<otherFracBitsValue, otherValueType>
                                                               >::value >::type());
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>
            vfc::TFixedPoint<FracBitsValue, ValueType>::switch_divide(
                                                        const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs,
                                                        vfc::true_t) const
{
    div_op_type f_op1 = static_cast<div_op_type>((*this).m_fpvalue);
    div_op_type f_op2 = static_cast<div_op_type>(f_rhs.fixedpoint());

    VFC_REQUIRE(this->isDivisionSafe(f_rhs));
    return TFixedPoint(static_cast<ValueType>(((f_op1 << static_cast<vfc::int32_t>(SHIFT)) / f_op2)),CNoShift());
}

template <vfc::int32_t FracBitsValue, class ValueType>
template<vfc::int32_t otherFracBitsValue, class otherValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>
            vfc::TFixedPoint<FracBitsValue, ValueType>::switch_divide(
                                                        const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs,
                                                        vfc::false_t) const
{
    const div_op_type temp = vfc::intern::TConvertFracBits<otherFracBitsValue, SHIFT>::shift(
                                                            static_cast<div_op_type>(f_rhs.fixedpoint()));
    div_op_type f_op1 = static_cast<div_op_type>((*this).m_fpvalue);
    div_op_type f_op2 = temp;
    VFC_REQUIRE(this->isDivisionSafe(f_rhs));

    return TFixedPoint(static_cast<ValueType>(((f_op1 << static_cast<vfc::int32_t>(SHIFT)) / f_op2)),CNoShift());
}

template <vfc::int32_t FracBitsValue, class ValueType>
template <vfc::int32_t otherFracBitsValue, class otherValueType>
inline
const vfc::TFixedPoint<FracBitsValue, ValueType>
vfc::TFixedPoint<FracBitsValue, ValueType>::operator/ (
                                            const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs) const
{
    return switch_divide(f_rhs, typename vfc::TInt2Boolean< vfc::TIsSameType< TFixedPoint<FracBitsValue, ValueType>,
                                                                 TFixedPoint<otherFracBitsValue, otherValueType>
                                                               >::value >::type());
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>
            vfc::TFixedPoint<FracBitsValue, ValueType>::switch_add(
                                                        const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs,
                                                        vfc::true_t) const
{
    return add((*this), f_rhs);
}

template <vfc::int32_t FracBitsValue, class ValueType>
template<vfc::int32_t otherFracBitsValue, class otherValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>
            vfc::TFixedPoint<FracBitsValue, ValueType>::switch_add(
                                                        const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs,
                                                        vfc::false_t) const
{
    TFixedPoint temp(vfc::intern::TConvertFracBits<otherFracBitsValue, SHIFT>::shift(f_rhs.fixedpoint()),  CNoShift());
    return add((*this), temp);
}

template <vfc::int32_t FracBitsValue, class ValueType>
template<vfc::int32_t otherFracBitsValue, class otherValueType>
inline
const vfc::TFixedPoint<FracBitsValue, ValueType>
vfc::TFixedPoint<FracBitsValue, ValueType>::operator+ (
                                            const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs) const
{
    return switch_add(f_rhs, typename vfc::TInt2Boolean< vfc::TIsSameType< TFixedPoint<FracBitsValue, ValueType>,
                                                                 TFixedPoint<otherFracBitsValue, otherValueType>
                                                               >::value >::type());
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>
            vfc::TFixedPoint<FracBitsValue, ValueType>::switch_subtract(
                                                        const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs,
                                                        vfc::true_t) const
{
    return subtract((*this), f_rhs);
}

template <vfc::int32_t FracBitsValue, class ValueType>
template<vfc::int32_t otherFracBitsValue, class otherValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>
            vfc::TFixedPoint<FracBitsValue, ValueType>::switch_subtract(
                                                     const vfc::TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs,
                                                     vfc::false_t) const
{
    TFixedPoint temp(vfc::intern::TConvertFracBits<otherFracBitsValue, SHIFT>::shift(f_rhs.fixedpoint()),  CNoShift());
    return subtract((*this), temp);
}

template <vfc::int32_t FracBitsValue, class ValueType>
template<vfc::int32_t otherFracBitsValue, class otherValueType>
inline
const vfc::TFixedPoint<FracBitsValue, ValueType>
vfc::TFixedPoint<FracBitsValue, ValueType>::operator- (
                                            const vfc::TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs) const
{
    return switch_subtract(f_rhs, typename vfc::TInt2Boolean< vfc::TIsSameType< TFixedPoint<FracBitsValue, ValueType>,
                                                                 TFixedPoint<otherFracBitsValue, otherValueType>
                                                               >::value >::type());
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline bool vfc::TFixedPoint<FracBitsValue, ValueType>::operator ==
                (const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs) const
{
    return ((*this).m_fpvalue == f_rhs.m_fpvalue);
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline bool vfc::TFixedPoint<FracBitsValue, ValueType>::operator !=
                (const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs) const
{
    return !((*this) == f_rhs);
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline bool vfc::TFixedPoint<FracBitsValue, ValueType>::operator <
                (const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs) const
{
    return ((*this).m_fpvalue < f_rhs.m_fpvalue);
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline bool vfc::TFixedPoint<FracBitsValue, ValueType>::operator <=
                (const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs) const
{
    return ((*this) == f_rhs) || ((*this) < f_rhs);
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline bool vfc::TFixedPoint<FracBitsValue, ValueType>::operator >
                (const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs) const
{
    return !((*this) <= f_rhs);
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline bool vfc::TFixedPoint<FracBitsValue, ValueType>::operator >=
            (const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs) const
{
    return !((*this) < f_rhs);
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>::TFixedPoint (void)
:   m_fpvalue(static_cast<value_type>(0))
{
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>::TFixedPoint
            (const value_type&   f_value_fp, CNoShift )
:   m_fpvalue(f_value_fp)
{
    //intentionally left blank
}

template <vfc::int32_t FracBitsValue, class ValueType>
template <vfc::int32_t OtherFracBitsValue, class OtherTypeValue>
inline vfc::TFixedPoint<FracBitsValue, ValueType>::TFixedPoint
            (const TFixedPoint<OtherFracBitsValue, OtherTypeValue>& f_value, CNoShift)
:   m_fpvalue(f_value.fixedpoint())
{
    VFC_ENSURE(((f_value.fixedpoint()) >= stlalias::numeric_limits<value_type>::min()) &&
        (f_value.fixedpoint() <= stlalias::numeric_limits<value_type>::max()));
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>::TFixedPoint
            (const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
:   m_fpvalue(f_value.m_fpvalue)
{
    //intentionally left blank
}

template <vfc::int32_t FracBitsValue, class ValueType>
template <typename InputType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>::TFixedPoint(const InputType& f_value)
:m_fpvalue(0)
{

#	ifdef VFC_NO_FLOAT
    VFC_STATIC_ASSERT(vfc::TIsIntegral<InputType>::value);
#	endif

    init((typename vfc::TRemoveCV<InputType>::type&)(f_value), // cast away const and/or volatile
        typename TInt2Boolean<TIsIntegral<InputType>::value>::type()); // chose either integral or floating init
}

template <vfc::int32_t FracBitsValue, class ValueType>
template <typename InputType>
inline void vfc::TFixedPoint<FracBitsValue, ValueType>::init(const InputType& f_value, vfc::true_t)
{

    VFC_REQUIRE(vfc::isPositive(f_value) ? (f_value <= (MAXINTVAL)) :
                                           (f_value >= (-MAXINTVAL - 1)));

    m_fpvalue = static_cast<value_type>(f_value) << static_cast<vfc::int32_t>(SHIFT);
}


namespace vfc
{
    template <vfc::int32_t FracBitsValue, class ValueType>
    bool fixedPointSumOverflowed(
        const vfc::TFixedPoint<FracBitsValue, ValueType>& f_a, //!< 1st input
        const vfc::TFixedPoint<FracBitsValue, ValueType>& f_b, //!< 2nd input
        vfc::TFixedPoint<FracBitsValue, ValueType>& f_sum)     //!< sum of inputs (might be overflowed)
    {
        // unchecked add with no assertions!  User has to handle it, even in debug mode
        f_sum.setFixedpoint(f_a.fixedpoint() + f_b.fixedpoint());
        // If operands differ in sign, they do not overflow.
        // If they have the same sign and the result has another one, overflow occurred.
        // See http://www.research.att.com/~bs/abstraction-and-machine.pdf page 13
        return( 0 != (
                        (f_a.fixedpoint() ^ f_sum.fixedpoint()) &
                        (f_b.fixedpoint() ^ f_sum.fixedpoint()) &
                        (1 << vfc::TFixedPoint<FracBitsValue, ValueType>::VALIDBITS)
                     )
              );
    }
} // namespace vfc


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: include/vfc/core/vfc_fixedpoint.inl  $
//  Revision 1.58 2011/05/18 17:38:56MESZ Muehlmann Karsten (CC/PJ-FA1) (MUK2LR) 
//  - faster float <-> FP (mantis3653)
//  Revision 1.57 2009/10/01 12:20:57CEST Gaurav Jain (RBEI/EAS3) (gaj2kor) 
//  - Clean separation of fpu-code and fpu-free code using VFC_NO_FLOAT.
//  Revision 1.56 2009/09/01 09:52:19GMT+05:30 Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Removal of floating point operation from init(). (mantis : 3012)
//  Revision 1.55 2009/06/16 01:12:33IST Muehlmann Karsten (CC-DA/ESV2) (MUK2LR)
//  - addition overflow handling (mantis2872)
//  Revision 1.54 2009/03/25 16:23:47CET Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Linebreak at 120 in all fixedpoint files. (mantis : 0002692)
//  -Updated functionality to handle the negative values. (mantis : 0002559)
//  Revision 1.53 2009/03/16 21:21:28IST Muehlmann Karsten (CC-DA/ESV2) (MUK2LR)
//  replace std:: with stlalias:: to enable switching of STL (mantis2721)
//  Revision 1.52 2009/01/31 08:29:56CET Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Removal of QAC++ warnings.
//  (Mantis : 0002548,0002497)
//  Revision 1.51 2008/12/16 13:47:20IST Voelz Henning (CC-DA/ESV4) (VOH2HI)
//  faster isMultiplicationSafe() and isDivisionSafe()
//  Revision 1.50 2008/12/11 11:09:47CET Voelz Henning (CC-DA/ESV1) (VOH2HI)
//  asserts added
//  Revision 1.49 2008/11/11 16:36:56CET Voelz Henning (CC-DA/ESV1) (VOH2HI)
//  setFixedpoint method added (manits:2414)
//  Revision 1.48 2008/09/30 16:55:32CEST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  - Resolution of compilation error for Fixed point cppunit.
//  (Mantis :2354)
//  Revision 1.47 2008/08/12 13:59:47IST Voelz Henning (CC-DA/ESV1) (VOH2HI)
//  one and zero as constant added
//  Revision 1.46 2008/04/08 17:21:32CEST Voelz Henning (AE-DA/ESV1) (VOH2HI)
//  bugfix assert isXXXSafe()
//  Revision 1.45 2008/03/28 13:38:37CET Voelz Henning (AE-DA/ESV1) (VOH2HI)
//  bugfix isMultiplicationSafe()
//  Revision 1.44 2008/03/26 16:20:42CET Renner Christian (AE-DA/ESV1) (REC1LR)
//  bugfix switch_multiply assert
//  Revision 1.43 2008/03/25 08:48:35CET Voelz Henning (AE-DA/ESV1) (VOH2HI)
//  compilation error fixed static_cast in isDivisionSafe()
//  Revision 1.42 2008/03/04 14:27:29CET Voelz Henning (AE-DA/ESV1) (VOH2HI)
//  bugfix: isDivisionSafe(); avoid division by zero
//  Revision 1.41 2008/03/04 09:55:12CET Voelz Henning (AE-DA/ESV1) (VOH2HI)
//  bugfix: isDivisionSafe() shift first and then check if two different fixedpoint types are used
//  Revision 1.40 2008/02/28 09:21:21CET Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  qac++,codecoverage testing done
//  Revision 1.39 2008/02/20 19:57:19IST Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  compilation error fixed
//  Revision 1.38 2008/02/20 11:10:28IST Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  TFixedpointOperationPrecision,TFixedpointOperationSize classes are removed.
//  Removal od OpearationPolicy from TFixedPoint class
//  made Fixedpoint as Fundamental type.
//  Removed the warnings.
//  Revision 1.36 2008/01/30 23:51:29IST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Now compiles with gcc 3.44. Runtime could not be checked.
//  Revision 1.35 2008/01/30 18:55:28CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  FixedPoint with 64bit valuetype should work now. When converting a 32bit fixedpoint into a 64bit fixedpoint MW CW 9.3 fails to compile.
//  Hack: always use 64bit as value_type if shifting.
//  Revision 1.34 2008/01/30 11:59:19CET Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  gcc compilation error fixed
//  Revision 1.33 2008/01/28 20:37:02IST Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  Appropriated typecasting done
//  Revision 1.31 2008/01/17 15:46:30IST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  internal namespace changed to intern;
//  gcc warnings removed
//  gcc error (typename) removed
//  Revision 1.30 2007/11/28 17:03:37CET Voelz Henning (AE-DA/ESV1) (voh2hi)
//  bugfix: vfc_require() remove floating point ops
//  Revision 1.29 2007/10/24 09:09:49CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  - obsolete methods removed
//  - cosmetics
//  Revision 1.28 2007/10/22 15:07:10CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  warnings removed
//  Revision 1.27 2007/10/22 08:33:29CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  methods inlined
//  Revision 1.26 2007/10/17 16:46:15CEST vmr1kor
//  numeric_cast function added
//  Revision 1.25 2007/10/17 15:20:30IST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  TFixedPoint operations now using vfc_taylorpoly.hpp. Code from vfc::atan2 should be moved there too.
//  Revision 1.23 2007/10/16 13:49:27CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  Moved function definitions into inl.
//  Revision 1.22 2007/10/15 17:46:40CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  bugfix isDivisionSafe()
//  Revision 1.21 2007/10/15 17:30:37CEST Renner Christian (AE-DA/ESV1) (rec1lr)
//  renamed isDivisionSafe()
//  Revision 1.20 2007/10/15 16:57:34CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  implementations moved to inl file
//  Revision 1.19 2007/09/26 15:05:12CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  bugfix
//  Revision 1.18 2007/09/26 11:27:34CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  bugfix: float and double c'tor with too small values
//  Revision 1.17 2007/09/25 17:07:58CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  assert in conversion c'tor corrected
//  Revision 1.16 2007/09/24 14:50:34CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  variable max added
//  Revision 1.15 2007/09/24 10:17:57CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  operations moved to _ops file
//  Revision 1.14 2007/09/20 09:11:02CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  epsilon is not a method but a static constant
//  Revision 1.13 2007/09/19 10:53:10CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  bug fix asserts
//  Revision 1.12 2007/09/17 17:36:06CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  small bugfixes, sqrt for int64
//  Revision 1.11 2007/09/07 14:14:27CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  Added is_null CppUnit test. Removed warning about implicit cast in init(float32_t).
//  Revision 1.10 2007/09/07 13:51:10CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  Fixed operator+=, operator-= and we hopefully have a working constructor for vfc_types now. Furthermore added a volatile test to the CppUnit tests.
//  Revision 1.9 2007/09/03 13:43:25CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  to_fp_x<>() and to_fp_op<>() replaced by conversion c'tor due to problems with gcc compiler
//  Revision 1.8 2007/08/31 09:13:51CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  shift operator added
//  Revision 1.7 2007/08/30 15:02:27CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  to_fp_x<> (), to_fp_op<>(), min(), max() added; friend for comparison operators added
//  Revision 1.6 2007/08/29 13:49:53CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  atan(), asin(), acos() added; no friend operators*/-+; operator*/+- added for different fracbits fixedpoints
//  Revision 1.4 2007/08/27 13:59:25CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  Fixed constructor when using volatile floating. Removed (due static_cast) warning about unsigned long long from CW 9.4.
//  Revision 1.3 2007/08/27 08:15:21CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  c'tors and initialize changed
//  Revision 1.2 2007/08/24 16:30:54CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  inline function
//  conversion from float to fixed point with bit operations
//  Revision 1.1 2007/08/22 17:07:11CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_fixedpoint/vfc_fixedpoint.pj
//  Revision 1.8 2007/08/22 17:00:10CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  second walkthrough findings: template c'tor; code moved from hpp to inl; using vfc::abs(); rename TypePromtionTrait
//  Revision 1.7 2007/08/16 14:27:30CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  warnings removed if instanciated with int64_t
//  Revision 1.6 2007/08/16 09:25:23CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  first walkthrough findings:
//  no unsigned integer support; int64_t support; typetrait for promotion_type; mult_op_type and div_op_type; rename to_float(); asserts; div. smaller changes
//  Revision 1.5 2007/08/14 14:25:00CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  VFC_ENSURE replaced by VFC_REQUIRE
//  Revision 1.4 2007/08/13 15:21:16CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  abs removed, use vfc::abs() [vfc_math.inl 1.30 or higher required]
//  Revision 1.3 2007/08/10 09:39:41CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  - TFixedPointOperation Policy added
//  - "error management"
//  - uintXX support
//  Revision 1.2 2007/08/03 17:26:03CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - removed friend declaration
//  Revision 1.1 2006/12/07 09:01:20CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/zvh2hi/zvh2hi.pj
//  Revision 1.2 2006/12/01 10:37:38CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - fixed bug in overloaded operators
//=============================================================================
