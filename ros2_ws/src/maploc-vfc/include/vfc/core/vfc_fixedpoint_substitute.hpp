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
//       Compiler(s): VS7.1, VS8.0, CW9.3, CWembedded8.5
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
///     $Source: include/vfc/core/vfc_fixedpoint_substitute.hpp $
///     $Revision: 1.30 $
///     $Author: Muehlmann Karsten (CC/PJ-FA1) (MUK2LR) $
///     $Date: 2010/11/10 15:08:58MEZ $
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


#ifndef TFIXEDPOINT_SUBSTITUTE_HPP
#define TFIXEDPOINT_SUBSTITUTE_HPP

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_math.hpp"
#include "vfc/core/vfc_metaprog.hpp"
#include "vfc/core/vfc_type_traits.hpp"
#include "vfc/core/vfc_static_assert.hpp"
#include "vfc/core/vfc_assert.hpp"

#include "vfc/core/vfc_fixedpoint_math.hpp"

#include <limits>
#include <cmath>


namespace vfc
{

    template <typename floating_type, vfc::int32_t bits, vfc::int32_t frac_bits>
    class TFixedPointSubstitute
    {
        VFC_STATIC_ASSERT(vfc::TIsFloating<floating_type>::value);
    public:
        enum { SHIFT = frac_bits };

        struct CNoShift {};

        static const TFixedPointSubstitute epsilon;
        static const TFixedPointSubstitute max;
        static const TFixedPointSubstitute zero;
        static const TFixedPointSubstitute one;

        typedef floating_type value_type;

        // **** c'tors ****
        // copy and conversion c'tor
        template<typename other_floating_type, vfc::int32_t other_bits, vfc::int32_t other_frac_bits>
        inline TFixedPointSubstitute(const TFixedPointSubstitute<other_floating_type, other_bits, other_frac_bits>& f_rhs)
        {
            (*this).m_value = static_cast<floating_type>(f_rhs.getValue());
            #ifndef NDEBUG
            m_dbgMaxValue = 0.0;
            m_dbgMinValue = 0.0;
            dbgMaxValue();
            #endif
        }

        // float32_t/float32_t64_t/int32_t c'tor
        inline TFixedPointSubstitute(void) : m_value(static_cast<floating_type>(0))
        {
            #ifndef NDEBUG
            m_dbgMaxValue = 0.0;
            m_dbgMinValue = 0.0;
            dbgMaxValue();
            #endif
        }

        template <typename InputType>
        inline TFixedPointSubstitute(const InputType& f_rhs)
        {
            VFC_STATIC_ASSERT(vfc::TIsIntegral<InputType>::value || vfc::TIsFloating<InputType>::value);


            m_value = static_cast<floating_type>(f_rhs);
            #ifndef NDEBUG
            m_dbgMaxValue = 0.0;
            m_dbgMinValue = 0.0;
            dbgMaxValue();
            #endif
        }

        template <typename InputType>
        inline TFixedPointSubstitute(const InputType& f_rhs, CNoShift)
        {
            (*this).m_value = f_rhs / static_cast<floating_type>(static_cast<InputType>(1) << frac_bits);
            #ifndef NDEBUG
            m_dbgMaxValue = 0.0;
            m_dbgMinValue = 0.0;
            dbgMaxValue();
            #endif
        }

        template<typename other_floating_type, vfc::int32_t other_bits, vfc::int32_t other_frac_bits>
        inline TFixedPointSubstitute(const TFixedPointSubstitute<other_floating_type, other_bits, other_frac_bits>& f_rhs, CNoShift)
        {
            const vfc::int32_t shift_i32 = frac_bits - other_frac_bits;
            if (vfc::isPositive(shift_i32))
            {
                (*this).m_value = static_cast<floating_type>(f_rhs.getValue()) / static_cast<floating_type>(std::max(1L, 1L << shift_i32));
            }
            else
            {
                (*this).m_value = static_cast<floating_type>(f_rhs.getValue()) * static_cast<floating_type>(std::max(1L, 1L << vfc::abs(shift_i32)));
            }
            #ifndef NDEBUG
            m_dbgMaxValue = 0.0;
            m_dbgMinValue = 0.0;
            dbgMaxValue();
            #endif
        }


        ~TFixedPointSubstitute(void) {};


        // **** operators ****
        inline TFixedPointSubstitute& operator=(const TFixedPointSubstitute& f_rhs)
        {
            (*this).m_value = f_rhs.m_value;
            #ifndef NDEBUG
            dbgMaxValue();
            #endif
            return (*this);
        }

        // comparison operators
        inline bool    operator == (const TFixedPointSubstitute& f_rhs) const
        {
            return ((*this).m_value == f_rhs.m_value);
        }

        inline bool    operator != (const TFixedPointSubstitute& f_rhs) const
        {
            return !((*this) == f_rhs);
        }

        inline bool    operator < (const TFixedPointSubstitute& f_rhs) const
        {
            return ((*this).m_value < f_rhs.m_value);
        }

        inline bool    operator <= (const TFixedPointSubstitute& f_rhs) const
        {
            return ((*this) == f_rhs) || ((*this) < f_rhs);
        }

        inline bool    operator > (const TFixedPointSubstitute& f_rhs) const
        {
            return !((*this) <= f_rhs);
        }

        inline bool    operator >= (const TFixedPointSubstitute& f_rhs) const
        {
            return !((*this) < f_rhs);
        }



        inline TFixedPointSubstitute operator-(void) const
        {
            return TFixedPointSubstitute((*this).m_value * static_cast<floating_type>(-1));
        }

        inline TFixedPointSubstitute operator*(const TFixedPointSubstitute& f_rhs) const
        {
            return TFixedPointSubstitute((*this).m_value * f_rhs.m_value);
        }

        inline TFixedPointSubstitute operator/(const TFixedPointSubstitute& f_rhs) const
        {
            return TFixedPointSubstitute((*this).m_value / f_rhs.m_value);
        }

        inline TFixedPointSubstitute operator+(const TFixedPointSubstitute& f_rhs) const
        {
            return TFixedPointSubstitute((*this).m_value + f_rhs.m_value);
        }
        inline TFixedPointSubstitute operator-(const TFixedPointSubstitute& f_rhs) const
        {
            return TFixedPointSubstitute((*this).m_value - f_rhs.m_value);
        }

        inline TFixedPointSubstitute& operator*=(const TFixedPointSubstitute& f_rhs)
        {
            (*this).m_value *= f_rhs.m_value;
            #ifndef NDEBUG
            dbgMaxValue();
            #endif
            return (*this);
        }

        inline TFixedPointSubstitute& operator/=(const TFixedPointSubstitute& f_rhs)
        {
            (*this).m_value /= f_rhs.m_value;
            #ifndef NDEBUG
            dbgMaxValue();
            #endif
            return (*this);
        }

        inline TFixedPointSubstitute& operator+=(const TFixedPointSubstitute& f_rhs)
        {
            (*this).m_value += f_rhs.m_value;
            #ifndef NDEBUG
            dbgMaxValue();
            #endif
            return (*this);
        }

        inline TFixedPointSubstitute& operator-=(const TFixedPointSubstitute& f_rhs)
        {
            (*this).m_value -= f_rhs.m_value;
            #ifndef NDEBUG
            dbgMaxValue();
            #endif
            return (*this);
        }

        inline TFixedPointSubstitute& operator*=(const vfc::float64_t& f_rhs)
        {
            (*this).m_value *= f_rhs;
            #ifndef NDEBUG
            dbgMaxValue();
            #endif
            return (*this);
        }

        inline TFixedPointSubstitute& operator/=(const vfc::float64_t& f_rhs)
        {
            (*this).m_value /= f_rhs;
            #ifndef NDEBUG
            dbgMaxValue();
            #endif
            return (*this);
        }

        inline TFixedPointSubstitute& operator+=(const vfc::float64_t& f_rhs)
        {
            (*this).m_value += f_rhs;
            #ifndef NDEBUG
            dbgMaxValue();
            #endif
            return (*this);
        }

        inline TFixedPointSubstitute& operator-=(const vfc::float64_t& f_rhs)
        {
            (*this).m_value -= f_rhs;
            #ifndef NDEBUG
            dbgMaxValue();
            #endif
            return (*this);
        }


        inline TFixedPointSubstitute operator<<(const vfc::int32_t shift_i32) const
        {
            VFC_REQUIRE(shift_i32 >= 0);
            return TFixedPointSubstitute((*this).m_value * static_cast<floating_type>(1L << shift_i32));
        }

        inline TFixedPointSubstitute operator>>(const vfc::int32_t shift_i32) const
        {
            VFC_REQUIRE(shift_i32 >= 0);
            return TFixedPointSubstitute((*this).m_value / static_cast<floating_type>(1L << shift_i32));
        }

        inline TFixedPointSubstitute& operator++  (void)
        {
            (*this).m_value += static_cast<floating_type>(1);
            dbgMaxValue();
            return (*this);
        }

        inline TFixedPointSubstitute     operator++  (int)
        {
            TFixedPointSubstitute temp(*this);
            this->operator++();
            return temp;
        }

        inline TFixedPointSubstitute& operator--  (void)
        {
            (*this).m_value -= static_cast<floating_type>(1);
            dbgMaxValue();
            return (*this);
        }

        inline TFixedPointSubstitute     operator--  (int)
        {
            TFixedPointSubstitute temp(*this);
            this->operator--();
            return temp;
        }

        // **** conversion methods ****
        inline vfc::float32_t  to_float32(void) const { return static_cast<vfc::float32_t>((*this).m_value); }
        inline vfc::float64_t  to_float64(void) const { return static_cast<vfc::float64_t>((*this).m_value); }
        inline vfc::int32_t    to_int(void)     const { return static_cast<vfc::int32_t>((*this).m_value);   }
        inline vfc::int32_t    to_nint(void)    const { return static_cast<vfc::int32_t>((*this).m_value + 0.5);   }  // check TBD
        inline vfc::int32_t    to_floor(void)   const { return static_cast<vfc::int32_t>((*this).m_value);   }  // check TBD
        inline vfc::int32_t    to_ceil(void)    const { return static_cast<vfc::int32_t>((*this).m_value);   }  // check TBD


        // **** other methods ****
        inline floating_type getValue(void)  const  { return (*this).m_value; }
        // fixedpoint method cannot be rebuilt in this class, because not all information is avaiable
        inline vfc::int32_t fixedpoint(void) const
        {
            return static_cast<vfc::int32_t>((*this).m_value * static_cast<floating_type>(1L << frac_bits));
        }
        inline bool is_null(void) const { return (vfc::isZero(static_cast<floating_type>(0))); }
        inline bool isDivisionSafe(const TFixedPointSubstitute& f_rhs)  const    { return (false == vfc::isZero(f_rhs.m_value));}
        inline bool isMultiplicationSafe(const TFixedPointSubstitute& ) const    { return true; }
        inline bool isAdditionSafe(const TFixedPointSubstitute& )       const    { return true; }
        inline bool isLeftShiftSafe(const vfc::int32_t )                const    { return true; }

        // **** intern trigonometric ****
        inline TFixedPointSubstitute sin(void)  const { return (std::sin( (*this).m_value)); }
        inline TFixedPointSubstitute cos(void)  const { return (std::cos( (*this).m_value)); }
        inline TFixedPointSubstitute tan(void)  const { return (std::tan( (*this).m_value)); }
        inline TFixedPointSubstitute asin(void) const { return (std::asin((*this).m_value)); }
        inline TFixedPointSubstitute acos(void) const { return (std::acos((*this).m_value)); }
        inline TFixedPointSubstitute atan(void) const { return (std::atan((*this).m_value)); }

    private:
        // private member to store m_value
        floating_type m_value;



        // this method should only be used for development and debugging purposes to get the ranges for fixedpoint implementation
        void dbgMaxValue(void)
        {
            #ifndef NDEBUG
                if (vfc::abs(m_value) > m_dbgMaxValue)
                {
                    m_dbgMaxValue = vfc::abs(m_value);
                }
                if(vfc::isZero(m_dbgMinValue))
                {
                    m_dbgMinValue = m_dbgMaxValue;
                }
                if (  (vfc::abs(m_value) < m_dbgMinValue)
                    &&(!isZero(m_value)))
                {
                    m_dbgMinValue = vfc::abs(m_value);
                }
            #endif
        }
        #ifndef NDEBUG
            floating_type m_dbgMaxValue;
            floating_type m_dbgMinValue;
        #endif

    };

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    const TFixedPointSubstitute<floating_type, bits, frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits>::zero = TFixedPointSubstitute<floating_type, bits, frac_bits>
                                                                    (static_cast<floating_type>(0.));

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    const TFixedPointSubstitute<floating_type, bits, frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits>::one = TFixedPointSubstitute<floating_type, bits, frac_bits>
                                                                    (static_cast<floating_type>(1.));

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    const TFixedPointSubstitute<floating_type, bits, frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits>::epsilon = TFixedPointSubstitute<floating_type, bits, frac_bits>
                                            (static_cast<floating_type>(1.) / static_cast<floating_type>(1L << frac_bits));

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    const TFixedPointSubstitute<floating_type, bits, frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits>::max = TFixedPointSubstitute<floating_type, bits, frac_bits>
            (static_cast<floating_type>((1L << ((bits - frac_bits) - 1L))) - (static_cast<floating_type>(1.) /
             static_cast<floating_type>(1L << frac_bits)));

    // trigonometric methods
    template<vfc::int32_t TaylorSummandCnt, typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> sin(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return f_arg.sin();
    }


    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> sin(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return f_arg.sin();
    }

    template<vfc::int32_t TaylorSummandCnt, typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> cos(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return f_arg.cos();
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> cos(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return f_arg.cos();
    }

    template<vfc::int32_t TaylorSummandCnt, typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> tan(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return f_arg.tan();
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> tan(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return f_arg.tan();
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> asin(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return f_arg.asin();
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> acos(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return f_arg.acos();
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> atan(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return f_arg.atan();
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> atan2(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg1,
                                                          const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg2)
    {
        return TFixedPointSubstitute<floating_type, bits, frac_bits>(std::atan2(f_arg1.getValue(), f_arg2.getValue()));
    }

    // other
    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> abs(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return TFixedPointSubstitute<floating_type, bits, frac_bits>(vfc::abs(f_arg.getValue()));
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> sqrt(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return TFixedPointSubstitute<floating_type, bits, frac_bits>(vfc::sqrt(f_arg.getValue()));
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> sqr(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return TFixedPointSubstitute<floating_type, bits, frac_bits>(vfc::sqr(f_arg.getValue()));
    }


    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> pow(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return TFixedPointSubstitute<floating_type, bits, frac_bits>(std::pow(f_arg.getValue()));
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> log(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return TFixedPointSubstitute<floating_type, bits, frac_bits>(std::log(f_arg.getValue()));
    }


    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> pow(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_base,
                                                        const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_exp)
    {
        return TFixedPointSubstitute<floating_type, bits, frac_bits>(std::pow(f_base.getValue(), f_exp.getValue()));
    }

    template <typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    inline vfc::TFixedPointSubstitute<floating_type, bits, frac_bits> exp(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_value)
    {
        return TFixedPointSubstitute<floating_type, bits, frac_bits>(static_cast<floating_type>(::exp(f_value.getValue())));
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> fmod(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg)
    {
        return TFixedPointSubstitute<floating_type, bits, frac_bits>(vfc::fmod(f_arg.getValue()));
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> min(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg1,
                                                        const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg2)
    {
        return TFixedPointSubstitute<floating_type, bits, frac_bits>(std::min(f_arg1.getValue(), f_arg2.getValue()));
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>
    TFixedPointSubstitute<floating_type, bits, frac_bits> max(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg1,
                                                        const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_arg2)
    {
        return TFixedPointSubstitute<floating_type, bits, frac_bits>(std::max(f_arg1.getValue(), f_arg2.getValue()));
    }


    ///  the nint function returns the nearest integer of specified value
    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>  inline
    vfc::int32_t nint    (const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_value)
    {
        return f_value.to_nint();
    }

    /// the floor function returns an integer value representing the largest integer that is less than or equal to specified value.
    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>  inline
    vfc::int32_t floor   (const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_value)
    {
        return f_value.to_floor();
    }

    /// the ceil function an integer value representing the smallest integer that is greater than or equal to specified value.
    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>  inline
    vfc::int32_t ceil    (TFixedPointSubstitute<floating_type, bits, frac_bits>& f_value)
    {
        return f_value.to_ceil();
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>  inline
    bool  isEqual(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_value1,
                       const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_value2,
                       const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_delta)
    {
        return (isEqual(f_value1.getValue(), f_value2.getValue(), f_delta.getValue()));
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>  inline
    bool  isPositive(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_value)
    {
        return (isPositive(f_value.getValue()));
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>  inline
    bool isNegative(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_value)
    {
         return (isNegative(f_value.getValue()));
    }

    template<typename floating_type, vfc::int32_t bits,  vfc::int32_t frac_bits>  inline
    bool  isZero(const TFixedPointSubstitute<floating_type, bits, frac_bits>& f_value)
    {
        return (isZero(f_value.getValue()));
    }

    //template <vfc::int32_t dummyBitsValue, vfc::int32_t dummyFracValue>
    template <>
    struct TFixedPointValuePolicy< vfc::TFixedPointSubstitute<vfc::float64_t, 32,  20> >
    {
        typedef vfc::TFixedPointSubstitute<vfc::float64_t, 32,  20> fixedpoint_type;

        static inline fixedpoint_type G_PI_2()       { return fixedpoint_type(vfc::G_PI_2); }
        static inline fixedpoint_type G_PI()         { return fixedpoint_type(vfc::G_PI); }
        static inline fixedpoint_type G_3PI_2()      { return fixedpoint_type(3.0 * vfc::G_PI_2); }
        static inline fixedpoint_type G_2PI()        { return fixedpoint_type(vfc::G_2PI); }
        static inline fixedpoint_type G_NEGATIVE_1() { return fixedpoint_type(-1.0); }
        static inline fixedpoint_type G_1()          { return fixedpoint_type(1.0); }
    };

} // namespace vfc


#endif // #ifndef TFixedPointSubstitute_HPP

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: include/vfc/core/vfc_fixedpoint_substitute.hpp  $
//  Revision 1.30 2010/11/10 15:08:58MEZ Muehlmann Karsten (CC/PJ-FA1) (MUK2LR) 
//  (re-)enable compilation of TFixedPointSubstitute (mantis3524)
//  Revision 1.29 2008/12/03 09:49:30CET Voelz Henning (CC-DA/ESV4) (VOH2HI) 
//  exp corrected
//  Revision 1.28 2008/12/01 18:26:06CET Voelz Henning (CC-DA/ESV1) (VOH2HI) 
//  exp added
//  Revision 1.27 2008/10/17 14:06:09CEST Gaurav Jain (RBEI/EAE5) (gaj2kor) 
//  -Removal of compilation error.
//  (Mantis : 2383 )
//  Revision 1.26 2008/09/11 14:16:47IST Voelz Henning (CC-DA/ESV1) (VOH2HI)
//  mantis 2338  provide zero and one
//  Revision 1.25 2008/02/25 13:25:19CET Renner Christian (AE-DA/ESV1) (rec1lr)
//  added value_type
//  Revision 1.24 2008/02/20 06:42:39CET Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  Removal of OpearationPolicy
//  Revision 1.23 2007/12/19 20:17:10IST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  log() added
//  Revision 1.22 2007/10/22 10:07:55CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  sin / cos / tan with 4 template args added
//  Revision 1.21 2007/10/15 17:37:43CEST Renner Christian (AE-DA/ESV1) (rec1lr)
//  renamed isDivisionSafe()
//  Revision 1.20 2007/10/12 09:50:00CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  const correctness
//  Revision 1.19 2007/10/10 12:59:40CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  warnings removed
//  Revision 1.18 2007/10/10 10:24:31CEST Renner Christian (AE-DA/ESV1) (rec1lr)
//  added isLeftShiftSafe()
//  Revision 1.17 2007/10/09 13:08:33CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  isMultiplicationSafe(), isAdditionSafe() added
//  Revision 1.16 2007/10/04 11:41:32CEST Renner Christian (AE-DA/ESV1) (rec1lr)
//  added defines for non-debug code
//  Revision 1.15 2007/10/04 08:17:43CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  fixedpoint() method implemented
//  Revision 1.14 2007/10/02 16:24:00CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  compiler warnings removed using c'tor with integer
//  Revision 1.13 2007/10/01 17:49:15CEST Renner Christian (AE-DA/ESV1) (rec1lr)
//  added debugMinValue
//  Revision 1.12 2007/09/27 16:20:24CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  bugfix noshift c'tor
//  Revision 1.11 2007/09/27 12:01:39CEST Renner Christian (AE-DA/ESA3) (rec1lr)
//  added isDevisionSafe()
//  Revision 1.10 2007/09/26 16:17:42CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  if NDEBUG is not set; the maximum value is stored for development and debugging
//  Revision 1.9 2007/09/26 11:00:06CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  bugfix epsilon/max; operator++(int) and operator--(int) added
//  Revision 1.8 2007/09/25 13:55:26CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  bugfix return value to_float64()
//  Revision 1.7 2007/09/25 10:18:54CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  bugfix: assert in operator<< and  operator>>
//  Revision 1.6 2007/09/24 14:50:35CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  variable max added
//  Revision 1.5 2007/09/24 10:38:54CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  isZero, isPositive, isNegative, isEqual added
//  Revision 1.4 2007/09/24 10:17:44CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  sqr() added
//  Revision 1.3 2007/09/21 14:09:59CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  more functionality added
//  Revision 1.2 2007/09/21 11:05:44CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  bugfix namespace
//  Revision 1.1 2007/09/21 09:36:39CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_fixedpoint/vfc_fixedpoint.pj
//=============================================================================
