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
//       Compiler(s): VS7.1
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: zvh2hi
//  Department: CR/AEM
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_math.inl $
///     $Revision: 1.55 $
///     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
///     $Date: 2016/01/27 14:59:32MEZ $
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


// stdlib includes
#include <cmath>
#include <algorithm>
#include <limits.h>  // CHAR_BIT

#if defined (VFC_COMPILER_ARMRVCT) 
#   if defined (VFC_ARM_DETECTED) && (__ARMCC_VERSION >= 410713) && defined(VFC_HAS_ARM_NEON)
#       include <arm_neon.h>                   // used for intrinsic functions macros from RVCT
#   endif
#elif defined(VFC_COMPILER_VISUALC) 
#   if !defined(VFC_CLR_DETECTED)
#       include <intrin.h>
#   endif
#endif

// vfc/core includes
#include "vfc/core/vfc_assert.hpp"          // used for VFC_REQUIRE
#include "vfc/core/vfc_type_traits.hpp"     // used for TIsFloating<>
#include "vfc/core/vfc_metaprog.hpp"        // used for TInt2Boolean<>

namespace vfc
{   // namespace vfc opened

    //---------------------------------------------------------------------
    // conditional doxygen documentation
    //! @cond VFC_DOXY_INTERN
    //---------------------------------------------------------------------

    namespace intern
    {   // namespace intern opened
        template <class T>
        inline T           clampValueToMinMax (const T& f_min, const T& f_value, const T& f_max, false_t)
        {
            return (f_value < f_min)?(f_min):((f_value>f_max)?(f_max):(f_value));
        }

        template <class T>
        inline T           clampValueToMinMax (const T& f_min, const T& f_value, const T& f_max, true_t)
        {
#if defined(VFC_COMPILER_ARMRVCT) && defined(VFC_ARM_DETECTED) && (__ARMCC_VERSION >= 410713) && defined(VFC_HAS_ARM_NEON)
            vfc::float32_t f_result_f32=0.f;
            // if (f_min > f_value) ? f_min : f_value
            f_result_f32 = vget_lane_f32(vmax_f32(vdup_n_f32(f_value), vdup_n_f32(f_min)  ), 0);
            // if (f_max < f_value) ? f_max : f_value
            f_result_f32 = vget_lane_f32(vmin_f32(vdup_n_f32(f_result_f32), vdup_n_f32(f_max)  ), 0);
            return (f_result_f32);
#else
            return (f_value < f_min)?(f_min):((f_value>f_max)?(f_max):(f_value));
#endif
        }

        // unsigned implementation just returns value
        template <class T>  inline
        T   integer_abs (const T& f_value, true_t)  {   return f_value;}

        //! branch free abs implementation for signed integer types
        template <class T>  inline
        T   integer_abs (const T& f_value, false_t)
        {
            VFC_STATIC_ASSERT(!TIsFloating<T>::value);
            VFC_STATIC_ASSERT(!TIsUnsignedArithmetic<T>::value);
            VFC_REQUIRE2((f_value != (stlalias::numeric_limits<T>::min)()) ,
                "f_value is equal to the minimum normalized value(stlalias::numeric_limits<T>::min()). \
                    vfc::abs not possible for this value");
            return (f_value - ((f_value + f_value) & (f_value >> (sizeof(T)*CHAR_BIT-1))));
        }

        template <class T>  inline
        T   abs (const T& f_value, true_t)  {   return ::fabs(f_value);}

        template <class T>  inline
        T   abs (const T& f_value, false_t)
        {
            return static_cast<T>(integer_abs(f_value,
            typename TInt2Boolean<vfc::TIsUnsignedArithmetic<T>::value>::type()));
        }

        template <class T>    inline
        bool    isINF(const T& f_value, true_t)
        {
#ifdef VFC_COMPILER_ARMRVCT
            return isinf(f_value);
#else
            // PRQA S 3270 ++
            return (       (stlalias::numeric_limits<T>::infinity() == f_value)
                   ||      (stlalias::numeric_limits<T>::infinity() == -f_value));
            // PRQA S 3270 --
#endif
        }

        template <class T>    inline
        bool    isINF (const T& , false_t)
        {
            return false;
        }

        template <class T>    inline
        bool    isZero (const T& f_value, true_t)
        {
            return (       (stlalias::numeric_limits<T>::epsilon() > -f_value)
                    &&     (stlalias::numeric_limits<T>::epsilon() >  f_value));
        }

        template <class T>    inline
        bool    isZero (const T& f_value, false_t)
        {
            return (static_cast<T>(0) == f_value);
        }

        template <class T>    inline
        bool    isPositive (const T& f_value, true_t)
        {
            return ( stlalias::numeric_limits<T>::epsilon() <= f_value);
        }

        template <class T>    inline
        bool    isPositive (const T& f_value, false_t)
        {
            return (static_cast<T>(0) < f_value);
        }

        template <class T>    inline
        bool    isNegative (const T& f_value, true_t, false_t)
        {
            return ( stlalias::numeric_limits<T>::epsilon() <= -f_value);
        }

        template <class T>    inline
        bool    isNegative (const T& f_value, false_t, false_t)
        {
            return (static_cast<T>(0) > f_value);
        }

        template <class T>    inline
        bool    isNegative (const T& f_value, false_t, true_t)
        {
            return false;
        }

        template <class T>    inline
        bool    isEqual    (const T& f_value1, const T& f_value2, false_t)
        {
            return (f_value1 == f_value2);
        }

        template <class T>    inline
        bool    isEqual    (const T& f_value1, const T& f_value2, true_t)
        {
            const T ONE = static_cast<T>(1);

            const T abs1 = ::fabs(f_value1);
            const T abs2 = ::fabs(f_value2);

            const T maxabs12  = (stlalias::max)(abs1,abs2);

            if (ONE >= maxabs12)
            {
                // absolute error for small values
                return (::fabs(f_value1-f_value2) < stlalias::numeric_limits<T>::epsilon());
            }
            else
            {
                // relative error for great values
                return (::fabs(f_value1-f_value2) < (stlalias::numeric_limits<T>::epsilon()*maxabs12));
            }
        }

        //! numeric_cast specialization for floating to floating type
        template <class ReturnType, class ArgumentType> inline
        ReturnType    numeric_cast(const ArgumentType& f_value, true_t, true_t)
        {
            return static_cast<ReturnType>    (f_value);
        }

        //! numeric_cast specialization for integral to floating type
        template <class ReturnType, class ArgumentType> inline
        ReturnType    numeric_cast(const ArgumentType& f_value, true_t, false_t)
        {
            return static_cast<ReturnType>    (f_value);
        }

        //! numeric_cast specialization for floating to integral type
        template <class ReturnType, class ArgumentType> inline
        ReturnType    numeric_cast(const ArgumentType& f_value, false_t, true_t)
        {
            return static_cast<ReturnType>    (::vfc::nint(f_value));
        }

        //! numeric_cast specialization for integral to integral type
        template <class ReturnType, class ArgumentType> inline
        ReturnType    numeric_cast(const ArgumentType& f_value, false_t, false_t)
        {
            return static_cast<ReturnType>    (f_value);
        }

        //! used for type pruning.  Allows "conversion" \c float32_t <->
        //! \c int32_t without violating strict aliasing rules.
        union UFastRsqrtAliasingHelper {
            vfc::float32_t m_val_f32;
            vfc::int32_t m_val_i32;
        };

#ifndef VFC_NO_INT64
        //! used for type pruning. Allows "conversion" \c float64_t <->
        //! \c int64_t without violating strict aliasing rules.
        union UFastRsqrtAliasing64Helper {
            vfc::float64_t m_val_f64;
            vfc::int64_t m_val_i64;
        }; 
#endif

    }   // namespace intern closed

    //---------------------------------------------------------------------
    //! @endcond
    // of VFC_DOXY_INTERN
    //---------------------------------------------------------------------

}   // namespace vfc closed



//=============================================================================
// test for NAN
//=============================================================================

    template <class T>    inline
    bool    vfc::isNAN     (const T& f_value)
    {
#if defined(VFC_COMPILER_ARMRVCT)
        return isnan(f_value);
#elif defined (VFC_COMPILER_AC6)
        return isnan(f_value);
#else
        // source: Appendix of IEEE 754 floating point standard
        // PRQA S 3270 ++
        return (f_value!=f_value);
        // PRQA S 3270 --
#endif
    }

//=============================================================================
// test for +INF, -INF
//=============================================================================

    template <class T>  inline
    bool    vfc::isINF       (const T& f_value)
    {
        return intern::isINF(f_value, typename TInt2Boolean<TIsFloating<T>::value>::type());
    }

//=============================================================================
// tests for ZERO
//=============================================================================

    template <class T>    inline
    bool    vfc::isZero    (const T& f_value)
    {
        return intern::isZero(f_value, typename TInt2Boolean<TIsFloating<T>::value>::type());
    }

    template <class T>    inline
    bool    vfc::notZero    (const T& f_value)
    {
        return !isZero(f_value);
    }

    template <class T>    inline
    bool    vfc::isPositive    (const T& f_value)
    {
        return intern::isPositive(f_value, typename TInt2Boolean<TIsFloating<T>::value>::type());
    }

    template <class T>    inline
    bool    vfc::notPositive    (const T& f_value)
    {
        return !isPositive(f_value);
    }

    template <class T>    inline
    bool    vfc::isNegative    (const T& f_value)
    {
        return intern::isNegative(  f_value,
                                    typename TInt2Boolean<TIsFloating<T>::value>::type(),
                                    typename TInt2Boolean<TIsUnsignedArithmetic<T>::value>::type());
    }

    template <class T>    inline
    bool    vfc::notNegative    (const T& f_value)
    {
        return !isNegative(f_value);
    }

//=============================================================================
// tests for EQUALITY
//=============================================================================

    template <class T>    inline
    bool    vfc::isEqual    (const T& f_value1, const T& f_value2)
    {
        return intern::isEqual( f_value1, f_value2, typename TInt2Boolean<TIsFloating<T>::value>::type());
    }

    template <class T>
    bool    vfc::isEqual        (const T& f_value1, const T& f_value2, const T& f_epsilon)
    {
        VFC_STATIC_ASSERT(TIsFloating<T>::value);
        const T l1norm = abs(f_value1-f_value2);
        return (l1norm<=f_epsilon);
    }

    template <class T>    inline
    bool    vfc::notEqual    (const T& f_value1, const T& f_value2)
    {
        return !isEqual( f_value1, f_value2);
    }

//=============================================================================
// checked divide
//=============================================================================

    template <class T>    inline
    T    vfc::divide (const T& f_num, const T& f_denom)
    {
        VFC_REQUIRE2(!isZero(f_denom),"DivisionByZero");
        return f_num/f_denom;
    }

//=============================================================================
// more mathematical functions
//=============================================================================
    inline vfc::float32_t vfc::floor(vfc::float32_t f_value) {
      return stlalias::floor(f_value);
    }

    inline vfc::float32_t vfc::ceil(vfc::float32_t f_value) {
      return stlalias::ceil(f_value);
    }

#if defined (VFC_COMPILER_ARMRVCT) && defined (VFC_ARM_DETECTED) && (__ARMCC_VERSION >= 410713)
#pragma push
#pragma arm
#endif
    inline
    vfc::int32_t    vfc::clampNegValueToZero(int32_t f_value)
    {
#if defined (VFC_COMPILER_ARMRVCT) && defined (VFC_ARM_DETECTED) && (__ARMCC_VERSION >= 410713)
        vfc::int32_t l_result_i32=f_value;
        __asm
        {
            CMN f_value, 0
            MOVMI  l_result_i32, 0
        }
        return l_result_i32;
#else
        return f_value & ~(f_value >> 31);
#endif
    }
#if defined (VFC_COMPILER_ARMRVCT) && defined (VFC_ARM_DETECTED) && (__ARMCC_VERSION >= 410713)
#pragma pop
#endif

    inline
    vfc::float64_t    vfc::clampNegValueToZero(vfc::float64_t f_value)
    {
      return (0.0>f_value) ? static_cast<vfc::float64_t>(0.0) : f_value;
    }

    inline
    vfc::float32_t    vfc::clampNegValueToZero(vfc::float32_t f_value)
    {
#if defined (VFC_COMPILER_ARMRVCT) && defined (VFC_ARM_DETECTED) && (__ARMCC_VERSION >= 410713) && defined(VFC_HAS_ARM_NEON)
      return vget_lane_f32(vmax_f32(vdup_n_f32(0.0f), vdup_n_f32(f_value)), 0);// Pass vectors to VMAX instruction
#else
      return (0>f_value) ? static_cast<vfc::float32_t>(0) : f_value;
#endif
    }

    template <class T>    inline
    T       vfc::clampNegValueToZero(const T& f_value)
    {
      return (0>f_value) ? static_cast<T>(0) : f_value;
    }

    template <class T>  inline
    T   vfc::abs (const T& f_value)
    {
        return intern::abs(f_value, typename TInt2Boolean<TIsFloating<T>::value>::type());
    }

    inline
    vfc::int32_t    vfc::nint   (vfc::float64_t f_value)
    {
        vfc::int32_t l_ret_i32 = 0;

        // specialized versions for some platforms or compilers
#if defined(VFC_COMPILER_VISUALC) && !defined(VFC_CLR_DETECTED)
        // ms visual c++ with x86 and disabled CLR
        l_ret_i32 = _mm_cvtsd_si32(_mm_set_sd(f_value));
#elif defined(VFC_COMPILER_MWERKS) && defined(VFC_PROCESSOR_IX86)
        // metrowerks codewarrior with x86
        asm
        {
            fld f_value
            fistp l_ret_i32
        }
#elif defined(VFC_COMPILER_ARMRVCT) && defined(VFC_ARM_DETECTED) && (__ARMCC_VERSION >= 410713) && defined(VFC_HAS_ARM_NEON)
        // ARM RVCT 4.1 (patch 4, build 713) (and up, hopefully) finally understands VFP in inline assembly
        vfc::float32_t l_rounded_f32;
        __asm
        {
            vcvtr.s32.f64 l_rounded_f32, f_value
            vmov          l_ret_i32, l_rounded_f32
        }
#else
        // std c fallback
        l_ret_i32 = (f_value >= 0.)
            ? static_cast<vfc::int32_t>(f_value + 0.5)
            : static_cast<vfc::int32_t>(f_value - 0.5);
#endif

        return l_ret_i32;
    }

    inline
    vfc::int32_t    vfc::nint   (vfc::float32_t f_value)
    {
        vfc::int32_t l_ret_i32 = 0;

        // specialized versions for some platforms or compilers
#if defined(VFC_COMPILER_VISUALC) && !defined(VFC_CLR_DETECTED)
        // ms visual c++ with x86 and disabled CLR
        l_ret_i32 = _mm_cvtss_si32(_mm_set_ss(f_value));
#elif defined(VFC_COMPILER_MWERKS) && defined(VFC_PROCESSOR_IX86)
        // metrowerks codewarrior with x86
        asm
        {
            fld f_value
            fistp l_ret_i32
        }
#elif defined(VFC_COMPILER_ARMRVCT) && defined(VFC_ARM_DETECTED) && (__ARMCC_VERSION >= 410713) && defined(VFC_HAS_ARM_NEON)
        // ARM RVCT 4.1 (patch 4, build 713) (and up, hopefully) finally understands VFP in inline assembly
        vfc::float32_t l_rounded_f32;
        __asm
        {
            vcvtr.s32.f32 l_rounded_f32, f_value
            vmov          l_ret_i32, l_rounded_f32
        }
#else
        // std c fallback
        l_ret_i32 = (f_value >= 0.)
            ? static_cast<vfc::int32_t>(f_value + 0.5)
            : static_cast<vfc::int32_t>(f_value - 0.5);
#endif

        return l_ret_i32;
    }

    inline
    vfc::int32_t    vfc::isqrt  (vfc::int32_t f_value)
    {
        VFC_REQUIRE( 0 <= f_value );

        if (f_value <= 1)
        {
            return f_value;
        }

        vfc::uint32_t l_shift_u32 = 1U;
        vfc::uint32_t l_valueTemp_u32 = f_value - 1U;

        if (l_valueTemp_u32 > 65535U)
        {
            l_shift_u32 = l_shift_u32+8U;
            l_valueTemp_u32 = l_valueTemp_u32 >> 16U;
        }
        if (l_valueTemp_u32 > 255U)
        {
            l_shift_u32 = l_shift_u32+4U;
            l_valueTemp_u32 = l_valueTemp_u32 >> 8U;
        }
        if (l_valueTemp_u32 > 15U)
        {
            l_shift_u32 = l_shift_u32+2U;
            l_valueTemp_u32 = l_valueTemp_u32 >> 4U;
        }
        if (l_valueTemp_u32 > 3U)
        {
            ++l_shift_u32;
        }

        vfc::uint32_t l_g0_u32 = 1U << l_shift_u32;
        vfc::uint32_t l_g1_u32 = (l_g0_u32 + (static_cast<vfc::uint32_t>(f_value) >> l_shift_u32)) >> 1U;

        while (l_g1_u32 < l_g0_u32)
        {
            l_g0_u32 = l_g1_u32;
            l_g1_u32 = (l_g0_u32 + (static_cast<vfc::uint32_t>(f_value)/l_g0_u32)) >> 1U;
        }

        return static_cast<vfc::int32_t>(l_g0_u32);
    }

    inline
    vfc::float32_t vfc::fast_rsqrt(vfc::float32_t f_value)
    {
        VFC_REQUIRE( 0.f <= f_value );
        vfc::float32_t halfValue = 0.5f*f_value;
        // bit magic, relies on IEEE 32bit floating point representation,
        // original constant from Quake3 was 0x5f3759df, see paper cited above.
        vfc::intern::UFastRsqrtAliasingHelper fi;
        fi.m_val_f32 = f_value;
        fi.m_val_i32 = 0x5f375a86 - (fi.m_val_i32 >> 1);

        // Newton step to increase accuracy.
        return fi.m_val_f32 * (1.5f - halfValue * fi.m_val_f32 * fi.m_val_f32);
    }

#ifndef VFC_NO_INT64
    inline
    vfc::float64_t vfc::fast_rsqrt(vfc::float64_t f_value)
    {
        VFC_REQUIRE( static_cast<vfc::float64_t>(0.0) <= f_value );
        vfc::float64_t halfValue = static_cast<vfc::float64_t>(0.5)*f_value;
        // constant 0x5fe6eb50c7b537a9, see link cited above.
        vfc::intern::UFastRsqrtAliasing64Helper fi;
        fi.m_val_f64 = f_value;
        fi.m_val_i64 = 0x5fe6eb50c7b537a9ll - (fi.m_val_i64 >> 1);

        // Newton step to increase accuracy.
        return fi.m_val_f64 * (static_cast<vfc::float64_t>(1.5) - halfValue * fi.m_val_f64 * fi.m_val_f64);
    }
#endif

    template <class ReturnType, class ArgumentType> inline
    ReturnType    vfc::numeric_cast(const ArgumentType& f_value)
    {
        return intern::numeric_cast<ReturnType>
                        (    f_value,
                            typename TInt2Boolean<TIsFloating<ReturnType>::value>::type(),
                            typename TInt2Boolean<TIsFloating<ArgumentType>::value>::type()
                        );
    }

    template<class ValueType>
    inline
    typename vfc::TSqrtTraits<ValueType>::ArgType  vfc::sqrt   (ValueType f_value)
    {
        // typecast the Valuetype parameter to float64_t just to validate the precondition.
        // float64_t is used as that would be the greater size of the input being received.
        VFC_REQUIRE (0.0 <= static_cast<vfc::float64_t>(f_value));
        return vfc::internSqrt(f_value, typename TInt2Boolean<vfc::TIsSameType<ValueType,vfc::float64_t>::value>::type());
    }


    template<class ValueType>
    inline
    typename vfc::TSqrtTraits<ValueType>::ArgType  vfc::internSqrt   (ValueType f_value, true_t)
    {
#if defined(VFC_COMPILER_ARMRVCT) && defined(VFC_ARM_DETECTED) && (__ARMCC_VERSION >= 410713) && defined(VFC_HAS_ARM_NEON)
        vfc::float64_t l_value = static_cast<vfc::float64_t>(f_value);
            __asm
            {
                vsqrt.f64 l_value, l_value
            }
            return l_value;
#else
        return ::sqrt(f_value); 
#endif
    }

    template<class ValueType>
    inline
    typename vfc::TSqrtTraits<ValueType>::ArgType  vfc::internSqrt   (ValueType f_value, false_t)
    {
        vfc::float32_t l_value = static_cast<vfc::float32_t>(f_value);
#if defined(VFC_COMPILER_ARMRVCT) && defined(VFC_ARM_DETECTED) && (__ARMCC_VERSION >= 410713) && defined(VFC_HAS_ARM_NEON)
            __asm
            {
                vsqrt.f32 l_value, l_value
            }
            return l_value;
#else
    return ::sqrt(l_value); 
#endif
    }


    template <class T>
    inline T           vfc::clampValueToMinMax (const T& f_min, const T& f_value, const T& f_max)
    {

        return (vfc::intern::clampValueToMinMax(f_min, f_value, f_max,
            typename TInt2Boolean<vfc::TIsSameType<T,vfc::float32_t>::value>::type()));

    }

    inline
    vfc::float64_t    vfc::log    (float64_t f_value)
    {
        VFC_REQUIRE(0. < f_value);
        return ::log(f_value);
    }

    inline
    vfc::float64_t    vfc::log10    (float64_t f_value)
    {
        VFC_REQUIRE(0. < f_value);
        return ::log10(f_value);
    }

    inline
    vfc::float64_t   vfc::curt  (float64_t f_value)
    {
        return ( f_value >= 0. ) ? ::pow (f_value,G_1_3) : -::pow (-f_value, G_1_3);
    }

    template <class T>  inline
    vfc::int32_t     vfc::signum    (const T& f_value)
    {
        return (f_value>static_cast<T>(0))-(f_value<static_cast<T>(0));
    }

    template <class T>    inline
    vfc::float64_t    vfc::heaviside   (const T& f_value)
    {
        return 0.5*static_cast<float64_t>(vfc::signum(f_value))+0.5;
    }

    template <class T>    inline
    T   vfc::sqr   (const T& f_value)
    {
        return f_value*f_value;
    }

//=============================================================================
// alignment
//=============================================================================

    inline
    bool    vfc::isPow2      (size_t f_value)
    {
        return ( (0 == (f_value & (f_value - 1))) && (0 != f_value));
    }

    inline
    vfc::size_t  vfc::alignUp     (size_t f_value, size_t f_alignment)
    {
        // check precondition: alignment is power of two
        VFC_REQUIRE(isPow2(f_alignment));
        return (f_value + (f_alignment-1)) & (~(f_alignment-1));
    }

    inline
    vfc::size_t  vfc::alignDown   (size_t f_value, size_t f_alignment)
    {
        // check precondition: alignment is power of two
        VFC_REQUIRE(isPow2(f_alignment));
        return f_value & ~(f_alignment-1);
    }

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_math.inl  $
//  Revision 1.55 2016/01/27 14:59:32MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Support ARM v6 Compilerfor ARMv8 (mantis0004492)
//  Revision 1.54 2014/08/15 15:58:19MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - add isqrt function to vfc_math (mantis0004538)
//  Revision 1.53 2014/07/25 13:01:08MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_math uses NEON intrinsics, but only checks for ARM target, not CPU specific (mantis0004628)
//  Revision 1.52 2014/01/28 11:47:25MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_math: fix compiler warnings with arm v5 compiler (mantis0004392)
//  Revision 1.51 2013/09/19 12:46:00MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - int64_t is used in vfc_math.inl without check (mantis0004299)
//  Revision 1.50 2013/01/16 12:10:16MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - fast_rsqrt(float64) is not calculating the reciprocal square root but the square root only (mantis 4203)
//  Revision 1.49 2012/12/18 08:27:31MEZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.48 2012/01/26 14:19:10MEZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed win nint() implementation from inline assembly to sse intrinsics (mantis 3850)
//  Revision 1.47 2012/01/10 13:09:17MEZ Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) 
//  - Removal of Compiler Warning(mantis 4058)
//  Revision 1.46 2011/12/09 19:59:26IST Sudhakar Nannapaneni (RBEI/ESB3) (SNU5KOR) 
//  - Changes done to make the sqrt method as a generic one (mantis - 3811)
//  Revision 1.45 2011/06/20 09:49:05IST Gaurav Jain (RBEI/ESB3) (gaj2kor)
//  -Added VFP instructions on ARM for clamp related functions. (Mantis : 003811).
//  Revision 1.44 2011/05/18 20:50:58IST Muehlmann Karsten (CC/ESV2) (MUK2LR)
//  - mantis3090 (NaN tests on ARM failing)
//  - mantis3749 (nint optimization for ARM)
//  Revision 1.43 2009/05/28 09:19:04CEST Muehlmann Karsten (CC/ESV2) (MUK2LR)
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.42 2009/02/02 16:05:02CET Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Removal of QAC++ warnings.
//  (Mantis : 0002500)
//  Revision 1.41 2009/01/22 16:36:31IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Remove the unused variables.
//  (Mantis : 0002461)
//  Revision 1.40 2008/10/23 19:58:39IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - fixed wrong std:: usage (mantis 2406)
//  Revision 1.39 2008/09/02 16:42:12CEST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  Addition of VFC_STATIC_ASSERT(TIsFloating<T>::value); in IsEqual function.
//  (Mantis : 1700)
//  Revision 1.38 2008/09/01 19:27:57IST Dhananjay N (RBEI/EAE6) (dhn1kor)
//  assertion added to  vfc::intern::integer_abs (const T& f_value, false_t)  function to test the condition
//   f_value equals (std::numeric_limits<T>::min)(). VFC_REQUIRE2  is used as assertion.(mantis1790)
//
//  Precedence confusion in  QAC++ 2.5. Warning Rule 8.0.3 Removed.(mantis2219)
//  Revision 1.37 2008/08/29 16:27:17IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - moved swap and sort implementation to vfc_algorithm.{hpp,inl} (mantis 2323)
//  - rewrote vfc::swap (mantis 2323)
//  Revision 1.36 2008/08/25 13:28:57CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added log10 wrapper with pre-condition check (mantis2312)
//  - added some docu
//  Revision 1.35 2008/08/11 13:52:45CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - rewrote conditional nint() compilation (mantis2065)
//  Revision 1.34 2007/11/21 16:00:06CET Muehlmann Karsten (AE-DA/ESV1) (muk2lr)
//  - move integer_abs() definition in front of first use inside abs() (mantis1763)
//  Revision 1.33 2007/10/30 17:21:43CET Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added static_cast to signum function (mantis1824)
//  Revision 1.32 2007/08/24 13:35:33CEST dkn2kor
//  - missing parenthesis added (mantis 1706)
//  Revision 1.31 2007/08/16 22:48:21IST Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  - replaced magic number "8" by CHAR_BIT (mantis1763)
//  Revision 1.30 2007/08/10 16:22:49CEST Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  vfc::abs() rework (mantis1763)
//  Revision 1.29 2007/08/10 10:06:39CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added fast_abs function for 64bit integral types (mantis1762)
//  Revision 1.28 2007/08/02 15:50:07CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added conditional doxygen documentation generation of vfc::intern (mantis1758)
//  - added sorting functions (mantis1757)
//  Revision 1.27 2007/06/26 14:28:14CEST Hissmann Michael (CR/AEM5) (ihm2si)
//  - added CLR support (new macro VFC_CLR_DETECTED) (mantis 0001721)
//  - fixed warning warning C4793 in vfc_math.inl (mantis 0001721)
//  Revision 1.26 2007/05/25 12:22:53CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - re-implemented signum() and heaviside() function (mantis1668)
//  Revision 1.25 2007/04/16 09:20:55CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added pre-conditions for math functions (mantis1575)
//  Revision 1.24 2007/03/29 13:31:15CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added isINF() function (mantis1535)
//  - fixed inconsistency between isEqual(0,epsilon) and isZero(epsilon) (mantis1536)
//  - replaced ::std scopae for libc functions with global scope (mantis1534)
//  Revision 1.23 2007/03/01 11:37:13CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - make use of the  VFC_PROCESSOR_XXX defines (mantis 1201)
//  Revision 1.22 2007/02/16 09:46:17CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - adaption to new errorhandling (mantis 1429)
//  Revision 1.21 2007/01/15 11:13:46CET Heiler Matthias (CR/AEM5) (HME2HI)
//  Longer name for internal type pruning union.  (mantis 1365, note 1519)
//  Revision 1.20 2007/01/09 11:19:59CET Heiler Matthias (CR/AEM5) (HME2HI)
//  Corrected comments (second try) and indentation.
//  Renamed the float-integer union "fi32_t" to "Ufi32_t" to conform to coding standards.
//  (mantis 1365)
//  Revision 1.19 2007/01/08 10:09:36CET Heiler Matthias (CR/AEM5) (HME2HI)
//  Moved new type in vfc::intern.  Resurrected lost comment in fast_rsqrt.  (mantis 1365)
//  Revision 1.18 2007/01/04 17:20:14CET Heiler Matthias (CR/AEM5) (HME2HI)
//  Fixed bug in fast_rsqrt.
//  Updated Makefiles to report more warnings.
//  Added target "test" to Makefiles.
//  (mantis 1365)
//  Revision 1.17 2006/11/23 10:40:47CET Lauer Paul-Sebastian (CR/AEM5) (lap2hi)
//  - added template version of clampNegValueToZero() (mantis1320)
//  Revision 1.16 2006/11/16 14:41:21CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.15 2006/11/06 10:36:03CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added missing type cast for avoiding compiler warnings (mantis1255)
//  Revision 1.14 2006/10/23 16:39:07CEST Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  missing include (mantis1227)
//  Revision 1.13 2006/10/09 09:52:04CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - changes to use VFC_COMPILER/PLATFORM_XXX instead of platform/compiler specific defines (mantis 1166)
//  Revision 1.12 2006/10/06 13:15:28CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - changed VFC_EPPC define to VFC_EPPC_DETECTED (mantis 1149)
//  Revision 1.11 2006/07/07 09:42:27CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  -added isEqual() function with user defined epsilon (mantis1118)
//  -added clampValueToMinMax() (mantis1119)
//  Revision 1.10 2006/07/03 18:42:42CEST Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  small improvement and doc in rsqrt test
//  Revision 1.9 2006/05/22 09:47:45CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  -added function isPow2() (mantis1081)
//  -changed pre-condition assertion ia alignUp() and alignDown() (mantis1082)
//  Revision 1.8 2006/05/19 17:58:29CEST Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  added assert to check alignment is power of two (mantis1077)
//  Revision 1.7 2006/03/03 10:43:55CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added missing <algorithm> include (mantis1018)
//  Revision 1.6 2006/02/24 15:49:37CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -fixed problem with argument order dependent results in isEqual() for floating point types (mantis1013)
//  -note: more documentation is necessary for isEqual() floating point implementaion
//  Revision 1.5 2006/01/27 15:52:22CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added alignUp() and alignDown() functions
//  Revision 1.4 2006/01/26 12:39:52CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added missing argument in notNegative() function (mantis:982)
//  Revision 1.3 2005/12/21 11:36:12CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -code cleanup
//  Revision 1.2 2005/10/28 11:00:21CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added clampNegValueToZero(), fast_abs()
//=============================================================================
