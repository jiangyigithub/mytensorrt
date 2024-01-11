//=================================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2007 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy, use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=================================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/core
//          Synopsis:
//  Target system(s):
//       Compiler(s): VS7.1
//=================================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=================================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: zvh2hi
//  Department: CR/AEM
//=================================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_math.hpp $
///     $Revision: 1.29 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/11/20 17:38:34MEZ $
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
//=================================================================================


#ifndef VFC_MATH_HPP_INCLUDED
#define VFC_MATH_HPP_INCLUDED

#include <limits>                    // used for numeric_limits<>

#include "vfc/core/vfc_types.hpp"    // used for fundamental types
#include "vfc/core/vfc_config.hpp"            // used for predefines for compiler-checks
#include "vfc/core/vfc_metaprog.hpp"

namespace vfc
{    // namespace vfc opened

//=============================================================================
//  DOYGEN DEFGROUP vfc_group_core_algorithms_math BEGIN
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_core_algorithms_math Math
/// @ingroup vfc_group_core_algorithms
/// @brief Math Functions and Constants.
/// @{
//=============================================================================

//=============================================================================
// definitions of useful mathematical constants
//=============================================================================

    // trig
    //-------------------------------------------------------------------------
    const float64_t G_PI        = 3.1415926535897932384626433832795;        //!< yes, its really pi!
    const float64_t G_2PI       = 2.*3.1415926535897932384626433832795;                                  //!< 2*pi
    const float64_t G_PI_2      = 3.1415926535897932384626433832795/2.;                                  //!< pi/2
    const float64_t G_PI_4      = 3.1415926535897932384626433832795/4.;                                  //!< pi/4
    const float64_t G_1_PI      = 1./3.1415926535897932384626433832795;                                  //!< 1/pi
    const float64_t G_2_PI      = 2./3.1415926535897932384626433832795;                                  //!< 2/pi

    const float64_t G_DEG2RAD   = 3.1415926535897932384626433832795/180.;                                //!< degree to radiant factor (pi/180)
    const float64_t G_RAD2DEG   = 180./3.1415926535897932384626433832795;                                //!< radiant to degree factor (180/pi)

    const float64_t G_1_3       = 1./3.;                                    //!< 1/3

    // sqrt
    //-------------------------------------------------------------------------
    const float64_t G_SQRT2     = 1.4142135623730950488016887242096;        //!< sqrt(2)
    const float64_t G_SQRT3     = 1.7320508075688772935274463415059;        //!< sqrt(3)
    const float64_t G_1_SQRT2   = 1./1.4142135623730950488016887242096;     //!< 1/sqrt(2)

    // e
    //-------------------------------------------------------------------------
    const float64_t G_E         = 2.7182818284590452353602874713527;        //!< e
    const float64_t G_LOG2E     = 1.44269504088896340736;                   //!< log2(e)
    const float64_t G_LOG10E    = 0.434294481903251827651;                  //!< log10(e)

    // ln
    //-------------------------------------------------------------------------
    const float64_t G_LN2       = 0.69314718055994530941723212145818;       //!< ln(2)
    const float64_t G_LN10      = 2.3025850929940456840179914546844;        //!< ln(10)

#ifndef VFC_NO_FLOAT
#ifndef VFC_PLATFORM_V8XX
    //! defines the difference between 1 and the smallest value greater than 1 that float64_t can represent.
    const float64_t G_EPSILON_F64    = stlalias::numeric_limits<float64_t>::epsilon();

    //! defines the difference between 1 and the smallest value greater than 1 that float32_t can represent.
    const float32_t G_EPSILON_F32    = stlalias::numeric_limits<float32_t>::epsilon();
#endif
#endif

//=============================================================================
// To segregate float32_t and float64_t to perform sqrt operation
//=============================================================================
    template<class ValueType>
    class TSqrtTraits
    {
    public :
        typedef typename vfc::TIf<vfc::TIsSameType<ValueType, vfc::float64_t>::value,vfc::float64_t,vfc::float32_t>::type ArgType;
    };


//=============================================================================
// typed constants
//=============================================================================

    //-------------------------------------------------------------------------
    //! returns zero with specified type.
    //! @par usage:
    //! @code
    //! float32_t zero_val = typedZero<float32_t>();
    //! @endcode
    //-------------------------------------------------------------------------
    template <class ValueType> inline
    ValueType   typedZero   (void) { return static_cast<ValueType>(0);}

    //-------------------------------------------------------------------------
    //! returns one with specified type.
    //! @sa typedZero()
    //-------------------------------------------------------------------------
    template <class ValueType>  inline
    ValueType   typedOne    (void) { return static_cast<ValueType>(1);}

//=============================================================================
// test for NAN
//=============================================================================

    //-------------------------------------------------------------------------
    //! returns true if given value is NAN, false otherwise.
    //-------------------------------------------------------------------------
    template <class T>
    bool    isNAN       (const T& f_value);

//=============================================================================
// test for +INF, -INF
//=============================================================================

    //-------------------------------------------------------------------------
    //! returns true if given value is +INF or -INF, false otherwise.
    //-------------------------------------------------------------------------
    template <class T>
    bool    isINF       (const T& f_value);

//=============================================================================
// tests for ZERO
//=============================================================================

    //-------------------------------------------------------------------------
    /// returns true if specified value equals zero, false otherwise.
    /// uses an appropriate epsilon for floating point values ]-epsilon,epsilon[
    //-------------------------------------------------------------------------
    template <class T>
    bool    isZero        (const T& f_value);

    //-------------------------------------------------------------------------
    /// returns true if specified value is NOT zero.
    /// uses an appropriate epsilon for floating point values
    /// [-inf,-epsilon] || [epsilon,+inf]
    //-------------------------------------------------------------------------
    template <class T>
    bool    notZero        (const T& f_value);

    //-------------------------------------------------------------------------
    /// returns true if specified value is greater zero ]0,+inf].
    /// uses an appropriate epsilon for floating point values [epsilon,+inf].
    //-------------------------------------------------------------------------
    template <class T>
    bool    isPositive    (const T& f_value);

    //-------------------------------------------------------------------------
    /// returns true if specified value is zero or less [-inf,0].
    /// uses an appropriate epsilon for floating point values [-inf,+epsilon[.
    /// @note   keep in mind, that a floating point @a val can have a positive sign
    ///         although notPositive(@a val) evaluates to @a true;
    //-------------------------------------------------------------------------
    template <class T>
    bool    notPositive    (const T& f_value);

    //-------------------------------------------------------------------------
    /// returns true, if specified value is less zero [-inf,0[ .
    /// uses an appropriate epsilon for floating point values [-inf,-epsilon] .
    //-------------------------------------------------------------------------
    template <class T>
    bool    isNegative    (const T& f_value);

    //-------------------------------------------------------------------------
    /// returns true, if specified value is zero or greater [0,+inf].
    /// uses an appropriate epsilon for floating point values ]-epsilon,+inf].
    /// @note   keep in mind, that a floating point @a val can have a negative sign
    ///         although notNegative(@a val) evaluates to @a true;
    //-------------------------------------------------------------------------
    template <class T>
    bool    notNegative    (const T& f_value);


//=============================================================================
// tests for EQUALITY
//=============================================================================

    //-------------------------------------------------------------------------
    /// returns true if specified values are equal, false otherwise,
    /// uses an adapting epsilon for floating point values.
    /// @note isEqual(infinity,infinity) fails.
    //-------------------------------------------------------------------------
    template <class T>
    bool    isEqual     (const T& f_value1, const T& f_value2);

    //-------------------------------------------------------------------------
    /// returns true if specified values are equal within given epsilon.
    //-------------------------------------------------------------------------
    template <class T>
    bool    isEqual     (const T& f_value1, const T& f_value2, const T& f_epsilon);

    //-------------------------------------------------------------------------
    /// returns true if specified values are NOT equal, false otherwise,
    /// uses an adapting epsilon for floating point values.
    /// @note notEqual(infinity,infinity) fails.
    /// @sa isEqual()
    //-------------------------------------------------------------------------
    template <class T>
    bool    notEqual    (const T& f_value1, const T& f_value2);

//=============================================================================
// checked divide
//=============================================================================

    //-------------------------------------------------------------------------
    /// returns numerator divided by denominator @f$ y= \frac{num}{denom} @f$
    /// @pre f_denom it not zero (within an epsilon for floating point types)
    /// @sa isZero()
    //-------------------------------------------------------------------------
    template <class T>
    T        divide      (const T& f_num, const T& f_denom);

//=============================================================================
// more mathematical functions
//=============================================================================

    //-------------------------------------------------------------------------
    /// rounds up the input of type vfc::float32_t to the greatest integer less
    /// than or equal to the input
    /// @return
    //-------------------------------------------------------------------------
    vfc::float32_t floor(vfc::float32_t f_value);

    //-------------------------------------------------------------------------
    /// rounds up the input of type vfc::float32_t to the least integer greater
    /// than or equal to the input
    /// @return
    //-------------------------------------------------------------------------
    vfc::float32_t ceil(vfc::float32_t f_value);

    //-------------------------------------------------------------------------
    /// clamps specified value of int32 to zero as lower bound
    /// @return @f[  y= \left \{ \begin{array}{cl} 0 & ,x<0 \\ x & ,x>=0  \\ \end{array} \right \}  @f]
    //-------------------------------------------------------------------------
    int32_t     clampNegValueToZero(int32_t f_value);

    //-------------------------------------------------------------------------
    /// clamps specified value of float32 to zero as lower bound
    /// @return @f[  y= \left \{ \begin{array}{cl} 0 & ,x<0 \\ x & ,x>=0  \\ \end{array} \right \}  @f]
    //-------------------------------------------------------------------------
    float32_t   clampNegValueToZero(float32_t f_value);

    //-------------------------------------------------------------------------
    /// clamps specified value of float64 to zero as lower bound
    /// @return @f[  y= \left \{ \begin{array}{cl} 0 & ,x<0 \\ x & ,x>=0  \\ \end{array} \right \}  @f]
    //-------------------------------------------------------------------------
    float64_t   clampNegValueToZero(float64_t f_value);

    //-------------------------------------------------------------------------
    /// clamps specified value to zero as lower bound
    /// @return @f[  y= \left \{ \begin{array}{cl} 0 & ,x<0 \\ x & ,x>=0  \\ \end{array} \right \}  @f]
    //-------------------------------------------------------------------------
    template <class T>
    T           clampNegValueToZero(const T& f_value);

    //-------------------------------------------------------------------------
    /// clamps specified value to upper and lower bound.
    /// @return @f[  y= \left \{ \begin{array}{cl} min & ,x<=min \\ x & ,min<x<max  \\ max & ,x>= max \\ \end{array} \right \}  @f]
    //-------------------------------------------------------------------------
    template <class T>
    T           clampValueToMinMax (const T& f_min, const T& f_value, const T& f_max);

    //-------------------------------------------------------------------------
    //! abs wrapper, uses fast_abs() for integral types
    //! @return @f[  y= \left \{ \begin{array}{rl} -x & ,x<0 \\ x & ,x>=0 \\ \end{array} \right \}  @f]
    //-------------------------------------------------------------------------
    template <class T>
    T           abs         (const T& f_value);

    //-------------------------------------------------------------------------
    //! rounds floating point value to nearest integer.
    //! The rounding method is round-to-nearest-even for the optimized path
    //! ( {cw,vc}-win32-x86 ) and round-half-up for the stdc fallback.
    //! @note  We will provide both rounding methods with dedicated functions
    //! in an upcoming release.
    //-------------------------------------------------------------------------
    int32_t     nint        (float64_t f_value);

    //-------------------------------------------------------------------------
    //! rounds floating point value to nearest integer.
    //! The rounding method is round-to-nearest-even for the optimized path
    //! ( {cw,vc}-win32-x86 ) and round-half-up for the stdc fallback.
    //! @note  We will provide both rounding methods with dedicated functions
    //! in an upcoming release.
    //-------------------------------------------------------------------------
    int32_t     nint        (float32_t f_value);

    //-------------------------------------------------------------------------
    //! returns an int32 approximation of square root of given int32 argument
    //! See also hackers delight:
    //! -# http://www.hackersdelight.org/hdcodetxt/isqrt.c.txt
    //-------------------------------------------------------------------------
    int32_t     isqrt       (int32_t f_value);

    //-------------------------------------------------------------------------
    //! returns a fast approximation of reciprocal square root of given float32 argument
    //! @f$ y \simeq \frac{1}{\sqrt{x}} @f$, only works with IEEE 754 floating point standard (32bit).
    //! The bit magic is accurate to ~0.175%. (~4% before used newton step) @n
    //! For an in depth discussion, see
    //! -# http://www.math.purdue.edu/~clomont/Math/Papers/2003/InvSqrt.pdf
    //! -# http://www.google.de/search?hl=de&q=0x5f3759df&meta=
    //! .
    //! @pre f_value has to be positive!
    //-------------------------------------------------------------------------
    float32_t   fast_rsqrt  (float32_t f_value);

    //-------------------------------------------------------------------------
    //! returns a fast approximation of reciprocal square root of given float64 argument
    //! @f$ y \simeq \frac{1}{\sqrt{x}} @f$, only works with IEEE 754 floating point standard (64bit).
    //! See
    //! -# // http://en.wikipedia.org/wiki/Fast_inverse_square_root
    //! .
    //! @pre f_value hast to be positive!
    //-------------------------------------------------------------------------
#ifndef VFC_NO_INT64
    float64_t   fast_rsqrt  (float64_t f_value);
#endif

    //-------------------------------------------------------------------------
    //! returns the square root of given value of type "ValueType" @f$ y= \sqrt{x} @f$.
    //! @pre f_value hast to be (exact) zero or positive
    //-------------------------------------------------------------------------
    template<class ValueType>
    inline
    typename TSqrtTraits<ValueType>::ArgType  sqrt   (ValueType f_value);

    //-------------------------------------------------------------------------
    //! returns the square root of given value of type float64_t
    //-------------------------------------------------------------------------
    template<class ValueType>
    inline
        typename TSqrtTraits<ValueType>::ArgType  internSqrt   (ValueType f_value, vfc::true_t);

    //-------------------------------------------------------------------------
    //! returns the square root of given value of type float32_t
    //-------------------------------------------------------------------------
    template<class ValueType>
    inline
    typename TSqrtTraits<ValueType>::ArgType  internSqrt   (ValueType f_value, vfc::false_t);

    //-------------------------------------------------------------------------
    //! returns the natural logarithm (base e) of given value.
    //! @pre f_value hast to be positive
    //-------------------------------------------------------------------------
    float64_t   log         (float64_t f_value);

    //-------------------------------------------------------------------------
    //! returns the base-10 logarithm of specified value.
    //! @pre f_value hast to be positive
    //-------------------------------------------------------------------------
    float64_t   log10       (float64_t f_value);

    //-------------------------------------------------------------------------
    //! returns the cubic root of given value.
    //-------------------------------------------------------------------------
    float64_t   curt        (float64_t f_value);

    //-------------------------------------------------------------------------
    /// signum function.
    /// @return -1 for negative values, 1 for positive and zero if specified value is zero.
    /// @f[  y= \left \{ \begin{array}{rl} -1 & ,x<0 \\ 0 & ,x=0 \\ 1 & ,x>0 \\ \end{array} \right \}  @f]
    //-------------------------------------------------------------------------
    template <class T>  inline
    int32_t     signum      (const T& f_value);


    //-------------------------------------------------------------------------
    /// heaviside function.
    /// @return 0.0 for negative values, 1.0 for positive and 0.5 if value is zero.
    /// @f[  y= \left \{ \begin{array}{cl} 0.0 & ,x<0 \\ 0.5 & ,x=0 \\ 1.0 & ,x>0 \\ \end{array} \right \}  @f]
    //-------------------------------------------------------------------------
    template <class T>    inline
    float64_t   heaviside   (const T& f_value);

    //-------------------------------------------------------------------------
    //! returns given argument squared @f$ y = x^2 @f$.
    //-------------------------------------------------------------------------
    template <class T>    inline
    T           sqr         (const T& f_value);

//=============================================================================
// numeric casts
//=============================================================================

    //-------------------------------------------------------------------------
    /// casts a arithmetic value to a specified type, rounds to nearest integer
    /// if necessary.
    /// @par usage:
    /// @code
    /// int32_t val_i32 = numeric_cast<int32_t>(3.789);
    /// @endcode
    //-------------------------------------------------------------------------
    template <class ReturnType, class ArgumentType>
    ReturnType    numeric_cast(const ArgumentType& f_value);

//=============================================================================
// alignment
//=============================================================================

    //-------------------------------------------------------------------------
    //! returns true if specified value is power of two.
    //-------------------------------------------------------------------------
    bool    isPow2      (size_t f_value);

    //-------------------------------------------------------------------------
    //! aligns value upwards the nearest multiple of alignment, alignment has
    //! to be a power-of-two.
    //-------------------------------------------------------------------------
    size_t  alignUp     (size_t f_value, size_t f_alignment);

    //-------------------------------------------------------------------------
    //! aligns value downwards the nearest multiple of alignment, alignment has
    //! to be a power-of-two.
    //-------------------------------------------------------------------------
    size_t  alignDown   (size_t f_value, size_t f_alignment);

//=============================================================================
//  DOYGEN DEFGROUP vfc_group_core_algorithms_math END
//-------------------------------------------------------------------------
/// @}
//=============================================================================

}    // namespace vfc closed

#include "vfc/core/vfc_math.inl"

#endif //VFC_MATH_HPP_INCLUDED


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_math.hpp  $
//  Revision 1.29 2014/11/20 17:38:34MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_math uses vfc_config predefines for compiler-checks, but does not #include vfc_config.hpp (mantis0004687)
//  Revision 1.28 2014/08/15 15:57:12MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - add isqrt function to vfc_math (mantis0004538)
//  Revision 1.27 2013/09/19 12:46:38MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - int64_t is used in vfc_math.inl without check (mantis0004299)
//  Revision 1.26 2013/01/16 12:19:47MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - fast_rsqrt(float64) is not calculating the reciprocal square root but the square root only (mantis 4203)
//  Revision 1.25 2012/12/18 08:27:31MEZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.24 2011/12/05 12:18:23MEZ Sudhakar Nannapaneni (RBEI/ESB3) (SNU5KOR) 
//  - Changes done to make the sqrt method as a generic one (mantis - 3811)
//  Revision 1.23 2011/06/20 09:47:27IST Gaurav Jain (RBEI/ESB3) (gaj2kor)
//  -Added VFP instructions on ARM for clamp related functions. (Mantis : 003811).
//  Revision 1.22 2011/05/18 20:50:27IST Muehlmann Karsten (CC/ESV2) (MUK2LR)
//  - mantis3090 (NaN tests on ARM failing)
//  - mantis3749 (nint optimization for ARM)
//  Revision 1.21 2010/10/13 13:44:29MESZ Jaeger Thomas (CC/EPV2) (JAT2HI)
//  - numerical constants are not ROMable (mantis3403)
//  Revision 1.20 2010/09/09 09:15:05MESZ Jaeger Thomas (CC/EPV2) (JAT2HI)
//  - numerical constants are not ROMable (mantis3403)
//  Revision 1.19 2008/08/29 16:27:17IST Zitzewitz Henning von (CR/AEM6) (ZVH2HI)
//  - moved swap and sort implementation to vfc_algorithm.{hpp,inl} (mantis 2323)
//  - rewrote vfc::swap (mantis 2323)
//  Revision 1.18 2008/08/25 13:28:56CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added log10 wrapper with pre-condition check (mantis2312)
//  - added some docu
//  Revision 1.17 2007/08/10 16:22:49CEST Muehlmann Karsten (CC-DA/ESV1) (muk2lr)
//  vfc::abs() rework (mantis1763)
//  Revision 1.16 2007/08/10 10:06:37CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added fast_abs function for 64bit integral types (mantis1762)
//  Revision 1.15 2007/08/02 15:50:07CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added conditional doxygen documentation generation of vfc::intern (mantis1758)
//  - added sorting functions (mantis1757)
//  Revision 1.14 2007/07/23 09:31:22CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - doxygen grouping (mantis1744)
//  Revision 1.13 2007/06/22 15:12:25CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - corrected redeclared function parameter names (mantis 1691)
//  Revision 1.12 2007/04/16 09:15:45CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added docu (mantis1377)
//  - replaced old header/footer
//  Revision 1.11 2007/03/29 13:23:43CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added docu (mantis1377)
//  - added isINF() function (mantis1535)
//  - added docu for isEqual() infinity failure (mantis1379)
//  Revision 1.10 2006/11/23 10:40:20CET Lauer Paul-Sebastian (CR/AEM5) (lap2hi)
//  - added template version of clampNegValueToZero() (mantis1320)
//  Revision 1.9 2006/11/16 14:41:14CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.8 2006/07/07 09:42:27CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  -added isEqual() function with user defined epsilon (mantis1118)
//  -added clampValueToMinMax() (mantis1119)
//  Revision 1.7 2006/05/22 09:47:45CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  -added function isPow2() (mantis1081)
//  -changed pre-condition assertion ia alignUp() and alignDown() (mantis1082)
//  Revision 1.6 2006/01/27 15:52:21CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added alignUp() and alignDown() functions
//  Revision 1.5 2005/12/21 11:36:11CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -code cleanup
//  Revision 1.4 2005/11/02 14:19:51CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added isPositive(), notPositive, isNegative() and notNegative() funcs
//  Revision 1.3 2005/11/02 09:41:52CET Muehlmann Karsten (AE-DA/ESA3) * (MUK2LR)
//  take sqrt() always from ::std
//  Revision 1.2 2005/10/28 11:00:21CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added clampNegValueToZero(), fast_abs()
//=============================================================================
