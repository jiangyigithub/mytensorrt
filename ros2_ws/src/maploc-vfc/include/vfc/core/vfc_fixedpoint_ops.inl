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
///     $Source: include/vfc/core/vfc_fixedpoint_ops.inl $
///     $Revision: 1.58 $
///     $Author: Gaurav Jain (RBEI/ESD1) (gaj2kor) $
///     $Date: 2009/10/01 12:21:12MESZ $
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

#include <cmath>

#include "vfc/core/vfc_math.hpp"
#include "vfc/core/vfc_fixedpoint.hpp"
#include "vfc/core/vfc_fixedpoint_types.hpp"
#include "vfc/core/vfc_fixedpoint_lut_math.hpp"

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType> vfc::operator-(
                                                        const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return vfc::TFixedPoint<FracBitsValue, ValueType>(-f_value.fixedpoint(),
                                                      typename vfc::TFixedPoint<FracBitsValue, ValueType>::CNoShift());
}

template <class ResultType, vfc::int32_t FracBitsValue, class ValueType>
inline ResultType  vfc::numeric_cast(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return vfc::intern::numeric_cast<ResultType>(f_value,
                                             typename vfc::TInt2Boolean<vfc::TIsFloating<ResultType>::value>::type());
}

///  the nint function returns the nearest integer of specified value
template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::int32_t vfc::nint    (const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return f_value.to_nint();
}

/// the floor function returns an integer value representing the largest integer
/// that is less than or equal to specified value.
template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::int32_t vfc::floor   (const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return f_value.to_floor();
}

/// the ceil function an integer value representing the smallest integer
/// that is greater than or equal to specified value.
template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::int32_t vfc::ceil    (const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return f_value.to_ceil();
}

//================================================================================
// vfc::sqrt
//--------------------------------------------------------------------------------
/// returns the square root of specified fixpoint value.
/// @par Description:
/// The following comes from Graphics Gems V, gem I.3
/// It is iterative, with the precision basically doubling every iteration.
/// @note The number of fractional bits MUST be divisible by 2.
//================================================================================
template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>  vfc::sqrt(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    // test if fractional bits is divisable by 2
    VFC_STATIC_ASSERT (!(FracBitsValue & 0x01));

     typedef typename vfc::TIf<(!vfc::TIsSameType< ValueType, vfc::int64_t >::value),
                                vfc::uint32_t,
                                vfc::uint64_t>::type uType_t;
     uType_t counter = (vfc::TIsSameType< ValueType, vfc::int64_t >::value) ? 31 : 15;  // for 32bit or less types
     uType_t bits    = (vfc::TIsSameType< ValueType, vfc::int64_t >::value) ? 62 : 30;  // for 64bit

    // test ist argument is negative
    VFC_REQUIRE(0 <= f_value.fixedpoint());

    uType_t root    = static_cast<uType_t>(0);                       // clear root
    uType_t remHi   = static_cast<uType_t>(0);                       // clear high part of partial remainder
    uType_t remLo   = static_cast<uType_t>(f_value.fixedpoint());    // get argument into low part of partial remainder
    uType_t testDiv = static_cast<uType_t>(0);                       // clear test
    uType_t count   = counter + static_cast<uType_t>(FracBitsValue >> 1); // iterations

    do
    {
        remHi = (remHi << 2) | (remLo >> bits);   // get 2 bits of arg
        remLo <<= 2;
        root  <<= 1;                     // Get ready for the next bit in the root
        testDiv = (root << 1) + 1;      // Test radical

        if (remHi >= testDiv)
        {
            remHi -= testDiv;
            root += 1;
        }
    }
    while (0 != count--);

    return vfc::TFixedPoint<FracBitsValue, ValueType>(static_cast<ValueType>(root),
                      typename vfc::TFixedPoint<FracBitsValue, ValueType>::CNoShift() );
}

namespace
{   // namespace open

    //================================================================================
    // vfc::int_sqrt
    //--------------------------------------------------------------------------------
    /// returns the square root of specified fixpoint value with integer precision
    /// @par Description:
    /// The following comes from http://www.azillionmonkeys.com/qed/sqroot.html
    /// not the right place for this function, has to be moved to vfc_core
    /// no fixedpoint dependancy
    //================================================================================
    const vfc::uint32_t G_SQQ_TABLE[] = {
       0U,  16U,  22U,  27U,  32U,  35U,  39U,  42U,  45U,  48U,  50U,  53U,  55U,  57U,
      59U,  61U,  64U,  65U,  67U,  69U,  71U,  73U,  75U,  76U,  78U,  80U,  81U,  83U,
      84U,  86U,  87U,  89U,  90U,  91U,  93U,  94U,  96U,  97U,  98U,  99U, 101U, 102U,
     103U, 104U, 106U, 107U, 108U, 109U, 110U, 112U, 113U, 114U, 115U, 116U, 117U, 118U,
     119U, 120U, 121U, 122U, 123U, 124U, 125U, 126U, 128U, 128U, 129U, 130U, 131U, 132U,
     133U, 134U, 135U, 136U, 137U, 138U, 139U, 140U, 141U, 142U, 143U, 144U, 144U, 145U,
     146U, 147U, 148U, 149U, 150U, 150U, 151U, 152U, 153U, 154U, 155U, 155U, 156U, 157U,
     158U, 159U, 160U, 160U, 161U, 162U, 163U, 163U, 164U, 165U, 166U, 167U, 167U, 168U,
     169U, 170U, 170U, 171U, 172U, 173U, 173U, 174U, 175U, 176U, 176U, 177U, 178U, 178U,
     179U, 180U, 181U, 181U, 182U, 183U, 183U, 184U, 185U, 185U, 186U, 187U, 187U, 188U,
     189U, 189U, 190U, 191U, 192U, 192U, 193U, 193U, 194U, 195U, 195U, 196U, 197U, 197U,
     198U, 199U, 199U, 200U, 201U, 201U, 202U, 203U, 203U, 204U, 204U, 205U, 206U, 206U,
     207U, 208U, 208U, 209U, 209U, 210U, 211U, 211U, 212U, 212U, 213U, 214U, 214U, 215U,
     215U, 216U, 217U, 217U, 218U, 218U, 219U, 219U, 220U, 221U, 221U, 222U, 222U, 223U,
     224U, 224U, 225U, 225U, 226U, 226U, 227U, 227U, 228U, 229U, 229U, 230U, 230U, 231U,
     231U, 232U, 232U, 233U, 234U, 234U, 235U, 235U, 236U, 236U, 237U, 237U, 238U, 238U,
     239U, 240U, 240U, 241U, 241U, 242U, 242U, 243U, 243U, 244U, 244U, 245U, 245U, 246U,
     246U, 247U, 247U, 248U, 248U, 249U, 249U, 250U, 250U, 251U, 251U, 252U, 252U, 253U,
     253U, 254U, 254U, 255
    };
}   // namespace closed

inline vfc::uint32_t vfc::int_sqrt(const vfc::uint32_t x)
{
    VFC_REQUIRE(x < 0x1000000);
    vfc::uint32_t xn = 0;
    if (x >= 0x10000)
    {
        if (x >= 0x100000)
        {
            if (x >= 0x400000)
            {
                xn = G_SQQ_TABLE[x>>16U] << 4U;
            }
            else
            {
                xn = G_SQQ_TABLE[x>>14U] << 3U;
            }
        }
        else
        {
            if (x >= 0x40000)
            {
                xn = G_SQQ_TABLE[x>>12U] << 2U;
            }
            else
            {
                xn = G_SQQ_TABLE[x>>10U] << 1U;
            }
            xn = (xn + 1U + x / xn) >> 1U;
            if ((xn * xn) > x) // Correct rounding if necessary
            {
                --xn;
            }

            return xn;
        }
    }
    else
    {
        if (x >= 0x100)
        {
            if (x >= 0x1000)
            {
                if (x >= 0x4000)
                {
                    xn = (G_SQQ_TABLE[x>>8U] >> 0U) + 1U;
                }
                else
                {
                    xn = (G_SQQ_TABLE[x>>6U] >> 1U) + 1U;
                }
            }
            else
            {
                if (x >= 0x400)
                {
                    xn = (G_SQQ_TABLE[x>>4U] >> 2U) + 1U;
                }
                else
                {
                    xn = (G_SQQ_TABLE[x>>2U] >> 3U) + 1U;
                }

               if ((xn * xn) > x) // Correct rounding if necessary
               {
                    --xn;
               }
               return xn;
            }
        }
        else
        {
            return G_SQQ_TABLE[x] >> 4U;
        }

        // Run two iterations of the standard convergence formula
        xn = (xn + 1U + x / xn) >> 1U;
        xn = (xn + 1U + x / xn) >> 1U;
        if ((xn * xn) > x) // Correct rounding if necessary
        {
            --xn;
        }
        return xn;
    }
    return xn;
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>  vfc::sqr(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return (f_value * f_value);
}



template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>  vfc::abs(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return vfc::TFixedPoint<FracBitsValue, ValueType>(vfc::abs(f_value.fixedpoint()),
                                           typename vfc::TFixedPoint<FracBitsValue, ValueType>::CNoShift());
}
#ifndef    VFC_NO_FLOAT
template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType> vfc::exp(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return vfc::intern::fixedlut_exp(f_value); // LUT O(1)
}
#endif

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType> vfc::log(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return vfc::intern::fixedlut_log(f_value); // LUT O(1)
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>  vfc::sin(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return vfc::intern::fixedlut_sin(f_value); // LUT O(1), internal implementation using cos lut
}


template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>  vfc::cos(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return vfc::intern::fixedlut_cos(f_value); // LUT O(1)
}


template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType> vfc::tan(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return vfc::intern::fixedlut_tan(f_value); // LUT O(1), internal implementation using sin/cos lut
}

#ifndef    VFC_NO_FLOAT
// pow(a, b) = exp(b * log(a)) should not be a large value for accuracy
template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType> vfc::pow(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_base,
    const vfc::TFixedPoint<FracBitsValue, ValueType>& f_exponent)
{
    return vfc::intern::fixedlut_pow(f_base, f_exponent); // LUT O(1)
}
#endif

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>  vfc::fmod(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_x,
                                                             const vfc::TFixedPoint<FracBitsValue, ValueType>& f_y)
{
    return vfc::TFixedPoint<FracBitsValue, ValueType>(
                                                (f_x.fixedpoint() % f_y.fixedpoint()),
                                                typename vfc::TFixedPoint<FracBitsValue, ValueType>::CNoShift());
}


template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType> vfc::atan2(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_dy,
    const vfc::TFixedPoint<FracBitsValue, ValueType>& f_dx)
{
    typedef vfc::TFixedPoint<FracBitsValue, ValueType> fixedpoint_type;

    VFC_STATIC_ASSERT(fixedpoint_type::INTBITS >= 2); // at least space needed to store pi

    fixedpoint_type atan_fp;

    if(f_dy.isDivisionSafe(f_dx))
    {
        atan_fp = vfc::atan(f_dy / f_dx);

        if (vfc::isNegative(f_dx))
        {
            if (vfc::isNegative(f_dy))
            {
                atan_fp -= vfc::TFixedPointValuePolicy<fixedpoint_type>::G_PI();
            }
            else
            {
                atan_fp += vfc::TFixedPointValuePolicy<fixedpoint_type>::G_PI();
            }
        }
    }
    else
    {
        if ( ((vfc::isNegative(f_dx)) && !(vfc::isNegative(f_dy))) ||
            (!(vfc::isNegative(f_dx)) && (vfc::isNegative(f_dy))) )
        {
            // ((-dx, dy) Or (dx ,-dy))
            atan_fp = (-vfc::TFixedPointValuePolicy<fixedpoint_type>::G_PI_2());
        }
        else
        {
            // ((-dx, -dy) Or (dx ,dy))
            atan_fp = (vfc::TFixedPointValuePolicy<fixedpoint_type>::G_PI_2());
        }
    }

    return (atan_fp);
}


template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType> vfc::atan(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return vfc::intern::atan(f_value);
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>  vfc::asin(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return vfc::intern::fixedlut_asin(f_value);  // O(log(n)), n size of lut
}


template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>  vfc::acos(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return vfc::intern::fixedlut_acos(f_value); // O(log(n)), n size of lut
}


template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>  vfc::min(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_lhs,
                                                            const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs)
{
    return vfc::TFixedPoint<FracBitsValue, ValueType>(stlalias::min(f_lhs.fixedpoint(), f_rhs.fixedpoint()),
                                                  typename vfc::TFixedPoint<FracBitsValue, ValueType>::CNoShift());
}


template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>  vfc::max(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_lhs,
                                                            const vfc::TFixedPoint<FracBitsValue, ValueType>& f_rhs)
{
    return vfc::TFixedPoint<FracBitsValue, ValueType>(stlalias::max(f_lhs.fixedpoint(), f_rhs.fixedpoint()),
                                                typename vfc::TFixedPoint<FracBitsValue, ValueType>::CNoShift());
}


template <vfc::int32_t FracBitsValue, class ValueType>
inline bool  vfc::isEqual(const TFixedPoint<FracBitsValue, ValueType>& f_value1,
                                                                const TFixedPoint<FracBitsValue, ValueType>& f_value2,
                                                                const TFixedPoint<FracBitsValue, ValueType>& f_delta)
{
    if (vfc::abs(f_value1 - f_value2) > f_delta)
    {
        return false;
    }

    return true;
    //return (vfc::isEqual(f_value1.fixedpoint(), f_value2.fixedpoint(), f_delta.fixedpoint()));
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline bool  vfc::isPositive(const TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return (vfc::isPositive(f_value.fixedpoint()));
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline bool  vfc::isNegative(const TFixedPoint<FracBitsValue, ValueType>& f_value)
{
     return (vfc::isNegative(f_value.fixedpoint()));
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline bool  vfc::isZero(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return (vfc::isZero(f_value.fixedpoint()));
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline bool  vfc::notZero(const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    return (!vfc::isZero(f_value.fixedpoint()));
}

template <vfc::int32_t FracBitsValue, class ValueType>
inline vfc::TFixedPoint<FracBitsValue, ValueType>  vfc::signum (
                                                        const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
{
    typedef vfc::TFixedPoint<FracBitsValue, ValueType> fixedpoint_type;
    return fixedpoint_type((f_value > fixedpoint_type(static_cast<ValueType>(0),
                            typename fixedpoint_type::CNoShift())) -
                           (f_value < fixedpoint_type(static_cast<ValueType>(0),
                            typename fixedpoint_type::CNoShift())));
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: include/vfc/core/vfc_fixedpoint_ops.inl  $
//  Revision 1.58 2009/10/01 12:21:12MESZ Gaurav Jain (RBEI/ESD1) (gaj2kor) 
//  - Clean separation of fpu-code and fpu-free code using VFC_NO_FLOAT.
//  Revision 1.57 2009/03/25 20:29:47GMT+05:30 Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  - Linebreak at 120 in all fixedpoint files. (mantis 0002692)
//  Revision 1.56 2009/02/04 18:43:58IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Removal of QAC++ warnings.
//  (Mantis : 0002499)
//  Revision 1.55 2009/01/19 13:34:59IST Voelz Henning (CC-DA/ESV4) (VOH2HI)
//  make TFixedpoint compile with vfc cp 1.12 mantis:0002527
//  Revision 1.54 2008/10/21 15:41:13CEST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  -Removal of sign limitation from vfc::atan2().
//  (Mantis :2371)
//  Revision 1.53 2008/10/15 14:31:23IST Mayer Bjoern (CC-DA/ESV1) (MAB2LR)
//  improvement in atan2: returning -pi/2 in appropriate situations
//  Revision 1.52 2008/04/09 11:23:24CEST Voelz Henning (CC-DA/ESV1) (VOH2HI)
//  bugfix
//  Revision 1.51 2008/04/09 11:18:10CEST Voelz Henning (AE-DA/ESV1) (VOH2HI)
//  variable names changed
//  Revision 1.50 2008/04/04 10:57:33CEST Voelz Henning (AE-DA/ESV1) (VOH2HI)
//  fixedlut for acos
//  Revision 1.49 2008/04/04 10:41:28CEST Voelz Henning (AE-DA/ESV1) (VOH2HI)
//  use fixedlut for atan
//  Revision 1.48 2008/03/12 11:02:52CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (SF71LR)
//  promotion_type and lut_operation_type are now public in fixedpoint. lut algorithm are now using the lut_operation_type which is 64bit for 32bit and 64bit fixedpoint. reworked atan so it can handle fixedpoint types with less fraction bits.
//  Revision 1.47 2008/02/28 09:21:40CET Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  qac++,codecoverage testing done
//  Revision 1.46 2008/02/20 11:13:02IST Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  Removal of OpearationPolicy
//  Revision 1.45 2008/01/30 23:26:30IST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  asin / acos now use a normal lut for the implementation.
//  Revision 1.44 2008/01/28 16:05:22CET Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  Done TypeCasting for const values
//  Revision 1.43 2008/01/17 15:46:29IST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  internal namespace changed to intern;
//  gcc warnings removed
//  gcc error (typename) removed
//  Revision 1.42 2008/01/16 17:27:09CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Moved atan from vfc_fixedpoint_trig.hpp to vfc_fixedpoint_lut_math.inl.
//  Removed all trig solutions that are not used in the ldw project. vfc_fixedpoint_lut_math.hpp is now the only header, which contains trig algorithm.
//  Removed all little template helpers from vfc_fixedpoint_trig_helper.hpp.
//  Revision 1.41 2008/01/16 15:57:39CET Voelz Henning (AE-DA/ESV1) (voh2hi)
//  methods redirected to LUT implementations
//  Revision 1.40 2007/12/20 13:06:45CET Voelz Henning (AE-DA/ESV1) (voh2hi)
//  use lut implementation for sin/cos/tan
//  Revision 1.39 2007/12/20 08:39:02CET Voelz Henning (AE-DA/ESV1) (voh2hi)
//  #pragma warning removed
//  Revision 1.38 2007/12/19 15:42:49CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Exposed lut based trig functions to vfc:: namespace.
//  Revision 1.37 2007/12/19 14:50:36CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Added log to ops.
//  Revision 1.36 2007/12/10 18:01:59CET Renner Christian (AE-DA/ESV1) (rec1lr)
//  secured atan2
//  Revision 1.35 2007/12/10 13:55:20CET Renner Christian (AE-DA/ESV1) (rec1lr)
//  changed call of atan calculation (more accuracy at small values)
//  Revision 1.34 2007/12/04 16:17:40CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Changed static const intergral to enum type -> Needed cast for constructor. Moved all definitions and helping calculations from vfc_fixedpoint_trig to vfc_fixedpoint_trig_helper.hpp.
//  Revision 1.33 2007/11/26 14:48:19CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Added test for arcsin/arccos.
//  Revision 1.32 2007/11/26 14:28:50CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Arcsin / arccos implemention.
//  Revision 1.30 2007/11/26 09:31:27CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Better usage for pow(a, b). Exp now has a template computed return type.
//  Revision 1.29 2007/11/19 16:26:30CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Made vfc::exp vfc::pow for fixedpoint more usable.
//  vfc::pow has some restrictions about the maximal value that is allowed. (pow(a, b) == exp(b * log(a)) : b * log(a) must fit into a fp32p28).
//  CppUnit test now testing nearly all function. Still some test need a rewrite.
//  Revision 1.28 2007/11/19 08:38:00CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Missing vfc::
//  Revision 1.27 2007/11/14 15:05:41CET Voelz Henning (AE-DA/ESV1) (voh2hi)
//  compiler warnings removed
//  Revision 1.25 2007/11/12 17:54:42CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Fix in exp function.
//  Revision 1.24 2007/11/12 17:35:46CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Modified fixedpoint pow. Now the user have to take care about the return type and the maximum value.
//  Revision 1.23 2007/11/12 11:33:39CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Made atan more usable (but if using it with too high values it becomes more inaccurate!). fixedpoint ops now work with intern method.
//  Revision 1.22 2007/10/31 15:19:05CET Renner Christian (AE-DA/ESV1) (rec1lr)
//  using fixedpoint pow()
//  Revision 1.21 2007/10/30 14:11:59CET Renner Christian (AE-DA/ESV1) (rec1lr)
//  temporary check in for int_sqrt function (has to be moved o vfc_core, because no fixedpoint dependency)
//  Revision 1.20 2007/10/24 10:34:01CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  bug fix tan<>
//  Revision 1.19 2007/10/23 10:27:32CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  gcc error fixed
//  Revision 1.18 2007/10/23 10:08:09CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  vfc::signum() added
//  Revision 1.17 2007/10/22 13:31:18CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  interface adaptions to new fixedpoint_trig implementation
//  Revision 1.16 2007/10/22 11:04:41CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  notZero added
//  Revision 1.15 2007/10/22 10:25:13CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  bugfix sin / tan
//  Revision 1.14 2007/10/22 10:02:13CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  sin / cos / tan with template taylor summands
//  Revision 1.13 2007/10/22 08:52:18CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  inlining
//  Revision 1.12 2007/10/19 16:09:03CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  sin / cos / tan replaced by fixedpoint specialization
//  Revision 1.10.1.1 2007/10/19 13:31:48CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  new specialized sin / cos / tan added
//  Revision 1.10 2007/10/15 17:31:12CEST Renner Christian (AE-DA/ESV1) (rec1lr)
//  renamed isDivisionSafe()
//  Revision 1.9 2007/10/12 13:16:26CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  bugfix atan2() in case f_y == 0
//  Revision 1.8 2007/10/12 09:49:47CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  atan() and atan2() changed
//  Revision 1.7 2007/10/08 15:02:20CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  include added
//  Revision 1.6 2007/10/08 14:09:11CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  trig methods replaced by approximation
//  Revision 1.5 2007/10/04 09:00:30CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  gcc errors with trigonometric function calls fixed
//  Revision 1.4 2007/10/02 10:48:23CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  Not using static_cast anymore.
//  Revision 1.3 2007/09/24 11:43:17CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  bugfix: sqr()
//  Revision 1.2 2007/09/24 10:38:53CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  isZero, isPositive, isNegative, isEqual added
//  Revision 1.1 2007/09/24 10:17:36CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_fixedpoint/vfc_fixedpoint.pj
//=============================================================================
