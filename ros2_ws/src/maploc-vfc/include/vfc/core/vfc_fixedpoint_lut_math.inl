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
///     $Source: include/vfc/core/vfc_fixedpoint_lut_math.inl $
///     $Revision: 1.47 $
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

#ifndef VFC_FIXEDPOINT_LUT_MATH_INL
#define VFC_FIXEDPOINT_LUT_MATH_INL

#include "vfc/core/vfc_fixedpoint_types.hpp"
#include "vfc/core/vfc_fixedpoint_ops.hpp"
#include "vfc/core/vfc_fixedpoint_math.hpp"
#include <cmath>


namespace vfc
{

    namespace intern
    {

        // if the value range is not even -2PI..2PI we do not need to fmod
        template <vfc::int32_t FracBitsValue, class ValueType>
        struct TQuadPeriod
        {
            typedef vfc::TFixedPoint<FracBitsValue, ValueType> value_type;
            static inline value_type value(const value_type& f_arg)
            {
                using namespace vfc;
                return fmod(vfc::abs(f_arg), vfc::TFixedPointValuePolicy<value_type>::G_2PI());
            }
        };

        template <class ValueType>
        struct TQuadPeriod<30, ValueType>
        {
            typedef vfc::TFixedPoint<30, ValueType> value_type;
            static inline value_type value(const value_type& f_arg)
            {
                return vfc::abs(f_arg);
            }
        };

        template <class ValueType>
        struct TQuadPeriod<29, ValueType>
        {
            typedef vfc::TFixedPoint<29, ValueType> value_type;
            static inline value_type value(const value_type& f_arg)
            {
                return vfc::abs(f_arg);
            }
        };

        // if we cannot store more than 2PI in a fixedpoint, there is no need to check for the right quad of period
        template <vfc::int32_t FracBitsValue, class ValueType>
        inline vfc::TFixedPoint<FracBitsValue, ValueType> quad_period(
                                                          const vfc::TFixedPoint<FracBitsValue, ValueType>& f_arg)
        {
            return TQuadPeriod<FracBitsValue, ValueType>::value(f_arg);
        }

        template <typename ValueType>
        struct TDefaultArcTan
        {
            static const vfc::int32_t size = sizeof(typename ValueType::value_type) * CHAR_BIT;
            typedef typename vfc::TFixedPoint<size - 1 // sign bit
                - (2 * ValueType::INTBITS) - 1, // enough space to store ((value * value) + G_0_p_28))
                typename ValueType::value_type //, precision over size
                > operation_type;
        };

        //! fast atan implementation
        //! http://www.lightsoft.co.uk/PD/stu/stuchat37.html
        //! http://de.wikipedia.org/wiki/Arkustangens_und_Arkuskotangens
        template <typename ValueType>
        inline ValueType atan(const ValueType& f_arg)
        {
            //VFC_STATIC_ASSERT(ValueType::SHIFT >= 23); // we need 23 fracbits to be at least 0.005 rad accurate
            VFC_STATIC_ASSERT(ValueType::INTBITS >= 1); // result is max +-pi/2

            typedef typename TDefaultArcTan<ValueType>::operation_type op_type;

            op_type value = op_type(vfc::abs(f_arg));
            if(value <= TFixedPointValuePolicy<op_type>::G_1())
            {
                value = (value / (TFixedPointValuePolicy<op_type>::G_1() +
                    (TFixedPointValuePolicy<op_type>::G_0p28() * value * value)));
            }
            else
            {
                value = TFixedPointValuePolicy<op_type>::G_PI_2() -
                    (value / (TFixedPointValuePolicy<op_type>::G_0p28() + (value * value)));
            }

            if(vfc::isPositive(f_arg))
            {
                return ValueType(value);
            }
            else
            {
                return ValueType(-value);
            }
        }

        //! \brief like signum(x) but for x = 0 -> 1
        template <vfc::int32_t FracBitsValue, class ValueType>
        inline TFixedPoint<FracBitsValue, ValueType>  signum_null_is_one(
                                                      const vfc::TFixedPoint<FracBitsValue, ValueType>& f_value)
        {
            typedef vfc::TFixedPoint<FracBitsValue, ValueType> fixedpoint_type;
            return fixedpoint_type((f_value >= fixedpoint_type(0, typename fixedpoint_type::CNoShift())) -
                                   (f_value < fixedpoint_type(0, typename fixedpoint_type::CNoShift())));
        }

        //! \brief interpolate between a value pair withing a lut (fixedpoint)
        //! \return The interpolated value.
        //! \todo bounding checks for the multiplication and? the division are not correct (false positives!)
        template <typename FixedPoint>
        inline vfc::int32_t interpolate_value(
                                  const vfc::int32_t f_lut[][2], //!< must be a fixedpoint value with SHIFT fracbits
                                  vfc::int32_t f_arg, //!< must be a fixedpoint value with SHIFT fracbits
                                  vfc::int32_t f_begin, //!< must be a fixedpoint value with SHIFT fracbits
                                  vfc::int32_t f_end) //!< must be a fixedpoint value with SHIFT fracbits
        {
            const vfc::int64_t n = f_lut[f_end][1] - f_lut[f_begin][1];
            const vfc::int64_t d = f_lut[f_end][0] - f_lut[f_begin][0];

            // check devision
            VFC_REQUIRE(!vfc::isZero(d));
            VFC_REQUIRE(((static_cast<vfc::int64_t>(1) <<
                       (((sizeof(vfc::int64_t) * CHAR_BIT) - 1) - FixedPoint::SHIFT )) - 1) >= vfc::abs(n));
            //VFC_REQUIRE(stlalias::numeric_limits<vfc::int32_t>::max() >=
            //                                           vfc::abs((n << FixedPoint::SHIFT) / d));  // check cast

            const vfc::int64_t m = (n << FixedPoint::SHIFT) / d;

            // check multiplication
            VFC_REQUIRE( (vfc::abs(static_cast<vfc::int64_t>(f_arg - f_lut[f_begin][0])) <
                vfc::abs(static_cast<vfc::int64_t>(stlalias::numeric_limits<vfc::int64_t>::max() /
                stlalias::max(vfc::abs(m), static_cast<vfc::int64_t>(1))))));   // avoid div by zero with max()
            VFC_REQUIRE(stlalias::numeric_limits<vfc::int32_t>::max() >=
                vfc::abs((static_cast<vfc::int64_t>(f_arg - f_lut[f_begin][0]) * m) >>
                FixedPoint::SHIFT));// check cast

            const vfc::int32_t lin = f_lut[f_begin][1] +
                static_cast<vfc::int32_t>((static_cast<vfc::int64_t>(f_arg - f_lut[f_begin][0]) * m) >>
                FixedPoint::SHIFT);
            return lin;
        }

        //! \brief Recursive look up for the right value pair in a lut plus interpolation.
        //! \return The interpolated value.
        template <typename FixedPoint>
        inline vfc::int32_t find_value(const vfc::int32_t f_lut[][2],
                                       vfc::int32_t f_arg,
                                       vfc::int32_t f_begin,
                                       vfc::int32_t f_end)
        {
            const vfc::int32_t middle = (f_begin + f_end) >> 1;

            if(middle == f_begin)
            {
                return interpolate_value<FixedPoint>(f_lut, f_arg, f_begin, f_end);
            }

            if(f_lut[middle][0] >= f_arg)
            {
                return find_value<FixedPoint>(f_lut, f_arg, f_begin, middle);
            }
            else
            {
                return find_value<FixedPoint>(f_lut, f_arg, middle, f_end);
            }
        }

        //! \brief Returns the index of a fixedstep lut.
        //! \return The index of a fixed step lut.
        template <typename FixedPoint>
        inline vfc::int32_t fixedlut_idx(vfc::int32_t f_value,
                                         vfc::int32_t f_step_size)
        {
            vfc::int64_t idx = (static_cast<vfc::int64_t>(f_value) << FixedPoint::SHIFT) / f_step_size; // get idx

            // to floor, since we have to interpolate between both idx
            idx >>= FixedPoint::SHIFT;

            return static_cast<vfc::int32_t>(idx);
        }

        //! \brief interpolate values in a lut (fixedpoint)
        //! \return The interpolated value (fixedpoint).
        template <typename FixedPoint>
        inline vfc::int32_t interpolate_fixed_lut(
                                const vfc::int32_t f_lut[], //!< must be a fixedpoint value with SHIFT fracbits
                                vfc::int32_t f_step_size,   //!< must be a fixedpoint value with SHIFT fracbits
                                vfc::int32_t f_arg,         //!< must be a fixedpoint value with SHIFT fracbits
                                vfc::int32_t f_begin_idx,
                                vfc::int32_t f_end_idx)
        {
            const vfc::int64_t n = f_lut[f_end_idx] - f_lut[f_begin_idx];
            const vfc::int64_t d = f_step_size;

            // check devision
            VFC_REQUIRE(!vfc::isZero(d));
            VFC_REQUIRE(((static_cast<vfc::int64_t>(1) <<
                       (((sizeof(vfc::int64_t) * CHAR_BIT) - 1) - FixedPoint::SHIFT )) - 1) >= vfc::abs(n));
            VFC_REQUIRE(stlalias::numeric_limits<vfc::int32_t>::max() >=
                        vfc::abs((n << FixedPoint::SHIFT) / d));  // check cast

            const vfc::int64_t m = (n << FixedPoint::SHIFT) / d;

            // get the f_arg of the idx
            const vfc::int64_t arg_begin = ((static_cast<vfc::int64_t>(f_begin_idx)
                                                                      << FixedPoint::SHIFT) // int to fixedpoint
                * f_step_size) >> FixedPoint::SHIFT; // fixedpoint multiply

            // check multiplication
            VFC_REQUIRE( (vfc::abs(static_cast<vfc::int64_t>(f_arg - arg_begin)) <
                vfc::abs(static_cast<vfc::int64_t>(stlalias::numeric_limits<vfc::int64_t>::max() /
                stlalias::max(vfc::abs(m), static_cast<vfc::int64_t>(1))))));   // avoid div by zero with max()
            VFC_REQUIRE(stlalias::numeric_limits<vfc::int32_t>::max() >=
                       vfc::abs((static_cast<vfc::int64_t>(f_arg - arg_begin) * m) >> FixedPoint::SHIFT));// check cast


            const vfc::int32_t lin = f_lut[f_begin_idx] +
                  static_cast<vfc::int32_t>((static_cast<vfc::int64_t>(f_arg - arg_begin) * m) >> FixedPoint::SHIFT);
            return lin;
        }

        //! \brief fixed step lut base cos implementation
        //! Complexity is O(1)
        //! Precision is about 0.0001.
        template <typename ValueType>
        inline ValueType fixedlut_cos(const ValueType& f_value)
        {
            typedef vfc::TFixedPoint<28, vfc::int32_t> op_type; // the lut is using 28bit frac, DO NOT use convenience type here

            VFC_STATIC_ASSERT(sizeof(vfc::int32_t) == sizeof(typename ValueType::value_type));

            static const vfc::int32_t lut[] = {
                268435456, 268376327, 268198969, 267903460,
                267489928, 266958557, 266309581, 265543285,
                264660007, 263660136, 262544113, 261312429,
                259965628, 258504301, 256929094, 255240699,
                253439861, 251527374, 249504079, 247370868,
                245128680, 242778504, 240321375, 237758375,
                235090634, 232319326, 229445673, 226470940,
                223396438, 220223522, 216953588, 213588079,
                210128475, 206576302, 202933125, 199200547,
                195380214, 191473809, 187483052, 183409702,
                179255553, 175022435, 170712213, 166326785,
                161868085, 157338075, 152738752, 148072142,
                143340300, 138545311, 133689288, 128774370,
                123802722, 118776534, 113698020, 108569418,
                103392987, 98171007, 92905780, 87599624,
                82254877, 76873893, 71459044, 66012714,
                60537303, 55035223, 49508898, 43960763,
                38393261, 32808845, 27209976, 21599120,
                15978748, 10351337, 4719367, -914682
            };
            static const op_type::value_type step_size = 5634294; // 0.0209894

            // get the right quadrant
            op_type period_value(quad_period(f_value));

            if(period_value <= vfc::TFixedPointValuePolicy<op_type>::G_PI_2())
            {
                const vfc::int32_t idx = fixedlut_idx<op_type>(period_value.fixedpoint(),
                    step_size);

                VFC_REQUIRE(idx < (vfc::int32_t)((sizeof(lut) / sizeof(vfc::int32_t)) - 1)); // because of interpolate

                const vfc::int32_t value = interpolate_fixed_lut<op_type>(lut,
                    step_size,
                    period_value.fixedpoint(),
                    idx,
                    idx + 1);

                return ValueType(op_type(value, op_type::CNoShift()));
            }
            else if(period_value <= vfc::TFixedPointValuePolicy<op_type>::G_PI())
            {
                period_value = vfc::TFixedPointValuePolicy<op_type>::G_PI() - period_value;

                const vfc::int32_t idx = fixedlut_idx<op_type>(period_value.fixedpoint(),
                    step_size);

                VFC_REQUIRE(idx < (vfc::int32_t)((sizeof(lut) / sizeof(vfc::int32_t)) - 1)); // because of interpolate

                const vfc::int32_t value = interpolate_fixed_lut<op_type>(lut,
                    step_size,
                    period_value.fixedpoint(),
                    idx,
                    idx + 1);

                return ValueType(-op_type(value, op_type::CNoShift()));
            }
            else if(period_value <= vfc::TFixedPointValuePolicy<op_type>::G_3PI_2())
            {
                period_value = period_value - vfc::TFixedPointValuePolicy<op_type>::G_PI();

                const vfc::int32_t idx = fixedlut_idx<op_type>(period_value.fixedpoint(),
                    step_size);

                VFC_REQUIRE(idx < (vfc::int32_t)((sizeof(lut) / sizeof(vfc::int32_t)) - 1)); // because of interpolate

                const vfc::int32_t value = interpolate_fixed_lut<op_type>(lut,
                    step_size,
                    period_value.fixedpoint(),
                    idx,
                    idx + 1);

                return ValueType(-op_type(value, op_type::CNoShift()));
            }
            else
            {
                period_value = vfc::TFixedPointValuePolicy<op_type>::G_2PI() - period_value;

                const vfc::int32_t idx = fixedlut_idx<op_type>(period_value.fixedpoint(),
                    step_size);

                VFC_REQUIRE(idx < (vfc::int32_t)((sizeof(lut) / sizeof(vfc::int32_t)) - 1)); // because of interpolate

                const vfc::int32_t value = interpolate_fixed_lut<op_type>(lut,
                    step_size,
                    period_value.fixedpoint(),
                    idx,
                    idx + 1);

                return ValueType(op_type(value, op_type::CNoShift()));
            }
        }

        //! \brief fixed step lut base sin implementation
        //! The function is implemented using the fixedlut_cos implementation.
        //! Complexity and precsion same as fixedlut_cos.
        template <typename ValueType>
        inline ValueType fixedlut_sin(const ValueType& f_value)
        {
            if(vfc::isPositive(f_value))
            {
                return fixedlut_cos(vfc::TFixedPointValuePolicy<ValueType>::G_PI_2() - f_value);
            }
            else
            {
                return -fixedlut_cos(f_value + vfc::TFixedPointValuePolicy<ValueType>::G_PI_2());
            }
        }

        //! \brief fixed step lut base tan implementation
        //! Complexity is O(fixedlut_sin) + O(fixedlut_cos).
        //! Precision is precision of fixedlut_cos + fixedlut_sin (when using far enough away from k * PI/2)
        //! \warning tan(k * PI/2) is very inaccurate!
        template <typename ValueType>
        inline ValueType fixedlut_tan(const ValueType& f_value)
        {
            const ValueType l_sin(fixedlut_sin(f_value));
            const ValueType l_cos(fixedlut_cos(f_value));

            if (l_sin.isDivisionSafe(l_cos))
            {
                return (l_sin / l_cos);
            }
            else
            {
                // return the max or min of the fixedpoint
                return vfc::intern::signum_null_is_one(l_sin) *
                       vfc::intern::signum_null_is_one(l_cos) *
                       ValueType::max;
            }
        }

        //! \brief lut based exp implementation
        //! The lut was generated with a 0.00001 precision. Complexity is O(1).
        //! Precision should be 0.00001 for small arguments. Since we have a multiplication
        //! and some fixedpoint convertions the result gets inaccurate for higher arguments.
        //! This could be fixed if using a 64bit ln2 approximation!
        //! \return The user is responsible that the output fits into its ValueType.
        //! exp(x) = 2^k * exp(x - k*ln2)
        template <typename ValueType>
        inline ValueType fixedlut_exp(const ValueType& f_value)
        {
            // check input range
            VFC_REQUIRE(ValueType::max.to_float64() >= ::exp(f_value.to_float64()));

            typedef vfc::TFixedPoint<28, vfc::int32_t> op_type; // type of the lut, DO NOT use fp32p28_t
            VFC_STATIC_ASSERT(sizeof(op_type::value_type) == sizeof(typename ValueType::value_type));

            static const op_type::value_type lut[] = {
                268435456, 269737724, 271046309, 272361244,
                273682557, 275010281, 276344445, 277685083,
                279032224, 280385900, 281746144, 283112987,
                284486460, 285866597, 287253429, 288646990,
                290047310, 291454425, 292868365, 294289166,
                295716858, 297151478, 298593057, 300041629,
                301497229, 302959891, 304429648, 305906536,
                307390588, 308881841, 310380327, 311886084,
                313399145, 314919547, 316447325, 317982514,
                319525151, 321075272, 322632913, 324198111,
                325770902, 327351323, 328939411, 330535204,
                332138738, 333750052, 335369182, 336996168,
                338631046, 340273856, 341924636, 343583424,
                345250259, 346925181, 348608229, 350299441,
                351998858, 353706520, 355422466, 357146736,
                358879372, 360620413, 362369900, 364127875,
                365894379, 367669452, 369453137, 371245474,
                373046508, 374856278, 376674828, 378502201,
                380338439, 382183585, 384037683, 385900775,
                387772906, 389654119, 391544458, 393443968,
                395352694, 397270679, 399197969, 401134609,
                403080644, 405036120, 407001082, 408975577,
                410959652, 412953351, 414956723, 416969814,
                418992670, 421025341, 423067872, 425120313,
                427182710, 429255113, 431337570, 433430130,
                435532841, 437645753, 439768916, 441902379,
                444046191, 446200405, 448365069, 450540234,
                452725952, 454922274, 457129250, 459346934,
                461575376, 463814629, 466064745, 468325778,
                470597779, 472880803, 475174902, 477480131,
                479796543, 482124193, 484463135, 486813424,
                489175115, 491548263, 493932924, 496329154,
                498737009, 501156546, 503587820, 506030889,
                508485810, 510952641, 513431439, 515922263,
                518425171, 520940220, 523467472, 526006984,
                528558815, 531123027, 533699678, 536288830,
                538890542
            };
            static const op_type::value_type step_size = 1299119; // 0.0048396

            // get the abs
            const ValueType abs_arg = vfc::abs(f_value);

            // k is used to shift the argument to be between 0 .. ln(2)
            const op_type::value_type k = vfc::floor(abs_arg / TFixedPointValuePolicy<ValueType>::G_LN2());

            op_type::value_type arg = op_type(
                                   abs_arg - (TFixedPointValuePolicy<ValueType>::G_LN2() * ValueType(k))).fixedpoint();

            const vfc::int32_t idx = fixedlut_idx<op_type>(arg,
                step_size);

            // lut /sizeof = lutsize = 145
            VFC_REQUIRE(idx < (static_cast<vfc::int32_t>(sizeof(lut) /
                                                        (sizeof( op_type::value_type))))); // because of interpolate

            vfc::int32_t value = interpolate_fixed_lut<op_type>(lut,
                step_size,
                arg,
                idx,
                idx + 1);

            const vfc::int32_t shift = k - (op_type::SHIFT - ValueType::SHIFT);

            if(vfc::isPositive(shift))
            {
                value <<= shift;
            }
            else
            {
                value >>= (-shift);
            }

            if(vfc::isPositive(f_value))
            {
                // the value may be negative (because we are using signed types, switch to 64bit to solve this problem)
                if(vfc::isPositive(value))
                {
                    return ValueType(value, typename ValueType::CNoShift());
                }
                else
                {
                    return -ValueType(value, typename ValueType::CNoShift());
                }
            }
            else
            {
                if(vfc::isZero(value))
                {
                    // big negative values -> zero
                    return ValueType::zero;
                }

                // the value may be negative (because we are using signed types, switch to 64bit to solve this problem)
                if(vfc::isPositive(value))
                {
                    return vfc::TFixedPointValuePolicy<ValueType>::G_1() /
                        ValueType(value, typename ValueType::CNoShift());
                }
                else
                {
                    return -vfc::TFixedPointValuePolicy<ValueType>::G_1() /
                        ValueType(value, typename ValueType::CNoShift());
                }
            }
        }

        //! \brief lut based log implementation
        //! The lut was generated with a 0.0001 precision. Complexity is O(1).
        //! \return The output must fit into a vfc::fp32p26_t.
        template <typename ValueType>
        inline ValueType fixedlut_log(const ValueType& f_value)
        {
            // input/output must have the same type as the lut
            typedef vfc::TFixedPoint<26, vfc::int32_t> op_type; // type of the lut
            VFC_STATIC_ASSERT(sizeof( op_type::value_type) == sizeof(typename ValueType::value_type));

            // check input range
            VFC_REQUIRE(vfc::isPositive(f_value));

            static const op_type::value_type lut[] = {
                -46516319, -45130568, -43772852, -42442062,
                -41137150, -39857128, -38601064, -37368079,
                -36157339, -34968056, -33799483, -32650910,
                -31521665, -30411108, -29318631, -28243653,
                -27185624, -26144016, -25118330, -24108083,
                -23112820, -22132102, -21165509, -20212641,
                -19273114, -18346558, -17432622, -16530964,
                -15641261, -14763199, -13896478, -13040808,
                -12195911, -11361518, -10537373, -9723227,
                -8918839, -8123979, -7338423, -6561956,
                -5794371, -5035466, -4285047, -3542927,
                -2808923, -2082862, -1364571, -653888,
                49348, 745291, 1434091, 2115894,
                2790839, 3459063, 4120699, 4775876,
                5424717, 6067346, 6703879, 7334432,
                7959114, 8578036, 9191301, 9799013,
                10401272, 10998173, 11589812, 12176280,
                12757668, 13334062, 13905548, 14472208,
                15034123, 15591373, 16144033, 16692179,
                17235884, 17775219, 18310255, 18841058,
                19367696, 19890234, 20408734, 20923258,
                21433868, 21940622, 22443578, 22942793,
                23438322, 23930218, 24418535, 24903324,
                25384637, 25862522, 26337028, 26808203,
                27276092
            };
            static const op_type::value_type step_size = 700079; // 0.010432

            // http://de.wikipedia.org/wiki/Logarithmus
            // ln(x) = m * ln(2) + ln(pow(2, -m) * x)
            // -> we only have to save 0.5 .. 1.5 in the lut

            typename ValueType::value_type l_value = f_value.fixedpoint();
            const typename ValueType::value_type cpy_value = l_value;

            // get highest bit
            vfc::int32_t shift = 0;
            while(l_value >>= 1)
            {
                shift++;
            }

            const typename ValueType::value_type least_shift_bits = 1;
            if(shift > least_shift_bits)
            {
                // shift is only 0 if f_arg is 0 and that is not valid
                l_value = (cpy_value >> (shift - least_shift_bits));
                // get a value as near as possible to 1
                if(l_value & 1) // is the first bit is set then we can shift one more
                    shift++;
            }

            // difference between shift and frac bits of input value
            const typename ValueType::value_type diff = shift - ValueType::SHIFT;
            // !!! shift < 31 and ValueType::SHIFT > 1 make diff 0..30 < 2^5

            // get the factor between the input type and the norm type
            const typename ValueType::value_type fac = op_type::SHIFT - diff - ValueType::SHIFT;

            // the input type needs at least one int bit
            VFC_STATIC_ASSERT(ValueType::INTBITS >= 1);
            typename ValueType::value_type norm_value = (fac > 0) ? (cpy_value << fac) : (cpy_value >> -fac);

            op_type result = op_type(46516319,  op_type::CNoShift()) * // log(2)
                op_type(diff);

            // the lut starts from 0.5, therefore we have to modify the norm_value
            norm_value -= 33554432; // 0.5

            const vfc::int32_t idx = fixedlut_idx<op_type>(norm_value,
                step_size);

            VFC_REQUIRE(idx < static_cast<vfc::int32_t>(sizeof(lut) /
                                                       (sizeof( op_type::value_type)))); // because of interpolate

            const vfc::int32_t lin_value = interpolate_fixed_lut<op_type>(lut,
                step_size,
                norm_value,
                idx,
                idx + 1);

            result += op_type(lin_value,  op_type::CNoShift());

            // due precision the result might not fit into the ValueType
            // if this happens map it to the max values
            // this function should use 64bit and should not rely on some hardcoded fixedpoint!!!!
            // when using 64bit check if we need the code below!
            if(ValueType::max.to_int() < vfc::abs(result.to_int()))
            {
                if(vfc::isPositive(result))
                {
                    return ValueType::max;
                }
                else
                {
                    return -ValueType::max;
                }
            }
            else
            {
                return ValueType(result);
            }
        }

        //! \brief lut based pow implementation
        //! Complexity is O(1).
        //! Implementation details: pow(a, b) = exp(b * log(a))
        //! Precision: The precision depends of the exp and log precision values.
        //! precision of log = p_log, precision of exp = p_exp
        //! pow(a, b) -> precision of pow is at least = p_exp + (exp(1.5 + p_log) - exp(1.5))
        //! with p_log = p_exp = 0.0001 -> p_pow = 0.00055
        //! \return The user is responsible that the output fits into its ValueType.
        template <typename ValueType>
        inline ValueType fixedlut_pow(const ValueType& f_base, const ValueType& f_exponent)
        {
            if(!f_exponent.fixedpoint())
            {
                return vfc::TFixedPointValuePolicy<ValueType>::G_1();
            }

            if(!f_base.fixedpoint())
            {
                return ValueType(0, typename ValueType::CNoShift());
            }

            // input check
            VFC_REQUIRE(vfc::isPositive(f_base));

            ValueType tmp = fixedlut_log(f_base);
            tmp *= f_exponent;
            return fixedlut_exp(tmp);
        }

        //! \brief lut based arcsin implementation
        //! The lut was generated with a 0.0001 precision. Complexity is O(log(n)) n = size of the lut.
        //! If value is below 0.94 complexity is O(1).
        //! \f_value Input range: -1 .. 1.
        //! \return Output range: -PI/2 .. PI/2.
        template <typename ValueType>
        inline ValueType fixedlut_asin(const ValueType& f_value)
        {
            // place to store PI/2
            VFC_STATIC_ASSERT(ValueType::INTBITS >= 1);

            // check input range
            VFC_REQUIRE(vfc::abs(f_value) <= ValueType(1));

            // input/output must have the same type as the lut
            typedef vfc::TFixedPoint<28, vfc::int32_t> op_type; // type of the lut
            VFC_STATIC_ASSERT(sizeof( op_type::value_type) == sizeof(typename ValueType::value_type));

            // lut from asin(0.94 - 1.=
            static const op_type::value_type lut[][2] = {
                {252329328, 328197323}, {253323598, 331156582},
                {254285863, 334112034}, {255216168, 337063822},
                {256114558, 340012088}, {256981077, 342956972},
                {257815763, 345898609}, {258618654, 348837134},
                {259389788, 351772680}, {260129204, 354705399},
                {260836930, 357635400}, {261512995, 360562812},
                {262157429, 363487761}, {262770261, 366410376},
                {263351516, 369330777}, {263901219, 372249089},
                {264419392, 375165434}, {264906074, 378080039},
                {265361267, 380992917}, {265784987, 383904188},
                {265985051, 385359258}, {266177254, 386813970},
                {266361596, 388268339}, {266538081, 389722381},
                {266706710, 391176108}, {266867485, 392629536},
                {267020407, 394082680}, {267165478, 395535554},
                {267302699, 396988171}, {267432072, 398440547},
                {267553597, 399892700}, {267667277, 401344639},
                {267773112, 402796380}, {267871104, 404247937},
                {267961252, 405699324}, {268043558, 407150555},
                {268118024, 408601645}, {268184649, 410052608},
                {268243434, 411503457}, {268269887, 412228844},
                {268294380, 412954208}, {268316913, 413679551},
                {268337487, 414404874}, {268356101, 415130180},
                {268372756, 415855470}, {268387452, 416580745},
                {268400187, 417306009}, {268410964, 418031261},
                {268419781, 418756505}, {268426639, 419481742},
                {268429333, 419844358}, {268431537, 420206974},
                {268433251, 420569588}, {268434476, 420932202},
                {268434904, 421113508}, {268435211, 421294815},
                {268435394, 421476121}, {268435440, 421566775},
                {268435456, 421657428}
            };
            static const size_t lut_size = sizeof(lut) / (2 * sizeof( op_type::value_type)); // 59

            // fixed step lut from 0. - 0.94
            static const op_type::value_type fix_lut[] = {
                0, 1470800, 2941645, 4412578,
                5883643, 7354885, 8826349, 10298077,
                11770116, 13242509, 14715300, 16188535,
                17662259, 19136515, 20611351, 22086810,
                23562938, 25039780, 26517383, 27995793,
                29475056, 30955218, 32436325, 33918426,
                35401567, 36885796, 38371161, 39857709,
                41345490, 42834552, 44324945, 45816718,
                47309921, 48804605, 50300821, 51798620,
                53298053, 54799174, 56302034, 57806688,
                59313188, 60821591, 62331950, 63844321,
                65358761, 66875326, 68394074, 69915064,
                71438353, 72964003, 74492074, 76022626,
                77555722, 79091426, 80629800, 82170910,
                83714821, 85261600, 86811315, 88364035,
                89919829, 91478768, 93040924, 94606370,
                96175182, 97747434, 99323205, 100902571,
                102485613, 104072412, 105663051, 107257614,
                108856187, 110458858, 112065715, 113676850,
                115292356, 116912328, 118536861, 120166055,
                121800012, 123438833, 125082624, 126731493,
                128385550, 130044909, 131709683, 133379991,
                135055955, 136737697, 138425345, 140119029,
                141818882, 143525041, 145237647, 146956844,
                148682779, 150415605, 152155477, 153902558,
                155657012, 157419008, 159188723, 160966336,
                162752033, 164546006, 166348451, 168159573,
                169979582, 171808695, 173647135, 175495136,
                177352936, 179220783, 181098934, 182987656,
                184887223, 186797922, 188720049, 190653912,
                192599831, 194558138, 196529180, 198513317,
                200510923, 202522389, 204548124, 206588554,
                208644124, 210715298, 212802565, 214906433,
                217027439, 219166144, 221323138, 223499040,
                225694505, 227910220, 230146911, 232405347,
                234686337, 236990741, 239319472, 241673497,
                244053845, 246461615, 248897976, 251364180,
                253861568, 256391579, 258955758, 261555773,
                264193425, 266870663, 269589603, 272352551,
                275162023, 278020775, 280931838, 283898556,
                286924633, 290014190, 293171837, 296402751,
                299712786, 303108601, 306597820, 310189248,
                313893135, 317721537, 321688794, 325812183,
                330112837
            };
            static const op_type::value_type fix_step_size = 1470793; // 0.00547913

            vfc::int32_t arg = op_type(f_value).fixedpoint();

            if(vfc::isPositive(f_value))
            {
                if(arg > 252329328) // 0.94
                {
                    return ValueType(op_type(find_value<op_type>(lut, // lut
                        arg, // value
                        0,
                        lut_size), op_type::CNoShift()));
                }
                else
                {
                    const vfc::int32_t idx = fixedlut_idx<op_type>(arg,
                        fix_step_size);

                    VFC_REQUIRE(idx < static_cast<vfc::int32_t>(sizeof(fix_lut) /
                                                             (sizeof( op_type::value_type))));// because of interpolate

                    const vfc::int32_t value = interpolate_fixed_lut<op_type>(fix_lut,
                        fix_step_size,
                        arg,
                        idx,
                        idx + 1);

                    return ValueType(op_type(value,  op_type::CNoShift()));
                }
            }
            else
            {
                arg = -arg;
                if(arg > 252329328) // 0.94
                {
                    return -ValueType(op_type(find_value<op_type>(lut, // lut
                        arg, // value
                        0,
                        lut_size),  op_type::CNoShift()));
                }
                else
                {
                    const vfc::int32_t idx = fixedlut_idx<op_type>(arg,
                        fix_step_size);

                    VFC_REQUIRE(idx < static_cast<vfc::int32_t>(sizeof(fix_lut) /
                                                             (sizeof( op_type::value_type))));// because of interpolate

                    const vfc::int32_t value = interpolate_fixed_lut<op_type>(fix_lut,
                        fix_step_size,
                        arg,
                        idx,
                        idx + 1);

                    return -ValueType(op_type(value,  op_type::CNoShift()));
                }
            }
        }

        //! \brief lut based arccos implementation
        //! The arccos is implemented using the arcsin implementation
        //! (see fixedlut_asin for details about complexity and precision).
        //! \f_value Input range: -1 .. 1.
        //! \return Value range: 0..PI .
        template <typename ValueType>
        inline ValueType fixedlut_acos(const ValueType& f_value)
        {
            // need space to store PI
            VFC_STATIC_ASSERT(ValueType::INTBITS >= 2);

            return vfc::TFixedPointValuePolicy<ValueType>::G_PI_2() - fixedlut_asin(f_value);
        }

    } // end intern

} // end vfc

#endif // VFC_FIXEDPOINT_LUT_MATH_INL

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: include/vfc/core/vfc_fixedpoint_lut_math.inl  $
//  Revision 1.47 2010/11/10 15:08:58MEZ Muehlmann Karsten (CC/PJ-FA1) (MUK2LR) 
//  (re-)enable compilation of TFixedPointSubstitute (mantis3524)
//  Revision 1.46 2009/06/02 15:52:44CEST Jaeger Thomas (CC-DA/ESV2) (JAT2HI) 
//  - checkin for merge
//  Revision 1.44 2009/04/24 09:00:32CEST Voelz Henning (CC-DA/ESV4) (VOH2HI) 
//  - bugfix: fixedlut_exp() div by zero for big negative arguments (mantis:2801)
//  Revision 1.43 2009/04/01 11:25:39CEST Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  -Resolution of issue w.r.t. the similar local and function argument variable naming (value and f_value). (mantis 2498)
//  Revision 1.42 2009/03/25 20:29:43IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  - Linebreak at 120 in all fixedpoint files. (mantis 0002692)
//  Revision 1.41 2009/01/31 13:04:46IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Removal of QAC++ warnings.
//  (Manis : 0002498)
//  Revision 1.40 2009/01/27 15:43:59IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Inclusion of footer and proper indendation.
//  (Matnis : 0002540)
//=============================================================================
