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
//       Compiler(s): VS8.0, CW9.3
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//  Name: sf71lr
//  Department:
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: include/vfc/core/vfc_fixedpoint_math.hpp $
///     $Revision: 1.21 $
///     $Author: Muehlmann Karsten (CC/PJ-FA1) (MUK2LR) $
///     $Date: 2010/11/10 15:08:57MEZ $
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

#ifndef VFC_FIXEDPOINT_MATH_HPP_INCLUDED
#define VFC_FIXEDPOINT_MATH_HPP_INCLUDED

#include "vfc/core/vfc_fixedpoint.hpp"
#include "vfc/core/vfc_fixedpoint_types.hpp"
#include <limits>

namespace vfc
{

//! \todo implement 0
//! \todo implement size ops
//! \todo implement other value types

/*
// generate values
template <ValueType>
struct TFixedPointOut
{
    TFixedPointOut()
    {
        std::cout << std::endl << ValueType::SHIFT << std::endl;
        std::cout << ValueType(vfc::G_PI_2).fixedpoint() << std::endl
            << ValueType(vfc::G_PI).fixedpoint() << std::endl
            << ValueType(vfc::G_PI * (3./2.)).fixedpoint() << std::endl
            << ValueType(vfc::G_2PI).fixedpoint() << std::endl
            << ValueType(-1).fixedpoint() << std::endl
            << ValueType(1).fixedpoint() << std::endl;
    }
};

int main()
{
    { TFixedPointOut< vfc::TFixedPoint<22, vfc::int32_t> > value; }
}
*/

template <typename ValueType>
struct TFixedPointValuePolicy
{
    typedef ValueType fixedpoint_type;

    static inline fixedpoint_type G_0()       { return fixedpoint_type(0,  fixedpoint_type::CNoShift()); }
    static inline fixedpoint_type G_EPSILON() { return fixedpoint_type(1,  fixedpoint_type::CNoShift()); }
	static inline fixedpoint_type G_MAX()     { return fixedpoint_type(stlalias::numeric_limits<vfc::int32_t>::max, 
                                                                         fixedpoint_type::CNoShift()); }
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<30, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<30, vfc::int32_t>  fixedpoint_type;

    // only 1 int bit
    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(1686629713,  fixedpoint_type::CNoShift()); } // PI / 2
    // static const vfc::fixedpoint_type G_PI; // PI
    // static const vfc::fixedpoint_type G_3PI_2; // (3 / 2) * PI
    // static const vfc::fixedpoint_type G_2PI; // 2 * PI
    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-1073741824,  fixedpoint_type::CNoShift()); }// -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(1073741824,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(300647710, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(744261117, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<29, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<29, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(843314856,  fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(1686629713,  fixedpoint_type::CNoShift()); } // PI

    // only 2 INT bits
    //static const fixedpoint_type G_3PI_2(); // (3 / 2) * PI
    //static const fixedpoint_type G_2PI(); // 2 * PI
    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-536870912,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(536870912,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(150323855, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(372130558, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<28, vfc::int32_t> >
{
    typedef vfc::TFixedPoint<28, vfc::int32_t> fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(421657428,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(843314856,   fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(1264972284,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(1686629713,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-268435456,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(268435456,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(75161927, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(186065279, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<27, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<27, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(210828714,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(421657428,   fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(632486142,   fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(843314856,   fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-134217728,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(134217728,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(37580963, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(93032639, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<26, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<26, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(105414357,  fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(210828714,  fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(316243071,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(421657428,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-67108864,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(67108864,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(18790481, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(46516319, fixedpoint_type::CNoShift()); } //! needed for exp

};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<25, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<25, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(52707178,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(105414357,  fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(158121535,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(210828714,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-33554432,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(33554432,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(9395240, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(23258159, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<24, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<24, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(26353589,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(52707178,   fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(79060767,   fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(105414357,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-16777216,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(16777216,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(4697620, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(11629079, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<23, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<23, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(13176794,  fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(26353589,  fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(39530383,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(52707178,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-8388608,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(8388608,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(2348810, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(5814539, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<22, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<22, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return  vfc::TFixedPoint<22, vfc::int32_t> (6588397,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return  vfc::TFixedPoint<22, vfc::int32_t> (13176794,  fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return  vfc::TFixedPoint<22, vfc::int32_t> (19765191,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return  vfc::TFixedPoint<22, vfc::int32_t> (26353589,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return  vfc::TFixedPoint<22, vfc::int32_t> (-4194304,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return  vfc::TFixedPoint<22, vfc::int32_t> (4194304,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(1174405, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(2907269, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<21, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<21, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(3294198,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(6588397,   fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(9882595,   fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(13176794,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-2097152,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(2097152,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(587202, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(1453634, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<20, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<20, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(1647099,   fixedpoint_type::CNoShift());; } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(3294198,   fixedpoint_type::CNoShift());; } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(4941297,   fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(6588397,   fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-1048576,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(1048576,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(293601, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(726817, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<19, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<19, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(823549,  fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(1647099,  fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(2470648,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(3294198,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-524288,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(524288,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(146800, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(363408, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<18, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<18, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(411774,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(823549,   fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(1235324,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(1647099,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-262144,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(262144,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(73400, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(181704, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<17, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<17, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(205887,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(411774,   fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(617662,   fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(823549,   fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-131072,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(131072,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(36700, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(90852, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<16, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<16, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(102943,  fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(205887,  fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(308831,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(411774,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-65536,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(65536,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(18350, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(45426, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<15, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<15, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return  vfc::TFixedPoint<15, vfc::int32_t> (51471,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return  vfc::TFixedPoint<15, vfc::int32_t> (102943,  fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return  vfc::TFixedPoint<15, vfc::int32_t> (154415,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return  vfc::TFixedPoint<15, vfc::int32_t> (205887,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return  vfc::TFixedPoint<15, vfc::int32_t> (-32768,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return  vfc::TFixedPoint<15, vfc::int32_t> (32768,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(9175, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(22713, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<14, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<14, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(25735,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(51471,   fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(77207,   fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(102943,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-16384,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(16384,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(4587, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(11356, fixedpoint_type::CNoShift()); } //! needed for exp

};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<13, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<13, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(12867,  fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(25735,  fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(38603,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(51471,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-8192,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(8192,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(2293, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(5678, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<12, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<12, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(6433,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(12867,  fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(19301,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(25735,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-4096,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(4096,   fixedpoint_type::CNoShift()); } // 1

    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(1146, fixedpoint_type::CNoShift()); } //! needed for atan

    static inline fixedpoint_type G_LN2()
    { return fixedpoint_type(2839, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<11, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<11, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(3216,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(6433,   fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(9650,   fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(12867,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-2048,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(2048,   fixedpoint_type::CNoShift()); } // 1
    // not accurate!
    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(573, fixedpoint_type::CNoShift()); } //! needed for atan

    // not accurate! static inline fixedpoint_type G_LN2()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<10, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<10, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(1608,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(3216,   fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(4825,   fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(6433,   fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-1024,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(1024,   fixedpoint_type::CNoShift()); } // 1
    // not accurate!
    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(286, fixedpoint_type::CNoShift()); } //! needed for atan

    // not accurate! static inline fixedpoint_type G_LN2()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<9, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<9, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(804,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(1608,  fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(2412,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(3216,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-512,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(512,   fixedpoint_type::CNoShift()); } // 1
    // not accurate!
    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(143, fixedpoint_type::CNoShift()); } //! needed for atan

    // not accurate! static inline fixedpoint_type G_LN2()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<8, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<8, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(402,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(804,   fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(1206,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(1608,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-256,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(256,   fixedpoint_type::CNoShift()); } // 1
    // not accurate!
    static inline fixedpoint_type G_0p28()
    { return fixedpoint_type(71, fixedpoint_type::CNoShift()); } //! needed for atan

    // not accurate! static inline fixedpoint_type G_LN2()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<7, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<7, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(201,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(402,   fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(603,   fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(804,   fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-128,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(128,   fixedpoint_type::CNoShift()); } // 1

    // not accurate! static inline fixedpoint_type G_0p28()
    //{ return fixedpoint_type(35, fixedpoint_type::CNoShift()); } //! needed for atan

    // not accurate! static inline fixedpoint_type G_LN2()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<6, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<6, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(100,  fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(201,  fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(301,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(402,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-64,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(64,   fixedpoint_type::CNoShift()); } // 1

    // not accurate! static inline fixedpoint_type G_0p28()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for atan

    // not accurate! static inline fixedpoint_type G_LN2()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<5, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<5, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(50,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(100,  fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(150,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(201,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-32,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(32,   fixedpoint_type::CNoShift()); } // 1

    // not accurate! static inline fixedpoint_type G_0p28()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for atan

    // not accurate! static inline fixedpoint_type G_LN2()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<4, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<4, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(25,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(50,   fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(75,   fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(100,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-16,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(16,   fixedpoint_type::CNoShift()); } // 1

    // not accurate! static inline fixedpoint_type G_0p28()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for atan

    // not accurate! static inline fixedpoint_type G_LN2()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<3, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<3, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(12,  fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(25,  fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(37,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(50,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-8,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(8,   fixedpoint_type::CNoShift()); } // 1

    // not accurate! static inline fixedpoint_type G_0p28()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for atan

    // not accurate! static inline fixedpoint_type G_LN2()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for exp
};

template <>
struct TFixedPointValuePolicy< vfc::TFixedPoint<2, vfc::int32_t> >
{
    typedef  vfc::TFixedPoint<2, vfc::int32_t>  fixedpoint_type;

    static inline fixedpoint_type G_PI_2()
    { return fixedpoint_type(6,   fixedpoint_type::CNoShift()); } // PI / 2

    static inline fixedpoint_type G_PI()
    { return fixedpoint_type(12,  fixedpoint_type::CNoShift()); } // PI

    static inline fixedpoint_type G_3PI_2()
    { return fixedpoint_type(18,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    static inline fixedpoint_type G_2PI()
    { return fixedpoint_type(25,  fixedpoint_type::CNoShift()); } // 2 * PI

    static inline fixedpoint_type G_NEGATIVE_1()
    { return fixedpoint_type(-4,  fixedpoint_type::CNoShift()); } // -1

    static inline fixedpoint_type G_1()
    { return fixedpoint_type(4,   fixedpoint_type::CNoShift()); } // 1

    // not accurate! static inline fixedpoint_type G_0p28()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for atan

    // not accurate! static inline fixedpoint_type G_LN2()
    //{ return fixedpoint_type(X, fixedpoint_type::CNoShift()); } //! needed for exp
};

// (Msg Disable 474: This literal is of the non standard type 'long long'.)
// PRQA S 474  ++
// muk2lr: deactivated 64bit constants (unused up to now) to enable TFloatingPointSubstitute compilation
// template <>
// struct TFixedPointValuePolicy<vfc::fp64p32_t>
// {
    // typedef vfc::fp64p32_t fixedpoint_type;

    // static inline fixedpoint_type G_PI_2()
    // { return fixedpoint_type(6746518852LL,   fixedpoint_type::CNoShift()); } // PI / 2

    // static inline fixedpoint_type G_PI()
    // { return fixedpoint_type(13493037705LL,  fixedpoint_type::CNoShift()); } // PI

    // static inline fixedpoint_type G_3PI_2()
    // { return fixedpoint_type(20239556557LL,  fixedpoint_type::CNoShift()); } // (3 / 2) * PI

    // static inline fixedpoint_type G_2PI()
    // { return fixedpoint_type(26986075409LL,  fixedpoint_type::CNoShift()); } // 2 * PI

    // static inline fixedpoint_type G_NEGATIVE_1()
    // { return fixedpoint_type(-4294967296LL,  fixedpoint_type::CNoShift()); } // -1

    // static inline fixedpoint_type G_1()
    // { return fixedpoint_type(4294967296LL,   fixedpoint_type::CNoShift()); } // 1

    // static inline fixedpoint_type G_0p28()
    // { return fixedpoint_type(1202590843LL,   fixedpoint_type::CNoShift()); } //! needed for atan

    // static inline fixedpoint_type G_LN2()
    // { return fixedpoint_type(2977044472LL,   fixedpoint_type::CNoShift()); } //! needed for exp
// };
// PRQA S 474  --
// (Msg Enable 474: This literal is of the non standard type 'long long'.)

}  // close namespace vfc

#endif

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: include/vfc/core/vfc_fixedpoint_math.hpp  $
//  Revision 1.21 2010/11/10 15:08:57MEZ Muehlmann Karsten (CC/PJ-FA1) (MUK2LR) 
//  (re-)enable compilation of TFixedPointSubstitute (mantis3524)
//  Revision 1.20 2009/05/28 10:12:06CEST Muehlmann Karsten (CC-DA/ESV2) (MUK2LR) 
//  - replace std:: with stlalias:: for STL parts (not for math nor iostream) (mantis2720)
//  Revision 1.19 2009/03/25 15:59:45CET Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  - Linebreak at 120 in all fixedpoint files. (mantis 0002692)
//  Revision 1.18 2009/01/31 12:58:00IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Removal of QAC++ warnings.
//  (Mantis : 0002548)
//  Revision 1.17 2008/09/11 14:15:39IST Voelz Henning (CC-DA/ESV4) (VOH2HI)
//  remerge and close branch, because no longer  int64 with long_int supported
//  Revision 1.13.1.1 2008/08/12 10:29:09CEST Voelz Henning (CC-DA/ESV1) (VOH2HI)
//  floating point c'tor removed
//  Revision 1.13 2008/01/08 16:32:27CET Voelz Henning (AE-DA/ESV1) (voh2hi)
//  gcc bugfix
//  Revision 1.12 2008/01/08 10:06:04CET Voelz Henning (AE-DA/ESV1) (voh2hi)
//  remove TFixedPointValuePolicy<vfc::fp64s32_t> due to problems with subsitute class
//  Revision 1.11 2008/01/04 15:11:18CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Added some 64bit types.
//  Revision 1.10 2007/11/19 08:38:54CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Enables 0p28 for more fixedpoint types.
//  Revision 1.9 2007/11/05 16:10:09CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Fixed bug in exp. Pow now needs an extra template argument needed to work with exp.
//  Revision 1.8 2007/11/05 09:09:11CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  added atan and minor change in ln
//  Revision 1.7 2007/10/22 12:55:03CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  gcc bugfix
//  Revision 1.6 2007/10/22 11:10:55CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  gcc errors fixed
//  Revision 1.5 2007/10/22 09:29:26CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  cosmetics; MAX, EPSILON and 0 added
//  Revision 1.4 2007/10/19 16:46:38CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  Replaced static const members with static inline functions.
//  Revision 1.3 2007/10/19 14:33:08CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  definitions put to inl file
//  Revision 1.2 2007/10/19 13:30:58CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  PI/2 added for fp32p30_t
//  Revision 1.1 2007/10/19 13:13:21CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_fixedpoint/vfc_fixedpoint.pj
//=============================================================================
