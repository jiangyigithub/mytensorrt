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
//       Projectname: dummy_test
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
//  Department:
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_algorithm2d.inl $
///     $Revision: 1.7 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/08/12 13:41:36MESZ $
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

#include "vfc/core/vfc_assert.hpp"
#include "vfc/core/vfc_algorithm_helper.hpp"
#include "vfc/core/vfc_metaprog.hpp" 


//! loop unrolling workaround for compilers choking otherwise (GCC 3.4.1 on PPC and ARM RVCT 4.0.697)
#if ( (defined VFC_COMPILER_GCC) && (defined VFC_EPPC_DETECTED) && (__GNUC__ == 3) && (__GNUC_MINOR__ == 4) && (__GNUC_PATCHLEVEL__ ==1) )
#    define VFC_PATCH_UNROLL_VARIABLE_VOLATILE
#endif

#if ( (defined VFC_COMPILER_ARMRVCT) && (defined VFC_ARM_DETECTED) && (__ARMCC_VERSION == 400697) )
#    define VFC_PATCH_UNROLL_VARIABLE_VOLATILE
#endif

//=============================================================================
// modified STL for 2D random-access iterators (op()(n1,n2) round brackets)
//=============================================================================

//=============================================================================
//    vfc::for_each_2d_round_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The algorithm for_each is very flexible, allowing the modification of each
/// element within a range in different, user-specified ways.
/// Templatized functions may be reused in a modified form by passing different
/// parameters. User-defined functions may accumulate information within an
/// internal state that the algorithm may return after processing all of the
/// elements in the range. @n
/// The elements in the range @b must be accessible with a 2D round bracket
/// operator()(int,int).
/// @par implicit functor interface
/// @code void FuncType::operator()(ValueType&) @endcode
/// @return the function object.
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d.hpp
//=============================================================================

template <class InArgType, class FuncType, class CountType>
inline
FuncType    vfc::for_each_2d_round_n(InArgType& f_inArg, FuncType f_func, CountType f_count1,
    CountType f_count2)
{
    VFC_REQUIRE(f_count1 >= 0);
    VFC_REQUIRE(f_count2 >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count2 = f_count2;
    const unsigned_counttype unaligned = (l_count2&intern::CUnroll4::BITMASK);

    for (CountType i = 0; i < f_count1; ++i)
    {
#ifdef VFC_PATCH_UNROLL_VARIABLE_VOLATILE
        volatile
#endif
        unsigned_counttype j = 0;
        while (j < unaligned)
        {
            f_func( f_inArg(i, j));
            ++j;
        }

        while (j < l_count2)
        {
            f_func( f_inArg(i, j));
            f_func( f_inArg(i, j+1));
            f_func( f_inArg(i, j+2));
            f_func( f_inArg(i, j+3));
            j+=intern::CUnroll4::OFFSET;
        }
    }
    return f_func;
}

//=============================================================================
//    vfc::for_each_2d_round_n()
//-----------------------------------------------------------------------------
/// The elements in the range @b must be accessible with a 2D round bracket
/// operator()(int,int).
/// @sa for_each_2d_round_n
/// @par implicit functor interface
/// @code void FuncType::operator()(Value1Type&, Value2Type&) @endcode
/// @return the function object.
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d.hpp
//=============================================================================

template <class IOArg1Type, class IOArg2Type, class FuncType, class CountType>
inline
FuncType    vfc::for_each_2d_round_n(IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2,
    FuncType f_func, CountType f_count1, CountType f_count2)
{
    VFC_REQUIRE(f_count1 >= 0);
    VFC_REQUIRE(f_count2 >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count2 = f_count2;
    const unsigned_counttype unaligned = (l_count2&intern::CUnroll4::BITMASK);

    for (CountType i = 0; i < f_count1; ++i)
    {
#ifdef VFC_PATCH_UNROLL_VARIABLE_VOLATILE
        volatile
#endif
        unsigned_counttype j = 0;
        while(j < unaligned)
        {
            f_func( f_ioArg1(i, j), f_ioArg2(i, j));
            ++j;
        }

        while (j < l_count2)
        {
            f_func( f_ioArg1(i, j),     f_ioArg2(i, j));
            f_func( f_ioArg1(i, j+1),   f_ioArg2(i, j+1));
            f_func( f_ioArg1(i, j+2),   f_ioArg2(i, j+2));
            f_func( f_ioArg1(i, j+3),   f_ioArg2(i, j+3));
            j+=intern::CUnroll4::OFFSET;
        }
    }
    return f_func;
}

//=============================================================================
//    vfc::for_each_2d_round_n()
//-----------------------------------------------------------------------------
/// The elements in the range @b must be accessible with a 2D round bracket
/// operator()(int,int).
/// @sa for_each_2d_round_n
/// @par implicit functor interface
/// @code
/// void FuncType::operator()(Value1Type&, Value2Type&, Value3Type&)
/// @endcode
/// @return the function object.
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d.hpp
//=============================================================================

template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType,
    class CountType>
inline
FuncType    vfc::for_each_2d_round_n(IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2,
    IOArg3Type& f_ioArg3, FuncType f_func, CountType f_count1, CountType f_count2)
{
    VFC_REQUIRE(f_count1 >= 0);
    VFC_REQUIRE(f_count2 >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count2 = f_count2;
    const unsigned_counttype unaligned = (l_count2&intern::CUnroll4::BITMASK);

    for (CountType i = 0; i < f_count1; ++i)
    {
#ifdef VFC_PATCH_UNROLL_VARIABLE_VOLATILE
        volatile
#endif
        unsigned_counttype j = 0;
        while(j < unaligned)
        {
            f_func( f_ioArg1(i, j), f_ioArg2(i, j), f_ioArg3(i,j));
            ++j;
        }

        while (j < l_count2)
        {
            f_func( f_ioArg1(i, j),     f_ioArg2(i, j),     f_ioArg3(i,j));
            f_func( f_ioArg1(i, j+1),   f_ioArg2(i, j+1),   f_ioArg3(i,j+1));
            f_func( f_ioArg1(i, j+2),   f_ioArg2(i, j+2),   f_ioArg3(i,j+2));
            f_func( f_ioArg1(i, j+3),   f_ioArg2(i, j+3),   f_ioArg3(i,j+3));
            j+=intern::CUnroll4::OFFSET;
        }
    }
    return f_func;
}

//=============================================================================
//    vfc::generate_2d_round_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The function object is invoked for each element in the range and does not
/// need to return the same value each time it is called. It may, for example,
/// read from a file or refer to and modify a local state.The generator's result
/// type must be convertible to the value type of the forward iterators for the
/// range. @n
/// The elements in the range @b must be accessible with a 2D round bracket
/// operator()(int,int).
/// @par implicit functor interface
/// @code ValueType FuncType::operator()() @endcode
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d.hpp
//=============================================================================

template <class FwArgType, class FuncType, class CountType>
inline
void        vfc::generate_2d_round_n(FwArgType& f_fwArg, FuncType f_func,
    CountType f_count1, CountType f_count2)
{
    VFC_REQUIRE(f_count1 >= 0);
    VFC_REQUIRE(f_count2 >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count2 = f_count2;
    const unsigned_counttype unaligned = (l_count2&intern::CUnroll4::BITMASK);

    for (CountType i = 0; i < f_count1; ++i)
    {
#ifdef VFC_PATCH_UNROLL_VARIABLE_VOLATILE
        volatile
#endif
        unsigned_counttype j = 0;
        while(j < unaligned)
        {
            f_fwArg(i, j) = f_func();
            ++j;
        }

        while (j < l_count2)
        {
            f_fwArg(i, j)      = f_func();
            f_fwArg(i, j+1)    = f_func();
            f_fwArg(i, j+2)    = f_func();
            f_fwArg(i, j+3)    = f_func();
            j+=intern::CUnroll4::OFFSET;
        }
    }
}

//=============================================================================
//    vfc::transform_2d_round_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The elements in the range @b must be accessible with a 2D round bracket
/// operator()(int,int).\n
/// The destination range must be large enough to contain the transformed
/// source range. If f_destArg is set equal to f_inArg, then the source and
/// destination ranges will be the same and the sequence will be modified in
/// place.
/// @par implicit functor interface
/// @code DestValueType FuncType::operator()(const InValueType&) @endcode
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d.hpp
//=============================================================================

template <class InArgType, class OutArgType, class FuncType, class CountType>
inline
void        vfc::transform_2d_round_n(const InArgType& f_inArg, OutArgType& f_destArg,
    FuncType f_func, CountType f_count1, CountType f_count2)
{
    VFC_REQUIRE(f_count1 >= 0);
    VFC_REQUIRE(f_count2 >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count2 = f_count2;
    const unsigned_counttype unaligned = (l_count2&intern::CUnroll4::BITMASK);

    for (CountType i = 0; i < f_count1; ++i)
    {
#ifdef VFC_PATCH_UNROLL_VARIABLE_VOLATILE
        volatile
#endif
        unsigned_counttype j = 0;
        while(j < unaligned)
        {
            f_destArg(i, j) = f_func(f_inArg(i, j));
            ++j;
        }

        while (j < l_count2)
        {
            f_destArg(i, j)      = f_func(f_inArg(i, j));
            f_destArg(i, j+1)    = f_func(f_inArg(i, j+1));
            f_destArg(i, j+2)    = f_func(f_inArg(i, j+2));
            f_destArg(i, j+3)    = f_func(f_inArg(i, j+3));
            j+=intern::CUnroll4::OFFSET;
        }
    }
}

//=============================================================================
//    vfc::transform_2d_round_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The elements in the range @b must be accessible with a 2D round bracket
/// operator()(int,int).\n
/// The destination range must be large enough to contain the transformed
/// source range. If f_destArg is set equal to f_inArg, then the source and
/// destination ranges will be the same and the sequence will be modified in
/// place.
/// @par implicit functor interface
/// @code
/// DestValueType FuncType::operator()(const InValue1Type&, const InValue2Type&)
/// @endcode
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d.hpp
//=============================================================================

template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType,
    class CountType>
inline
void        vfc::transform_2d_round_n(const InArg1Type& f_inArg1,
const InArg2Type& f_inArg2, OutArgType& f_destArg, FuncType f_func, CountType f_count1,
    CountType f_count2)
{
    VFC_REQUIRE(f_count1 >= 0);
    VFC_REQUIRE(f_count2 >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count2 = f_count2;
    const unsigned_counttype unaligned = (l_count2&intern::CUnroll4::BITMASK);

    for (CountType i = 0; i < f_count1; ++i)
    {
#ifdef VFC_PATCH_UNROLL_VARIABLE_VOLATILE
        volatile
#endif
        unsigned_counttype j = 0;
        while(j < unaligned)
        {
            f_destArg(i, j) = f_func(f_inArg1(i, j),f_inArg2(i, j));
            ++j;
        }

        while (j < l_count2)
        {
            f_destArg(i, j)     = f_func(f_inArg1(i, j),    f_inArg2(i, j));
            f_destArg(i, j+1)   = f_func(f_inArg1(i, j+1),  f_inArg2(i, j+1));
            f_destArg(i, j+2)   = f_func(f_inArg1(i, j+2),  f_inArg2(i, j+2));
            f_destArg(i, j+3)   = f_func(f_inArg1(i, j+3),  f_inArg2(i, j+3));
            j+=intern::CUnroll4::OFFSET;
        }
    }
}

//=============================================================================
// modified STL for 2D random-access iterators arg[n1][n2] (square brackets)
//=============================================================================

//=============================================================================
//    vfc::for_each_2d_square_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The algorithm for_each is very flexible, allowing the modification of each
/// element within a range in different, user-specified ways.
/// Templatized functions may be reused in a modified form by passing different
/// parameters.User-defined functions may accumulate information within an
/// internal state that the algorithm may return after processing all of the
/// elements in the range.@n
/// The elements in the range @b must be accessible with a 2D square
/// bracket arg[n1][n2].
/// @par implicit functor interface
/// @code void FuncType::operator()(ValueType&) @endcode
/// @return the function object.
/// @par Example usage:
/// @code
/// std::vector< std::vector<int32_t> > iMat;
/// //...
/// int32_t sum = for_each_2d_square_n(iMat,TSum<int32_t>(), 2, 2).result();
/// // equals:
/// // sum = iMat[0][0] + iMat[0][1] + iMat[1][0] + iMat[1][1];
/// @endcode
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d.hpp
//=============================================================================

template <class InArgType, class FuncType, class CountType>
inline
FuncType    vfc::for_each_2d_square_n(InArgType& f_inArg, FuncType f_func,
    CountType f_count1, CountType f_count2)
{

    VFC_REQUIRE(f_count1 >= 0);
    VFC_REQUIRE(f_count2 >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count2 = f_count2;
    const unsigned_counttype unaligned = (l_count2&intern::CUnroll4::BITMASK);

    for (CountType i = 0; i < f_count1; ++i)
    {
#ifdef VFC_PATCH_UNROLL_VARIABLE_VOLATILE
        volatile
#endif
        unsigned_counttype j = 0;
        while(j < unaligned)
        {
            f_func( f_inArg[i][j]);
            ++j;
        }

        while (j < l_count2)
        {
            f_func( f_inArg[i][j]);
            f_func( f_inArg[i][j+1]);
            f_func( f_inArg[i][j+2]);
            f_func( f_inArg[i][j+3]);
            j+=intern::CUnroll4::OFFSET;
        }
    }
    return f_func;
}

//=============================================================================
//    vfc::for_each_2d_square_n()
//-----------------------------------------------------------------------------
/// The elements in the range @b must be accessible with a 2D square bracket
/// arg[n1][n2].
/// @sa for_each_2d_square_n
/// @par implicit functor interface
/// @code void FuncType::operator()(Value1Type&, Value2Type&) @endcode
/// @return the function object.
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d.hpp
//=============================================================================

template <class IOArg1Type, class IOArg2Type, class FuncType, class CountType>
inline
FuncType    vfc::for_each_2d_square_n(IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2,
    FuncType f_func, CountType f_count1, CountType f_count2)
{
    VFC_REQUIRE(f_count1 >= 0);
    VFC_REQUIRE(f_count2 >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count2 = f_count2;
    const unsigned_counttype unaligned = (l_count2&intern::CUnroll4::BITMASK);

    for (CountType i = 0; i < f_count1; ++i)
    {
#ifdef VFC_PATCH_UNROLL_VARIABLE_VOLATILE
        volatile
#endif
        unsigned_counttype j = 0;
        while(j < unaligned)
        {
            f_func( f_ioArg1[i][j], f_ioArg2[i][j]);
            ++j;
        }

        while (j < l_count2)
        {
            f_func( f_ioArg1[i][j],     f_ioArg2[i][j]);
            f_func( f_ioArg1[i][j+1],   f_ioArg2[i][j+1]);
            f_func( f_ioArg1[i][j+2],   f_ioArg2[i][j+2]);
            f_func( f_ioArg1[i][j+3],   f_ioArg2[i][j+3]);
            j+=intern::CUnroll4::OFFSET;
        }
    }
    return f_func;
}

//=============================================================================
//    vfc::for_each_2d_square_n()
//-----------------------------------------------------------------------------
/// The elements in the range @b must be accessible with a 2D square bracket
/// arg[n1][n2].
/// @sa for_each_2d_square_n
/// @par implicit functor interface
/// @code
/// void FuncType::operator()(Value1Type&, Value2Type&, Value3Type&)
/// @endcode
/// @return the function object.
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d.hpp
//=============================================================================

template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType,
    class CountType>
inline
FuncType    vfc::for_each_2d_square_n(IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2,
    IOArg3Type& f_ioArg3, FuncType f_func, CountType f_count1, CountType f_count2)
{
    VFC_REQUIRE(f_count1 >= 0);
    VFC_REQUIRE(f_count2 >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count2 = f_count2;
    const unsigned_counttype unaligned = (l_count2&intern::CUnroll4::BITMASK);

    for (CountType i = 0; i < f_count1; ++i)
    {
#ifdef VFC_PATCH_UNROLL_VARIABLE_VOLATILE
        volatile
#endif
        unsigned_counttype j = 0;
        while(j < unaligned)
        {
            f_func( f_ioArg1[i][j], f_ioArg2[i][j], f_ioArg3[i][j]);
            ++j;
        }

        while (j < l_count2)
        {
            f_func( f_ioArg1[i][j],     f_ioArg2[i][j],     f_ioArg3[i][j]);
            f_func( f_ioArg1[i][j+1],   f_ioArg2[i][j+1],   f_ioArg3[i][j+1]);
            f_func( f_ioArg1[i][j+2],   f_ioArg2[i][j+2],   f_ioArg3[i][j+2]);
            f_func( f_ioArg1[i][j+3],   f_ioArg2[i][j+3],   f_ioArg3[i][j+3]);
            j+=intern::CUnroll4::OFFSET;
        }
    }
    return f_func;
}

//=============================================================================
//    vfc::generate_2d_square_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The function object is invoked for each element in the range and does not
/// need to return the same value each time it is called. It may, for example,
/// read from a file or refer to and modify a local state. The generator's
/// result type must be convertible to the value type of the forward iterators
/// for the range. @n
/// The elements in the range @b must be accessible with a 2D square
/// bracket arg[n1][n2].
/// @par implicit functor interface
/// @code ValueType FuncType::operator()() @endcode
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d.hpp
//=============================================================================

template <class FwArgType, class FuncType, class CountType>
inline
void        vfc::generate_2d_square_n(FwArgType& f_fwArg, FuncType f_func,
    CountType f_count1, CountType f_count2)
{
    VFC_REQUIRE(f_count1 >= 0);
    VFC_REQUIRE(f_count2 >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count2 = f_count2;
    const unsigned_counttype unaligned = (l_count2&intern::CUnroll4::BITMASK);

    for (CountType i = 0; i < f_count1; ++i)
    {
#ifdef VFC_PATCH_UNROLL_VARIABLE_VOLATILE
        volatile
#endif
        unsigned_counttype j = 0;

        while(j < unaligned)
        {
            f_fwArg[i][j] = f_func();
            ++j;
        }

        while (j < l_count2)
        {
            f_fwArg[i][j]      = f_func();
            f_fwArg[i][j+1]    = f_func();
            f_fwArg[i][j+2]    = f_func();
            f_fwArg[i][j+3]    = f_func();
            j+=intern::CUnroll4::OFFSET;
        }
    }
}

//=============================================================================
//    vfc::transform_2d_square_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The elements in the range @b must be accessible with a 2D square bracket
/// arg[n1][n2]. @n
/// The destination range must be large enough to contain the transformed
/// source range. If f_destArg is set equal to f_inArg, then the source and
/// destination ranges will be the same and the sequence will be modified in
/// place.
/// @par implicit functor interface
/// @code DestValueType FuncType::operator()(const InValueType&) @endcode
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d.hpp
//=============================================================================

template <class InArgType, class OutArgType, class FuncType, class CountType>
inline
void        vfc::transform_2d_square_n(const InArgType& f_inArg, OutArgType& f_destArg,
    FuncType f_func, CountType f_count1, CountType f_count2)
{
    VFC_REQUIRE(f_count1 >= 0);
    VFC_REQUIRE(f_count2 >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count2 = f_count2;
    const unsigned_counttype unaligned = (l_count2&intern::CUnroll4::BITMASK);

    for (CountType i = 0; i < f_count1; ++i)
    {
#ifdef VFC_PATCH_UNROLL_VARIABLE_VOLATILE
        volatile
#endif
        unsigned_counttype j = 0;
        while(j < unaligned)
        {
            f_destArg[i][j] = f_func(f_inArg[i][j]);
            ++j;
        }

        while (j < l_count2)
        {
            f_destArg[i][j]      = f_func(f_inArg[i][j]);
            f_destArg[i][j+1]    = f_func(f_inArg[i][j+1]);
            f_destArg[i][j+2]    = f_func(f_inArg[i][j+2]);
            f_destArg[i][j+3]    = f_func(f_inArg[i][j+3]);
            j+=intern::CUnroll4::OFFSET;
        }
    }
}

//=============================================================================
//    vfc::transform_2d_square_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The elements in the range @b must be accessible with a 2D square bracket
/// arg[n1][n2].
/// The destination range must be large enough to contain the transformed
/// source range. If f_destArg is set equal to f_inArg, then the source and
/// destination ranges will be the same and the sequence will be modified in
/// place.
/// @par implicit functor interface
/// @code
/// DestValueType FuncType::operator()(const InValue1Type&, const InValue2Type&)
/// @endcode
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d.hpp
//=============================================================================

template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType,
    class CountType>
inline
void        vfc::transform_2d_square_n(const InArg1Type& f_inArg1, const InArg2Type& f_inArg2,
    OutArgType& f_destArg, FuncType f_func, CountType f_count1, CountType f_count2)
{
    VFC_REQUIRE(f_count1 >= 0);
    VFC_REQUIRE(f_count2 >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count2 = f_count2;
    const unsigned_counttype unaligned = (l_count2&intern::CUnroll4::BITMASK);

    for (CountType i = 0; i < f_count1; ++i)
    {
#ifdef VFC_PATCH_UNROLL_VARIABLE_VOLATILE
        volatile
#endif
        unsigned_counttype j = 0;
        while(j < unaligned)
        {
            f_destArg[i][j] = f_func(f_inArg1[i][j],f_inArg2[i][j]);
            ++j;
        }

        while (j < l_count2)
        {
            f_destArg[i][j]     = f_func(f_inArg1[i][j],    f_inArg2[i][j]);
            f_destArg[i][j+1]   = f_func(f_inArg1[i][j+1],  f_inArg2[i][j+1]);
            f_destArg[i][j+2]   = f_func(f_inArg1[i][j+2],  f_inArg2[i][j+2]);
            f_destArg[i][j+3]   = f_func(f_inArg1[i][j+3],  f_inArg2[i][j+3]);
            j+=intern::CUnroll4::OFFSET;
        }
    }
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_algorithm2d.inl  $
//  Revision 1.7 2014/08/12 13:41:36MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_algorithm: loop unrolling factor shall be made "unsigned" after the assertion for >0 (mantis0004593)
//  Revision 1.6 2012/12/18 08:27:28MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.5 2010/08/13 16:50:36MESZ Muehlmann Karsten (CC/PJ-FA1) (MUK2LR) 
//  prevent optimizing away unrolled loops not only fpr GCC 3.4.1 on PPC, but also ARM RVCT 4.0.697 (build 821 will have a fix for this) (mantis3274)
//  Revision 1.4 2009/07/14 05:42:49CEST Pavithra K T (RBEI/EAC1) (pkt1kor) 
//  - change of code to follow with coding rule 12.0.1 and to change revision style (mantis2864).
//  Revision 1.3 2009/07/08 09:18:56IST Pavithra K T (RBEI/EAC1) (pkt1kor) 
//  - gcc 341/eppc xcross infinite loop bug resolved by making the variable j volatile
//  -Mantis 2864
//  Revision 1.2 2007/06/22 18:42:24IST Jaeger Thomas (CC-DA/ESV2) (jat2hi)
//  - corrected redeclared function parameter names (mantis 1691)
//  Revision 1.1 2007/02/23 13:36:10CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/core.pj
//  Revision 1.7 2007/02/13 14:30:57CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - changed VFC_ASSERT to VFC_REQUIRE
//  Revision 1.6 2007/01/23 11:16:28CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - update: use algorith_helper for the CUnroll4 struct to prevent double definition
//  Revision 1.5 2007/01/23 10:18:53CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - changed the loop unrolling from for to while loops (mantis 1309)
//  Revision 1.4 2007/01/18 08:53:00CET dkn2kor
//  - Replaced size_t with a template parameter CountType (mantis1252)
//  Revision 1.3 2006/12/19 20:28:53IST dkn2kor
//  fixed the bug in 2d loop unrolling
//  Revision 1.2 2006/11/21 21:52:27IST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - documentation corrections
//  Revision 1.1 2006/11/16 16:46:13CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/algorithm_looped/algorithm_looped.pj
//  Revision 1.2 2006/11/16 15:56:12CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - changed the count parameter types from size_t to signed int
//  - added assertions for count parameters
//  - added loop unrolling (stepsize 4)
//  Revision 1.1 2006/10/19 16:01:46CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/algorithm_looped/algorithm_looped.pj
//  Revision 1.2 2006/10/06 09:34:37CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - made the functions STL conform
//  - removed one indirection
//  Revision 1.1 2006/10/05 09:20:39CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/algorithm_looped/algorithm_looped.pj
//  Revision 1.3 2006/09/29 14:44:43CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - added/fixed the generate functions
//  Revision 1.2 2006/09/29 13:07:35CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - added the square loops
//  - renamed "Unroll" to "Loop"
//  Revision 1.1 2006/09/29 12:48:23CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/internal-projects/ipj_ivs_AlgoDevelop/ipj_ivs_LinAlgBenchmark/matrixeval2/extern/uwnfu/uwnfu.pj
//  Revision 1.6 2006/09/29 10:51:12CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - removed one left unused argument
//  Revision 1.5 2006/09/29 09:36:25CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  -visual enhancements
//  Revision 1.4 2006/09/29 08:54:48CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - removed the unused arguments from the recursion ends
//  Revision 1.3 2006/09/28 18:02:25CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  -fixed and added docu
//  Revision 1.2 2006/09/28 11:35:26CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  -renamed algorithm2 to algorithm_idx2 in template metaprogram classes
//=============================================================================
