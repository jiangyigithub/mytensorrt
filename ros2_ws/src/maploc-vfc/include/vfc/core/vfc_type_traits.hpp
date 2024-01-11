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
//  Department:
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_type_traits.hpp $
///     $Revision: 1.23 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/03/27 10:40:21MEZ $
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

#ifndef VFC_TYPE_TRAITS_HPP_INCLUDED
#define VFC_TYPE_TRAITS_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_type_traits_def.hpp"
#include "vfc/core/vfc_metaprog.hpp"    // used for TMin<>

namespace vfc
{    // namespace vfc opened

    //=========================================================================
    //  DOYGEN DEFGROUP vfc_group_core_generic_typetraits BEGIN
    //-------------------------------------------------------------------------
    /// @defgroup vfc_group_core_generic_typetraits Type Traits
    /// @ingroup vfc_group_core_generic
    /// @brief vfc type traits.
    /// <b>Generic programming</b> -- that is, writing code that works with any data
    /// type meeting a set of requirements -- has become the method of choice
    /// for delivering reusable code.
    ///
    /// However, there are times in generic programming when generic just isn't
    /// good enough -- sometimes the differences between types are too great for
    /// an efficient generic implementation.
    ///
    /// <b>This is when the traits technique becomes important.</b>
    ///
    /// By encapsulating those properties that need to be considered on a
    /// type-by-type basis inside a traits class, you can minimize the amount of
    /// code that has to differ from one type to another, and maximize the amount
    /// of generic code.
    ///
    /// For example, when working with character strings, one common operation
    /// is to determine the length of a null-terminated string.
    /// Clearly, it's possible to write generic code that can do this, but it
    /// turns out that there are much more efficient methods available.
    /// The C library functions @e strlen and @e wcslen, for instance, are usually
    /// written in assembler, and with suitable hardware support can be considerably
    /// faster than a generic version written in C++.
    /// The authors of the C++ Standard Library realized this, and abstracted
    /// the properties of char and wchar_t into the class char_ traits.
    /// Generic code that works with character strings can simply use
    /// @e char_traits<>::length to determine the length of a null-terminated string,
    /// safe in the knowledge that specializations of char_traits will use the
    /// most appropriate method available to them.
    ///
    /// <b>Type Traits</b>
    ///
    /// Class @e char_traits is a classic example of a collection of type-specific
    /// properties wrapped up in a single class -- what Nathan Myers terms a
    /// "baggage class" (see "Traits," by Nathan Myers, C++ Report, June 1995).
    /// In the vfc type-traits module we have written a set of very specific
    /// traits classes, each of which encapsulate a single trait from the C++
    /// type system.
    /// For example, is a type a pointer or a reference type, or does a type have a
    /// trivial constructor, or a const qualifier?
    /// The type-traits classes share a unified design. Each class has a single
    /// member called @e value -- a compile-time constant that is true if the type
    /// has the specified property, and false otherwise.
    /// These classes can be used in generic programming to determine the properties
    /// of a given type and introduce optimizations that are appropriate for that case.
    ///
    /// @b Implementation
    ///
    /// There are far too many separate classes contained in the type-traits library
    /// to give a full implementation here (see the source code in the vfc
    /// library for full details).
    /// However, most of the implementation is fairly repetitive anyway, so in
    /// this article we will give you a flavor for how some of the classes are
    /// implemented.
    /// Beginning with possibly the simplest class in the library, TIsVoid has
    /// a member @e value that is true only if T is @e void
    ///
    /// @code
    /// template <class T>
    /// struct TIsVoid
    /// {
    ///     enum { value = false};
    /// };
    ///
    /// template <>
    /// struct TIsVoid<void>
    /// {
    ///     enum { value = true};
    /// };
    /// @endcode
    ///
    /// Here we have defined the primary version of the template class TIsVoid,
    /// and provided a full specialization when T is void.
    /// While full specialization of a template class is an important technique,
    /// you sometimes need a solution that is halfway between a fully generic solution,
    /// and a full specialization.
    /// This is exactly the situation for which the standards committee defined
    /// partial template-class specialization.
    /// To illustrate, consider the class @e TIsPointer. Here we needed a primary
    /// version that handles all the cases where T is not a pointer, and a partial
    /// specialization to handle all the cases where T is a pointer.
    ///
    /// <b>Optimized copy</b>
    ///
    /// As an @b example of how the type-traits classes can be used, consider the
    /// standard library algorithm copy:
    ///
    /// @code
    /// template<typename Iter1, typename Iter2>
    /// Iter2 copy(Iter1 first, Iter1 last, Iter2 out);
    /// @endcode
    ///
    /// Obviously, there's no problem writing a generic version of copy that
    /// works for all iterator types Iter1 and Iter2; however, there are some
    /// circumstances when the copy operation can best be performed by a call to
    /// memcpy.
    ///
    /// To implement copy in terms of memcpy, all of the following
    /// conditions need to be met:
    /// -   Both of the iterator types Iter1 and Iter2 must be pointers.
    /// -   Both Iter1 and Iter2 must point to the same type, excluding const
    ///     and volatile qualifiers.
    /// -   The type pointed to by Iter1 must have a trivial assignment operator.
    ///     By "trivial assignment operator," we mean that the type is either a
    ///     scalar type (that is, an arithmetic type, enumeration type, pointer,
    ///     pointer to member, or const- or volatile-qualified version of one
    ///     of these types) or:
    /// -   The type has no user-defined assignment operator.
    /// -   The type does not have any data members that are references.
    /// -   All base classes, and all data member objects must have trivial
    ///     assignment operators.
    /// .
    ///
    /// If all these conditions are met, then a type can be copied using memcpy
    /// rather than using a compiler-generated assignment operator.
    ///
    /// The type-traits library provides a class THasTrivialCopy,
    /// such that THasTrivialCopy::value is true only if T has a trivial
    /// assignment operator. This class "just works" for scalar types, but has
    /// to be explicitly specialized for class/struct types that also happen to
    /// have a trivial assignment operator.
    /// In other words, if THasTrivialCopy gives the wrong answer, it will give
    /// the safe wrong answer -- that trivial assignment is not allowable.
    /// @{
    //=========================================================================

    //=========================================================================
    //  TIsVoid<>
    //-------------------------------------------------------------------------
    //! @struct vfc::TIsVoid
    //! @brief brief.
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(TIsVoid, false);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsVoid, void, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsVoid, void const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsVoid, void volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsVoid, void const volatile, true);

    //=========================================================================
    //  TIsC99Integral<>
    //-------------------------------------------------------------------------
    //! @struct vfc::TIsC99Integral
    //! @brief brief!
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(TIsC99Integral, false);

    // signed
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int8_t, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int16_t, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int32_t, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int8_t const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int16_t const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int32_t const, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int8_t volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int16_t volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int32_t volatile, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int8_t const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int16_t const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int32_t const volatile, true);

    // unsigned
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint8_t, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint16_t, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint32_t, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint8_t const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint16_t const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint32_t const, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint8_t volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint16_t volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint32_t volatile, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint8_t const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint16_t const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint32_t const volatile, true);

#ifndef    VFC_NO_INT64
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int64_t, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint64_t, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int64_t const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint64_t const, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int64_t volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint64_t volatile, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, int64_t const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99Integral, uint64_t const volatile, true);
#endif

    //=========================================================================
    //  TIsCPPIntegral<>
    //-------------------------------------------------------------------------
    //! @struct vfc::TIsCPPIntegral
    //! @brief brief!
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(TIsCPPIntegral, false);

    // bool
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, bool, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, bool const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, bool volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, bool const volatile, true);

// PRQA S 2427, 2428 ++
    // signed
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed char, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed short, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed int, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed long, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed char const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed short const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed int const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed long const, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed char volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed short volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed int volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed long volatile, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed char const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed short const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed int const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed long const volatile, true);

    // unsigned
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned char, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned short, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned int, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned long, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned char const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned short const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned int const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned long const, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned char volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned short volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned int volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned long volatile, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned char const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned short const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned int const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned long const volatile, true);

// PRQA S 47, 48 ++
#ifdef VFC_HAS_LONG_LONG
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed long long, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned long long, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed long long const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned long long const, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed long long volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned long long volatile, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, signed long long const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPIntegral, unsigned long long const volatile, true);
#endif
// PRQA S 47, 48 --

    //=========================================================================
    //  TIsIntegral<>
    //-------------------------------------------------------------------------
    //! brief.
    //=========================================================================

    template <class T>
    struct TIsIntegral
    {
        enum { value = TIsCPPIntegral<T>::value || TIsC99Integral<T>::value };
    };

    //=========================================================================
    //  TIsFloating<>
    //-------------------------------------------------------------------------
    //! @struct vfc::TIsFloating
    //! @brief brief.
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(TIsFloating, false);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating, float, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating, double, true);
    // ISO 14882 §3.9.1, long double
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating, long double, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating, float const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating, double const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating, long double const, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating, float volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating, double volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating, long double volatile, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating, float const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating, double const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating, long double const volatile, true);

    //=========================================================================
    //  TIsArithmetic<> = Floating || Integral
    //-------------------------------------------------------------------------
    //! @struct vfc::TIsArithmetic
    //! @brief brief.
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(TIsArithmetic,(TIsFloating<T>::value || TIsIntegral<T>::value));

    //=========================================================================
    //  TIsCPPUnsignedArithmetic<>
    //-------------------------------------------------------------------------
    //! @struct vfc::TIsCPPUnsignedArithmetic
    //! @brief brief.
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(TIsCPPUnsignedArithmetic, false);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned char, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned short, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned int, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned long, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned char const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned short const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned int const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned long const, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned char volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned short volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned int volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned long volatile, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned char const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned short const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned int const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned long const volatile, true);

// PRQA S 48 ++
#ifdef VFC_HAS_LONG_LONG
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned long long, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned long long const, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned long long volatile, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsCPPUnsignedArithmetic, unsigned long long const volatile, true);
#endif
// PRQA S 48--
// PRQA S 2427, 2428 --

    //=========================================================================
    //  TIsC99UnsignedArithmetic<>
    //-------------------------------------------------------------------------
    //! @struct vfc::TIsC99UnsignedArithmetic
    //! @brief brief.
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(TIsC99UnsignedArithmetic, false);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint8_t, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint16_t, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint32_t, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint8_t const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint16_t const, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint32_t const, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint8_t volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint16_t volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint32_t volatile, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint8_t const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint16_t const volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint32_t const volatile, true);

#ifndef    VFC_NO_INT64
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint64_t, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint64_t const, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint64_t volatile, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsC99UnsignedArithmetic, uint64_t const volatile, true);
#endif

    //=========================================================================
    //  TIsUnsignedArithmetic<>
    //-------------------------------------------------------------------------
    //! brief.
    //=========================================================================

    template <class T>
    struct TIsUnsignedArithmetic
    {
        enum { value = TIsCPPUnsignedArithmetic<T>::value || TIsC99UnsignedArithmetic<T>::value };
    };

    //=========================================================================
    //  TIsFundamental<>  = Arithmetic || void
    //-------------------------------------------------------------------------
    //! @struct vfc::TIsFundamental
    //! @brief brief.
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(TIsFundamental, (TIsArithmetic<T>::value || TIsVoid<T>::value));

    //=========================================================================
    //  TIsPointer<>
    //-------------------------------------------------------------------------
    //! @struct vfc::TIsPointer
    //! @brief brief.
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(TIsPointer, false);

    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(TIsPointer, T*, true);
    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(TIsPointer, T* const, true);
    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(TIsPointer, T* volatile, true);
    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(TIsPointer, T* const volatile, true);

    //=========================================================================
    //  TIsPOD<>  = Fundamental || Pointer
    //-------------------------------------------------------------------------
    //! @struct vfc::TIsPOD
    //! @brief brief.
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(TIsPOD,(TIsFundamental<T>::value || TIsPointer<T>::value));

    //=========================================================================
    //  THasTrivialCTor<>
    //-------------------------------------------------------------------------
    //! @struct vfc::THasTrivialCTor
    //! @brief brief.
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(THasTrivialCTor, TIsPOD<T>::value);

    //=========================================================================
    //  THasTrivialDTor<>
    //-------------------------------------------------------------------------
    //! @struct vfc::THasTrivialDTor
    //! @brief brief.
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(THasTrivialDTor, TIsPOD<T>::value);

    //=========================================================================
    //  THasTrivialCopy<>   = memcpy succeeds
    //-------------------------------------------------------------------------
    //! @struct vfc::THasTrivialCopy
    //! @brief brief.
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(THasTrivialCopy, TIsPOD<T>::value);

    //=========================================================================
    //  THasTrivialSetZero<>   = memset(0) succeeds
    //-------------------------------------------------------------------------
    //! @struct vfc::THasTrivialSetZero
    //! @brief brief.
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(THasTrivialSetZero, TIsPOD<T>::value);

    //=========================================================================
    //  THasTrivialSet<>   = memset(val) succeeds
    //-------------------------------------------------------------------------
    //! @struct vfc::THasTrivialSet
    //! @brief brief.
    //=========================================================================

    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(THasTrivialSet, false);

// PRQA S 2428 ++
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(THasTrivialSet, signed char, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(THasTrivialSet, unsigned char, true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(THasTrivialSet, signed char volatile, true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(THasTrivialSet, unsigned char volatile, true);
// PRQA S 2428 --

    //=========================================================================
    //  TIsArray<>
    //-------------------------------------------------------------------------
    //! @struct vfc::TIsArray
    //! @brief brief.
    //=========================================================================

    template<class ValueType>
    struct TIsArray
    {
        enum 
        { 
            value = false 
        };
    };

    template<class ValueType, vfc::int32_t SizeValue>
    struct TIsArray<ValueType[SizeValue]>
    {
        enum 
        { 
            value = true 
        };
    };

    template<class ValueType>
    struct TIsArray<ValueType[]>
    {
        enum 
        { 
            value = true 
        };
    };

    //=========================================================================
    //  Helper classed for TAlignmentOf<>
    //=========================================================================

    //-------------------------------------------------------------------------
    // conditional doxygen documentation
    //! @cond VFC_DOXY_INTERN
    //-------------------------------------------------------------------------

        namespace intern
        {


            //! helper struct for computing memory alignment
            //! @sa TAlignmentOf
            template <class T>
            class TAlignmentHelper
            {
                TAlignmentHelper();    // intentionally not implemented
                char_t      m_char;        // add one char to increment sizeof(T)
                T           m_argument;
            };

        }

    //-------------------------------------------------------------------------
    //! @endcond
    // of VFC_DOXY_INTERN
    //-------------------------------------------------------------------------

    //=========================================================================
    //  TAlignmentOf<>
    //-------------------------------------------------------------------------
    //! Traits for computing memory alignment restrictions for specified type T.
    //! @par Descpription:
    //! For an indepth documentation see @ref page_core_memory_alignment
    //! @param T type for which the memory alignment should be determined.
    //! @author zvh2hi
    //=========================================================================

    template <class T>
    struct TAlignmentOf
    {
        enum    
        { 
            value =    TMin
                            <sizeof(intern::TAlignmentHelper<T>)-sizeof(T),
                             sizeof(T)
                            >::value
        };
    };

    // assuming that T& is implemented as T*
    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(TAlignmentOf, T&, TAlignmentOf<T*>::value);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TAlignmentOf, void, 0);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TAlignmentOf, void const, 0);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TAlignmentOf, void volatile, 0);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TAlignmentOf, void const volatile, 0);

    //=========================================================================
    // TRectAreaTypeTraits
    //-------------------------------------------------------------------------
    //! Trait to identify the type promotion.
    //! @par Descpription:
    //! ValueType is used to pass the unit in case of siunit or simple POD.
    //! @param ValueType type for which the area has to be calculated.
    //! @author zvh2hi
    //! @ingroup vfc_group_core_types
    //=========================================================================
    template <class ValueType>
    struct TRectAreaTypeTraits
    {
        VFC_STATIC_ASSERT(!(vfc::TIsUnsignedArithmetic<ValueType>::value));
        typedef ValueType area_type;
    };

    //=========================================================================
    //  DOYGEN DEFGROUP vfc_group_core_generic_typetraits END
    //-------------------------------------------------------------------------
    /// @}
    //=========================================================================

}    // namespace vfc closed

#endif //VFC_TYPE_TRAITS_HPP_INCLUDED


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_type_traits.hpp  $
//  Revision 1.23 2014/03/27 10:40:21MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Add an isArray trait to vfc_type_traits (mantis0004456)
//  Revision 1.22 2013/07/24 10:50:30MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_type_traits.hpp: violation coding rule 1.2.4 (mantis0004264)
//  Revision 1.21 2012/10/23 11:40:56MESZ SNU5KOR 
//  -Added support for the new module TRect under vfc::core (mantis3349)
//  Revision 1.20 2009/02/03 05:49:58CET Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Removal of QAC++ warnings.
//  (Mantis : 0002507)
//  Revision 1.19 2008/08/25 18:20:08IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - changed TAlignmentHelper from struct to class (mantis 1697)
//  Revision 1.18 2007/08/02 15:48:25CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added conditional doxygen documentation generation of vfc::intern (mantis1758)
//  Revision 1.17 2007/07/23 09:36:06CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - doxygen grouping (mantis1744)
//  - added documentation
//  Revision 1.16 2007/05/11 13:08:29CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added documentation for TAlignmentOf (mantis1611)
//  Revision 1.15 2006/12/07 16:59:01CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added missing specializations (mantis1337)
//  Revision 1.14 2006/11/16 14:41:11CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.13 2006/11/09 14:55:16CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added TIsUnsignedArithmetic<> specialization for unsigned int type (mantis1286)
//  - added qualified (const, volatile, const volatile) types as well (mantis1286)
//  Revision 1.12 2006/11/03 09:51:16CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - corrected projectname
//  Revision 1.11 2006/11/03 09:47:08CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - bug fix in THasTrivialSet (mantis1248)
//  Revision 1.10 2006/03/09 09:17:34CET Muehlmann Karsten (AE-DA/ESA3) * (muk2lr)
//  - split unsigned arithmetic into C99 and CPP, TIsUnsignedArithmetic combines both (mantis1024)
//  Revision 1.9 2006/03/06 15:24:16CET Muehlmann Karsten (AE-DA/ESA3) * (muk2lr)
//  added specialization for unsigned long long
//  Revision 1.8 2005/10/28 10:30:21CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/core.pj
//  Revision 1.7 2005/10/06 16:52:27CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/vfc.pj
//  Revision 1.6 2005/04/20 14:20:57CEST zvh2hi
//  fixed several c99 / C++ type problems - works now for CW and MSVC
//  Revision 1.5 2005/04/18 17:14:07CEST zvh2hi
//  moved TInt2Boolean<> from vfc_type_traits.hpp to vfc_metaprog.hpp
//  Revision 1.4 2005/04/18 11:57:57CEST zvh2hi
//  -added TAlignmentOf<>
//  -moved #defines to vfc_type_traits_def.hpp
//  Revision 1.3 2005/04/07 14:55:53CEST zvh2hi
//  changed enum values from 0/1 to false/true - using standard bool - int conversion
//  Revision 1.2 2005/04/06 17:07:34CEST zvh2hi
//  added Integral to Boolean type traits
//=============================================================================
