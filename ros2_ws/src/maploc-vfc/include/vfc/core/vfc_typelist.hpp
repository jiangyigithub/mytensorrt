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
//        Name: sf71lr 
//  Department: 
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_typelist.hpp $
///     $Revision: 1.8 $
///     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
///     $Date: 2015/11/05 12:56:30MEZ $
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

#ifndef VFC_TYPELIST_HPP_INCLUDED
#define VFC_TYPELIST_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

namespace vfc 
{
    //=========================================================================
    //  DOYGEN DEFGROUP vfc_group_core_generic_typelist BEGIN
    //-------------------------------------------------------------------------
    /// @defgroup vfc_group_core_generic_typelist Typelist
    /// @ingroup vfc_group_core_generic
    /// @brief vfc typelist implementation.
    /// Typelists are an important generic programming technique.\n
    /// They add new capabilities for library writers: expressing and
    /// manipulating arbitrarily large collections of types, generating data
    /// structures and code from such collections, and more.\n
    /// Typelists are useful when you have to write the same code - either
    /// declarative or imperative - for a collection of types. They enable you
    /// to abstract and generalite entities that escape all other template 
    /// programming techniques. For this reason, typelists are the enabling
    /// means for genuinely new idioms and library implementations.\n
    /// At compile time, typelists offer most primitive functions that lists of
    /// values typically implement: add (TTypeListAppend), access (TTypeListTypeAt),
    /// search (TTypeListIndexOf), erase, replace and more. The code that 
    /// implements typelist manipulation is confined to a pure functional style 
    /// because there are no compile-time mutable values - a type or compile-
    /// time constant, once defines, cannot be changed. For this reason most
    /// typelist processing relies on recursive templates and pattern matching
    /// through partial template specialization.\n
    /// @sa 
    /// - TTypeList
    /// - TTypeListSize
    /// - TTypeListAppend
    /// - TTypeListTypeAt
    /// - TTypeListIndexOf
    /// - TGenTypeList1
    /// .
    /// @{
    //=========================================================================
    

    //=========================================================================
    // TTypeList<>
    //-------------------------------------------------------------------------
    /// Typelist list node.
    /// Typelists hold two types. They are accessible through the Head and Tail
    /// inner names.\n 
    /// Typelists are devoid of any value: Their bodies are empty,
    /// they don't hold any state, and they don't define any functionality.
    /// at runtime typelists don't carry any value at all. Their only reason is 
    /// to carry type information.\n
    /// There is a little problem, though. We can express typelists of two types
    /// or more, but we're unable to express typelists containing zero or one
    /// type. What's needed is a null list type, and the CTypeListNullType is
    /// exactly suited for such use. We establish the convention that every
    /// typelist must end with a CTypeListNullType. \n
    /// Now we can define a typelist of only one element:
    /// @par
    /// @code
    /// typedef TTypeList<int, CTypeListNullType> OneTypeOnly; 
    /// @endcode
    ///
    /// For linearizing typelist creation we provide the TGenTypeList1 types
    /// generators.
    /// @sa 
    /// - CTypeListNullType
    /// - TGenTypeList1
    /// @author sf71lr 
    //=========================================================================

    template < typename HeadType, typename TailType> 
    struct TTypeList 
    {
        typedef HeadType Head;
        typedef TailType Tail;
    };
    
    //=========================================================================
    // CTypeListNullType
    //-------------------------------------------------------------------------
    /// Typelist termination marker.
    /// Every typelelist must end with a CTypeListNullType. CTypeListNullType
    /// serves as a useful termination marker, much like '\\0' that helps
    /// traditional C string functions.
    /// @sa TTypeList
    /// @author sf71lr 
    //=========================================================================

    struct 
    CTypeListNullType {};

    //=========================================================================
    // TTypeListSize<>
    //-------------------------------------------------------------------------
    /// @struct vfc::TTypeListSize
    /// @brief Returns the length of the specified type list.
    /// The idea underlying most typelists manipulations is to exploit 
    /// recursive templates, which are templates that uses instantiations of 
    /// themselves as part of their definition. The implementation of 
    /// TTypeListSize uses partial template specialization to distinguish
    /// between a CTypeListNullType and a TTypeList. When the Tail becomes 
    /// CTypeListNullType the recursion is stopped and so is the length 
    /// calculation which comes back nicely with the result.
    /// @return the list size stored in TTypeListSize<>::value
    /// @sa TTypeList
    /// @author sf71lr 
    //=========================================================================

    template <typename ListType> 
    struct TTypeListSize;

    //=========================================================================
    // TTypeListIndexOf<>
    //-------------------------------------------------------------------------
    /// @struct vfc::TTypeListIndexOf
    /// @brief Returns the index of the first occurence of the type in a typelist.
    /// TIndexOf::value holds either the index or -1 if the type was not found.
    /// @return the index or -1 (not found) in TTypeListIndexOf<>::value.
    /// @sa TTypeList
    /// @author sf71lr 
    //=========================================================================

    template <typename ListType, typename SearchType> 
    struct TTypeListIndexOf;

    //=========================================================================
    // TTypeListTypeAt<>
    //-------------------------------------------------------------------------
    /// @struct vfc::TTypeListTypeAt
    /// @brief Returns the type at specified index.
    /// Having access by index to the elements of a typelist is a desireable
    /// feature. It linearizes typelist access, making it easier to manipulate
    /// typelists comfortably.
    /// @return the type at specified index in TTypeListTypeAt<>::type
    /// @note results in a compile time error if specified IndexValue is 
    /// out-of-bounds.
    /// @sa TTypeList
    /// @author sf71lr 
    //=========================================================================

    template <class ListType, int32_t IndexValue> 
    struct TTypeListTypeAt;

    //=========================================================================
    // TTypeListAppend<>
    //-------------------------------------------------------------------------
    /// @struct vfc::TTypeListAppend
    /// @brief Appends a type or a typelist to specified type list.
    /// @return the enlarged type list in TTypeListAppend<>::type
    /// @sa TTypeList
    /// @author zvh2hi 
    //=========================================================================

    template <class ListType, class AppendType> 
    struct TTypeListAppend;

    //=========================================================================
    //  TGenTypeList1
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation of one type only.
    /// Right of the bat, typelists are just too LISP-ish to be easy to use.
    /// LISP-style constructs are great for LISP-programmers, but they don't
    /// dovetail nicely with C++. For instance, here's a typelist of integral
    /// types:
    /// @par
    /// @code
    /// typedef TTypeList   <  
    ///                     signed char,
    ///                     TTypeList   <  
    ///                                 short int,
    ///                                 TTypeList   <  
    ///                                             int,
    ///                                             TTypeList   < 
    ///                                                         long int,
    ///                                                         CTypeListNullType
    ///                                                         >
    ///                                             >
    ///                                 >
    ///                     > SignedIntegrals;
    /// @endcode
    ///
    /// Typelists might be a cool concept, but they definitely need a nicer
    /// packaging.\n
    /// In order to linearize typelist creation, vfc's typelist library
    /// defines several generator types that transform the recursion into
    /// simple enumeration, at the expense of tedious repetition. This is not
    /// a problem, however. The repitition is done only once and it scales 
    /// typelists to a large library-defined number (24).\n
    /// Now the earlier type definition can be expressed in a more pleasant
    /// way:
    /// @par
    /// @code
    /// typedef TGenTypeList4   <   signed char,
    ///                             short int,
    ///                             int,
    ///                             long int
    ///                         >::type SignedIntegrals;
    /// @endcode
    /// @sa 
    /// -   TGenTypeList2
    /// -   TGenTypeList3
    /// -   TGenTypeList4
    /// -   TGenTypeList5
    /// -   TGenTypeList6
    /// -   TGenTypeList7
    /// -   TGenTypeList8
    /// -   TGenTypeList9
    /// -   TGenTypeList10
    /// -   TGenTypeList11
    /// -   TGenTypeList12
    /// -   TGenTypeList13
    /// -   TGenTypeList14
    /// -   TGenTypeList15
    /// -   TGenTypeList16
    /// -   TGenTypeList17
    /// -   TGenTypeList18
    /// -   TGenTypeList19
    /// -   TGenTypeList20
    /// -   TGenTypeList21
    /// -   TGenTypeList22
    /// -   TGenTypeList23
    /// -   TGenTypeList24
    /// -   TTypeList
    /// @author jat2hi
    //=========================================================================

    template<class T1>
    struct TGenTypeList1
    {
        typedef 
        vfc::TTypeList<T1, vfc::CTypeListNullType> type;
    };

    //=========================================================================
    //  TGenTypeList2
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation of two types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2>
    struct TGenTypeList2
    {
        typedef 
        vfc::TTypeList<T1, typename vfc::TGenTypeList1<T2>::type > type;
    };

    //=========================================================================
    //  TGenTypeList3
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation of three types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3>
    struct TGenTypeList3
    {
        typedef 
        vfc::TTypeList<T1, typename vfc::TGenTypeList2<T2, T3>::type > type;
    };

    //=========================================================================
    //  TGenTypeList4
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation of four types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4>
    struct TGenTypeList4
    {
        typedef 
        vfc::TTypeList<T1, typename vfc::TGenTypeList3<T2, T3, T4>::type > type;
    };

    //=========================================================================
    //  TGenTypeList5
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation of five types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5>
    struct TGenTypeList5
    {
        typedef 
        vfc::TTypeList<T1, typename vfc::TGenTypeList4<T2, T3, T4, T5>::type > type;
    };

    //=========================================================================
    //  TGenTypeList6
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation of six types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6>
    struct TGenTypeList6
    {
        typedef 
        vfc::TTypeList<T1, typename vfc::TGenTypeList5<T2, T3, T4, T5, T6>::type > type;
    };

    //=========================================================================
    //  TGenTypeList7
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation of seven types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7>
    struct TGenTypeList7
    {
        typedef 
        vfc::TTypeList<T1, typename vfc::TGenTypeList6<T2, T3, T4, T5, T6,
                                                          T7>::type > type;
    };

    //=========================================================================
    //  TGenTypeList8
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation of eight types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8>
    struct TGenTypeList8
    {
        typedef 
        vfc::TTypeList<T1, typename vfc::TGenTypeList7<T2, T3, T4, T5, T6,
                                                          T7, T8>::type > type;
    };

    //=========================================================================
    //  TGenTypeList9
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation of nine types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9>
    struct TGenTypeList9
    {
        typedef 
        vfc::TTypeList<T1, typename vfc::TGenTypeList8<T2, T3, T4, T5, T6,
                                                          T7, T8, T9>::type > type;
    };

    //=========================================================================
    //  TGenTypeList10
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation of ten types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10>
    struct TGenTypeList10
    {
        typedef 
        vfc::TTypeList<T1, typename vfc::TGenTypeList9<T2, T3, T4, T5, T6,
                                                          T7, T8, T9, T10>::type > type;
    };

    //=========================================================================
    //  TGenTypeList11
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation of eleven types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10, class T11>
    struct TGenTypeList11
    {
        typedef 
        vfc::TTypeList<T1, typename vfc::TGenTypeList10<T2, T3, T4, T5, T6,
                                                           T7, T8, T9, T10, T11>::type > type;
    };

    //=========================================================================
    //  TGenTypeList12
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation of twelfe types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10, class T11, class T12>
    struct TGenTypeList12
    {
        typedef 
        vfc::TTypeList<T1, typename vfc::TGenTypeList11<T2, T3, T4, T5, T6,
                                                           T7, T8, T9, T10, T11, T12>::type > type;
    };
    
    //=========================================================================
    //  TGenTypeList13
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation 
    /// of thirteen types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10, class T11, class T12,
             class T13>
    struct TGenTypeList13
    {
        typedef 
        vfc::TTypeList< T1, typename vfc::TGenTypeList12<T2, T3, T4, T5, T6,
                                                         T7, T8, T9, T10, T11, 
                                                         T12, T13
                                                        >::type > type;
    };
    
    //=========================================================================
    //  TGenTypeList14
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation 
    /// of fourteen types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10, class T11, class T12,
             class T13, class T14>
    struct TGenTypeList14
    {
        typedef 
        vfc::TTypeList< T1, typename vfc::TGenTypeList13<T2, T3, T4, T5, T6,
                                                         T7, T8, T9, T10, T11, 
                                                         T12, T13, T14
                                                        >::type > type;
    };
    
    //=========================================================================
    //  TGenTypeList15
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation 
    /// of fifteen types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10, class T11, class T12,
             class T13, class T14, class T15>
    struct TGenTypeList15
    {
        typedef 
        vfc::TTypeList< T1, typename vfc::TGenTypeList14<T2, T3, T4, T5, T6,
                                                         T7, T8, T9, T10, T11, 
                                                         T12, T13, T14, T15
                                                        >::type > type;
    };
    
    //=========================================================================
    //  TGenTypeList16
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation 
    /// of sixteen types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10, class T11, class T12,
             class T13, class T14, class T15, class T16>
    struct TGenTypeList16
    {
        typedef 
        vfc::TTypeList< T1, typename vfc::TGenTypeList15<T2, T3, T4, T5, T6,
                                                         T7, T8, T9, T10, T11, 
                                                         T12, T13, T14, T15, 
                                                         T16
                                                        >::type > type;
    };
    
    //=========================================================================
    //  TGenTypeList17
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation 
    /// of seventeen types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10, class T11, class T12,
             class T13, class T14, class T15, class T16, class T17>
    struct TGenTypeList17
    {
        typedef 
        vfc::TTypeList< T1, typename vfc::TGenTypeList16<T2, T3, T4, T5, T6,
                                                         T7, T8, T9, T10, T11,
                                                         T12, T13, T14, T15, 
                                                         T16, T17
                                                        >::type > type;
    };
    
    //=========================================================================
    //  TGenTypeList18
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation 
    /// of eighteen types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10, class T11, class T12,
             class T13, class T14, class T15, class T16, class T17, class T18>
    struct TGenTypeList18
    {
        typedef 
        vfc::TTypeList< T1, typename vfc::TGenTypeList17<T2, T3, T4, T5, T6,
                                                         T7, T8, T9, T10, T11,
                                                         T12, T13, T14, T15, 
                                                         T16, T17, T18
                                                        >::type > type;
    };
    
    //=========================================================================
    //  TGenTypeList19
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation 
    /// of nineteen types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10, class T11, class T12,
             class T13, class T14, class T15, class T16, class T17, class T18,
             class T19>
    struct TGenTypeList19
    {
        typedef 
        vfc::TTypeList< T1, typename vfc::TGenTypeList18<T2, T3, T4, T5, T6,
                                                         T7, T8, T9, T10, T11,
                                                         T12, T13, T14, T15,
                                                         T16, T17, T18, T19
                                                        >::type > type;
    };
    
    //=========================================================================
    //  TGenTypeList20
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation 
    /// of twenty types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10, class T11, class T12,
             class T13, class T14, class T15, class T16, class T17, class T18,
             class T19, class T20>
    struct TGenTypeList20
    {
        typedef 
        vfc::TTypeList< T1, typename vfc::TGenTypeList19<T2, T3, T4, T5, T6,
                                                         T7, T8, T9, T10, T11,
                                                         T12, T13, T14, T15,
                                                         T16, T17, T18, T19, 
                                                         T20
                                                        >::type > type;
    };
    
    //=========================================================================
    //  TGenTypeList21
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation 
    /// of twenty-one types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10, class T11, class T12,
             class T13, class T14, class T15, class T16, class T17, class T18,
             class T19, class T20, class T21>
    struct TGenTypeList21
    {
        typedef 
        vfc::TTypeList< T1, typename vfc::TGenTypeList20<T2, T3, T4, T5, T6,
                                                         T7, T8, T9, T10, T11,
                                                         T12, T13, T14, T15, 
                                                         T16, T17, T18, T19, 
                                                         T20, T21
                                                        >::type > type;
    };
    
    //=========================================================================
    //  TGenTypeList22
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation 
    /// of twenty-two types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10, class T11, class T12,
             class T13, class T14, class T15, class T16, class T17, class T18,
             class T19, class T20, class T21, class T22>
    struct TGenTypeList22
    {
        typedef 
        vfc::TTypeList< T1, typename vfc::TGenTypeList21<T2, T3, T4, T5, T6,
                                                         T7, T8, T9, T10, T11,
                                                         T12, T13, T14, T15, 
                                                         T16, T17, T18, T19,
                                                         T20, T21, T22
                                                        >::type > type;
    };
    
    //=========================================================================
    //  TGenTypeList23
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation 
    /// of twenty-three types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10, class T11, class T12,
             class T13, class T14, class T15, class T16, class T17, class T18,
             class T19, class T20, class T21, class T22, class T23>
    struct TGenTypeList23
    {
        typedef 
        vfc::TTypeList< T1, typename vfc::TGenTypeList22<T2, T3, T4, T5, T6,
                                                          T7, T8, T9, T10, T11,
                                                          T12, T13, T14, T15, 
                                                          T16, T17, T18, T19, 
                                                          T20, T21, T22, T23
                                                         >::type > type;
    };
    
    //=========================================================================
    //  TGenTypeList24
    //-------------------------------------------------------------------------
    /// typelist generator for linearizing typelist creation 
    /// of twenty-four types.
    /// @sa TGenTypeList1
    //=========================================================================

    template<class T1, class T2, class T3, class T4, class T5, class T6,
             class T7, class T8, class T9, class T10, class T11, class T12,
             class T13, class T14, class T15, class T16, class T17, class T18,
             class T19, class T20, class T21, class T22, class T23, class T24>
    struct TGenTypeList24
    {
        typedef 
        vfc::TTypeList< T1, typename vfc::TGenTypeList23<T2, T3, T4, T5, T6,
                                                         T7, T8, T9, T10, T11,
                                                         T12, T13, T14, T15, 
                                                         T16, T17, T18, T19, 
                                                         T20, T21, T22, T23, 
                                                         T24
                                                        >::type > type;
    };
    
    //=========================================================================
    //  DOYGEN DEFGROUP vfc_group_core_generic_typelist END
    //-------------------------------------------------------------------------
    /// @}
    //=========================================================================

} // vfc namespace

#include "vfc/core/vfc_typelist.inl"

#endif // VFC_TYPELIST_HPP_INCLUDED



//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_typelist.hpp  $
//  Revision 1.8 2015/11/05 12:56:30MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  -  base_lib::vfc_adaption_TGenTypeList (mantis0004973)
//  Revision 1.7 2012/12/18 08:27:32MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.6 2011/01/21 12:53:30MEZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - sometimes .hpp or namespace qualifiers are missing (mantis3599)
//  Revision 1.5 2007/07/23 09:21:14MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - added docu (mantis1744)
//  Revision 1.4 2007/07/18 16:31:27CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen documentation grouping (mantis1744)
//  Revision 1.3 2007/03/15 10:05:42CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed naming of generator structs (mantis1489)
//  Revision 1.2 2007/03/14 16:43:33CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - replaced convenience macros by templated structs (mantis 1489)
//  Revision 1.1 2006/12/15 11:10:35CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/core.pj
//=============================================================================
