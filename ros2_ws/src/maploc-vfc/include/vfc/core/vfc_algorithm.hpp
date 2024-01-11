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
//       Projectname:
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
//        Name:
//  Department:
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @ingroup vfc_group_core_algorithms_stl
/// @par Revision History:
///     $Source: vfc_algorithm.hpp $
///     $Revision: 1.13 $
///     $Author: gaj2kor $
///     $Date: 2009/02/03 13:22:54MEZ $
///     $Locker:  $
///     $Name: 0032 RC1  $
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

#ifndef VFC_ALGORITHM_HPP_INCLUDED
#define VFC_ALGORITHM_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"   // used for size_t

namespace vfc
{    // namespace vfc opened

    //=========================================================================
    //  DOYGEN ADDTOGROUP vfc_group_core_algorithms_stl BEGIN
    //-------------------------------------------------------------------------
    /// @addtogroup vfc_group_core_algorithms_stl STL Enhancements
    /// @ingroup vfc_group_core_algorithms
    /// @brief STL enhancements.
    /// @{
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    //! applies a specified function object to each pair of elements of given
    //! sequences in a forward order  within a range and returns the function
    //! object.
    //! \par implicit function object interface:
    //! \code
    //! class FunctorType
    //! {
    //!     void operator()(const ValueType1& val1, const ValueType2& val2);
    //! };
    //! \endcode
    //! @author zvh2hi
    //! $Source: vfc_algorithm.hpp $
    //-------------------------------------------------------------------------
    template <class InputIterator1, class InputIterator2, class FunctorType>
    FunctorType    for_each        (  InputIterator1  f_first1, //!< an input iterator addressing the position of the first element in the first sequence in the range to be operated on
                                      InputIterator1  f_last1,  //!< an input iterator addressing the position one past the final element in the first sequence in the range operated on.
                                      InputIterator2  f_first2, //!< an input iterator addressing the position of the first element in the second sequence in the range to be operated on
                                      FunctorType     f_func    //!< user-defined function object that is applied to each pair of elements in the range.
                                );

    //-------------------------------------------------------------------------
    //! assigns the values of specified number of elements from a source range
    //! to a destination range, iterating through the source sequence of elements
    //! and assigning them new positions in a forward direction.
    //! @author zvh2hi
    //! $Source: vfc_algorithm.hpp $
    //-------------------------------------------------------------------------
    template <class InputIterator, class OutputIterator, class CountType>
    void        copy_n            (   InputIterator   f_first,  //!< an input iterator addressing the position of the first element in the source range.
                                      OutputIterator  f_result, //!< an output iterator addressing the position of the first element in the destination range.
                                      CountType       f_count   //!< an unsigned integer type specifying the number of elements to be assigned
                                );

    //-------------------------------------------------------------------------
    //! assigns the values of a specified number of elements from a source range
    //! to a destination range, iterating through the source sequence of elements
    //! and assigning them new positions in a backward direction.
    //! @author zvh2hi
    //! $Source: vfc_algorithm.hpp $
    //-------------------------------------------------------------------------
    template <class InputIterator, class OutputIterator, class CountType>
    void        copy_backward_n    (  InputIterator    f_first,  //!< a bidirectional iterator addressing the position of the first element in the source range.
                                      OutputIterator   f_result, //!< a bidirectional iterator addressing the position of the one past the final element in the destination range.
                                      CountType        f_count   //!< an unsigned integer type specifying the number of elements
                                );

    //-------------------------------------------------------------------------
    //! assigns a new value to a specified number of elements in a range beginning
    //! with a particular element.
    //! replaces std::fill_n() with an loop-unrolled implementation
    //! @author zvh2hi
    //! $Source: vfc_algorithm.hpp $
    //-------------------------------------------------------------------------
    template <class OutputIterator, class CountType, class T>
    void        fill_n            (  OutputIterator  f_first,  //!< an output iterator addressing the position of the first element in the range to be assigned the f_value.
                                     CountType       f_count,  //!< an unsigned integer type specifying the number of elements to be assigned the f_value
                                     const T&        f_value   //!< the f_value to be assigned to elements in the range [f_first, f_first + f_count).
                                );

    //-------------------------------------------------------------------------
    //! assigns the values of a specified number of elements from a memory
    //! contiguous source range to a compatible destination range.
    //! If some regions of the source area and the destination overlap,
    //! contiguous_move_n ensures that the original elements in the overlapping
    //! region are copied before being overwritten
    //! @author zvh2hi
    //! $Source: vfc_algorithm.hpp $
    //-------------------------------------------------------------------------
    template <class InputIterator, class OutputIterator, class CountType>
    void    contiguous_move_n    (  InputIterator    f_first,  //!< an input iterator addressing the position of the first element in the source range.
                                    OutputIterator   f_result, //!< an output iterator addressing the position of the first element in the destination range.
                                    CountType        f_count   //!< an unsigned integer type specifying the number of elements to be assigned
                                );

    //-------------------------------------------------------------------------
    //! assigns the values of a specified number of elements from a memory
    //! contiguous source range to a compatible destination range,
    //! iterating through the source sequence of elements and assigning them
    //! new positions in a forward direction.
    //! If the source and destination overlap, the behavior of contiguous_copy_n
    //! is undefined. Use contiguous_move_n to handle overlapping regions.
    //! @author zvh2hi
    //! $Source: vfc_algorithm.hpp $
    //-------------------------------------------------------------------------
    template <class InputIterator, class OutputIterator, class CountType>
    void    contiguous_copy_n    (   InputIterator  f_first,  //!< an input iterator addressing the position of the first element in the source range.
                                    OutputIterator  f_result, //!< an output iterator addressing the position of the first element in the destination range.
                                    CountType       f_count   //!< an unsigned integer type specifying the number of elements to be assigned
                                );

    //-------------------------------------------------------------------------
    //! assigns a new value to a specified number of elements in a contiguous
    //! range beginning with a particular element.
    //! use the optimized version contiguous_fill_zero_n() for assigning zero
    //! to a contiguous range
    //! @author zvh2hi
    //! $Source: vfc_algorithm.hpp $
    //-------------------------------------------------------------------------
    template <class OutputIterator, class CountType, class T>
    void    contiguous_fill_n   (   OutputIterator  f_first,  //!< an output iterator addressing the position of the first element in the range to be assigned the f_value.
                                    CountType       f_count,  //!< an unsigned integer type specifying the number of elements to be assigned the f_value
                                    const T&        f_value   //!< the f_value to be assigned to elements in the range [f_first, f_first + f_count).
                                );

    //-------------------------------------------------------------------------
    //! assigns zero to a specified number of elements in a contiguous range
    //! beginning with a particular element.
    //! @author zvh2hi
    //! $Source: vfc_algorithm.hpp $
    //-------------------------------------------------------------------------
    template <class OutputIterator, class CountType>
    void    contiguous_fill_zero_n    ( OutputIterator   f_first,  //!< an output iterator addressing the position of the first element in the range to be assigned zero.
                                        CountType        f_count   //!< an unsigned integer type specifying the number of elements to be assigned zero
                                    );

    ///////////////////////////////////////////////////////////////////////////
    // for_each_n() generate_n() transform_n()
    ///////////////////////////////////////////////////////////////////////////


    //-----------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of
    /// elements in a forward order.
    /// The algorithm for_each is very flexible, allowing the modification of each
    /// element within a range in different, user-specified ways.
    /// Templatized functions may be reused in a modified form by passing different
    /// parameters. User-defined functions may accumulate information within an
    /// internal state that the algorithm may return after processing all of the
    /// elements in the range. \n
    /// The range referenced must be valid; all pointers must be dereferenceable
    /// and, within the sequence, the last position must be reachable from the first
    /// by incrementation.
    /// @par implicit functor interface
    /// @code void FuncType::operator()(ValueType&) @endcode
    /// @param f_ioIt       input / output iterator addressing the position of the first
    ///                     element to be operated on
    /// @param f_func       user-defined function object that is applied to each of
    ///                     elements in the range.
    /// @param f_countValue unsigned integer type specifying the number of elements.
    /// @return the function object.
    /// @par Example usage:
    /// @code
    /// int32_t array[6];
    /// int32_t sum = for_each_n(array,TSumFunc<int>(),6);
    /// @endcode
    /// @author zvh2hi
    /// $Source: vfc_algorithm.hpp $
    //-----------------------------------------------------------------------------

    template <class IOItType, class FuncType, class CountType>
    FuncType    for_each_n      (   IOItType  f_ioIt,
                                    FuncType  f_func,
                                    CountType f_countValue
                                );


    //-----------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of pairs
    /// from two ranges in a forward order.
    /// @sa for_each_n
    /// @par implicit functor interface
    /// @code void FuncType::operator()(Value1Type&, Value2Type&) @endcode
    /// @param f_ioIt1      input / output iterator addressing the position of the first
    ///                     element to be operated on
    /// @param f_ioIt2      input / output iterator addressing the position of the first
    ///                     element to be operated on
    /// @param f_func       user-defined function object that is applied to each pair of
    ///                     elements in the range.
    /// @param f_countValue unsigned integer type specifying the number of elements.
    /// @author zvh2hi
    /// $Source: vfc_algorithm.hpp $
    //-----------------------------------------------------------------------------
    template <class IOIt1Type, class IOIt2Type, class FuncType, class CountType>
    FuncType    for_each_n  (   IOIt1Type f_ioIt1,
                                IOIt2Type f_ioIt2,
                                FuncType  f_func,
                                CountType f_countValue
                            );

    //-----------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of triples
    /// from three ranges in a forward order.
    /// @sa for_each_n
    /// @par implicit functor interface
    /// @code
    /// void FuncType::operator()(Value1Type&, Value2Type&, Value3Type&)
    /// @endcode
    /// @param f_ioIt1      input / output iterator addressing the position of the first
    ///                     element to be operated on
    /// @param f_ioIt2      input / output iterator addressing the position of the first
    ///                     element to be operated on
    /// @param f_ioIt3      input / output iterator addressing the position of the first
    ///                     element to be operated on
    /// @param f_func       user-defined function object that is applied to each of
    ///                     elements in the range.
    /// @param f_countValue unsigned integer type specifying the number of elements.
    /// @author zvh2hi
    /// $Source: vfc_algorithm.hpp $
    //-----------------------------------------------------------------------------
    template <class IOIt1Type, class IOIt2Type, class IOIt3Type, class FuncType, class CountType>
    FuncType    for_each_n  (   IOIt1Type f_ioIt1,
                                IOIt2Type f_ioIt2,
                                IOIt3Type f_ioIt3,
                                FuncType  f_func,
                                CountType f_countValue
                            );

    //-----------------------------------------------------------------------------
    /// Assigns the values generated by a function object to a specified
    /// number of element in a range.
    /// The function object is invoked for each element in the range and does not
    /// need to return the same value each time it is called. It may, for example,
    /// read from a file or refer to and modify a local state. The generator's
    /// result type must be convertible to the value type of the forward iterators
    /// for the range. The range referenced must be valid; all pointers must be
    /// dereferenceable and, within the sequence, the last position must be
    /// reachable from the first by incrementation.
    /// @par implicit functor interface
    /// @code ValueType FuncType::operator()() @endcode
    /// @param f_fwIt       output iterator addressing the position of the first
    ///                     element to be operated on
    /// @param f_func       user-defined function object that is applied to each of
    ///                     elements in the range.
    /// @param f_countValue unsigned integer type specifying the number of elements.
    /// @return the position one past the last assigned value.
    /// @author zvh2hi
    /// $Source: vfc_algorithm.hpp $
    //-----------------------------------------------------------------------------
    template <class FwItType, class FuncType, class CountType>
    void        generate_n  (   FwItType  f_fwIt,
                                FuncType  f_func,
                                CountType f_countValue
                            );


    //-----------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of element
    /// in a range and copies the return values of the function object
    /// into a destination range.
    /// The ranges referenced must be valid; all pointers must be dereferenceable
    /// and within each sequence the last position must be reachable from the first
    /// by incrementation.\n
    /// The destination range must be large enough to contain the transformed source
    /// range.\n
    /// If f_destIt is set equal to f_inIt, then the source and destination ranges
    /// will be the same and the sequence will be modified in place.
    /// @par implicit functor interface
    /// @code DestValueType FuncType::operator()(const InValueType&) @endcode
    /// @param f_inIt       input iterator addressing the position of the first
    ///                     element to be operated on
    /// @param f_destIt     output iterator addressing the position of the first
    ///                     element to be operated on
    /// @param f_func       user-defined function object that is used to each of
    ///                     elements in the input and destination.
    /// @param f_countValue unsigned integer type specifying the number of elements.
    /// @return
    /// An output iterator addressing the position one past the final element in the destination
    /// range that is receiving the output elements transformed by the function object.
    /// @author zvh2hi
    /// $Source: vfc_algorithm.hpp $
    //-----------------------------------------------------------------------------
    template <class InItType, class OutItType, class FuncType, class CountType>
    OutItType   transform_n     (   InItType  f_inIt,
                                    OutItType f_destIt,
                                    FuncType  f_func,
                                    CountType f_countValue
                                );

    //-----------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of pairs
    /// from two source ranges and copies the return values of the function
    /// object into a destination range.
    /// @sa transform_n
    /// @par implicit functor interface
    /// @code
    /// DestValueType FuncType::operator()(const InValue1Type&, const InValue2Type&)
    /// @endcode
    /// @par Example usage:
    /// @code
    /// // seq1 += seq2
    /// int seq1[] = { 1, 2, 3, 4, 5 };
    /// int seq2[] = { 5, 4, 3, 2, 1 };
    /// transform_n ( seq1, seq2, seq1, std::plus<int>(), 5);
    /// @endcode
    /// @param f_inIt1      input iterator addressing the position of the first
    ///                     element to be operated on
    /// @param f_inIt2      n input iterator addressing the position of the first
    ///                     element to be operated on
    /// @param f_destIt     output iterator addressing the position of the first
    ///                     element to be operated on
    /// @param f_func       user-defined function object that is used to each of
    ///                     elements in the inputs and destination.
    /// @param f_countValue unsigned integer type specifying the number of elements.
    /// @author zvh2hi
    /// $Source: vfc_algorithm.hpp $
    //-----------------------------------------------------------------------------
    template <class InIt1Type, class InIt2Type, class OutItType, class FuncType, class CountType>
    OutItType   transform_n     (   InIt1Type f_inIt1,
                                    InIt2Type f_inIt2,
                                    OutItType f_destIt,
                                    FuncType  f_func,
                                    CountType f_countValue
                                );

    //=============================================================================
    // modified STL for 1D random-access iterators (op[](n) square brackets)
    //=============================================================================

    //-----------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of
    /// elements from a range in a forward order.
    /// The algorithm for_each is very flexible, allowing the modification of each
    /// element within a range in different, user-specified ways.
    /// Templatized functions may be reused in a modified form by passing different
    /// parameters. User-defined functions may accumulate information within an
    /// internal state that the algorithm may return after processing all of the
    /// elements in the range.\n
    /// The elements in the range @b must be accessible with the square bracket
    /// operator[](int) (random-access iterator).
    /// @par implicit functor interface
    /// @code void FuncType::operator()(ValueType&) @endcode
    /// @param f_ioArg      input / output argument
    /// @param f_func       user-defined function object that is applied to each of
    ///                     elements in the specified argument.
    /// @param f_countValue unsigned integer type specifying the number of elements.
    /// @return the function object.
    /// @author zvh2hi
    /// $Source: vfc_algorithm.hpp $
    //-----------------------------------------------------------------------------
    template <class IOArgType, class FuncType, class CountType>
    FuncType    for_each_square_n   (   IOArgType& f_ioArg,
                                        FuncType   f_func,
                                        CountType  f_countValue
                                    );


    //-----------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of pairs
    /// from two ranges in a forward order.
    /// The elements in the range @b must be accessible with the square bracket
    /// operator[](int) (random-access iterator).
    /// @sa for_each_square_n
    /// @par implicit functor interface
    /// @code void FuncType::operator()(Value1Type&, Value2Type&) @endcode
    /// @param f_ioArg1     input / output argument
    /// @param f_ioArg2     input / output argument
    /// @param f_func       user-defined function object that is applied to each pair of
    ///                     elements in the specified argument.
    /// @param f_countValue unsigned integer type specifying the number of elements.
    /// @return the function object.
    /// @author zvh2hi
    /// $Source: vfc_algorithm.hpp $
    //-----------------------------------------------------------------------------
    template <class IOArg1Type, class IOArg2Type, class FuncType, class CountType>
    FuncType    for_each_square_n   (   IOArg1Type& f_ioArg1,
                                        IOArg2Type& f_ioArg2,
                                        FuncType    f_func,
                                        CountType   f_countValue
                                    );

    //-----------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of triples
    /// from three ranges in a forward order.
    /// The elements in the range @b must be accessible with the square bracket
    /// operator[](int) (random-access iterator).
    /// @sa for_each_square_n
    /// @par implicit functor interface
    /// @code void FuncType::operator()(Value1Type&, Value2Type&, Value3Type&) @endcode
    /// @return the function object.
    /// @param f_ioArg1     input / output argument
    /// @param f_ioArg2     input / output argument
    /// @param f_ioArg3     input / output argument
    /// @param f_func       user-defined function object that is applied to each of
    ///                     elements in the range.
    /// @param f_countValue unsigned integer type specifying the number of elements.
    /// @author zvh2hi
    /// $Source: vfc_algorithm.hpp $
    //-----------------------------------------------------------------------------
    template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType, class CountType>
    FuncType    for_each_square_n   (   IOArg1Type& f_ioArg1,
                                        IOArg2Type& f_ioArg2,
                                        IOArg3Type& f_ioArg3,
                                        FuncType    f_func,
                                        CountType   f_countValue
                                    );

    //-----------------------------------------------------------------------------
    /// Assigns the values generated by a function object to a specified
    /// number of element in a range.
    /// @par Description:
    /// The function object is invoked for each element in the range and does not
    /// need to return the same value each time it is called. It may, for example,
    /// read from a file or refer to and modify a local state.
    /// The generator's result type must be convertible to the value type of the
    /// forward iterators for the range.\n
    /// The elements in the range @b must be accessible with the square bracket
    /// operator[](int) (random-access iterator).
    /// @par implicit functor interface
    /// @code ValueType FuncType::operator()() @endcode
    /// @param f_outArg     output argument
    /// @param f_func       user-defined function object that is applied to each of
    ///                     elements in the range.
    /// @param f_countValue unsigned integer type specifying the number of elements.
    /// @author zvh2hi
    /// $Source: vfc_algorithm.hpp $
    //-----------------------------------------------------------------------------
    template <class OutArgType, class FuncType, class CountType>
    void        generate_square_n   (   OutArgType& f_outArg,
                                        FuncType    f_func,
                                        CountType   f_countValue
                                    );

    //-----------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of element
    /// in a range and copies the return values of the function object
    /// into a destination range.
    /// @par Description:
    /// The elements in the range @b must be accessible with the square bracket
    /// operator[](int) (random-access iterator).\n
    /// The destination range must be large enough to contain the transformed
    /// source range.\n
    /// If f_destIt is set equal to f_inIt, then the source and destination
    /// ranges will be the same and the sequence will be modified in place.
    /// @par implicit functor interface
    /// @code DestValueType FuncType::operator()(const InValueType&) @endcode
    /// @param f_inArg      input argument
    /// @param f_destArg    output argument
    /// @param f_func       user-defined function object that is used to each of
    ///                     elements in the input and destination.
    /// @param f_countValue unsigned integer type specifying the number of elements.
    /// @author zvh2hi
    /// $Source: vfc_algorithm.hpp $
    //-----------------------------------------------------------------------------
    template <class InArgType, class OutArgType, class FuncType, class CountType>
    void        transform_square_n  (   const InArgType& f_inArg,
                                        OutArgType& f_destArg,
                                        FuncType    f_func,
                                        CountType   f_countValue
                                    );

    //-----------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of pairs
    /// from two source ranges and copies the return values of the function
    /// object into a destination range.
    /// @par Description:
    /// The elements in the range @b must be accessible with the square bracket
    /// operator[](int) (random-access iterator).\n
    /// @par implicit functor interface
    /// @code DestValueType FuncType::operator()(const InValue1Type&, const InValue2Type&) @endcode
    /// @param f_inIt1      input argument
    /// @param f_inIt2      input argument
    /// @param f_destIt     output argument
    /// @param f_func       user-defined function object that is used to each of
    ///                     elements in the inputs and destination.
    /// @param f_countValue unsigned integer type specifying the number of elements.
    /// @author zvh2hi
    /// $Source: vfc_algorithm.hpp $
    //-----------------------------------------------------------------------------
    template <  class InArg1Type, class InArg2Type, class OutArgType,
                class FuncType, class CountType>
    void        transform_square_n  (   const InArg1Type&   f_inArg1,
                                        const InArg2Type&   f_inArg2,
                                        OutArgType&         f_destArg,
                                        FuncType            f_func,
                                        CountType           f_countValue
                                    );

    //=============================================================================
    // sorting
    //=============================================================================

    //-------------------------------------------------------------------------
    //! Assigns the contents of a to b and the contents of b to a.
    //! This implementation of swap makes one call to a copy constructor and two
    //! calls to an assignment operator; roughly, then, it should be expected to
    //! take about the same amount of time as three assignments.
    //! In many cases, however, it is possible to write a specialized version of
    //! swap that is far more efficient.
    //! @note   The implementation of this utility contains various workarounds:
    //! - swap_impl is put outside the vfc namespace, to avoid infinite
    //! recursion (causing stack overflow) when swapping objects of a primitive
    //! type.
    //! - swap_impl has a using-directive, rather than a using-declaration,
    //! because some compilers (including MSVC 7.1, Borland 5.9.3, and
    //! Intel 8.1) don't do argument-dependent lookup when it has a
    //! using-declaration instead.
    //! - vfc::swap has two template arguments, instead of one, to
    //! avoid ambiguity when swapping objects of a vfc type that does
    //! not have its own vfc::swap overload.
    //! @author zvh2hi
    //! $Source: vfc_algorithm.hpp $
    //-------------------------------------------------------------------------
    template <class Arg1Type, class Arg2Type>  inline
    void swap ( Arg1Type& f_left, Arg2Type& f_right );

    //-------------------------------------------------------------------------
    //! sorts the specified items into ascending order.
    //! @author zvh2hi
    //! $Source: vfc_algorithm.hpp $
    //-------------------------------------------------------------------------
    template <class ArgType>
    void    sort ( ArgType& f_a, ArgType& f_b );

    //-------------------------------------------------------------------------
    //! sorts the specified items into ascending order.
    //! @author zvh2hi
    //! $Source: vfc_algorithm.hpp $
    //-------------------------------------------------------------------------
    template <class ArgType>
    void    sort ( ArgType& f_a, ArgType& f_b, ArgType& f_c );

    //-------------------------------------------------------------------------
    //! sorts the specified items into ascending order.
    //! @author zvh2hi
    /// $Source: vfc_algorithm.hpp $
    //-------------------------------------------------------------------------
    template <class ArgType>
    void    sort ( ArgType& f_a, ArgType& f_b, ArgType& f_c, ArgType& f_d );


    //=========================================================================
    //  DOYGEN ADDTOGROUP vfc_group_core_algorithms_stl END
    //-------------------------------------------------------------------------
    /// @}
    //-------------------------------------------------------------------------

}    // namespace vfc closed

#include "vfc/core/vfc_algorithm.inl"

#endif //VFC_ALGORITHM_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_algorithm.hpp  $
//  Revision 1.13 2009/02/03 13:22:54MEZ gaj2kor 
//  -Removal of QAC++ warning.
//  (Mantis : 0002446)
//  Revision 1.12 2008/08/29 16:28:03IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - moved swap and sort implementation to vfc_algorithm.{hpp,inl} (mantis 2323)
//  - rewrote vfc::swap (mantis 2323)
//  - merged doxygen docs (mantis 2326)
//  Revision 1.11 2008/08/26 15:39:29CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - fixed implementation bug in contiguous_{copy,move}_n functions (mantis 2277)
//  Revision 1.10 2007/08/02 15:52:33CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added conditional doxygen documentation generation of vfc::intern (mantis1758)
//  Revision 1.9 2007/07/23 09:25:44CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - doxygen grouping (mantis1744)
//  Revision 1.8 2007/02/23 13:43:05CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - added the functions from the algorithm_looped proposal (mantis 1225)
//  - replaced a type by a template parameter (mantis 1252)
//  - changed duffs device to while loops (mantis 1346)
//  Revision 1.7 2006/11/16 14:41:08CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.6 2006/01/27 13:07:50CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added documentation
//  -changed includes
//  Revision 1.5 2005/11/18 15:39:54CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added for_each() for processing two sequences in parallel
//  Revision 1.4 2005/11/04 16:59:25CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -moved intern declarations into *.inl file
//  Revision 1.2 2005/10/06 16:55:40CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/vfc.pj
//  Revision 1.1 2005/04/07 14:56:06CEST zvh2hi
//  Initial revision
//  Member added to project /import/mks/data/projects/cv/vfc/include/vfc/vfc.pj
//=============================================================================
