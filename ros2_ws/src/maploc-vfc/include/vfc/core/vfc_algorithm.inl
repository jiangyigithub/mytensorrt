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
/// @par Revision History:
///     $Source: vfc_algorithm.inl $
///     $Revision: 1.21 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/11/21 14:53:43MEZ $
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

//libc
#include <cstring>    // memmove(), memset(), memcpy()
#include <iterator>    // iterator_traits<T>
#include <algorithm>   // used for std::swap()
// vfc
#include "vfc/core/vfc_type_traits.hpp" 
#include "vfc/core/vfc_metaprog.hpp" 
// THasTrivialSet<T>, THasTrivialSetZero<T>, THasTrivialCopy<T>
#include "vfc/core/vfc_assert.hpp"    
#include "vfc/core/vfc_static_assert.hpp"    
// VFC_ASSERT
#include "vfc/core/vfc_algorithm_helper.hpp"

namespace vfc
{    // namespace vfc opened
    //-------------------------------------------------------------------------
    // conditional doxygen documentation
    //! @cond VFC_DOXY_INTERN
    //-------------------------------------------------------------------------
    namespace intern
    {    
        //---------------------------------------------------------------------
        // contiguous_move
        //---------------------------------------------------------------------

        template <class InputIterator, class OutputIterator, class CountType> 
        void    contiguous_move_n    (  InputIterator f_first, OutputIterator f_result, 
                                        CountType f_count, true_t);

        template <class InputIterator, class OutputIterator, class CountType> 
        void    contiguous_move_n    (  InputIterator f_first, OutputIterator f_result, 
                                        CountType f_count, false_t);

        //---------------------------------------------------------------------
        // contiguous_copy
        //---------------------------------------------------------------------

        template <class InputIterator, class OutputIterator, class CountType> 
        void    contiguous_copy_n    (  InputIterator f_first, OutputIterator f_result, 
                                        CountType f_count, true_t);

        template <class InputIterator, class OutputIterator, class CountType> 
        void    contiguous_copy_n    (  InputIterator f_first, OutputIterator f_result, 
                                        CountType f_count, false_t);

        //---------------------------------------------------------------------
        // contiguous_fill_n
        //---------------------------------------------------------------------

        template <class OutputIterator, class CountType, class ValueType>  
        void    contiguous_fill_n    (  OutputIterator f_first, CountType f_count, 
                                        const ValueType& f_value, true_t);

        template <class OutputIterator, class CountType, class ValueType>  
        void    contiguous_fill_n    (  OutputIterator f_first, CountType f_count, 
                                        const ValueType& f_value, false_t);

        //---------------------------------------------------------------------
        // contiguous_fill_zero_n
        //---------------------------------------------------------------------

        template <class OutputIterator, class CountType>  inline
        void    contiguous_fill_zero_n    ( OutputIterator f_first, 
                                            CountType f_count, true_t );
        
        template <class OutputIterator, class CountType>  inline
        void    contiguous_fill_zero_n    ( OutputIterator f_first, 
                                            CountType f_count, false_t );

    }    // namespace intern closed
    //-------------------------------------------------------------------------
    //! @endcond    
    // of VFC_DOXY_INTERN
    //-------------------------------------------------------------------------
}    // namespace vfc closed

//---------------------------------------------------------------------
// for_each
//---------------------------------------------------------------------

template <class InputIterator1, class InputIterator2, class FunctorType>    inline
FunctorType    vfc::for_each        (InputIterator1 f_first1, InputIterator1 f_last1, 
                                     InputIterator2 f_first2, FunctorType f_func)
{
    while (f_first1 != f_last1)
    {
        f_func(*f_first1,*f_first2);
        ++f_first1, ++f_first2 ;
    }
    return f_func;
} 

//---------------------------------------------------------------------
// copy_n
//---------------------------------------------------------------------

template <class InputIterator, class OutputIterator, class CountType>    inline
void    ::vfc::copy_n    (InputIterator f_first, OutputIterator f_result, 
                          CountType f_count)
{
    VFC_REQUIRE(f_count >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_count;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;

    // process unaligned elements first
    while (i<unaligned)
    {
        *(f_result++) = *(f_first++);
        ++i;
    }

    // process aligned elements in chunks of four
    while (i<l_count)
    {
        *(f_result++) = *(f_first++);
        *(f_result++) = *(f_first++);
        *(f_result++) = *(f_first++);
        *(f_result++) = *(f_first++);
        i+=intern::CUnroll4::OFFSET;
    }
}

//---------------------------------------------------------------------
// copy_backward_n
//---------------------------------------------------------------------

template <class InputIterator, class OutputIterator, class CountType>    inline
void    ::vfc::copy_backward_n    (InputIterator f_first, OutputIterator f_result, 
                                   CountType f_count)
{
    VFC_REQUIRE(f_count >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_count;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;

    // process unaligned elements first
    while (i<unaligned)
    {
        *(--f_result) = *(--f_first);
        ++i;
    }

    // process aligned elements in chunks of four
    while (i<l_count)
    {
        *(--f_result) = *(--f_first);
        *(--f_result) = *(--f_first);
        *(--f_result) = *(--f_first);
        *(--f_result) = *(--f_first);
        i+=intern::CUnroll4::OFFSET;
    }
}

//---------------------------------------------------------------------
// fill_n
//---------------------------------------------------------------------

template <class OutputIterator, class CountType, class ValueType>    inline
void    ::vfc::fill_n            (OutputIterator f_first, CountType f_count, 
                                  const ValueType& f_value)
{
    VFC_REQUIRE(f_count >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_count;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;

    // process unaligned elements first
    while (i<unaligned)
    {
        *(f_first++) =f_value;
        ++i;
    }

    // process aligned elements in chunks of four
    while (i<l_count)
    {
        *(f_first++) = f_value;
        *(f_first++) = f_value;
        *(f_first++) = f_value;
        *(f_first++) = f_value;
        i+=intern::CUnroll4::OFFSET;
    }  
}

//---------------------------------------------------------------------
// intern::contiguous_copy_n
//---------------------------------------------------------------------

template <class InputIterator, class OutputIterator, class CountType>    inline
void    ::vfc::intern::contiguous_copy_n    (InputIterator f_first, OutputIterator f_result, 
                                             CountType f_count, true_t)
{
    typedef typename stlalias::iterator_traits<InputIterator>::value_type   input_value_type;
    typedef typename stlalias::iterator_traits<OutputIterator>::value_type  output_value_type;

    VFC_STATIC_ASSERT(    sizeof (input_value_type ) == sizeof(output_value_type) );

    std::memcpy    (   static_cast<void*>(&(*f_result)), 
                    static_cast<const void*>(&(*f_first)), 
                    f_count * sizeof(input_value_type)
                );
}

template <class InputIterator, class OutputIterator, class CountType>    inline
void    ::vfc::intern::contiguous_copy_n    (InputIterator f_first, OutputIterator f_result, 
                                             CountType f_count, false_t)
{
    ::vfc::copy_n(f_first, f_result, f_count);
}

//---------------------------------------------------------------------
// intern::contiguous_fill_n
//---------------------------------------------------------------------

template <class OutputIterator, class CountType, class ValueType>    inline
void    ::vfc::intern::contiguous_fill_n    (OutputIterator f_first, CountType f_count, 
                                             const ValueType& f_value, true_t)
{
    typedef typename stlalias::iterator_traits<OutputIterator>::value_type value_type;

    // PRQA S 2427 ++
    std::memset    (       static_cast<void*>(&(*f_first)), 
                        static_cast<int>(f_value), 
                        f_count * sizeof(value_type)
                    );
    // PRQA S 2427 --
}

template <class OutputIterator, class CountType, class ValueType>    inline 
void    ::vfc::intern::contiguous_fill_n    (OutputIterator f_first, CountType f_count, 
                                             const ValueType& f_value, false_t)
{
    ::vfc::fill_n( f_first, f_count, f_value);
}

//---------------------------------------------------------------------
// intern::contiguous_move_n
//---------------------------------------------------------------------

template <class InputIterator, class OutputIterator, class CountType>    inline
void    ::vfc::intern::contiguous_move_n    (InputIterator f_first, OutputIterator f_result, 
                                             CountType f_count, true_t)
{
    typedef typename stlalias::iterator_traits<InputIterator>::value_type   input_value_type;
    typedef typename stlalias::iterator_traits<OutputIterator>::value_type  output_value_type;

    VFC_STATIC_ASSERT(    sizeof (input_value_type ) == sizeof(output_value_type) );

    std::memmove    (   static_cast<void*>(&(*f_result)), 
                    static_cast<const void*>(&(*f_first)), 
                    f_count * sizeof(input_value_type)
                );
}

template <class InputIterator, class OutputIterator, class CountType>    inline
void    ::vfc::intern::contiguous_move_n    (InputIterator f_first, OutputIterator f_result, 
                                             CountType f_count, false_t)
{
    // sequences of same type may overlap, so check it
    //
    // (destit <= srcit)
    //     s|-------------|
    // d|-------------|
    //      
    // (destit >= srcit + f_count)
    // s|-------------|
    //                  d|-------------|
    //
    if (    (static_cast<void*>(&(*f_result)) <= static_cast<const void*>(&(*f_first))) 
        ||  (static_cast<void*>(&(*f_result)) >= static_cast<const void*>(&(*f_first) + f_count) ) 
        )   
    {
        // use forward copy
        ::vfc::copy_n( f_first, f_result, f_count);
    }
    else
    {
        // use backward copy
        ::vfc::copy_backward_n( f_first+f_count, f_result+f_count, f_count);
    }
}

//---------------------------------------------------------------------
// intern::contiguous_fill_zero_n
//---------------------------------------------------------------------

template <class OutputIterator, class CountType>  inline
void    vfc::intern::contiguous_fill_zero_n    (OutputIterator f_first, CountType f_count, true_t )
{
    typedef typename stlalias::iterator_traits<OutputIterator>::value_type value_type;

    // PRQA S 2427 ++
    std::memset    (   static_cast<void*>(&(*f_first)), 
                    static_cast<int>(0), 
                    f_count * sizeof(value_type)
                );
    // PRQA S 2427 --
}

template <class OutputIterator, class CountType>  inline
void    vfc::intern::contiguous_fill_zero_n    (OutputIterator f_first, CountType f_count, false_t )
{
    typedef typename stlalias::iterator_traits<OutputIterator>::value_type value_type;

    ::vfc::contiguous_fill_n(f_first, f_count, static_cast<value_type>(0) );

}

//---------------------------------------------------------------------
//  contiguous_move_n
//---------------------------------------------------------------------

template <class InputIterator, class OutputIterator, class CountType>    inline
void    ::vfc::contiguous_move_n    (InputIterator f_first, OutputIterator f_result, 
                                     CountType f_count)
{
    typedef typename stlalias::iterator_traits<InputIterator>::value_type   input_value_type;
    typedef typename stlalias::iterator_traits<OutputIterator>::value_type  output_value_type;

    typedef typename ::vfc::TInt2Boolean
                        <   ::vfc::THasTrivialCopy<input_value_type>::value 
                        &&  ::vfc::TIsSameType
                                <   input_value_type, 
                                    output_value_type>::value
                        >::type use_optimized_path_type;

    ::vfc::intern::contiguous_move_n    (   f_first, 
                                            f_result, 
                                            f_count, 
                                            use_optimized_path_type()
                                        );
}

//---------------------------------------------------------------------
//  contiguous_copy_n
//---------------------------------------------------------------------

template <class InputIterator, class OutputIterator, class CountType> inline
void    ::vfc::contiguous_copy_n    (InputIterator f_first, OutputIterator f_result, 
                                     CountType f_count)
{
    typedef typename stlalias::iterator_traits<InputIterator>::value_type   input_value_type;
    typedef typename stlalias::iterator_traits<OutputIterator>::value_type  output_value_type;

    typedef typename ::vfc::TInt2Boolean
                        <   ::vfc::THasTrivialCopy<input_value_type>::value 
                        &&  ::vfc::TIsSameType
                                <   input_value_type, 
                                    output_value_type>::value
                        >::type use_optimized_path_type;

    ::vfc::intern::contiguous_copy_n    (   f_first, 
                                            f_result, 
                                            f_count, 
                                            use_optimized_path_type());
}

//---------------------------------------------------------------------
//  contiguous_fill_n
//---------------------------------------------------------------------

template <class OutputIterator, class CountType, class ValueType>  inline
void    ::vfc::contiguous_fill_n    (OutputIterator f_first, CountType f_count, 
                                     const ValueType& f_value)
{
    typedef typename stlalias::iterator_traits<OutputIterator>::value_type value_type;

    typedef typename ::vfc::TInt2Boolean
                        <    THasTrivialSet<value_type>::value
                        >::type  use_optimized_path;

    ::vfc::intern::contiguous_fill_n    (   f_first, 
                                            f_count, 
                                            f_value,
                                            use_optimized_path()
                                        );
}

//---------------------------------------------------------------------
//  contiguous_fill_zero_n
//---------------------------------------------------------------------

template <class OutputIterator, class CountType>  inline
void    ::vfc::contiguous_fill_zero_n    (OutputIterator f_first, CountType f_count)
{
    typedef typename stlalias::iterator_traits<OutputIterator>::value_type value_type;

    typedef typename ::vfc::TInt2Boolean
                        <    THasTrivialSetZero<value_type>::value
                        >::type  use_optimized_path;

    ::vfc::intern::contiguous_fill_zero_n   (   f_first, 
                                                f_count, 
                                                use_optimized_path()
                                            );
}


//---------------------------------------------------------------------
//  for_each_n
//---------------------------------------------------------------------

template <class IOItType, class FuncType, class CountType>
inline
FuncType    vfc::for_each_n (IOItType f_ioIt, FuncType f_func, CountType f_countValue)
{
    VFC_REQUIRE(f_countValue >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_countValue;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;
    while (i<unaligned)
    {
        f_func( *f_ioIt);
        ++f_ioIt;
        ++i;
    }

    while (i<l_count)
    {
        f_func( *f_ioIt);
        ++f_ioIt;

        f_func( *f_ioIt);
        ++f_ioIt;

        f_func( *f_ioIt);
        ++f_ioIt;

        f_func( *f_ioIt);
        ++f_ioIt;
        i+=intern::CUnroll4::OFFSET;
    }  

    return f_func;
}

//---------------------------------------------------------------------
//  for_each_n
//---------------------------------------------------------------------

template <class IOIt1Type, class IOIt2Type, class FuncType, class CountType>
inline
FuncType    vfc::for_each_n (IOIt1Type f_ioIt1, IOIt2Type f_ioIt2, FuncType f_func, 
                             CountType f_countValue)
{
    VFC_REQUIRE(f_countValue >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_countValue;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;
    while (i<unaligned)
    {
        f_func( *f_ioIt1, *f_ioIt2);
        ++f_ioIt1; 
        ++f_ioIt2;
        ++i;
    }

    while (i<l_count)
    {
        f_func( *f_ioIt1, *f_ioIt2);
        ++f_ioIt1; 
        ++f_ioIt2;

        f_func( *f_ioIt1, *f_ioIt2);
        ++f_ioIt1; 
        ++f_ioIt2;

        f_func( *f_ioIt1, *f_ioIt2);
        ++f_ioIt1; 
        ++f_ioIt2;

        f_func( *f_ioIt1, *f_ioIt2);
        ++f_ioIt1; 
        ++f_ioIt2;
        i+=intern::CUnroll4::OFFSET;
    }  
   
    return f_func;
}

//---------------------------------------------------------------------
//  for_each_n
//---------------------------------------------------------------------

template <class IOIt1Type, class IOIt2Type, class IOIt3Type, class FuncType, class CountType>
inline
FuncType    vfc::for_each_n (IOIt1Type f_ioIt1, IOIt2Type f_ioIt2, IOIt3Type f_ioIt3, 
                             FuncType f_func, CountType f_countValue)
{
    VFC_REQUIRE(f_countValue >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_countValue;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;
    while (i<unaligned)
    {
        f_func( *f_ioIt1, *f_ioIt2, *f_ioIt3);
        ++f_ioIt1; 
        ++f_ioIt2;
        ++f_ioIt3;
        ++i;
    }

    while (i<l_count)
    {
        f_func( *f_ioIt1, *f_ioIt2, *f_ioIt3);
        ++f_ioIt1; 
        ++f_ioIt2;
        ++f_ioIt3;

        f_func( *f_ioIt1, *f_ioIt2, *f_ioIt3);
        ++f_ioIt1; 
        ++f_ioIt2;
        ++f_ioIt3;

        f_func( *f_ioIt1, *f_ioIt2, *f_ioIt3);
        ++f_ioIt1; 
        ++f_ioIt2;
        ++f_ioIt3;

        f_func( *f_ioIt1, *f_ioIt2, *f_ioIt3);
        ++f_ioIt1; 
        ++f_ioIt2;
        ++f_ioIt3;
        i+=intern::CUnroll4::OFFSET;
    }

    return f_func;
}

//---------------------------------------------------------------------
//  generate_n
//---------------------------------------------------------------------

template <class FwItType, class FuncType, class CountType>
inline
void        vfc::generate_n (FwItType f_fwIt, FuncType f_func, CountType f_countValue)
{
    VFC_REQUIRE(f_countValue >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_countValue;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;
    while (i<unaligned)
    {
        *f_fwIt = f_func();
        ++f_fwIt;
        ++i;
    }

    while (i<l_count)
    {
        *f_fwIt = f_func();
        ++f_fwIt;

        *f_fwIt = f_func();
        ++f_fwIt;

        *f_fwIt = f_func();
        ++f_fwIt;

        *f_fwIt = f_func();
        ++f_fwIt;
        i+=intern::CUnroll4::OFFSET;
    }
}

//---------------------------------------------------------------------
//  transform_n
//---------------------------------------------------------------------

template <class InItType, class OutItType, class FuncType, class CountType>
inline
OutItType   vfc::transform_n (InItType f_inIt, OutItType f_destIt, FuncType f_func, 
                              CountType f_countValue)
{
    VFC_REQUIRE(f_countValue >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_countValue;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;
    while (i<unaligned)
    {
        *f_destIt = f_func(*f_inIt);
        ++f_inIt;
        ++f_destIt;
        ++i;
    }

    while (i<l_count)
    {
        *f_destIt = f_func(*f_inIt);
        ++f_inIt;
        ++f_destIt;

        *f_destIt = f_func(*f_inIt);
        ++f_inIt;
        ++f_destIt;

        *f_destIt = f_func(*f_inIt);
        ++f_inIt;
        ++f_destIt;

        *f_destIt = f_func(*f_inIt);
        ++f_inIt;
        ++f_destIt;
        i+=intern::CUnroll4::OFFSET;
    }

    return f_destIt;
}

//---------------------------------------------------------------------
//  transform_n
//---------------------------------------------------------------------

template <class InIt1Type, class InIt2Type, class OutItType, class FuncType, class CountType>
inline
OutItType   vfc::transform_n (InIt1Type f_inIt1, InIt2Type f_inIt2, OutItType f_destIt, 
                              FuncType f_func, CountType f_countValue)
{
    VFC_REQUIRE(f_countValue >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_countValue;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;
    while (i<unaligned)
    {
        *f_destIt = f_func( *f_inIt1, *f_inIt2);
        ++f_inIt1;
        ++f_inIt2;
        ++f_destIt;
        ++i;
    }

    while (i<l_count)
    {
        *f_destIt = f_func( *f_inIt1, *f_inIt2);
        ++f_inIt1;
        ++f_inIt2;
        ++f_destIt;

        *f_destIt = f_func( *f_inIt1, *f_inIt2);
        ++f_inIt1;
        ++f_inIt2;
        ++f_destIt;

        *f_destIt = f_func( *f_inIt1, *f_inIt2);
        ++f_inIt1;
        ++f_inIt2;
        ++f_destIt;

        *f_destIt = f_func( *f_inIt1, *f_inIt2);
        ++f_inIt1;
        ++f_inIt2;
        ++f_destIt;
        i+=intern::CUnroll4::OFFSET;
    }
    
    return f_destIt;
}

//=============================================================================
// modified STL for 1D random-access iterators (op[](n) square brackets)
//=============================================================================

//---------------------------------------------------------------------
//  for_each_square_n
//---------------------------------------------------------------------

template <class IOArgType, class FuncType, class CountType>
inline
FuncType    vfc::for_each_square_n (IOArgType& f_ioArg, FuncType f_func, CountType f_countValue)
{
    VFC_REQUIRE(f_countValue >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_countValue;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;
    while (i<unaligned)
    {
        f_func( f_ioArg[i]);
        ++i;
    }

    while (i<l_count)
    {
        f_func( f_ioArg[i]);
        f_func( f_ioArg[i+1]);
        f_func( f_ioArg[i+2]);
        f_func( f_ioArg[i+3]);
        i+=intern::CUnroll4::OFFSET;
    }
    
    return f_func;
}

//---------------------------------------------------------------------
//  for_each_square_n
//---------------------------------------------------------------------

template <class IOArg1Type, class IOArg2Type, class FuncType, class CountType>
inline
FuncType    vfc::for_each_square_n (IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2, 
                                    FuncType f_func, CountType f_countValue)
{
    VFC_REQUIRE(f_countValue >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_countValue;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;
    while (i<unaligned)
    {
        f_func( f_ioArg1[i], f_ioArg2[i]);
        ++i;
    }

    while (i<l_count)
    {
        f_func( f_ioArg1[i], f_ioArg2[i]);
        f_func( f_ioArg1[i+1], f_ioArg2[i+1]);
        f_func( f_ioArg1[i+2], f_ioArg2[i+2]);
        f_func( f_ioArg1[i+3], f_ioArg2[i+3]);
        i+=intern::CUnroll4::OFFSET;
    }
    return f_func;
}

//---------------------------------------------------------------------
//  for_each_square_n
//---------------------------------------------------------------------

template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType, class CountType>
inline
FuncType    vfc::for_each_square_n (IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2, 
                                    IOArg3Type& f_ioArg3, FuncType f_func, CountType f_countValue)
{
    VFC_REQUIRE(f_countValue >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_countValue;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;
    while (i<unaligned)
    {
        f_func( f_ioArg1[i], f_ioArg2[i], f_ioArg3[i]);
        ++i;
    }

    while (i<l_count)
    {
        f_func( f_ioArg1[i], f_ioArg2[i], f_ioArg3[i]);
        f_func( f_ioArg1[i+1], f_ioArg2[i+1], f_ioArg3[i+1]);
        f_func( f_ioArg1[i+2], f_ioArg2[i+2], f_ioArg3[i+2]);
        f_func( f_ioArg1[i+3], f_ioArg2[i+3], f_ioArg3[i+3]);
        i+=intern::CUnroll4::OFFSET;
    }

    return f_func;
}

//---------------------------------------------------------------------
//  generate_square_n
//---------------------------------------------------------------------

template <class OutArgType, class FuncType, class CountType>
inline
void        vfc::generate_square_n (OutArgType& f_outArg, FuncType f_func, CountType f_countValue)
{
    VFC_REQUIRE(f_countValue >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_countValue;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;
    while (i<unaligned)
    {
        f_outArg[i] = f_func();
        ++i;
    }

    while (i<l_count)
    {
        f_outArg[i] = f_func();
        f_outArg[i+1] = f_func();
        f_outArg[i+2] = f_func();
        f_outArg[i+3] = f_func();
        i+=intern::CUnroll4::OFFSET;
    }
}

//---------------------------------------------------------------------
//  transform_square_n
//---------------------------------------------------------------------

template <class InArgType, class OutArgType, class FuncType, class CountType>
inline
void        vfc::transform_square_n (const InArgType& f_inArg, OutArgType& f_destArg, 
                                     FuncType f_func, CountType f_countValue)
{
    VFC_REQUIRE(f_countValue >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_countValue;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;
    while (i<unaligned)
    {
        f_destArg[i] = f_func(f_inArg[i]);
        ++i;
    }

    while (i<l_count)
    {
        f_destArg[i] = f_func(f_inArg[i]);
        f_destArg[i+1] = f_func(f_inArg[i+1]);
        f_destArg[i+2] = f_func(f_inArg[i+2]);
        f_destArg[i+3] = f_func(f_inArg[i+3]);
        i+=intern::CUnroll4::OFFSET;
    }
}

//---------------------------------------------------------------------
//  transform_square_n
//---------------------------------------------------------------------

template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType, class CountType>
inline
void        vfc::transform_square_n (const InArg1Type& f_inArg1, const InArg2Type& f_inArg2, 
                                     OutArgType& f_destArg, FuncType f_func, CountType f_countValue)
{
    VFC_REQUIRE(f_countValue >= 0);

    typedef typename vfc::TSigned2Unsigned<CountType>::type unsigned_counttype;

    const unsigned_counttype l_count = f_countValue;
    const unsigned_counttype unaligned = (l_count&intern::CUnroll4::BITMASK);

    unsigned_counttype i = 0;
    while (i<unaligned)
    {
        f_destArg[i] = f_func( f_inArg1[i], f_inArg2[i]);
        ++i;
    }

    while (i<l_count)
    {
        f_destArg[i] = f_func( f_inArg1[i], f_inArg2[i]);
        f_destArg[i+1] = f_func( f_inArg1[i+1], f_inArg2[i+1]);
        f_destArg[i+2] = f_func( f_inArg1[i+2], f_inArg2[i+2]);
        f_destArg[i+3] = f_func( f_inArg1[i+3], f_inArg2[i+3]);
        i+=intern::CUnroll4::OFFSET;
    }

}
    //-------------------------------------------------------------------------
    // conditional doxygen documentation
    //! @cond VFC_DOXY_INTERN
    //-------------------------------------------------------------------------
    namespace vfc_swap_impl
    {
        template<class ValueType> inline
        void swap_impl(ValueType& f_left, ValueType& f_right)
        {
            using namespace stlalias;   //use std::swap if argument dependent lookup fails
            swap( f_left, f_right);
        }

        template<class ValueType, vfc::int32_t SizeValue> inline
        void swap_impl(ValueType (& f_left)[SizeValue], ValueType (& f_right)[SizeValue])
        {
            for (vfc::int32_t i = 0; i < SizeValue; ++i)
            {
                ::vfc_swap_impl::swap_impl(f_left[i], f_right[i]);
            }
        }
    }
    //-------------------------------------------------------------------------
    //! @endcond    
    // of VFC_DOXY_INTERN
    //-------------------------------------------------------------------------

    //---------------------------------------------------------------------
    //  swap
    //---------------------------------------------------------------------

    template<class Arg1Type, class Arg2Type> inline
    void vfc::swap(Arg1Type& f_left, Arg2Type& f_right)
    {
        ::vfc_swap_impl::swap_impl(f_left, f_right);
    }

    
    //---------------------------------------------------------------------
    //  sort
    //---------------------------------------------------------------------

    template <class ArgType>  inline
    void    vfc::sort ( ArgType& f_a, ArgType& f_b )
    {
        // sorting
        //  a  b
        //  <-->

        if ( f_a > f_b )
        {
            ::vfc::swap( f_a, f_b);
        }
    }

    //---------------------------------------------------------------------
    //  sort
    //---------------------------------------------------------------------

    template <class ArgType> inline
    void    vfc::sort ( ArgType& f_a, ArgType& f_b, ArgType& f_c )
    {
        // sorting
        //  a  b  c
        //  <-->
        //  <----->
        //     <-->

        ::vfc::sort (f_a, f_b);
        ::vfc::sort (f_a, f_c);
        ::vfc::sort (f_b, f_c);
    }

    //---------------------------------------------------------------------
    //  sort
    //---------------------------------------------------------------------

    template <class ArgType> inline
    void    vfc::sort ( ArgType& f_a, ArgType& f_b, ArgType& f_c, ArgType& f_d )
    {
        // sorting
        //  a  b  c  d
        //  <-->
        //        <-->
        //  <----->
        //     <----->
        //     <-->

        ::vfc::sort ( f_a, f_b);
        ::vfc::sort ( f_c, f_d);
        ::vfc::sort ( f_a, f_c);
        ::vfc::sort ( f_b, f_d);
        ::vfc::sort ( f_b, f_c);
    }


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_algorithm.inl  $
//  Revision 1.21 2014/11/21 14:53:43MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_algorithm: #include the C++ header <cstring> instead of the C header <string.h> (mantis0004652)
//  Revision 1.20 2014/08/12 16:23:59MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_algorithm: loop unrolling factor shall be made "unsigned" after the assertion for >0 (mantis0004593)
//  Revision 1.19 2012/12/18 08:27:28MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.18 2009/02/02 14:54:58MEZ Gaurav Jain (RBEI/ESB3) (gaj2kor) 
//  -Removal of QAC++ warnings.
//  (Mantis : 0002545)
//  Revision 1.17 2008/08/29 16:28:04IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - moved swap and sort implementation to vfc_algorithm.{hpp,inl} (mantis 2323)
//  - rewrote vfc::swap (mantis 2323)
//  - merged doxygen docs (mantis 2326)
//  Revision 1.16 2008/08/28 10:45:32CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced runtime with compile time check (mantis 2277)
//  Revision 1.15 2008/08/26 15:39:30CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - fixed implementation bug in contiguous_{copy,move}_n functions (mantis 2277)
//  Revision 1.14 2008/01/29 10:32:58CET Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - fixed compiler warnings with contiguous_fill_zero_n (mantis2032)
//  Revision 1.13 2007/08/24 13:33:48CEST dkn2kor 
//  - missing parenthesis added (mantis 1714) 
//  Revision 1.12 2007/08/02 19:22:33IST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - added conditional doxygen documentation generation of vfc::intern (mantis1758)
//  Revision 1.11 2007/03/29 15:59:50CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - fixed include (mantis1534)
//  Revision 1.10 2007/03/29 15:28:09CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced std with stlalias (mantis1534)
//  Revision 1.9 2007/02/23 13:43:06CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added the functions from the algorithm_looped proposal (mantis 1225)
//  - replaced a type by a template parameter (mantis 1252)
//  - changed duffs device to while loops (mantis 1346)
//  Revision 1.8 2006/11/16 14:41:12CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.7 2006/01/26 17:29:49CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added missing typenames
//  Revision 1.6 2005/12/06 15:30:39CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -fixed missing typenames (thx to ti and jat2hi)
//  Revision 1.5 2005/11/18 15:39:55CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added for_each() for processing two sequences in parallel
//  Revision 1.4 2005/11/04 16:59:26CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -moved intern declarations into *.inl file
//  Revision 1.2 2005/10/06 16:56:01CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/vfc.pj
//  Revision 1.1 2005/04/07 14:56:06CEST zvh2hi 
//  Initial revision
//  Member added to project /import/mks/data/projects/cv/vfc/include/vfc/vfc.pj
//=============================================================================
