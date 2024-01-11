//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2006 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorised copy, use or 
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
///     $Source: vfc_algorithm_fixed.inl $
///     $Revision: 1.6 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/08/02 15:52:32MESZ $
///     $Locker:  $
///     $Name: 0032 RC1 Hello KW07  $
///     $State: In_Development $
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

#include "vfc/core/vfc_static_assert.hpp"

namespace vfc
{   // namespace vfc opened

    //-------------------------------------------------------------------------
    // conditional doxygen documentation
    //! @cond VFC_DOXY_INTERN
    //-------------------------------------------------------------------------

    namespace intern
    {   // namespace intern opened
        
        //=============================================================================
        // Template Metaprograms for fixed-size loop-unrolling of STL like algorithms
        //=============================================================================

        //=============================================================================
        //    vfc::intern::TUnroll1DForwardIteratorForward<>
        //-----------------------------------------------------------------------------
        // algorithms for one dimensional sequences with forward iterators (++it)
        // e.g. int* sequence;
        //      std::list<int>::iterator
        //=============================================================================

        template <int32_t LoopCounterValue>
        struct TUnroll1DForwardIteratorForward
        {
             template <class IOItType, class FuncType>
            static inline
            void    for_each(IOItType& f_ioIt, FuncType& f_func)
            {
                f_func( *f_ioIt);

                TUnroll1DForwardIteratorForward<LoopCounterValue-1>::for_each(  ++f_ioIt, 
                                                                                f_func);
            }

            template <class IOIt1Type, class IOIt2Type, class FuncType>
            static inline
            void    for_each(IOIt1Type& f_ioIt1, IOIt2Type& f_ioIt2, FuncType& f_func)
            {
                f_func( *f_ioIt1, *f_ioIt2);

                TUnroll1DForwardIteratorForward<LoopCounterValue-1>::for_each(  ++f_ioIt1, 
                                                                                ++f_ioIt2, 
                                                                                f_func);
            }

            template <class IOIt1Type, class IOIt2Type, class IOIt3Type, class FuncType>
            static inline
            void    for_each(IOIt1Type& f_ioIt1, IOIt2Type& f_ioIt2, IOIt3Type& f_ioIt3, FuncType& f_func)
            {
                f_func( *f_ioIt1, *f_ioIt2, *f_ioIt3);

                TUnroll1DForwardIteratorForward<LoopCounterValue-1>::for_each(  ++f_ioIt1, 
                                                                                ++f_ioIt2, 
                                                                                ++f_ioIt3, 
                                                                                f_func);
            }

            template <class OutItType, class FuncType> 
            static inline
            void    generate(OutItType& f_outIt, FuncType& f_func)
            {
                *f_outIt = f_func();

                TUnroll1DForwardIteratorForward<LoopCounterValue-1>::generate(  ++f_outIt,
                                                                                f_func);
            }

            template <class InItType, class OutItType, class FuncType> 
            static inline
            void    transform(  InItType&  f_inIt, 
                                OutItType& f_destIt, 
                                FuncType&  f_func)
            {
                *f_destIt = f_func(*f_inIt);

                TUnroll1DForwardIteratorForward<LoopCounterValue-1>::transform( ++f_inIt, 
                                                                                ++f_destIt, 
                                                                                f_func);
            }

            template <class InIt1Type, class InIt2Type, class OutItType, class FuncType> 
            static inline
            void    transform(  InIt1Type&  f_inIt1, 
                                InIt2Type&  f_inIt2, 
                                OutItType&  f_destIt, 
                                FuncType&   f_func)
            {
                *f_destIt = f_func( *f_inIt1, *f_inIt2);

                TUnroll1DForwardIteratorForward<LoopCounterValue-1>::transform( ++f_inIt1, 
                                                                                ++f_inIt2, 
                                                                                ++f_destIt, 
                                                                                f_func);
            }
        };

        //=============================================================================
        //    vfc::intern::TUnroll1DForwardIteratorForward<0>
        //-----------------------------------------------------------------------------
        // specialization for stopping recursion
        //=============================================================================

        template <>
        struct TUnroll1DForwardIteratorForward<0>
        {
            template <class IOItType, class FuncType>
            static inline
            void    for_each(IOItType&, FuncType&)
            {
                    // stop recursion!
            }

            template <class IOIt1Type, class IOIt2Type, class FuncType>
            static inline
            void    for_each(IOIt1Type&, IOIt2Type&, FuncType&)
            {
                    // stop recursion!
            }

            template <class IOIt1Type, class IOIt2Type, class IOIt3Type, class FuncType>
            static inline
            void    for_each(IOIt1Type&, IOIt2Type&, IOIt3Type&, FuncType&)
            {
                    // stop recursion!
            }

            template <class OutItType, class FuncType> 
            static inline
            void    generate(OutItType&, FuncType&)
            {
                    // stop recursion!
            }

            template <class InItType, class OutItType, class FuncType> 
            static inline
            void    transform(  InItType&, 
                                OutItType&, 
                                FuncType& )
            {
                   // stop recursion!
            }

            template <class InIt1Type, class InIt2Type, class OutItType, class FuncType> 
            static inline
            void    transform(  InIt1Type&, 
                                InIt2Type&, 
                                OutItType&, 
                                FuncType&)
            {
                  // stop recursion!
            }
        };

        //=============================================================================
        //    vfc::intern::TUnroll1DRandomIteratorForward<>
        //-----------------------------------------------------------------------------
        // algorithms for one dimensional arguments with random access operator ( arg[n] )
        // e.g. int* vec1;
        //      int  vec2[5];
        //      std::vector<int> vec3(7);
        //=============================================================================

        template <int32_t LoopCounterValue, int32_t OffsetValue=0>
        struct TUnroll1DRandomIteratorForward
        {
            template <class IOArgType, class FuncType>
            static inline
            void    for_each(IOArgType& f_ioArg, FuncType& f_func)
            {
                f_func( f_ioArg[OffsetValue]);

                TUnroll1DRandomIteratorForward<LoopCounterValue-1,OffsetValue+1>::for_each( f_ioArg, 
                                                                                            f_func);
            }

            template <class IOArg1Type, class IOArg2Type, class FuncType>
            static inline
            void    for_each(IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2, FuncType& f_func)
            {
                f_func( f_ioArg1[OffsetValue], f_ioArg2[OffsetValue]);

                TUnroll1DRandomIteratorForward<LoopCounterValue-1,OffsetValue+1>::for_each( f_ioArg1, 
                                                                                            f_ioArg2, 
                                                                                            f_func);
            }

            template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType>
            static inline
            void    for_each(IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2, IOArg3Type& f_ioArg3, FuncType& f_func)
            {
                f_func( f_ioArg1[OffsetValue], f_ioArg2[OffsetValue], f_ioArg3[OffsetValue]);

                TUnroll1DRandomIteratorForward<LoopCounterValue-1,OffsetValue+1>::for_each( f_ioArg1, 
                                                                                            f_ioArg2, 
                                                                                            f_ioArg3, 
                                                                                            f_func);
            }

            template <class OutArgType, class FuncType> 
            static inline
            void    generate(OutArgType& f_outArg, FuncType& f_func)
            {
                f_outArg[OffsetValue] = f_func();

                TUnroll1DRandomIteratorForward<LoopCounterValue-1,OffsetValue+1>::generate( f_outArg,
                                                                                            f_func);
            }

            template <class InArgType, class OutArgType, class FuncType> 
            static inline
            void    transform(  const InArgType&  f_inArg, 
                                OutArgType& f_destArg, 
                                FuncType&  f_func)
            {
                f_destArg[OffsetValue] = f_func(f_inArg[OffsetValue]);

                TUnroll1DRandomIteratorForward<LoopCounterValue-1,OffsetValue+1>::transform(f_inArg, 
                                                                                            f_destArg, 
                                                                                            f_func);
            }

            template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType> 
            static inline
            void    transform(  const InArg1Type&  f_inArg1, 
                                const InArg2Type&  f_inArg2, 
                                OutArgType&  f_destArg, 
                                FuncType&   f_func)
            {
                f_destArg[OffsetValue] = f_func( f_inArg1[OffsetValue], f_inArg2[OffsetValue]);

                TUnroll1DRandomIteratorForward<LoopCounterValue-1,OffsetValue+1>::transform(f_inArg1, 
                                                                                            f_inArg2, 
                                                                                            f_destArg, 
                                                                                            f_func);
            }
        };

        //=============================================================================
        //    vfc::intern::TUnroll1DRandomIteratorForward<0>
        //-----------------------------------------------------------------------------
        // specialization for stopping recursion
        //=============================================================================

        template <int32_t OffsetValue>
        struct TUnroll1DRandomIteratorForward<0,OffsetValue>
        {
            template <class IOArgType, class FuncType>
            static inline
            void    for_each(IOArgType&, FuncType&)
            {
                // stop recursion!
            }

            template <class IOArg1Type, class IOArg2Type, class FuncType>
            static inline
            void    for_each(IOArg1Type&, IOArg2Type&, FuncType&)
            {
                    // stop recursion!  
            }

            template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType>
            static inline
            void    for_each(IOArg1Type&, IOArg2Type&, IOArg3Type&, FuncType&)
            {
                    // stop recursion!
            }

            template <class OutArgType, class FuncType> 
            static inline
            void    generate(OutArgType&, FuncType&)
            {
                   // stop recursion!
            }

            template <class InArgType, class OutArgType, class FuncType> 
            static inline
            void    transform(  const InArgType&, 
                                OutArgType&, 
                                FuncType& )
            {
                    // stop recursion!
            }

            template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType> 
            static inline
            void    transform(  const InArg1Type&, 
                                const InArg2Type&, 
                                OutArgType&, 
                                FuncType&  )
            {
                    // stop recursion!
            }
        };
    }   // namespace intern closed

    //-------------------------------------------------------------------------
    //! @endcond 
    // of VFC_DOXY_INTERN
    //-------------------------------------------------------------------------

}   // namespace vfc closed

//=============================================================================
// original STL algorithms with metaprogramming loop-unrolling
//=============================================================================

//=============================================================================
//    vfc::for_each_fixed_n()
//-----------------------------------------------------------------------------
/// @par Description:
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
/// @return the function object.   
/// @par Example usage:   
/// @code   
/// int32_t array[6];   
/// int32_t sum = for_each_fixed_n<6>(array,TSumFunc<int>());   
/// @endcode
/// @author zvh2hi
/// @par Requirements: 
/// - vfc_algorithm_fixed.hpp  
//=============================================================================

template <  vfc::int32_t CountValue, 
            class IOItType, 
            class FuncType>
inline
FuncType    vfc::for_each_fixed_n (IOItType f_inIt, FuncType f_func)
{
    VFC_STATIC_ASSERT(CountValue>=0);

    vfc::intern::TUnroll1DForwardIteratorForward<CountValue>::for_each(f_inIt, f_func);
    return f_func;
}

//=============================================================================
//    vfc::for_each_fixed_n()
//-----------------------------------------------------------------------------
/// @sa for_each_fixed_n
/// @par implicit functor interface
/// @code void FuncType::operator()(Value1Type&, Value2Type&) @endcode
/// @author zvh2hi
/// @par Requirements: 
/// - vfc_algorithm_fixed.hpp  
//=============================================================================

template <  vfc::int32_t CountValue, 
            class IOIt1Type, class IOIt2Type, 
            class FuncType>
inline
FuncType    vfc::for_each_fixed_n (IOIt1Type f_ioIt1, IOIt2Type f_ioIt2, FuncType f_func)
{
    VFC_STATIC_ASSERT(CountValue>=0);

    vfc::intern::TUnroll1DForwardIteratorForward<CountValue>::for_each( f_ioIt1, 
                                                                        f_ioIt2, 
                                                                        f_func);
    return f_func;
}

//=============================================================================
//    vfc::for_each_fixed_n()
//-----------------------------------------------------------------------------
/// @sa for_each_fixed_n
/// @par implicit functor interface
/// @code 
/// void FuncType::operator()(Value1Type&, Value2Type&, Value3Type&) 
/// @endcode
/// @author zvh2hi
/// @par Requirements: 
/// - vfc_algorithm_fixed.hpp  
//=============================================================================

template <  vfc::int32_t CountValue, 
            class IOIt1Type, class IOIt2Type, class IOIt3Type, 
            class FuncType>
inline
FuncType    vfc::for_each_fixed_n ( IOIt1Type f_ioIt1, IOIt2Type f_ioIt2, IOIt3Type f_ioIt3, 
                                    FuncType f_func)
{
    VFC_STATIC_ASSERT(CountValue>=0);

    vfc::intern::TUnroll1DForwardIteratorForward<CountValue>::for_each( f_ioIt1, 
                                                                        f_ioIt2, 
                                                                        f_ioIt3, 
                                                                        f_func);
    return f_func;
}

//=============================================================================
//    vfc::generate_fixed_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The function object is invoked for each element in the range and does not 
/// need to return the same value each time it is called. It may, for example, 
/// read from a file or refer to and modify a local state. The generator's 
/// result type must be convertible to the value type of the forward iterators 
/// for the range. The range referenced must be valid; all pointers must be 
/// dereferenceable and, within the sequence, the last position must be 
/// reachable from the first by incrementation.
/// @par implicit functor interface
/// @code ValueType FuncType::operator()() @endcode
/// @return the position one past the last assigned value.
/// @author zvh2hi
/// @par Requirements: 
/// - vfc_algorithm_fixed.hpp  
//=============================================================================

template <  vfc::int32_t CountValue, 
            class FwItType, 
            class FuncType>
inline
void        vfc::generate_fixed_n (FwItType f_fwIt, FuncType f_func)
{
    VFC_STATIC_ASSERT(CountValue>=0);

    vfc::intern::TUnroll1DForwardIteratorForward<CountValue>::generate( f_fwIt, 
                                                                        f_func);
}

//=============================================================================
//    vfc::transform_fixed_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The ranges referenced must be valid; all pointers must be dereferenceable 
/// and within each sequence the last position must be reachable from the first 
/// by incrementation.\n 
/// The destination range must be large enough to contain the transformed source 
/// range.\n
/// If f_destIt is set equal to f_inIt, then the source and destination ranges 
/// will be the same and the sequence will be modified in place. 
/// @par implicit functor interface
/// @code DestValueType FuncType::operator()(const InValueType&) @endcode
/// @return 
/// An output iterator addressing the position one past the final element in the destination 
/// range that is receiving the output elements transformed by the function object.
/// @author zvh2hi
/// @par Requirements: 
/// - vfc_algorithm_fixed.hpp  
//=============================================================================

template <  vfc::int32_t CountValue, 
            class InItType, class OutItType, 
            class FuncType>
inline
OutItType   vfc::transform_fixed_n (InItType f_inIt, OutItType f_destIt, 
                                    FuncType f_func)
{
    VFC_STATIC_ASSERT(CountValue>=0);

    vfc::intern::TUnroll1DForwardIteratorForward<CountValue>::transform(f_inIt, 
                                                                        f_destIt, 
                                                                        f_func);
    return f_destIt;
}

//=============================================================================
//    vfc::transform_fixed_n()
//-----------------------------------------------------------------------------
/// @sa transform_fixed_n
/// @par implicit functor interface
/// @code 
/// DestValueType FuncType::operator()(const InValue1Type&, const InValue2Type&) 
/// @endcode
/// @par Example usage:
/// @code
/// // seq1 += seq2
/// int seq1[] = { 1, 2, 3, 4, 5 };
/// int seq2[] = { 5, 4, 3, 2, 1 };
/// transform_fixed_n<5> ( seq1, seq2, seq1, std::plus<int>());
/// @endcode
/// @author zvh2hi
/// @par Requirements: 
/// - vfc_algorithm_fixed.hpp  
//=============================================================================

template <  vfc::int32_t CountValue, 
            class InIt1Type, class InIt2Type, class OutItType, 
            class FuncType>
inline
OutItType   vfc::transform_fixed_n (InIt1Type f_inIt1, InIt2Type f_inIt2, OutItType f_destIt, 
                                    FuncType f_func)
{
    VFC_STATIC_ASSERT(CountValue>=0);

    vfc::intern::TUnroll1DForwardIteratorForward<CountValue>::transform(f_inIt1, 
                                                                        f_inIt2, 
                                                                        f_destIt, 
                                                                        f_func);
    return f_destIt;
}

//=============================================================================
// modified STL for 1D random-access iterators (op[](n) square brackets)
//=============================================================================

//=============================================================================
//    vfc::for_each_square_fixed_n()
//-----------------------------------------------------------------------------
/// @par Description:
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
/// @return the function object.
/// @author zvh2hi
/// @par Requirements: 
/// - vfc_algorithm_fixed.hpp  
//=============================================================================

template <  vfc::int32_t CountValue, 
            class InArgType, 
            class FuncType>
inline
FuncType    vfc::for_each_square_fixed_n (InArgType& f_inArg, FuncType f_func)
{
    VFC_STATIC_ASSERT(CountValue>=0);

    vfc::intern::TUnroll1DRandomIteratorForward<CountValue>::for_each(f_inArg, f_func);
    return f_func;
}

//=============================================================================
//    vfc::for_each_square_fixed_n()
//-----------------------------------------------------------------------------
/// The elements in the range @b must be accessible with the square bracket 
/// operator[](int) (random-access iterator).
/// @sa for_each_square_fixed_n
/// @par implicit functor interface
/// @code void FuncType::operator()(Value1Type&, Value2Type&) @endcode
/// @return the function object.
/// @author zvh2hi
/// @par Requirements: 
/// - vfc_algorithm_fixed.hpp  
//=============================================================================

template <  vfc::int32_t CountValue, 
            class IOArg1Type, class IOArg2Type, 
            class FuncType>
inline
FuncType    vfc::for_each_square_fixed_n (  IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2, 
                                            FuncType f_func)
{
    VFC_STATIC_ASSERT(CountValue>=0);

    vfc::intern::TUnroll1DRandomIteratorForward<CountValue>::for_each(  f_ioArg1, 
                                                                        f_ioArg2, 
                                                                        f_func);
    return f_func;
}

//=============================================================================
//    vfc::for_each_square_fixed_n()
//-----------------------------------------------------------------------------
/// The elements in the range @b must be accessible with the square bracket 
/// operator[](int) (random-access iterator).
/// @sa for_each_square_fixed_n
/// @par implicit functor interface
/// @code void FuncType::operator()(Value1Type&, Value2Type&, Value3Type&) @endcode
/// @return the function object.
/// @author zvh2hi
/// @par Requirements: 
/// - vfc_algorithm_fixed.hpp  
//=============================================================================

template <  vfc::int32_t CountValue, 
            class IOArg1Type, class IOArg2Type, class IOArg3Type, 
            class FuncType>
inline
FuncType    vfc::for_each_square_fixed_n (  IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2, IOArg3Type& f_ioArg3, 
                                            FuncType f_func)
{
    VFC_STATIC_ASSERT(CountValue>=0);

    vfc::intern::TUnroll1DRandomIteratorForward<CountValue>::for_each(  f_ioArg1, 
                                                                        f_ioArg2, 
                                                                        f_ioArg3, 
                                                                        f_func);
    return f_func;
}

//=============================================================================
//    vfc::generate_square_fixed_n()
//-----------------------------------------------------------------------------
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
/// @author zvh2hi
/// @par Requirements: 
/// - vfc_algorithm_fixed.hpp  
//=============================================================================

template <  vfc::int32_t CountValue, 
            class OutArgType, 
            class FuncType>
inline
void        vfc::generate_square_fixed_n (OutArgType& f_outArg, FuncType f_func)
{
    VFC_STATIC_ASSERT(CountValue>=0);

    vfc::intern::TUnroll1DRandomIteratorForward<CountValue>::generate(f_outArg, f_func);
}

//=============================================================================
//    vfc::transform_square_fixed_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The elements in the range @b must be accessible with the square bracket 
/// operator[](int) (random-access iterator).\n
/// The destination range must be large enough to contain the transformed 
/// source range.\n
/// If f_destIt is set equal to f_inIt, then the source and destination 
/// ranges will be the same and the sequence will be modified in place. 
/// @par implicit functor interface
/// @code DestValueType FuncType::operator()(const InValueType&) @endcode
/// @author zvh2hi
/// @par Requirements: 
/// - vfc_algorithm_fixed.hpp  
//=============================================================================

template <  vfc::int32_t CountValue, 
            class InArgType, class OutArgType, 
            class FuncType>
inline
void        vfc::transform_square_fixed_n ( const InArgType& f_inArg, 
                                            OutArgType& f_destArg, 
                                            FuncType f_func)
{
    VFC_STATIC_ASSERT(CountValue>=0);

    vfc::intern::TUnroll1DRandomIteratorForward<CountValue>::transform( f_inArg, 
                                                                        f_destArg, 
                                                                        f_func);
}

//=============================================================================
//    vfc::transform_square_fixed_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The elements in the range @b must be accessible with the square bracket 
/// operator[](int) (random-access iterator).\n
/// @par implicit functor interface
/// @code DestValueType FuncType::operator()(const InValue1Type&, const InValue2Type&) @endcode
/// @author zvh2hi
/// @par Requirements: 
/// - vfc_algorithm_fixed.hpp  
//=============================================================================

template <  vfc::int32_t CountValue, 
            class InArg1Type, class InArg2Type, class OutArgType, 
            class FuncType>
inline
void        vfc::transform_square_fixed_n ( const InArg1Type& f_inArg1, 
                                            const InArg2Type& f_inArg2, 
                                            OutArgType& f_destArg, 
                                            FuncType f_func)
{
    VFC_STATIC_ASSERT(CountValue>=0);

    vfc::intern::TUnroll1DRandomIteratorForward<CountValue>::transform( f_inArg1,    
                                                                        f_inArg2, 
                                                                        f_destArg, 
                                                                        f_func);
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_algorithm_fixed.inl  $
//  Revision 1.6 2007/08/02 15:52:32MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - added conditional doxygen documentation generation of vfc::intern (mantis1758)
//  Revision 1.5 2007/03/05 14:36:30CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - removed unused argument (mantis1480)
//  Revision 1.4 2007/01/05 04:15:32CET dkn2kor 
//  - fixed declaration error (mantis1371)
//  Revision 1.3 2006/11/16 18:59:22IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced size_t with int32_t (mantis1305)
//  - replaced tabs with spaces (mantis1294)
//  - fixed bug in for_each unrolling (mantis1306)
//  Revision 1.2 2006/11/06 11:18:38CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added missing vfc scope (mantis1253)
//  - cosmetics
//=============================================================================
