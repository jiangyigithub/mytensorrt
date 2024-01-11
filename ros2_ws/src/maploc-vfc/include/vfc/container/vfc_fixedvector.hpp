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
//       Projectname: vfc/container
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
//        Name: Thomas Jaeger
//  Department: AE-DA/ESA3
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_fixedvector.hpp $
///     $Revision: 1.11 $
///     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
///     $Date: 2016/11/28 09:12:16MEZ $
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

#ifndef VFC_FIXEDVECTOR_HPP_INCLUDED
#define VFC_FIXEDVECTOR_HPP_INCLUDED

// vfc/core includes
#include "vfc/core/vfc_types.hpp"  // used for size_t
#include "vfc/core/vfc_type_traits.hpp" // used for hasTrivialD/Ctor_t
#include "vfc/core/vfc_metaprog.hpp"   // used for TInt2Boolean
#include "vfc/core/vfc_static_assert.hpp"

// vfc/memory includes
#include "vfc/memory/vfc_fixedblock_allocator.hpp" // used for default AllocatorType

#include <iterator>
#include <utility>

namespace vfc 
{

//=============================================================================
// E X P O R T E D   C L A S S
//
// TFixedVector
//
//--------------------------------------------------------------------
//! A template class to replace the std::vector.
//! The template class "TFixedVector" is a wrapper for an old-fashioned
//! C-style array. It provides (limited!) functionality from the
//! std::vector interface, so it could be used as a replacement
//! for std::vector.
//! @param ValueType The type of the data the vector holds
//! @param CapacityValue The static maximum size of the vector
//! @param AllocatorType (optional) memory allocator to be used
//=============================================================================
    template<class ValueType, 
             vfc::int32_t CapacityValue, 
             class AllocatorType = vfc::TFixedBlockAllocator<ValueType, CapacityValue> >
    class TFixedVector 
    {
        VFC_STATIC_ASSERT(0 <= CapacityValue);

    public:
        enum 
        { 
            MAX_SIZE = CapacityValue //!< The maximum size of the vector (defined at compile time).
        };

        // type definitions
        typedef ValueType                                        value_type;
        typedef ValueType*                                       iterator;
        typedef const ValueType*                                 const_iterator;
        typedef stlalias::reverse_iterator<iterator>             reverse_iterator;
        typedef stlalias::reverse_iterator<const_iterator>       const_reverse_iterator;
        typedef ValueType&                                       reference;
        typedef const ValueType&                                 const_reference;
        typedef int32_t                                          size_type;
        typedef ptrdiff_t                                        difference_type;

        typedef typename TInt2Boolean<THasTrivialCTor<ValueType>::value>::type    hasTrivialCTor_t;
        typedef typename TInt2Boolean<THasTrivialDTor<ValueType>::value>::type    hasTrivialDTor_t;

        //======================================================
        // constructors, destructor
        //======================================================
        //! Default constructor, takes no arguments.
        TFixedVector();

        //! Default constructor, takes one argument: the reserved size of the vector.
        //! The maximum allowed reserved size is the vectors static capacity (template
        //! parameter CapacityValue). 
        //! Elements stored from zero to f_n-1 are default constructed. Reserved POD types
        //! are uninitialized.
        explicit TFixedVector(size_type f_n);

        //! Default constructor, takes two arguments: the size of the vector and default value.
        //! The maximum allowed reserved size is the vectors static capacity (template
        //! parameter CapacityValue).
        //! Elements stored from zero to f_n-1 are constructed with f_default.
        TFixedVector(size_type f_n, const ValueType& f_default);

        //! Copy constructor.
        TFixedVector(const TFixedVector& f_rhs);

        //! Allocator constructor
        explicit
        TFixedVector(const AllocatorType& f_alloc)
        : m_usedElements(0),
          m_basePointer_p(0),
          m_alloc(f_alloc)
        {
            m_basePointer_p = m_alloc.allocate(MAX_SIZE);
        }

        //! Allocator constructor
        TFixedVector(size_type f_count, const AllocatorType& f_alloc)
        : m_usedElements(f_count),
          m_basePointer_p(0),
          m_alloc(f_alloc)
        {
            m_basePointer_p = m_alloc.allocate(MAX_SIZE);
        }

        //! Destructor, destructs stored elements (if any) and frees memory.
        ~TFixedVector();

        //! Assignment operator.
        //! Assignes only TFixedVector types which have equal template arguments.
        const TFixedVector& operator=(const TFixedVector& f_rhs);

        //======================================================
        // methods
        //======================================================

        //! begin() returns pointer to the first stored element.
        iterator begin();

        //! begin() returns const pointer to the first stored element.
        const_iterator begin() const;

        //! end() returns pointer after the last stored element.
        iterator end();

        //! end() returns const pointer after the last stored element.
        const_iterator end() const;

        //! rbegin() returns pointer to the first stored element of the reversed vector.
        reverse_iterator rbegin();

        //! begin() returns const pointer to the first stored element of the reversed vector.
        const_reverse_iterator rbegin() const;

        //! end() returns pointer after the last stored element of the reversed vector.
        reverse_iterator rend();

        //! end() returns const pointer after the last stored element of the reversed vector.
        const_reverse_iterator rend() const;

        //! operator[] returns reference to the element at position f_idx
        //! in vector.
        reference operator[](size_type f_idx);
        
        //! operator[] returns const reference to the element at position f_idx
        //! in vector.
        const_reference operator[](size_type f_idx) const;

        //! front() returns reference to the first stored element.
        reference front();

        //! front() returns const reference to the first stored element.
        const_reference front() const;
        
        //! back() returns reference to the last stored element.
        reference back();
        
        //! back() returns const reference to the last stored element.
        const_reference back() const;

        //! size() returns the number of used elements in the vector.
        size_type size() const;

        //! empty() returns true if the vector holds no elements. Otherwise
        //! returns false.
        bool empty() const;

        //! capacity() returns the maximum number of elements (defined
        //! at compile time) the vector can store.
        size_type capacity() const;

        //! max_size() returns the maximum possible sequence length.
        size_type max_size() const;

        //! push_back() constructs a copy of f_elem after the last stored element
        //! of the vector.
        //! \pre 
        //! The user has to ensure that this function is not called on a full
        //! vector.
        void push_back(const ValueType& f_elem);

#if __cplusplus >= 201103L
        //! emplace_back() constructs a new elements after the last stored element
        //! of the vector.
        //! \pre
        //! The user has to ensure that this function is not called on a full
        //! vector.
        template<class... Args>
        reference emplace_back(Args&&... f_args);
#endif

        //! pop_back() destructs the last element of the vector.
        //! \pre 
        //! The user has to ensure that this function is not called on an empty
        //! vector.
        void pop_back();

        //! clear() destructs all elements of the vector. The vector is
        //! empty after calling clear().
        void clear();

        //! resize() changes the number of stored elements. If f_newsize<oldsize,
        //! the elements from f_newsize to oldsize-1 are destructed.
        //! if f_newsize>oldsize, the elements from oldsize to f_newsize-1 are
        //! default constructed, POD types are not initialized.
        //! \pre 
        //! The user has to ensure that the newsize does not exceed the static
        //! capacity of the vector.
        void resize(size_type f_newsize);

        //! resize() changes the number of stored elements. If f_newsize<oldsize,
        //! the elements from f_newsize to oldsize-1 are destructed.
        //! if f_newsize>oldsize, the elements from oldsize to f_newsize-1 are
        //! constructed with f_default.
        //! \pre 
        //! The user has to ensure that the newsize does not exceed the static
        //! capacity of the vector.
        void resize(size_type f_newsize, const ValueType& f_default);

        //! Use TFixedVector as C array. Direct read/write access to data, returns
        //! pointer to the first element.
        ValueType* c_array();

        //! Use TFixedVector as const C array. Direct readonly access to data, returns
        //! const pointer to the first element.
        const ValueType* c_array() const;

        //-------------------------------------------------------------------------
        /// @brief              erase all the elements in the vector and copies the specified
        //                      element in the empty vector
        /// @param f_param_r    f_value is the element to be set to the FixedVector
        //-------------------------------------------------------------------------
        void set(const ValueType& f_value);

        //! Potentially dangerous(!) adding of an allocated, but not initialized element at the end.
        //! Immediately construct it like this:
        //! @code new (myFixedVector.insert_uninitialized_back()) myPayload(ctorArgs); @endcode
        iterator insert_uninitialized_back();

    private:
        vfc::int32_t m_usedElements;   //!< Counter: how much elements are actually used
        ValueType* m_basePointer_p; //!< Basepointer: points to the start of allocated memory
        AllocatorType m_alloc;      //!< Allocator holds the vectors memory

        //! Allocate the static memory.
        bool mem_alloc();

        //! Initialize the first f_count elements if DontCallCTorType is false. If it is
        //! true (e.g. for POD types) no initialization is done.
        template<class DontCallCTorType>
        void init(size_type f_count, const ValueType&, DontCallCTorType);

        //! Change the number of stored elements. If oldsize<f_newsize, the new
        //! elements are constructed with f_default if dontCallConstructor is false.
        template<class DontCallCTorType>
        void resize_intern(size_type f_newsize, const ValueType& f_default, DontCallCTorType);

        //! Check if a number of elements fits in the vectors capacity.
        bool capacity_check(size_type f_numElements) const;

        void construct(iterator f_start, iterator f_end, const ValueType& elem, true_t);
        void construct(iterator f_start, iterator f_end, const ValueType& elem, false_t);

        void destruct(iterator f_start, iterator f_end, true_t);
        void destruct(iterator f_start, iterator f_end, false_t);

        void set(const ValueType& f_value, vfc::true_t);
        void set(const ValueType& f_value, vfc::false_t);
    };

} // namespace vfc

#include "vfc/container/vfc_fixedvector.inl"

#endif //VFC_FIXEDVECTOR_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fixedvector.hpp  $
//  Revision 1.11 2016/11/28 09:12:16MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Fixedvector: add reverse iterators (mantis0005387)
//  Revision 1.10 2014/05/16 13:08:54MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Add TList member function to create uninitialized entry (mantis0003517)
//  Revision 1.9 2014/05/09 13:46:21MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Add TList member function to create uninitialized entry (mantis0003517)
//  Revision 1.8 2007/10/31 14:04:25MEZ vmr1kor 
//  max_size() function Added
//  Mantis Id :- 0001809
//  Revision 1.7 2007/09/19 18:35:10IST vmr1kor 
//  - set() function added (mantis 1771)
//  Revision 1.6 2007/02/27 20:38:57IST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added ctors which take an allocator object as argument (mantis 1472)
//  Revision 1.5 2007/02/19 10:30:52CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - major rework of the fixedvector: adaption to new error handling, bug fixes, documentation (mantis 1388)
//  Revision 1.4 2007/01/09 09:32:55CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - bugfixes for functions isIndexInRange() and capacity_check() (mantis 1364)
//  Revision 1.3 2006/11/21 14:39:35CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - changed size_t types to signed int (mantis 1314)
//  Revision 1.2 2006/11/16 14:41:20CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.1 2006/10/23 16:46:42CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/container/container.pj
//=============================================================================
