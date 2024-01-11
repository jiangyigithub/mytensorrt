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
///     $Source: vfc_fixedvector.inl $
///     $Revision: 1.15 $
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

#include <utility>                      // used for forward
#include "vfc/core/vfc_assert.hpp"      // used for VFC_ENSURE 
#include "vfc/core/vfc_type_traits.hpp" //
#include "vfc/core/vfc_metaprog.hpp"    // used for TIf()  
#include "vfc/core/vfc_algorithm.hpp"   // used for fill_n(),contiguous_fill_n()

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>  
inline
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::TFixedVector() :
    m_usedElements(0),
    m_basePointer_p(0),
    m_alloc()
{
    mem_alloc();

    VFC_ENSURE( (0 != m_basePointer_p) && (0 == m_usedElements) );
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::TFixedVector(size_type f_n) :
    m_usedElements(0),
    m_basePointer_p(0),
    m_alloc()
{
    VFC_REQUIRE( (f_n <= MAX_SIZE) && (0 <= f_n) );

    init(f_n, ValueType(), hasTrivialCTor_t());

    VFC_ENSURE( (0 != m_basePointer_p) && (f_n == m_usedElements) );
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline 
vfc::TFixedVector<ValueType,
                  CapacityValue, 
                  AllocatorType>::TFixedVector(size_type f_n, const ValueType& f_default) :
    m_usedElements(0),
    m_basePointer_p(0),
    m_alloc()
{
    VFC_REQUIRE( (f_n <= MAX_SIZE) && (0 <= f_n) );

    init(f_n, f_default, false_t());

    VFC_ENSURE( (0 != m_basePointer_p) && (f_n == m_usedElements) );
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline 
vfc::TFixedVector<ValueType,
                  CapacityValue, 
                  AllocatorType>::TFixedVector(const TFixedVector& f_rhs) :
    m_usedElements(0),
    m_basePointer_p(0),
    m_alloc()
{
    if (true == mem_alloc())
    {
        this->operator=(f_rhs);
    }

    VFC_ENSURE(size() == f_rhs.size());
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline 
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::~TFixedVector()
{
    if (0 != m_basePointer_p)
    {
        if (m_usedElements > 0)
        {
            destruct(m_basePointer_p,m_basePointer_p + m_usedElements,hasTrivialDTor_t());
        }
        m_alloc.deallocate(reinterpret_cast<typename AllocatorType::pointer>(m_basePointer_p),MAX_SIZE);
        m_basePointer_p = 0;
    }
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
const vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>& 
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::operator=(const TFixedVector& f_rhs)
{
    if ( (this != &f_rhs) && (0 != m_basePointer_p) )
    {
        clear();
        for (vfc::int32_t count = 0; count < f_rhs.m_usedElements; ++count)
        {
            m_alloc.construct(reinterpret_cast<typename AllocatorType::pointer>(m_basePointer_p + count),
                              f_rhs[count]);
        }
        m_usedElements = f_rhs.size();
    }

    VFC_ENSURE( (size() == f_rhs.size()) );

    return *this;
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::iterator 
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::begin()
{ 
    return m_basePointer_p;
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::const_iterator 
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::begin() const
{
    return m_basePointer_p;
}
        

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::iterator 
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::end()
{
    return m_basePointer_p + m_usedElements;
}
        

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::const_iterator 
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::end() const
{
    return m_basePointer_p + m_usedElements;
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::reverse_iterator
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::rbegin()
{
    return static_cast<reverse_iterator>(end());
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::const_reverse_iterator
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::rbegin() const
{
    return static_cast<const_reverse_iterator>(end());
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::reverse_iterator
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::rend()
{
    return static_cast<reverse_iterator>(begin());
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::const_reverse_iterator
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::rend() const
{
    return static_cast<const_reverse_iterator>(begin());
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::reference 
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::operator[](size_type f_idx) 
{ 
    VFC_REQUIRE( (f_idx < size()) && (0 <= f_idx) );

    return (m_basePointer_p[f_idx]);
}
        

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::const_reference 
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::operator[](size_type f_idx) const 
{     
    VFC_REQUIRE( (f_idx < size()) && (0 <= f_idx) );

    return (m_basePointer_p[f_idx]); 
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::reference 
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::front() 
{ 
    VFC_REQUIRE(!empty());
    return *m_basePointer_p; 
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::const_reference 
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::front() const 
{
    VFC_REQUIRE(!empty());
    return *m_basePointer_p;
}
        

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::reference 
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::back() 
{ 
    VFC_REQUIRE(!empty());
    return m_basePointer_p[m_usedElements-1]; 
}
        

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::const_reference 
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::back() const 
{ 
    VFC_REQUIRE(!empty());
    return m_basePointer_p[m_usedElements-1]; 
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::size_type 
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::size() const
{ 
    return m_usedElements;
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
bool vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::empty() const 
{
    return (0 == m_usedElements);
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::size_type  
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::capacity() const
{ 
    if (0 != m_basePointer_p)
    {
        return MAX_SIZE; 
    }
    else
    {
        return 0;
    }
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::size_type  
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::max_size() const
{ 
    if (0 != m_basePointer_p)
    {
        return MAX_SIZE; 
    }
    else
    {
        return 0;
    }
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::push_back(const ValueType& f_elem)
{
    VFC_REQUIRE(true == capacity_check(m_usedElements+1));

    m_alloc.construct(reinterpret_cast<typename AllocatorType::pointer>
                                                  (m_basePointer_p + m_usedElements), f_elem);
    ++m_usedElements;
}


#if __cplusplus >= 201103L
template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
template<class... Args>
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::reference
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::emplace_back(Args&&... f_args)
{
    VFC_REQUIRE(true == capacity_check(m_usedElements+1));
    return *new (insert_uninitialized_back()) value_type(stlalias::forward<Args>(f_args)...);
}
#endif


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::pop_back()
{
    VFC_REQUIRE(!empty());

    m_alloc.destroy(reinterpret_cast<typename AllocatorType::pointer>
                                                      (m_basePointer_p + (m_usedElements - 1)));
    --m_usedElements;
    
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::clear()
{
    // destruct the previous used elements
    if (m_usedElements > 0)
    {
        destruct(begin(), end(), hasTrivialDTor_t());
        m_usedElements = 0;
    }

    VFC_ENSURE(0 == m_usedElements);
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::resize(size_type f_newsize)
{
    VFC_REQUIRE( (f_newsize <= MAX_SIZE) && (0 <= f_newsize) );

    resize_intern(f_newsize, ValueType(), hasTrivialCTor_t());

    VFC_ENSURE(f_newsize == m_usedElements);
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::resize(size_type f_newsize,
                                                                        const ValueType& f_default)
{
    VFC_REQUIRE( (f_newsize <= MAX_SIZE) && (0 <= f_newsize) );

    resize_intern(f_newsize, f_default, false_t());

    VFC_ENSURE(f_newsize == m_usedElements);
}


//=============================================================================
//  vfc::set(const ValueType& f_value)
//-----------------------------------------------------------------------------
/// @par Description:
/// erase all the elements in the vector and copies the specified
/// element in the empty vector 
/// incase of objects calls it's destructor.
/// @author vmr1kor
/// @ingroup vfc_group_containers
/// @par Requirements: 
/// - vfc_fixedvector.hpp  
//=============================================================================
template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::set(const ValueType& f_value)
{
    set(f_value, typename TIf<vfc::TIsPOD<ValueType>::value, true_t, false_t>::type());
    
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::set(const ValueType& f_value, true_t)
{
    vfc::contiguous_fill_n(m_basePointer_p, m_usedElements , f_value);
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::set(const ValueType& f_value, false_t)
{
    vfc::fill_n(m_basePointer_p, m_usedElements , f_value);
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::iterator
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::insert_uninitialized_back()
{
    VFC_REQUIRE(true == capacity_check(m_usedElements+1));
    // do nothing, allocation is long done, construction must take place outside after this call
    ++m_usedElements;
    return m_basePointer_p + m_usedElements - 1;
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
bool vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::mem_alloc()
{
    m_basePointer_p = reinterpret_cast<value_type*>(m_alloc.allocate(MAX_SIZE));
    VFC_ENSURE(0 != m_basePointer_p);
    return (0 != m_basePointer_p);
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
template<class DontCallCTorType>
inline 
void vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::init(size_type f_count, 
                                                                      const ValueType& f_default, 
                                                                      DontCallCTorType)
{
    VFC_REQUIRE( (f_count <= MAX_SIZE) && (0 <= f_count) );

    if (true == mem_alloc())
    {
        construct(m_basePointer_p, m_basePointer_p + f_count, f_default, DontCallCTorType());
        m_usedElements = f_count;
    }

    VFC_ENSURE( (0 != m_basePointer_p) && (f_count == m_usedElements) );
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
template<class DontCallCTorType>
inline
void vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::resize_intern(size_type f_newsize,
                                                                        const ValueType& f_default,
                                                                        DontCallCTorType)
{
    VFC_REQUIRE( (f_newsize <= MAX_SIZE) && (0 <= f_newsize));
    
    if (f_newsize < m_usedElements)
    {
        size_type oldSize = m_usedElements; 
        m_usedElements = 0;

        // destruct the previous used elements        
        destruct(m_basePointer_p + f_newsize, 
                 m_basePointer_p + oldSize, hasTrivialDTor_t());
    }
    else if (f_newsize > m_usedElements)
    {
        // construct the new elements
        construct(m_basePointer_p + m_usedElements,
                  m_basePointer_p + f_newsize,
                  f_default, DontCallCTorType());
    }
    else
    {
        // nothing, size does not change
    }

    m_usedElements = f_newsize;

    VFC_ENSURE(f_newsize == m_usedElements);
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
ValueType* vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::c_array() 
{ 
    return m_basePointer_p; 
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
const ValueType* vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::c_array() const
{ 
    return m_basePointer_p; 
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
bool 
vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::capacity_check(size_type f_numElements) const
{
    if( (0 <= f_numElements) && (f_numElements <= MAX_SIZE ))
    {
        return true;
    }
    else
    {
        return false;
    }
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::construct(iterator, iterator, 
                                                                           const ValueType&, 
                                                                           true_t)
{
    // nothing
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::construct(iterator f_start, 
                                                                           iterator f_end, 
                                                                           const ValueType& elem,
                                                                           false_t)
{
    VFC_REQUIRE(f_start <= f_end);
    for (iterator iter = f_start; iter != f_end; ++iter)
    {
        m_alloc.construct(reinterpret_cast<typename AllocatorType::pointer>(iter), elem);
    }
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::destruct(iterator, iterator, true_t)
{
    // nothing
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedVector<ValueType, CapacityValue, AllocatorType>::destruct(iterator f_start, 
                                                                          iterator f_end, 
                                                                          false_t)
{
    VFC_ASSERT(f_start <= f_end);
    for (iterator iter = f_start; iter != f_end; ++iter)
    {
        m_alloc.destroy(reinterpret_cast<typename AllocatorType::pointer>(iter));
    }
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fixedvector.inl  $
//  Revision 1.15 2016/11/28 09:12:16MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Fixedvector: add reverse iterators (mantis0005387)
//  Revision 1.14 2014/05/16 13:08:54MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Add TList member function to create uninitialized entry (mantis0003517)
//  Revision 1.13 2014/05/09 13:46:23MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Add TList member function to create uninitialized entry (mantis0003517)
//  Revision 1.12 2012/12/18 08:29:56MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.11 2007/10/31 14:04:26MEZ vmr1kor 
//  max_size() function Added
//  Mantis Id :- 0001809
//  Revision 1.10 2007/09/19 18:35:11IST vmr1kor 
//  - set() function added (mantis 1771)
//  Revision 1.9 2007/03/09 17:39:26IST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added m_alloc member to initialiser list (mantis 1487)
//  Revision 1.8 2007/02/19 10:30:52CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - major rework of the fixedvector: adaption to new error handling, bug fixes, documentation (mantis 1388)
//  Revision 1.7 2007/01/09 09:32:55CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - bugfixes for functions isIndexInRange() and capacity_check() (mantis 1364)
//  Revision 1.6 2006/12/05 11:33:54CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - bugfix: the copy ctor now allocates memory (mantis 1335)
//  Revision 1.5 2006/11/21 17:45:38CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added a missing semicolon and a linebreak (mantis 1317)
//  Revision 1.4 2006/11/21 14:46:16CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - changed size_t types to signed int and added asserts (mantis 1314)
//  - implemented function at() in terms of operator[] (mantis 1315)
//  Revision 1.3 2006/11/08 08:46:45CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - changed std::size_t to vfc::size_t (mantis 1280)
//  Revision 1.2 2006/10/24 08:22:38CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added missing typename keywords (mantis 1229)
//  Revision 1.1 2006/10/23 16:38:10CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/container/container.pj
//=============================================================================
