//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2008 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorised copy, use or 
//  distribution is an offensive act against international law and may be 
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/container
//          Synopsis: fixed circular buffer.
//  Target system(s): 
//       Compiler(s): c++ std conformal
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes: 
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: vmr1kor
//  Department: 
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_fixedcircularbuffer.inl $
///     $Revision: 1.4 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/12/08 10:25:35MEZ $
///     $Locker:  $
///     $Name:  $
///     $State: in_work $
///
/// @par Review Information:
/// - Reviewed version: 
/// - Type (use 'X' to mark):
///     - [ ] Formal Review
///     - [x] Walkthrough
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

#include <algorithm>                // lexicographical_compare
#include <utility>                  // forward
#include "vfc/core/vfc_types.hpp"   // used for int32_t
#include "vfc/core/vfc_assert.hpp"  // used for VFC_ASSERT()


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::TFixedCircularBuffer():
m_basePointer_p(0),
m_alloc(),
m_read_p(0),
m_write_p(0),
m_size(0)
{
    m_basePointer_p = m_alloc.allocate(BUFFER_SIZE);
    VFC_ENSURE(0 != m_basePointer_p);
    m_read_p = m_write_p = m_basePointer_p ;
}

// Copy Constructor
template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::TFixedCircularBuffer(
    const circularBuffer_type& f_param_r) :
m_basePointer_p(0),
m_alloc(),
m_read_p(0),
m_write_p(0),
m_size(0)
{
    m_basePointer_p = m_alloc.allocate(BUFFER_SIZE);

    VFC_ENSURE(0 != m_basePointer_p);

    m_read_p = m_write_p = m_basePointer_p ;

    operator=(f_param_r);
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::TFixedCircularBuffer(
    const AllocatorType& f_alloc)
: 
    m_basePointer_p(0),
    m_alloc(f_alloc),
    m_read_p(0),
    m_write_p(0),
    m_size(0)
{
    m_basePointer_p = m_alloc.allocate(BUFFER_SIZE);
    VFC_ENSURE(0 != m_basePointer_p);
    m_read_p = m_write_p = m_basePointer_p ;
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
const
inline
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>& 
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::operator=
                                                (const TFixedCircularBuffer<ValueType,
                                                 CapacityValue, AllocatorType>& f_param_r
                                                 )
{
    if (this != &f_param_r)
    {
        //  clear the circular buffer
        clear();

        if( !f_param_r.empty() )
        {
            // if readpointer < writepointer , push elements to the Destination 
            // circularBuffer from readpointer to the writepointer
            if(f_param_r.m_read_p < f_param_r.m_write_p)
            {
                pointer read_p = f_param_r.m_read_p ;

                while( read_p != f_param_r.m_write_p)
                {
                    this->push(*read_p);
                    read_p++;
                }
            }

            // if readpointer > writepointer , push elements to the Destination
            // from readpointer till end of circularBuffer and from starting pointer
            // till writepointer
            else
            {
                pointer end_p = &(f_param_r.m_basePointer_p[BUFFER_SIZE-1]) ;

                for (pointer read_p = f_param_r.m_read_p ; read_p <= end_p ; read_p++)
                {
                    this->push(*read_p);
                }
                for (pointer read_p = &(f_param_r.m_basePointer_p[0]) ; read_p < f_param_r.m_write_p ; read_p++)
                {
                    this->push(*read_p);
                }
            }
        }
    }
    return *this;
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline 
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::~TFixedCircularBuffer()
{
    clear();

    m_alloc.deallocate(m_basePointer_p,BUFFER_SIZE);

    m_basePointer_p = 0;
    m_write_p = m_read_p = m_basePointer_p ;
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::push_intern(const ValueType* f_param_r)
{
    if(BUFFER_SIZE == m_size)
    {
        m_alloc.destroy(m_write_p);
    }
    
    if (f_param_r)
    {
        //construct the value by Allocator Construct.
        m_alloc.construct(m_write_p, *f_param_r);
    }
    // else leave newnode_p's payload completely uninitialized

    if (true == IsOverFlow()) 
    {
        incrementPointer(&m_read_p);
    }
    else
    {
        ++m_size;
    }

    //Increment Write pointer
    incrementPointer(&m_write_p);
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::push(const ValueType& f_param_r)
{
    this->push_intern(& f_param_r);
}

#if __cplusplus >= 201103L
template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
template<class... Args>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::reference
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::emplace(Args&&... f_args)
{
    return *new (&push_uninitialized()) value_type(stlalias::forward<Args>(f_args)...);
}
#endif

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::reference
 vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::push_uninitialized()
{
    this->push_intern(0);
    return this->back();
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::pop()
{
    VFC_REQUIRE(m_size > 0);

    m_alloc.destroy(m_read_p);
    --m_size;
    incrementPointer(&m_read_p);
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::allocator_type
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::get_allocator() const
{
   return m_alloc ;
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::reference
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::back()
{
    VFC_REQUIRE(m_size > 0);
    pointer decr_p = 0;
    getDecrementPointer(m_write_p,&decr_p);
    return *decr_p;
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::const_reference
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::back() const
{
    VFC_REQUIRE(m_size > 0);
    pointer decr_p = 0;
    getDecrementPointer(m_write_p,&decr_p);
    return *decr_p;
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::reference
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::front()
{
    VFC_REQUIRE(m_size > 0);
    return *m_read_p;
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::const_reference
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::front() const
{
    VFC_REQUIRE(m_size > 0);
    return *m_read_p;
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::reference
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::operator[](int32_t f_index)
{
    VFC_REQUIRE( (f_index < m_size) && (0 <= f_index) );

    if((m_read_p+f_index) <= (&m_basePointer_p[BUFFER_SIZE-1]) )
    {
        return *(m_read_p+f_index) ;
    }
    else
    {
        return m_basePointer_p [ f_index - (&m_basePointer_p[BUFFER_SIZE-1] - m_read_p) - 1 ] ;
    }
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::const_reference
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::operator[](int32_t f_index) const
{
    VFC_REQUIRE( (f_index < m_size) && (0 <= f_index) );

    if((m_read_p+f_index) <= (&m_basePointer_p[BUFFER_SIZE-1]) )
    {
        return *(m_read_p+f_index) ;
    }
    else
    {
        return m_basePointer_p [ f_index - (&m_basePointer_p[BUFFER_SIZE-1] - m_read_p) - 1 ] ;
    }
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
bool vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::empty() const
{
    return (0 == m_size);
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::size_type
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::size() const
{
    return m_size;
}


template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::size_type
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::capacity() const
{
    return BUFFER_SIZE ;
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::clear()
{
    vfc::int32_t size = m_size ;
    for( vfc::int32_t i = 0 ; i < size ; ++i)
    {
        this->pop();
    }

    VFC_ENSURE(0 == m_size);
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::iterator
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::begin()
{
    return CIterator(this,0);  // 0(zero) signify the index
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::const_iterator
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::begin() const
{
    return CConstIterator(this,0); // 0(zero) signify the index
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::iterator
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::end()
{
    return CIterator(this,size()); // size() signify the index
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::const_iterator
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::end() const
{
    return CConstIterator(this,size());    // size() signify the index
}

template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::reverse_iterator
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::rbegin()
{
    return static_cast<reverse_iterator>(end()); 
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::const_reverse_iterator
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::rbegin() const
{
    return static_cast<const_reverse_iterator>(end()); 
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::reverse_iterator
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::rend()
{
    return static_cast<reverse_iterator>(begin()); 
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::const_reverse_iterator
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::rend() const
{
    return static_cast<const_reverse_iterator>(begin()); 
}



// CConstIterator Functions

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator const &
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::operator=(CConstIterator const & f_rhs)
{
    if(this != &f_rhs)
    {
        m_index = f_rhs.m_index;
        m_cirBuff_cp = f_rhs.m_cirBuff_cp;
    }
    return *this ;
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator&
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::operator ++()
{
    m_index++ ;
    return* this ;
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator
// PRQA S 2427 ++
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::operator ++(int)
// PRQA S 2427 --
{
    self_type temp = *this ;
    m_index++ ;
    return temp ;
}


template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator&
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::operator --()
{
    --m_index ;
    return *this ;
}


template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator
// PRQA S 2427 ++
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::operator --(int)
// PRQA S 2427 --
{
    self_type temp = *this ;
    --m_index ;
    return temp;
}


template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
const typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::
    operator +(vfc::int32_t f_val) const
{
    self_type iterTemp = *this ;
    iterTemp += f_val ;
    return iterTemp ;
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator&
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::operator += ( vfc::int32_t f_val)
{
    m_index+= f_val ;
    return *this ;
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
const typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::
    operator - (vfc::int32_t f_val) const
{
    self_type iterTemp = *this ;
    iterTemp -= f_val ;
    return iterTemp ;
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator&
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::
    operator -= ( vfc::int32_t f_val)
{
    m_index -=  f_val ;
    return *this ;
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
bool 
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::operator < (
    const typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator& f_self_type) 
    const
{
    return ( m_index < f_self_type.m_index );
}


template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
bool 
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::operator <= (
    const typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator& f_self_type) 
    const
{
    return ( m_index <= f_self_type.m_index );
}


template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
bool 
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::operator > (
    const typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator& f_self_type) 
    const
{
    return ( m_index > f_self_type.m_index );
}


template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
bool 
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::operator >= (
    const typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator& f_self_type) 
    const
{
    return ( m_index >= f_self_type.m_index );
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::reference
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::operator*() const
{
    return (m_cirBuff_cp->operator [](m_index));
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::pointer
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::operator->() const
{
    return &(m_cirBuff_cp->operator [](m_index));
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
bool 
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::operator != (
    const typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator& f_self_type) 
    const
{
    return ( m_index != f_self_type.m_index );
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
bool 
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator::operator == ( 
    const typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CConstIterator& f_self_type) 
    const
{
    return ( m_index == f_self_type.m_index );
}


// CIterator Functions

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator&
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator::operator ++()
{
    this->m_index++ ;
    return *this ;
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator
// PRQA S 2427 ++
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator::operator ++(int)
// PRQA S 2427 --
{
    self_type temp = *this ;
    this->m_index++ ;
    return temp ;
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator&
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator::operator --()
{
    --(this->m_index) ;
    return *this ;
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator
// PRQA S 2427 ++
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator::operator --(int)
// PRQA S 2427 --
{
    self_type temp = *this ;
    --(this->m_index) ;
    return temp;
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator::reference
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator::operator*() const
{
    return (const_cast<circularBuffer_type *>(this->m_cirBuff_cp))->operator[] (this->m_index);
}


template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator::pointer
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator::operator->() const
{
    return &((const_cast<circularBuffer_type *>(this->m_cirBuff_cp))->operator[](this->m_index));
}


template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
const typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator::operator +(vfc::int32_t f_val) const
{
    self_type iterTemp = *this ;
    iterTemp += f_val ;
    return iterTemp ;
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator&
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator::operator += ( vfc::int32_t f_val)
{
    this->m_index+= f_val ;
    return *this ;
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
const typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator
 vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator::operator - (vfc::int32_t f_val) const
{
    self_type iterTemp = *this ;
    iterTemp -= f_val ;
    return iterTemp ;
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
typename vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator&
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::CIterator::operator -= ( vfc::int32_t f_val)
{
    this->m_index -=  f_val ;
    return *this ;
}

// Global Functions

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline
bool vfc::operator == (
        const TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>& f_cirBuffer1_r,
        const TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>& f_cirBuffer2_r)
{
    if ( f_cirBuffer1_r.size() != f_cirBuffer2_r.size() )
    {
        return false;
    }
    else
    {
        return stlalias::equal(f_cirBuffer1_r.begin(), f_cirBuffer1_r.end(), f_cirBuffer2_r.begin());
    }
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline 
bool vfc::operator !=(
    const TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>& f_cirBuffer1_r, 
    const TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>& f_cirBuffer2_r)
{ 
    return !(f_cirBuffer1_r == f_cirBuffer2_r); 
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline 
bool vfc::operator <(
    const TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>& f_cirBuffer1_r,
    const TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>& f_cirBuffer2_r)
{ 
    return stlalias::lexicographical_compare(f_cirBuffer1_r.begin(), f_cirBuffer1_r.end(), 
        f_cirBuffer2_r.begin(), f_cirBuffer2_r.end()); 
}


template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline 
bool vfc::operator >(
    const TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>& f_cirBuffer1_r, 
    const TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>& f_cirBuffer2_r)
{ 
    return f_cirBuffer2_r < f_cirBuffer1_r; 
}


template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline 
bool vfc::operator <=(
    const TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>& f_cirBuffer1_r, 
    const TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>& f_cirBuffer2_r)
{ 
    return !(f_cirBuffer2_r < f_cirBuffer1_r); 
}

template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
inline 
bool vfc::operator >=(
    const TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>& f_cirBuffer1_r, 
    const TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>& f_cirBuffer2_r)
{ 
    return !(f_cirBuffer1_r < f_cirBuffer2_r); 
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
bool vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::IsOverFlow() const
{
    // check if we reach the last read element
    if ((m_size + 1) > BUFFER_SIZE)
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
void 
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::incrementPointer(pointer* f_index_pp) const
{
    if(*f_index_pp == &m_basePointer_p[BUFFER_SIZE-1])
    {
        *f_index_pp = m_basePointer_p ;
    }
    else
    {
        (*f_index_pp)++ ;    
    }
}


template<class ValueType, vfc::int32_t CapacityValue, class AllocatorType> 
inline
void 
vfc::TFixedCircularBuffer<ValueType, CapacityValue, AllocatorType>::getDecrementPointer
                                        (pointer f_write_p ,pointer* f_decr_pp ) const
{
    if(f_write_p == m_basePointer_p)
    {
        *f_decr_pp = &m_basePointer_p[BUFFER_SIZE-1] ;
    }
    else
    {
        *f_decr_pp = --f_write_p ;
    }
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fixedcircularbuffer.inl  $
//  Revision 1.4 2014/12/08 10:25:35MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_fixedcircularbuffer: fix MISRA warning (0004847)
//  Revision 1.3 2014/09/25 10:08:41MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Add TCircularBuffer member function to create uninitialized entry (mantis0004553)
//  Revision 1.2 2011/09/23 07:49:32MESZ Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) 
//  -  Modifiable l-value on left side of operator==,violating rule no:8.0.4(mantis:4004)
//  Revision 1.1 2010/08/11 21:19:44IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/container/container.pj
//  Revision 1.4 2009/02/05 05:10:22MEZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Removal of QAC++ warnings.
//  (Mantis : 0002562)
//  Revision 1.3 2009/01/08 11:34:17IST Vinaykumar Setty (RBEI/EAC1) (vmr1kor) 
//  - TFixedCircularBuffer destructor call destroy() on elements that were never constructed problem fixed ( mantis :- 2424 )
//  Revision 1.2 2008/09/30 15:58:52IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor) 
//  - Review comments and QAC++ related issues are implemented ( mantis :- 001574,002341,002342 )
//  Revision 1.1 2008/09/04 14:09:30IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/circular_buffer/circular_buffer.pj
//=============================================================================
