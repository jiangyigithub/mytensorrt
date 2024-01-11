//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2007 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorised copy, use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/container
//          Synopsis: Linked List class.
//  Target system(s):
//       Compiler(s): c++ std conformal
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: dkn2kor
//  Department:
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_list.inl $
///     $Revision: 1.7 $
///     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
///     $Date: 2014/11/21 12:38:11MEZ $
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

#include <limits>                               //used for numeric_limits
#include <functional>                           //equal_to
#include <utility>                              //forward
#include "vfc/core/vfc_assert.hpp"              //VFC_REQUIRE, VFC_ENSURE


namespace
{
    template <class ValueType>
    class TRemovePredicate
    {
        public:
            //constructor
            explicit TRemovePredicate(const ValueType& f_lhs) : m_lhs(f_lhs)
            {
            }
            bool operator() (const ValueType& f_rhs)
            {
                 return (m_lhs == f_rhs);
            }
        private:
            const ValueType& m_lhs;
    };
}

// maxSizePolicy Definition
inline vfc::int32_t vfc::intern::CListMaxsizePolicy::maxSizePolicy()
{
    return stlalias::numeric_limits<vfc::int32_t>::max();
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::TList() : m_numElements(0),
        m_nodeAlloc(), m_head()
{
    //intentionally left blank
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::TList(
    const nodealloc_type& f_alloc_r) : m_numElements(0),
        m_nodeAlloc(f_alloc_r), m_head()
{
    //intentionally left blank
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::TList(
    size_type f_count) : m_numElements(0),
        m_nodeAlloc(), m_head()
{
    VFC_REQUIRE(max_size() >= f_count );

    insert(end(), f_count, value_type());

    VFC_ENSURE ( size() == f_count );
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::TList(
    size_type f_count, const value_type& f_value_r) : m_numElements(0),
        m_nodeAlloc(), m_head()
{
    VFC_REQUIRE(max_size() >= f_count);

    insert(end(), f_count, f_value_r);

    VFC_ENSURE ( size() == f_count );
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::TList(
    size_type f_count, const value_type& f_value_r,const nodealloc_type& f_alloc_r) : m_numElements(0),
        m_nodeAlloc(f_alloc_r), m_head()
{
    VFC_REQUIRE(max_size() >= f_count);

    insert(end(), f_count, f_value_r);

    VFC_ENSURE ( size() == f_count );
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::TList(
    const list_type& f_rhs_r) : m_numElements(0),
        m_nodeAlloc(static_cast<nodealloc_type>(f_rhs_r.get_allocator())), m_head()
{
    //insert each of the element available in the other list
    insert(end(), f_rhs_r.begin(), f_rhs_r.end());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
template<class InIteratorType>
inline
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::TList(
    InIteratorType f_first, InIteratorType f_last) : m_numElements(0),m_head()
{
    insert(end(), f_first, f_last);
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
template<class InIteratorType>
inline
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::TList(
    InIteratorType f_first, InIteratorType f_last, const nodealloc_type& f_alloc_r) :
    m_numElements(0), m_nodeAlloc(f_alloc_r), m_head()
{
    insert(end(), f_first, f_last);
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
const vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>&
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::operator= (const list_type& f_rhs_r)
{
    if(this != &f_rhs_r)
    {
        assign(f_rhs_r.begin(), f_rhs_r.end());
    }
    return *this ;
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::~TList()
{
    clear();
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::iterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::begin()
{
    return CListIterator(m_head.next());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::const_iterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::begin() const
{
    return CListConstIterator(m_head.next());
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::iterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::end()
{
    return CListIterator(&m_head);
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::const_iterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::end() const
{
    return CListConstIterator(const_cast<position_type*>(&m_head));
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::reverse_iterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::rbegin()
{
    return reverse_iterator(end());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::const_reverse_iterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::rbegin() const
{
    return const_reverse_iterator(end());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::reverse_iterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::rend()
{
    return reverse_iterator(begin());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::const_reverse_iterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::rend() const
{
    return const_reverse_iterator(begin());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
bool vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::empty() const
{
    return (0 == m_numElements);
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::size_type
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::size() const
{
    return m_numElements;
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::size_type
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::max_size() const
{
    return MaxSizePolicyType::maxSizePolicy();
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::size_type
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::capacity() const
{
    return max_size();
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::resize(size_type f_newSize)
{
    resize(f_newSize,ValueType());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::resize(size_type f_newSize , const value_type& f_default)
{
    VFC_REQUIRE( (f_newSize <= max_size()) && (0 <= f_newSize));

    if(f_newSize < m_numElements)
    {
        size_type diffSize = m_numElements - f_newSize ;
        for(size_type count = 0; count < diffSize; count++)
        {
            pop_back();
        }
    }
    else if ( f_newSize > m_numElements)
    {
        iterator iterEnd = end() ;
        size_type diffSize = f_newSize - m_numElements ;
        insert(iterEnd, diffSize , f_default);
    }

    VFC_ENSURE ( m_numElements == f_newSize );
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::allocator_type
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::get_allocator() const
{
    return allocator_type(m_nodeAlloc);
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::push_back(
    const value_type& f_value_r)
{
    VFC_REQUIRE(max_size() > m_numElements);
    insert(end(), f_value_r);
}

#if __cplusplus >= 201103L
template<class ValueType, class AllocatorType, class MaxSizePolicyType>
template<class... Args>
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::reference
vfc::TList<ValueType, AllocatorType, MaxSizePolicyType>::emplace_back(Args&&... f_args)
{
    VFC_REQUIRE(max_size() > m_numElements);
    return *new (&(insertElement(end())->value())) value_type(stlalias::forward<Args>(f_args)...);
}
#endif

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::iterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::insert_uninitialized_back()
{
    VFC_REQUIRE(max_size() > m_numElements);
    return CListIterator(insertElement(end()));
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::pop_back()
{
    VFC_REQUIRE(0 != m_numElements);
    removeElement(m_head.prev());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::push_front(
    const value_type& f_value_r)
{
    VFC_REQUIRE(max_size() > m_numElements);
    insert(begin(), f_value_r);
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::iterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::insert_uninitialized_front()
{
    VFC_REQUIRE(max_size() > m_numElements);
    return CListIterator(insertElement(begin()));
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::pop_front()
{
    VFC_REQUIRE(0 != m_numElements);
    erase(begin());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::iterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::insert(
    iterator f_pos, const value_type& f_value_r)
{
    VFC_REQUIRE(max_size() > m_numElements);

    CListIterator iter(insertElement(f_pos, &f_value_r));
    return iter;
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::insert(
    iterator f_pos, size_type f_count, const value_type& f_value_r)
{
    VFC_REQUIRE(f_count >= 0);
    VFC_REQUIRE(max_size() >= (m_numElements + f_count));
    for(size_type count = f_count; count > 0; --count)
    {
        insert(f_pos, f_value_r);
    }
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
template<class InIteratorType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::insert(
    iterator f_pos, InIteratorType f_first, InIteratorType f_last)
{
    VFC_REQUIRE(max_size() >= (m_numElements + stlalias::distance(f_first, f_last)));
    for(InIteratorType first =  f_first; first != f_last; first++)
    {
        insert(f_pos, (*first));
    }
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::iterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::insert_uninitialized(
    iterator f_pos)
{
    VFC_REQUIRE(max_size() > m_numElements);
    return CListIterator(insertElement(f_pos));
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::assign(
    size_type f_count, const value_type& f_value_r)
{
    VFC_REQUIRE(f_count >= 0);
    VFC_REQUIRE(max_size() >= f_count);

    iterator iter = begin();
    //replace the existing values with the new values
    for ( ; iter != end() && f_count > 0; ++iter, --f_count)
    {
        *iter = f_value_r;
    }

    if (f_count > 0)
    {
        //if the input count is greater than the existing size of the list
        //then add more values
        insert(end(), f_count, f_value_r);
    }
    else
    {
        //remove the values
        erase(iter, end());
    }
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
template<typename InIteratorType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::assign(
    InIteratorType f_first, InIteratorType f_last)
{
    VFC_REQUIRE(max_size() >= stlalias::distance(f_first, f_last));
    iterator source_it = begin();
    iterator end_it = end();
    for ( ; source_it != end_it && f_first != f_last; ++source_it, ++f_first)
    {
        *source_it = *f_first;
    }
    if (f_first == f_last)
    {
        erase(source_it, end_it);
    }
    else
    {
        insert(end_it, f_first, f_last);
    }
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::iterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::erase(
    iterator f_pos)
{
    VFC_REQUIRE(0 != m_numElements && 0!= f_pos.m_node_p);
    return CListIterator(removeElement(f_pos.m_node_p));
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::iterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::erase(
    iterator f_first, iterator f_last)
{
    VFC_REQUIRE((stlalias::distance(f_first, f_last) <= m_numElements) &&
                (0!= f_first.m_node_p) && (0!= f_last.m_node_p));

    CListIterator iter(removeRange(
        f_first.m_node_p, f_last.m_node_p)) ;

    return iter ;
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::remove(
    const value_type& f_value_r)
{
    remove_if(TRemovePredicate<value_type>(f_value_r));
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
template<class PredicateType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::remove_if(
    PredicateType f_predicate)
{
    iterator first = begin();
    iterator last = end();
    while (first != last)
    {
        if (f_predicate(*first))
        {
            first = erase(first);
        }
        else
        {
            ++first;
        }
    }
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::reverse()
{
    // for Fundamental datatypes, use stlalias::reverse
    // for UserDefined Data types, use TList::reverse function
    reverse(typename TIf<TIsFundamental<value_type>::value,
        true_t, false_t>::type());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::swap(
    list_type& f_rhs_r)
{
    //dont do anything if the list specified is same as this list
    if(this == &f_rhs_r)
    {
        return;
    }

    if(this->get_allocator() == f_rhs_r.get_allocator())
    {
        swapNodes(f_rhs_r);
    }
    else
    {
        swapData(f_rhs_r);
    }
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::unique()
{
    unique(stlalias::equal_to<ValueType>());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
template<class BinaryPredicateType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::unique(
    BinaryPredicateType f_binaryPredicate)
{
    //return if there are not elements in the list
    if (empty())
    {
        return;
    }

    iterator first = begin();
    iterator last = end();

    iterator next = first;
    while (++next != last)
    {
        if (f_binaryPredicate(*first, *next))
        {
            erase(next);
        }
        else
        {
            first = next;
        }
        next = first;
    }
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::splice(
    iterator f_pos, list_type& f_list_r)
{
    splice(f_pos,f_list_r,f_list_r.begin(),f_list_r.end());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::splice(
    iterator f_pos, list_type& f_list_r, iterator f_iter)
{
    iterator next = f_iter;
    ++next;
    splice(f_pos,f_list_r,f_iter,next);
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::splice(
    iterator f_pos, list_type& f_list_r, iterator f_first, iterator f_last)
{
    VFC_REQUIRE(0!= f_pos.m_node_p &&
                0!= f_first.m_node_p &&
                0!= f_last.m_node_p);

    const size_type count = static_cast<size_type>(stlalias::distance(f_first, f_last));

    VFC_REQUIRE(count <= f_list_r.size() );
    VFC_REQUIRE(max_size() >= (m_numElements + count));

    if (f_first != f_last)
    {
        if(this->get_allocator() != f_list_r.get_allocator())
        {
            insert(f_pos, f_first, f_last);
            f_list_r.erase(f_first, f_last);
        }
        else
        {
            relinkNode(f_pos, f_first, f_last);
            f_list_r.m_numElements -= count;
            m_numElements += count;
        }
    }
    else
    {
        //intentionally left blank
    }

}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::merge(
    list_type& f_list_r)
{
    merge(f_list_r, stlalias::less<ValueType>());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
template<class OrderingType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::merge(
    list_type& f_list_r, OrderingType f_orderType)
{
    VFC_REQUIRE ( max_size() >= (m_numElements + f_list_r.size()) );

    if (this != &f_list_r)
    {
        iterator first1 = begin();
        iterator last1 = end();
        iterator first2 = f_list_r.begin();
        iterator last2 = f_list_r.end();
        if(this->get_allocator() != f_list_r.get_allocator())
        {
            while (first1 != last1 && first2 != last2)
            {
                if (f_orderType(*first2, *first1))
                {
                    iterator next = first2;
                    ++next;
                    splice(first1, f_list_r, first2);
                    first2 = next;
                }
                else
                {
                    ++first1;
                }
            }
            if (first2 != last2)
            {
                splice(last1, f_list_r, first2, last2);
            }
        }
        else
        {
            //relink nodes
            while (first1 != last1 && first2 != last2)
            {
                if (f_orderType(*first2, *first1))
                {
                    iterator next = first2;
                    relinkNode(first1, first2, ++next);
                    first2 = next;
                }
                else
                {
                    ++first1;
                }
            }
            if (first2 != last2)
            {
                relinkNode(last1, first2, last2);
            }
            m_numElements += f_list_r.m_numElements;
            f_list_r.m_numElements = 0;
        }
    }

    VFC_ENSURE(f_list_r.empty());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::sort()
{
    sort(stlalias::less<ValueType>());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
template <class ComparisonPredicateType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::sort(
    ComparisonPredicateType f_predicate)
{
    //Nothing to do if there arent 2 items
    if(2 > m_numElements)
    {
        return;
    }

    //take the current list
    iterator curr, iter;
    curr = iter = begin();
    //point to the second element
    ++curr;
    while(curr != end())
    {
        if(f_predicate(*curr , *iter))
        {
            //do the swap and continue
            iterator next = curr;
            relinkNode(iter, curr, ++next);
            curr = next;
            iter = begin();
        }
        else
        {
            ++iter;
        }
        if(iter == curr)
        {
            ++curr;
            iter = begin();
        }
    }
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::clear()
{
    erase(begin(), end());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::reference
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::front()
{
    VFC_REQUIRE(!empty());
    return *begin();
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::const_reference
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::front() const
{
    VFC_REQUIRE(!empty());
    return *begin();
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::reference
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::back()
{
    VFC_REQUIRE(!empty());
    return *(--end());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::const_reference
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::back() const
{
    VFC_REQUIRE(!empty());
    return *(--end());
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::node_type*
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::insertElement(
    iterator f_pos,
    const value_type* f_value_p)
{
    VFC_REQUIRE(max_size() > m_numElements);

    position_type* prevNode_p = f_pos.m_node_p->prev() ;
    position_type* nextNode_p = f_pos.m_node_p ;

    //allocate memory for the node
    node_type*  newnode_p = m_nodeAlloc.allocate(1);

    VFC_REQUIRE(0 != newnode_p);

    if (0 != newnode_p)
    {
        if (f_value_p)
        {
            //construct the value by Allocator Construct.
            m_nodeAlloc.construct(newnode_p, CNode(*f_value_p));
        }
        // else leave newnode_p's payload completely uninitialized

        //set the linked list
        position_type::linkNode(newnode_p, prevNode_p, nextNode_p);
        ++m_numElements;
    }
    return newnode_p;
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::node_type*
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::insertElement(
    iterator f_pos)
{
    VFC_REQUIRE(max_size() > m_numElements);

    position_type* prevNode_p = f_pos.m_node_p->prev() ;
    position_type* nextNode_p = f_pos.m_node_p ;

    //allocate memory for the node
    node_type*  newnode_p = m_nodeAlloc.allocate(1);

    VFC_REQUIRE(0 != newnode_p);

    if (0 != newnode_p)
    {
        // else leave newnode_p's payload completely uninitialized

        //set the linked list
        position_type::linkNode(newnode_p, prevNode_p, nextNode_p);
        ++m_numElements;
    }
    return newnode_p;
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::position_type*
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::removeElement(
    position_type* f_node_p)
{
    VFC_REQUIRE (0 != f_node_p);
    VFC_REQUIRE (f_node_p != &m_head);

    node_type* const node_p = static_cast<node_type*>(f_node_p);

    //set the linked list
    CNodeBase* const l_prev_p = f_node_p->prev();
    CNodeBase* const l_next_p = f_node_p->next();
    position_type* retNode = position_type::unlinkNode(l_prev_p, l_next_p);

    //calls the destructor of the value_type class
    m_nodeAlloc.destroy(node_p);

    //remove memory allocated for the node
    m_nodeAlloc.deallocate(node_p, 1);

    --m_numElements;

    return retNode;
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::position_type*
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::removeRange(
    position_type* f_startNode_p, position_type* f_endNode_p)
{
    VFC_REQUIRE((0 != f_startNode_p) && (0!= f_endNode_p));

    node_type*  current_p = static_cast<node_type*>(f_startNode_p);
    position_type*  next_p = 0;

    position_type* retNode = f_endNode_p;
    //do unlinking only if the there are nodes to remove
    if(f_startNode_p != f_endNode_p)
    {
        //set the linked list
        retNode = position_type::unlinkNode(
            current_p->prev(), f_endNode_p);
    }
    while (current_p != f_endNode_p)
    {
        next_p = current_p->next();
        m_nodeAlloc.destroy(current_p);
        m_nodeAlloc.deallocate(current_p, 1);
        --m_numElements;
        current_p = static_cast<node_type*>(next_p);
    }
    return retNode;
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::reverse(true_t)
{
    stlalias::reverse(begin(), end());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::reverse(
    false_t)
{
    //how does this logic work
    //assume list = 1,2,3,4
    //iter 1: 2,1,3,4
    //iter 2: 3,2,1,4
    //iter 3: 4,3,2,1
    if (2 <= m_numElements)
    {   // no point in reversing if the count there is only 1 element
        iterator last = end();
        for (iterator next = ++begin(); next != last; )
        {   // move next element to beginning
            iterator curr = next;
            relinkNode(begin(), curr, ++next);
        }
    }
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::swapNodes(
    list_type& f_rhs_r)
{
    //this list is empty. so only relink the nodes from the other list.
    if(empty())
    {
        relinkNode(end(), f_rhs_r.begin(), f_rhs_r.end());
    }
    //RHS list is empty. so only relink the nodes from the this list.
    else if(f_rhs_r.empty())
    {
        relinkNode(f_rhs_r.end(), begin(), end());
    }
    //Both lists have data. so swap the nodes
    else
    {
        position_type::swapNodeBase(m_head, f_rhs_r.m_head);
    }
    //swap the counts
    size_type temp = m_numElements;
    m_numElements = f_rhs_r.m_numElements;
    f_rhs_r.m_numElements = temp;
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::swapData(
    list_type& f_rhs_r)
{
    //loop the list

    iterator lhsFirst = begin();
    iterator lhsLast = end();
    iterator rhsFirst = f_rhs_r.begin();
    iterator rhsLast = f_rhs_r.end();
    for ( ; rhsFirst != rhsLast && lhsFirst != lhsLast; ++lhsFirst, ++rhsFirst)
    {
        //exchange the items
        stlalias::swap(*lhsFirst, *rhsFirst);
    }
    if (lhsFirst == lhsLast)
    {
        //remove from right and add to left
        insert(lhsLast, rhsFirst, rhsLast);
        f_rhs_r.erase(rhsFirst, rhsLast);
    }
    else
    {
        //remove from left and add to right
        f_rhs_r.insert(rhsLast, lhsFirst, lhsLast);
        erase(lhsFirst, lhsLast);
    }
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::relinkNode(
    iterator f_pos, iterator f_first, iterator f_last)
{
    position_type::relinkNodeBase( f_pos.m_node_p,
        f_first.m_node_p , f_last.m_node_p );
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Global functions
///////////////////////////////////////////////////////////////////////////////////////////////


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
bool vfc::operator==(
    const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list1_r,
    const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list2_r)
{
    if(f_list1_r.size() != f_list2_r.size())
    {
        return false;
    }

    typedef typename TList<ValueType,AllocatorType,MaxSizePolicyType>::const_iterator const_iterator;

    const_iterator end1 = f_list1_r.end();
    const_iterator iter1 = f_list1_r.begin();

    const_iterator iter2 = f_list2_r.begin();

    while ( (iter1 != end1) && (*iter1 == *iter2))
    {
        ++iter1;
        ++iter2;
    }
    return (iter1 == end1);
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
bool vfc::operator<(
    const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list1_r,
    const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list2_r)
{
    return stlalias::lexicographical_compare(f_list1_r.begin(), f_list1_r.end(),
        f_list2_r.begin(), f_list2_r.end());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
bool vfc::operator!=(
    const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list1_r,
    const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list2_r)
{
    return !(f_list1_r == f_list2_r);
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
bool vfc::operator>(
    const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list1_r,
    const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list2_r)
{
    return f_list2_r < f_list1_r;
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
bool vfc::operator<=(
    const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list1_r,
    const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list2_r)
{
    return !(f_list2_r < f_list1_r);
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
bool vfc::operator>=(
    const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list1_r,
    const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list2_r)
{
    return !(f_list1_r < f_list2_r);
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void vfc::swap(
    TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list1_r,
    TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list2_r)
{
    f_list1_r.swap(f_list2_r);
}

///////////////////////////////////////////////////////////////////////////////////////////////
// CNodeBase implementation
///////////////////////////////////////////////////////////////////////////////////////////////

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase::CNodeBase() :
    m_prev_p(0),m_next_p(0)
{
    m_prev_p = this ;
    m_next_p = this ;
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase::CNodeBase(CNodeBase const & f_rhs) :
    m_prev_p(f_rhs.m_prev_p), m_next_p(f_rhs.m_next_p)
{
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase::~CNodeBase()
{
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase*
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase::next() const
{
    return m_next_p;
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase*&
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase::next()
{
    return m_next_p;
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase*
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase::prev()  const
{
    return m_prev_p;
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase*&
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase::prev()
{
    return m_prev_p;
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase*
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase::linkNode(
    CNodeBase* f_node_p, CNodeBase* f_prev_p,
    CNodeBase* f_next_p)
{
    f_node_p->m_prev_p = f_prev_p;
    f_node_p->m_next_p = f_next_p;
    f_prev_p->next() = f_node_p;
    f_next_p->prev() = f_node_p;
    return f_node_p;
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase*
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase::unlinkNode(
    CNodeBase* f_prev_p, CNodeBase* f_next_p)
{
    f_prev_p->next() = f_next_p;
    f_next_p->prev() = f_prev_p;
    return f_next_p;
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase::swapNodeBase(
    CNodeBase& f_node1_r, CNodeBase& f_node2_r)
{
    CNodeBase* temp_p = f_node1_r.m_prev_p;
    f_node1_r.m_prev_p = f_node2_r.m_prev_p;
    f_node2_r.m_prev_p = temp_p;
    temp_p = f_node1_r.m_next_p;
    f_node1_r.m_next_p = f_node2_r.m_next_p;
    f_node2_r.m_next_p = temp_p;
    f_node1_r.m_next_p->m_prev_p = &f_node1_r;
    f_node1_r.m_prev_p->m_next_p = &f_node1_r;
    f_node2_r.m_next_p->m_prev_p = &f_node2_r;
    f_node2_r.m_prev_p->m_next_p = &f_node2_r;
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
void
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CNodeBase::relinkNodeBase(
    CNodeBase* f_pos_p, CNodeBase* f_first_p,
    CNodeBase* f_last_p)
{
    CNodeBase* prevToLastNode = f_last_p->prev();
    f_first_p->prev()->next() = f_last_p;
    f_last_p->prev() = f_first_p->prev();

    //relink in the current list
    f_first_p->prev() = f_pos_p->prev();
    prevToLastNode->next() = f_pos_p;

    f_pos_p->prev()->next() = f_first_p;
    f_pos_p->prev() = prevToLastNode;
}


///////////////////////////////////////////////////////////////////////////////////////////////
// CListIterator implementation
///////////////////////////////////////////////////////////////////////////////////////////////

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator::CListIterator (
           const typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator& f_iter_r)
           : CListConstIterator()
{
    operator=(f_iter_r);
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator::reference
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator::operator*() const
{
    return ((static_cast<node_type*>(this->m_node_p))->value());
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator::pointer
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator::operator->() const
{
    return &static_cast<node_type*>(this->m_node_p)->value();
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator&
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator::operator++()
{
    this->m_node_p = this->m_node_p->next();
    return *this;
}

// PRQA S 2427 ++
template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator::operator++(int)
{
    self_type temp = *this;
    this->m_node_p = this->m_node_p->next();
    return temp;
}
// PRQA S 2427 --

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator&
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator::operator--()
{
    this->m_node_p = this->m_node_p->prev();
    return *this;
}

// PRQA S 2427 ++
template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator::operator--(int)
{
    self_type temp = *this;
    this->m_node_p = this->m_node_p->prev();
    return temp ;
}
// PRQA S 2427 --

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
const typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator&
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator::operator= (
           const typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListIterator& f_iter_r)
{
    if( this != &f_iter_r)
    {
        this->m_node_p = f_iter_r.m_node_p;
    }
    return *this;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// CListConstIterator implementation
///////////////////////////////////////////////////////////////////////////////////////////////

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
const typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator&
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator::operator= (
          const typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator& f_iter_r)
{
    if( this != &f_iter_r)
    {
        m_node_p = f_iter_r.m_node_p;
    }
    return *this;
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator::reference
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator::operator*( ) const
{
    return ((static_cast<const node_type*>(m_node_p))->value());
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator::pointer
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator::operator->() const
{
    return &static_cast<const node_type*>(m_node_p)->value();
}


template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator&
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator::operator++ ()
{
    m_node_p = m_node_p->next();
    return *this;
}

// PRQA S 2427 ++
template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator::operator++ ( int )
{
    self_type temp = *this;
    m_node_p = m_node_p->next();
    return temp;
}
// PRQA S 2427 --

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator&
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator::operator--()
{
    m_node_p = m_node_p->prev();
    return *this;
}

// PRQA S 2427 ++
template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator::operator--(int)
{
    self_type temp = *this;
    m_node_p = m_node_p->prev();
    return temp ;
}
// PRQA S 2427 --

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
bool
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator::operator==(
    const typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator& f_rhs_r
    ) const
{
    return m_node_p == f_rhs_r.m_node_p;
}

template<class ValueType, class AllocatorType, class MaxSizePolicyType>
inline
bool
vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator::operator!=(
    const typename vfc::TList<ValueType, AllocatorType,MaxSizePolicyType>::CListConstIterator& f_rhs_r
    ) const
{
    return !(*this == f_rhs_r);
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_list.inl  $
//  Revision 1.7 2014/11/21 12:38:11MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - vfc_list: CNodeBase Coding Rule violation: performs shallow copy on its pointer members (mantis0004738)
//  Revision 1.6 2014/11/20 16:38:50MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_list: MISRA warning in removeElement() (mantis0004835)
//  Revision 1.5 2014/08/18 16:19:40MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - ...should be explicitly initialized in the copy constructor (mantis0004594)
//  Revision 1.4 2014/05/16 13:29:02MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Add TList member function to create uninitialized entry (mantis0003517)
//  Revision 1.3 2014/05/09 15:47:01MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Add TList member function to create uninitialized entry (mantis0003517)
//  Revision 1.2 2014/05/09 14:53:09MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Add TList member function to create uninitialized entry (mantis0003517)
//  Revision 1.1 2010/08/11 17:44:19MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/container/container.pj
//  Revision 1.12 2009/02/05 10:04:18MEZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Removal of QAC++ warnings.
//  (Mantis : 0002563)
//  Revision 1.11 2008/10/20 14:12:13IST Vinaykumar Setty (RBEI/EAC1) (vmr1kor)
//  - vfc_assert.hpp is missing in list praposal (Mantis:- 2382)
//  Revision 1.10 2008/09/30 15:47:14IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  - review comments are implemented ( mantis id:- 1793 )
//  Revision 1.9 2008/09/11 17:59:45IST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  -Addition of capacity() function
//  (Mantis : 2080)
//  Revision 1.8 2008/09/01 16:02:40IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  - CList review comments are incorporated( Mantis id :- 1793)
//  - Linker error got fixed ( Mantis id :- 2309)
//  Revision 1.7 2008/02/05 16:05:36IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  CListIterator is Inheriting from CListConstIterator
//  Few Performance issues are addressed
//  doxyComments are added
//  Made TList and TFixedList , two seperate classes
//  Revision 1.6 2007/10/31 16:55:13IST vmr1kor
//  split the default ctor into two: "TList(void)" and "explicit TList(nodealloc_type)
//  Implemented resize() function
//  Mantis Id :- 0001793
//  Revision 1.5 2007/09/27 18:27:03IST vmr1kor
//  != , == operators added for TListCostIterators
//  removed getIterator,getConstIterator,getNode
//  QACPP error got fixed
//  Revision 1.4 2007/09/24 15:00:09IST vmr1kor
//  - moved classes CNodeBase , CNode , CListIterator , CListConstIterator inside TList
//  - Mantis id - 0001385
//  Revision 1.3 2007/09/03 18:48:15IST vmr1kor
//  operator = () function added
//  Revision 1.2 2007/03/26 19:47:21IST dkn2kor
//  - defect on splice and remove element fixed
//=============================================================================


