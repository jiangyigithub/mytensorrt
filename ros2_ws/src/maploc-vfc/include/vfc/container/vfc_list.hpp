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
///     $Source: vfc_list.hpp $
///     $Revision: 1.10 $
///     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
///     $Date: 2014/11/21 12:36:46MEZ $
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

#ifndef VFC_LIST_HPP_INCLUDED
#define VFC_LIST_HPP_INCLUDED

// stdlib includes
#include <iterator>
#include <algorithm>                            //used std::reverse

// vfc/core includes
#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_type_traits.hpp"         // used for hasTrivialD/Ctor_t
#include "vfc/core/vfc_metaprog.hpp"            // used for TInt2Boolean
#include "vfc/core/vfc_static_assert.hpp"

// vfc/memory includes
#include "vfc/memory/vfc_freestore_allocator.hpp"       // TFreeScaleAllocator

namespace vfc
{   // namespace vfc opened

    namespace intern
    {   // namespace intern opened

        struct CListMaxsizePolicy
        {
            static vfc::int32_t maxSizePolicy();
        };
    }   // namespace intern closed

    //=============================================================================
    //  TList<>
    //-----------------------------------------------------------------------------
    //! Linked List class.
    //! The list class is a template class of sequence containers that maintain
    //! their elements in a linear arrangement and allow efficient insertions and
    //! deletions at any location within the sequence. The sequence is stored as a
    //! bidirectional linked list of elements, each containing a member of some type Type.
    //! $Source: vfc_list.hpp $
    //! @param ValueType        DataType to be stored.
    //! @param AllocatorType    AllocatorType used.
    //! @author                 dkn2kor
    //! @ingroup                vfc_group_containers
    //=============================================================================
    template<class ValueType,
    class AllocatorType = vfc::TFreeStoreAllocator<ValueType>,
    class MaxSizePolicyType = intern::CListMaxsizePolicy >
    class TList
    {
    public:

        //typdefes
        typedef TList<ValueType,AllocatorType,MaxSizePolicyType>                list_type;
        /// A type that represents the data type stored in a list.
        typedef ValueType                                                       value_type;
        /// A type that counts the number of elements in a list.
        typedef vfc::int32_t                                                    size_type;
        /// A type that provides the difference between two iterators that refer to
        /// elements within the same list.
        typedef ptrdiff_t                                                       difference_type;
        /// A type that represents the allocator class for a list object.
        typedef AllocatorType                                                   allocator_type;

        typedef stlalias::bidirectional_iterator_tag                            iterator_category;
        typedef typename TInt2Boolean<THasTrivialCTor<value_type>::value>::type hasTrivialCTor_t;
        typedef typename TInt2Boolean<THasTrivialDTor<value_type>::value>::type hasTrivialDTor_t;

    private:

        //=============================================================================
        //  CNodeBase
        //-----------------------------------------------------------------------------
        //! CNodeBase is the base class and used by TList as a datastructure
        //! for linked list implementation.
        //! Each node has pointer to both left and right, to facilitate the mechanism of
        //! bi-directional list.
        //! next() , prev() interfaces enable access the previous and next nodes
        //! respectively.
        //! $Source: vfc_list.hpp $
        //! @author         dkn2kor
        //! @ingroup        vfc_group_containers
        //=============================================================================

        class CNodeBase
        {
        public:

            //---------------------------------------------------------------------
            //! Default constructor.
            //! $Source: vfc_list.hpp $
            //! @author     dkn2kor
            //---------------------------------------------------------------------
            CNodeBase();

            //---------------------------------------------------------------------
            //! Copy constructor.
            //! $Source: vfc_list.hpp $
            //! @author     jat2hi
            //---------------------------------------------------------------------
            CNodeBase(CNodeBase const & f_rhs);

            //---------------------------------------------------------------------
            //! Destructor.
            //! $Source: vfc_list.hpp $
            //! @author     jat2hi
            //---------------------------------------------------------------------
            ~CNodeBase();

            //---------------------------------------------------------------------
            //! Returns the next node.
            //! $Source: vfc_list.hpp $
            //! @return returns pointer to CNodeBase
            //! @author dkn2kor
            //---------------------------------------------------------------------
            CNodeBase*  next()  const;

            //---------------------------------------------------------------------
            //! Returns a reference to the next node.
            //! $Source: vfc_list.hpp $
            //! @return returns reference to CNodeBase.
            //! @author dkn2kor
            //---------------------------------------------------------------------
            CNodeBase*& next();

            //---------------------------------------------------------------------
            //! Returns the previous node.
            //! $Source: vfc_list.hpp $
            //! @return return pointer to previous node.
            //! @author dkn2kor
            //---------------------------------------------------------------------
            CNodeBase*  prev()  const;


            //---------------------------------------------------------------------
            //! Returns a reference to the previous node.
            //! $Source: vfc_list.hpp $
            //! @return returns reference to previous node.
            //! @author dkn2kor
            //---------------------------------------------------------------------
            CNodeBase*& prev();

            //---------------------------------------------------------------------
            //! Links f_node_p inbetween f_prev_p & f_next_p.
            //! $Source: vfc_list.hpp $
            //! @param  f_node_p    node to be linked between f_prev_p and f_next_p.
            //! @param  f_prev_p    pointer to previous node.
            //! @param  f_next_p    pointer to next node.
            //! @return returns pointer to the CNodeBase class.
            //! @author dkn2kor
            //---------------------------------------------------------------------
            static CNodeBase* linkNode(CNodeBase* f_node_p,
                CNodeBase* f_prev_p, CNodeBase* f_next_p);

            //---------------------------------------------------------------------
            //! Unlinks a node inbetween f_prev_p & f_next_p
            //! and sets the linked list in the proper order.
            //! $Source: vfc_list.hpp $
            //! @param  f_prev_p    pointer to previous node.
            //! @param  f_next_p    pointer to next node.
            //! @return returns pointer to the next node.
            //! @author dkn2kor
            //---------------------------------------------------------------------
            static CNodeBase* unlinkNode(CNodeBase* f_prev_p,
                CNodeBase* f_next_p);

            //---------------------------------------------------------------------
            //! Swaps a nodes f_node1_r & f_node2_r.
            //! $Source: vfc_list.hpp $
            //! @param  f_node1_r    reference to CNodeBase.
            //! @param  f_node2_r    reference to CNodeBase.
            //! @author dkn2kor
            //---------------------------------------------------------------------
            static void swapNodeBase(CNodeBase& f_node1_r,
                CNodeBase& f_node2_r);

            //---------------------------------------------------------------------
            //! sequence from f_first_p to one before f_last_p is inserted before f_pos_p.
            //! $Source: vfc_list.hpp $
            //! @param  f_pos_p     position to be inserted.
            //! @param  f_first_p   position of the first element to be relinked.
            //! @param  f_last_p    position just beyond the last element to be relinked.
            //! @author dkn2kor
            //---------------------------------------------------------------------
            static void relinkNodeBase(CNodeBase* f_pos_p,
                CNodeBase* f_first_p, CNodeBase* f_last_p);
        private:
            //---------------------------------------------------------------------
            //! Declare assignment operator as private.
            //! $Source: vfc_list.hpp $
            //! @author     jat2hi
            //---------------------------------------------------------------------
            CNodeBase const & operator=(CNodeBase const & f_rhs);

            CNodeBase       *m_prev_p;    //!< Stores the previous node ptr
            CNodeBase       *m_next_p;    //!< Stores the next node ptr
        };


        //=============================================================================
        //  CNode
        //-----------------------------------------------------------------------------
        //! CNode class is used for storing the value added to the list.
        //! $Source: vfc_list.hpp $
        //! @author                 dkn2kor
        //! @ingroup                vfc_group_containers
        //=============================================================================
        class CNode : public CNodeBase
        {
        public:
            typedef ValueType   value_type;

            //---------------------------------------------------------------------
            //! Constructor takes value_type parameter.
            //! $Source: vfc_list.hpp $
            //! @author dkn2kor
            //---------------------------------------------------------------------
            explicit CNode(const value_type& f_value) : CNodeBase(), m_value(f_value) { }

            //---------------------------------------------------------------------
            //! Returns the reference of the contained value.
            //! $Source: vfc_list.hpp $
            //! @return returns reference of the value.
            //! @author dkn2kor
            //---------------------------------------------------------------------
            inline value_type& value() { return m_value; }

            //---------------------------------------------------------------------
            //! Returns the constant reference of the contained value.
            //! $Source: vfc_list.hpp $
            //! @return returns const reference of the value.
            //! @author dkn2kor
            //---------------------------------------------------------------------
            inline const value_type& value() const { return m_value; }

        private:
            value_type  m_value;      //!< Stores the value the node
        };

    public:
        typedef CNode                                                       node_type;
        typedef CNodeBase                                                   position_type;

    protected:
        typedef typename allocator_type::template rebind<node_type>::other  nodealloc_type;
        ///////////////////////////////////////////////////////////////////////////////////////////////
        // iterators
        ///////////////////////////////////////////////////////////////////////////////////////////////

        //=============================================================================
        //  CListConstIterator
        //-----------------------------------------------------------------------------
        //! CListConstIterator provides a bidirectional iterator that
        //! can read a const element in a list.
        //! $Source: vfc_list.hpp $
        //! @author         dkn2kor
        //! @ingroup        vfc_group_containers
        //=============================================================================
    private:
        class CListConstIterator
        {
        public:
            typedef CListConstIterator                          self_type;
            typedef ptrdiff_t                                   difference_type;
            typedef stlalias::bidirectional_iterator_tag        iterator_category;
            typedef ValueType                                   value_type;
            typedef const ValueType*                            pointer;
            typedef const ValueType&                            reference;

            /// Default c'tor.
            CListConstIterator() : m_node_p(0){}

            /// Copy constructor.
            CListConstIterator(const CListConstIterator& f_iter_r) :  m_node_p(f_iter_r.m_node_p)
            {
                //intentionally left blank
            }

            /// Constructor taking position type
            explicit CListConstIterator(position_type* f_pos_p) : m_node_p(f_pos_p)
            {
                //intentionally left blank
            }

            /// Assignment operator.
            const CListConstIterator& operator=(const CListConstIterator& f_iter_r);

            /// Returns a const reference to the object.
            reference operator*() const;

            /// Returns a const pointer to the object.
            pointer operator->() const ;

            /// Prefix increment iterator.
            self_type& operator++();

            /// Postfix decrement iterator.
            // (Msg Disable 2427 : Direct use of fundamental type.)
            // PRQA S 2427 ++
            self_type operator++(int);
            // PRQA S 2427 --
            // (Msg Enable 2427 : Direct use of fundamental type.)

            /// Prefix decrement iterator.
            self_type& operator--();

            /// Postfix decrement iterator.
            // (Msg Disable 2427 : Direct use of fundamental type.)
            // PRQA S 2427 ++
            self_type operator--(int);
            // PRQA S 2427 --
            // (Msg Enable 2427 : Direct use of fundamental type.)

            /// Equality comparison operator.
            bool operator==(const self_type& f_rhs_r) const;

            /// Inequality comparison operator.
            bool operator!=(const self_type& f_rhs_r) const;

        protected:
            position_type*  m_node_p;           //! Current node pointer
            friend class    TList ;             //! TList class can access private variables
        };

        //=======================================================================
        //  CListIterator
        //-----------------------------------------------------------------------
        //! A type that provides a bidirectional iterator that can
        //! read or modify any element in a list.
        //! $Source: vfc_list.hpp $
        //! @author         dkn2kor
        //! @ingroup        vfc_group_containers
        //=======================================================================
        class CListIterator : public CListConstIterator
        {
            public:
                typedef CListConstIterator                          base_type;
                typedef CListIterator                               self_type;
                typedef ptrdiff_t                                   difference_type;
                typedef stlalias::bidirectional_iterator_tag        iterator_category;
                typedef ValueType                                   value_type;
                typedef ValueType*                                  pointer;
                typedef ValueType&                                  reference;

                /// Default constructor
                CListIterator() : base_type()
                {
                    // intentionally left blank
                }

                /// copy constructor
                CListIterator(const CListIterator& f_iter_r);

                /// Returns a reference to the object.
                reference operator*() const;

                /// Returns a pointer to the object.
                pointer operator->() const ;

                /// Prefix increment iterator
                self_type& operator++() ;

                /// Post increment iterator.
                // (Msg Disable 2427 : Direct use of fundamental type.)
                // PRQA S 2427 ++
                self_type operator++(int);
                // PRQA S 2427 --
                // (Msg Enable 2427 : Direct use of fundamental type.)

                /// Prefix decrement iterator
                self_type& operator--();

                /// Postfix decrement iterator.
                // (Msg Disable 2427 : Direct use of fundamental type.)
                // PRQA S 2427 ++
                self_type operator--(int);
                // PRQA S 2427 --
                // (Msg Enable 2427 : Direct use of fundamental type.)

                /// Assignment operator
                const CListIterator& operator= (const CListIterator& f_iter_r);

            private:

                explicit CListIterator(const base_type& f_iter_r);

                explicit CListIterator(position_type* f_pos_p) : base_type(f_pos_p)
                {
                    //intentionally left blank
                }

                friend class    TList ;             //! TList class can access private members
        };



    public:

        /// A type that provides a pointer to an element in a list.
        typedef typename allocator_type::pointer                            pointer;
        /// A type that provides a pointer to a const element in a list.
        typedef typename allocator_type::const_pointer                      const_pointer;
        /// A type that provides a reference to a element stored in a list for reading
        /// and performing operations.
        typedef typename allocator_type::reference                          reference;
        /// A type that provides a reference to a const element stored in a list for
        /// reading and performing const operations.
        typedef typename allocator_type::const_reference                    const_reference;
        /// A type that provides a bidirectional iterator that can read a const element in a list.
        typedef CListConstIterator                                          const_iterator;
        /// A type that provides a bidirectional iterator that can read or modify
        /// any element in a list.
        typedef CListIterator                                               iterator;
        /// A type that provides a bidirectional iterator that can read any const element
        /// in a list.
        typedef stlalias::reverse_iterator<const_iterator>                  const_reverse_iterator;
        /// A type that provides a bidirectional iterator that can read or modify an element in
        /// a reversed list.
        typedef stlalias::reverse_iterator<iterator>                        reverse_iterator;


    public:

        //---------------------------------------------------------------------
        //! Default constructor.
        //! $Source: vfc_list.hpp $
        //! @author dkn2kor
        //---------------------------------------------------------------------
        TList(void);


        //---------------------------------------------------------------------
        //! Creates the list with a specific allocator.
        //! $Source: vfc_list.hpp $
        //! @author dkn2kor
        //---------------------------------------------------------------------
        explicit TList(const nodealloc_type& f_alloc_r);

        //---------------------------------------------------------------------
        //! Creates a list with default elements.
        //! Constructor fills the list with the specified number
        //! copies of default-constructed element.
        //! $Source: vfc_list.hpp $
        //! @param  f_count     The number of elements to initially create.
        //! @author dkn2kor
        //---------------------------------------------------------------------
        explicit TList(size_type f_count);

        //-----------------------------------------------------------------------------
        //! Creates a list with specified element value.
        //! Constructor fills the list with the specified number
        //! copies of element with the given value.
        //! $Source: vfc_list.hpp $
        //! @param  f_count     The number of elements to initially create.
        //! @param  f_value_r   value to be filled.
        //! @author dkn2kor
        //=============================================================================
        TList(size_type f_count, const value_type& f_value_r);

        //-----------------------------------------------------------------------------
        //! Creates a list with specified element value.
        //! Constructor fills the list with the specified number
        //! copies of element with the given value.
        //! $Source: vfc_list.hpp $
        //! @param  f_count     The number of elements to initially create.
        //! @param  f_value_r   value to be filled.
        //! @param  f_alloc_r   node type allocator.
        //! @author vmr1kor
        //=============================================================================
        TList(size_type f_count, const value_type& f_value_r, const nodealloc_type& f_alloc_r);

        //-----------------------------------------------------------------------------
        //! Copy constructor.
        //! $Source: vfc_list.hpp $
        //! @param  f_rhs_r  A list with identical element type, size value and allocator type.
        //! @author dkn2kor
        //=============================================================================
        TList(const list_type& f_rhs_r);

        //-----------------------------------------------------------------------------
        //! Builds a list from the given range.
        //! Create a list consisting of copies of the elements in the other list.
        //! $Source: vfc_list.hpp $
        //! @param  f_first  Position of the first element in the range of elements to be copied.
        //! @param  f_last   Position of the first element beyond the range of elements to be copied.
        //! @author dkn2kor
        //=============================================================================
        template<class InIteratorType>
        TList(InIteratorType f_first, InIteratorType f_last);

        //-----------------------------------------------------------------------------
        //! Builds a list from the given range.
        //! Create a list consisting of copies of the elements in the other list.
        //! $Source: vfc_list.hpp $
        //! @param  f_first  Position of the first element in the range of elements to be copied.
        //! @param  f_last   Position of the first element beyond the range of elements to be copied.
        //! @param  f_alloc_r   node type allocator.
        //! @author dkn2kor
        //=============================================================================
        template<class InIteratorType>
        TList(InIteratorType f_first, InIteratorType f_last, const nodealloc_type& f_alloc_r);

        //-----------------------------------------------------------------------------
        //! Assignment operator.
        //! $Source: vfc_list.hpp $
        //! @param  f_rhs_r  A list with identical element type, size value and allocator type.
        //! @author dkn2kor
        //=============================================================================
        const list_type& operator=(const list_type& f_rhs_r);

        //-----------------------------------------------------------------------------
        //! Destructor.
        //! $Source: vfc_list.hpp $
        //! @author dkn2kor
        //=============================================================================
        ~TList();

        ///////////////////////////////////////////////////////////////////////////////////////////////
        // iterator funcs
        ///////////////////////////////////////////////////////////////////////////////////////////////

        //-----------------------------------------------------------------------------
        //! Returns an iterator addressing the first element in a list.
        //! $Source: vfc_list.hpp $
        //! @return Returns a bidirectional iterator addressing the first element in the list
        //!         or to the location succeeding an empty list.
        //! @author dkn2kor
        //=============================================================================
        iterator begin();

        //-----------------------------------------------------------------------------
        //! Returns a const iterator addressing the first element in a list..
        //! $Source: vfc_list.hpp $
        //! @return Returns a bidirectional const iterator addressing the first element in the list
        //!         or to the location succeeding an empty list
        //! @author dkn2kor
        //=============================================================================
        const_iterator begin() const;

        //-----------------------------------------------------------------------------
        //! Returns an iterator that addresses the location succeeding the last element in a list.
        //! $Source: vfc_list.hpp $
        //! @return Returns a bidirectional iterator that addresses the location succeeding the last
        //!         element in a list. If the list is empty, then list::end == list::begin.
        //! @author dkn2kor
        //=============================================================================
        iterator end();

        //-----------------------------------------------------------------------------
        //! Returns a const iterator that addresses the location succeeding the last element in a list.
        //! @note If the list is empty, then list.end() == list.begin().
        //! $Source: vfc_list.hpp $
        //! @return Returns a const bidirectional iterator that addresses the location succeeding the last
        //!         element in a list. If the list is empty, then list::end == list::begin.
        //! @author dkn2kor
        //=============================================================================
        const_iterator end() const;

        //-----------------------------------------------------------------------------
        //! Returns an iterator addressing the first element in a reversed list.
        //! $Source: vfc_list.hpp $
        //! @return Returns a reverse bidirectional iterator addressing the first element in a
        //!         reversed list (or addressing what had been the last element in the unreversed list).
        //! @author dkn2kor
        //=============================================================================
        reverse_iterator rbegin();

        //-----------------------------------------------------------------------------
        //! Returns a const iterator addressing the first element in a reversed list.
        //! $Source: vfc_list.hpp $
        //! @return Returns a const reverse bidirectional iterator addressing the first element in a
        //!         reversed list (or addressing what had been the last element in the unreversed list).
        //! @author dkn2kor
        //=============================================================================
        const_reverse_iterator rbegin() const;

        //-----------------------------------------------------------------------------
        //! Returns an iterator that addresses the location succeeding the last element in a reversed list.
        //! $Source: vfc_list.hpp $
        //! @return Returns a reverse bidirectional iterator that addresses the location succeeding the
        //!         last element in a reversed list (the location that had preceded the first
        //!         element in the unreversed list).
        //! @author dkn2kor
        //=============================================================================
        reverse_iterator rend();

        //-----------------------------------------------------------------------------
        //! Returns a const iterator that addresses the location succeeding the last element in a reversed list.
        //! $Source: vfc_list.hpp $
        //! @return Returns a const reverse bidirectional iterator that addresses the location succeeding the
        //!         last element in a reversed list (the location that had preceded the first
        //!         element in the unreversed list).
        //! @author dkn2kor
        //=============================================================================
        const_reverse_iterator rend() const;

        ///////////////////////////////////////////////////////////////////////////////////////////////
        // size related funcs
        ///////////////////////////////////////////////////////////////////////////////////////////////

        //-----------------------------------------------------------------------------
        //! Tests if a list is empty.
        //! $Source: vfc_list.hpp $
        //! @return Retruns true if the list is empty; false if the list is not empty.
        //! @author dkn2kor
        //=============================================================================
        bool empty() const;

        //-----------------------------------------------------------------------------
        //! Returns the number of elements in the list.
        //! $Source: vfc_list.hpp $
        //! @return Returns the number of elements in the list.
        //! @author dkn2kor
        //=============================================================================
        size_type size() const;

        //-----------------------------------------------------------------------------
        //! Returns the length of the longest sequence that the object can control.
        //! $Source: vfc_list.hpp $
        //! @return Returns the length of the longest sequence that the object can control.
        //! @author dkn2kor
        //=============================================================================
        size_type max_size() const;

        //-----------------------------------------------------------------------------
        //! Returns the length of the longest sequence that the object can control.
        //! $Source: vfc_list.hpp $
        //! @return Returns the length of the longest sequence that the object can control.
        //! @author dkn2kor
        //=============================================================================
        size_type capacity() const;

        //-----------------------------------------------------------------------------
        //! Specifies a new size for a list.
        //! $Source: vfc_list.hpp $
        //! @param  f_newSize   The new size of the list.
        //! @param  f_default   The value of the new elements to be added to the list.
        //! @note   The value of the new elements will be added if the new size is larger
        //!         that the original size
        //! @author dkn2kor
        //=============================================================================
        void resize(size_type f_newSize, const value_type& f_default);

        //-----------------------------------------------------------------------------
        //! Specifies a new size for a list.
        //! $Source: vfc_list.hpp $
        //! @param  f_newSize   The new size of the list.
        //! @author dkn2kor
        //=============================================================================
        void resize(size_type f_newSize);

        ///////////////////////////////////////////////////////////////////////////////////////////////
        // funcs
        ///////////////////////////////////////////////////////////////////////////////////////////////

        //-----------------------------------------------------------------------------
        //! Returns a copy of the allocator object used to construct a list.
        //! $Source: vfc_list.hpp $
        //! @return The allocator used by the list.
        //! @author dkn2kor
        //=============================================================================
        allocator_type get_allocator() const;

        //-----------------------------------------------------------------------------
        //! Adds an element to the end of a list.
        //! It increases the list size by one.
        //! The function creates an element at the end of the list and
        //! assigns the given data to it. If the list is full, push_back does
        //! not do anything but simply returns.
        //! $Source: vfc_list.hpp $
        //! @param  f_value_r  The element added to the end of the list.
        //! @author dkn2kor
        //=============================================================================
        void push_back(const value_type& f_value_r);

#if __cplusplus >= 201103L
        //! Constructs an element to the end of a list.
        //! It increases the list size by one.
        //! The function creates an element at the end of the list by calling its
        //! constructor with the arguments given to this function. If the list is
        //! full, the behavior of emplace_back is undefined.
        //! $Source: vfc_list.hpp $
        //! @param   f_args   arguments to forward to the constructor of the element
        //! @return  reference to the constructed element
        //! @author bet1pal
        template<class... Args>
        reference emplace_back(Args&&... f_args);
#endif

        //-----------------------------------------------------------------------------
        //! Adds an UNINITIALIZED element to the end of a list.
        //! If the (TFixed...) list is full, insert_uninitialized_back does not do anything but simply returns.
        //! Use in conjunction with placement new, if you have list elements that
        //! are very costly to copy and you still want to use a value container:
        //! @code new (&*myList.insert_uninitialized_back()) myElement(ctorArgs); @endcode
        //! $Source: vfc_list.hpp $
        //! @author muk2lr
        //=============================================================================
        iterator insert_uninitialized_back();

        //-----------------------------------------------------------------------------
        //! Deletes the element at the end of a list.
        //! It reduces the list by one.
        //! Note that no data is returned, and if the last element's data
        //! is needed, it should be retrieved before pop_back() is called.
        //! If the list is empty, pop_back does not do anything but simply returns.
        //! $Source: vfc_list.hpp $
        //! @author dkn2kor
        //=============================================================================
        void pop_back();

        //-----------------------------------------------------------------------------
        //! Adds an element to the beginning of a list.
        //! It increases the list by one.
        //! The function creates an element at the front of the list and
        //! assigns the given data to it. If the list is full, push_front does
        //! not do anything but simply returns.
        //! $Source: vfc_list.hpp $
        //! @param  f_value_r  The element added to the begining of the list.
        //! @author dkn2kor
        //=============================================================================
        void push_front(const value_type& f_value_r);

        //-----------------------------------------------------------------------------
        //! Adds an UNINITIALIZED element to the beginning of a list.
        //! If the (TFixed...) list is full, insert_uninitialized_front does not do anything but simply returns.
        //! Use in conjunction with placement new, if you have list elements that
        //! are very costly to copy and you still want to use a value container:
        //! @code new (&*myList.insert_uninitialized_front()) myElement(ctorArgs); @endcode
        //! $Source: vfc_list.hpp $
        //! @author muk2lr
        //=============================================================================
        iterator insert_uninitialized_front();

        //-----------------------------------------------------------------------------
        //! Deletes the element at the begining of a list.
        //! It reduces the list by one.
        //! Note that no data is returned, and if the first element's data
        //! is needed, it should be retrieved before pop_front() is called.
        //! If the list is empty, pop_front does not do anything but simply returns.
        //! $Source: vfc_list.hpp $
        //! @author dkn2kor
        //=============================================================================
        void pop_front();

        //-----------------------------------------------------------------------------
        //! Inserts a element into list before specified position.
        //! $Source: vfc_list.hpp $
        //! @param  f_pos  The position in the list where the first element is inserted.
        //! @param  f_value_r  The value of the element being inserted into the list.
        //! @return returns an iterator that points to the position where the new element was inserted.
        //! @author dkn2kor
        //=============================================================================
        iterator insert(iterator f_pos, const value_type& f_value_r);

        //-----------------------------------------------------------------------------
        //! Inserts a number of elements into the list before specified position.
        //! $Source: vfc_list.hpp $
        //! @param  f_pos  The position in the list where the first element is inserted.
        //! @param  f_num  Number of elements to be inserted.
        //! @param  f_value_r  The value of the element being inserted into the list.
        //! @author dkn2kor
        //=============================================================================
        void insert(iterator f_pos, size_type f_count, const value_type& f_value_r);

        //-----------------------------------------------------------------------------
        //! Inserts a range of elements into the list before specified position.
        //! $Source: vfc_list.hpp $
        //! @param  f_pos  The position in the list where the first element is inserted.
        //! @param  f_first  Position of the first element in the range of elements in the
        //!                  argument list to be copied.
        //! @param  f_last   Position of the first element beyond the range of elements in
        //!                  the argument list to be copied.
        //! @author dkn2kor
        //=============================================================================
        template<class InIteratorType>
        void insert(iterator f_pos, InIteratorType f_first, InIteratorType f_last);

        //-----------------------------------------------------------------------------
        //! Inserts an UNINITIALIZED element after the given position.
        //! If the (TFixed...) list is full, insert_uninitialized simply returns.
        //! Use in conjunction with placement new, if you have list elements that
        //! are very costly to copy and you still want to use a value container:
        //! @code new (&(*myList.insert_uninitialized())) myElement(ctorArgs); @endcode
        //! $Source: vfc_list.hpp $
        //! @author muk2lr
        //=============================================================================
        iterator insert_uninitialized(iterator f_pos);

        //-----------------------------------------------------------------------------
        //! Assigns a given value to a list.
        //! Function fills a list with the specified number of copies
        //! Note that the assignment completely changes the list
        //! and that the resulting list's size is the same as the count
        //! $Source: vfc_list.hpp $
        //! @param  f_count  The number of copies of an element being inserted into the list.
        //! @param  f_value_r  The value of the element being inserted into the list.
        //! @author dkn2kor
        //=============================================================================
        void assign(size_type f_count, const value_type& f_value_r);

        //-----------------------------------------------------------------------------
        //! Assigns a range to a list.
        //! Function fills a list with the specified range
        //! Note that the assignment completely changes the list and
        //! that the resulting list's size is the same as the number of
        //! elements assigned.  Old data is lost.
        //! $Source: vfc_list.hpp $
        //! @param  f_first  Position of the first element in the range of elements to be copied
        //!                  from the argument list.
        //! @param  f_last   Position of the first element just beyond the range of elements to be copied from the argument list.
        //! @author dkn2kor
        //=============================================================================
        template<typename InIteratorType>
        void assign(InIteratorType f_first, InIteratorType f_last);

        //-----------------------------------------------------------------------------
        //! Removes an element in a list from specified position.
        //! beyond any elements removed, or a pointer to the end of the list if no such element exists.
        //! Function will erase the element at the given position and thus
        //! shorten the list by one.
        //! The user is also cautioned that this function only erases the element,
        //! and that if the element is itself a pointer, the pointed-to memory is not touched
        //! Managing the pointer is the user's responsibilty.
        //! $Source: vfc_list.hpp $
        //! @param  f_pos  Position of the element to be removed from the list.
        //! @return  A bidirectional iterator that designates the first element remaining
        //! @author dkn2kor
        //=============================================================================
        iterator erase(iterator f_pos);

        //-----------------------------------------------------------------------------
        //! Removes a range of elements in a list from specified position.
        //! beyond any elements removed, or a pointer to the end of the list if no such element exists.
        //! Function will erase the element at the given position and thus
        //! shorten the list by one.
        //! Function will erase the specified range of elements
        //! and shorten the list accordingly.
        //! The user is also cautioned that this function only erases the elements, and that if the
        //! elements themselves are pointers, the pointed-to memory is not
        //! touched .  Managing the pointer is the user's responsibilty.
        //! $Source: vfc_list.hpp $
        //! @param  f_first  Position of the first element removed from the list.
        //! @param  f_last  Position just beyond the last element removed from the list.
        //! @return  A bidirectional iterator that designates the first element remaining
        //! @author dkn2kor
        //=============================================================================
        iterator erase(iterator f_first, iterator f_last);

        //-----------------------------------------------------------------------------
        //! Erases elements in a list that match a specified value.
        //! The order of the elements remaining is not affected.
        //! Note that this function only erases the elements, and
        //! that if the elements themselves are pointers, the  pointed-to memory
        //! is not touched. Managing the pointer is the user's responsibilty.
        //! $Source: vfc_list.hpp $
        //! @param  f_value_r  The value which, if held by an element,
        //! will result in that element's removal from the list.
        //! @author dkn2kor
        //=============================================================================
        void remove(const value_type& f_value_r);

        //-----------------------------------------------------------------------------
        //! Erases elements from a list for which a specified predicate is satisfied.
        //! Erases every element in the list for which the predicate
        //! returns true.  The order of the elements remaining is not affected.
        //! Note that this function only erases the elements, and that if the
        //! elements themselves are pointers, the pointed-to memory is
        //! not touched .  Managing the pointer is the user's responsibilty.
        //! $Source: vfc_list.hpp $
        //! @param  f_predicate The unary predicate which, if satisfied by an
        //! element, results in the deletion of that element from the list.
        //! @author dkn2kor
        //=============================================================================
        template<class PredicateType>
        void remove_if(PredicateType f_predicate);

        //-----------------------------------------------------------------------------
        //! Reverses the order in which the elements occur in a list.
        //! $Source: vfc_list.hpp $
        //! @author dkn2kor
        //=============================================================================
        void reverse();

        //-----------------------------------------------------------------------------
        //! Exchanges the elements of two lists.
        //! $Source: vfc_list.hpp $
        //! @param  f_rhs_r  A list of the same element and allocator types.
        //! @author dkn2kor
        //=============================================================================
        void swap(list_type& f_rhs_r);

        //-----------------------------------------------------------------------------
        //! Removes adjacent elements that are equal in the list.
        //! For each consecutive set of elements with the same value,
        //! remove all but the first one.  Remaining elements stay in
        //! list order.  Note that this function only erases the
        //! elements, and that if the elements themselves are pointers,
        //! the pointed-to memory is not touched in any way.  Managing
        //! the pointer is the user's responsibilty.
        //! $Source: vfc_list.hpp $
        //! @author dkn2kor
        //=============================================================================
        void unique();

        //-----------------------------------------------------------------------------
        //! Removes adjacent elements that satisfy some other binary predicate from the list.
        //! This function assumes that the list is sorted, so that all duplicate elements
        //! are adjacent. Duplicates that are not adjacent will not be deleted. Remaining
        //! elements stay in list order.
        //! Note that this function only erases the elements, and that if the
        //! elements themselves are pointers, the pointed-to memory is not
        //! touched in any way. Managing the pointer is the user's responsibilty.
        //! $Source: vfc_list.hpp $
        //! @param  f_binaryPredicate  The binary predicate used to compare successive elements.
        //! @author dkn2kor
        //=============================================================================
        template<class BinaryPredicateType>
        void unique(BinaryPredicateType f_binaryPredicate);

        //-----------------------------------------------------------------------------
        //! Removes all elements from the argument list and inserts them into this list.
        //! The elements of given list are inserted in front of the element specified.
        //! The source list becomes an empty list at the end of the operation.
        //! Note If the number of elements to spliced exceed the capacity of the list,
        //! then the function leaves both the lists unaltered
        //! $Source: vfc_list.hpp $
        //! @param  f_pos  The position in the target list before which the elements of the
        //! argument list are to be inserted.
        //! @param  f_list_r  The argument list that is to be inserted into this list.
        //! @author dkn2kor
        //=============================================================================
        void splice(iterator f_pos, list_type& f_list_r);

        //-----------------------------------------------------------------------------
        //! Removes one element from the argument list and inserts them into this list.
        //! Removes the element in given list referenced by iterator and
        //! inserts it into the current list before specified position.
        //! Note If the number of elements to spliced exceed the capacity of the list,
        //! then the function leaves both the lists unaltered
        //! $Source: vfc_list.hpp $
        //! @param  f_pos  The position in the target list before which the elements of the
        //! argument list are to be inserted.
        //! @param  f_list_r  The argument list that is to be inserted into this list.
        //! @param  f_iter  Iterator referencing the element to be moved.
        //! @author dkn2kor
        //=============================================================================
        void splice(iterator f_pos, list_type& f_list_r, iterator f_iter);

        //-----------------------------------------------------------------------------
        //! Removes a range of elements from the argument list and inserts them into this list.
        //! Removes elements in the given range and inserts them before position specified position.
        //! Note if f_pos is in between the specified f_first & f_last the
        //! output cannot be predicted.
        //! Note If the number of elements to spliced exceed the capacity of the list,
        //! then the function leaves both the lists unaltered.
        //! $Source: vfc_list.hpp $
        //! @param  f_pos  The position in the target list before which the elements of the
        //! argument list are to be inserted.
        //! @param  f_list_r  The argument list that is to be inserted into this list.
        //! @param  f_first  Iterator referencing the start of range in the argument list.
        //! @param  f_last  Iterator referencing the end of range in the argument list.
        //! @author dkn2kor
        //=============================================================================
        void splice(iterator f_pos, list_type& f_list_r, iterator f_first,
            iterator f_last);

        //-----------------------------------------------------------------------------
        //! Removes the elements from the argument list, inserts them into the target list,
        //! and orders the new, combined set of elements in ascending order.
        //! Assumes that both lists are sorted according to operator < ().
        //! Merges elements of f_list_r into this list in sorted order,
        //! leaving f_list_r empty when complete. Elements in this list precede elements in
        //! f_list_r they are equal.
        //! Note If the number of elements to merged exceed the capacity of the list,
        //! then the function leaves both the lists unaltered.
        //! $Source: vfc_list.hpp $
        //! @param  f_list_r  Sorted list to merge.
        //! @author dkn2kor
        //=============================================================================
        void merge(list_type& f_list_r);

        //-----------------------------------------------------------------------------
        //! Removes the elements from the argument list, inserts them into the target list,
        //! and orders the new, combined set of elements in some other specified order.
        //! Assumes that both the lists are sorted according to
        //! defined Comparison function.  Merges elements of f_list_r
        //! into this list in sorted order, leaving f_list_r empty when complete.
        //! Elements in this list precede elements in f_list_r that are equivalent
        //! according to the Comparison function.
        //! Note If the number of elements to merged exceed the capacity of the list,
        //! then the function leaves both the lists unaltered.
        //! $Source: vfc_list.hpp $
        //! @param  f_list_r  Sorted list to merge.
        //! @param  f_orderType Comparison function definining sort order.
        //! @author dkn2kor
        //=============================================================================
        template<class OrderingType>
        void merge(list_type& f_list_r, OrderingType f_orderType);

        //-----------------------------------------------------------------------------
        //! Arranges the elements of a list in ascending order.
        //! Equivalent elements remain in list order.
        //! Inserstion sort technique used.
        //! @note   Time Complexity of the sort function (O (N2))
        //! $Source: vfc_list.hpp $
        //! @author dkn2kor
        //! @ingroup vfc_group_containers
        //=============================================================================
        void sort();

        //-----------------------------------------------------------------------------
        //! Arranges the elements of a list with respect to some
        //! user-specified order relation.
        //! Equivalent elements remain in list order.
        //! Inserstion sort technique used.
        //! $Source: vfc_list.hpp $
        //! @author dkn2kor
        //=============================================================================
        template <class ComparisonPredicateType>
        void sort(ComparisonPredicateType f_predicate);

        //-----------------------------------------------------------------------------
        //! Erases all the elements of a list.
        //! Note that this function only erases the elements,
        //! and that if the elements themselves are pointers,
        //! Managing the pointer is the user's responsibilty.
        //! $Source: vfc_list.hpp $
        //! @author dkn2kor
        //=============================================================================
        void clear();

///////////////////////////////////////////////////////////////////////////////////////////////
// element access
///////////////////////////////////////////////////////////////////////////////////////////////

        //-----------------------------------------------------------------------------
        //! Returns a reference to the first element in a list.
        //! $Source: vfc_list.hpp $
        //! @return     Returns a reference to the first element in a list,
        //! If the list is empty, the return is undefined.
        //! @author dkn2kor
        //=============================================================================
        reference front();

        //-----------------------------------------------------------------------------
        //! Returns a constant reference to the first element in a list
        //! $Source: vfc_list.hpp $
        //! @return     Returns a constant reference to the first element in a list.
        //! If the list is empty, the return is undefined
        //! @author dkn2kor
        //=============================================================================
        const_reference front() const;

        //-----------------------------------------------------------------------------
        //! Returns a reference to the last element of a list.
        //! $Source: vfc_list.hpp $
        //! @return     Returns a reference to the last element of a list.
        //! If the list is empty, the return value is undefined.
        //! @author dkn2kor
        //=============================================================================
        reference back();

        //-----------------------------------------------------------------------------
        //! constant reference to the last element of a list
        //! $Source: vfc_list.hpp $
        //! @return     Returns a constant reference to the last element of a list.
        //! If the list is empty, the return value is undefined.
        //! @author dkn2kor
        //=============================================================================
        const_reference back() const;

    private:

        //-----------------------------------------------------------------------------
        /// function creates a node with the specified value and inserts the
        /// node and returns pointer to CNode else returns a zero CNode pointer
        /// if the list is full.
        //! $Source: vfc_list.hpp $
        //! @param f_pos                iterator of the inserted element.
        //! @param f_value_p            points to element that will be inserted or 0,
        //!                             if you want to insert an uninitialized element
        //!                             that needs to be constructed by the user!
        /// @return return a pointer to newly created node.
        /// @author dkn2kor
        //=============================================================================
        node_type* insertElement(iterator f_pos, const value_type* f_value_p);

        //-----------------------------------------------------------------------------
        /// function creates an unitialized node with the specified value and inserts the
        /// node and returns pointer to CNode else returns a zero CNode pointer
        /// if the list is full.
        //! $Source: vfc_list.hpp $
        //! @param f_pos                iterator of the inserted element.
        /// @return return a pointer to newly created node.
        /// @author dkn2kor
        //=============================================================================
        node_type* insertElement(iterator f_pos);
        
        /// function removed the specified element from the list and relinks the chain
        position_type* removeElement(position_type* f_node_p);

        /// function removed the specified range of elements from the list and relinks the chain
        position_type* removeRange(position_type* f_startNode_p,
                                   position_type* f_endNode_p);

        // reverse for PODs
        void reverse(true_t);

        /// reverse for UDTs (Non PODs)
        void reverse(false_t);

        /// Swap functionality for node wise swapping
        void swapNodes(list_type& f_rhs_r);

        /// Swap functionality for swapping values
        void swapData(list_type& f_rhs_r);

        //-----------------------------------------------------------------------------
        //! sequence from f_first to one before f_last is inserted before f_pos.
        //! $Source: vfc_list.hpp $
        //! @param f_pos                position to be inserted.
        //! @param f_first              position of the first element to be relinked.
        //! @param f_last               position just beyond the last element to be relinked.
        /// @author dkn2kor
        //=============================================================================
        void relinkNode(iterator f_pos, iterator f_first, iterator f_last);

    private:

        size_type           m_numElements;          //!< Counter: how much elements are actually contained
        nodealloc_type      m_nodeAlloc;
        position_type       m_head;
    };

    //-----------------------------------------------------------------------------
    //! List equality comparison.
    //! This is an equivalence relation.  It is linear in the size of
    //! the lists.  Lists are considered equivalent if their sizes are
    //! equal, and if corresponding elements compare equal.
    //! $Source: vfc_list.hpp $
    //! @param  f_list1_r  list.
    //! @param  f_list2_r  list of the same type as f_list1_r.
    //! @return  True if the size and elements of the lists are equal.
    //! @author dkn2kor
    //! @ingroup vfc_containers
    //=============================================================================
    template<class ValueType, class AllocatorType, class MaxSizePolicyType>
    bool operator==(const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list1_r,
        const TList<ValueType,AllocatorType,MaxSizePolicyType>& f_list2_r);

    //-----------------------------------------------------------------------------
    //! List ordering relation.
    //! This is a total ordering relation.  It is linear in the size of the
    //! lists.  The elements must be comparable with <.
    //! $Source: vfc_list.hpp $
    //! @param  f_list1_r  list.
    //! @param  f_list2_r  list of the same type as f_list1_r.
    //! @return  True if f_list1_r is lexicographically less than f_list2_r.
    //! @author dkn2kor
    //! @ingroup vfc_containers
    //=============================================================================
    template<class ValueType, class AllocatorType, class MaxSizePolicyType>
    bool operator<(const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list1_r,
        const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list2_r);


    //-----------------------------------------------------------------------------
    //! List in equality comparison.
    //! This is an equivalence relation.  It is linear in the size of
    //! the lists.  Lists are considered equivalent if their sizes are
    //! equal, and if corresponding elements compare equal.
    //! $Source: vfc_list.hpp $
    //! @param  f_list1_r  list.
    //! @param  f_list2_r  list of the same type as f_list1_r.
    //! @return  False if the size and elements of the lists are equal.
    //! @author dkn2kor
    //! @ingroup vfc_containers
    //=============================================================================
    template<class ValueType, class AllocatorType, class MaxSizePolicyType>
    bool operator!=(const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list1_r,
        const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list2_r);

    //-----------------------------------------------------------------------------
    //! List ordering relation.
    //! This is a total ordering relation.  It is linear in the size of the
    //! lists.  The elements must be comparable with >.
    //! $Source: vfc_list.hpp $
    //! @param  f_list1_r  list.
    //! @param  f_list2_r  list of the same type as f_list1_r.
    //! @return  True if f_list1_r is lexicographically greater than f_list2_r.
    //! @author dkn2kor
    //! @ingroup vfc_containers
    //=============================================================================
    template<class ValueType, class AllocatorType, class MaxSizePolicyType>
    bool operator>(const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list1_r,
        const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list2_r);


    //-----------------------------------------------------------------------------
    //! List ordering relation.
    //! This is a total ordering relation.  It is linear in the size of the
    //! lists.  The elements must be comparable with <=.
    //! $Source: vfc_list.hpp $
    //! @param  f_list1_r  list.
    //! @param  f_list2_r  list of the same type as f_list1_r.
    //! @return  True if f_list1_r is lexicographically less than or equal to f_list2_r.
    //! @author dkn2kor
    //! @ingroup vfc_containers
    //=============================================================================
    template<class ValueType, class AllocatorType, class MaxSizePolicyType>
    bool operator<=(const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list1_r,
        const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list2_r);


    //-----------------------------------------------------------------------------
    //! List ordering relation.
    //! This is a total ordering relation.  It is linear in the size of the
    //! lists.  The elements must be comparable with >=.
    //! $Source: vfc_list.hpp $
    //! @param  f_list1_r  list.
    //! @param  f_list2_r  list of the same type as f_list1_r.
    //! @return  True if f_list1_r is lexicographically greater than or equal to f_list2_r.
    //! @author dkn2kor
    //! @ingroup vfc_containers
    //=============================================================================
    template<class ValueType, class AllocatorType, class MaxSizePolicyType>
    bool operator>=(const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list1_r,
        const TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list2_r);


    //---------------------------------------------------------------------
    //! Swaps exchanges the elements of two lists f_list1_r & f_list2_r.
    //! $Source: vfc_list.hpp $
    //! @param  f_list1_r    The list providing the elements to be swapped, or the
    //!                      list whose elements are to be exchanged with those of the list f_list2_r.
    //! @param  f_list2_r    A list whose elements are to be exchanged with those of the list f_list1_r.
    //! @author dkn2kor
    //! @ingroup vfc_containers
    //---------------------------------------------------------------------
    template<class ValueType, class AllocatorType, class MaxSizePolicyType>
    void swap(TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list1_r,
        TList<ValueType, AllocatorType,MaxSizePolicyType>& f_list2_r);

}   // namespace vfc closed

#include "vfc/container/vfc_list.inl"

#endif //VFC_LIST_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_list.hpp  $
//  Revision 1.10 2014/11/21 12:36:46MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - vfc_list: CNodeBase Coding Rule violation: performs shallow copy on its pointer members (mantis0004738)
//  Revision 1.9 2014/08/18 16:53:43MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - TList::CNode::CNode() unnecessary (mantis0004442)
//  Revision 1.8 2014/05/16 14:00:26MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Add TList member function to create uninitialized entry (mantis0003517
//  Revision 1.7 2014/05/16 13:49:35MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Add TList member function to create uninitialized entry (mantis0003517)
//  Revision 1.6 2014/05/09 15:47:01MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Add TList member function to create uninitialized entry (mantis0003517)
//  Revision 1.5 2014/05/09 14:53:07MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Add TList member function to create uninitialized entry (mantis0003517)
//  Revision 1.4 2010/08/12 08:17:03MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  -  Add the vfc_list proposal to the main trunk (merged missing includes) (mantis0001793)
//  Revision 1.3 2010/08/12 08:16:03MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  -  Add the vfc_list proposal to the main trunk (mantis0001793)
//  Revision 1.1.1.2 2010/08/11 17:44:19MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/container/container.pj
//  Revision 1.14 2010/05/06 14:17:48MESZ Vinaykumar Setty (RBEI/ESB2) (vmr1kor) 
//  - linker error in GHS compiler problem resolved ( mantis :- 3275 )
//  Revision 1.13 2009/02/05 14:34:16IST Gaurav Jain (RBEI/EAS3) (gaj2kor) 
//  -Removal of QAC++ warnings.
//  (Mantis : 0002563)
//  Revision 1.12 2008/09/30 15:47:12IST Vinaykumar Setty (RBEI/EAC1) (vmr1kor)
//  - review comments are implemented ( mantis id:- 1793 )
//  Revision 1.11 2008/09/11 17:59:42IST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  -Addition of capacity() function
//  (Mantis : 2080)
//  Revision 1.10 2008/09/01 16:02:38IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  - CList review comments are incorporated( Mantis id :- 1793)
//  - Linker error got fixed ( Mantis id :- 2309)
//  Revision 1.9 2008/02/05 16:04:46IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  CListIterator is Inheriting from CListConstIterator
//  Few Performance issues are addressed
//  doxyComments are added
//  Revision 1.8 2007/10/31 16:55:11IST vmr1kor
//  split the default ctor into two: "TList(void)" and "explicit TList(nodealloc_type)
//  Implemented resize() function
//  Mantis Id :- 0001793
//  Revision 1.7 2007/09/27 18:27:02IST vmr1kor
//  != , == operators added for TListCostIterators
//  removed getIterator,getConstIterator,getNode
//  QACPP error got fixed
//  Revision 1.6 2007/09/24 15:00:07IST vmr1kor
//  - moved classes CNodeBase , CNode , CListIterator , CListConstIterator inside TList
//  - Mantis id - 0001385
//  Revision 1.5 2007/09/03 18:48:09IST vmr1kor
//  operator = () function added
//  Revision 1.4 2007/06/18 13:18:34IST dkn2kor
//  - updated list to work with FixedMemPool Allocator (mantis1385)
//  Revision 1.3 2007/03/26 19:50:11IST dkn2kor
//  - updated splice and remove element
//=============================================================================

