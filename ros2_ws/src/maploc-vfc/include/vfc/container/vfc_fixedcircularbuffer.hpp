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
///     $Source: vfc_fixedcircularbuffer.hpp $
///     $Revision: 1.5 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/12/08 15:57:12MEZ $
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

#ifndef VFC_FIXEDCIRCULARBUFFER_HPP_INCLUDED
#define VFC_FIXEDCIRCULARBUFFER_HPP_INCLUDED

#include <iterator>                                 //    used for bidirectional_iterator_tag

#include "vfc/core/vfc_types.hpp"
#include "vfc/memory/vfc_fixedblock_allocator.hpp"  // used for default AllocatorType

namespace vfc 
{
    //=============================================================================
    //  TFixedCircularBuffer
    //-----------------------------------------------------------------------------
    //! TFixedCircularBuffer refers to an area in memory which is used to store incoming data. 
    //! When the buffer is filled, new data is written starting at the beginning of the buffer and 
    //! overwriting the old.The TFixedCircularBuffer is especially designed to provide fixed capacity storage.
    //! When its capacity is exhausted, newly inserted elements will cause elements either at the beginning
    //! to be overwritten.The TFixedCircularBuffer only allocates memory when created , capacity can be is 
    //! adjusted explicitly. Implemented in FIFO Manner.
    //! $Source: vfc_fixedcircularbuffer.hpp $
    //! @param ValueType        DataType to be stored
    //! @param CapacityValue    Capacity of the TFixedCircularBuffer
    //! @param AllocatorType    Allocator type used
    //! @author                 vmr1kor
    //! @ingroup                vfc_containers
    //=============================================================================

    template<class ValueType,
             vfc::int32_t CapacityValue,
             class AllocatorType = vfc::TFixedBlockAllocator<ValueType, CapacityValue> >
    class TFixedCircularBuffer
    {
        VFC_STATIC_ASSERT(0 <= CapacityValue);

    private:

        //=============================================================================
        //  CConstIterator
        //-----------------------------------------------------------------------------
        //! A type that provides a bidirectional iterator that 
        //! can read a const element in a TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @author                 vmr1kor
        //! @ingroup                vfc_containers
        //=============================================================================

        class CConstIterator
        {
        public:

            typedef CConstIterator                              self_type ;
            typedef ValueType                                   value_type;
            typedef const ValueType*                            pointer;
            typedef const ValueType&                            reference;
            typedef stlalias::bidirectional_iterator_tag        iterator_category;
            typedef ptrdiff_t                                   difference_type;

        
            //---------------------------------------------------------------------
            //! Default constructor, takes no arguments.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //---------------------------------------------------------------------
            CConstIterator() : m_index(0) , m_cirBuff_cp(0)
            {
            }

            //---------------------------------------------------------------------
            //! Destructor.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  jat2hi
            //---------------------------------------------------------------------
            ~CConstIterator()
            {
            }

            //---------------------------------------------------------------------
            //! Copy constructor.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author     jat2hi
            //---------------------------------------------------------------------
            CConstIterator(CConstIterator const & f_rhs) : m_index(f_rhs.m_index), m_cirBuff_cp(f_rhs.m_cirBuff_cp)
            {
            }

            //---------------------------------------------------------------------
            //! Assignment operator.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author     jat2hi
            //---------------------------------------------------------------------
            CConstIterator const & operator=(CConstIterator const & f_rhs);

            //====================================================================
            //! prefix increment iterator.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //====================================================================
            self_type& operator ++();

            //====================================================================
            //! post increment iterator.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //====================================================================
            // PRQA S 2427 ++
            self_type operator ++(int);
            // PRQA S 2427 --

            //====================================================================
            //! prefix decrement iterator.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //====================================================================
            self_type& operator --();

            //====================================================================
            //! postfix decrement iterator.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //====================================================================
            // PRQA S 2427 ++
            self_type operator --(int);
            // PRQA S 2427 --

            //====================================================================
            //! opeator + , add f_index to m_index.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //====================================================================
            const self_type operator + (vfc::int32_t f_val) const;

            self_type& operator += ( vfc::int32_t f_val);

            //====================================================================
            //! opeator - , decrement f_index from m_index.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //====================================================================
            const self_type operator - (vfc::int32_t f_val) const;

            self_type& operator -= ( vfc::int32_t f_val);

            //====================================================================
            //! tests if the object on the left side of the operator is 
            //! less than the object on the right side.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @return  Returns true when object on the left side of the operator
            //!          is less than the object on the right side else false. 
            //! @author  vmr1kor
            //====================================================================
            bool operator < (const self_type& f_self_type) const;

            //========================================================================
            //! tests if the object on the left side of the operator is 
            //! less than or equal to the object on the right side.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @return  Returns true when object on the left side of the operator is 
            //!          less than or equal to the object on the right side else false.
            //! @author  vmr1kor
            //========================================================================
            bool operator <= (const self_type& f_self_type) const;

            //========================================================================
            //! tests if the object on the left side of the operator is 
            //! greater than the object on the right side.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @return  Returns true when object on the left side of the operator is 
            //!          greater than the object on the right side else false.
            //! @author  vmr1kor
            //========================================================================
            bool operator > (const self_type& f_self_type) const;

            //========================================================================
            //! tests if the object on the left side of the operator is 
            //! greater than the object on the right side.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @return  Returns true when object on the left side of the operator is 
            //!          greater than the object on the right side else false.
            //! @author  vmr1kor
            //========================================================================
            bool operator >= (const self_type& f_self_type) const;

            //========================================================================
            //! tests if the object on the left side of the operator is 
            //! equal to the object on the right side.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @return  Returns true when object on the left side of the operator is 
            //!          equal to the object on the right side else false.
            //! @author  vmr1kor
            //========================================================================
            bool operator == ( const self_type& f_self_type) const;

            //========================================================================
            //! tests if the object on the left side of the operator is 
            //! not equal to the object on the right side.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @return  Returns true when object on the left side of the operator is 
            //!          not equal to the object on the right side else false.
            //! @author  vmr1kor
            //======================================================================== 
            bool operator != (const self_type& f_self_type) const;

            //========================================================================
            //! Returns reference to the object.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @return  Returns reference to the object
            //! @author  vmr1kor
            //======================================================================== 
            reference operator*() const;

            //========================================================================
            //! Returns pointer to the object.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @return  Returns pointer to the object.
            //! @author  vmr1kor
            //======================================================================== 
            pointer operator->() const;

        protected:

            //---------------------------------------------------------------------
            //! Parameterised Constructor takes TFixedCircularBuffer and f_index.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //---------------------------------------------------------------------
            CConstIterator(const TFixedCircularBuffer* f_value_p,vfc::int32_t f_index) 
            : m_index (f_index), m_cirBuff_cp(f_value_p)
            {
            }

            vfc::int32_t                    m_index;
            const TFixedCircularBuffer*     m_cirBuff_cp;
            friend class TFixedCircularBuffer ;             //! TFixedCircularBuffer can access private members
                                                            //! as well call the private constructor
        };  


        //=============================================================================
        //  CIterator
        //-----------------------------------------------------------------------------
        //! A type that provides a bidirectional iterator that can 
        //! read or modify any element in a TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @author                 vmr1kor
        //! @ingroup                vfc_containers
        //=============================================================================
        class CIterator : public CConstIterator
        {
        public:

            typedef CConstIterator                          base_type;
            typedef CIterator                               self_type ;
            typedef ValueType                               value_type;
            typedef ValueType*                              pointer;
            typedef ValueType&                              reference;
            typedef stlalias::bidirectional_iterator_tag    iterator_category;
            typedef ptrdiff_t                               difference_type;

            //---------------------------------------------------------------------
            //! Default constructor, takes no arguments.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //---------------------------------------------------------------------
            CIterator() : base_type()
            {
            }
        
            //========================================================================
            //! prefix increment iterator.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //========================================================================
            self_type& operator ++();

            //========================================================================
            //! post increment iterator.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //========================================================================
            // PRQA S 2427 ++
            self_type operator ++(int);
            // PRQA S 2427 --

            
            //========================================================================
            //! prefix decrement iterator.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //========================================================================
            self_type& operator --();

            //========================================================================
            //! postfix decrement iterator.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //========================================================================            
            // PRQA S 2427 ++
            self_type operator --(int);
            // PRQA S 2427 --

            //========================================================================
            //! Returns reference to the object.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @return  Returns reference to the object.
            //! @author  vmr1kor
            //========================================================================          
            reference operator*() const;

            //========================================================================
            //! Returns pointer to the object.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @return  Returns pointer to the object.
            //! @author  vmr1kor
            //========================================================================
            pointer operator->() const;

            //====================================================================
            //! opeator + , add f_index to m_index.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //====================================================================
            const self_type operator + (vfc::int32_t f_val) const;

            self_type& operator += ( vfc::int32_t f_val);

            //====================================================================
            //! opeator - , decrement f_index from m_index.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //====================================================================
            const self_type operator - (vfc::int32_t f_val) const;

            self_type& operator -= ( vfc::int32_t f_val);

        private:

            //---------------------------------------------------------------------
            //! constructor takes CConstIterator as input parameter.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //---------------------------------------------------------------------
            explicit CIterator(const CConstIterator& f_baseObj_r);

            //---------------------------------------------------------------------
            //! Parameterised Constructor takes TFixedCircularBuffer and f_index.
            //! $Source: vfc_fixedcircularbuffer.hpp $
            //! @author  vmr1kor
            //---------------------------------------------------------------------
            CIterator(TFixedCircularBuffer* f_value_p,vfc::int32_t f_index) 
            : base_type(f_value_p,f_index)
            {
            }

            friend class TFixedCircularBuffer ;                 //! TFixedCircularBuffer can access private variables
        };                                                      //! as well call the private constructor

    public:

        enum 
        {
            BUFFER_SIZE = CapacityValue  //! The static TFixedCircularBuffer size 
                                         //!(defined at compile time).
        };

        //! typedefs

        //! A type that represent the TFixedCircularBuffer Type
        typedef TFixedCircularBuffer<ValueType,CapacityValue,AllocatorType> circularBuffer_type ;
        
        //! A type that represent the data type stored in a TFixedCircularBuffer
        typedef ValueType                                                   value_type;

        //! A type that represents the allocator class for a TFixedCircularBuffer object.
        typedef AllocatorType                                               allocator_type;

        //! A type that provides a pointer to an element in a TFixedCircularBuffer.
        typedef typename allocator_type::pointer                            pointer;

        //! A type that provides a pointer to a const element in a TFixedCircularBuffer.
        typedef typename allocator_type::const_pointer                      const_pointer;
        
        //! A type that provides a reference to a element stored in a TFixedCircularBuffer for reading 
        //! and performing operations.
        typedef typename allocator_type::reference                          reference;
        
        //! A type that provides a reference to a const element stored in a TFixedCircularBuffer for 
        //! reading and performing const operations.
        typedef typename allocator_type::const_reference                    const_reference;

        //! A type that counts the number of elements in a TFixedCircularBuffer.
        typedef vfc::int32_t                                                size_type;

        //! A type that provides the difference between two iterators that refer to 
        //! elements within the same TFixedCircularBuffer.
        typedef ptrdiff_t                                                   difference_type;

        typedef stlalias::bidirectional_iterator_tag                        iterator_category;
        
        //! A type that provides bidirectional iterator that can read and modify
        //! any element in the TFixedCircularBuffer
        typedef CIterator                                                   iterator ;

        //! A type that provides a bidirectional iterator that can read a const element 
        //! in a TFixedCircularBuffer
        typedef CConstIterator                                              const_iterator;
        
        //! A type that provides a bidirectional reverse iterator that can read or modify 
        //! an element in a TFixedCircularBuffer
        typedef stlalias::reverse_iterator<iterator>                        reverse_iterator ;

        //! A type that provides a bidirectional reverse iterator that can read any const element 
        //! in a TFixedCircularBuffer
        typedef stlalias::reverse_iterator<const_iterator>                  const_reverse_iterator;

        //---------------------------------------------------------------------
        //! Default constructor, takes no arguments.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @author  vmr1kor
        //---------------------------------------------------------------------
        TFixedCircularBuffer();


        //---------------------------------------------------------------------
        //! Copy constructor.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @author  vmr1kor
        //---------------------------------------------------------------------
        TFixedCircularBuffer(const circularBuffer_type& f_param_r);

        //---------------------------------------------------------------------
        //! Allocator constructor.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @author  vmr1kor
        //---------------------------------------------------------------------
        explicit TFixedCircularBuffer(const AllocatorType& f_alloc);

        //---------------------------------------------------------------------
        //! Overloaded assignment operator.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @author  vmr1kor
        //---------------------------------------------------------------------
        const TFixedCircularBuffer& operator=(const TFixedCircularBuffer&);

        //---------------------------------------------------------------------
        //! Default Destructor.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @author  vmr1kor
        //---------------------------------------------------------------------
        ~TFixedCircularBuffer();


        //=============================================================================
        //! Returns the copy of the allocator object used to construct the TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @return  copy of the allocator object
        //! @author  vmr1kor
        //=============================================================================
        allocator_type get_allocator() const;


        //=============================================================================
        //! Adds an element to the back of the Circularbuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @param   f_param_r   The element added to the top of the Circularbuffer. 
        //! @return  Returns nothing.
        //! @author  vmr1kor
        //=============================================================================
        void push(const value_type& f_param_r);

#if __cplusplus >= 201103L
        //! Constructs an element at the end of the Circularbuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @param   f_args   arguments to forward to the constructor of the element
        //! @return  reference to the constructed element
        //! @author  bet1pal
        template<class... Args>
        reference emplace(Args&&... f_args);
#endif

        //-----------------------------------------------------------------------------
        //! Adds an UNINITIALIZED element to the back of the Circularbuffer.
        //! If the buffer is full, push_uninitialized overwrites the oldest element
        //! Use in conjunction with placement new, if you have buffer elements that
        //! are very costly to copy and you still want to use a Circularbuffer container:
        //! @return  Returns an iterator to the element inserted
        //! @code new (&*myCircBuff.push_uninitialized()) myElement(ctorArgs); @endcode
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @author bum7lr
        //=============================================================================
        reference push_uninitialized();
        
        //=============================================================================
        //! Remove the oldest available element.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @note    If TFixedCircularBuffer is empty throws static assertion.
        //! @return  Returns nothing.
        //! @author  vmr1kor
        //=============================================================================
        void pop();

        //=============================================================================
        //! Reference to the oldest element in the TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @return  Returns reference to oldest element.
        //! @author  vmr1kor
        //=============================================================================
        reference front();

        //=============================================================================
        //! Const reference to the oldest element in the TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @return  Returns const reference to oldest element.
        //! @author  vmr1kor
        //=============================================================================
        const_reference front() const;

        //=============================================================================
        //! reference to the newly added element.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @return  reference to the newly added element.
        //! @author  vmr1kor
        //=============================================================================
        reference back();


        //=============================================================================
        //! Const reference to the newly added element.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @return  Const reference to the newly added element.
        //! @author  vmr1kor
        //=============================================================================
        const_reference back() const;


        //=================================================================================
        //! Returns a reference to the element at position f_index in circulatBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @note       zero is the front() element
        //! @param      index of the TFixedCircularBuffer.
        //! @return     reference to a history element with speified index.
        //! @author     vmr1kor
        //==================================================================================
        reference operator[](int32_t f_index);

        
        //===================================================================================
        //! Returns a const reference to the element at position f_index in circulatBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @note       The index of element (Ex:- zero is the front() element).
        //! @param      index of the TFixedCircularBuffer.
        //! @return     Const reference to a history element with speified index.
        //! @author     vmr1kor
        //====================================================================================
        const_reference operator[](int32_t f_index) const;


        //=============================================================================
        //! Tests if a TFixedCircularBuffer is empty.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @return  true if the TFixedCircularBuffer is empty; false if the TFixedCircularBuffer is nonempty.
        //! @author  vmr1kor
        //=============================================================================
        bool empty() const;

        //=============================================================================
        //! Returns the number of elements in the TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @return  current length of TFixedCircularBuffer.
        //! @author  vmr1kor
        //=============================================================================
        size_type size() const;

        //=============================================================================
        //! Returns the maximum length of the TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @return  Maximum length of TFixedCircularBuffer.
        //! @author  vmr1kor
        //=============================================================================
        size_type capacity() const;


        //=============================================================================
        //! Erases the elements of the TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @return  Returns nothing.
        //! @author  vmr1kor
        //=============================================================================
        void clear();


        //===========================================================================================        
        //! Returns a random-access iterator to the first element in the TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @return  A random-access iterator addressing the first element in the TFixedCircularBuffer.
        //! @author  vmr1kor
        //===========================================================================================
        iterator begin();


        //=================================================================================================
        //! Returns a random-access Const iterator to the first element in the TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @return  A random-access Const iterator addressing the first element in the TFixedCircularBuffer.
        //! @author  vmr1kor
        //=================================================================================================
        const_iterator begin() const;

        //========================================================================================
        //! Returns a random-access iterator that points just beyond the end of the TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp $
        //! @return  A random-access iterator to the end of the TFixedCircularBuffer object.
        //! @author  vmr1kor
        //=========================================================================================
        iterator end();

        //==============================================================================================       
        //! Returns a random-access Const iterator that points just beyond the end of the TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp 
        //! @return  A random-access Const iterator to the end of the TFixedCircularBuffer object.
        //! @author  vmr1kor
        //==============================================================================================
        const_iterator end() const;

        //=====================================================================================
        //! Returns an iterator to the first element in a reversed TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp 
        //! @return  A reverse random-access iterator addressing the first element in a reversed 
        //!          TFixedCircularBuffer.
        //! @author  vmr1kor
        //=====================================================================================
        reverse_iterator rbegin();

        //===========================================================================================
        //! Returns an const iterator to the first element in a reversed TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp 
        //! @return  A reverse random-access const iterator addressing the first element in a reversed
        //!          TFixedCircularBuffer.
        //! @author  vmr1kor
        //============================================================================================
        const_reverse_iterator rbegin() const;

        //=============================================================================================
        //! Returns an iterator to the end of a reversed TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp 
        //! @return  A reverse random-access iterator that addresses the location succeeding the last 
        //!          element in a reversed TFixedCircularBuffer.
        //! @author  vmr1kor
        //=============================================================================================
        reverse_iterator rend();

        //=============================================================================================
        //! Returns an const iterator to the end of a reversed TFixedCircularBuffer.
        //! $Source: vfc_fixedcircularbuffer.hpp 
        //! @return  A reverse random-access const iterator that addresses the location succeeding the 
        //!          last element in a reversed TFixedCircularBuffer.
        //! @author  vmr1kor
        //==============================================================================================
        const_reverse_iterator rend() const;


     private:

        pointer             m_basePointer_p;            //! Basepointer: points to the start of allocated memory
        allocator_type      m_alloc;                    //! allocator type

        pointer             m_read_p;                   //! Read pointer
        pointer             m_write_p;                  //! Write pointer
        size_type           m_size;                     //! size of the CiruclarBuffer

        bool IsOverFlow() const;                        //! check for TFixedCircularBuffer Overflow
        
        void incrementPointer(pointer* f_index_pp) const;   //! Increment pointer , if overflow
                                                            //! since it is TFixedCircularBuffer , not 
                                                            //! possible just ++
        
        void getDecrementPointer(pointer f_write_p , pointer* f_decr_pp) const;  //! function called when back() 
                                                                                 //! called , since m_write_p is 
                                                                                 //! pointing to next memory location
                                                                                 //! and circular in nature , can't 
                                                                                 //! do blind decrement

        void push_intern(const value_type* f_param_r); //! pushes the param value in the buffer.
                                                       //! used in order to avoid code duplicates and
                                                       //! prevent from modifying public functions declaration
    };

    //====================================================================================
    //! TFixedCircularBuffer equality comparison.
    //! This is an equivalence relation.  It is linear in the size of the TFixedCircularBuffer.
    //! TFixedCircularBuffer are considered equivalent if their sizes are  equal,
    //! and if corresponding elements compare equal.
    //! $Source: vfc_fixedcircularbuffer.hpp 
    //! @param   f_cirBuffer1_r  TFixedCircularBuffer.
    //! @param   f_buffer2_r  TFixedCircularBuffer.
    //! @return  True if the size and elements of the TFixedCircularBuffer are equal.
    //! @author  vmr1kor
    //! @ingroup vfc_containers
    //====================================================================================
    template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
    bool operator == (const TFixedCircularBuffer<ValueType,CapacityValue,AllocatorType>& f_cirBuffer1_r,
            const TFixedCircularBuffer<ValueType,CapacityValue,AllocatorType>& f_cirBuffer2_r);

    template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
    bool operator != (const TFixedCircularBuffer<ValueType,CapacityValue,AllocatorType>& f_cirBuffer1_r,
            const TFixedCircularBuffer<ValueType,CapacityValue,AllocatorType>& f_cirBuffer2_r);


    //==========================================================================
    //! TFixedCircularBuffer ordering relation.
    //! This is a total ordering relation.  It is linear in the size of the
    //! TFixedCircularBuffer.  The elements must be comparable with <.
    //! $Source: vfc_fixedcircularbuffer.hpp 
    //! @param   f_cirBuffer1_r     TFixedCircularBuffer.
    //! @param   f_buffer2_r        CircularBuffer of the same type as f_cirBuffer1_r.
    //! @return  True if f_cirBuffer1_r is lexicographically less than f_buffer2_r.
    //! @author  vmr1kor
    //! @ingroup vfc_containers
    //==========================================================================
    template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
    bool operator < (const TFixedCircularBuffer<ValueType,CapacityValue,AllocatorType>& f_cirBuffer1_r,
            const TFixedCircularBuffer<ValueType,CapacityValue,AllocatorType>& f_cirBuffer2_r);

    template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
    bool operator > (const TFixedCircularBuffer<ValueType,CapacityValue,AllocatorType>& f_cirBuffer1_r,
            const TFixedCircularBuffer<ValueType,CapacityValue,AllocatorType>& f_cirBuffer2_r);

    template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
    bool operator <= (const TFixedCircularBuffer<ValueType,CapacityValue,AllocatorType>& f_cirBuffer1_r,
            const TFixedCircularBuffer<ValueType,CapacityValue,AllocatorType>& f_cirBuffer2_r);

    template<class ValueType,vfc::int32_t CapacityValue,class AllocatorType>
    bool operator >= (const TFixedCircularBuffer<ValueType,CapacityValue,AllocatorType>& f_cirBuffer1_r,
            const TFixedCircularBuffer<ValueType,CapacityValue,AllocatorType>& f_cirBuffer2_r);

} // namespace vfc

#include "vfc/container/vfc_fixedcircularbuffer.inl"

#endif //VFC_FIXEDCIRCULARBUFFER_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fixedcircularbuffer.hpp  $
//  Revision 1.5 2014/12/08 15:57:12MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_fixedcircularbuffer: fix MISRA warning (0004847)
//  Revision 1.4 2014/10/02 14:28:48MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Add TCircularBuffer member function to create uninitialized entry (mantis0004553)
//  Revision 1.3 2013/05/15 08:08:40MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_types include is missing (mantis4222)
//  Revision 1.2 2012/12/18 08:27:23MEZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.1 2010/08/11 17:49:43MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/container/container.pj
//  Revision 1.3 2009/02/05 05:10:21MEZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Removal of QAC++ warnings.
//  (Mantis : 0002562)
//  Revision 1.2 2008/09/30 15:58:50IST Vinaykumar Setty (RBEI/EAC1) (vmr1kor) 
//  - Review comments and QAC++ related issues are implemented ( mantis :- 001574,002341,002342 )
//  Revision 1.1 2008/09/04 14:09:08IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/circular_buffer/circular_buffer.pj
//=============================================================================
