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
//       Projectname: vfc
//          Synopsis:
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
///     $Source: vfc_fixedmempool_allocator.hpp $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2010/08/11 17:46:33MESZ $
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

#ifndef VFC_FIXEDMEMPOOL_ALLOCATOR_HPP_INCLUDED
#define VFC_FIXEDMEMPOOL_ALLOCATOR_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"               // used for size_t, int32_t
#include "vfc/memory/vfc_fixedmempool.hpp"      // used for TFixedMempool<>

namespace vfc
{   // namespace vfc opened


    //============================================================================
    // TFixedMemPoolAllocator
    //----------------------------------------------------------------------------
    //! The template class describes an object that manages storage allocation
    //! and freeing for arrays of objects of type ValueType.
    //! $Source: vfc_fixedmempool_allocator.hpp $
    //! @param   ValueType        DataType to be stored
    //! @param   SizeValue        Indicates the size of the arrays
    //! @Note: TFixedMemPoolAllocator does not work when the size of the ValueType
    //! is less than size of int32_t.
    //! @author dkn2kor
    //! @ingroup vfc_memory
    //============================================================================


    template <class ValueType, int32_t SizeValue>
    class TFixedMemPoolAllocator
    {
        VFC_STATIC_ASSERT(0 < SizeValue);

    public:

        //! typedefs

        //! A type that is managed by the allocator.
        //! Note: size of the ValueType has to be greater than size of int32_t.
        typedef ValueType           value_type;

        //! A type that provides a pointer to the type of object managed by the allocator.
        typedef value_type*         pointer;

        //! A type that provides a reference to the type of object managed by the allocator.
        typedef value_type&         reference;

        //! A type that provides a constant pointer to the type of object managed by
        //! the allocator.
        typedef const value_type*   const_pointer;

        //! A type that provides a constant reference to type of object managed by
        //! the allocator.
        typedef const value_type&   const_reference;

        //! An integral type that can represent the length of any sequence
        //! that an object of template class allocator can allocate.
        typedef int32_t             size_type;

        //! A signed integral type that can represent the difference between values of
        //! pointers to the type of object managed by the allocator.
        typedef ptrdiff_t           difference_type;

    public:

        //! A structure that enables an allocator for objects of one type to allocate storage for
        //! objects of an other type.
        template <class OtherValueType>
        struct rebind
        {
            typedef  TFixedMemPoolAllocator<OtherValueType,SizeValue> other;
        };

        //! A structure that enables an allocator for objects of one type to allocate storage for
        //! objects of an other type.
        template <class OtherValueType, int32_t OtherSizeValue>
        struct rebind2
        {
            typedef  TFixedMemPoolAllocator<OtherValueType,OtherSizeValue> other;
        };

    public:

        //---------------------------------------------------------------------
        //! Default constructor, takes no arguments.
        //! $Source: vfc_fixedmempool_allocator.hpp $
        //! @author dkn2kor
        //---------------------------------------------------------------------
        TFixedMemPoolAllocator(void);


        //---------------------------------------------------------------------
        //! Copy constructor.
        //! $Source: vfc_fixedmempool_allocator.hpp $
        //! @author dkn2kor
        //---------------------------------------------------------------------
        TFixedMemPoolAllocator (const TFixedMemPoolAllocator&);


        //-------------------------------------------------------------------------
        //! Template copy constructor.
        //! $Source: vfc_fixedmempool_allocator.hpp $
        //! @Note       does not assert if the other size value is greater than its size value
        //! @author     dkn2kor
        //-------------------------------------------------------------------------
        template <class OtherValueType, int32_t OtherSizeValue>
        TFixedMemPoolAllocator (const TFixedMemPoolAllocator<OtherValueType,OtherSizeValue>&)
        : m_memPool() {}


        //-------------------------------------------------------------------------
        //! Assingment operator.
        //! $Source: vfc_fixedmempool_allocator.hpp $
        //! @author     dkn2kor
        //-------------------------------------------------------------------------
        const TFixedMemPoolAllocator& operator=(const TFixedMemPoolAllocator&) { return *this; }


        //=============================================================================
        //! Allocates a block of memory large enough to store 1 element.
        //! $Source: vfc_fixedmempool_allocator.hpp $
        //! @param  f_count   The number of elements for which sufficient storage is to be allocated.
        //! @return           pointer to allocated memory block or 0, if allocation failed.
        //! @Note             TFixedMemPoolAllocator asserts if the count value is other than 1.
        //! @Note             Validate returned pointer for null value after allocate(..) call.
        //! @author           dkn2kor
        //=============================================================================
        pointer         allocate    (size_type f_count);



        //=============================================================================
        //! Allocates a block of memory large enough to store 1 element.
        //! The second argument is used as an hint for the underlying implementation.
        //! $Source: vfc_fixedmempool_allocator.hpp $
        //! @param  f_count   The number of elements for which sufficient storage is to be allocated.
        //! @param  void*     A const pointer that may assist the allocator object satisfy the request
        //!                   for storage by locating the address of an object allocated prior to the request.
        //! @return           pointer to allocated memory block or 0, if allocation failed
        //! @Note             TFixedMemPoolAllocator asserts if the count value is other than 1.
        //! @author           dkn2kor
        //=============================================================================
        pointer         allocate    (size_type f_count, const void*);



        //=============================================================================
        //! Frees a object from storage beginning at a specified position.
        //! $Source: vfc_fixedmempool_allocator.hpp $
        //! @param    f_ptr   A pointer to the first object to be deallocated from storage.
        //! @param    f_count The number of objects to be deallocated from storage.
        //! @Note             TFixedMemPoolAllocator asserts if the count value is other than 1.
        //! @author           dkn2kor
        //=============================================================================
        void            deallocate  (   pointer     f_ptr,
                                        size_type   f_count
                                    );


        //=============================================================================
        //! Constructs a specific type of object at a specified address that is initialized with a
        //! specified value.
        //! $Source: vfc_fixedmempool_allocator.hpp $
        //! @param    f_ptr   A pointer to the location where the object is to be constructed.
        //! @param    f_val   The value with which the object being constructed is to be initialized.
        //! @author           dkn2kor
        //=============================================================================
        void            construct   (pointer f_ptr, const value_type& f_val);


        //=============================================================================
        //! Calls an objects destructor without deallocating the memory where the object was stored.
        //! $Source: vfc_fixedmempool_allocator.hpp $
        //! @param    f_ptr   A pointer designating the address of the object to be destroyed
        //! @author           dkn2kor
        //=============================================================================
        void            destroy     (pointer f_ptr);


        //=============================================================================
        //! Finds the address of an object whose value is specified.
        //! $Source: vfc_fixedmempool_allocator.hpp $
        //! @param    f_val   A pointer designating the address of the object to be destroyed.
        //! @return           A nonconst pointer to the object found of nonconst value.
        //! @author           dkn2kor
        //=============================================================================
        pointer         address     (reference       f_val)  const;


        //=============================================================================
        //! Finds the address of an object whose value is specified.
        //! $Source: vfc_fixedmempool_allocator.hpp $
        //! @param    f_val   The value of the object whose address is being searched for.
        //! @return           A const pointer to the object found of const value.
        //! @author           dkn2kor
        //=============================================================================
        const_pointer   address     (const_reference f_val)  const;


        //=============================================================================
        //! Returns the number of elements of type Type that could be allocated by an
        //! object of class allocator before the free memory is used up.
        //! $Source: vfc_fixedmempool_allocator.hpp $
        //! @param    f_val   The const value of the object whose address is being searched for.
        //! @return           The number of elements that could be allocated.
        //! @author           dkn2kor
        //=============================================================================
        size_type       max_size    (void)   const;



    private:

        enum
        {
            ELEMENT_SIZE  = sizeof(ValueType),   //!< Indicates the size of ValueType
            SIZE_VALUE = SizeValue               //!< Indicates the size of the arrays
        };
        //!typedefs

        //! A type that provides Object For TFixedMempool
        typedef vfc::TFixedMempool<ELEMENT_SIZE, SIZE_VALUE> storage_type;

        storage_type m_memPool;
    };


    //==================================================================================
    //! Test for allocater equality.
    //! $Source: vfc_fixedmempool_allocator.hpp $
    //! @param    f_alloc1   The object of the type TFixedMemPoolAllocator<> to be Compared.
    //! @param    f_alloc2   The object of the type TFixedMemPoolAllocator<> to be Compared.
    //! @return   bool       Returns true when both the objects are Equal else false.
    //! @author              dkn2kor
    //! @ingroup             vfc_memory
    //==================================================================================
    template<   class Value1Type, int32_t Size1Value,
                class Value2Type, int32_t Size2Value> inline
    bool operator== (   const TFixedMemPoolAllocator<Value1Type, Size1Value>& f_alloc1,
                        const TFixedMemPoolAllocator<Value2Type, Size2Value>& f_alloc2)
    {
        // allocators are equal if and only if they are the identical object
        // we have to cast to void* because it is possible that
        // we are comparing allocators of different types
        return (static_cast<const void*>(&f_alloc1) == static_cast<const void*>(&f_alloc2));
    }



    //==================================================================================
    //! Test for allocater inequality.
    //! $Source: vfc_fixedmempool_allocator.hpp $
    //! @param    f_alloc1   The object of the type TFixedMemPoolAllocator<> to be Compared.
    //! @param    f_alloc2   The object of the type TFixedMemPoolAllocator<> to be Compared.
    //! @return   bool       Returns true when both the objects are NotEqual else false.
    //! @author              dkn2kor
    //! @ingroup             vfc_memory
    //==================================================================================
    template<   class Value1Type, int32_t Size1Value,
                class Value2Type, int32_t Size2Value> inline
    bool operator!= (   const TFixedMemPoolAllocator<Value1Type, Size1Value>& f_alloc1,
                        const TFixedMemPoolAllocator<Value2Type, Size2Value>& f_alloc2)
    {
        // just negate equality
        return !(f_alloc1 == f_alloc2);
    }


}   // namespace vfc closed

#include "vfc/memory/vfc_fixedmempool_allocator.inl"

#endif //VFC_FIXEDMEMPOOL_ALLOCATOR_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fixedmempool_allocator.hpp  $
//  Revision 1.1 2010/08/11 17:46:33MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/memory/memory.pj
//  Revision 1.5 2008/08/29 15:11:16MESZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  Documentation on how to test for allocation failure (Mantis : 2013)
//  Revision 1.4 2008/01/11 16:46:31IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  DoxyComments are changed
//  Mantis Id :- 0001807
//  Revision 1.3 2007/10/31 18:47:34IST vmr1kor
//  comments/docu to vfc_fixedmempool.hpp
//  Rename SizeValue , SIZEVALUE enum member
//  Mantis Id :- 0001807
//  Revision 1.2 2007/06/22 17:43:21IST dkn2kor
//  - comments changed
//  Revision 1.1 2007/06/18 13:19:51IST dkn2kor
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_list/vfc_list.pj
//=============================================================================
