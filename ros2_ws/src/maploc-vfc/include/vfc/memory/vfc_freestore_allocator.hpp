//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2007 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy, use or 
//  distribution is an offensive act against international law and may be 
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/memory
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
///     $Source: vfc_freestore_allocator.hpp $
///     $Revision: 1.4 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2012/12/18 08:27:36MEZ $
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

#ifndef VFC_FREESTORE_ALLOCATOR_HPP_INCLUDED
#define VFC_FREESTORE_ALLOCATOR_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

namespace vfc
{   // namespace vfc opened


    //=============================================================================
    //  TFreeStoreAllocator<>
    //-----------------------------------------------------------------------------
    /// generic allocator for objects of class ValueType.
    /// @ingroup vfc_group_memory_allocators
    //-----------------------------------------------------------------------------

    template<class ValueType>
    class TFreeStoreAllocator
    {
    public:
     
        typedef ValueType           value_type;
        typedef value_type*         pointer;
        typedef value_type&         reference;
        typedef const value_type*   const_pointer;
        typedef const value_type&   const_reference;

        typedef int32_t     size_type;
        typedef ptrdiff_t   difference_type;

    public:
        template<class OtherType>
        struct rebind
        {
            // convert an allocator<ValueType> to an allocator <OtherType>
            typedef TFreeStoreAllocator<OtherType> other;
        };

    public:
        /// construct default allocator (do nothing)
        TFreeStoreAllocator()   {    }

        /// construct by copying (do nothing)
        TFreeStoreAllocator(const TFreeStoreAllocator<ValueType>&)  {}
        
        /// construct from a related allocator (do nothing)
        template<class OtherType>
        TFreeStoreAllocator(const TFreeStoreAllocator<OtherType>&)  {}

        /// assign from a related allocator (do nothing)
        template<class OtherType>
        TFreeStoreAllocator<ValueType>& operator=(const TFreeStoreAllocator<OtherType>&)
        {   return (*this); }

    public:
        /// \return address of mutable f_arg
        pointer address(reference f_arg) const;

        /// \return address of nonmutable f_arg
        const_pointer address(const_reference f_arg) const;
        
        /// allocate array of f_count elements
        pointer allocate(size_type f_count);

        /// allocate array of f_count elements, ignore hint
        /// \return 0 pointer if allocation fails
        pointer allocate(size_type f_count, const void *);

        /// construct object at f_ptr with value f_arg
        void construct(pointer f_ptr, const ValueType& f_arg);

        /// destroy object at f_ptr
        void destroy(pointer f_ptr);

        /// deallocate object at f_ptr, ignore size
        void deallocate(pointer f_ptr, size_type);

        /// estimate maximum array size
        size_type max_size() const;
    };

    /// test for allocator equality (always true). @relates TFreeStoreAllocator
    template<class ValueType, class OtherType> inline
    bool operator==(const TFreeStoreAllocator<ValueType>&, const TFreeStoreAllocator<OtherType>&)
    {
        return (true);
    }

    /// test for allocator inequality (always false). @relates TFreeStoreAllocator
    template<class ValueType, class OtherType> inline
    bool operator!=(const TFreeStoreAllocator<ValueType>&, const TFreeStoreAllocator<OtherType>&)
    {
        return (false);
    }


}   // namespace vfc closed

#include "vfc/memory/vfc_freestore_allocator.inl"

#endif //VFC_ALLOCATOR_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_freestore_allocator.hpp  $
//  Revision 1.4 2012/12/18 08:27:36MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.3 2008/08/25 14:02:01MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with spaces (mantis1294)
//  Revision 1.2 2007/07/23 09:56:48CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.1 2007/02/15 10:14:44CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/memory/memory.pj
//=============================================================================
