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
//        Name: ZVH2HI
//  Department: CR/AEM
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_fixedblock_allocator.hpp $
///     $Revision: 1.14 $
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

#ifndef VFC_FIXEDBLOCK_ALLOCATOR_HPP_INCLUDED
#define VFC_FIXEDBLOCK_ALLOCATOR_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"               // used for size_t, int32_t
#include "vfc/core/vfc_static_assert.hpp"       // used for VFC_STATIC_ASSERT
#include "vfc/core/vfc_aligned_storage.hpp"     // used for TAlignedStorage<>

namespace vfc
{   // namespace vfc opened

    //=========================================================================
    //    vfc::TFixedBlockAllocator<>
    //-------------------------------------------------------------------------
    /// Allocates memory from a fixed-size contiguous memory block.
    /// @par Description:
    /// The TFixedBlockAllocator manages a fixed-size, contiguous memory area 
    /// divided into allocated and unallocated portions. 
    /// A break pointer (brk) contains the adress of the unallocated portion. 
    /// It initially points to the beginning of the internal memblock. 
    /// You allocate memory by moving the pointer upwards and you free memory 
    /// by moving the pointer downwards (stack-like).
    /// @note
    /// In order to succesfully reclaim previously allocated memory, the blocks 
    /// have to be freed in the reverse order they have been allocated - 
    /// otherwise they are lost until the whole allocator is destructed.
    /// @ingroup vfc_group_memory_allocators
    //=========================================================================
    
    template <class ValueType, int32_t SizeValue>
    class TFixedBlockAllocator
    {
        VFC_STATIC_ASSERT(0 < SizeValue);

    public:  
        typedef ValueType           value_type;
        typedef value_type*         pointer;
        typedef value_type&         reference;
        typedef const value_type*   const_pointer;
        typedef const value_type&   const_reference;

        typedef int32_t             size_type;
        typedef ptrdiff_t           difference_type;

    public:

        /// A structure that enables an allocator for objects of one type to allocate storage for objects of an other type.
        template <class OtherValueType>
        struct rebind
        {
            typedef  TFixedBlockAllocator<OtherValueType,SizeValue> other;
        };

        /// A structure that enables an allocator for objects of one type to allocate storage for objects of an other type.
        template <class OtherValueType, int32_t OtherSizeValue>
        struct rebind2
        {
            typedef  TFixedBlockAllocator<OtherValueType,OtherSizeValue> other;
        };
    
    public:  
        // c'tors
        /// default c'tor, initializes the brk pointer
        TFixedBlockAllocator(void);

        /// copy c'tor, initializes brk pointer but doesn't copy anything
        TFixedBlockAllocator (const TFixedBlockAllocator&);

        /// template copy c'tor, initializes brk pointer but doesn't copy anything
        template <class OtherValueType, int32_t OtherSizeValue>
        TFixedBlockAllocator (const TFixedBlockAllocator<OtherValueType,OtherSizeValue>&) 
        : m_memblock(), m_brk(begin()) {}

        /// assignment op, does nothing
        const TFixedBlockAllocator& operator=(const TFixedBlockAllocator&) { return *this;}

        //! d'tor.
        ~TFixedBlockAllocator(void) {}
        //----------------------------
        // allocation/deallocation
        //----------------------------

        /// Allocates a block of memory large enough to store at least some specified number of elements.
        /// @return pointer to allocated memory block or 0, if allocation failed
        pointer         allocate    (size_type f_count);
        
        /// Allocates a block of memory large enough to store at least some specified number of elements. 
        /// The second argument is used as an hint for the underlying implementation.
        /// @return pointer to allocated memory block or 0, if allocation failed
        pointer         allocate    (size_type f_count, const void*);
        
        /// Frees a specified number of objects from storage beginning at a specified position.
        void            deallocate  (   pointer const f_ptr,  ///< A pointer to the first object to be deallocated from storage. 
                                        size_type   f_count ///< The number of objects to be deallocated from storage. 
                                    );

        //----------------------------
        // construction/destruction
        //----------------------------

        /// Constructs a specific type of object at a specified address that is initialized with a specified value.
        void            construct   (pointer f_ptr, const value_type& f_val);
        
        /// Calls an objects destructor without deallocating the memory where the object was stored.
        void            destroy     (pointer f_ptr);

        //----------------------------
        // utilities 
        //----------------------------

        /// Finds the address of an object whose value is specified.
        pointer         address     (reference       f_val)  const;
        
        /// Finds the address of an object whose value is specified.
        const_pointer   address     (const_reference f_val)  const;
        
        /// Returns template argument SizeValue.
        size_type       max_size    (void)   const;

    private:
        typedef TAlignedStorage<SizeValue*sizeof(ValueType)>    storage_type;
  
    private:

        //----------------------------
        // convenience methods
        //----------------------------

        pointer         begin   (void);
        const_pointer   begin   (void)  const;
        pointer         end     (void);
        const_pointer   end     (void)  const;
    
    private:
        storage_type    m_memblock;
        pointer         m_brk;
    };

    /// test for allocator equality (always true). @relates     TFixedBlockAllocator
    template<   class Value1Type, int32_t Size1Value, 
                class Value2Type, int32_t Size2Value> inline
    bool operator== (   const TFixedBlockAllocator<Value1Type, Size1Value>& f_alloc1, 
                        const TFixedBlockAllocator<Value2Type, Size2Value>& f_alloc2)
    {    
        // allocators are equal if and only if they are the identical object
        // we have to cast to void* because it is possible that 
        // we are comparing allocators of different types
        return (static_cast<const void*>(&f_alloc1) == static_cast<const void*>(&f_alloc2));
    }

    /// test for allocator inequality (always false). @relates     TFixedBlockAllocator
    template<   class Value1Type, int32_t Size1Value, 
                class Value2Type, int32_t Size2Value> inline
    bool operator!= (   const TFixedBlockAllocator<Value1Type, Size1Value>& f_alloc1, 
                        const TFixedBlockAllocator<Value2Type, Size2Value>& f_alloc2)
    {    
        // just negate equality
        return !(f_alloc1 == f_alloc2);
    }


}   // namespace vfc closed

#include "vfc/memory/vfc_fixedblock_allocator.inl"


#endif //VFC_FIXEDBLOCK_ALLOCATOR_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fixedblock_allocator.hpp  $
//  Revision 1.14 2016/11/28 09:12:16MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Improve const-correctness of Fixedblock-Allocator(mantis0005273)
//  Revision 1.13 2012/12/18 08:27:34MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.12 2008/08/29 15:07:10MESZ Gaurav Jain (RBEI/ESB3) (gaj2kor) 
//  Addition of missing destructor. (Mantis :1718)
//  Revision 1.11 2008/08/25 20:33:55IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed max_size() implementation und updated docs (mantis 2311)
//  Revision 1.10 2007/07/23 10:23:21CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - added missing #include (mantis1747)
//  Revision 1.9 2007/07/23 09:55:23CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.8 2007/03/13 11:41:29CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added m_memblock() to c'tor initialization list (mantis1494)
//  Revision 1.7 2007/03/08 11:08:43CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - additional fix (mantis1486)
//  Revision 1.6 2007/03/08 11:04:12CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - fixed wrong type of rebind2's template argument (mantis1486)
//  - added docu
//  Revision 1.5 2007/02/15 09:34:27CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced uint32_t with int32_t 
//  - added comparison functions
//  Revision 1.4 2006/11/16 14:41:21CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.3 2006/10/12 10:47:44CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -changed header/footer templates
//  -changed documentation style
//  Revision 1.2 2006/10/12 10:33:54CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -fixed bug in assignment operator (mantis1203)
//=============================================================================
