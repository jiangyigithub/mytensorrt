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
///     $Source: vfc_fixedblock_allocator.inl $
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

#include <new>                      // used for placement new()
#include "vfc/core/vfc_assert.hpp"  // used for VFC_ASSERT(), VFC_REQUIRE(), VFC_ENSURE()
#include "vfc/core/vfc_util.hpp"    // used for nop

template <class ValueType, vfc::int32_t SizeValue>   inline
vfc::TFixedBlockAllocator<ValueType,SizeValue>::TFixedBlockAllocator(void) 
: m_memblock(), m_brk(begin())   
{
    // intentionally left blank
}

template <class ValueType, vfc::int32_t SizeValue>   inline
vfc::TFixedBlockAllocator<ValueType,SizeValue>::TFixedBlockAllocator (const TFixedBlockAllocator&) 
: m_memblock(), m_brk(begin()) 
{   
    // intentionally left blank
}

template <class ValueType, vfc::int32_t SizeValue>   inline
typename vfc::TFixedBlockAllocator<ValueType,SizeValue>::pointer         
vfc::TFixedBlockAllocator<ValueType,SizeValue>::address(reference       f_val)  const   
{   
    return (&f_val);
}

template <class ValueType, vfc::int32_t SizeValue>   inline
typename vfc::TFixedBlockAllocator<ValueType,SizeValue>::const_pointer   
vfc::TFixedBlockAllocator<ValueType,SizeValue>::address(const_reference f_val)  const   
{    
    return (&f_val);
}

template <class ValueType, vfc::int32_t SizeValue>   inline
void    vfc::TFixedBlockAllocator<ValueType,SizeValue>::deallocate  (pointer const f_ptr, size_type f_count)  
{   
    VFC_REQUIRE(0 <= f_count);
    // check if pointer lies in our range
    VFC_REQUIRE( (f_ptr >= begin()) && ((f_ptr+f_count) <= m_brk) );

    // check if the specified memory block is the last allocated block
    if ( (f_ptr + f_count) == m_brk )
    {
        // it is, so it is safe to reclaim memory
        m_brk -= f_count;
    }

    VFC_ENSURE(begin() <= m_brk);
}

template <class ValueType, vfc::int32_t SizeValue>   inline
typename vfc::TFixedBlockAllocator<ValueType,SizeValue>::pointer         
vfc::TFixedBlockAllocator<ValueType,SizeValue>::allocate    (size_type f_count)                 
{  
    VFC_REQUIRE (0 <= f_count);

    // this is a pre-condition which can be checked by caller -> assert()
    // because we are implementing a memory allocation function we return a 0 pointer instead

    if (f_count > max_size())
    {
        return 0;
    }
    
    // backup brk pointer
    pointer oldbrk = m_brk;
    
    // and adjust it to the end of the allocated area
    m_brk += f_count;
    
    VFC_ENSURE(end() >= m_brk);

    return oldbrk;
}

template <class ValueType, vfc::int32_t SizeValue>   inline
typename vfc::TFixedBlockAllocator<ValueType,SizeValue>::pointer         
vfc::TFixedBlockAllocator<ValueType,SizeValue>::allocate    (size_type f_count, const void*)            
{   
    return (allocate(f_count));    
}

template <class ValueType, vfc::int32_t SizeValue>   inline
void    vfc::TFixedBlockAllocator<ValueType,SizeValue>::construct   (pointer f_ptr, const value_type& f_val)    
{   
    // just call placement new
    new (f_ptr) value_type(f_val);
}

template <class ValueType, vfc::int32_t SizeValue>   inline
void    vfc::TFixedBlockAllocator<ValueType,SizeValue>::destroy     (pointer f_ptr)         
{   
    VFC_REQUIRE( 0 != f_ptr );
    // calls objects d'tor explicitly 
    vfc::nop(f_ptr);
    f_ptr->~value_type();
}


template <class ValueType, vfc::int32_t SizeValue>   inline
typename vfc::TFixedBlockAllocator<ValueType,SizeValue>::size_type         
vfc::TFixedBlockAllocator<ValueType,SizeValue>::max_size    (void)          const   
{   
    return SizeValue;
}

template <class ValueType, vfc::int32_t SizeValue>   inline
typename vfc::TFixedBlockAllocator<ValueType,SizeValue>::pointer         
vfc::TFixedBlockAllocator<ValueType,SizeValue>::begin(void)         
{   
    return static_cast<pointer>(m_memblock.begin());
}

template <class ValueType, vfc::int32_t SizeValue>   inline
typename vfc::TFixedBlockAllocator<ValueType,SizeValue>::const_pointer         
vfc::TFixedBlockAllocator<ValueType,SizeValue>::begin(void) const   
{   
    return static_cast<const_pointer>(m_memblock.begin());
}

template <class ValueType, vfc::int32_t SizeValue>   inline
typename vfc::TFixedBlockAllocator<ValueType,SizeValue>::pointer         
vfc::TFixedBlockAllocator<ValueType,SizeValue>::end(void)           
{   
    return static_cast<pointer>(m_memblock.end());
}

template <class ValueType, vfc::int32_t SizeValue>   inline
typename vfc::TFixedBlockAllocator<ValueType,SizeValue>::const_pointer         
vfc::TFixedBlockAllocator<ValueType,SizeValue>::end(void)   const   
{   
    return static_cast<const_pointer>(m_memblock.end());
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fixedblock_allocator.inl  $
//  Revision 1.11 2016/11/28 09:12:16MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Improve const-correctness of Fixedblock-Allocator(mantis0005273)
//  Revision 1.10 2008/10/23 12:39:58MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - fixed compiler warnings (mantis 2401)
//  Revision 1.9 2008/09/01 16:03:26CEST Dhananjay N (RBEI/EAE6) (dhn1kor) 
//  Precedence confusion in QAC++ 2.5 Warning Rule 8.0.3 is removed.(mantis2217)
//  Revision 1.8 2008/08/25 20:33:55IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed max_size() implementation und updated docs (mantis 2311)
//  Revision 1.7 2007/03/13 11:41:32CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added m_memblock() to c'tor initialization list (mantis1494)
//  Revision 1.6 2007/02/15 09:34:58CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - heavily reworked errorhandling (mantis1430)
//  - replaced uint32_t with int32_t 
//  Revision 1.5 2006/11/16 14:41:08CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.4 2006/11/06 13:35:46CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added an assert and a static_cast to fix type mismatch (mantis 1254)
//  Revision 1.3 2006/11/02 16:34:18CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added missing include (mantis1246)
//  Revision 1.2 2006/10/12 10:47:44CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -changed header/footer templates
//  -changed documentation style
//=============================================================================
