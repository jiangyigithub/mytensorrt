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
///     $Source: vfc_freestore_allocator.inl $
///     $Revision: 1.6 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (ZVH2HI) $
///     $Date: 2008/10/23 12:39:58MESZ $
///     $Locker:  $
///     $Name: 0032 RC1  $
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

// libc
#include <new>  // used for placement new()
#include <limits>

// core
#include "vfc/core/vfc_assert.hpp" // used for VFC_REQUIRE
#include "vfc/core/vfc_util.hpp" // used for nop()

template<class ValueType> inline
typename vfc::TFreeStoreAllocator<ValueType>::pointer 
vfc::TFreeStoreAllocator<ValueType>::address(reference f_arg) const  
{
    return (&f_arg);
}

template<class ValueType> inline
typename vfc::TFreeStoreAllocator<ValueType>::const_pointer 
vfc::TFreeStoreAllocator<ValueType>::address(const_reference f_arg) const  
{
    return (&f_arg);
}
        
template<class ValueType> inline
typename vfc::TFreeStoreAllocator<ValueType>::pointer 
vfc::TFreeStoreAllocator<ValueType>::allocate(size_type f_count)
{
    VFC_REQUIRE(0 <= f_count);
    return static_cast<value_type*>(operator new(f_count * sizeof (value_type)));
}

template<class ValueType> inline
typename vfc::TFreeStoreAllocator<ValueType>::pointer 
vfc::TFreeStoreAllocator<ValueType>::allocate(size_type f_count, const void *)
{
    VFC_REQUIRE(0 <= f_count);

    // ignoring allocation hint
    return (allocate(f_count));
}

template<class ValueType> inline
void
vfc::TFreeStoreAllocator<ValueType>::construct(pointer f_ptr, const ValueType& f_arg)
{
    VFC_REQUIRE( 0 != f_ptr );

    new (static_cast<void*>(f_ptr)) value_type(f_arg);
}

template<class ValueType> inline
void
vfc::TFreeStoreAllocator<ValueType>::destroy(pointer f_ptr)
{
    VFC_REQUIRE( 0 != f_ptr );
    vfc::nop(f_ptr); // satisfying vc_80
    f_ptr->~value_type();
}

template<class ValueType> inline
void
vfc::TFreeStoreAllocator<ValueType>::deallocate(pointer f_ptr, size_type)
{
    operator delete(f_ptr);
}

template<class ValueType> inline
typename vfc::TFreeStoreAllocator<ValueType>::size_type 
vfc::TFreeStoreAllocator<ValueType>::max_size() const
{
    return (stlalias::numeric_limits<size_type>::max)() / sizeof (ValueType);
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_freestore_allocator.inl  $
//  Revision 1.6 2008/10/23 12:39:58MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - fixed compiler warnings (mantis 2401)
//  Revision 1.5 2008/08/25 14:03:19CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed max_size() implementation (mantis 2310)
//  - replaced tabs with spaces (mantis 1294)
//  Revision 1.4 2008/08/11 12:53:45CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added missing include (mantis2279)
//  Revision 1.3 2007/08/16 15:07:11CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - added additional paranthesis to max()  (mantis1767)
//  Revision 1.2 2007/03/29 15:36:16CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced ::std with stlalias (mantis1534)
//=============================================================================
