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
//       Projectname:  vfc
//          Synopsis:
//  Target system(s):
//       Compiler(s):  c++ std conformal
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
///     $Source: vfc_fixedmempool_allocator.inl $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2010/08/11 17:46:34MESZ $
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

template <class ValueType, vfc::int32_t SizeValue>   inline
vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::TFixedMemPoolAllocator(void)
: m_memPool()
{
    // intentionally left blank
}

template <class ValueType, vfc::int32_t SizeValue>   inline
vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::TFixedMemPoolAllocator (const TFixedMemPoolAllocator&)
: m_memPool()
{
    // intentionally left blank
}

template <class ValueType, vfc::int32_t SizeValue>   inline
typename vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::pointer
vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::address(reference       f_val)  const
{
    return (&f_val);
}

template <class ValueType, vfc::int32_t SizeValue>   inline
typename vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::const_pointer
vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::address(const_reference f_val)  const
{
    return (&f_val);
}

template <class ValueType, vfc::int32_t SizeValue>   inline
void    vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::deallocate  (pointer f_ptr, size_type VFC_USE_VAR_ONLY_IN_ASSERTION(f_count))
{
    VFC_REQUIRE(1 == f_count);
    VFC_REQUIRE(0 != f_ptr);
    m_memPool.free(f_ptr);
}

template <class ValueType, vfc::int32_t SizeValue>   inline
typename vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::pointer
vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::allocate    (size_type VFC_USE_VAR_ONLY_IN_ASSERTION(f_count))
{
    VFC_REQUIRE(1 == f_count);

    value_type* currPtr_p = static_cast<value_type*>(m_memPool.malloc(ELEMENT_SIZE));

    return currPtr_p;
}

template <class ValueType, vfc::int32_t SizeValue>   inline
typename vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::pointer
vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::allocate    (size_type f_count, const void*)
{
    return (allocate(f_count));
}

template <class ValueType, vfc::int32_t SizeValue>   inline
void    vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::construct   (pointer f_ptr, const value_type& f_val)
{
    // just call placement new
    new (f_ptr) value_type(f_val);
}

template <class ValueType, vfc::int32_t SizeValue>   inline
void    vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::destroy     (pointer f_ptr)
{
    // calls objects d'tor explicitly
    f_ptr->~value_type();
}

template <class ValueType, vfc::int32_t SizeValue>   inline
typename vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::size_type
vfc::TFixedMemPoolAllocator<ValueType,SizeValue>::max_size    (void)          const
{
    //return value of max_size() is constant at runtime
    return static_cast<size_type>(SizeValue);
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fixedmempool_allocator.inl  $
//  Revision 1.1 2010/08/11 17:46:34MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/memory/memory.pj
//  Revision 1.6 2009/05/11 07:38:45MESZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Introduced new macro for the suppression of compiler warning.(mantis:2832)
//  Revision 1.5 2008/08/29 18:42:03IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  - max_size() must return the SizeValue
//  - allocate (size_type f_count) should not have the VFC_ENSURE postcondition. (Mantis 2013)
//  Revision 1.4 2008/01/11 16:46:34IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  DoxyComments are changed
//  Mantis Id :- 0001807
//  Revision 1.3 2007/10/31 18:47:40IST vmr1kor
//  comments/docu to vfc_fixedmempool.hpp
//  Rename SizeValue , SIZEVALUE enum member
//  max_size() may only return 1 or 0 (ISO 14882 § 20.1.5)
//  Mantis Id :- 0001807
//  Revision 1.2 2007/06/22 17:43:48IST dkn2kor
//  - allocate and deallocate count changed to 1
//  Revision 1.1 2007/06/18 13:19:52IST dkn2kor
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_list/vfc_list.pj
//=============================================================================
