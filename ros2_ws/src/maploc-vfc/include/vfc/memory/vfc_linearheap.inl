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
///     $Source: vfc_linearheap.inl $
///     $Revision: 1.12 $
///     $Author: gaj2kor $
///     $Date: 2009/02/03 12:53:35MEZ $
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

//libc includes
#include <string.h>                      // used for memset()

// vfc includes
#include "vfc/core/vfc_assert.hpp"      // used for VFC_ASSERT2(), VFC_REQUIRE()

template <vfc::int32_t SizeValue, vfc::int32_t AlignValue>    inline
vfc::TLinearHeap<SizeValue, AlignValue>::TLinearHeap(void)
:   m_brk (0),
    m_allocated(0)
{
    init();
}

template <vfc::int32_t SizeValue, vfc::int32_t AlignValue>    inline
vfc::TLinearHeap<SizeValue, AlignValue>::TLinearHeap(const TLinearHeap<SizeValue, AlignValue>& )
:   m_brk (0),
    m_allocated(0)
{
    init();
}

template <vfc::int32_t SizeValue, vfc::int32_t AlignValue>    inline
vfc::TLinearHeap<SizeValue, AlignValue>::~TLinearHeap(void)
{
    VFC_ENSURE2( 0 == m_allocated, "possible memory leakage in TLinearHeap<>~LinearHeap()" );
}

template <vfc::int32_t SizeValue, vfc::int32_t AlignValue>    inline
void*    vfc::TLinearHeap<SizeValue, AlignValue>::malloc    (size_type f_count)
{
    VFC_REQUIRE( 0 <= f_count );

    // this is a pre-condition which can be checked by caller -> assert()
    // because we are implementing a memory allocation function we return a 0 pointer instead

    if (max_size() < f_count)
    {
        return 0;
    }

    // calculate alignment
    size_t  aligned_count   = align_up(f_count);

    // save break pointer
    void*   old_brk         = static_cast<void*>(m_brk);

    // according UNIX definition of sbrk, a new block is cleared
    ::memset(old_brk,0,f_count);

    // adjust allocation counter
    m_allocated += f_count;

    // adjust sbrk
    m_brk       += aligned_count;

    // returns saved sbrk
    return old_brk;
}

template <vfc::int32_t SizeValue, vfc::int32_t AlignValue>    inline
void    vfc::TLinearHeap<SizeValue, AlignValue>::free    (void* f_memblock_vp, size_type f_count)
{
    VFC_REQUIRE ( 0 <= f_count );

    char_t* memblock_cp = static_cast<char_t*>(f_memblock_vp);

    // check, if memblock is within our memory range
    VFC_REQUIRE ( (memblock_cp >= m_storage.begin()) && ( (memblock_cp+f_count) <= (m_brk) ) );

    // calculate alignment
    size_t aligned_count = align_up(f_count);

    // decrement allocation count
    m_allocated -= f_count;

    // deallocation from TOS, so we can reclaim some memory
    if (( memblock_cp+aligned_count) == m_brk)
    {
        m_brk -= aligned_count;
    }
}

template <vfc::int32_t SizeValue, vfc::int32_t AlignValue>    inline
typename vfc::TLinearHeap<SizeValue, AlignValue>::size_type
vfc::TLinearHeap<SizeValue, AlignValue>::max_size    (void) const
{
    const char_t* end_p = static_cast<const char_t*>(m_storage.end());
    return (m_brk <= end_p)?static_cast<int32_t>(end_p-m_brk):0;
}

template <vfc::int32_t SizeValue, vfc::int32_t AlignValue>    inline
vfc::float64_t    vfc::TLinearHeap<SizeValue, AlignValue>::fragmentation    (void)    const
{
    const int32_t  used = SIZE-max_size();

    if (0 != used)
    {
        return 1.-static_cast<float64_t>(allocated_size())/used;
    }
    else
    {
        return 0.;
    }
}
template <vfc::int32_t SizeValue, vfc::int32_t AlignValue>    inline
void    vfc::TLinearHeap<SizeValue, AlignValue>::init (void)
{
    m_brk = static_cast<char_t*>(m_storage.begin());
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linearheap.inl  $
//  Revision 1.12 2009/02/03 12:53:35MEZ gaj2kor 
//  -Removal of QAC++ warning.
//  (Mantis : 0002447)
//  Revision 1.11 2009/01/22 16:40:15IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Removal of unused variables.
//  (Mantis : 0002462)
//  Revision 1.10 2007/07/17 11:26:25IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  - assert changed to ensure (mantis1737)
//  Revision 1.9 2007/03/29 19:08:07IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced ::std with global namespace (mantis1534)
//  Revision 1.8 2007/02/15 17:06:57CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - fixed void* pointer arithmetic bug (mantis1434)
//  Revision 1.7 2007/02/15 09:27:00CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - heavily reworked errorhandling (mantis1430)
//  - replaced old header/footers
//  Revision 1.6 2006/12/07 10:20:22CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced size_t with int32_t (mantis1325)
//  Revision 1.5 2006/11/16 14:41:10CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.4 2006/03/15 08:54:37CET Dilipkrishna Natesan (RBIN/EAE3 AE-DA/ESA3) * (DIN1LR)
//  -function name allocate and deallocate changed to malloc and free
//  added file header and revision history.
//=============================================================================
