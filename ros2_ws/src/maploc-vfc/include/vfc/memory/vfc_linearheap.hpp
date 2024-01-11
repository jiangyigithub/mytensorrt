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
//        Name: zvh2hi
//  Department: CR/AEM
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_linearheap.hpp $
///     $Revision: 1.16 $
///     $Author: gaj2kor $
///     $Date: 2008/09/11 13:58:08MESZ $
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

#ifndef VFC_LINEARHEAP_HPP_INCLUDED
#define VFC_LINEARHEAP_HPP_INCLUDED

// vfc includes
#include "vfc/core/vfc_types.hpp"           // used for size_t, max_aligned_t
#include "vfc/core/vfc_type_traits.hpp"     // used for TAlignmentOf<>
#include "vfc/core/vfc_metaprog.hpp"        // used for TIsPow2<>
#include "vfc/core/vfc_static_assert.hpp"   // used for VFC_STATIC_ASSERT
#include "vfc/core/vfc_aligned_storage.hpp" // used for TAlignedStorage<>

namespace vfc
{    // namespace vfc opened

    //=============================================================================
    //  TLinearHeap<>
    //-----------------------------------------------------------------------------
    //! Linear Heap Implementation.
    //! @par Description:
    //! A Linear Heap is a contiguous memory area divided into allocated and
    //! unallocated portions.
    //! A break pointer (brk) contains the adress of the unallocated portion. It
    //! initially points to the beginning of the heap.
    //! You allocate memory by moving the pointer upwards and you free memory by
    //! moving the pointer downwards (stack-like).
    //! @note
    //! In order to succesfully reclaim previously allocated memory, the blocks have
    //! to be freed in the reverse order they have been allocated.
    //! @par Allocation:
    //! @image html memory-linearheap-allocation-01.png
    //! @par Deallocation:
    //! @image html memory-linearheap-deallocation-01.png
    //!
    //! @param SizeValue  reserved heap size in bytes.
    //! @param AlignValue specified byte alignment for allocation with malloc, has to
    //!                   be a power of two. default: alignment of max_aligned_t
    //! @author zvh2hi
    //! @ingroup vfc_group_memory_managers
    //=============================================================================

    template    <   int32_t SizeValue,
                    int32_t AlignValue = TAlignmentOf<max_aligned_t>::value
                >
    class TLinearHeap
    {
        // template argument checks
        VFC_STATIC_ASSERT   ( 0 < SizeValue );
        VFC_STATIC_ASSERT   ( 0 < AlignValue );
        VFC_STATIC_ASSERT   ( TIsPow2<AlignValue>::value );

    public:

        enum    {    SIZE = SizeValue};
        enum    {    ALIGN = AlignValue};

        typedef int32_t size_type;

    public:
        //! default c'tor.
        TLinearHeap    (void);

        //! copy c'tor.
        TLinearHeap    (const TLinearHeap<SizeValue,AlignValue>& );

        //! d'tor.
        ~TLinearHeap    (void);

        //! asignment operator (does nothing).
        const TLinearHeap<SizeValue,AlignValue>&    operator= (const TLinearHeap<SizeValue,AlignValue>& )    { return *this;}

        //-----------------------------------------------------------------------------
        //! allocates n-bytes from linear heap, returns a pointer to the initial
        //! element of an contiguous memory block.
        //! Memory is appropriately aligned for all object types.
        //! @return pointer to allocated storage or 0 pointer if storage cannot be obtained.
        //-----------------------------------------------------------------------------
        void*    malloc    (size_type f_count);

        //-----------------------------------------------------------------------------
        //! frees the storage referenced by f_memblock_vp.
        //! f_memblock_vp shall be a pointer value obtained by malloc().
        //! f_count shall equal the value passed as the argument to the invocation
        //! of allocate which returned f_memblock_vp.
        //! Storage can only be reclaimed, if deallocation is performed in reverse
        //! order of allocation and the condition (f_memblock_vp+f_count) == m_brk is true.
        //-----------------------------------------------------------------------------
        void    free    (void* f_memblock_vp, size_type f_count);

        //! returns the largest value N for which the call allocate(N) might succeed.
        size_type    max_size         (void)    const;

        //! returns the number of currently allocated bytes.
        size_type    allocated_size   (void)    const    {    return m_allocated;}

        //! returns the percentage [0,1] of internal fragmentation
        //! (due to alignment and/or unreachable areas of deallocated memory).

        float64_t    fragmentation    (void)    const;

    private:

        //! adjusts the brk pointer to the first element of the storage block.
        void    init (void);

        //! aligns n upwards the nearest multiple of ALIGN .
        size_t    align_up      (size_t f_n)    const {    return (f_n + (ALIGN-1)) & (~(ALIGN-1));}

    private:
        //! aligned raw storage.
        TAlignedStorage<SIZE>    m_storage;
        //! break pointer, contains address of unallocated memory block, see UNIX sbrk() for details.
        char_t*     m_brk;
        //! allocation counter for statistics and debugging.
        size_type   m_allocated;
    };

    //=============================================================================
    //  TGenLinearHeap<>
    //-----------------------------------------------------------------------------
    //! type generator for generating a TLinearHeap<> type with optimal size and
    //! alignment for given ValueType and SizeValue.
    //! @param ValueType    type of objects to be stored inside the heap
    //! @param SizeValue    maximum number of objects
    //! @sa TLinearHeap
    //! @author zvh2hi
    //! @ingroup vfc_group_memory_manager
    //=============================================================================

    template <class ValueType, size_t SizeValue>
    struct TGenLinearHeap
    {
        typedef TLinearHeap<SizeValue*sizeof(ValueType),TAlignmentOf<ValueType>::value> type;
    };

}    // namespace vfc closed

#include "vfc/memory/vfc_linearheap.inl"

#endif //VFC_LINEARHEAP_HPP_INCLUDED


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linearheap.hpp  $
//  Revision 1.16 2008/09/11 13:58:08MESZ gaj2kor 
//  Updated the comment for copy constructor.
//  (Mantis : 1634)
//  Revision 1.15 2008/08/11 21:15:58IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - fixed bug in assignment op (mantis2213)
//  Revision 1.14 2008/08/11 10:23:16CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added images to docs (mantis2182)
//  Revision 1.13 2007/07/23 09:57:23CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - doxygen grouping (mantis1744)
//  Revision 1.12 2007/05/04 14:58:38CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - removed align_down() methof (mantis1613)
//  - added paranthesis to align_up() method (mantis1614)
//  Revision 1.11 2007/02/15 09:26:07CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - heavily reworked errorhandling (mantis1430)
//  - replaced C stlye comments with C++ comments
//  - cleaned-up include dependencies
//  Revision 1.10 2006/12/07 10:20:21CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced size_t with int32_t (mantis1325)
//  Revision 1.9 2006/11/16 14:41:06CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.8 2006/03/15 08:53:53CET Dilipkrishna Natesan (RBIN/EAE3 AE-DA/ESA3) * (DIN1LR)
//  -function name allocate and deallocate changed to malloc and free
//  Revision 1.7 2006/03/03 10:31:00CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added missing include "vfc_type_traits.hpp" (mantis1017)
//  Revision 1.6 2006/01/27 13:18:33CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -renamed non-tos error policies according cr/aem coding conventions
//  -added documentation
//  Revision 1.5 2005/10/28 10:32:09CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -changed directory of core files to vfc\include\core\...
//  Revision 1.4 2005/10/06 17:02:01CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/memory/memory.pj
//  Revision 1.3 2005/04/19 11:11:07CEST zvh2hi
//  fixed macro error in SNonTOSThrow
//  Revision 1.2 2005/04/18 17:14:46CEST zvh2hi
//  added non-TOS error policy
//  Revision 1.1 2005/04/18 12:00:30CEST zvh2hi
//  Initial revision
//  Member added to project /import/mks/data/projects/cv/vfc/include/vfc/memory/memory.pj
//=============================================================================

