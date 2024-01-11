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
///     $Source: vfc_fixedplacementmempool.hpp $
///     $Revision: 1.2 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2012/12/14 10:05:01MEZ $
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

#ifndef VFC_FIXEDPLACEMENTMEMPOOL_HPP_INCLUDED
#define VFC_FIXEDPLACEMENTMEMPOOL_HPP_INCLUDED

// vfc core includes
#include "vfc/core/vfc_types.hpp"           // used for uint32_t, ptrdiff_t, 
#include "vfc/core/vfc_static_assert.hpp"   // used for VFC_STATIC_ASSERT
#include "vfc/core/vfc_aligned_storage.hpp" // used for TAlignedStorage<>

// vfc/memory includes
#include "vfc/memory/vfc_fixedmempool.hpp"  // used as base class

namespace vfc
{   // namespace vfc opened

    //=============================================================================
    //  TFixedPlacementMempool<>
    //-----------------------------------------------------------------------------
    //! TFixedPlacementMempool is a user defined memory manager for fast 
    //! allocation/deallocation of objects of same size. 
    //! @par Description:
    //! TFixedMempool2 manages a fixed sized block of memory. It uses an embedded 
    //! freelist for constant time allocation/deallocation.
    //! Use this class if your application allocates/deallocates a huge number of 
    //! small objects at runtime (e.g. particle systems) or if your target platform.
    //! It inherits from TFixedMempool and extends the functionality through a 
    //! placement malloc, where the user can decide which chunk of memory to use for
    //! allocation
    //! has no heap or free store.
    //! @par
    //! @image html memory-fixedmempool-01.png
    //!
    //! @param ChunkSizeValue  chunk size in bytes
    //! @param ChunkCountValue  maximum number of chunks
    //! @author zvh2hi
    //! @ingroup vfc_group_memory_managers
    //=============================================================================

    template <int32_t ChunkSizeValue, int32_t ChunkCountValue>
    class TFixedPlacementMempool : public TFixedMempool<ChunkSizeValue, ChunkCountValue>
    {
        // template argument checks
        VFC_STATIC_ASSERT(0 < ChunkSizeValue);
        VFC_STATIC_ASSERT(0 < ChunkCountValue);
        typedef TFixedMempool<ChunkSizeValue, ChunkCountValue> baseClass_t;
    public: 
        // public enums 
        enum { MAX_CHUNK_SIZE   = baseClass_t::MAX_CHUNK_SIZE};                  //!< size of a single chunk in bytes
        enum { MAX_CHUNK_COUNT  = baseClass_t::MAX_CHUNK_COUNT};                 //!< total number of chunks
        enum { MAX_STORAGE_SIZE = baseClass_t::MAX_STORAGE_SIZE};  //!< resulting storage size in bytes
        
        typedef typename baseClass_t::size_type   size_type;

        typedef typename baseClass_t::offset_type  offset_type;

    public: 
        // public methods

        //! default c'tor, initializes embedded free list.
        TFixedPlacementMempool(void):baseClass_t() {}

        //! copy c'tor (copies nothing but initializes embedded free list).
        TFixedPlacementMempool(const TFixedPlacementMempool&):baseClass_t() {}

        //! d'tor.
        ~TFixedPlacementMempool(void);

        //! assignment op (does nothing).
        const TFixedPlacementMempool& operator=(const TFixedPlacementMempool&) { return *this;}
        
        //-------------------------------------------------------------------------------------------
        //! The placement_malloc function allocates a memory block of at least f_size bytes at the 
        //! given chunk aligned memory address, if the given address is in the free list
        //! To return a pointer to a type other than void, use a type cast on the return value. 
        //! The storage space pointed to by the return value is aligned to the given chunk size.
        //! If size is 0, placement_malloc allocates a complete chunk and returns a valid pointer to that item.
        //! If pointer given is not aligned to chunk size placement_malloc returns 0 pointer
        //! Always check the return from placement_malloc.
        //! @note
        //! In order to successfully do a placement malloc the given void pointer has to be aligned
        //! to the chunk size and has to be inside the memory block
        //! \return
        //! placement_malloc returns a void pointer to the allocated space, 
        //! or 0 if there is insufficient chunks available, f_size exceeds the specified chunk size,
        //! the given chunk pointer is not in the free list or it is not aligned to chunk size
        //-------------------------------------------------------------------------------------------
        void*   placement_malloc  (   size_type f_count,     //!< number of bytes to allocate
            //! address of chunk to use for allocation (chunk aligned, only members of the free list)
            const void* const f_chunkAddr_p );

    private:
        // private methods
        //! given an offset in the free list return the offset to the next free list element
        offset_type getNextOffset(offset_type f_offset)const;
        //! find given chunk offset in the free list and return that and the previous chnuk offset in list
        bool searchChunkOffsetInFreelist(offset_type f_offset, offset_type& f_prevOffset);
    };

    
}   // namespace vfc closed
#include "vfc/memory/vfc_fixedplacementmempool.inl"

#endif //VFC_FIXEDPLACEMENTMEMPOOL_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fixedplacementmempool.hpp  $
//  Revision 1.2 2012/12/14 10:05:01MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - missing fixedmempool #include (mantis 4194)
//  Revision 1.1 2012/12/11 13:11:38MEZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/memory/memory.pj
//  Revision 1.2 2012/12/07 09:26:45MEZ Alaa El-Din Omar (CC/ESV2) (ALO2HI) 
//  - mantis 0004190:
//    -- changes in implementation for faster speed
//    -- removed methods only needed for tests
//  Revision 1.1 2012/12/05 13:40:55MEZ Alaa El-Din Omar (CC/ESV2) (ALO2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/private_workspace/alo2hi/vfc/pc_ivs2_vfc/devl/include/vfc/memory/memory.pj
//  Revision 1.9 2010/10/06 16:10:37MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed member order (mantis3467)
//  Revision 1.8 2008/08/11 10:10:56MESZ Zitzewitz Henning von (CR/AEM6) (ZVH2HI) 
//  - added image for fixedmempool docs (mantis2182)
//  Revision 1.7 2007/07/23 09:55:48CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.6 2007/07/17 07:55:44CEST dkn2kor 
//  - added memory leak check (mantis1736)
//  Revision 1.5 2007/05/08 14:22:03IST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - fixed docu (mantis1632)
//  Revision 1.4 2007/02/15 09:33:19CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - heavily reworked errorhandling (mantis1430)
//  - replaced uint32_t with int32_t 
//  - replaced C style comments with C++ comments
//  Revision 1.3 2006/11/16 14:41:06CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.2 2006/04/10 15:37:20CEST Dilipkrishna Natesan (RBIN/EAE3 AE-DA/ESA3) * (DIN1LR) 
//  -added vfc namespace name to resolve compilation error in rational realtime (mantis1052)
//  Revision 1.1 2006/03/02 11:31:04CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/memory/memory.pj
//=============================================================================
