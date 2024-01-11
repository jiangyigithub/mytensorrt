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
///     $Source: vfc_fixedmempool.hpp $
///     $Revision: 1.11 $
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

#ifndef VFC_FIXEDMEMPOOL_HPP_INCLUDED
#define VFC_FIXEDMEMPOOL_HPP_INCLUDED

// vfc core includes
#include "vfc/core/vfc_types.hpp"           // used for uint32_t, ptrdiff_t, 
#include "vfc/core/vfc_static_assert.hpp"   // used for VFC_STATIC_ASSERT
#include "vfc/core/vfc_aligned_storage.hpp" // used for TAlignedStorage<>

namespace vfc
{   // namespace vfc opened

    //=============================================================================
    //  TFixedMempool<>
    //-----------------------------------------------------------------------------
    //! TFixedMempool is a user defined memory manager for fast 
    //! allocation/deallocation of objects of same size. 
    //! @par Description:
    //! TFixedMempool manages a fixed sized block of memory. It uses an embedded 
    //! freelist for constant time allocation/deallocation.
    //! Use this class if your application allocates/deallocates a huge number of 
    //! small objects at runtime (e.g. particle systems) or if your target platform 
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
    class TFixedMempool 
    {
        // template argument checks
        VFC_STATIC_ASSERT(0 < ChunkSizeValue);
        VFC_STATIC_ASSERT(0 < ChunkCountValue);

    public: 
        // public enums 
        enum { MAX_CHUNK_SIZE   = ChunkSizeValue};                  //!< size of a single chunk in bytes
        enum { MAX_CHUNK_COUNT  = ChunkCountValue};                 //!< total number of chunks
        enum { MAX_STORAGE_SIZE = MAX_CHUNK_SIZE*MAX_CHUNK_COUNT};  //!< resulting storage size in bytes
        
        typedef int32_t   size_type;

    public: 
        // public methods

        //! default c'tor, initializes embedded free list.
        TFixedMempool(void) { init();}

        //! copy c'tor (copies nothing but initializes embedded free list).
        TFixedMempool(const TFixedMempool&) { init();}

        //! d'tor.
        ~TFixedMempool(void);

        //! assignment op (does nothing).
        const TFixedMempool& operator=(const TFixedMempool&) { return *this;}
        
        //-------------------------------------------------------------------------------------------
        //! The malloc function allocates a memory block of at least f_size bytes. 
        //! To return a pointer to a type other than void, use a type cast on the return value. 
        //! The storage space pointed to by the return value is aligned to the given chunk size.
        //! If size is 0, malloc allocates a complete chunk and returns a valid pointer to that item. 
        //! Always check the return from malloc.
        //! \return
        //! malloc returns a void pointer to the allocated space, 
        //! or 0 if there is insufficient chunks available or f_size exceeds the specified chunk size. 
        //-------------------------------------------------------------------------------------------

        void*   malloc  (   size_type f_count     //!< number of bytes to allocate
                        );

        //-------------------------------------------------------------------------------------------
        //! Deallocates or frees a memory block.
        //! The free function deallocates a memory block (memblock) that was previously allocated 
        //! by a call to malloc. 
        //! If memblock is 0, the pointer is ignored and free immediately returns. 
        //-------------------------------------------------------------------------------------------

        void    free    (   void* f_memblock_vp //!< previously allocated memory block to be freed 
                        );

        //! returns the number of used (allocated) chunks.
        size_type    getUsedChunks   (void)   const   {   return m_usedChunks;}
        
        //! returns the total number of chunks (allocated and free).
        size_type    getTotalChunks  (void)   const   {   return MAX_CHUNK_COUNT;}
        
        //! returns the size of a single chunk in bytes.
        size_type    getChunkSize    (void)   const   {   return MAX_CHUNK_SIZE;} 

        //! frees all allocated chunks.
        void        clear   (void)  {   init();}

    private:
        // private methods

    protected:
        // protected methods

        //! inititalizes the embedded freelist
        void    init    (void);

    protected:
        // private typedefs
        typedef uint32_t  offset_type;
        typedef TAlignedStorage<MAX_STORAGE_SIZE> storage_type;

         // static assertion for ensuring safe freelist embedding
        VFC_STATIC_ASSERT(ChunkSizeValue>=sizeof(offset_type));

    protected:
        // protected members
        storage_type    m_storage;          //!< raw storage object
        offset_type     m_freeHeadOffset;   //!< offset of first chunk in freelist
        size_type       m_usedChunks;       //!< number of chunks in use
    };

    
}   // namespace vfc closed
#include "vfc/memory/vfc_fixedmempool.inl"

#endif //VFC_FIXEDMEMPOOL_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fixedmempool.hpp  $
//  Revision 1.11 2012/12/18 08:27:36MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.10 2012/12/05 13:18:27MEZ Alaa El-Din Omar (CC/ESV2) (ALO2HI) 
//  - mantis 0004190:
//   -- made private methods and members protected to allow inheritence in TFixedMemPool2
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
