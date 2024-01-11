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
///     $Source: vfc_fixedplacementmempool.inl $
///     $Revision: 1.2 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/01/28 13:56:38MEZ $
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


#include "vfc/core/vfc_assert.hpp"

template <vfc::int32_t ChunkSizeValue, vfc::int32_t ChunkCountValue>
inline
vfc::TFixedPlacementMempool<ChunkSizeValue, ChunkCountValue>::~TFixedPlacementMempool(void)
{
    VFC_ENSURE2( 0 == baseClass_t::m_usedChunks, "possible memory leakage in TFixedMempool2<>~TFixedMempool2()" );
}


template <vfc::int32_t ChunkSizeValue, vfc::int32_t ChunkCountValue>
void*   vfc::TFixedPlacementMempool<ChunkSizeValue,ChunkCountValue>::placement_malloc(size_type f_count,const void* const f_chunkAddr_p)
{

    VFC_REQUIRE (0 <= f_count);

    VFC_ASSERT(f_chunkAddr_p >= baseClass_t::m_storage.begin());

    ptrdiff_t diff = static_cast<char_t const*>(f_chunkAddr_p) - static_cast<char_t const*>(baseClass_t::m_storage.begin());

    VFC_REQUIRE2((0 <= diff ) && ( baseClass_t::MAX_STORAGE_SIZE > diff) && ( 0 == (diff % baseClass_t::MAX_CHUNK_SIZE)), "given chunk pointer not aligned");
    // explicitly typecast ptrdiff_t type from signed type to unsigned type, leading to very large numbers for negative pointers
    // this leads to errors in following check and NULL-Pointer is returned for out of bounds pointers
    const offset_type newOffset = static_cast<offset_type>(diff);
    // check if there are chunks left and requested size fits into chunk
    if ( ( baseClass_t::getTotalChunks() == baseClass_t::getUsedChunks() ) || ( baseClass_t::getChunkSize() < f_count ) ||
         ( 0 >= (baseClass_t::MAX_STORAGE_SIZE-newOffset) ) )
    {
        return 0;
    }
    // we know pointer is inside our memory area
    // paranoia check, may be omitted for performance reasons. Alignment check is faster than searching free list
    // for unaligned pointer, but should be omitted for guaranteed memory
    if( 0 != (newOffset % baseClass_t::MAX_CHUNK_SIZE) )
    {
        return 0;
    }

    offset_type prevOffset = 0;
    // special case handling if searched chunk offset is the head of free list
    if (newOffset == baseClass_t::m_freeHeadOffset)
    {
        baseClass_t::m_freeHeadOffset = *(reinterpret_cast<const offset_type*>(f_chunkAddr_p));
        // update stats
        ++baseClass_t::m_usedChunks;
        return const_cast<void*>(f_chunkAddr_p);
    }

    // search for given chunk offset in free list
    if (searchChunkOffsetInFreelist(newOffset, prevOffset))
    {
        //adjust previous chunk's freelist reference
        char_t* prev_chunk_pc = static_cast<char_t*>(baseClass_t::m_storage.begin())+prevOffset;
        *(reinterpret_cast<offset_type*>(prev_chunk_pc)) = *(reinterpret_cast<const offset_type*>(f_chunkAddr_p));

        // update stats
        ++baseClass_t::m_usedChunks;
        return const_cast<void*>(f_chunkAddr_p);
    }

    return 0;
}



template <vfc::int32_t ChunkSizeValue, vfc::int32_t ChunkCountValue>
inline
typename vfc::TFixedPlacementMempool<ChunkSizeValue,ChunkCountValue>::offset_type
vfc::TFixedPlacementMempool<ChunkSizeValue,ChunkCountValue>::getNextOffset(offset_type f_offset) const
{
    const char_t* chunk_pc = static_cast<const char_t*>(baseClass_t::m_storage.begin())+f_offset;
    return *(reinterpret_cast<const offset_type*>(chunk_pc));
}

template <vfc::int32_t ChunkSizeValue, vfc::int32_t ChunkCountValue>
bool vfc::TFixedPlacementMempool<ChunkSizeValue,ChunkCountValue>::searchChunkOffsetInFreelist(
    const offset_type f_offset, offset_type& f_prevOffset)
{
    offset_type currOffset = baseClass_t::m_freeHeadOffset;
    // loop over whole mem pool to find requested chunk (last element of free list should
    // point to itself)
    const size_type freeListSize = baseClass_t::MAX_CHUNK_COUNT-baseClass_t::m_usedChunks;

    for (size_type i=0; i<freeListSize; ++i)
    {
        // read next offset on free list
        const offset_type nextOffset = getNextOffset(currOffset);

        if (f_offset == nextOffset)
        {
            // found searched offset, export predecessor to caller for free list management
            f_prevOffset = currOffset;
            return true;
        }
        currOffset = nextOffset;
    }
    return false;// no chunk with given offset found
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fixedplacementmempool.inl  $
//  Revision 1.2 2014/01/28 13:56:38MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_fixedplacedmempool: fix compiler warning (mantis0004393)
//  Revision 1.1 2012/12/11 14:11:09MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/memory/memory.pj
//  Revision 1.2 2012/12/07 16:01:14MEZ Alaa El-Din Omar (CC/ESV2) (ALO2HI) 
//  - mantis 0004190:
//    -- changes in implementation for faster speed
//    -- removed methods only needed for tests
//  Revision 1.1 2012/12/05 17:24:01MEZ Alaa El-Din Omar (CC/ESV2) (ALO2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/private_workspace/alo2hi/vfc/pc_ivs2_vfc/devl/include/vfc/memory/memory.pj
//  Revision 1.8 2011/09/14 10:44:28MESZ Ambily Antony (RBEI/ESD1) (AAT1COB) 
//  - Added the missing parenthesis in the expression (mantis 4003)
//  Revision 1.7 2008/08/25 17:59:48IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - removed unused variable (mantis 2194)
//  Revision 1.6 2007/07/17 07:55:45CEST Dilip Krishna (RBEI/EAE6) (dkn2kor) 
//  - added memory leak check (mantis1736)
//  Revision 1.5 2007/06/22 18:46:06IST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - corrected redeclared function parameter name (mantis 1693)
//  Revision 1.4 2007/02/15 09:33:47CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - reworked errorhandling (mantis1430)
//  - replaced uint32_t with int32_t 
//  - replaced C style comments with C++ comments
//  Revision 1.3 2006/04/05 15:15:05CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added missing include
//  Revision 1.2 2006/03/02 13:06:59CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -changed char to char_t
//=============================================================================
