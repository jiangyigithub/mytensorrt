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
///     $Source: vfc_fixedmempool.inl $
///     $Revision: 1.8 $
///     $Author: Ambily Antony (RBEI/ESD4) (AAT1COB) $
///     $Date: 2011/09/14 10:44:28MESZ $
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
vfc::TFixedMempool<ChunkSizeValue, ChunkCountValue>::~TFixedMempool(void)
{
    VFC_ENSURE2( 0 == m_usedChunks, "possible memory leakage in TFixedMempool<>~TFixedMempool()" );
}

template <vfc::int32_t ChunkSizeValue, vfc::int32_t ChunkCountValue>
void*   vfc::TFixedMempool<ChunkSizeValue,ChunkCountValue>::malloc (size_type f_count)
{
    VFC_REQUIRE (0 <= f_count);

    // check if there are chunks left and requested size fits into chunk
    if ( ( getTotalChunks() == getUsedChunks() ) || ( getChunkSize() < f_count ) )
    {
        return 0;
    }

    // get pointer to next chunk in freelist
    char_t* chunk_pc = static_cast<char_t*>(m_storage.begin())+m_freeHeadOffset;

    // adjust freelist head
    m_freeHeadOffset = *(reinterpret_cast<offset_type*>(chunk_pc));

    // update stats
    ++m_usedChunks;

    return static_cast<void*>(chunk_pc);
}


template <vfc::int32_t ChunkSizeValue, vfc::int32_t ChunkCountValue>
void    vfc::TFixedMempool<ChunkSizeValue,ChunkCountValue>::free (void* f_memblock_vp)
{
    // early exit (allowed operation)
    if (0 == f_memblock_vp)
    {
        return ;
    }

    // get memory bounds
    const char_t* begin_pc   = static_cast<const char_t*>(m_storage.begin());
    
    // reinterpret void pointer
    char_t* chunk_pc = static_cast<char_t*>(f_memblock_vp);

    // check if pointer is inside memory block at all
    VFC_REQUIRE( (begin_pc<=chunk_pc) && ( static_cast<const char_t*>(m_storage.end()) > chunk_pc) );
        
    // calculate chunk offset
    ptrdiff_t chunkOffset = chunk_pc-begin_pc;
    
    // check if memory is properly aligned
    VFC_REQUIRE( 0==(chunkOffset%getChunkSize()) );

    // update freelist
    *(reinterpret_cast<offset_type*>(chunk_pc)) = m_freeHeadOffset;
    m_freeHeadOffset = static_cast<offset_type>(chunkOffset);

    // update stats
    --m_usedChunks;
}

template <vfc::int32_t ChunkSizeValue, vfc::int32_t ChunkCountValue>
void    vfc::TFixedMempool<ChunkSizeValue,ChunkCountValue>::init    (void)
{
    // embedded freelist setup
    char_t* begin_pc = static_cast<char_t*>(m_storage.begin());

    offset_type    chunkOffset=0;
    
    for (uint32_t i=0; i< MAX_CHUNK_COUNT; ++i, chunkOffset += MAX_CHUNK_SIZE  )
    {
        *(reinterpret_cast<offset_type*>(begin_pc+chunkOffset)) = chunkOffset + MAX_CHUNK_SIZE;
    }
    
    // point freelist head to the first chunk
    m_freeHeadOffset    = 0;
    
    // reset used chunk count
    m_usedChunks        = 0;        
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fixedmempool.inl  $
//  Revision 1.8 2011/09/14 10:44:28MESZ Ambily Antony (RBEI/ESD4) (AAT1COB) 
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
