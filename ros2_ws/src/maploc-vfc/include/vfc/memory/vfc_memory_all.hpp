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
//        Name: dkn2kor
//  Department: 
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_memory_all.hpp $
///     $Revision: 1.4 $
///     $Author: gaj2kor $
///     $Date: 2012/02/14 07:16:14MEZ $
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

#ifndef VFC_MEMORY_HPP_INCLUDED
#define VFC_MEMORY_HPP_INCLUDED

//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_memory
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_memory Memory 
/// @brief Memory manager, allocators and helper functions.
/// !documentation in progress - add detailed description here!
//=============================================================================

#include "vfc/memory/vfc_memory_util.hpp"

//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_memory_manager
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_memory_managers Memory Managers
/// @ingroup vfc_group_memory
/// @brief Memory Manager and utilities.
/// !documentation in progress - add detailed description here!
//=============================================================================

#include "vfc/memory/vfc_linearheap.hpp"
#include "vfc/memory/vfc_fixedmempool.hpp"

//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_memory_allocators
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_memory_allocators Allocators
/// @ingroup vfc_group_memory
/// @brief Standard-conforming allocator classes.
/// !documentation in progress - add detailed description here!
//=============================================================================

#include "vfc/memory/vfc_freestore_allocator.hpp"
#include "vfc/memory/vfc_fixedblock_allocator.hpp"
#include "vfc/memory/vfc_fixedmempool_allocator.hpp"



#endif //VFC_MEMORY_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_memory_all.hpp  $
//  Revision 1.4 2012/02/14 07:16:14MEZ gaj2kor 
//  - Inclusion of missing header file under vfc_memory_all.hpp (mantis : 3489)
//  Revision 1.3 2007/07/23 13:24:55IST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.2 2007/02/15 09:27:17CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - fixed projectname in comment
//  Revision 1.1 2007/01/23 15:12:08CET dkn2kor 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/memory/memory.pj
//=============================================================================
