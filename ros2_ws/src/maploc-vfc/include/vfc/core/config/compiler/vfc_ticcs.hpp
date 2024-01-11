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
//       Projectname: vfc/core/config/compiler
//          Synopsis: 
//  Target system(s): 
//       Compiler(s): Ti Code Composer Studio
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
///     $Source: vfc_ticcs.hpp $
///     $Revision: 1.4 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2013/01/01 11:38:16MEZ $
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

#ifndef VFC_TICCS_HPP_INCLUDED
#define VFC_TICCS_HPP_INCLUDED

#include "vfc/core/vfc_preprocessor.hpp"

/////////////////////////////////////
// version check
/////////////////////////////////////

#define VFC_COMPILER_TICCS
#define VFC_COMPILER_VERSION __TI_COMPILER_VERSION__
#define VFC_COMPILER_STRING VFC_JOIN("Ti Code Composer Studio version ", VFC_STRINGIZE(VFC_COMPILER_VERSION))

// check for ST TIC6X platform
#if defined (_TMS320C6X)
#   define VFC_TIC6X_DETECTED
#endif    


#if defined (_TMS470V7A8)
#   define VFC_TIC7A8_DETECTED
#endif    

////////////////////////////////////
// RTTI support
////////////////////////////////////

#define VFC_NO_RTTI

////////////////////////////////////
// exception support
////////////////////////////////////

#define VFC_NO_EXCEPTIONS

//////////////////////////////
// long long / __int64 support
//////////////////////////////

// no long long support!

//////////////////////////////
// cstdint support
//////////////////////////////

#define VFC_HAS_STDINT_H

//////////////////////////////
// endianess
//////////////////////////////

#if defined (_BIG_ENDIAN)
#   define VFC_BIG_ENDIAN
#endif

//////////////////////////////
// long long support
//////////////////////////////

#ifndef __STRICT_ANSI__
    #if defined (VFC_TIC7A8_DETECTED)
        #   define VFC_HAS_LONG_LONG
    #endif
#endif


////////////////////////////////////
// processor identification
////////////////////////////////////

// TI C6X product range
#if defined(_TMS320C6200)
#   define VFC_PROCESSOR_TMS320C6200
#elif defined(_TMS320C6400)
#   define VFC_PROCESSOR_TMS320C6400
#elif defined(_TMS320C6400_PLUS)
#   define VFC_PROCESSOR_TMS320C6400_PLUS
#elif defined(_TMS320C6700)
#   define VFC_PROCESSOR_TMS320C6700
#elif defined(_TMS320C6700_PLUS)
#   define VFC_PROCESSOR_TMS320C6700_PLUS
#elif defined(_TMS320C6740)
#   define VFC_PROCESSOR_TMS320C6740
#elif defined(__TI_TMS470_V7A8__)
#   define VFC_PROCESSOR_TMS470V7A8
#else
#   error "processor not supported!"
#endif

//macro abstraction for linker statements 
#define VFC_LINKER_SECT1 __attribute__
#define VFC_LINKER_SECT(sect_name) VFC_JOIN(VFC_LINKER_SECT1,(section##(sect_name)##) volatile)

#ifndef VFC_DECL_FORCEINLINE
#   define VFC_DECL_FORCEINLINE inline         // no other force inlining option found, hence simple inline is opted.           
#endif 

#endif //VFC_TICCS_HPP_INCLUDED


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_ticcs.hpp  $
//  Revision 1.4 2013/01/01 11:38:16MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Provide macro abstraction for compiler-spezific alignment settings (mantis4171)
//  Revision 1.3 2012/03/28 16:38:18IST Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) 
//  - macro abstraction for linker statements(mantis:3976)
//  Revision 1.2 2011/03/21 09:57:18IST Sudhakar Nannapaneni (RBEI/ESB) (SNU5KOR) 
//  - New compiler and processor "TMS470V7A8" added .(mantis : 3729)
//  Revision 1.1 2008/08/11 17:38:00MESZ Zitzewitz Henning von (CR/AEM6) (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/config/compiler/compiler.pj
//  Revision 1.1 2008/07/17 14:21:30CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/mantis2067/mantis2067.pj
//=============================================================================


