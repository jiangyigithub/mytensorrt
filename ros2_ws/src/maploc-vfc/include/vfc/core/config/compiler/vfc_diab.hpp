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
//       Compiler(s): WindRiver Diab Data Compiler
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
///     $Source: vfc_diab.hpp $
///     $Revision: 1.11 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2013/02/11 11:57:15MEZ $
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

#ifndef VFC_DIAB_HPP_INCLUDED
#define VFC_DIAB_HPP_INCLUDED

#include "vfc/core/vfc_preprocessor.hpp"

//////////////////////////////
// versions check
//////////////////////////////

// we don't support diab prior to version 5:
#if (__VERSION_NUMBER__ < 5000)
#    error "Compiler not supported or configured"
#endif
//
//    last known and checked version is 5.6.1:
#if (__VERSION_NUMBER__ > 5610)
#    error "Unknown compiler version"
#endif

///////////////////////////////
//  Diab C++ compiler setup
///////////////////////////////

// use predefined macros,
// see wr_compiler_users_guide_ppc.pdf section 6.1

#define VFC_COMPILER_STRING VFC_JOIN("Diab version ",VFC_STRINGIZE(__VERSION__))

#if defined(__ppc)
#    define VFC_EPPC_DETECTED
#endif

#ifndef VFC_COMPILER_DIAB
#   define VFC_COMPILER_DIAB
#endif

//////////////////////////////
// exception handling
//////////////////////////////

#ifndef __EXCEPTIONS
#    define VFC_NO_EXCEPTIONS
#endif

////////////////////////////////////
// RTTI support
////////////////////////////////////

#ifndef    _RTTI
#    define    VFC_NO_RTTI
#endif

//////////////////////////////
// NO cstdint support!
//////////////////////////////

#undef VFC_HAS_STDINT_H

//////////////////////////////
// endianess
//////////////////////////////

#if !defined(__LITTLE_ENDIAN__)
#    define VFC_BIG_ENDIAN
#endif

//////////////////////////////
// long long support
//////////////////////////////

// checked for x86 and PPC so far
#if (defined(__ppc) || defined(__i386))
#   define VFC_HAS_LONG_LONG
#endif

////////////////////////////////////
// processor identification
////////////////////////////////////

#if defined(__ppc)
#   define VFC_PROCESSOR_PPC
#else
#   error "processor not supported"
#endif

//macro abstraction for linker statements 
#define VFC_LINKER_SECT1 __attribute__
#define VFC_LINKER_SECT(sect_name) VFC_JOIN(VFC_LINKER_SECT1,((used,section##(sect_name)##)))

#ifndef VFC_DECL_FORCEINLINE
#   define VFC_DECL_FORCEINLINE inline 
#endif                                                       

#endif //VFC_DIAB_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_diab.hpp  $
//  Revision 1.11 2013/02/11 11:57:15MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Provide macro abstraction for compiler-spezific alignment settings (mantis4171)
//  Revision 1.10 2012/03/28 16:37:58IST Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) 
//  - macro abstraction for linker statements(mantis:3976)
//  Revision 1.9 2008/10/07 20:11:19IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - support version 5.6.1 (mantis 2188)
//  Revision 1.8 2007/08/10 17:23:05CEST Muehlmann Karsten (CC-DA/ESV1) (muk2lr) 
//  support version 5.5.0.0 (mantis1764)
//  Revision 1.7 2007/04/18 17:47:20CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - removed VFC_DEBUG #defines (mantis 1498)
//  - added VFC_PROCESSOR #define (mantis 1500, 1201)
//  Revision 1.6 2006/11/16 14:43:56CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.5 2006/11/10 10:42:31CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added _DEBUG and NDEBUG defines test to set/unset VFC_DEBUG (mantis 1287)
//  - added test that not both _DEBUG and NDEBUG are defined (mantis 1288)
//  Revision 1.4 2006/10/06 13:12:00CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - changed VFC_COMPILER define to VFC_COMPILER_STRING
//  - changed VFC_EPPC define to VFC_EPPC_DETECTED
//  - added define VFC_COMPILER_DIAB
//  (mantis 1149)
//  Revision 1.3 2006/08/21 17:46:32CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added the mantis ID for the previous check-in (mantis1138)
//  Revision 1.2 2006/08/21 10:25:35CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  bugfix: corrected the __VERSION_NUMBER__ macro
//  Revision 1.1 2006/05/23 15:15:55CEST Muehlmann Karsten (AE-DA/ESA3) (muk2lr) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/config/compiler/compiler.pj
//=============================================================================
