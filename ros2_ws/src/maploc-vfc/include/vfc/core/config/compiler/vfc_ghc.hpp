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
//       Compiler(s): GreenHills
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name:
//  Department:
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_ghc.hpp $
///     $Revision: 1.8 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2013/01/01 11:37:51MEZ $
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

#ifndef VFC_GHC_HPP_INCLUDED
#define VFC_GHC_HPP_INCLUDED

#include "vfc/core/vfc_preprocessor.hpp"
//////////////////////////////
// versions check
//////////////////////////////

// we don't know GHC prior to version 4.2.1:
#if (__GHS_VERSION_NUMBER < 421)
#    error "Compiler not supported or configured"
#endif

// last known and checked version is 4.2.4:
#if (__GHS_VERSION_NUMBER > 517)
#    error "Unknown compiler version"
#endif

///////////////////////////////
//  GreenHills C++ compiler setup
///////////////////////////////

#define VFC_COMPILER_VERSION VFC_STRINGIZE(VFC_COMPILER_VERSION)
#define VFC_COMPILER_STRING  VFC_JOIN("GreenHills version ", VFC_COMPILER_VERSION)

#if defined(__V850)
#    define VFC_V850_DETECTED
#endif

#ifndef VFC_COMPILER_GHS
#   define VFC_COMPILER_GHS
#endif

//////////////////////////////
// exception handling
//////////////////////////////

#ifndef __EXCEPTIONS
#    define VFC_NO_EXCEPTIONS
#endif

//////////////////////////////
// stdint.h support
//////////////////////////////

#define __STDC_LIMIT_MACROS
#define VFC_HAS_STDINT_H

//////////////////////////////
// long long support
//////////////////////////////

#ifndef __NO_LONGLONG
#   define VFC_HAS_LONG_LONG
#endif

////////////////////////////////////
// processor identification
////////////////////////////////////

#ifdef __V800
#   define VFC_PROCESSOR_V8XX
#elif __ppc
#   define VFC_EPPC_DETECTED
#else
#   error "processor not supported"
#endif

////////////////////////////////////
// VFC needs namespace support
////////////////////////////////////

#ifndef __NAMESPACES
#    error "VFC needs namespace support, see build_{v800,ppc}.{pdf,chm}"
#endif

////////////////////////////////////
// floating point support
////////////////////////////////////

#ifdef __NoFloat
#    define VFC_NO_FLOAT
#endif

//macro abstraction for linker statements 
#define VFC_LINKER_SECT1 __attribute__
#define VFC_LINKER_SECT(sect_name) VFC_JOIN(VFC_LINKER_SECT1,((used,section##(sect_name)##)))

#ifndef VFC_DECL_FORCEINLINE
#   define VFC_DECL_FORCEINLINE inline
#endif 


#endif //VFC_GHC_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_ghc.hpp  $
//  Revision 1.8 2013/01/01 11:37:51MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Provide macro abstraction for compiler-spezific alignment settings (mantis4171)
//  Revision 1.7 2012/03/28 16:38:11IST Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) 
//  - macro abstraction for linker statements(mantis:3976)
//  Revision 1.6 2011/12/19 13:36:43IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - change the v850 detection to use V850_DETECTED (mantis0003861)
//  Revision 1.5 2011/01/21 12:42:20MEZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - Support compiler GHS 5.17d for NEC target  (mantis3600)
//  Revision 1.4 2010/10/13 13:28:56IST Gaurav Jain (RBEI/ESB2) (gaj2kor)
//  - Resolved Greenhills compiler config elif/else bug. (mantis 0003443)
//  Revision 1.3 2010/08/11 17:17:14CEST Jaeger Thomas (CC/EPV2) (JAT2HI)
//  - merge the ghs support branch (mantis0003063)
//  Revision 1.2 2010/05/05 08:48:54MESZ Jaeger Thomas (CC/EPV2) (JAT2HI)
//  - mantis 0003306
//  Revision 1.1 2010/01/22 10:22:41CET Jaeger Thomas (CC/ESV2) (JAT2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/core/config/compiler/compiler.pj
//  Revision 1.1 2009/10/14 07:09:46CEST Sandeep Patil (RBEI/EAC1) (spp3kor)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/ghs_nec/ghs_nec.pj
//=============================================================================
