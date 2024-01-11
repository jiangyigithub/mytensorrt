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
//       Compiler(s): mwerks cw
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
///     $Source: vfc_mwerks.hpp $
///     $Revision: 1.20 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2013/05/15 15:37:31MESZ $
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

#ifndef VFC_METROWERKS_HPP_INCLUDED
#define VFC_METROWERKS_HPP_INCLUDED

#include "vfc/core/vfc_preprocessor.hpp"

/////////////////////////////////////
// try to detect eppc compiler
// amazingly  __embedded__ ist NOT
// defined on CW8.1 EPPC compiler ?!
/////////////////////////////////////
#if (defined(__POWERPC__) && !defined(macintosh))
#    define VFC_EPPC_DETECTED
#endif

/////////////////////////////////////
// versions check
/////////////////////////////////////

#if defined(VFC_EPPC_DETECTED)
#    if (__MWERKS__ < 0x2407)
#        error "Compiler not supported or configured"
#    endif
#    if (__MWERKS__ > 0x4010)
#        error "Unknown compiler version"
#    endif
#else
#    error "Unknown compiler version"
#endif


/////////////////////////////////////
//  Metrowerks C++ compiler setup
/////////////////////////////////////

// embedded PPC
#if defined(VFC_EPPC_DETECTED)
#    if __MWERKS__ == 0x2407
#        define    VFC_COMPILER_VERSION VFC_STRINGIZE(EPPC_7.0)
#    elif __MWERKS__ == 0x3004
#        define VFC_COMPILER_VERSION VFC_STRINGIZE(EPPC_8.0)
#    elif __MWERKS__ == 0x4010
#        define VFC_COMPILER_VERSION VFC_STRINGIZE(EPPC_8.5)
#    else
#        define VFC_COMPILER_VERSION __MWERKS__
#    endif
// all others
#else
#  define VFC_COMPILER_VERSION __MWERKS__
#endif

#define VFC_COMPILER_STRING VFC_JOIN("Metrowerks CodeWarrior C++ version ", VFC_STRINGIZE(VFC_COMPILER_VERSION))

#ifndef VFC_COMPILER_MWERKS
#   define VFC_COMPILER_MWERKS
#endif

////////////////////////////////////
// multi/single threaded target
////////////////////////////////////

#if     __ide_target("lib-multi-debug")\
    ||    __ide_target("lib-multi-release")\
    ||    __ide_target("lib-multi-dll-debug")\
    ||    __ide_target("lib-multi-dll-release")
#    define    VFC_MT
#endif

////////////////////////////////////
// RTTI support
////////////////////////////////////

#if !__option(RTTI)
#    define    VFC_NO_RTTI
#endif

////////////////////////////////////
// exception support
////////////////////////////////////

#if !__option(exceptions)
#   define VFC_NO_EXCEPTIONS
#endif

//////////////////////////////
// long long / __int64 support
//////////////////////////////

#if __option(longlong)
#    define    VFC_HAS_LONG_LONG
#endif

//////////////////////////////
// cstdint support
//////////////////////////////

// MSL defines stdint types only with C99 support (see <stdint.h>)
#if (defined(_MSL_C99) && (0 !=_MSL_C99))
#   define VFC_HAS_STDINT_H
#endif

//////////////////////////////
// endianess
//////////////////////////////

#if !__option(little_endian)
#    define VFC_BIG_ENDIAN
#endif

////////////////////////////////////
// processor identification
////////////////////////////////////

#if defined(__i386__)
#   define VFC_PROCESSOR_IX86
#elif defined(__POWERPC__)
#   define VFC_PROCESSOR_PPC
#else
#   error "processor not supported"
#endif

//macro abstraction for linker statements 
#define VFC_LINKER_SECT(sect_name) __declspec(section sect_name)

#ifndef VFC_DECL_FORCEINLINE
#   define VFC_DECL_FORCEINLINE inline
#endif 

#endif //VFC_METROWERKS_HPP_INCLUDED


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_mwerks.hpp  $
//  Revision 1.20 2013/05/15 15:37:31MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Remove support for codewarrior/win32 (mantis4067)
//  Revision 1.19 2013/01/01 11:38:03MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Provide macro abstraction for compiler-spezific alignment settings (mantis4171)
//  Revision 1.18 2012/03/28 16:38:14IST Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) 
//  - macro abstraction for linker statements(mantis:3976)
//  Revision 1.17 2011/06/15 16:06:24IST Sudhakar Nannapaneni (RBEI/ESD1) (SNU5KOR) 
//  -Resolved EPPC macro issues for CodeWarrior compiler (mantis3852)
//  Revision 1.16 2009/02/27 09:51:14IST Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  - Removal of else part from the flag _MSL_C99 check. (mantis 2587)
//  Revision 1.15 2009/02/25 16:07:53IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Addition on check for the _MSL_C99 value. (mantis2587)
//  Revision 1.14 2009/01/22 16:32:22IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Used "defined()" to ask for #defines
//  (Mantis : 0002209)
//  Revision 1.13 2007/07/11 13:25:42IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  - CW MSL defines stdint types only with C99 support (see <stdint.h>) (mantis 1739)
//  Revision 1.12 2007/03/01 11:36:52CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - added add VFC_PROCESSOR_XXX defines to unify processor identification (mantis 1201)
//  Revision 1.11 2007/02/16 15:19:08CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - removed the VFC_DEBUG define/undefine
//  Revision 1.10 2007/01/23 17:57:08CET Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  support CW 8.5 embedded PPC (mantis1392)
//  Revision 1.9 2006/11/16 14:43:56CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.8 2006/10/06 13:12:48CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - changed VFC_COMPILER define to VFC_COMPILER_STRING
//  - changed VFC_EPPC define to VFC_EPPC_DETECTED
//  - added define VFC_COMPILER_MWERKS
//  (mantis 1149)
//  Revision 1.7 2006/05/22 13:46:17CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  -excluded x86 cw prior to 9.0 from supported compilers (mantis1086)
//  Revision 1.5 2005/10/06 16:59:43CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/config/compiler/compiler.pj
//  Revision 1.4 2005/04/05 13:56:45CEST zvh2hi
//  added detection code for stdint.h and endianess
//  Revision 1.3 2005/04/04 14:06:14CEST zvh2hi
//  added detection code for multi-threading, rtti, long long support, debug target
//  Revision 1.2 2005/04/01 17:17:21CEST zvh2hi
//  changed EPPC detection, __embedded__ is amazingly not set with CW81 for EPPC
//  Revision 1.1 2005/04/01 15:52:55CEST zvh2hi
//  Initial revision
//  Member added to project /import/mks/data/projects/cv/vfc/include/vfc/config/compiler/compiler.pj
//=============================================================================


