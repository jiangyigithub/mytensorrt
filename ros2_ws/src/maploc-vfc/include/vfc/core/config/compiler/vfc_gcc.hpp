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
//       Compiler(s): gcc
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
///     $Source: vfc_gcc.hpp $
///     $Revision: 1.27 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/02/18 13:19:36MEZ $
///     $Locker: Jaeger Thomas (CC-DA/EPV2) (JAT2HI)(2014/09/17 15:30:24MESZ) $
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

#ifndef VFC_GCC_HPP_INCLUDED
#define VFC_GCC_HPP_INCLUDED

#include <cstddef>
#include "vfc/core/vfc_preprocessor.hpp"

////////////////////////////////////
// processor identification
////////////////////////////////////

#ifdef __i386__
#   define VFC_PROCESSOR_IX86
#elif defined (__x86_64__)
#   define VFC_PROCESSOR_IX86_64
#elif defined(__PPC)
#   define VFC_PROCESSOR_PPC
#   define VFC_EPPC_DETECTED
#elif defined (__MICROBLAZE__)
#   define VFC_PROCESSOR_MICROBLAZE
#   define VFC_EMB_DETECTED
#elif defined (__arm__)
#   define VFC_PROCESSOR_ARM
#   define VFC_ARM_DETECTED
#   if (mcpu == cortex-a8)
#       define VFC_PROCESSOR_ARM_CORTEXA8
#   endif
#elif defined(__aarch64__)
#   define VFC_PROCESSOR_ARM
#else
#   error "processor not supported"
#endif

//////////////////////////////
// versions check
//////////////////////////////

// we don't know gcc prior to version 3.4:
#if (__GNUC__ < 3) || ((__GNUC__ == 3) && (__GNUC_MINOR__ < 4))
#    error "Compiler not supported or configured"
#endif
//
//    last known and checked version is 4.6.1:
 #if (__GNUC__ > 4) || ((__GNUC__ == 4) && (__GNUC_MINOR__ > 6))
//#    error "Unknown compiler version"
#endif

// add a warning message if we detect the eppc cross compiler.
// it is only supported temporary, as it is not capable of running
// the vfc unittests successfully due to compiler bugs.
#if (((__GNUC__ == 3) && (__GNUC_MINOR__ == 4)) && (defined VFC_EPPC_DETECTED))
#   error "This compiler is not supported as it has known bugs"
#endif


///////////////////////////////
//  GNU C++ compiler setup
///////////////////////////////

#define VFC_COMPILER_STRING "GNU C++ version " __VERSION__

#ifndef VFC_COMPILER_GCC
#   define VFC_COMPILER_GCC
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

#ifdef _GLIBCXX_HAVE_STDINT_H
    #define VFC_HAS_STDINT_H
#endif

//////////////////////////////
// long long support
//////////////////////////////

#ifdef _GLIBCXX_USE_LONG_LONG
    #define VFC_HAS_LONG_LONG
#endif

//////////////////////////////
// endianess setup
//////////////////////////////
#ifdef VFC_EPPC_DETECTED
#   define VFC_BIG_ENDIAN
#endif

//macro abstraction for linker statements 
#define VFC_LINKER_SECT(sect_name) __attribute__((section(sect_name)))

#ifndef VFC_DECL_FORCEINLINE
#   define VFC_DECL_FORCEINLINE inline __attribute__((always_inline)) 
#endif

#endif //VFC_GCC_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_gcc.hpp  $
//  Revision 1.27 2014/02/18 13:19:36MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_gcc.hpp: replace tab by 4 spaces (mantis0004418)
//  Revision 1.26 2014/01/28 14:15:45MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_gcc.hpp uses macros of libstdc++ but does not include necessary header (mantis0004176)
//  Revision 1.25 2013/02/11 11:49:56MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Provide macro abstraction for compiler-spezific alignment settings (mantis4171)
//  Revision 1.24 2012/03/28 16:38:06IST Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) 
//  - macro abstraction for linker statements(mantis:3976)
//  Revision 1.23 2012/02/16 19:05:43IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - going up to 4.6.1 (mantis 4121)
//  Revision 1.22 2010/09/09 11:05:58MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - Remove support for gcc eppc 3.4.1. compiler completely (mantis3307)
//  Revision 1.21 2010/08/11 17:09:45MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - merge branch: add support for gnu arm compiler (mantis0002711)
//  Revision 1.20.1.3 2010/04/21 12:23:26MESZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Automatic-identification of targer subtype like (cortexa8) (mantis : 0003294)
//  Revision 1.20.1.2 2010/01/22 14:53:20GMT+05:30 Jaeger Thomas (CC/ESV2) (JAT2HI)
//  - added ARM and NEC support
//  Revision 1.2 2009/09/30 14:31:08CEST Jaeger Thomas (CC-DA/ESV2) (JAT2HI)
//  - allow gcc 4.4.x
//  Revision 1.1 2009/08/13 09:28:21CEST Jaeger Thomas (CC-DA/ESV2) (JAT2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/arm_rvds31_compiler/arm_rvds31_compiler.pj
//  Revision 1.20 2008/10/07 14:12:00CEST Zitzewitz Henning von (CR/AEM6) (ZVH2HI)
//  - added gcc 4.3.x to supported compilers (mantis 2216)
//  Revision 1.19 2008/08/11 17:41:37CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added stvc compiler and platforms to config module (mantis2067)
//  Revision 1.18 2007/11/12 17:27:15CET Jaeger Thomas (CC-DA/ESV1) (jat2hi) 
//  - if target processor is ppc, set the VFC_BIG_ENDIAN define (mantis 1922)
//  Revision 1.17 2007/10/01 18:13:03CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added temporary support for the xilinx gnu eppc cross compiler (mantis 1796)
//  Revision 1.16 2007/06/07 14:19:16CEST Koenig Matthias (CR/AEM5) (kon3hi) 
//  added gcc4.2 support
//  Revision 1.15 2007/04/02 13:36:57CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - set the define VFC_HAS_STDINT_H as gcc supports stdint.h (mantis 1530)
//  Revision 1.14 2007/03/02 11:47:44CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added ix86_64 to supported processors (mantis1201)
//  Revision 1.13 2007/03/01 11:36:52CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added add VFC_PROCESSOR_XXX defines to unify processor identification (mantis 1201)
//  Revision 1.12 2007/02/16 15:19:08CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - removed the VFC_DEBUG define/undefine
//  Revision 1.11 2007/02/02 09:25:10CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - we don't support gcc < 3.4 (mantis1380)
//  Revision 1.10 2007/01/08 09:37:38CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - fixed support version comment
//  Revision 1.9 2007/01/08 09:24:07CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - mantis addendum to rev 1.8: added gcc support up to 4.1 (mantis 1374)
//  Revision 1.8 2006/12/21 17:00:50CET Heiler Matthias (CR/AEM5) (HME2HI) 
//  Updated version support up to gcc 4.1
//  Revision 1.7 2006/11/16 14:43:56CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.6 2006/11/10 10:43:52CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added _DEBUG and NDEBUG defines test to set/unset VFC_DEBUG (mantis 1284)
//  - added test that not both _DEBUG and NDEBUG are defined (mantis 1288)
//  Revision 1.5 2006/10/30 09:27:01CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - improved gcc compatibility (mantis1234)
//  - replaced header/footer templates
//  Revision 1.4 2006/10/06 13:12:24CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - changed VFC_COMPILER define to VFC_COMPILER_STRING
//  - added define VFC_COMPILER_GCC
//  (mantis 1149)
//  Revision 1.3 2005/10/28 10:21:17CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/config/compiler/compiler.pj
//  Revision 1.2 2005/10/06 17:00:03CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/config/compiler/compiler.pj
//  Revision 1.1 2005/04/01 15:52:55CEST zvh2hi 
//  Initial revision
//  Member added to project /import/mks/data/projects/cv/vfc/include/vfc/config/compiler/compiler.pj
//=============================================================================
