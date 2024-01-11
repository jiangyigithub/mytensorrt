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
//       Compiler(s): ARM RVCT compiler
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: jat2hi
//  Department: CC-DA/ESV1
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_armrvct.hpp $
///     $Revision: 1.14 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/07/25 12:59:28MESZ $
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

#ifndef VFC_ARMRVCT_HPP_INCLUDED
#define VFC_ARMRVCT_HPP_INCLUDED

#include "vfc/core/vfc_preprocessor.hpp"

//////////////////////////////
// versions check
//////////////////////////////

// we support arm rvct version 4.1 (build 894) till 5.04 (build 82)
#if ((__ARMCC_VERSION < 410894) || (__ARMCC_VERSION > 5040081))
#    error "Compiler not supported or configured"
#endif


///////////////////////////////
//  ARM RVCT C++ compiler setup
///////////////////////////////

// use predefined macros,
// see DUI0348A_rvct_compiler_ref_guide.pdf section 4.8

#define VFC_COMPILER_STRING VFC_JOIN("ARM RVCT version ",VFC_STRINGIZE(__ARMCC_VERSION))

#if defined(__arm__)
#    define VFC_ARM_DETECTED
#endif

#ifndef VFC_COMPILER_ARMRVCT
#   define VFC_COMPILER_ARMRVCT
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

#ifndef __RTTI
#    define    VFC_NO_RTTI
#endif

//////////////////////////////
// stdint.h support
//////////////////////////////

#define VFC_HAS_STDINT_H

//////////////////////////////
// endianess
//////////////////////////////

#if defined(__BIG_ENDIAN)
#    define VFC_BIG_ENDIAN
#endif

//////////////////////////////
// long long support
//////////////////////////////

#ifndef __STRICT_ANSI__
#   define VFC_HAS_LONG_LONG
#endif

////////////////////////////////////
// processor identification
////////////////////////////////////

#if defined(__arm__)
#   define VFC_PROCESSOR_ARM
#   if defined (__TARGET_CPU_CORTEX_A8)
#       define VFC_PROCESSOR_ARM_CORTEXA8
#   elif defined (__TARGET_CPU_CORTEX_A9)
#       define VFC_PROCESSOR_ARM_CORTEXA9
#   elif defined (__TARGET_CPU_CORTEX_R4)
#       define VFC_PROCESSOR_ARM_CORTEXR4
#   else
#       error "processor not supported"
#   endif
#endif


////////////////////////////////////
// floating point support
////////////////////////////////////

#ifdef __TARGET_FPU_NONE
#    define VFC_NO_FLOAT
#endif


////////////////////////////////////
// neon support
////////////////////////////////////

#ifdef __ARM_NEON__
#   define VFC_HAS_ARM_NEON
#endif

//macro abstraction for linker statements 
#define VFC_LINKER_SECT1 __attribute__
#define VFC_LINKER_SECT(sect_name) VFC_JOIN(VFC_LINKER_SECT1,((used,section##(sect_name)##)))

#ifndef VFC_DECL_FORCEINLINE      
#   define VFC_DECL_FORCEINLINE __forceinline
#endif 


#endif //VFC_ARMRVCT_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_armrvct.hpp  $
//  Revision 1.14 2014/07/25 12:59:28MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Support ARM R4 target with ARM5 Compiler(mantis0004596)
//  Revision 1.13 2014/07/22 15:53:14MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Support ARM Compiler 5.04 build 82 (mantis0004627)
//  Revision 1.12 2014/03/03 15:40:39MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - support ARM compiler 5.04 build 49 (mantis0004395)
//  Revision 1.11 2013/09/19 13:49:42MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - support ARM compiler 5.03 build 102 (mantis0004309)
//  Revision 1.10 2013/05/14 13:54:55MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Provide macro abstraction for compiler-spezific alignment settings (mantis4171)
//  Revision 1.9 2013/05/13 16:45:25MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Support ARM Compiler 5.03u1 build 69 (mantis4235)
//  Revision 1.8 2012/10/31 17:16:47IST Sudhakar Nannapaneni (RBEI/ESD1) (SNU5KOR) 
//  - Added support for ARM DS-5 Compiler 5.01 build 113 (mantis4151)
//  Revision 1.7 2012/03/28 16:37:47IST Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) 
//  - macro abstraction for linker statements(mantis:3976)
//  Revision 1.6 2012/03/06 16:46:14IST Gaurav Jain (RBEI/ESD1) (gaj2kor) 
//  - Removed support for all compilers prior to rvct41 patch build 6. (mantis : 4057)
//  Revision 1.5 2011/12/19 15:08:05IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - enable usage of stdint.h (mantis0003356)
//  Revision 1.4 2011/12/16 15:05:04MEZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - rvct compiler: limitation to approved compiler versions is not strict enough (mantis0003862)
//  - vfc_armrvct.hpp seems erroneous (mantis0003615)
//  Revision 1.3 2010/08/11 17:20:46MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - merge the rvct support branch (mantis0002203)
//  Revision 1.2 2010/04/21 12:23:29MESZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Automatic-identification of targer subtype like (cortexa8) (mantis : 0003294)
//  Revision 1.1 2010/01/22 14:52:40GMT+05:30 Jaeger Thomas (CC/ESV2) (JAT2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/core/config/compiler/compiler.pj
//  Revision 1.2 2009/09/30 14:30:51CEST Jaeger Thomas (CC-DA/ESV2) (JAT2HI)
//  - corrected version match
//  Revision 1.1 2008/10/09 08:37:13CEST Jaeger Thomas (CC-DA/ESV2) (JAT2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/arm_rvds31_compiler/arm_rvds31_compiler.pj
//=============================================================================

