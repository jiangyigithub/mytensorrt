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
//       Compiler(s): ms visualc++
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
///     $Source: vfc_visualc.hpp $
///     $Revision: 1.20 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2013/02/11 13:56:58MEZ $
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

#ifndef VFC_VISUALC_HPP_INCLUDED
#define VFC_VISUALC_HPP_INCLUDED

#include "vfc/core/vfc_preprocessor.hpp"

//////////////////////////////
// versions check
//////////////////////////////

// we don't support Visual C++ prior to version 7.1:
#if (_MSC_VER < 1310)
#    error "Compiler not supported or configured"
#endif
#if (_MSC_VER > 1600)
#   error "Unknown compiler version"
#endif

////////////////////////////////////
//  Visual C++ compiler setup
////////////////////////////////////

#if (_MSC_VER == 1310)
#    define VFC_COMPILER_VERSION 7.1
#elif (_MSC_VER == 1400)
#    define VFC_COMPILER_VERSION 8.0
#elif (_MSC_VER == 1500)
#   define VFC_COMPILER_VERSION 9.0
#elif (_MSC_VER == 1600)
#   define VFC_COMPILER_VERSION 10.0
#else 
#   define VFC_COMPILER_VERSION _MSC_VER
#endif

#define VFC_COMPILER_STRING VFC_JOIN("Microsoft Visual C++ version ",VFC_STRINGIZE(VFC_COMPILER_VERSION))

#ifndef VFC_COMPILER_VISUALC
#   define VFC_COMPILER_VISUALC
#endif

////////////////////////////////////
// multi/single threaded target
////////////////////////////////////

#ifdef    _MT
#    define    VFC_MT
#endif

////////////////////////////////////
// RTTI support
////////////////////////////////////

#ifndef    _CPPRTTI
#    define    VFC_NO_RTTI
#endif

////////////////////////////////////
// exception support
////////////////////////////////////

#ifndef    _CPPUNWIND   
#  define    VFC_NO_EXCEPTIONS   
#endif 

//////////////////////////////
// long long / __int64 support
//////////////////////////////

#if (_MSC_VER >= 1200)
#   define VFC_HAS_MS_INT64
#endif
#if (_MSC_VER >= 1310) && defined(_MSC_EXTENSIONS)
#   define VFC_HAS_LONG_LONG
#endif

//////////////////////////////
// cstdint support
//////////////////////////////

// VC has no stdint.h support prior to ver 10.0
#if (_MSC_VER >= 1600)
#   define VFC_HAS_STDINT_H
#endif

//////////////////////////////
// endianess
//////////////////////////////

// supports only x86 = little endian targets

////////////////////////////////////
// force inline
////////////////////////////////////

#ifndef VFC_DECL_FORCEINLINE      
#   define VFC_DECL_FORCEINLINE __forceinline
#endif 

//////////////////////////////
// microsoft specific stuff
//////////////////////////////

// disable Win32 API's if compiler extentions are
// turned off:
#ifndef _MSC_EXTENSIONS
#   define VFC_DISABLE_WIN32
#endif

#define VFC_MSVC _MSC_VER

///////////////////////////////////////////
// support for CLR / C++/CLI / managed code
///////////////////////////////////////////

// cf. http://en.wikipedia.org/wiki/Common_Language_Runtime
// and cf. http://en.wikipedia.org/wiki/C%2B%2B/CLI
// If CLR usage is enabled, native code generation (__asm) should not
// (or even may not) be used, as it cannot be represented in CIL (MSIL).
// Below expression is according to:
// http://msdn2.microsoft.com/en-us/library/468x0ea1(VS.80).aspx

#if defined(_MANAGED) || defined(_M_CEE)
#    define VFC_CLR_DETECTED
#endif

////////////////////////////////////
// processor identification
////////////////////////////////////

#ifdef _M_IX86
#   define VFC_PROCESSOR_IX86
#elif _M_X64
#   define VFC_PROCESSOR_X64
#else
#   error "processor not supported"
#endif

//macro abstraction for linker statements 
#define VFC_LINKER_SECT1 __pragma
#define VFC_LINKER_SECT(sect_name) VFC_JOIN(VFC_LINKER_SECT1,(section##(sect_name)##))

#endif //VFC_VISUALC_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_visualc.hpp  $
//  Revision 1.20 2013/02/11 13:56:58MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Provide macro abstraction for compiler-spezific alignment settings (mantis4171)
//  Revision 1.19 2012/12/18 12:57:28IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.18 2012/04/04 14:01:58MESZ Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) 
//  - Removed "used" for compiler warning solving(mantis:3976)
//  Revision 1.17 2012/03/28 16:38:22IST Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) 
//  - macro abstraction for linker statements(mantis:3976)
//  Revision 1.16 2011/01/24 13:44:14IST Muehlmann Karsten (CC/ESV2) (MUK2LR) 
//  - support for Microsoft Visual Studio 2010 added (mantis3580)
//  Revision 1.15 2008/02/05 17:03:08MEZ Lauer Paul-Sebastian (CR/AEM5) (lap2hi) 
//  - added VC 9.0 support (mantis 2031)
//  Revision 1.14 2007/06/26 14:28:15CEST Hissmann Michael (CR/AEM5) (ihm2si) 
//  - added CLR support (new macro VFC_CLR_DETECTED) (mantis 0001721)
//  - fixed warning warning C4793 in vfc_math.inl (mantis 0001721)
//  Revision 1.13 2007/03/01 11:36:52CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added add VFC_PROCESSOR_XXX defines to unify processor identification (mantis 1201)
//  Revision 1.12 2007/02/16 15:19:08CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - removed the VFC_DEBUG define/undefine
//  Revision 1.11 2006/11/16 14:43:56CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.10 2006/11/10 10:46:22CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added NDEBUG define test to unset VFC_DEBUG and test that not both _DEBUG and NDEBUG are defined (mantis 1288)
//  Revision 1.9 2006/10/06 13:13:08CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - changed VFC_COMPILER define to VFC_COMPILER_STRING
//  - added define VFC_COMPILER_VISUALC
//  (mantis 1149)
//  Revision 1.8 2006/05/23 10:11:01CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -changed code comments for vc 8.0
//  -mantis re-check-in (mantis1087)
//  Revision 1.7 2006/05/23 10:07:44CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -added vc 8.0 to suppported compilers
//  Revision 1.6 2005/10/28 10:21:40CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/config/compiler/compiler.pj
//  Revision 1.5 2005/10/06 16:59:19CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/config/compiler/compiler.pj
//  Revision 1.4 2005/04/06 16:42:43CEST zvh2hi 
//  added forceinline support
//  Revision 1.3 2005/04/05 13:56:45CEST zvh2hi 
//  added detection code for stdint.h and endianess
//  Revision 1.2 2005/04/04 14:06:16CEST zvh2hi 
//  added detection code for multi-threading, rtti, long long support, debug target
//=============================================================================
