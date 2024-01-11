//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2006 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorised copy, use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/core
//          Synopsis:
//  Target system(s): cross platform
//       Compiler(s): c++ std
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: Thomas Jaeger
//  Department: CC-DA/ESV1
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_select_platform_config.hpp $
///     $Revision: 1.11 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2011/12/19 09:16:52MEZ $
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


#ifndef VFC_SELECT_PLATFORM_CONFIG_HPP_INCLUDED
#define VFC_SELECT_PLATFORM_CONFIG_HPP_INCLUDED

// linux:
#if defined(linux) || defined(__linux) || defined(__linux__)
#  define VFC_PLATFORM_CONFIG "vfc/core/config/platform/vfc_linux.hpp"
// win32:
#elif defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
#    define VFC_PLATFORM_CONFIG "vfc/core/config/platform/vfc_win32.hpp"
// eppc (VFC_EPPC_DETECTED is currently set by vfc_mwerks.hpp and vfc_diab.hpp)
#elif defined(VFC_EPPC_DETECTED)
#    define VFC_PLATFORM_CONFIG "vfc/core/config/platform/vfc_eppc.hpp"
// ST-TiC6X (VFC_TIC6X_DETECTED is currently set by vfc_ticcs.hpp)
#elif defined(VFC_TIC6X_DETECTED)
#    define VFC_PLATFORM_CONFIG "vfc/core/config/platform/vfc_st_tic6x.hpp"
#elif defined(VFC_TIC7A8_DETECTED)
#    define VFC_PLATFORM_CONFIG "vfc/core/config/platform/vfc_st_emb.hpp"
// ST-MicroBlaze (VFC_EMB_DETECTED is currently set by vfc_gcc.hpp)
#elif defined(VFC_EMB_DETECTED)
#    define VFC_PLATFORM_CONFIG "vfc/core/config/platform/vfc_st_emb.hpp"
// arm (VFC_ARM_DETECTED is currently set by vfc_armrvct.hpp)
#elif defined(VFC_ARM_DETECTED)
#    define VFC_PLATFORM_CONFIG "vfc/core/config/platform/vfc_ivs_arm.hpp"
// v850 (VFC_V850_DETECTED is currently set by vfc_ghc.hpp)
#elif defined(VFC_V850_DETECTED)
#    define VFC_PLATFORM_CONFIG "vfc/core/config/platform/vfc_v8xx.hpp"
#else
#    error "Unknown platform!"
#endif


#endif //VFC_SELECT_PLATFORM_CONFIG_HPP_INCLUDED


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_select_platform_config.hpp  $
//  Revision 1.11 2011/12/19 09:16:52MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - change the v850 detection to use V850_DETECTED (mantis0003861)
//  - unknown platform error is not active (mantis0004065)
//  Revision 1.10 2011/03/21 05:26:08MEZ Sudhakar Nannapaneni (RBEI/ESB3) (SNU5KOR) 
//  - New compiler and processor "TMS470V7A8" added .(mantis : 3729)
//  Revision 1.9 2010/08/11 14:43:09MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - merge from the ARM and NEC branch (mantis0003063, mantis0002203)
//  Revision 1.8.1.2 2010/01/22 10:23:21MEZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - added ARM and NEC support
//  Revision 1.3 2009/03/12 06:06:02CET Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  -Updated new arm platform configuration file name.(mantis:2203)
//  Revision 1.2 2008/10/09 12:07:39IST Jaeger Thomas (CC-DA/ESV2) (JAT2HI)
//  - updates
//  Revision 1.8 2008/08/11 17:41:38CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added stvc compiler and platforms to config module (mantis2067)
//  Revision 1.7 2006/11/16 14:43:56CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.6 2006/10/06 13:15:00CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - changed VFC_EPPC define to VFC_EPPC_DETECTED
//  (mantis 1149)
//  Revision 1.5 2006/05/23 15:16:03CEST Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  config for WindRiver Diab compiler added (matis1080)
//  Revision 1.4 2005/10/28 10:23:48CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/config/config.pj
//  Revision 1.3 2005/10/06 16:57:59CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/config/config.pj
//  Revision 1.2 2005/04/01 17:17:21CEST zvh2hi
//  changed EPPC detection, __embedded__ is amazingly not set with CW81 for EPPC
//  Revision 1.1 2005/04/01 15:53:20CEST zvh2hi
//  Initial revision
//  Member added to project /import/mks/data/projects/cv/vfc/include/vfc/config/config.pj
//=============================================================================

