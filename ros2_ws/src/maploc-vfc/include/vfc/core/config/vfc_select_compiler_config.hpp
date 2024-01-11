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
///     $Source: vfc_select_compiler_config.hpp $
///     $Revision: 1.7 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2010/08/11 14:44:07MESZ $
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

#ifndef VFC_SELECT_COMPILER_CONFIG_HPP_INCLUDED
#define VFC_SELECT_COMPILER_CONFIG_HPP_INCLUDED

//  GNU C++:
#if defined __GNUC__
#   define VFC_COMPILER_CONFIG "vfc/core/config/compiler/vfc_gcc.hpp"
//  Metrowerks CodeWarrior
#elif defined  __MWERKS__
#   define VFC_COMPILER_CONFIG "vfc/core/config/compiler/vfc_mwerks.hpp"
//  WindRiver Diab Data Compiler
#elif defined  __DCC__
#   define VFC_COMPILER_CONFIG "vfc/core/config/compiler/vfc_diab.hpp"
// Ti Code Composer Studio
#elif defined __TI_COMPILER_VERSION__
#   define VFC_COMPILER_CONFIG "vfc/core/config/compiler/vfc_ticcs.hpp"
//   ARM RVDS compiler
#elif defined __ARMCC_VERSION
#    define VFC_COMPILER_CONFIG "vfc/core/config/compiler/vfc_armrvct.hpp"
//  Microsoft Visual C++
//  Must remain the last #elif since some other vendors (Metrowerks, for
//  example) also #define _MSC_VER
#elif defined __EDG__
#    define VFC_COMPILER_CONFIG "vfc/core/config/compiler/vfc_ghc.hpp"
#elif defined _MSC_VER
#    define VFC_COMPILER_CONFIG "vfc/core/config/compiler/vfc_visualc.hpp"
#else
#   error "Unknown compiler!"
#endif

#endif //VFC_SELECT_COMPILER_CONFIG_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_select_compiler_config.hpp  $
//  Revision 1.7 2010/08/11 14:44:07MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - merge from the ARM and NEC branch (mantis0003063, mantis0002203)
//  Revision 1.6.1.2 2010/01/22 10:23:21MEZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - added ARM and NEC support
//  Revision 1.2 2008/10/09 08:37:39CEST Jaeger Thomas (CC-DA/ESV2) (JAT2HI) 
//  - updates
//  Revision 1.6 2008/08/11 17:41:37CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added stvc compiler and platforms to config module (mantis2067)
//  Revision 1.5 2006/11/16 14:43:57CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.4 2006/05/23 15:16:02CEST Muehlmann Karsten (AE-DA/ESA3) (muk2lr) 
//  config for WindRiver Diab compiler added (matis1080)
//  Revision 1.3 2005/10/28 10:23:25CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/config/config.pj
//  Revision 1.2 2005/10/06 16:58:25CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/config/config.pj
//=============================================================================


