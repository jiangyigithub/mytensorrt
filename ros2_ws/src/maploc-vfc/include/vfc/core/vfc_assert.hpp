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
//  Target system(s):
//       Compiler(s): VS7.1
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: Thomas Jaeger
//  Department: AE-DA/ESA3
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_assert.hpp $
///     $Revision: 1.17 $
///     $Author: Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) $
///     $Date: 2010/08/13 16:37:58MESZ $
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

#ifndef VFC_ASSERT_HPP_INCLUDED
#define VFC_ASSERT_HPP_INCLUDED

// stdlib includes
#include <cassert>
// vfc includes
#include "vfc/core/vfc_config.hpp"

// Disable warning "controlling expression is constant" 
// on ARM RVCT for VFC_ASSERT2, VFC_REQUIRE2 and VFC_ENSURE2
#ifdef VFC_COMPILER_ARMRVCT
#pragma diag_suppress 236
#endif

//=============================================================================
//  DOXYGEN ADDTOGROUP vfc_group_core_error
//-----------------------------------------------------------------------------
/// @addtogroup vfc_group_core_error
/// vfc and Design by contract.
/// !documentation in progress - add detailed description here!
/// @{

/// @def VFC_ASSERT
/// @brief general assertion macro.
/// @sa VFC_ASSERT2 if you want to specify an additional error string.

/// @def VFC_ASSERT2
/// @brief same as VFC_ASSERT with an additional error message string.
/// @sa VFC_ASSERT

/// @def VFC_REQUIRE
/// @brief macro for precondition checks.
/// @sa VFC_REQUIRE2 if you want to specify an additional error string.

/// @def VFC_REQUIRE2
/// @brief same as VFC_REQUIRE with an additional error message string.
/// @sa VFC_REQUIRE

/// @def VFC_ENSURE
/// @brief macro for post-condition checks.
/// @sa VFC_ENSURE2 if you want to specify an additional error string.

/// @def VFC_ENSURE2
/// @brief same as VFC_ENSURE with an additional error message string.
/// @sa VFC_ENSURE

/// @}
//=============================================================================

// NDEBUG enables VFC_NDEBUG
#if defined (NDEBUG)
#   undef  VFC_NDEBUG
#   define VFC_NDEBUG
#endif
// VFC_NDEBUG disables all checks
#if defined (VFC_NDEBUG)
#   undef   VFC_NO_PRECHECKS
#   define  VFC_NO_PRECHECKS
#   undef   VFC_NO_POSTCHECKS
#   define  VFC_NO_POSTCHECKS
#   undef   VFC_NO_ASSERTS
#   define  VFC_NO_ASSERTS
#endif

#ifdef VFC_NDEBUG
#   define VFC_USE_VAR_ONLY_IN_ASSERTION(VAR_NAME)
#else
#   define VFC_USE_VAR_ONLY_IN_ASSERTION(VAR_NAME) VAR_NAME
#endif

// use this for general condition checking
#if defined (VFC_NO_ASSERTS)
#   define VFC_ASSERT(EXP)          ( (void)0 )
#   define VFC_ASSERT2(EXP,MSG)     ( (void)0 )
#else
#   define VFC_ASSERT(EXP)          ( assert ( (EXP) ) )
#   define VFC_ASSERT2(EXP,MSG)     ( assert ( (EXP) && (MSG) ) )
#endif

// use this for checking of preconditions
#if defined (VFC_NO_PRECHECKS)
#   define VFC_REQUIRE(EXP)         ( (void)0 )
#   define VFC_REQUIRE2(EXP,MSG)    ( (void)0 )
#else
#   define VFC_REQUIRE(EXP)         ( assert ( (EXP) ) )
#   define VFC_REQUIRE2(EXP,MSG)    ( assert ( (EXP) && (MSG) ) )
#endif

// use this for checking of postconditions
#if defined (VFC_NO_POSTCHECKS)
#   define VFC_ENSURE(EXP)          ( (void)0 )
#   define VFC_ENSURE2(EXP,MSG)     ( (void)0 )
#else
#   define VFC_ENSURE(EXP)          ( assert ( (EXP) ) )
#   define VFC_ENSURE2(EXP,MSG)     ( assert ( (EXP) && (MSG) ) )
#endif



#endif //VFC_ASSERT_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_assert.hpp  $
//  Revision 1.17 2010/08/13 16:37:58MESZ Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) 
//  - suppress warning "controlling expression is constant" from ARM RVCT (mantis 3391)
//  Revision 1.16 2009/05/11 07:38:42CEST Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  -Introduced new macro for the suppression of compiler warning.(mantis:2832)
//  Revision 1.15 2009/05/08 13:44:04GMT+05:30 Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  - assert_nop() removed, due to runtime issue. (mantis : 2588)
//  Revision 1.14 2009/03/04 10:20:28GMT+05:30 Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Addition of NDEBUG check and define VFC_NDEBUG. (mantis 2597)
//  Revision 1.13 2009/03/03 20:04:43IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  - Replaced vfc::nop() with intern::assert_nop(). (mantis 2588)
//  Revision 1.12 2009/01/22 16:34:08IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Addition of nop() function.
//  (Mantis : 0002092, 0002460, 0002459 )
//  Revision 1.11 2007/07/18 20:06:10IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - doxygen documentation grouping (mantis1744)
//  Revision 1.10 2007/03/29 10:54:58CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added macro for disabling VFC_ASSERT (mantis1532)
//  - VFC_NDEBUG disables all checks (mantis1516)
//  Revision 1.9 2007/02/19 14:10:31CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - removed VFC_ASSERT_IF (mantis 1445)
//  Revision 1.8 2007/02/16 15:20:01CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - as VFC_DEBUG was replaced by VFC_NDEBUG, vfc assertions are switched off when VFC_NDEBUG is defined
//  Revision 1.7 2007/02/13 17:48:22CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - added functionality to switch  pre- and postcondition checks on and of (mantis 1426)
//  Revision 1.6 2006/11/16 14:41:11CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.5 2006/11/06 13:12:27CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - fixed wrong docu (mantis1260)
//  Revision 1.4 2005/12/20 11:43:32CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  -removed erroneous bracket in VFC_ASSERT_IF2
//  Revision 1.3 2005/10/28 10:27:23CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/core.pj
//  Revision 1.2 2005/10/06 16:54:59CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/vfc.pj
//=============================================================================
