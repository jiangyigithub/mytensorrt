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
//       Projectname:
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
//        Name: jat2hi
//  Department: AE-DA/ESA3
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_linalg.hpp $
///     $Revision: 1.3 $
///     $Author: gaj2kor $
///     $Date: 2008/08/29 15:04:18MESZ $
///     $Locker:  $
///     $Name: 0032 RC1  $
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

#ifndef VFC_LINALG_HPP_INCLUDED
#define VFC_LINALG_HPP_INCLUDED

//=============================================================================
// DOXYGEN DEFGROUP vfc_group_linalg_dynamic
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_linalg_dynamic dynamic size vectors and matrices
/// @ingroup vfc_group_linalg
/// @brief dynamic size linear algebra, blas level 1, 2 and 3 operations
//=============================================================================
#include "vfc/linalg/vfc_linalg_vector.hpp"
#include "vfc/linalg/vfc_linalg_matrix.hpp"
#include "vfc/linalg/vfc_linalg_ops1d.hpp"
#include "vfc/linalg/vfc_linalg_ops2d.hpp"
#include "vfc/linalg/vfc_linalg_ops1d2d.hpp"

//=============================================================================
//  DOXYGEN INGROUP vfc_group_linalg
//-----------------------------------------------------------------------------
/// @ingroup vfc_group_linalg Linalg
/// @brief  the alias wrapper & vector operation
/// !documentation in progress - add detailed description here!
//=============================================================================
#include "vfc/linalg/vfc_linalg_alias_wrapper.hpp"
#include "vfc/linalg/vfc_linalg_vector_ops.hpp"

#endif // VFC_LINALG_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg.hpp  $
//  Revision 1.3 2008/08/29 15:04:18MESZ gaj2kor 
//  Addition of doxygen defgroup (Mantis :2269)
//  Revision 1.2 2008/07/31 14:08:57IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:18IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================

