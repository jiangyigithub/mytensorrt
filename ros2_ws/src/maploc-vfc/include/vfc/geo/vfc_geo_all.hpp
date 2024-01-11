//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2007 by Robert Bosch GmbH. All rights reserved.
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
//        Name: dkn2kor
//  Department: 
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_geo_all.hpp $
///     $Revision: 1.2 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/07/23 09:39:56MESZ $
///     $Locker:  $
///     $Name: 0032 RC1 Hello KW07  $
///     $State: In_Development $
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

#ifndef VFC_GEO_HPP_INCLUDED
#define VFC_GEO_HPP_INCLUDED

//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_geo
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_geo Geo
/// @brief 2, 3 and 4-dim linear algebra, conic sections.
/// !documentation in progress - add detailed description here!
//=============================================================================

//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_geo_linalg
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_geo_linalg Linear Algebra
/// @ingroup vfc_group_geo
/// @brief 2,3 and 4-dim linear algebra, Quaternions.
/// !documentation in progress - add detailed description here!
//=============================================================================

#include "vfc/geo/vfc_vec2.hpp"
#include "vfc/geo/vfc_vec2_ops.hpp"
#include "vfc/geo/vfc_vec3.hpp"
#include "vfc/geo/vfc_vec3_ops.hpp"
#include "vfc/geo/vfc_vec4.hpp"
#include "vfc/geo/vfc_vec4_ops.hpp"

#include "vfc/geo/vfc_mat2.hpp"
#include "vfc/geo/vfc_mat2_ops.hpp"
#include "vfc/geo/vfc_matvec2_ops.hpp"
#include "vfc/geo/vfc_mat3.hpp"
#include "vfc/geo/vfc_mat3_ops.hpp"
#include "vfc/geo/vfc_matvec3_ops.hpp"
#include "vfc/geo/vfc_mat4.hpp"
#include "vfc/geo/vfc_mat4_ops.hpp"
#include "vfc/geo/vfc_matvec4_ops.hpp"
#include "vfc/geo/vfc_matquat_ops.hpp"

#include "vfc/geo/vfc_quat.hpp"
#include "vfc/geo/vfc_quat_ops.hpp"

//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_geo_conics
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_geo_conics Conic Sections
/// @ingroup vfc_group_geo
/// @brief Conic Sections and algorithms.
/// !documentation in progress - add detailed description here!
//=============================================================================

#include "vfc/geo/vfc_conics.hpp"
#include "vfc/geo/vfc_conics_util.hpp"




#endif //VFC_GEO_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_geo_all.hpp  $
//  Revision 1.2 2007/07/23 09:39:56MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.1 2007/01/23 15:11:46CET dkn2kor 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/geo/geo.pj
//=============================================================================
