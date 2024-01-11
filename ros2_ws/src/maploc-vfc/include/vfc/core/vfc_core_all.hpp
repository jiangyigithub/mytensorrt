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
///     $Source: vfc_core_all.hpp $
///     $Revision: 1.6 $
///     $Author: gaj2kor $
///     $Date: 2012/02/23 11:37:54MEZ $
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

#ifndef VFC_CORE_HPP_INCLUDED
#define VFC_CORE_HPP_INCLUDED

//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_core
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_core Core 
/// @brief vfc core functionality.
/// !documentation in progress - add detailed description here!
//=============================================================================

//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_core_config
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_core_config Configuration 
/// @ingroup vfc_group_core
//=============================================================================

#include "vfc/core/vfc_config.hpp"

//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_core_types
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_core_types Basic Types
/// @ingroup vfc_group_core
/// @brief C99 Types (stdint), N-Tuples, Trigonometric Types, 2d-Rectangle etc.
/// !documentation in progress - add detailed description here!
//=============================================================================

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_aligned_storage.hpp"
#include "vfc/core/vfc_rect.hpp"
#include "vfc/core/vfc_trig.hpp"
#include "vfc/core/vfc_tuple.hpp"
#include "vfc/core/vfc_endianess.hpp"

//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_core_algorithms
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_core_algorithms Basic Algorithms
/// @ingroup vfc_group_core
/// @brief Math, Bitalgos, STL algorithm enhancements, Endianess-Conversions.
/// !documentation in progress - add detailed description here!
//=============================================================================

#include "vfc/core/vfc_math.hpp"
#include "vfc/core/vfc_bitalgo.hpp"
#include "vfc/core/vfc_algorithm.hpp"
#include "vfc/core/vfc_algorithm_fixed.hpp"
#include "vfc/core/vfc_algorithm2d.hpp"
#include "vfc/core/vfc_algorithm2d_fixed.hpp"


//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_core_error
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_core_error Error-Signaling
/// @ingroup vfc_group_core
/// @brief Compile-Time and Runtime Assertions, Design by Contract.
/// !documentation in progress - add detailed description here!
//=============================================================================

#include "vfc/core/vfc_assert.hpp"
#include "vfc/core/vfc_static_assert.hpp"

//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_core_metaprogram
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_core_generic Generic Programming
/// @ingroup vfc_group_core
/// @brief Type Traits, Metaprogramming, Typelists.
/// !documentation in progress - add detailed description here!
//=============================================================================

#include "vfc/core/vfc_typelist.hpp"
#include "vfc/core/vfc_metaprog.hpp"
#include "vfc/core/vfc_type_traits.hpp"
#include "vfc/core/vfc_type_traits_def.hpp"

//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_core_misc
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_core_misc Miscellaneous
/// @ingroup vfc_group_core
/// @brief Misc functionality.
/// !documentation in progress - add detailed description here!
//=============================================================================

#include "vfc/core/vfc_preprocessor.hpp"
#include "vfc/core/vfc_util.hpp"

//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_core_siunits
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_core_siunits
/// @ingroup vfc_group_core
/// @brief Misc functionality.
/// Include si_units related headers
//=============================================================================

#include "vfc/core/vfc_siunits.hpp"
#include "vfc/core/vfc_siunits_convenienttypes.hpp"
#include "vfc/core/vfc_siunits_types.hpp"
#include "vfc/core/vfc_siunitssubstitute_types.hpp"

//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_core_bit_array_view
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_core_bit_array_view
/// @ingroup vfc_group_core
/// @brief Misc functionality.
/// Include the header of bit_array_view
//=============================================================================
#include "vfc/core/vfc_bit_array_view.hpp"

#endif //VFC_CORE_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_core_all.hpp  $
//  Revision 1.6 2012/02/23 11:37:54MEZ gaj2kor 
//  - Inclusion of missing header file into vfc_core_all.hpp (:mantis 3489)
//  Revision 1.5 2008/08/26 21:05:40IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added new file vfc_util.hpp (mantis 2271)
//  Revision 1.4 2007/07/23 09:19:12CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - renamed groups (mantis1744)
//  Revision 1.3 2007/07/18 16:11:02CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen documentation grouping (mantis1744)
//  Revision 1.2 2007/06/05 11:39:10CEST Lauer Paul-Sebastian (CR/AEM5) (lap2hi) 
//  - removed "vfc_except.hpp #include" (mantis1673)
//  Revision 1.1 2007/01/23 15:03:56CET dkn2kor 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/core.pj
//=============================================================================
