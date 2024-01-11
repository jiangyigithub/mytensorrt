//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2008 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy or use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  P R O J E C T   I N F O R M A T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/core
//  Target system(s):
//       Compiler(s): VS8.0
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: zvh2hi
//  Department: CR/AEM5
//=============================================================================
//  F I L E   C O N T E N T S   A N D   R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @brief misc. utility functions.
/// @par Synopsis:
///     This is the detailed description.
///
/// @par Revision History:
///     $Source: vfc_util.hpp $
///     $Revision: 1.3 $
///     $Author: gaj2kor $
///     $Date: 2009/05/08 10:35:44MESZ $
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

#ifndef VFC_UTIL_HPP_INCLUDED
#define VFC_UTIL_HPP_INCLUDED

namespace vfc
{   // namespace vfc opened

    //-----------------------------------------------------------
    /// no operation function.
    /// use this function to avoid spurious compiler warnings.
    /// $Source: vfc_util.hpp $
    /// @author zvh2hi
    /// @ingroup vfc_group_core_misc
    //-----------------------------------------------------------
    template <class ArgumentType> inline
    void    nop (const ArgumentType&) {}

}   // namespace vfc closed


#endif //VFC_UTIL_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_util.hpp  $
//  Revision 1.3 2009/05/08 10:35:44MESZ gaj2kor 
//  -assert_nop( ) removed, due to runtime issue. (mantis : 2588)
//  Revision 1.2 2009/03/03 20:05:10IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  - Addtion of intern::assert_nop(). (mantis 2588)
//  Revision 1.1 2008/08/26 21:05:10IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/core.pj
//=============================================================================
