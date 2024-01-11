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
//       Projectname: vfc/geo
//          Synopsis: Quaternion implementation
//  Target system(s): 
//       Compiler(s): VC_71
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
///     $Source: vfc_matquat_ops.hpp $
///     $Revision: 1.4 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/07/23 09:44:24MESZ $
///     $Locker:  $
///     $Name: 0032 RC1 Hello KW07  $
///     $State: In_Development $
///
/// @par Review Information:
/// - Reviewed version: 
/// - Type (use "X" to mark):
///        - [ ] Formal Review
///        - [ ] Walkthrough
///        - [ ] Inspection
/// - State including date (DD.MM.YYYY)
///        - [--.--.----] Preparation
///        - [--.--.----] Review audit 
///        - [--.--.----] Integration of findings
///        - [--.--.----] Test
///        - [--.--.----] Verification of integration of findings
///        - [--.--.----] Review release
///    - Responsible:
///    - Review-Document:        
//=============================================================================

#ifndef VFC_MATQUAT_OPS_HPP_INCLUDED
#define VFC_MATQUAT_OPS_HPP_INCLUDED

#include "vfc/geo/vfc_mat3.hpp"     // used for TMatrix3<T>
#include "vfc/geo/vfc_mat4.hpp"     // used for TMatrix4<T>
#include "vfc/geo/vfc_quat.hpp"     // used for TQuaternion<T>

namespace vfc
{   // namespace vfc opened
    
    //-----------------------------------------------------------------------------
    /// calculates and returns a quaternion from specified 3d rotation matrix. 
    /// @pre the specified matrix has to be a non-degenerated rotation matrix 
    /// @author zvh2hi
    /// @relatesalso TMatrix3
    //=============================================================================

    template <class T>
    TQuaternion<T>  toQuaternion    (const TMatrix3<T>&     f_mat3);

    //-----------------------------------------------------------------------------
    /// calculates and returns a 3D rotation matrix from given unit length quaternion. 
    /// @pre the specified quaternion has to be unit length
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //=============================================================================

    template <class T>
    TMatrix3<T>     toMatrix3       (const TQuaternion<T>&  f_quat);


    //-----------------------------------------------------------------------------
    /// calculates and returns a homogenous 3D rotation matrix from given unit 
    /// length quaternion. 
    /// @pre the specified quaternion has to be unit length
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix4<T>     toMatrix4       (const TQuaternion<T>&  f_quat);


}   // namespace vfc closed

#include "vfc/geo/vfc_matquat_ops.inl"

#endif //VFC_MATQUAT_OPS_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_matquat_ops.hpp  $
//  Revision 1.4 2007/07/23 09:44:24MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.3 2006/11/16 14:41:16CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.2 2006/10/13 10:08:17CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed header/footer templates
//  Revision 1.1 2006/07/07 10:07:03CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/geo/geo.pj
//=============================================================================
