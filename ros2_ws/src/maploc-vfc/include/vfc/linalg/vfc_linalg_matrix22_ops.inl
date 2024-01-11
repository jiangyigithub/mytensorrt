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
//        Name: dkn2kor
//  Department: 
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_linalg_matrix22_ops.inl $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (jat2hi) $
///     $Date: 2007/05/09 10:21:20MESZ $
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

#include "vfc/core/vfc_static_assert.hpp"   // used for VFC_STATIC_ASSERT()
#include "vfc/core/vfc_assert.hpp"   // used for VFC_ASSERT2()
#include "vfc/core/vfc_math.hpp"            // used for isZero()

template <class T>    inline 
T    vfc::linalg::det    (const TMatrix22<T>& mat)
{    
    return    ( mat(0,0)*mat(1,1) - mat(1,0)*mat(0,1) );
}

template <class T>    inline 
T    vfc::linalg::trace    (const TMatrix22<T>& mat)
{    
    return    ( mat(0,0) + mat(1,1) );
}

template <class T>    inline  
vfc::linalg::TMatrix22<T>    vfc::linalg::inverse (const TMatrix22<T>& mat)
{    
    // only support inverse for floating types
    VFC_STATIC_ASSERT(vfc::TIsFloating<T>::value);
    
    const    T   d    = det(mat);
    
    VFC_REQUIRE2(!isZero(d),"TMatrix22<T> is singular");
        
    const    T    oneOverDet = static_cast<T>(1)/d;
    
    return TMatrix22<T>    (     oneOverDet*mat(1,1), -oneOverDet*mat(0,1), 
                            -oneOverDet*mat(1,0),  oneOverDet*mat(0,0)
                        );
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_matrix22_ops.inl  $
//  Revision 1.1 2007/05/09 10:21:20MESZ Jaeger Thomas (CC-DA/EPV2) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
