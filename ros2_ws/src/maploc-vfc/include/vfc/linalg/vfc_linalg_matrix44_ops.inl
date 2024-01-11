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
//       Projectname: vfc/linalg
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
//        Name: zvh2hi
//  Department: CR/AEM
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_linalg_matrix44_ops.inl $
///     $Revision: 1.2 $
///     $Author: gaj2kor $
///     $Date: 2008/08/29 15:04:51MESZ $
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


#include "vfc/core/vfc_math.hpp"            // used for isZero()
#include "vfc/core/vfc_static_assert.hpp"   // used for VFC_STATIC_ASSERT()
#include "vfc/core/vfc_assert.hpp"   // used for VFC_ASSERT2()
#include "vfc/core/vfc_metaprog.hpp"        // used for TIsFloating<>

template <class T>    inline
T    vfc::linalg::det    (const TMatrix44<T>& mat)
{
    const T da = mat(0,2)*mat(1,3) - mat(1,2)*mat(0,3);
    const T db = mat(0,1)*mat(1,3) - mat(1,1)*mat(0,3);
    const T dc = mat(0,1)*mat(1,2) - mat(1,1)*mat(0,2);
    const T dd = mat(0,0)*mat(1,3) - mat(1,0)*mat(0,3);
    const T de = mat(0,0)*mat(1,2) - mat(1,0)*mat(0,2);
    const T df = mat(0,0)*mat(1,1) - mat(1,0)*mat(0,1);

    return    (    -    mat(3,0)*( mat(2,1)*da - mat(2,2)*db + mat(2,3)*dc)
                +    mat(3,1)*( mat(2,0)*da - mat(2,2)*dd + mat(2,3)*de)
                -    mat(3,2)*( mat(2,0)*db - mat(2,1)*dd + mat(2,3)*df)
                +    mat(3,3)*( mat(2,0)*dc - mat(2,1)*de + mat(2,2)*df)
            );
}

template <class T>    inline
T    vfc::linalg::trace    (const TMatrix44<T>& mat)
{
    return    ( mat(0,0) + mat(1,1) + mat(2,2) + mat(3,3) );
}

template <class T>    inline
vfc::linalg::TMatrix44<T>    vfc::linalg::inverse (const TMatrix44<T>& mat)
{
    // only support inverse for floating types
    VFC_STATIC_ASSERT(vfc::TIsFloating<T>::value);

    const    T deta = mat(0,2)*mat(1,3) - mat(0,3)*mat(1,2);
    const    T detb = mat(0,1)*mat(1,3) - mat(0,3)*mat(1,1);
    const    T detc = mat(0,1)*mat(1,2) - mat(0,2)*mat(1,1);
    const    T detd = mat(0,0)*mat(1,3) - mat(1,0)*mat(0,3);
    const    T dete = mat(0,0)*mat(1,2) - mat(1,0)*mat(0,2);
    const    T detf = mat(0,0)*mat(1,1) - mat(1,0)*mat(0,1);

    const T d =    -    mat(3,0)*( mat(2,1)*deta - mat(2,2)*detb + mat(2,3)*detc)
                +    mat(3,1)*( mat(2,0)*deta - mat(2,2)*detd + mat(2,3)*dete)
                -    mat(3,2)*( mat(2,0)*detb - mat(2,1)*detd + mat(2,3)*detf)
                +    mat(3,3)*( mat(2,0)*detc - mat(2,1)*dete + mat(2,2)*detf);

    VFC_REQUIRE2(!isZero(d),"TMatrix44<T> is singular");

    const    T    oneOverDet = static_cast<T>(1)/d;

    const    T detg = mat(2,2)*mat(3,3) - mat(2,3)*mat(3,2);
    const    T deth = mat(1,2)*mat(3,3) - mat(1,3)*mat(3,2);
    const    T deti = mat(1,2)*mat(2,3) - mat(1,3)*mat(2,2);
    const    T detj = mat(2,1)*mat(3,3) - mat(2,3)*mat(3,1);
    const    T detk = mat(1,1)*mat(3,3) - mat(1,3)*mat(3,1);
    const    T detl = mat(1,1)*mat(2,3) - mat(1,3)*mat(2,1);
    const    T detm = mat(2,1)*mat(3,2) - mat(2,2)*mat(3,1);
    const    T detn = mat(1,1)*mat(3,2) - mat(1,2)*mat(3,1);
    const    T deto = mat(1,1)*mat(2,2) - mat(1,2)*mat(2,1);
    const    T detp = mat(0,2)*mat(3,3) - mat(0,3)*mat(3,2);
    const    T detq = mat(0,2)*mat(2,3) - mat(0,3)*mat(2,2);
    const    T detr = mat(0,1)*mat(3,3) - mat(0,3)*mat(3,1);
    const    T dets = mat(0,1)*mat(2,3) - mat(0,3)*mat(2,1);
    const    T dett = mat(0,1)*mat(3,2) - mat(0,2)*mat(3,1);
    const    T detu = mat(0,1)*mat(2,2) - mat(0,2)*mat(2,1);

    return TMatrix44<T> (    oneOverDet*(    mat(1,1)*detg - mat(2,1)*deth + mat(3,1)*deti),
                            oneOverDet*(-    mat(0,1)*detg + mat(2,1)*detp - mat(3,1)*detq),
                            oneOverDet*(    mat(0,1)*deth - mat(1,1)*detp + mat(3,1)*deta),
                            oneOverDet*(-    mat(0,1)*deti + mat(1,1)*detq - mat(2,1)*deta),

                            oneOverDet*(-    mat(1,0)*detg + mat(2,0)*deth - mat(3,0)*deti),
                            oneOverDet*(    mat(0,0)*detg - mat(2,0)*detp + mat(3,0)*detq),
                            oneOverDet*(-    mat(0,0)*deth + mat(1,0)*detp - mat(3,0)*deta),
                            oneOverDet*(    mat(0,0)*deti - mat(1,0)*detq + mat(2,0)*deta),

                            oneOverDet*(    mat(1,0)*detj - mat(2,0)*detk + mat(3,0)*detl),
                            oneOverDet*(-    mat(0,0)*detj + mat(2,0)*detr - mat(3,0)*dets),
                            oneOverDet*(    mat(0,0)*detk - mat(1,0)*detr + mat(3,0)*detb),
                            oneOverDet*(-    mat(0,0)*detl + mat(1,0)*dets - mat(2,0)*detb),

                            oneOverDet*(-    mat(1,0)*detm + mat(2,0)*detn - mat(3,0)*deto),
                            oneOverDet*(    mat(0,0)*detm - mat(2,0)*dett + mat(3,0)*detu),
                            oneOverDet*(-    mat(0,0)*detn + mat(1,0)*dett - mat(3,0)*detc),
                            oneOverDet*(    mat(0,0)*deto - mat(1,0)*detu + mat(2,0)*detc)
                        );
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_matrix44_ops.inl  $
//  Revision 1.2 2008/08/29 15:04:51MESZ gaj2kor 
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.1 2007/05/09 13:51:21IST Jaeger Thomas (CC-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
