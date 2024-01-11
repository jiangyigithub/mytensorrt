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
///     $Source: vfc_matquat_ops.inl $
///     $Revision: 1.5 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/12/02 14:58:42MEZ $
///     $Locker:  $
///     $Name:  $
///     $State: in_work $
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

#include "vfc/core/vfc_types.hpp"           // needed for int32_t
#include "vfc/core/vfc_assert.hpp"          // needed for VFC_ASSERT(), VFC_REQUIRE()
#include "vfc/core/vfc_static_assert.hpp"   // needed for VFC_STATIC_ASSERT()
#include "vfc/core/vfc_math.hpp"            // needed for isEqual(), sqrt(), isPositive()
#include "vfc/geo/vfc_mat3_ops.hpp"         // needed for trace()



template <class T>  inline
vfc::TQuaternion<T>  vfc::toQuaternion (const TMatrix3<T>& f_mat3)
{
    VFC_STATIC_ASSERT(TIsFloating<T>::value);
    
    const   T   ONE_T = static_cast<T>(1);
    const   T   TWO_T = static_cast<T>(2);

    VFC_REQUIRE ( isEqual(ONE_T, det(f_mat3), static_cast<T>(0.0001)));

    const   T   tracem = trace(f_mat3)+ONE_T;
    
    T help1;
    T help2;
    
    TQuaternion<T> retq;

    if (isPositive(tracem))    //tracem is positive
    {
        help1= static_cast<T>(sqrt(tracem));
        help2= ONE_T/(TWO_T*help1);

        retq.set            (  ( f_mat3(2,1)-f_mat3(1,2) ) * help2,
                                ( f_mat3(0,2)-f_mat3(2,0) ) * help2,
                                ( f_mat3(1,0)-f_mat3(0,1) ) * help2,
                                help1/TWO_T);
    }
    else                //tracem is negative
    {        
        int32_t    maxel = 0;

        // find max diagonal element
        if (f_mat3(1,1) > f_mat3(maxel,maxel)) 
        {
            maxel = 1;
        }
        if (f_mat3(2,2) > f_mat3(maxel,maxel)) 
        {
            maxel = 2;
        }

        switch (maxel)
        {
            //--------------------------------------------------------------------------------------
            case 0:     help1 = static_cast<T>(sqrt(f_mat3(0,0)-f_mat3(1,1)-f_mat3(2,2)+ONE_T));
                        help2 = ONE_T/(TWO_T*help1);    

                        retq.set    (   help1/TWO_T,
                                    (f_mat3(0,1)+f_mat3(1,0))*help2,
                                    (f_mat3(0,2)+f_mat3(2,0))*help2,
                                    (f_mat3(2,1)-f_mat3(1,2))*help2);
                        break;  
            //--------------------------------------------------------------------------------------
            case 1:     help1 = static_cast<T>(sqrt(f_mat3(1,1)-f_mat3(0,0)-f_mat3(2,2)+ONE_T));
                        help2 = ONE_T/(TWO_T*help1);
                        
                        retq.set    (   (f_mat3(0,1)+f_mat3(1,0))*help2,
                                         help1/TWO_T,
                                        (f_mat3(1,2)+f_mat3(2,1))*help2,
                                        (f_mat3(0,2)-f_mat3(2,0))*help2);
                        break;
            //--------------------------------------------------------------------------------------
            case 2:     help1 = static_cast<T>(sqrt(f_mat3(2,2)-f_mat3(0,0)-f_mat3(1,1)+ONE_T));    //2*z
                        help2 = ONE_T/(TWO_T*help1);    // 1/(4*z)
                        
                        retq.set  (   (f_mat3(0,2)+f_mat3(2,0))*help2,
                                      (f_mat3(1,2)+f_mat3(2,1))*help2,
                                       help1/TWO_T,
                                      (f_mat3(1,0)-f_mat3(0,1))*help2);
                        break;
            //--------------------------------------------------------------------------------------/
            default:    VFC_ASSERT(false);  // we should never get here!!
                        break;
            
        }
    }

    return retq;
}



template <class T>  inline
vfc::TMatrix3<T>     vfc::toMatrix3     (const TQuaternion<T>& f_quat)
{
    const   T   ONE_T = static_cast<T>(1);
    const   T   TWO_T = static_cast<T>(2);

    VFC_REQUIRE2(   isEqual(ONE_T,length(f_quat)),
                    "quaternion has to be unit length");

    const T qxx = f_quat.x()*f_quat.x();
    const T qyy = f_quat.y()*f_quat.y();
    const T qzz = f_quat.z()*f_quat.z();
    const T qww = f_quat.w()*f_quat.w();

    const T qxy = f_quat.x()*f_quat.y();
    const T qxz = f_quat.x()*f_quat.z();

    const T qyz = f_quat.y()*f_quat.z();
    
    const T qwx = f_quat.w()*f_quat.x();
    const T qwy = f_quat.w()*f_quat.y();
    const T qwz = f_quat.w()*f_quat.z();

    return TMatrix3<T>  (   qww+qxx-qyy-qzz,    TWO_T*(qxy-qwz),    TWO_T*(qxz+qwy),
                            TWO_T*(qxy+qwz),    qww-qxx+qyy-qzz,    TWO_T*(qyz-qwx),
                            TWO_T*(qxz-qwy),    TWO_T*(qyz+qwx),    qww-qxx-qyy+qzz );
}

template <class T>  inline
vfc::TMatrix4<T>     vfc::toMatrix4     (const TQuaternion<T>& f_quat)
{
    const T ZERO_T  = static_cast<T>(0);
    const T ONE_T   = static_cast<T>(1);

    TMatrix3<T> mat3 = toMatrix3(f_quat);
    
    return TMatrix4<T> (    mat3(0,0),  mat3(0,1),  mat3(0,2),  ZERO_T,
                            mat3(1,0),  mat3(1,1),  mat3(1,2),  ZERO_T,
                            mat3(2,0),  mat3(2,1),  mat3(2,2),  ZERO_T,
                            ZERO_T,     ZERO_T,     ZERO_T,     ONE_T);
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_matquat_ops.inl  $
//  Revision 1.5 2014/12/02 14:58:42MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_matquat_ops.inl: compiler warnings (mantis0004059)
//  Revision 1.8 2014/05/28 14:41:33MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  = vfc mirror 1.24
//  Revision 1.4 2007/07/23 09:44:07MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - moved documentation to header (mantis1744)
//  Revision 1.3 2006/11/16 14:41:05CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.2 2006/10/13 10:08:19CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed header/footer templates
//  Revision 1.1 2006/07/07 10:07:03CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/geo/geo.pj
//=============================================================================
