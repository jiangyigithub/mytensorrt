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
///     $Source: vfc_quat_ops.inl $
///     $Revision: 1.6 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/07/23 09:46:40MESZ $
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

#include "vfc/core/vfc_type_traits.hpp"     // used for TIsFloating<>
#include "vfc/core/vfc_static_assert.hpp"   // used for VFC_STATIC_ASSERT
#include "vfc/core/vfc_assert.hpp"          // used for VFC_REQUIRE(), VFC_REQUIRE2()
#include "vfc/core/vfc_math.hpp"            // used for isEqual(), sqrt(), 


template <class T>  inline
T   vfc::dot (const TQuaternion<T>& f_a_quat, const TQuaternion<T>& f_b_quat)
{
    return      f_a_quat.x()*f_b_quat.x()
            +   f_a_quat.y()*f_b_quat.y()
            +   f_a_quat.z()*f_b_quat.z()
            +   f_a_quat.w()*f_b_quat.w();
}


template <class T>  inline
vfc::TQuaternion<T>        vfc::conjugate    (const TQuaternion<T>& f_quat)
{
    return TQuaternion<T>(-f_quat.x(),-f_quat.y(),-f_quat.z(),f_quat.w());
}


template <class T>  inline
vfc::TQuaternion<T>        vfc::inverse        (const TQuaternion<T>& f_quat)
{
    VFC_REQUIRE2(   isEqual(static_cast<T>(1),length(f_quat)),
                    "quaternion has to be unit length to be inverted"
                );

    return conjugate(f_quat);
}


template <class T>  inline
void            vfc::identity    (TQuaternion<T>& f_quat)
{
    f_quat.set(    static_cast<T>(0),
                static_cast<T>(0),
                static_cast<T>(0),
                static_cast<T>(1));
}

template <class T>  inline
T    vfc::length        (const TQuaternion<T>& f_quat)
{
    const T sqrSum  =   dot(f_quat,f_quat);

    return static_cast<T>(sqrt(sqrSum));
}

template <class T>  inline
vfc::TQuaternion<T>        vfc::normalized    (const TQuaternion<T>& f_quat)
{
    return f_quat/length(f_quat);
}


template <class T>  inline
T    vfc::normalize    (TQuaternion<T>& f_quat)
{
    const T len = length(f_quat); 
    f_quat/=len; 
    return len;
}




template <class T>  inline
void    vfc::fromAxisAngleRotation    (TQuaternion<T>& f_quat, const T& f_xComp, const T& f_yComp, const T& f_zComp, const CRadian& f_phi)
{
    VFC_STATIC_ASSERT( TIsFloating<T>::value );

    const T norm = static_cast<T>(sqrt(sqr(f_xComp)+sqr(f_yComp)+sqr(f_zComp)));

    // sets quat to identity, if rotation axis degenerates to point at origin
    // don't assert

    if ( isZero(norm)) 
    {     
        identity(f_quat);
    }
    else
    {
        const   T       oneOverNorm     = static_cast<T>(1)/norm;
        const   CRadian angleHalf        = static_cast<T>(0.5)*f_phi;
        const   T       sinAngleHalf    = static_cast<T>(sin(angleHalf));

        // with unit length axis vector(x,y,z) 
        // => quat is unit getLength as well:  
        //      sin2*x2 + sin2*y2 + sin2*z2 + cos2 
        //  =   (x2+y2+z2)*sin2+cos2                    | with x2+y2+z2 == 1.
        //  =   sin2+cos2
        //  =   1.
        
        f_quat.set  (   (sinAngleHalf * f_xComp * oneOverNorm),
                        (sinAngleHalf * f_yComp * oneOverNorm),
                        (sinAngleHalf * f_zComp * oneOverNorm),
                        static_cast<T>(cos (angleHalf))
                    );
    }

}


template <class T, class AxisType>  inline
void    vfc::fromAxisAngleRotation    (TQuaternion<T>& f_quat, const AxisType& f_axis, const CRadian& f_phi)
{
    fromAxisAngleRotation   (   f_quat,
                                static_cast<T>(f_axis[0]),
                                static_cast<T>(f_axis[1]),
                                static_cast<T>(f_axis[2]),
                                f_phi
                            );
}



template <class T>  inline
void    vfc::fromXAxisRotation    (TQuaternion<T>& f_quat, const CRadian& f_angleX)
{
    fromAxisAngleRotation   (   f_quat,
                                static_cast<T>(1),
                                static_cast<T>(0),
                                static_cast<T>(0),
                                f_angleX
                            );
}



template <class T>  inline
void    vfc::fromYAxisRotation    (TQuaternion<T>& f_quat, const CRadian& f_angleY)
{
    fromAxisAngleRotation   (   f_quat,
                                static_cast<T>(0),
                                static_cast<T>(1),
                                static_cast<T>(0),
                                f_angleY
                            );
}



template <class T>  inline
void    vfc::fromZAxisRotation    (TQuaternion<T>& f_quat, const CRadian& f_angleZ)
{
    fromAxisAngleRotation   (   f_quat,
                                static_cast<T>(0),
                                static_cast<T>(0),
                                static_cast<T>(1),
                                f_angleZ
                            );
}

template <class T>  inline    
vfc::TQuaternion<T>    vfc::slerp    (   const TQuaternion<T>&   f_q_quat, 
                                    const TQuaternion<T>&   f_r_quat,
                                    const float64_t         f_t                 
                                )
{
    VFC_STATIC_ASSERT (TIsFloating<T>::value);

    // spherical linear interpolation (slerp)

    // function requires two unit length quaternions
    VFC_REQUIRE2(   (       isEqual(static_cast<T>(1),length(f_q_quat))
                        &&  isEqual(static_cast<T>(1),length(f_r_quat))),
                    "quaternions have to be unit length"
                );

    // function requires 0<=f_t<=1
    VFC_REQUIRE2(   ( 0 <= f_t && 1>= f_t), 
                    "interpolation parameter has to be between [0..1]"
                );
    
    // calculating _half_ angle phi between two unit length quaternions
    // cos(phi/2) = q0.w*q1.w + q0.x*q1.x + q0.y*q1.y + q0.z * q1.z 
    //            = q0 <dot> q1 

    T cosPhi = dot(f_q_quat, f_r_quat);

    T negate_2nd_if_needed = static_cast<T>(1);
    if (vfc::isNegative(cosPhi))
    {
        negate_2nd_if_needed = static_cast<T>(-1);
        cosPhi = -cosPhi;
    }
    
    if (cosPhi < static_cast<T>(1))
    {
        const CRadian   phiHalf(::acos(static_cast<float64_t>(cosPhi)));
    const float64_t sinPhiHalf = sin(phiHalf);

    // return original quaternion if phi is a multiple of 360 degrees 
    if (isZero(sinPhiHalf))
    {
        return f_q_quat;
    }

    const float64_t    oneOverSinPhiHalf    = 1./sinPhiHalf;

    const float64_t fac1 = sin( phiHalf*(1.-f_t))  * oneOverSinPhiHalf;
    const float64_t fac2 = sin( phiHalf*f_t)       * oneOverSinPhiHalf;

    return      (static_cast<T>(fac1)*f_q_quat)
                + (negate_2nd_if_needed*static_cast<T>(fac2)*f_r_quat);
    }
    else
    {
        return f_q_quat;
    }
}

template <class T>  inline    
vfc::TQuaternion<T>    vfc::squad    
(   const TQuaternion<T>& f_p_quat, 
    const TQuaternion<T>& f_a_quat, 
    const TQuaternion<T>& f_b_quat, 
    const TQuaternion<T>& f_q_quat, 
    const float64_t f_t)
{
    const float64_t k = 2. * f_t * (1.-f_t);

    TQuaternion<T>  slerpPQ = slerp( f_p_quat, f_q_quat, f_t);
    TQuaternion<T>  slerpAB = slerp( f_a_quat, f_b_quat, f_t);
    
    return slerp( slerpPQ, slerpAB, k);
}

//=============================================================================
//    vfc::operator*()
//-----------------------------------------------------------------------------
/// @author zvh2hi
/// @relates TQuaternion
/// @par Requirements: 
/// - vfc_quat_ops.hpp
//=============================================================================

template <class T>  inline
vfc::TQuaternion<T>        vfc::operator*    (const TQuaternion<T>& f_op1, const TQuaternion<T>& f_op2)
{
    return    TQuaternion<T>(        f_op1.y()*f_op2.z() + f_op1.x()*f_op2.w() + f_op1.w()*f_op2.x() - f_op1.z()*f_op2.y() ,
                                f_op1.z()*f_op2.x() + f_op1.y()*f_op2.w() + f_op1.w()*f_op2.y() - f_op1.x()*f_op2.z() ,
                                f_op1.x()*f_op2.y() + f_op1.z()*f_op2.w() + f_op1.w()*f_op2.z() - f_op1.y()*f_op2.x() ,
                                f_op1.w()*f_op2.w() - f_op1.x()*f_op2.x() - f_op1.y()*f_op2.y() - f_op1.z()*f_op2.z());

}

//=============================================================================
//    vfc::operator+()
//-----------------------------------------------------------------------------
/// @author zvh2hi
/// @relates TQuaternion
/// @par Requirements: 
/// - vfc_quat_ops.hpp
//=============================================================================

template <class T>  inline
vfc::TQuaternion<T>        vfc::operator+    (const TQuaternion<T>& f_op1, const TQuaternion<T>& f_op2)
{
    return    TQuaternion<T>  (   f_op1.x() + f_op2.x(), 
                                f_op1.y() + f_op2.y(),
                                f_op1.z() + f_op2.z(),
                                f_op1.w() + f_op2.w()
                            );    
}

//=============================================================================
//    vfc::operator-()
//-----------------------------------------------------------------------------
/// @author zvh2hi
/// @relates TQuaternion
/// @par Requirements: 
/// - vfc_quat_ops.hpp
//=============================================================================

template <class T>  inline
vfc::TQuaternion<T>        vfc::operator-    (const TQuaternion<T>& f_op1, const TQuaternion<T>& f_op2)
{
    return    TQuaternion<T>  (   f_op1.x() - f_op2.x(), 
                                f_op1.y() - f_op2.y(),
                                f_op1.z() - f_op2.z(),
                                f_op1.w() - f_op2.w()
                            );    
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_quat_ops.inl  $
//  Revision 1.6 2007/07/23 09:46:40MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - moved documentation to header (mantis1744)
//  Revision 1.5 2007/03/29 15:33:23CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced ::std with global namespace (mantis1534)
//  Revision 1.4 2007/01/23 15:09:49CET dkn2kor 
//  - removed extra bracket (mantis1390)
//  Revision 1.3 2006/11/16 19:11:19IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.2 2006/10/13 10:00:25CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed header/footer template    
//  - added documentation
//  Revision 1.1 2006/07/07 10:07:03CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/geo/geo.pj
//=============================================================================
