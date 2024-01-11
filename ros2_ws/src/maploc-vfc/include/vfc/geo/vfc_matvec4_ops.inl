/********************************************************************
* C O P Y R I G H T
*--------------------------------------------------------------------
* Copyright (c) 2005 by Robert Bosch GmbH. All rights reserved.
*
* This file is property of Robert Bosch GmbH. Any unauthorised copy,
* use or distribution is an offensive act against international law
* and may me prosecuted under federal law. Its content is company
* confidential.
*--------------------------------------------------------------------
* D E S C R I P T I O N
*--------------------------------------------------------------------
*   Projectname: ..\..\..\..\vfc\dummy\win32\vc_71\vfc_dummy
*      Synopsis: 
* Target system: 
*      Compiler: VS7.1
*--------------------------------------------------------------------
* N O T E S
*--------------------------------------------------------------------
* Notes: 
*--------------------------------------------------------------------
* I N I T I A L   A U T H O R   I D E N T I T Y
*--------------------------------------------------------------------
*       Name: 
* Department: 
*--------------------------------------------------------------------
* R E V I S I O N   I N F O R M A T I O N
*------------------------------------------------------------------*/
/*!   \file
*     \par Revision History
*     $Source: vfc_matvec4_ops.inl $
*     $Revision: 1.3 $
*     $Author: Muehlmann Karsten (CC/EYN2) (muk2lr) $
*     $Date: 2007/03/16 11:35:07MEZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

#include "vfc/core/vfc_assert.hpp"

template <class T>    inline
vfc::TMatrix4<T>    vfc::dyad        (const TVector4<T>& f_a_vec4, const TVector4<T>& f_b_vec4)
{
    return TMatrix4<T>    (    f_a_vec4.x()*f_b_vec4.x(), f_a_vec4.x()*f_b_vec4.y(), 
                            f_a_vec4.x()*f_b_vec4.z(), f_a_vec4.x()*f_b_vec4.w(),
                            f_a_vec4.y()*f_b_vec4.x(), f_a_vec4.y()*f_b_vec4.y(), 
                            f_a_vec4.y()*f_b_vec4.z(), f_a_vec4.y()*f_b_vec4.w(),
                            f_a_vec4.z()*f_b_vec4.x(), f_a_vec4.z()*f_b_vec4.y(), 
                            f_a_vec4.z()*f_b_vec4.z(), f_a_vec4.z()*f_b_vec4.w(),
                            f_a_vec4.w()*f_b_vec4.x(), f_a_vec4.w()*f_b_vec4.y(), 
                            f_a_vec4.w()*f_b_vec4.z(), f_a_vec4.w()*f_b_vec4.w()
                        );

}


template <class T>    inline  
vfc::TVector4<T>    vfc::operator*    (const TMatrix4<T>& f_mat4, const TVector4<T>& f_vec4)
{
    return TVector4<T> (    f_mat4(0,0)*f_vec4.x() + f_mat4(0,1)*f_vec4.y() + f_mat4(0,2)*f_vec4.z() + f_mat4(0,3)*f_vec4.w(),
                            f_mat4(1,0)*f_vec4.x() + f_mat4(1,1)*f_vec4.y() + f_mat4(1,2)*f_vec4.z() + f_mat4(1,3)*f_vec4.w(),
                            f_mat4(2,0)*f_vec4.x() + f_mat4(2,1)*f_vec4.y() + f_mat4(2,2)*f_vec4.z() + f_mat4(2,3)*f_vec4.w(),
                            f_mat4(3,0)*f_vec4.x() + f_mat4(3,1)*f_vec4.y() + f_mat4(3,2)*f_vec4.z() + f_mat4(3,3)*f_vec4.w()
                        );
}

template <class T>    inline  
vfc::TVector3<T>    vfc::operator*    (const TMatrix4<T>& f_mat4, const TVector3<T>& f_vec3)
{
    const T  denom = f_mat4(3,0)*f_vec3.x()+f_mat4(3,1)*f_vec3.y()+f_mat4(3,2)*f_vec3.z()+f_mat4(3,3);

    VFC_REQUIRE2( !isZero(denom), "Division by zero");

    const T    fac = static_cast<T>(1)/denom;

    return TVector3<T>(        fac*(f_mat4(0,0)*f_vec3.x() + f_mat4(0,1)*f_vec3.y() + f_mat4(0,2)*f_vec3.z() + f_mat4(0,3)),
                            fac*(f_mat4(1,0)*f_vec3.x() + f_mat4(1,1)*f_vec3.y() + f_mat4(1,2)*f_vec3.z() + f_mat4(1,3)),
                            fac*(f_mat4(2,0)*f_vec3.x() + f_mat4(2,1)*f_vec3.y() + f_mat4(2,2)*f_vec3.z() + f_mat4(2,3)));
}

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_matvec4_ops.inl  $
Revision 1.3 2007/03/16 11:35:07MEZ Muehlmann Karsten (CC/EYN2) (muk2lr) 
replaced exception by precondition (mantis1420)
Revision 1.2 2006/11/16 14:41:19CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
********************************************************************/
