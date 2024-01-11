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
*   Projectname: vfc/geo
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
*     $Source: vfc_vec3_ops.inl $
*     $Revision: 1.16 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (ZVH2HI) $
*     $Date: 2008/10/07 14:02:21MESZ $
*     $Locker:  $
*     $Name: 0032 RC1  $
*     $State: in_work $
*/
/*******************************************************************/

// stdlib includes
#include <cmath>
#include <algorithm>    // used for min(),max()

// vfc/core includes
#include "vfc/core/vfc_math.hpp"

template <class T>    inline
vfc::TVector3<T>        vfc::abs    (const TVector3<T>& f_vec3)
{
    return TVector3<T>  (   abs(f_vec3.x()),
                            abs(f_vec3.y()),
                            abs(f_vec3.z())
                        );
}

template <class T>    inline
vfc::TVector3<T>        vfc::min    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3)
{
    return TVector3<T>    (    (stlalias::min)(f_a_vec3.x(),f_b_vec3.x()),
                            (stlalias::min)(f_a_vec3.y(),f_b_vec3.y()),
                            (stlalias::min)(f_a_vec3.z(),f_b_vec3.z())
                        );
}

template <class T>    inline
vfc::TVector3<T>        vfc::max    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3)
{
    return TVector3<T>    (    (stlalias::max)(f_a_vec3.x(),f_b_vec3.x()),
                            (stlalias::max)(f_a_vec3.y(),f_b_vec3.y()),
                            (stlalias::max)(f_a_vec3.z(),f_b_vec3.z())
                        );
}

template <class T>    inline
T    vfc::sum    (const TVector3<T>& f_vec3)
{
    return (f_vec3.x()+f_vec3.y()+f_vec3.z());
}

template <class T>    inline
T    vfc::dot    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3)
{
    return (f_a_vec3.x()*f_b_vec3.x() + f_a_vec3.y()*f_b_vec3.y() + f_a_vec3.z()*f_b_vec3.z());
}

template <class T>    inline
vfc::TVector3<T>        vfc::cross    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3)
{
    return TVector3<T>    (    f_a_vec3.y()*f_b_vec3.z()-f_a_vec3.z()*f_b_vec3.y(),
                            f_a_vec3.z()*f_b_vec3.x()-f_a_vec3.x()*f_b_vec3.z(),
                            f_a_vec3.x()*f_b_vec3.y()-f_a_vec3.y()*f_b_vec3.x());
}

template <class T>    inline
T    vfc::norm_L1        (const TVector3<T>& f_vec3)
{
    return sum(abs(f_vec3));
}

template <class T>    inline
T    vfc::norm_L2        (const TVector3<T>& f_vec3)
{
    return static_cast<T>(::sqrt(static_cast<float64_t>(dot(f_vec3,f_vec3))));
}

template <class T>    inline
T    vfc::norm_Linf    (const TVector3<T>& f_vec3)
{
    return stlalias::max(   abs(f_vec3.x()),
                            stlalias::max(  abs(f_vec3.y()),
                                            abs(f_vec3.z())));
}

template <class T>    inline
T    vfc::length        (const TVector3<T>& f_vec3)
{
    return norm_L2(f_vec3);
}

template <class T>    inline
vfc::TVector3<T>        vfc::normalized    (const TVector3<T>& f_vec3)
{
    return f_vec3/length(f_vec3);
}

template <class T>    inline
T    vfc::normalize    (TVector3<T>& f_vec3)
{
    const T len = length(f_vec3);
    f_vec3/=len;
    return len;
}

template <class T>    inline
vfc::CRadian        vfc::angle    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3)
{
    const float64_t num     = static_cast<float64_t>(dot(f_a_vec3,f_b_vec3));
    const float64_t denom   = ::sqrt(static_cast<float64_t>( dot(f_a_vec3,f_a_vec3) * dot(f_b_vec3,f_b_vec3) ));
    
    VFC_REQUIRE( notZero(denom) );
    VFC_REQUIRE( denom >= vfc::abs(num) );
    
    return CRadian    (::acos(num/denom));
}

template <class T>    inline
T    vfc::area    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3)
{
    return length(cross(f_a_vec3,f_b_vec3));
}

template <class T>    inline
T    vfc::volume    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3, const TVector3<T>& f_c_vec3)
{
    // return dot(f_a_vec3,cross(f_b_vec3,f_c_vec3));

    return        f_a_vec3.x()*(f_b_vec3.y()*f_c_vec3.z()-f_b_vec3.z()*f_c_vec3.y())
            +    f_a_vec3.y()*(f_b_vec3.z()*f_c_vec3.x()-f_b_vec3.x()*f_c_vec3.z())
            +    f_a_vec3.z()*(f_b_vec3.x()*f_c_vec3.y()-f_b_vec3.y()*f_c_vec3.x());
}

template <class T>    inline
vfc::TVector3<T>    vfc::operator-    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3)
{
    return TVector3<T>    (    f_a_vec3.x()-f_b_vec3.x(),
                            f_a_vec3.y()-f_b_vec3.y(),
                            f_a_vec3.z()-f_b_vec3.z()
                        );
}

template <class T>    inline
vfc::TVector3<T>    vfc::operator+    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3)
{
    return TVector3<T>    (    f_a_vec3.x()+f_b_vec3.x(),
                            f_a_vec3.y()+f_b_vec3.y(),
                            f_a_vec3.z()+f_b_vec3.z()
                        );
}

template <class T>    inline
vfc::TVector3<T>    vfc::operator-    (const TVector3<T>& f_vec3)
{
    return TVector3<T>    (    -f_vec3.x(),
                            -f_vec3.y(),
                            -f_vec3.z()
                        );
}

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_vec3_ops.inl  $
Revision 1.16 2008/10/07 14:02:21MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- rewrote vfc::angle() function (mantis 2369)
Revision 1.15 2008/07/11 10:52:26CEST Dilip Krishna (RBEI/EAE6) (dkn2kor) 
- Removed paranthesis added by mistake (mantis2168)
Revision 1.14 2008/07/10 17:37:18IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
- fixed min/max errors due to macro definitions of min & max (mantis2168)
Revision 1.13 2007/03/29 19:05:31IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
- replaced ::std with stlalias (mantis1534)
Revision 1.12 2007/03/16 11:35:07CET Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
replaced exception by precondition (mantis1420)
Revision 1.11 2006/11/16 14:41:10CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
- replaced tabs with 4 spaces (mantis1294)
Revision 1.10 2006/03/03 10:41:04CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-added missing <algorithm> include (mantis1015)
Revision 1.9 2006/01/26 12:38:13CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-fixed bug in normalized() function (mantis:967)
Revision 1.8 2005/12/19 15:38:54CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-normalize() returns old length again
Revision 1.7 2005/12/19 15:21:14CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-renamed as_normalized() to normalized()
Revision 1.6 2005/12/07 16:11:46CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-restructuring
Revision 1.5 2005/11/30 16:18:16CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-fixed l2 norm calculation
Revision 1.4 2005/11/30 09:59:47CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-more ops
Revision 1.3 2005/11/08 08:51:38CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-added inline to all definitions
Revision 1.2 2005/11/07 18:21:01CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-changed project name to vfc/geo
********************************************************************/
