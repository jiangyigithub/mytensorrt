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
*     $Source: vfc_vec2_ops.inl $
*     $Revision: 1.15 $
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
vfc::TVector2<T>        vfc::abs    (const TVector2<T>& vec)
{
    return TVector2<T>    (    abs(vec.x()),
                            abs(vec.y())
                        );
}

template <class T>    inline
vfc::TVector2<T>        vfc::min    (const TVector2<T>& op1, const TVector2<T>& op2)
{
    return TVector2<T>    (    (stlalias::min)(op1.x(),op2.x()),
                            (stlalias::min)(op1.y(),op2.y())
                        );
}

template <class T>    inline
vfc::TVector2<T>        vfc::max    (const TVector2<T>& op1, const TVector2<T>& op2)
{
    return TVector2<T>    (    (stlalias::max)(op1.x(),op2.x()),
                            (stlalias::max)(op1.y(),op2.y())
                        );
}

template <class T>    inline
T    vfc::sum    (const TVector2<T>& vec)
{
    return (vec.x()+vec.y());
}

template <class T>    inline
T    vfc::dot    (const TVector2<T>& op1, const TVector2<T>& op2)
{
    return (op1.x()*op2.x() + op1.y()*op2.y());
}

template <class T>    inline
T    vfc::cross    (const TVector2<T>& op1, const TVector2<T>& op2)
{
    return (    op1.x()*op2.y()
            -   op1.y()*op2.x());
}

template <class T>    inline
T    vfc::norm_L1        (const TVector2<T>& vec)
{
    return sum(abs(vec));
}

template <class T>    inline
T    vfc::norm_L2        (const TVector2<T>& vec)
{
    return static_cast<T>(::sqrt(static_cast<float64_t>(dot(vec,vec))));
}

template <class T>    inline
T    vfc::norm_Linf    (const TVector2<T>& vec)
{
    return (stlalias::max)(abs(vec.x()),abs(vec.y()));
}

template <class T>    inline
T    vfc::length        (const TVector2<T>& vec)
{
    return norm_L2(vec);
}

template <class T>    inline
vfc::TVector2<T>        vfc::normalized    (const TVector2<T>& vec)
{
    return vec/length(vec);
}

template <class T>    inline
T    vfc::normalize    (TVector2<T>& vec)
{
    const T len = length(vec);
    vec/=len;
    return len;
}

template <class T>    inline
vfc::CRadian        vfc::angle    (const TVector2<T>& f_op1, const TVector2<T>& f_op2)
{
    const float64_t num     = static_cast<float64_t>(dot(f_op1,f_op2));
    const float64_t denom   = ::sqrt(static_cast<float64_t>( dot(f_op1,f_op1) * dot(f_op2,f_op2) ));
    
    VFC_REQUIRE( notZero(denom) );
    VFC_REQUIRE( denom >= vfc::abs(num) );
    
    return CRadian    (::acos(num/denom));
}

template <class T>    inline
T    vfc::area    (const TVector2<T>& op1, const TVector2<T>& op2)
{
    return cross(op1,op2);
}

template <class T>    inline
vfc::TVector2<T>    vfc::operator-    (const TVector2<T>& op1, const TVector2<T>& op2)
{
    return TVector2<T>    (    op1.x()-op2.x(),
                            op1.y()-op2.y()
                        );
}

template <class T>    inline
vfc::TVector2<T>    vfc::operator+    (const TVector2<T>& op1, const TVector2<T>& op2)
{
    return TVector2<T>    (    op1.x()+op2.x(),
                            op1.y()+op2.y()
                        );
}

template <class T>    inline
vfc::TVector2<T>    vfc::operator-    (const TVector2<T>& vec)
{
    return TVector2<T>    (    -vec.x(),
                            -vec.y()
                        );
}

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_vec2_ops.inl  $
Revision 1.15 2008/10/07 14:02:21MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- rewrote vfc::angle() function (mantis 2369)
Revision 1.14 2008/07/11 10:52:27CEST Dilip Krishna (RBEI/EAE6) (dkn2kor) 
- Removed paranthesis added by mistake (mantis2168)
Revision 1.13 2008/07/10 17:37:20IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
- fixed min/max errors due to macro definitions of min & max (mantis2168)
Revision 1.12 2007/03/29 19:04:41IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
- replaced ::std with stlalias (mantis1534)
Revision 1.11 2007/03/16 11:35:08CET Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
replaced exception by precondition (mantis1420)
Revision 1.10 2006/11/16 14:41:19CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
- replaced tabs with 4 spaces (mantis1294)
Revision 1.9 2006/03/03 10:39:37CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-added missing <algorithm> include (mantis1015)
Revision 1.8 2006/01/26 12:38:13CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-fixed bug in normalized() function (mantis:967)
Revision 1.7 2005/12/19 15:38:55CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-normalize() returns old length again
Revision 1.6 2005/12/19 15:21:15CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-renamed as_normalized() to normalized()
Revision 1.5 2005/12/07 16:11:48CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-restructuring
Revision 1.4 2005/11/30 16:18:16CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-fixed l2 norm calculation
Revision 1.3 2005/11/30 09:59:49CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-more ops
Revision 1.2 2005/11/08 12:19:42CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-update
********************************************************************/
