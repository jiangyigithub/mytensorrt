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
*     $Source: vfc_mat3_ops.inl $
*     $Revision: 1.7 $
*     $Author: Muehlmann Karsten (CC/EYN2) (muk2lr) $
*     $Date: 2007/03/16 11:35:07MEZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

#include "vfc/core/vfc_assert.hpp"

template <class T>    inline 
T    vfc::det    (const TMatrix3<T>& f_mat)
{    
    return        (    f_mat(2,0) * ( f_mat(0,1)*f_mat(1,2)-f_mat(1,1)*f_mat(0,2)) )
            -   (    f_mat(2,1) * ( f_mat(0,0)*f_mat(1,2)-f_mat(1,0)*f_mat(0,2)) )
            +   (    f_mat(2,2) * ( f_mat(0,0)*f_mat(1,1)-f_mat(1,0)*f_mat(0,1)) );
}

template <class T>    inline 
T    vfc::trace    (const TMatrix3<T>& f_mat)
{    
    return    ( f_mat(0,0) + f_mat(1,1) + f_mat(2,2) );
}

template <class T>    inline  
vfc::TMatrix3<T>    vfc::inverse (const TMatrix3<T>& f_mat)
{    
    // only support inverse for floating types
    VFC_STATIC_ASSERT(vfc::TIsFloating<T>::value);
    
    const    T   d    = det(f_mat);
    
    VFC_REQUIRE2(!isZero(d), "TMatrix3<T> is singular");
        
    const    T    oneOverDet = static_cast<T>(1)/d;
    
    return TMatrix3<T>    (    oneOverDet * ( f_mat(1,1)*f_mat(2,2) - f_mat(2,1)*f_mat(1,2) ),
                            oneOverDet * ( f_mat(2,1)*f_mat(0,2) - f_mat(2,2)*f_mat(0,1) ),
                            oneOverDet * ( f_mat(0,1)*f_mat(1,2) - f_mat(1,1)*f_mat(0,2) ),
                            oneOverDet * ( f_mat(2,0)*f_mat(1,2) - f_mat(1,0)*f_mat(2,2) ),
                            oneOverDet * ( f_mat(0,0)*f_mat(2,2) - f_mat(2,0)*f_mat(0,2) ),
                            oneOverDet * ( f_mat(1,0)*f_mat(0,2) - f_mat(0,0)*f_mat(1,2) ),
                            oneOverDet * ( f_mat(1,0)*f_mat(2,1) - f_mat(2,0)*f_mat(1,1) ),
                            oneOverDet * ( f_mat(2,0)*f_mat(0,1) - f_mat(0,0)*f_mat(2,1) ),
                            oneOverDet * ( f_mat(0,0)*f_mat(1,1) - f_mat(1,0)*f_mat(0,1) )
                        );
}

template <class T>    inline  
vfc::TMatrix3<T>    vfc::transpose (const TMatrix3<T>& f_mat)
{    
    return TMatrix3<T>    (    f_mat(0,0), f_mat(1,0), f_mat(2,0),
                            f_mat(0,1), f_mat(1,1), f_mat(2,1),
                            f_mat(0,2), f_mat(1,2), f_mat(2,2)    );

}

template <class T>    inline  
void    vfc::identity (TMatrix3<T>& f_mat)
{    
    f_mat.set    (    static_cast<T>(1), static_cast<T>(0), static_cast<T>(0),
                    static_cast<T>(0), static_cast<T>(1), static_cast<T>(0),
                    static_cast<T>(0), static_cast<T>(0), static_cast<T>(1)
            );
}

template <class T>    inline
vfc::TMatrix3<T>        vfc::operator-    (const TMatrix3<T>& f_mat)
{
    return TMatrix3<T>    (    -f_mat(0,0), -f_mat(0,1), -f_mat(0,2),
                            -f_mat(1,0), -f_mat(1,1), -f_mat(1,2),
                            -f_mat(2,0), -f_mat(2,1), -f_mat(2,2)    );
}

// binary

template <class T>    inline
vfc::TMatrix3<T>        vfc::operator*    (const TMatrix3<T>& f_op1, const TMatrix3<T>& f_op2)
{
    return TMatrix3<T> (    f_op1(0,0)*f_op2(0,0) + f_op1(0,1)*f_op2(1,0) + f_op1(0,2)*f_op2(2,0),
                            f_op1(0,0)*f_op2(0,1) + f_op1(0,1)*f_op2(1,1) + f_op1(0,2)*f_op2(2,1),
                            f_op1(0,0)*f_op2(0,2) + f_op1(0,1)*f_op2(1,2) + f_op1(0,2)*f_op2(2,2),

                            f_op1(1,0)*f_op2(0,0) + f_op1(1,1)*f_op2(1,0) + f_op1(1,2)*f_op2(2,0),
                            f_op1(1,0)*f_op2(0,1) + f_op1(1,1)*f_op2(1,1) + f_op1(1,2)*f_op2(2,1),
                            f_op1(1,0)*f_op2(0,2) + f_op1(1,1)*f_op2(1,2) + f_op1(1,2)*f_op2(2,2),

                            f_op1(2,0)*f_op2(0,0) + f_op1(2,1)*f_op2(1,0) + f_op1(2,2)*f_op2(2,0),
                            f_op1(2,0)*f_op2(0,1) + f_op1(2,1)*f_op2(1,1) + f_op1(2,2)*f_op2(2,1),
                            f_op1(2,0)*f_op2(0,2) + f_op1(2,1)*f_op2(1,2) + f_op1(2,2)*f_op2(2,2));
}

template <class T>    inline
vfc::TMatrix3<T>        vfc::operator+    (const TMatrix3<T>& f_op1, const TMatrix3<T>& f_op2)
{
    return TMatrix3<T> (    f_op1(0,0) + f_op2(0,0), f_op1(0,1) + f_op2(0,1), f_op1(0,2) + f_op2(0,2),
                            f_op1(1,0) + f_op2(1,0), f_op1(1,1) + f_op2(1,1), f_op1(1,2) + f_op2(1,2),
                            f_op1(2,0) + f_op2(2,0), f_op1(2,1) + f_op2(2,1), f_op1(2,2) + f_op2(2,2));
}

template <class T>    inline
vfc::TMatrix3<T>        vfc::operator-    (const TMatrix3<T>& f_op1, const TMatrix3<T>& f_op2)
{
    return TMatrix3<T> (    f_op1(0,0) - f_op2(0,0), f_op1(0,1) - f_op2(0,1), f_op1(0,2) - f_op2(0,2),
                            f_op1(1,0) - f_op2(1,0), f_op1(1,1) - f_op2(1,1), f_op1(1,2) - f_op2(1,2),
                            f_op1(2,0) - f_op2(2,0), f_op1(2,1) - f_op2(2,1), f_op1(2,2) - f_op2(2,2));
}

template <class T>
void    vfc::fromXAxisRotation    (TMatrix3<T>& f_mat, const CRadian& f_angleX)
{
    T    cx = static_cast<T>(cos(f_angleX));
    T    sx = static_cast<T>(sin(f_angleX));

    f_mat.set(static_cast<T>(1),static_cast<T>(0),static_cast<T>(0),
             static_cast<T>(0), cx, -sx,
             static_cast<T>(0), sx, cx);
}


template <class T>
void    vfc::fromYAxisRotation    (TMatrix3<T>& f_mat, const CRadian& f_angleY)
{
    T    cy = static_cast<T>(cos(f_angleY));
    T    sy = static_cast<T>(sin(f_angleY));

    f_mat.set(cy,static_cast<T>(0),sy,
             static_cast<T>(0), static_cast<T>(1), static_cast<T>(0),
             -sy, static_cast<T>(0), cy);
}

template <class T>
void    vfc::fromZAxisRotation    (TMatrix3<T>& f_mat, const CRadian& f_angleZ)
{
    T    cz = static_cast<T>(cos(f_angleZ));
    T    sz = static_cast<T>(sin(f_angleZ));

    f_mat.set( cz,-sz, static_cast<T>(0),
               sz, cz, static_cast<T>(0),
               static_cast<T>(0), static_cast<T>(0), static_cast<T>(1));
}

template <class T>
void    vfc::fromZYXAxisRotation    (TMatrix3<T>& f_mat, const CRadian& f_angleX, const CRadian& f_angleY, const CRadian& f_angleZ)
{
    T    cx = static_cast<T>(cos(f_angleX));
    T    sx = static_cast<T>(sin(f_angleX));

    T    cy = static_cast<T>(cos(f_angleY));
    T    sy = static_cast<T>(sin(f_angleY));

    T    cz = static_cast<T>(cos(f_angleZ));
    T    sz = static_cast<T>(sin(f_angleZ));

    f_mat.set(   cy * cz,   sx * sy * cz - cx * sz,       cx * sy * cz + sx * sz,
                        cy * sz,   sx * sy * sz + cx * cz,       cx * sy * sz - sx * cz,
                        - sy,          sx * cy,                           cx * cy);
}

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_mat3_ops.inl  $
Revision 1.7 2007/03/16 11:35:07MEZ Muehlmann Karsten (CC/EYN2) (muk2lr) 
replaced exception by precondition (mantis1420)
Revision 1.6 2006/11/16 14:41:15CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.5 2006/07/07 15:23:27CEST Muehlmann Karsten (AE-DA/ESA3) (muk2lr) 
allow inverse only for floating point types (mantis1110)
Revision 1.4 2006/02/14 09:06:10CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-changed function names
Revision 1.3 2005/12/19 15:34:41CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-changed rotation functions
Revision 1.2 2005/12/15 10:17:52CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-added generation of rotation matrices
Revision 1.1 2005/12/15 10:03:44CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
Initial revision
********************************************************************/




