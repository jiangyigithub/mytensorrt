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
*     $Source: vfc_mat2_ops.inl $
*     $Revision: 1.7 $
*     $Author: Muehlmann Karsten (CC/EYN2) (muk2lr) $
*     $Date: 2007/03/16 11:35:08MEZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

#include "vfc/core/vfc_assert.hpp"

template <class T>    inline 
T    vfc::det    (const TMatrix2<T>& mat)
{    
    return    ( mat(0,0)*mat(1,1) - mat(1,0)*mat(0,1) );
}

template <class T>    inline 
T    vfc::trace    (const TMatrix2<T>& mat)
{    
    return    ( mat(0,0) + mat(1,1) );
}

template <class T>    inline  
vfc::TMatrix2<T>    vfc::inverse (const TMatrix2<T>& mat)
{    
    // only support inverse for floating types
    VFC_STATIC_ASSERT(vfc::TIsFloating<T>::value);
    
    const    T   d    = det(mat);
    
    VFC_REQUIRE2(!isZero(d), "TMatrix2<T> is singular");
        
    const    T    oneOverDet = static_cast<T>(1)/d;
    
    return TMatrix2<T>    (     oneOverDet*mat(1,1), -oneOverDet*mat(0,1), 
                            -oneOverDet*mat(1,0),  oneOverDet*mat(0,0)
                        );
}

template <class T>    inline  
vfc::TMatrix2<T>    vfc::transpose (const TMatrix2<T>& mat)
{    
    return TMatrix2<T>    (    mat(0,0), mat(1,0), 
                            mat(0,1), mat(1,1)
                        );

}

template <class T>    inline  
void    vfc::identity (TMatrix2<T>& mat)
{    
    mat.set    (    static_cast<T>(1), static_cast<T>(0), 
                static_cast<T>(0), static_cast<T>(1)
            );
}

template <class T>    inline
vfc::TMatrix2<T>        vfc::operator-    (const TMatrix2<T>& mat)
{
    return TMatrix2<T>    (    -mat(0,0), -mat(0,1), 
                            -mat(1,0), -mat(1,1)
                        );
}

// binary

template <class T>    inline
vfc::TMatrix2<T>        vfc::operator*    (const TMatrix2<T>& op1, const TMatrix2<T>& op2)
{
    return TMatrix2<T> (    op1(0,0)*op2(0,0) + op1(0,1)*op2(1,0), op1(0,0)*op2(0,1) + op1(0,1)*op2(1,1),
                            op1(1,0)*op2(0,0) + op1(1,1)*op2(1,0), op1(1,0)*op2(0,1) + op1(1,1)*op2(1,1)
                        );
}

template <class T>    inline
vfc::TMatrix2<T>        vfc::operator+    (const TMatrix2<T>& op1, const TMatrix2<T>& op2)
{
    return TMatrix2<T> (    op1(0,0) + op2(0,0), op1(0,1) + op2(0,1), 
                            op1(1,0) + op2(1,0), op1(1,1) + op2(1,1)
                        );
}

template <class T>    inline
vfc::TMatrix2<T>        vfc::operator-    (const TMatrix2<T>& op1, const TMatrix2<T>& op2)
{
    return TMatrix2<T> (    op1(0,0) - op2(0,0), op1(0,1) - op2(0,1),
                            op1(1,0) - op2(1,0), op1(1,1) - op2(1,1)
                        );
}

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_mat2_ops.inl  $
Revision 1.7 2007/03/16 11:35:08MEZ Muehlmann Karsten (CC/EYN2) (muk2lr) 
replaced exception by precondition (mantis1420)
Revision 1.6 2006/11/16 14:41:07CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.5 2006/07/03 18:43:06CEST Muehlmann Karsten (AE-DA/ESA3) (muk2lr) 
allow inverse only for floating point types (mantis1110)
Revision 1.4 2006/01/26 12:41:37CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-added missing comma in operator-() (mantis:975)
Revision 1.3 2005/12/07 16:08:54CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-added scalar-mat functions as friend 
Revision 1.2 2005/11/30 09:59:50CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-more ops
Revision 1.1 2005/11/16 17:45:15CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/geo/geo.pj
********************************************************************/
