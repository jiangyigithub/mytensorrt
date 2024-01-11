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
*     $Source: vfc_matvec2_ops.inl $
*     $Revision: 1.2 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (ZVH2HI) $
*     $Date: 2006/11/16 14:41:05MEZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

template <class T>    inline
vfc::TMatrix2<T>    vfc::dyad        (const TVector2<T>& f_a_vec2, const TVector2<T>& f_b_vec2)
{
    return TMatrix2<T>    (    f_a_vec2.x()*f_b_vec2.x(), f_a_vec2.x()*f_b_vec2.y(), 
                            f_a_vec2.y()*f_b_vec2.x(), f_a_vec2.y()*f_b_vec2.y()
                        );

}


template <class T>    inline  
vfc::TVector2<T>    vfc::operator*    (const TMatrix2<T>& f_mat2, const TVector2<T>& f_vec2)
{
    return TVector2<T> (    f_mat2(0,0)*f_vec2.x() + f_mat2(0,1)*f_vec2.y(),
                            f_mat2(1,0)*f_vec2.x() + f_mat2(1,1)*f_vec2.y()
                        );
}

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_matvec2_ops.inl  $
Revision 1.2 2006/11/16 14:41:05MEZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
********************************************************************/
