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
*     $Source: vfc_matvec3_ops.inl $
*     $Revision: 1.4 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (ZVH2HI) $
*     $Date: 2006/11/16 14:41:11MEZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

template <class T>    inline
vfc::TMatrix3<T>    vfc::dyad        (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3)
{
    return TMatrix3<T>    (    f_a_vec3.x()*f_b_vec3.x(), f_a_vec3.x()*f_b_vec3.y(), f_a_vec3.x()*f_b_vec3.z(), 
                            f_a_vec3.y()*f_b_vec3.x(), f_a_vec3.y()*f_b_vec3.y(), f_a_vec3.y()*f_b_vec3.z(), 
                            f_a_vec3.z()*f_b_vec3.x(), f_a_vec3.z()*f_b_vec3.y(), f_a_vec3.z()*f_b_vec3.z()
                        );

}


template <class T>    inline  
vfc::TVector3<T>    vfc::operator*    (const TMatrix3<T>& f_mat3, const TVector3<T>& f_vec3)
{
    return TVector3<T> (    f_mat3(0,0)*f_vec3.x() + f_mat3(0,1)*f_vec3.y() + f_mat3(0,2)*f_vec3.z(), 
                            f_mat3(1,0)*f_vec3.x() + f_mat3(1,1)*f_vec3.y() + f_mat3(1,2)*f_vec3.z(), 
                            f_mat3(2,0)*f_vec3.x() + f_mat3(2,1)*f_vec3.y() + f_mat3(2,2)*f_vec3.z()
                        );
}

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_matvec3_ops.inl  $
Revision 1.4 2006/11/16 14:41:11MEZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.3 2006/08/08 16:07:31CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
-undone last revision (1.2) changes, functionality is already in vfc_mat3_ops implemented
Revision 1.2 2006/07/31 15:21:51CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (SF71LR) 
Added missing function setAVector getAVector (taken from BTC_RotMatrix)
Revision 1.1 2005/12/07 16:13:19CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/geo/geo.pj
********************************************************************/
