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
*     $Source: vfc_matvec3_ops.hpp $
*     $Revision: 1.5 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
*     $Date: 2007/07/23 09:45:51MESZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

#ifndef VFC_MATVEC3_OPS_HPP_INCLUDED
#define VFC_MATVEC3_OPS_HPP_INCLUDED

#include "vfc/geo/vfc_vec3.hpp"
#include "vfc/geo/vfc_mat3.hpp"

namespace vfc
{    // namespace vfc opened

    //-----------------------------------------------------------------------------
    /// calculates the dyadic product of specified 3-dim vectors.
    /// @author zvh2hi
    /// @relatesalso TVector3
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix3<T>        dyad        (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3);

    //-----------------------------------------------------------------------------
    /// returns matrix*vector product.
    /// @author zvh2hi
    /// @relatesalso TVector3
    //-----------------------------------------------------------------------------

    template <class T>
    TVector3<T>        operator*    (const TMatrix3<T>& f_mat3, const TVector3<T>& f_vec3);

}    // namespace vfc closed

#include "vfc/geo/vfc_matvec3_ops.inl"

#endif //VFC_MATVEC3_OPS_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_matvec3_ops.hpp  $
Revision 1.5 2007/07/23 09:45:51MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
- doxygen grouping (mantis1744)
Revision 1.4 2006/11/16 14:41:07CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.3 2006/08/08 16:07:31CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
-undone last revision (1.2) changes, functionality is already in vfc_mat3_ops implemented
Revision 1.2 2006/07/31 15:21:50CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (SF71LR) 
Added missing function setAVector getAVector (taken from BTC_RotMatrix)
Revision 1.1 2005/12/07 16:13:19CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/geo/geo.pj
********************************************************************/
