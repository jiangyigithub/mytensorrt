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
*     $Source: vfc_matvec4_ops.hpp $
*     $Revision: 1.3 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
*     $Date: 2007/07/23 09:46:11MESZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

#ifndef VFC_MATVEC4_OPS_HPP_INCLUDED
#define VFC_MATVEC4_OPS_HPP_INCLUDED

#include "vfc/geo/vfc_vec3.hpp"
#include "vfc/geo/vfc_vec4.hpp"
#include "vfc/geo/vfc_mat4.hpp"

namespace vfc
{    // namespace vfc opened

    //-----------------------------------------------------------------------------
    /// calculates the dyadic product of specified 4-dim vectors.
    /// @author zvh2hi
    /// @relatesalso TVector4
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix4<T>        dyad        (const TVector4<T>& f_a_vec4, const TVector4<T>& f_b_vec4);

    //-----------------------------------------------------------------------------
    /// returns matrix*vector product.
    /// @author zvh2hi
    /// @relatesalso TVector4
    //-----------------------------------------------------------------------------

    template <class T>
    TVector4<T>        operator*    (const TMatrix4<T>& f_mat4, const TVector4<T>& f_vec4);

    //-----------------------------------------------------------------------------
    /// returns matrix*vector product, augments 3-dim vector with homogeneous 
    /// component w=1.
    /// @author zvh2hi
    /// @relatesalso TVector3
    //-----------------------------------------------------------------------------

    template <class T>
    TVector3<T>        operator*    (const TMatrix4<T>& f_mat4, const TVector3<T>& f_vec3);


}    // namespace vfc closed

#include "vfc/geo/vfc_matvec4_ops.inl"

#endif //VFC_MATVEC4_OPS_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_matvec4_ops.hpp  $
Revision 1.3 2007/07/23 09:46:11MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
- doxygen grouping (mantis1744)
Revision 1.2 2006/11/16 14:41:13CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
********************************************************************/
