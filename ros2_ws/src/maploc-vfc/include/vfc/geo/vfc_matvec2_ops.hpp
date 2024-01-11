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
*     $Source: vfc_matvec2_ops.hpp $
*     $Revision: 1.4 $
*     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
*     $Date: 2011/01/21 12:53:33MEZ $
*     $Locker:  $
*     $Name:  $
*     $State: in_work $
*/
/*******************************************************************/

#ifndef VFC_MATVEC2_OPS_HPP_INCLUDED
#define VFC_MATVEC2_OPS_HPP_INCLUDED

#include "vfc/geo/vfc_vec2.hpp"
#include "vfc/geo/vfc_mat2.hpp"

namespace vfc
{    // namespace vfc opened

    //-----------------------------------------------------------------------------
    /// calculates the dyadic product of specified 2-dim vectors.
    /// @author zvh2hi
    /// @relatesalso TVector2
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix2<T>        dyad        (const TVector2<T>& f_a_vec2, const TVector2<T>& f_b_vec2);

    //-----------------------------------------------------------------------------
    /// returns matrix*vector product.
    /// @author zvh2hi
    /// @relatesalso TVector2
    //-----------------------------------------------------------------------------

    template <class T>
    TVector2<T>        operator*    (const TMatrix2<T>& f_mat2, const TVector2<T>& f_vec2);

}    // namespace vfc closed

#include "vfc/geo/vfc_matvec2_ops.inl"

#endif //VFC_MATVEC2_OPS_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_matvec2_ops.hpp  $
Revision 1.4 2011/01/21 12:53:33MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
- sometimes .hpp or namespace qualifiers are missing (mantis3599)
Revision 1.3 2007/07/23 09:45:11MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
- doxygen grouping (mantis1744)
Revision 1.2 2006/11/16 14:41:16CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
********************************************************************/
