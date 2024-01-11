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
*     $Source: vfc_mat2_ops.hpp $
*     $Revision: 1.5 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
*     $Date: 2007/07/23 09:41:04MESZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

#ifndef VFC_MAT2_OPS_HPP_INCLUDED
#define VFC_MAT2_OPS_HPP_INCLUDED

#include "vfc/geo/vfc_vec2.hpp"
#include "vfc/geo/vfc_mat2.hpp"

namespace vfc
{    // namespace vfc opened

    //-----------------------------------------------------------------------------
    //! returns the determinant of specified matrix.
    //! @author zvh2hi
    //! @relatesalso TMatrix2
    //-----------------------------------------------------------------------------

    template <class T>
    T                det            (const TMatrix2<T>& mat);

    //-----------------------------------------------------------------------------
    //! returns trace of specified matrix.
    //! @author zvh2hi
    //! @relatesalso TMatrix2
    //-----------------------------------------------------------------------------

    template <class T>
    T                trace        (const TMatrix2<T>& mat);

    //-----------------------------------------------------------------------------
    //! returns inverse of specified matrix.
    //! @author zvh2hi
    //! @relatesalso TMatrix2
    //-----------------------------------------------------------------------------
    
    template <class T>
    TMatrix2<T>        inverse        (const TMatrix2<T>& mat);

    //-----------------------------------------------------------------------------
    //! returns specified matrix transposed.
    //! @author zvh2hi
    //! @relatesalso TMatrix2
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix2<T>        transpose    (const TMatrix2<T>& mat);

    //-----------------------------------------------------------------------------
    //! sets specified matrix to identity.
    //! @author zvh2hi
    //! @relatesalso TMatrix2
    //-----------------------------------------------------------------------------

    template <class T>
    void            identity    (TMatrix2<T>& mat);

    //-----------------------------------------------------------------------------
    //! unary minus operator.
    //! @author zvh2hi
    //! @relatesalso TMatrix2
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix2<T>        operator-    (const TMatrix2<T>& mat);

    //-----------------------------------------------------------------------------
    //! returns matrix*matrix product.
    //! @author zvh2hi
    //! @relatesalso TMatrix2
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix2<T>        operator*    (const TMatrix2<T>& op1, const TMatrix2<T>& op2);

    //-----------------------------------------------------------------------------
    //! binary plus operator.
    //! @author zvh2hi
    //! @relatesalso TMatrix2
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix2<T>        operator+    (const TMatrix2<T>& op1, const TMatrix2<T>& op2);

    //-----------------------------------------------------------------------------
    //! binary minus operator.
    //! @author zvh2hi
    //! @relatesalso TMatrix2
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix2<T>        operator-    (const TMatrix2<T>& op1, const TMatrix2<T>& op2);

}    // namespace vfc closed

#include "vfc/geo/vfc_mat2_ops.inl"

#endif //VFC_MAT2_OPS_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_mat2_ops.hpp  $
Revision 1.5 2007/07/23 09:41:04MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
- doxygen grouping
Revision 1.4 2006/11/16 14:41:18CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.3 2005/12/07 16:08:51CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-added scalar-mat functions as friend 
Revision 1.2 2005/11/30 09:59:49CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-more ops
********************************************************************/
