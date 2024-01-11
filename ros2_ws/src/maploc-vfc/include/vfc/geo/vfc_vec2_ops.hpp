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
*     $Source: vfc_vec2_ops.hpp $
*     $Revision: 1.13 $
*     $Author: gaj2kor $
*     $Date: 2009/01/31 08:39:59MEZ $
*     $Locker:  $
*     $Name: 0032 RC1  $
*     $State: in_work $
*/
/*******************************************************************/

#ifndef VFC_VEC2_OPS_HPP_INCLUDED
#define VFC_VEC2_OPS_HPP_INCLUDED

#include "vfc/core/vfc_trig.hpp"
#include "vfc/geo/vfc_vec2.hpp"         //using TVector2

namespace vfc
{    // namespace vfc opened
    ///////////////////////////////////////////////////////////////////////////
    // elementwise ops
    ///////////////////////////////////////////////////////////////////////////

    //-------------------------------------------------------------------------
    //! returns componentwise abs of specified vector.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    TVector2<T>        abs    (const TVector2<T>& vec);

    //-------------------------------------------------------------------------
    //! returns componentwise min of specified vector.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    TVector2<T>        min    (const TVector2<T>& op1, const TVector2<T>& op2);

    //-------------------------------------------------------------------------
    //! returns componentwise max of specified vector.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    TVector2<T>        max    (const TVector2<T>& op1, const TVector2<T>& op2);

    //-------------------------------------------------------------------------
    //! returns sum of vector components.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    T    sum    (const TVector2<T>& vec);

    ///////////////////////////////////////////////////////////////////////////
    // cross, dot
    ///////////////////////////////////////////////////////////////////////////

    //-------------------------------------------------------------------------
    //! returns dot product of specified vectors.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    T    dot    (const TVector2<T>& op1, const TVector2<T>& op2);

    //-------------------------------------------------------------------------
    //! returns cross product of specified vectors.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    T    cross    (const TVector2<T>& op1, const TVector2<T>& op2);

    ///////////////////////////////////////////////////////////////////////////
    // vector norm
    ///////////////////////////////////////////////////////////////////////////

    //-------------------------------------------------------------------------
    //! returns L1 norm of specified vector.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    T    norm_L1        (const TVector2<T>& vec);

    //-------------------------------------------------------------------------
    //! returns L2 norm of specified vector.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    T    norm_L2        (const TVector2<T>& vec);

    //-------------------------------------------------------------------------
    //! returns Linf norm of specified vector.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    T    norm_Linf    (const TVector2<T>& vec);

    //-------------------------------------------------------------------------
    //! returns length of specified vector.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    T    length        (const TVector2<T>& vec);

    //-------------------------------------------------------------------------
    //! returns specified vector normalized.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    TVector2<T>        normalized    (const TVector2<T>& vec);

    //-------------------------------------------------------------------------
    //! normalizes specified vector, returns old vector length.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    T    normalize    (TVector2<T>& vec);

    ///////////////////////////////////////////////////////////////////////////
    // derived values: distance, angle, area, volume
    ///////////////////////////////////////////////////////////////////////////

    //-------------------------------------------------------------------------
    //! returns angle between specified vectors in radians.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    CRadian        angle    (const TVector2<T>& f_op1, const TVector2<T>& f_op2);

    //-------------------------------------------------------------------------
    //! returns area spanned by specified vectors.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    T    area    (const TVector2<T>& op1, const TVector2<T>& op2);

    ///////////////////////////////////////////////////////////////////////////
    // overloaded ops
    ///////////////////////////////////////////////////////////////////////////

    //-------------------------------------------------------------------------
    //! unary minus operator.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    TVector2<T>    operator-    (const TVector2<T>& vec);

    //-------------------------------------------------------------------------
    //! binary minus operator.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    TVector2<T>    operator-    (const TVector2<T>& op1, const TVector2<T>& op2);

    //-------------------------------------------------------------------------
    //! binary plus operator.
    //! @author zvh2hi
    //! @relatesalso TVector2
    //-------------------------------------------------------------------------

    template <class T>
    TVector2<T>    operator+    (const TVector2<T>& op1, const TVector2<T>& op2);


}    // namespace vfc closed

#include "vfc/geo/vfc_vec2_ops.inl"

#endif //VFC_VEC2_OPS_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_vec2_ops.hpp  $
Revision 1.13 2009/01/31 08:39:59MEZ gaj2kor 
- Inclusion of missing header file.
(Mantis : 0002509)
Revision 1.12 2008/10/07 17:32:20IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
- rewrote vfc::angle() function (mantis 2369)
Revision 1.11 2008/07/11 10:52:29CEST Dilip Krishna (RBEI/EAE6) (dkn2kor)
- Removed paranthesis added by mistake (mantis2168)
Revision 1.10 2008/07/10 17:37:19IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
- fixed min/max errors due to macro definitions of min & max (mantis2168)
Revision 1.9 2007/07/23 13:18:09IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
- doxygen grouping (mantis1744)
Revision 1.8 2006/11/16 14:41:13CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
- replaced tabs with 4 spaces (mantis1294)
Revision 1.7 2006/03/02 13:31:58CET Dilipkrishna Natesan (RBIN/EAE3 AE-DA/ESA3) * (DIN1LR)
-Added vfc_trig.hpp include file (mantis1014)
Revision 1.6 2005/12/19 15:38:56CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-normalize() returns old length again
Revision 1.5 2005/12/19 15:21:16CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-renamed as_normalized() to normalized()
Revision 1.4 2005/12/07 16:11:47CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-restructuring
Revision 1.3 2005/11/30 09:59:49CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-more ops
Revision 1.2 2005/11/08 12:19:41CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-update
********************************************************************/
