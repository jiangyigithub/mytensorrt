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
*     $Source: vfc_vec3_ops.hpp $
*     $Revision: 1.11 $
*     $Author: gaj2kor $
*     $Date: 2008/09/11 14:25:49MESZ $
*     $Locker:  $
*     $Name: 0032 RC1  $
*     $State: in_work $
*/
/*******************************************************************/

#ifndef VFC_VEC3_OPS_HPP_INCLUDED
#define VFC_VEC3_OPS_HPP_INCLUDED

#include "vfc/core/vfc_trig.hpp"
#include "vfc/geo/vfc_vec3.hpp"

namespace vfc
{    // namespace vfc opened

    ///////////////////////////////////////////////////////////////////////////
    // elementwise ops
    ///////////////////////////////////////////////////////////////////////////

    //-------------------------------------------------------------------------
    //! returns componentwise abs of specified vector.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    TVector3<T>        abs    (const TVector3<T>& f_vec3);

    //-------------------------------------------------------------------------
    //! returns componentwise min of specified vector.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    TVector3<T>        min    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3);

    //-------------------------------------------------------------------------
    //! returns componentwise max of specified vector.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    TVector3<T>        max    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3);

    //-------------------------------------------------------------------------
    //! returns sum of vector components.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    T    sum    (const TVector3<T>& f_vec3);

    ///////////////////////////////////////////////////////////////////////////
    // cross, dot
    ///////////////////////////////////////////////////////////////////////////

    //-------------------------------------------------------------------------
    //! returns dot product of specified vectors.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    T    dot    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3);

    //-------------------------------------------------------------------------
    //! returns cross product of specified vectors.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    TVector3<T>        cross    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3);

    ///////////////////////////////////////////////////////////////////////////
    // vector norm
    ///////////////////////////////////////////////////////////////////////////

    //-------------------------------------------------------------------------
    //! returns L1 norm of specified vector.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    T    norm_L1        (const TVector3<T>& f_vec3);

    //-------------------------------------------------------------------------
    //! returns L2 norm of specified vector.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    T    norm_L2        (const TVector3<T>& f_vec3);

    //-------------------------------------------------------------------------
    //! returns Linf norm of specified vector.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    T    norm_Linf    (const TVector3<T>& f_vec3);

    //-------------------------------------------------------------------------
    //! returns length of specified vector.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    T    length        (const TVector3<T>& f_vec3);

    //-------------------------------------------------------------------------
    //! returns specified vector normalized.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    TVector3<T>        normalized    (const TVector3<T>& f_vec3);

    //-------------------------------------------------------------------------
    //! normalizes specified vector, returns old vector length.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    T    normalize    (TVector3<T>& f_vec3);

    ///////////////////////////////////////////////////////////////////////////
    // derived values: distance, angle, area, volume
    ///////////////////////////////////////////////////////////////////////////

    //-------------------------------------------------------------------------
    //! returns angle between specified vectors in radians.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    CRadian        angle    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3);

    //-------------------------------------------------------------------------
    //! returns area spanned by specified vectors.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    T    area    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3);

    //-------------------------------------------------------------------------
    //! returns volume spanned by specified vectors.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    T    volume    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3, const TVector3<T>& f_c_vec3);

    ///////////////////////////////////////////////////////////////////////////
    // overloaded ops
    ///////////////////////////////////////////////////////////////////////////

    //-------------------------------------------------------------------------
    //! unary minus operator.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    TVector3<T>    operator-    (const TVector3<T>& f_vec3);

    //-------------------------------------------------------------------------
    //! binary minus operator.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    TVector3<T>    operator-    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3);

    //-------------------------------------------------------------------------
    //! binary plus operator.
    //! @author zvh2hi
    //! @relatesalso TVector3
    //-------------------------------------------------------------------------

    template <class T>
    TVector3<T>    operator+    (const TVector3<T>& f_a_vec3, const TVector3<T>& f_b_vec3);

}    // namespace vfc closed

#include "vfc/geo/vfc_vec3_ops.inl"

#endif //VFC_VEC3_OPS_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_vec3_ops.hpp  $
Revision 1.11 2008/09/11 14:25:49MESZ gaj2kor 
-Resolution of QAC++ warning related to Rule 9.0.3
(Mantis : 2221)
Revision 1.10 2008/07/11 14:22:28IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
- Removed paranthesis added by mistake (mantis2168)
Revision 1.9 2008/07/10 17:37:17IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
- fixed min/max errors due to macro definitions of min & max (mantis2168)
Revision 1.8 2007/07/23 13:19:41IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
- doxygen grouping (mantis1744)
Revision 1.7 2006/11/16 14:41:06CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
- replaced tabs with 4 spaces (mantis1294)
Revision 1.6 2005/12/19 15:38:55CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-normalize() returns old length again
Revision 1.5 2005/12/19 15:21:15CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-renamed as_normalized() to normalized()
Revision 1.4 2005/12/07 16:11:45CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-restructuring
Revision 1.3 2005/11/30 09:59:46CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-more ops
Revision 1.2 2005/11/07 18:21:01CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-changed project name to vfc/geo
********************************************************************/
