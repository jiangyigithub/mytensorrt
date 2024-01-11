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
*     $Source: vfc_conics_util.hpp $
*     $Revision: 1.8 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
*     $Date: 2007/07/23 09:39:22MESZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

#ifndef VFC_CONICS_UTIL_HPP_INCLUDED
#define VFC_CONICS_UTIL_HPP_INCLUDED


#include "vfc/core/vfc_trig.hpp"

namespace vfc
{    // namespace vfc opened

    //-------------------------------------------------------------------------
    //! calculates a geometric description of an ellipse from conic section 
    //! parameters.
    //! general equitation of a conic section is: Ax2+Bxy+Cy2+Dx+Ey+F=0
    //! @return false, if given conic does not fulfill ellipse constraints 
    //! @author zvh2hi
    //! @ingroup vfc_group_geo_conics
    //-------------------------------------------------------------------------
    template <class T>
    bool calcEllipseFromGeneralConic
    (    
        // [in] 6 conic section coeffs 
        const T& A, const T& B, const T& C, const T& D, const T& E, const T& F,
        
        T& xc,          //!< [out] ellipse center x
        T& yc,          //!< [out] ellipse center y
        T& a,           //!< [out] ellipse semi-major axis
        T& b,           //!< [out] ellipse semi-minor axis
        CRadian& theta  //!< [out] ellipse rotation angle [rad]
    );
  
    //-------------------------------------------------------------------------
    //! calculates general conic section coeffs from geometric ellipse 
    //! description.
    //! @author zvh2hi
    //! @ingroup vfc_group_geo_conics
    //-------------------------------------------------------------------------

    template <class T>
    void calcGeneralConicFromEllipse
    (
        const T& xc,            //!< [in] ellipse center x
        const T& yc,            //!< [in] ellipse center y
        const T& a,             //!< [in] ellipse semi-major axis
        const T& b,             //!< [in] ellipse semi-minor axis
        const CRadian& theta,   //!< [in] ellipse rotation angle [rad]
        
        // [out] 6 conic section coeffs 
           T& A, T& B, T& C, T& D, T& E, T& F
    );
      
    //-------------------------------------------------------------------------
    //! calculates axis-aligned bounding box (AABB) from arbitrary rotated 
    //! ellipse with conic section coeff E(A,B,C,D,E,F).
    //! general equitation of a conic section is: Ax2+Bxy+Cy2+Dx+Ey+F=0
    //! @return false, if given conic does not fulfill ellipse constraints
    //! @author zvh2hi
    //! @ingroup vfc_group_geo_conics
    //-------------------------------------------------------------------------

    template <class T>
    bool calcAABBFromGeneralConicEllipse
    (
        // [in] 6 conic section coeffs 
        const T& A, const T& B, const T& C, const T& D, const T& E, const T& F,

         T&     minx,   //!< [out] minx of aabb
         T&     miny,   //!< [out] miny of aabb
         T&     maxx,   //!< [out] maxx of aabb
         T&     maxy    //!< [out] maxx of aabb
     );
     
    //-------------------------------------------------------------------------
    //! calculates axis-aligned bounding box (AABB) from arbitrary rotated 
    //! ellipse E(cx,cy,a,b,phi).
    //! @author zvh2hi
    //! @ingroup vfc_group_geo_conics
    //-------------------------------------------------------------------------

    template <class T>
    void calcAABBFromEllipse
    (    
        const T&        cx,         //!< [in] ellipse center x
        const T&        cy,         //!< [in] ellipse center y
        const T&        a,          //!< [in] semi-major axis
        const T&        b,          //!< [in] semi-minor axis
        const CRadian&  theta_rad,  //!< [in] rotation angle [rad]
         
        T&                 minx,    //!< [out] minx of aabb
        T&                 miny,    //!< [out] miny of aabb
        T&                 maxx,    //!< [out] maxx of aabb
        T&                 maxy     //!< [out] maxx of aabb
     );


}    // namespace vfc closed

#include "vfc/geo/vfc_conics_util.inl"

#endif //VFC_CONICS_UTIL_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_conics_util.hpp  $
Revision 1.8 2007/07/23 09:39:22MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
- doxygen grouping (mantis1744)
- added documentation
Revision 1.7 2006/11/16 14:41:20CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.6 2005/11/10 17:51:58CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-changed docu
Revision 1.5 2005/11/07 16:35:43CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-changed projectname to vfc/geo
Revision 1.4 2005/11/03 09:52:51CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-added documentation
Revision 1.3 2005/11/02 15:13:17CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-fixed documentation and interface
Revision 1.2 2005/11/02 14:59:40CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-fixed interface inconsistency
********************************************************************/
