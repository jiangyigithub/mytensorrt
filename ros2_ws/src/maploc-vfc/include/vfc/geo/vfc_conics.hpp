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
*     $Source: vfc_conics.hpp $
*     $Revision: 1.8 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
*     $Date: 2007/07/23 09:40:34MESZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

#ifndef VFC_CONICS_HPP_INCLUDED
#define VFC_CONICS_HPP_INCLUDED

// vfc core includes
#include "vfc/core/vfc_math.hpp"
#include "vfc/core/vfc_trig.hpp"

namespace vfc
{    // namespace vfc opened

    //=========================================================================
    //  TConicSection<>
    //-------------------------------------------------------------------------
    //! represents a general conic section equitation: 
    //! Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0.
    //! @author zvh2hi
    //! @ingroup vfc_group_geo_conics
    //=========================================================================

    template <class T>    
    class TConicSection
    {
    public:
        typedef T value_type;
        TConicSection(  const T& A=0, const T& B=0, const T& C=0, 
                        const T& D=0, const T& E=0, const T& F=0)
        :    m_A(A), m_B(B), m_C(C), m_D(D), m_E(E), m_F(F) {}

        T    evaluate_xy (const T& x, const T& y) { return m_A*x*x + m_B*x*y + m_C*y*y + m_D*x + m_E*y + m_F;}

        void    set     (   const T& A, const T& B, const T& C, 
                            const T& D, const T& E, const T& F);

        void    set        (const T& val)    { set(val,val,val,val,val,val);} 

        const   T& A    (void)    const    {    return m_A;}
        const   T& B    (void)    const    {    return m_B;}
        const   T& C    (void)    const    {    return m_C;}
        const   T& D    (void)    const    {    return m_D;}
        const   T& E    (void)    const    {    return m_E;}
        const   T& F    (void)    const    {    return m_F;}
                
                T& A    (void)             {    return m_A;}
                T& B    (void)             {    return m_B;}
                T& C    (void)             {    return m_C;}
                T& D    (void)             {    return m_D;}
                T& E    (void)             {    return m_E;}
                T& F    (void)             {    return m_F;}
    private:
        T    m_A, m_B, m_C, m_D, m_E, m_F;
    };

    //=========================================================================
    //  TEllipse<> 
    //-------------------------------------------------------------------------
    //! represents an arbitrary rotated and translated ellipse.
    //! @code
    //! (X-C)t * Rt * D * R * (X-C) = 1
    //! C=ellipse center [Cx Cy]t
    //! R=rotation matrix [ cos(t) sin(t)]
    //!                   [-sin(t) cos(t)]
    //! D=major/minor axis [ 1/a^2    0   ]
    //!                    [   0    1/b^2 ]
    //! @endcode
    //! @author zvh2hi
    //! @ingroup vfc_group_geo_conics
    //=========================================================================
    template <class T>
    class TEllipse
    {
    public:
        typedef T value_type;

        //! c'tor, sets ellipse to unit circle as default
        TEllipse    (   const T& centerX    = static_cast<T>(0),    //!< ellipse center x coordinate 
                        const T& centerY    = static_cast<T>(0),    //!< ellipse center y coordinate
                        const T& a          = static_cast<T>(1),    //!< length of semi-major axis
                        const T& b          = static_cast<T>(1),    //!< length of semi-minor axis
                        const CRadian& theta_rad = CRadian()        //!< rotation angle in radians
                    );        
        
        void    set (   const T& centerX,        //!< ellipse center x coordinate 
                        const T& centerY,        //!< ellipse center y coordinate
                        const T& a,                //!< length of semi-major axis
                        const T& b,                //!< length of semi-minor axis
                        const CRadian& theta_rad//!< rotation angle in radians
                    );        
        
        void    setA        (const T& a);
        void    setB        (const T& b);
        void    setTheta    (const CRadian& theta_rad);
        void    setCenter   (const T& xc, T yc);

        const T&        getA        (void)    const    {    return m_a;}
        const T&        getB        (void)    const    {    return m_b;}
        const CRadian&  getTheta    (void)    const    {    return m_theta_rad;}
        const T&        getCenterX  (void)    const    {    return m_xc;}
        const T&        getCenterY  (void)    const    {    return m_yc;}
        
        //! evaluates ellipse equitation with p(x,y), returns 1, if p lies on ellipse perimeter
        T        evaluate_xy    (const T& x, const T& y);
        
        //! returns perimeter point p(x,y) from given phi
        void    evaluate_phi(const CRadian& phi, T&x, T& y);

    private:

        // original parameters
        T        m_xc, m_yc;
        T        m_a, m_b;
        CRadian    m_theta_rad;

        // derived (and cached) parameters
        T    m_oneOverA2, m_oneOverB2;
        T    m_ct, m_st;
    };

    //-------------------------------------------------------------------------
    //! convenience function, calculates an ellipse object from a conic section 
    //! object.
    //! @author zvh2hi
    //! @relatesalso TConicSection
    //-------------------------------------------------------------------------
    template <class T>  
    bool    calcEllipseFromGeneralConic (   const   TConicSection<T>&   f_conicSection, //!< [in] ConicSection Object
                                                    TEllipse<T>&        f_ellipse       //!< [out] Ellipse object
                                        );     

    //-------------------------------------------------------------------------
    //! convenience function, calculates an conic section object from geometric 
    //! ellipse object.
    //! @author zvh2hi
    //! @relatesalso TEllipse
    //-------------------------------------------------------------------------
    template <class T>
    void    calcGeneralConicFromEllipse (   const   TEllipse<T>&        f_ellipse,      //!< [in] Ellipse object
                                                    TConicSection<T>&   f_conicSection  //!< [out] ConicSection Object
                                        );    

}    // namespace vfc closed

#include "vfc/geo/vfc_conics.inl"

#endif //VFC_CONICS_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_conics.hpp  $
Revision 1.8 2007/07/23 09:40:34MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
- doxygen grouping (mantis1744)
Revision 1.7 2006/11/16 14:41:10CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.6 2006/02/14 09:32:14CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-added two convenience function calcEllipseFromGeneralConic() and calcGeneralConicFromEllipse() (mantis1009)
Revision 1.5 2006/02/13 15:52:40CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-fixed bug in TConicSection<>::evaluate_xy() (mantis1007)
Revision 1.4 2005/11/07 16:35:44CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-changed projectname to vfc/geo
Revision 1.3 2005/11/03 09:53:14CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-added set() methods
Revision 1.2 2005/11/02 14:22:10CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-renaming
********************************************************************/
