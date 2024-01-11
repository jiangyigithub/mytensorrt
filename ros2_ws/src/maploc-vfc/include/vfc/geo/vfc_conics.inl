//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2006 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy, use or 
//  distribution is an offensive act against international law and may be 
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/geo
//          Synopsis: 
//  Target system(s): 
//       Compiler(s): VS7.1
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes: 
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: zvh2hi
//  Department: CR/AEM
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_conics.inl $
///     $Revision: 1.8 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (ZVH2HI) $
///     $Date: 2006/11/16 14:41:16MEZ $
///     $Locker:  $
///     $Name: 0032 RC1 Hello KW07  $
///     $State: In_Development $
///
/// @par Review Information:
/// - Reviewed version: 
/// - Type (use 'X' to mark):
///     - [ ] Formal Review
///     - [ ] Walkthrough
///     - [ ] Inspection
/// - State including date (DD.MM.YYYY)
///     - [--.--.----] Preparation
///     - [--.--.----] Review audit
///     - [--.--.----] Integration of findings
///     - [--.--.----] Test
///     - [--.--.----] Verification of integration of findings
///     - [--.--.----] Review release
/// - Responsible: 
/// - Review-Document: 
//=============================================================================


// vfc geo includes
#include "vfc/geo/vfc_conics_util.hpp"  // used for calcEllipseFromGeneralConic()

template <class T>
void    vfc::TConicSection<T>::set        
(    const T& A, const T& B, const T& C, 
    const T& D, const T& E, const T& F)
{
    m_A = A; 
    m_B = B; 
    m_C = C; 
    m_D = D; 
    m_E = E; 
    m_F = F;
}

template <class T>
vfc::TEllipse<T>::TEllipse 
(    const T& centerX, const T& centerY, 
    const T& a, const T& b, 
    const CRadian& theta_rad
)
:    m_xc(centerX), m_yc(centerY), 
    m_a(a), m_b(b), 
    m_theta_rad(theta_rad),
    m_oneOverA2(static_cast<T>(1)/(a*a)), 
    m_oneOverB2(static_cast<T>(1)/(b*b)),
    m_ct(static_cast<T>(cos(theta_rad))), 
    m_st(static_cast<T>(sin(theta_rad)))    
{}

template <class T>
void    vfc::TEllipse<T>::set    
(    const T& centerX,
    const T& centerY,
    const T& a,        
    const T& b,        
    const CRadian& theta_rad
)
{
    setCenter(centerX,centerY);
    setA(a);
    setB(b);
    setTheta(theta_rad);
}

template <class T>
void    vfc::TEllipse<T>::setA    (const T& a)                 
{ 
    m_a = a; 
    m_oneOverA2 = static_cast<T>(1)/(a*a);
}

template <class T>
void    vfc::TEllipse<T>::setB    (const T& b)                 
{    
    m_b = b; 
    m_oneOverB2 = static_cast<T>(1)/(b*b);
}

template <class T>
void    vfc::TEllipse<T>::setTheta    (const CRadian& theta_rad)    
{ 
    m_theta_rad = theta_rad; 
    m_ct = static_cast<T>(cos(theta_rad)), 
    m_st = static_cast<T>(sin(theta_rad));
}

template <class T>
void    vfc::TEllipse<T>::setCenter     (const T& xc, T yc)         
{ 
    m_xc = xc; 
    m_yc = yc;
}

//! evaluates ellipse equitation with p(x,y), returns 1, if p lies on ellipse perimeter
template <class T>
T    vfc::TEllipse<T>::evaluate_xy    (const T& x, const T& y)
{
    // transform point into ellipse coordinate-system (translate & rotate)
    const     T    xs =  m_ct*(x-m_xc) + m_st*(y-m_yc);
    const    T    ys = -m_st*(x-m_xc) + m_ct*(y-m_yc);
    
    // evaluate normal ellipse equitation
    return     xs*xs*m_oneOverA2 + ys*ys*m_oneOverB2;
}

//! returns perimeter point p(x,y) from given phi
template <class T>
void    vfc::TEllipse<T>::evaluate_phi    (const CRadian& phi, T&x, T& y)
{
    const T xe = m_a * static_cast<T>(cos(phi));
    const T ye = m_b * static_cast<T>(sin(phi));
    
    x = xe * m_ct - ye * m_st + m_xc;
    y = xe * m_st + ye * m_ct + m_yc;
}

template <class T>  inline
bool    vfc::calcEllipseFromGeneralConic(const TConicSection<T>& f_conicSection, TEllipse<T>& f_ellipse)
{
    T       xc, yc, a, b; 
    CRadian theta;

    if ( true == calcEllipseFromGeneralConic  (     f_conicSection.A(), f_conicSection.B(), f_conicSection.C(),
                                                    f_conicSection.D(), f_conicSection.E(), f_conicSection.F(),
                                                    xc, yc, a, b, theta)    )
    {
        f_ellipse.setCenter(xc, yc);
        f_ellipse.setA(a);
        f_ellipse.setB(b);
        f_ellipse.setTheta(theta);
        return true;
    }
    else
    {
        return false;
    }
}

template <class T>  inline
void    vfc::calcGeneralConicFromEllipse(const TEllipse<T>& f_ellipse, TConicSection<T>&   f_conicSection)
{
    calcGeneralConicFromEllipse(    f_ellipse.getCenterX(), f_ellipse.getCenterY(),
                                    f_ellipse.getA(), f_ellipse.getB(),
                                    f_ellipse.getTheta(),
                                    f_conicSection.A(), f_conicSection.B(), f_conicSection.C(),
                                    f_conicSection.D(), f_conicSection.E(), f_conicSection.F()
                                );


}



//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_conics.inl  $
//  Revision 1.8 2006/11/16 14:41:16MEZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.7 2006/11/10 15:40:10CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - reordered initialization list (mantis1293)
//  - replaced header/footer templates
//  Revision 1.6 2006/02/14 09:42:54CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -fixed syntax error in calcEllipseFromGeneralConic()
//  Revision 1.5 2006/02/14 09:32:14CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added two convenience function calcEllipseFromGeneralConic() and calcGeneralConicFromEllipse() (mantis1009)
//  Revision 1.4 2005/11/07 16:35:45CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -changed projectname to vfc/geo
//  Revision 1.3 2005/11/03 09:53:39CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added set() methods
//  Revision 1.2 2005/11/02 14:21:47CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -renaming
//=============================================================================
