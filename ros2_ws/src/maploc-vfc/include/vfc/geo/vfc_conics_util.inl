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
///     $Source: vfc_conics_util.inl $
///     $Revision: 1.14 $
///     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
///     $Date: 2015/11/06 10:07:37MEZ $
///     $Locker:  $
///     $Name:  $
///     $State: in_work $
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
    
#include <math.h>

template <class T>    inline
bool vfc::calcEllipseFromGeneralConic
(    
    const T& A, const T& B, const T& C, const T& D, const T& E, const T& F,
    T& xc, T& yc, 
    T& a,
    T& b, 
    CRadian& theta
)
{
    static const T L_ONE  = static_cast<T>(1.0);
    static const T L_TWO  = static_cast<T>(2.0);
    static const T L_FOUR = static_cast<T>(4.0);

    const T D2 =  A*C - B*B/L_FOUR;

    if (notPositive(D2))
        return false;

    T det = A*(C*F - E*E/L_FOUR) - B*(B*F- D*E/L_TWO)/L_FOUR + D*(B*E/L_TWO-C*D)/L_FOUR;
    
    if (isZero(det))
        return false;

    // abs shall be used, as it is branchless. Better coverage, better optimisation.
	det = vfc::abs(det);

    xc = (E*B - L_TWO*C*D)/(L_FOUR*D2);
    yc = (D*B - L_TWO*A*E)/(L_FOUR*D2);

    const T trace = A + C;
    const T disc2 = trace*trace - L_FOUR*D2;
    
    const T disc = ::sqrt(disc2);

    if (trace<=disc)
        return false;

    const T cmaj = (trace+disc)*D2/(L_TWO*det);
    const T cmin = (trace-disc)*D2/(L_TWO*det);

    // cmin<=0 <=> trace<=disc
    // cmaj<=0 <=> trace<=-disc
    // As disc>0 cmin<=0 implies cmaj<=0 so '|| !isPositive(cmaj)' is useless

    a = L_ONE/::sqrt(cmin);  
    b = L_ONE/::sqrt(cmaj);

    //Find the angle that diagonalizes the upper 2x2 sub-matrix
    const T den = C - A;
    
    // and return the negative of this angle
    theta  = CRadian(-0.5 * ::atan2(B, den));

    return true;
}

template <class T>    inline
void vfc::calcGeneralConicFromEllipse
(
    const T& xc,const T& yc, 
    const T& a,    const T& b, 
    const CRadian& theta,
    T& A, T& B, T& C, T& D, T& E, T& F
)  
{
      const T a2 = a*a;
    const T b2 = b*b;

    const T ct = static_cast<T>(cos(-theta));
    const T st = static_cast<T>(sin(-theta));

    const T c2 = ct*ct;
    const T s2 = st*st;
    const T cs = ct*st;
        
    A = (a2*s2 + b2*c2);
    B = (2*(a2-b2)*cs);
    C = (a2*c2 + b2*s2);
    D = (-2*(a2*s2 + b2*c2)*xc - 2*(a2-b2)*cs*yc);
    E = (-2*(a2-b2)*cs*xc - 2*(a2*c2 + b2*s2)*yc);
    F = ((a2*s2 +b2*c2)*xc*xc + 2*(a2-b2)*cs*xc*yc + (a2*c2 + b2*s2)*yc*yc - a2*b2);
}

/*!    algorithm taken from a gamedev.net forum article 
    from dave eberly (author of Geometric Tools for CG):
    
    The mathematical construction is based on finding the two extreme 
    points of the ellipse given a direction vector D. 
    If Q(x,y) = 0 is the quadratic equation that implicitly defines 
    the ellipse, then gradient(Q)(x,y) = (dQ/dx, dQ/dy) is perpendicular 
    to the ellipse at the point (x,y). 
    You want to find the two ellipse points for which gradient(Q) is 
    parallel to D. Define perp(x,y) = (y,-x). The parallel condition is 
    equivalent to Dot(perp(D),gradient(Q)) = 0. 
    This is a linear equation in x and y. The condition Q = 0 is a 
    quadratic equation in x and y. 
    Combining the two yields a quadratic equation in one variable, 
    and from the geometry it must have two distinct real-valued 
    solutions. Solve for these, and then replace in Q = 0 to solve for 
    the other variables.
        
    In the case of D = (1,0) or (0,1) the solution 
    simplifies as follows ;)
*/
template <class T>    inline
void vfc::calcAABBFromEllipse
(    const T&        cx,     
    const T&         cy,       
       const T&         a,         
       const T&         b,        
       const CRadian&    theta_rad,
     T&                 minx,     
     T&                 miny,     
     T&                 maxx,     
     T&                 maxy    
)        
{
    const T c = static_cast<T>(cos(theta_rad));
    const T s = static_cast<T>(sin(theta_rad));
    const T c2 = c*c;
    const T s2 = s*s;

    const T a2 = a*a;
    const T b2 = b*b;

    const T C = c2 * a2 + s2 * b2;
    const T A = s2 * a2 + c2 * b2;

    // it's always safe to calculate the square root here, 
    // so we don't have to check

    const T rx = ::sqrt(C);
    const T ry = ::sqrt(A);

    minx = cx-rx;
    miny = cy-ry;
    maxx = cx+rx;
    maxy = cy+ry;
}

template <class T>    inline
bool vfc::calcAABBFromGeneralConicEllipse
(
    const T& A, const T& B, const T& C, const T& D, const T& E, const T& F,
     T&     minx, T&  miny, T& maxx, T& maxy    
)        
{
    T    xc, yc, a, b;
    CRadian theta;
    if (!calcEllipseFromGeneralConic(A,B,C,D,E,F,xc,yc,a,b,theta))
        return false;
    calcAABBFromEllipse(xc,yc,a,b,theta,minx,miny,maxx,maxy);
    return true;
}



//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_conics_util.inl  $
//  Revision 1.14 2015/11/06 10:07:37MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  -  proposal for vfc_conics_util improvements (mantis0004859)
//  Revision 1.13 2007/03/29 15:30:28MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced ::std with global namespace (mantis1534)
//  Revision 1.12 2006/11/16 14:41:07CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.11 2006/02/14 09:43:30CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -changed generic code for avoiding compiler warnings
//  Revision 1.10 2006/02/14 09:32:46CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added static_cast<>() for avoiding compiler warnings
//  Revision 1.9 2005/11/24 10:27:35CET Muehlmann Karsten (AE-DA/ESA3) * (MUK2LR) 
//  more checks if ellipse parameters are requested from non-ellipse conic
//  Revision 1.8 2005/11/07 16:35:44CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -changed projectname to vfc/geo
//  Revision 1.7 2005/11/04 16:58:05CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -changed cosf(), sinf(), to cos(), sin()
//  -calculate AABB from General Conic in two steps ... (need more time for a better solution)
//  Revision 1.6 2005/11/04 15:52:01CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -changed remaining sqrtf() to sqrf()
//  Revision 1.5 2005/11/02 19:14:23CET Muehlmann Karsten (AE-DA/ESA3) * (MUK2LR) 
//  non-float32 versions of sqrt and atan2
//  forgotten types added
//  Revision 1.4 2005/11/02 18:22:38CET Muehlmann Karsten (AE-DA/ESA3) * (MUK2LR) 
//  corrected typo
//  Revision 1.2 2005/11/02 15:00:45CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added ckecks for ellipse constraints
//=============================================================================
