//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2006 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorised copy, use or 
//  distribution is an offensive act against international law and may be 
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/geo
//          Synopsis: Quaternion implementation
//  Target system(s): 
//       Compiler(s): VC_71
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
///     $Source: vfc_quat_ops.hpp $
///     $Revision: 1.4 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/07/23 09:46:51MESZ $
///     $Locker:  $
///     $Name: 0032 RC1 Hello KW07  $
///     $State: In_Development $
///
/// @par Review Information:
/// - Reviewed version: 
/// - Type (use "X" to mark):
///        - [ ] Formal Review
///        - [ ] Walkthrough
///        - [ ] Inspection
/// - State including date (DD.MM.YYYY)
///        - [--.--.----] Preparation
///        - [--.--.----] Review audit 
///        - [--.--.----] Integration of findings
///        - [--.--.----] Test
///        - [--.--.----] Verification of integration of findings
///        - [--.--.----] Review release
///    - Responsible:
///    - Review-Document:        
//=============================================================================

#ifndef VFC_QUAT_OPS_HPP_INCLUDED
#define VFC_QUAT_OPS_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_trig.hpp"

#include "vfc/geo/vfc_quat.hpp"

namespace vfc
{    // namespace vfc opened

    //-----------------------------------------------------------------------------
    /// returns the dot product of two quaternions. @relatesalso TQuaternion
    /// @return value of dot product
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------
    
    template <class T>
    T   dot (const TQuaternion<T>& f_a_quat, const TQuaternion<T>& f_b_quat);

    //-----------------------------------------------------------------------------
    /// returns the conjugate (q*) of given quaternion.
    /// @return conjugate of specified quaternion
    /// @par Algorithm:
    /// @f[\hat{q}^*=\begin{pmatrix}-qx\\-qy\\-qz\\qw\end{pmatrix}@f]
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------

    template <class T>
    TQuaternion<T>        conjugate    (const TQuaternion<T>& f_quat); 

    //-----------------------------------------------------------------------------
    /// returns the inverse (q-1) of given unit-quaternion. 
    /// @return inverse of specified quaternion
    /// @pre    specified quaternion has to be unit length
    /// @par    Algorithm:
    /// @f[\hat{q}^{-1}=\hat{q}^*=\begin{pmatrix}-qx\\-qy\\-qz\\qw\end{pmatrix}\qquad ;\left \Vert \hat{q} \right \Vert = 1@f]
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------

    template <class T>
    TQuaternion<T>        inverse        (const TQuaternion<T>& f_quat);

    //-----------------------------------------------------------------------------
    /// sets specified quaternion to a identity quaternion q(0,0,0,1).
    /// @return void
    /// @par    Algorithm:
    /// @f[\hat{q}_I=\begin{pmatrix}0\\0\\0\\1\end{pmatrix}@f]
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------

    template <class T>
    void            identity    (TQuaternion<T>& f_quat);

    //-----------------------------------------------------------------------------
    /// returns the length (L2 norm) of specified quaternion. 
    /// @return L2 norm of specified quaternion 
    /// @par    Algorithm:
    /// @f[l=\sqrt{q \bullet  q}@f]
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------

    template <class T>
    T    length        (const TQuaternion<T>& f_quat);
 

    //-----------------------------------------------------------------------------
    /// returns a normalized copy of specified quaternion. 
    /// @return the normalized quaternion
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------

    template <class T>
    TQuaternion<T>        normalized    (const TQuaternion<T>& f_quat);

    //-----------------------------------------------------------------------------
    /// normalizes the specified quaternion and returns the length before normalization. 
    /// @return the length of the specified quaternion before normalization
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------

    template <class T>
    T    normalize    (TQuaternion<T>& f_quat);


    //-----------------------------------------------------------------------------
    /// sets quaternion to specified axis-angle rotation, the axis vector doesn't 
    /// have to be normalized. 
    /// @pre the value type of the specified quaternion has to be a floating point type
    /// @par Algorithm:
    /// @f[\hat{q}=\begin{pmatrix}x\ \sin (\frac{\phi}{2})\\y\ \sin (\frac{\phi}{2})\\z\ \sin (\frac{\phi}{2})\\\cos (\frac{\phi}{2})\end{pmatrix}@f]
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------

    template <class T>
    void    fromAxisAngleRotation    (TQuaternion<T>& f_quat, const T& f_xComp, const T& f_yComp, const T& f_zComp, const CRadian& f_phi);


    //-----------------------------------------------------------------------------
    /// sets quaternion to specified axis-angle rotation, the axis can be of any 
    /// type if it offers a subscript operator []. 
    /// @pre the value type of the specified quaternion has to be a floating point type
    /// @sa fromAxisAngleRotation()
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------

    template <class T, class AxisType>
    void    fromAxisAngleRotation    (TQuaternion<T>& f_quat, const AxisType& f_axis, const CRadian& f_phi);

    //-----------------------------------------------------------------------------
    /// sets quaternion to a rotation around x-axis with specified angle. 
    /// @pre the value type of the specified quaternion has to be a floating point type
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------

    template <class T>
    void    fromXAxisRotation       (TQuaternion<T>& f_quat, const CRadian& f_angleX);


    //-----------------------------------------------------------------------------
    /// sets quaternion to a rotation around y-axis with specified angle.
    /// @pre the value type of the specified quaternion has to be a floating point type
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------

    template <class T>
    void    fromYAxisRotation        (TQuaternion<T>& f_quat, const CRadian& f_angleY);


    //-----------------------------------------------------------------------------
    /// sets quaternion to a rotation around z-axis with specified angle.
    /// @pre the value type of the specified quaternion has to be a floating point type
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------

    template <class T>
    void    fromZAxisRotation        (TQuaternion<T>& f_quat, const CRadian& f_angleZ);


    //-----------------------------------------------------------------------------
    /// Spherical linear interpolation (slerp). 
    /// @par Description:
    /// Sperical linear interpolation is an operation that, given two unit 
    /// quaternions q and r, and a parameter @f$t\ \in \left [ 0,1 \right ]@f$, 
    /// computes an interpolated quaternion.\n  
    /// For values of t the slerp function computes unique interpolated quaternions 
    /// that together constitute the shortest arc on four-dimensional unit sphere from 
    /// q(t=0) to r(t=1). The arc is located on the circle that is formed from the 
    /// intersection between the plane given by q,r and the origin, and the 
    /// four-dimensional unit sphere.\n 
    /// The computed rotation quaternion rotates around a fixed axis with constant 
    /// speed.
    /// @par Implemented formula:
    /// @f[slerp(q,r,t) = \frac{\sin (\phi\ (1-t))}{\sin (\phi)}q+\frac{\sin (\phi \ t)}{\sin (\phi)}r@f]
    /// @f[\text{with} \qquad \cos (\phi) = q \bullet r @f]
    /// @return the interpolated quaternion
    /// @pre 
    /// - the value type of the specified quaternion has to be a floating point type
    /// - q and r have to be unit length
    /// - @f$t\ \in \left [ 0,1 \right ]@f$
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------

    template <class T>    
    TQuaternion<T>    slerp    (   const TQuaternion<T>&   f_q_quat,   ///< start quaternion (t=0)
                                const TQuaternion<T>&   f_r_quat,   ///< end quaternion (t=1)
                                const float64_t         f_t         ///< interpolation parameter t E[0,1]
                            ); 
    

    //-----------------------------------------------------------------------------
    /// spherical cubic (quadrangle)  interpolation (squad). 
    /// @par Description:
    /// The function squad does a cubic interpolation between data points p and q 
    /// by an amount @f$t\ \in \left [ 0,1 \right ]@f$.\n
    /// To achieve C2 continuity between curve segments, a cubic interpolation must 
    /// be done. In quaternion space this is somewhat complicated. The method used 
    /// is to compose a cubic interpolation as a set of three linear interpolations.\n 
    /// First between the data points and two other (carefully chosen) points, and 
    /// then between the remaining points by an amount k specified by the logistic 
    /// equation @f$k = 2t(1-t)@f$.\n 
    /// If the auxiliary points are chosen properly, then C2 continuity can be ensured. 
    /// @par
    /// @f[squad ( p, a, b, q, t) = slerp(\ slerp( p, q, t),\ slerp( a, b, t),\ 2t(1-t))@f]
    /// @par
    /// The points a and b are called inner quadrangle points, and have to be chosen 
    /// carefully so that continuity is guaranteed across segments. 
    /// Given a quadrangle of quaternions,  where @f$q_i@f$ and @f$q_{i+1}@f$ are our 
    /// data points, we can guarantee C2 continuity by choosing @f$s_i@f$ and @f$s_{i+1}@f$ 
    /// as follows:
    /// @par
    /// @f[\text{for} \qquad squad(q_i,s_i,s_{i+1},q_{i+1})@f]
    /// @f[s_i = q_i * \exp \left (-\frac {\log (q_i^{-1}\bullet q_{i+1})+\log (q_i^{-1}\bullet q_{i-1})}{4}\right )@f]
    /// @return the interpolated quaternion
    /// @pre 
    /// - the value type of the specified quaternion has to be a floating point type
    /// - p,a,b,q have to be unit length
    /// - @f$t\ \in \left [ 0,1 \right ]@f$
    /// @author zvh2hi
    /// @relatesalso TQuaternion
    //-----------------------------------------------------------------------------

    template <class T>      
    TQuaternion<T>    squad    (   const TQuaternion<T>&   f_p_quat,   ///< start quaternion (t=0)
                                const TQuaternion<T>&   f_a_quat,   ///< inner quadrangle point 
                                const TQuaternion<T>&   f_b_quat,   ///< inner quadrangle point 
                                const TQuaternion<T>&   f_q_quat,   ///< end quaternion (t=1)
                                const float64_t         f_t         ///< interpolation parameter t E[0,1]
                            );
    
    // binary

    //-----------------------------------------------------------------------------
    /// binary mul operator.
    /// @author zvh2hi
    /// @relatesalso TQuaternion 
    //-----------------------------------------------------------------------------
    template <class T>
    TQuaternion<T>        operator*    (const TQuaternion<T>& f_op1, const TQuaternion<T>& f_op2);

    //-----------------------------------------------------------------------------
    /// binary plus operator.
    /// @author zvh2hi
    /// @relatesalso TQuaternion 
    //-----------------------------------------------------------------------------
    template <class T>
    TQuaternion<T>        operator+    (const TQuaternion<T>& f_op1, const TQuaternion<T>& f_op2);

    //-----------------------------------------------------------------------------
    /// binary minus operator.
    /// @author zvh2hi
    /// @relatesalso TQuaternion 
    //-----------------------------------------------------------------------------
    template <class T>
    TQuaternion<T>        operator-    (const TQuaternion<T>& f_op1, const TQuaternion<T>& f_op2);


}    // namespace vfc closed

#include "vfc/geo/vfc_quat_ops.inl"

#endif //VFC_QUAT_OPS_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_quat_ops.hpp  $
//  Revision 1.4 2007/07/23 09:46:51MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.3 2006/11/16 14:41:12CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.2 2006/10/13 10:00:24CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed header/footer template    
//  - added documentation
//  Revision 1.1 2006/07/07 10:07:03CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/geo/geo.pj
//=============================================================================
