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
*     $Source: vfc_mat3_ops.hpp $
*     $Revision: 1.11 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
*     $Date: 2007/07/23 09:42:06MESZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

#ifndef VFC_MAT3_OPS_HPP_INCLUDED
#define VFC_MAT3_OPS_HPP_INCLUDED

#include "vfc/core/vfc_trig.hpp"
#include "vfc/geo/vfc_mat3.hpp"

namespace vfc
{    // namespace vfc opened

    //-----------------------------------------------------------------------------
    //! returns the determinant of specified matrix.
    //! @author zvh2hi
    //! @relatesalso TMatrix3
    //-----------------------------------------------------------------------------

    template <class T>
    T                det            (const TMatrix3<T>& f_mat);

    //-----------------------------------------------------------------------------
    //! returns trace of specified matrix.
    //! @author zvh2hi
    //! @relatesalso TMatrix3
    //-----------------------------------------------------------------------------

    template <class T>
    T                trace        (const TMatrix3<T>& f_mat);
    
    //-----------------------------------------------------------------------------
    //! returns inverse of specified matrix.
    //! @author zvh2hi
    //! @relatesalso TMatrix3
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix3<T>        inverse        (const TMatrix3<T>& f_mat);

    //-----------------------------------------------------------------------------
    //! returns specified matrix transposed.
    //! @author zvh2hi
    //! @relatesalso TMatrix3
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix3<T>        transpose    (const TMatrix3<T>& f_mat);

    //-----------------------------------------------------------------------------
    //! sets specified matrix to identity.
    //! @code
    //!     1   0   0
    //! I=  0   1   0
    //!     0   0   1
    //! @endcode
    //! @author zvh2hi
    //! @relatesalso TMatrix3
    //-----------------------------------------------------------------------------

    template <class T>
    void    identity    (TMatrix3<T>& f_mat);

    //-----------------------------------------------------------------------------
    //! sets given matrix to a rotation matrix about x-axis with specified angle.
    //! @code
    //!       1   0   0
    //! Rx=   0  cx -sx
    //!       0  sx  cx
    //! @endcode
    //! @author zvh2hi
    //! @relatesalso TMatrix3
    //-----------------------------------------------------------------------------
    template <class T>
    void    fromXAxisRotation   (TMatrix3<T>& f_mat, const CRadian& f_angleX);

    //-----------------------------------------------------------------------------
    //! sets given matrix to a rotation matrix about y-axis with specified angle.
    //! @code
    //!      cy   0  sy
    //! Ry=   0   1   0
    //!     -sy   0  cy
    //! @endcode
    //! @author zvh2hi
    //! @relatesalso TMatrix3
    //-----------------------------------------------------------------------------

    template <class T>
    void    fromYAxisRotation    (TMatrix3<T>& f_mat, const CRadian& f_angleY);

    //-----------------------------------------------------------------------------
    //! sets given matrix to a rotation matrix about z-axis with specified angle.
    //! @code
    //!     cz -sz   0
    //! Rz= sz  cx   0
    //!      0   0  -1
    //! @endcode
    //! @author zvh2hi
    //! @relatesalso TMatrix3
    //-----------------------------------------------------------------------------

    template <class T>
    void    fromZAxisRotation    (TMatrix3<T>& f_mat, const CRadian& f_angleZ);

    //-----------------------------------------------------------------------------
    //! sets given matrix to a concatenated rotation matrix from specified 
    //! euler angles with order: Rzyx = Rz*Ry*Rx.
    //! @code
    //! concatenated matrix Rzyx = Rz * Ry * Rx is:
    //! 
    //!         cy * cz,  sx * sy * cz - cx * sz,  cx * sy * cz + sx * sz
    //! Rzyx =  cy * sz,  sx * sy * sz + cx * cz,  cx * sy * sz - sx * cz
    //!           -sy,            sx * cy,               cx * cy
    //! @endcode
    //! @author zvh2hi
    //! @relatesalso TMatrix3
    //-----------------------------------------------------------------------------

    template <class T>
    void    fromZYXAxisRotation(    TMatrix3<T>&    f_mat, 
                                    const CRadian&  f_angleX, 
                                    const CRadian&  f_angleY, 
                                    const CRadian&  f_angleZ);
    
    //-----------------------------------------------------------------------------
    //! unary minus operator.
    //! @author zvh2hi
    //! @relatesalso TMatrix3
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix3<T>        operator-    (const TMatrix3<T>& f_mat);

    //-----------------------------------------------------------------------------
    //! returns matrix*matrix product.
    //! @author zvh2hi
    //! @relatesalso TMatrix3
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix3<T>        operator*    (const TMatrix3<T>& f_op1, const TMatrix3<T>& f_op2);

    //-----------------------------------------------------------------------------
    //! binary plus operator.
    //! @author zvh2hi
    //! @relatesalso TMatrix3
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix3<T>        operator+    (const TMatrix3<T>& f_op1, const TMatrix3<T>& f_op2);

    //-----------------------------------------------------------------------------
    //! binary minus operator.
    //! @author zvh2hi
    //! @relatesalso TMatrix3
    //-----------------------------------------------------------------------------

    template <class T>
    TMatrix3<T>        operator-    (const TMatrix3<T>& f_op1, const TMatrix3<T>& f_op2);

}    // namespace vfc closed

#include "vfc/geo/vfc_mat3_ops.inl"

#endif //VFC_MAT3_OPS_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_mat3_ops.hpp  $
Revision 1.11 2007/07/23 09:42:06MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
- doxygen grouping (mantis1744)
Revision 1.10 2006/11/16 14:41:09CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.9 2006/02/14 09:06:09CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-changed function names
Revision 1.8 2005/12/19 15:34:40CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-changed rotation functions
Revision 1.7 2005/12/15 10:45:06CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-changed docu
Revision 1.6 2005/12/15 10:17:52CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-added generation of rotation matrices
Revision 1.5 2005/12/07 16:09:22CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-moved f_mat*vec to own header
Revision 1.4 2005/11/30 09:59:47CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-more ops
Revision 1.3 2005/11/08 14:56:36CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-renamed funcs
Revision 1.2 2005/11/07 18:20:57CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-changed project name to vfc/geo
********************************************************************/
