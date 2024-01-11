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
//       Projectname:
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
//        Name: dkn2kor
//  Department:
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_linalg_matrix33_ops.hpp $
///     $Revision: 1.4 $
///     $Author: gaj2kor $
///     $Date: 2008/08/29 15:04:49MESZ $
///     $Locker:  $
///     $Name: 0032 RC1  $
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

#ifndef VFC_LINALG_MATRIX33_OPS_HPP_INCLUDED
#define VFC_LINALG_MATRIX33_OPS_HPP_INCLUDED

#include "vfc/linalg/vfc_linalg_matrix33.hpp"
#include "vfc/core/vfc_trig.hpp"

namespace vfc
{    // namespace vfc opened
    namespace linalg
    {    // namespace linalg opened

        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg_tiny BEGIN
        //-------------------------------------------------------------------------
        /// @addtogroup vfc_group_linalg_tiny
        /// @{
        //-------------------------------------------------------------------------
        //---------------------------------------------------------------------
        //!  Returns matrix determinant
        //! @param f_mat     Object of TMatrix33
        //! @return Returns matrix determinant
        //! @author  jat2hi
        //! $Source: vfc_linalg_matrix33_ops.hpp $
        //---------------------------------------------------------------------
        template <class T>
        T                det            (const TMatrix33<T>& f_mat);

        //---------------------------------------------------------------------
        //!  Returns matrix trace
        //! @param f_mat     Object of TMatrix33
        //! @return Returns matrix trace
        //! @author  jat2hi
        //! $Source: vfc_linalg_matrix33_ops.hpp $
        //---------------------------------------------------------------------
        template <class T>
        T                trace        (const TMatrix33<T>& f_mat);

        //---------------------------------------------------------------------
        //!  Returns the inverse of given matrix
        //! @param f_mat     Object of TMatrix33
        //! @return Returns matrix inverse
        //! @author  jat2hi
        //! $Source: vfc_linalg_matrix33_ops.hpp $
        //---------------------------------------------------------------------
        template <class T>
        TMatrix33<T>        inverse        (const TMatrix33<T>& f_mat);

        //---------------------------------------------------------------------
        //!  Sets given matrix to a rotation matrix about x-axis with specified angle
        //! @param f_mat     Object of TMatrix33
        //! @param f_angleX  Object of CRadian
        //! @author  jat2hi
        //! $Source: vfc_linalg_matrix33_ops.hpp $
        //---------------------------------------------------------------------
        template <class ValueType>
        void fromXAxisRotation (TMatrix33<ValueType>& f_mat,
                                const CRadian& f_angleX);

        //---------------------------------------------------------------------
        //!  Sets given matrix to a rotation matrix about y-axis with specified angle
        //! @param f_mat     Object of TMatrix33
        //! @param f_angleY  Object of CRadian
        //! @author  jat2hi
        //! $Source: vfc_linalg_matrix33_ops.hpp $
        //---------------------------------------------------------------------
        template <class ValueType>
        void fromYAxisRotation (TMatrix33<ValueType>& f_mat,
                                const CRadian& f_angleY);

        //---------------------------------------------------------------------
        //!  Sets given matrix to a rotation matrix about z-axis with specified angle
        //! @param f_mat     Object of TMatrix33
        //! @param f_angleZ  Object of CRadian
        //! @author  jat2hi
        //! $Source: vfc_linalg_matrix33_ops.hpp $
        //---------------------------------------------------------------------
        template <class ValueType>
        void fromZAxisRotation (TMatrix33<ValueType>& f_mat,
                                const CRadian& f_angleZ);

        //---------------------------------------------------------------------
        //!  Sets given matrix to a concatenated rotation matrix from specified angles
        //! @param f_mat     Object of TMatrix33
        //! @param f_angleX  Object of CRadian
        //! @param f_angleY  Object of CRadian
        //! @param f_angleZ  Object of CRadian
        //! @author  jat2hi
        //! $Source: vfc_linalg_matrix33_ops.hpp $
        //---------------------------------------------------------------------
        template <class ValueType>
        void fromZYXAxisRotation (TMatrix33<ValueType>& f_mat,
                                const CRadian& f_angleX,
                                const CRadian& f_angleY,
                                const CRadian& f_angleZ);

        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg_tiny END
        //-------------------------------------------------------------------------
        /// @}
        //-------------------------------------------------------------------------

    }    // namespace linalg closed
}    // namespace vfc closed

#include "vfc/linalg/vfc_linalg_matrix33_ops.inl"

#endif //VFC_LINALG_MATRIX33_OPS_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_matrix33_ops.hpp  $
//  Revision 1.4 2008/08/29 15:04:49MESZ gaj2kor 
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.3 2008/07/31 14:08:27IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.2 2007/06/22 18:40:00IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  - corrected redeclared function parameter names (mantis 1692)
//  Revision 1.1 2007/05/09 10:21:21CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
