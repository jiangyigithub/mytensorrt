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
///     $Source: vfc_linalg_ops1d2d_mn.hpp $
///     $Revision: 1.3 $
///     $Author: gaj2kor $
///     $Date: 2008/08/29 15:04:58MESZ $
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

#ifndef VFC_LINALG_OPS1D2D_MN_HPP_INCLUDED
#define VFC_LINALG_OPS1D2D_MN_HPP_INCLUDED

#include "vfc/linalg/vfc_linalg_ops1d2d_common.hpp"
#include "vfc/linalg/vfc_linalg_exp1d2d_mn.hpp"
namespace vfc
{
    namespace linalg
    {
        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg_fixed BEGIN
        //-------------------------------------------------------------------------
        /// @addtogroup vfc_group_linalg_fixed
        /// @{
        //-------------------------------------------------------------------------
        //---------------------------------------------------------------------
        //! subView function overload to extract a partial view of
        //! a given fixed size matrix
        //! @param f_src_r     Object of TMatrixBase
        //! @param f_row0      Number of rows
        //! @param f_col0      Number of columns
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d2d_mn.hpp $
        //---------------------------------------------------------------------
        template <vfc::int32_t SubViewSizeValue,
            class ValueType, class MatrixType,
            vfc::int32_t RowValue, vfc::int32_t ColValue>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DSubViewFrom2DOp<ValueType,
                TMatrixConstRef<ValueType, MatrixType,
                    TStaticRectangle<RowValue, ColValue> >,
                TStaticRectangle<SubViewSizeValue, 1> >,
            TStaticRectangle<SubViewSizeValue, 1> >
        subView(const TMatrixBase<ValueType, MatrixType,
                TStaticRectangle<RowValue, ColValue> >& f_src_r,
            vfc::int32_t f_row0, vfc::int32_t f_col0);

        //---------------------------------------------------------------------
        //! subView function overload to extract a partial view of
        //! a given fixed size 2d expression
        //! @param f_src_r     Object of TExp2D
        //! @param f_row0      Number of rows
        //! @param f_col0      Number of columns
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d2d_mn.hpp $
        //---------------------------------------------------------------------
        template <vfc::int32_t SubViewSizeValue,
            class ValueType, class OperatorType,
            vfc::int32_t RowValue, vfc::int32_t ColValue>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DSubViewFrom2DOp<ValueType,
                intern::TExp2D<ValueType, OperatorType,
                    TStaticRectangle<RowValue, ColValue> >,
                TStaticRectangle<SubViewSizeValue, 1> >,
            TStaticRectangle<SubViewSizeValue, 1> >
        subView(const intern::TExp2D<ValueType, OperatorType,
                TStaticRectangle<RowValue, ColValue> >& f_src_r,
            vfc::int32_t f_row0, vfc::int32_t f_col0);
        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg_fixed END
        //-------------------------------------------------------------------------
        /// @}
        //-------------------------------------------------------------------------

    }
}

#include "vfc/linalg/vfc_linalg_ops1d2d_mn.inl"

#endif // VFC_LINALG_OPS1D2D_MN_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_ops1d2d_mn.hpp  $
//  Revision 1.3 2008/08/29 15:04:58MESZ gaj2kor 
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.2 2008/07/31 14:08:40IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:23IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
