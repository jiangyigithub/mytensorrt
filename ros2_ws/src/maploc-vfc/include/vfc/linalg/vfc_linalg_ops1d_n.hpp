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
///     $Source: vfc_linalg_ops1d_n.hpp $
///     $Revision: 1.3 $
///     $Author: gaj2kor $
///     $Date: 2008/08/29 15:04:55MESZ $
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

#ifndef VFC_LINALG_OPS1D_N_HPP_INCLUDED
#define VFC_LINALG_OPS1D_N_HPP_INCLUDED

#include "vfc/linalg/vfc_linalg_ops1d_common.hpp"
#include "vfc/linalg/vfc_linalg_exp1d_n.hpp"

namespace vfc
{// namespace vfc opened

    namespace linalg
    {// namespace linalg opened

        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg_fixed BEGIN
        //-------------------------------------------------------------------------
        /// @addtogroup vfc_group_linalg_fixed
        /// @{
        //-------------------------------------------------------------------------
        //---------------------------------------------------------------------
        //! subView function overload to extract a partial view of
        //! a given fixed size vector
        //! @param f_src_r     Object of TVectorBase
        //! @param f_pos0
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_n.hpp $
        //---------------------------------------------------------------------
        template <vfc::int32_t SubViewSizeValue, class ValueType,
            class VectorType, vfc::int32_t RowValue>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DSubViewFrom1DOp<ValueType,
                TVectorConstRef<ValueType, VectorType, TStaticRectangle<RowValue, 1> >,
                TStaticRectangle<SubViewSizeValue, 1> >,
            TStaticRectangle<SubViewSizeValue, 1> >
        subView(const TVectorBase<ValueType, VectorType, TStaticRectangle<RowValue, 1> >& f_src_r,
            vfc::int32_t f_pos0);

        //---------------------------------------------------------------------
        //! subView function overload to extract a partial view of
        //! a given fixed size 1d expression
        //! @param f_src_r     Object of TExp1D
        //! @param f_pos0
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d_n.hpp $
        //---------------------------------------------------------------------
        template <vfc::int32_t SubViewSizeValue, class ValueType,
            class OperatorType, vfc::int32_t RowValue>
        const
        intern::TExp1D<ValueType,
            intern::TExp1DSubViewFrom1DOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, TStaticRectangle<RowValue, 1> >,
                TStaticRectangle<SubViewSizeValue, 1> >,
            TStaticRectangle<SubViewSizeValue, 1> >
        subView(const intern::TExp1D<ValueType, OperatorType, TStaticRectangle<RowValue, 1> >& f_src_r,
            vfc::int32_t f_pos0);

        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg_fixed END
        //-------------------------------------------------------------------------
        /// @}
        //-------------------------------------------------------------------------

    }   // namespace linalg closed


}   // namespace vfc closed

#include "vfc/linalg/vfc_linalg_ops1d_n.inl"

#endif // VFC_LINALG_OPS1D_N_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_ops1d_n.hpp  $
//  Revision 1.3 2008/08/29 15:04:55MESZ gaj2kor 
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.2 2008/07/31 14:08:37IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:24IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
