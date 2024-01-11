//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2007 by Robert Bosch GmbH. All rights reserved.
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
///     $Source: vfc_linalg_ops1d_n.inl $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (jat2hi) $
///     $Date: 2007/05/09 10:21:24MESZ $
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

template <vfc::int32_t SubViewSizeValue, class ValueType, 
    class VectorType, vfc::int32_t RowValue>
const 
vfc::linalg::intern::TExp1D<ValueType, 
    vfc::linalg::intern::TExp1DSubViewFrom1DOp<ValueType,
        vfc::linalg::TVectorConstRef<ValueType, VectorType, 
            vfc::linalg::TStaticRectangle<RowValue, 1> >,
        vfc::linalg::TStaticRectangle<SubViewSizeValue, 1> >,
    vfc::linalg::TStaticRectangle<SubViewSizeValue, 1> >
vfc::linalg::subView(
    const TVectorBase<ValueType, VectorType, TStaticRectangle<RowValue, 1> >& f_src_r,
    vfc::int32_t f_pos0)
{
    return intern::TExp1D<ValueType, 
        intern::TExp1DSubViewFrom1DOp<ValueType,
            TVectorConstRef<ValueType, VectorType, TStaticRectangle<RowValue, 1> >,
            TStaticRectangle<SubViewSizeValue, 1> >,
        TStaticRectangle<SubViewSizeValue, 1> >(
            intern::TExp1DSubViewFrom1DOp<ValueType,
                TVectorConstRef<ValueType, VectorType, TStaticRectangle<RowValue, 1> >,
                TStaticRectangle<SubViewSizeValue, 1> >(f_src_r, f_pos0));
}

template <vfc::int32_t SubViewSizeValue, class ValueType, 
    class OperatorType, vfc::int32_t RowValue>
const 
vfc::linalg::intern::TExp1D<ValueType, 
    vfc::linalg::intern::TExp1DSubViewFrom1DOp<ValueType,
        vfc::linalg::intern::TExp1D<ValueType, OperatorType, 
            vfc::linalg::TStaticRectangle<RowValue, 1> >,
        vfc::linalg::TStaticRectangle<SubViewSizeValue, 1> >,
    vfc::linalg::TStaticRectangle<SubViewSizeValue, 1> >
vfc::linalg::subView(
    const intern::TExp1D<ValueType, OperatorType, TStaticRectangle<RowValue, 1> >& f_src_r,
    vfc::int32_t f_pos0)
{
    return intern::TExp1D<ValueType, 
        intern::TExp1DSubViewFrom1DOp<ValueType,
            intern::TExp1D<ValueType, OperatorType, TStaticRectangle<RowValue, 1> >,
            TStaticRectangle<SubViewSizeValue, 1> >,
        TStaticRectangle<SubViewSizeValue, 1> >(
            intern::TExp1DSubViewFrom1DOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, TStaticRectangle<RowValue, 1> >,
                TStaticRectangle<SubViewSizeValue, 1> >(f_src_r, f_pos0));
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_ops1d_n.inl  $
//  Revision 1.1 2007/05/09 10:21:24MESZ Jaeger Thomas (CC-DA/EPV2) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
