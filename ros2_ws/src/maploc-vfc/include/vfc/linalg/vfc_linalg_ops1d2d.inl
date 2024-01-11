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
///     $Source: vfc_linalg_ops1d2d.inl $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (jat2hi) $
///     $Date: 2007/05/09 10:21:23MESZ $
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

template <class ValueType, class MatrixType, class ShapeType>
const 
vfc::linalg::intern::TExp1D<ValueType, 
    vfc::linalg::intern::TExp1DSubViewFrom2DOp<ValueType,
        vfc::linalg::TMatrixConstRef<ValueType, MatrixType, ShapeType>,
        vfc::linalg::CDynamicRectangle>,
    vfc::linalg::CDynamicRectangle>
vfc::linalg::subView(const TMatrixBase<ValueType, MatrixType, ShapeType>& f_src_r,
    vfc::int32_t f_row0, vfc::int32_t f_col0,
    vfc::int32_t f_count)
{
    return intern::TExp1D<ValueType, 
        intern::TExp1DSubViewFrom2DOp<ValueType,
            TMatrixConstRef<ValueType, MatrixType, ShapeType>,
            CDynamicRectangle>,
        CDynamicRectangle>(
            intern::TExp1DSubViewFrom2DOp<ValueType,
                TMatrixConstRef<ValueType, MatrixType, ShapeType>,
                CDynamicRectangle>(f_src_r, f_row0, f_col0, f_count));
}

template <class ValueType, class OperatorType, class ShapeType>
const 
vfc::linalg::intern::TExp1D<ValueType, 
    vfc::linalg::intern::TExp1DSubViewFrom2DOp<ValueType,
        vfc::linalg::intern::TExp2D<ValueType, OperatorType, ShapeType>,
        vfc::linalg::CDynamicRectangle>,
    vfc::linalg::CDynamicRectangle>
vfc::linalg::subView(const intern::TExp2D<ValueType, OperatorType, ShapeType>& f_src_r,
    vfc::int32_t f_row0, vfc::int32_t f_col0,
    vfc::int32_t f_count)
{
    return intern::TExp1D<ValueType, 
        intern::TExp1DSubViewFrom2DOp<ValueType,
            intern::TExp2D<ValueType, OperatorType, ShapeType>,
            CDynamicRectangle>,
        CDynamicRectangle>(
            intern::TExp1DSubViewFrom2DOp<ValueType,
                intern::TExp2D<ValueType, OperatorType, ShapeType>,
                CDynamicRectangle>(f_src_r, f_row0, f_col0, f_count));
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_ops1d2d.inl  $
//  Revision 1.1 2007/05/09 10:21:23MESZ Jaeger Thomas (CC-DA/EPV2) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
