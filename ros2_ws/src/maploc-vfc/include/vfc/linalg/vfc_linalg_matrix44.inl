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
///     $Source: vfc_linalg_matrix44.inl $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (jat2hi) $
///     $Date: 2007/05/09 10:21:21MESZ $
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

template <class ValueType>
inline
vfc::linalg::TMatrix44<ValueType>::TMatrix44(
    const ValueType& f_value00_r, const ValueType& f_value01_r,
    const ValueType& f_value02_r, const ValueType& f_value03_r,
    const ValueType& f_value10_r, const ValueType& f_value11_r,
    const ValueType& f_value12_r, const ValueType& f_value13_r,
    const ValueType& f_value20_r, const ValueType& f_value21_r,
    const ValueType& f_value22_r, const ValueType& f_value23_r,
    const ValueType& f_value30_r, const ValueType& f_value31_r,
    const ValueType& f_value32_r, const ValueType& f_value33_r) 
    : TMatrixMN<ValueType, 4, 4>()
{
    operator()(0, 0) = f_value00_r;
    operator()(0, 1) = f_value01_r;
    operator()(0, 2) = f_value02_r;
    operator()(0, 3) = f_value03_r;
    operator()(1, 0) = f_value10_r;
    operator()(1, 1) = f_value11_r;
    operator()(1, 2) = f_value12_r;
    operator()(1, 3) = f_value13_r;
    operator()(2, 0) = f_value20_r;
    operator()(2, 1) = f_value21_r;
    operator()(2, 2) = f_value22_r;
    operator()(2, 3) = f_value23_r;
    operator()(3, 0) = f_value30_r;
    operator()(3, 1) = f_value31_r;
    operator()(3, 2) = f_value32_r;
    operator()(3, 3) = f_value33_r;
}

template <class ValueType>
inline
vfc::linalg::TMatrix44<ValueType>::TMatrix44(
    const TMatrix44<ValueType>& f_param_r)
    : TMatrixMN<ValueType, 4, 4>(f_param_r)
{
}

template <class ValueType>
inline
vfc::linalg::TMatrix44<ValueType>::TMatrix44(
    const TMatrixMN<ValueType, 4, 4>& f_param_r)
    : TMatrixMN<ValueType, 4, 4>(f_param_r)
{
}

template <class ValueType>
inline
void vfc::linalg::TMatrix44<ValueType>::set(const ValueType& f_value00_r, const ValueType& f_value01_r,
                const ValueType& f_value02_r, const ValueType& f_value03_r,
                const ValueType& f_value10_r, const ValueType& f_value11_r,
                const ValueType& f_value12_r, const ValueType& f_value13_r,
                const ValueType& f_value20_r, const ValueType& f_value21_r,
                const ValueType& f_value22_r, const ValueType& f_value23_r,
                const ValueType& f_value30_r, const ValueType& f_value31_r,
                const ValueType& f_value32_r, const ValueType& f_value33_r) 
{
    operator()(0, 0) = f_value00_r;
    operator()(0, 1) = f_value01_r;
    operator()(0, 2) = f_value02_r;
    operator()(0, 3) = f_value03_r;
    operator()(1, 0) = f_value10_r;
    operator()(1, 1) = f_value11_r;
    operator()(1, 2) = f_value12_r;
    operator()(1, 3) = f_value13_r;
    operator()(2, 0) = f_value20_r;
    operator()(2, 1) = f_value21_r;
    operator()(2, 2) = f_value22_r;
    operator()(2, 3) = f_value23_r;
    operator()(3, 0) = f_value30_r;
    operator()(3, 1) = f_value31_r;
    operator()(3, 2) = f_value32_r;
    operator()(3, 3) = f_value33_r;
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_matrix44.inl  $
//  Revision 1.1 2007/05/09 10:21:21MESZ Jaeger Thomas (CC-DA/EPV2) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
