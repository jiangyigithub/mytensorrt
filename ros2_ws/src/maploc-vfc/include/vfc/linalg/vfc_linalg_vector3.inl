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
///     $Source: vfc_linalg_vector3.inl $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (jat2hi) $
///     $Date: 2007/05/09 10:21:27MESZ $
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
vfc::linalg::TVector3<ValueType>::TVector3(
    const ValueType& f_value0_r, const ValueType& f_value1_r,
    const ValueType& f_value2_r) 
    : TVectorN<ValueType, 3>()
{
    operator[](0) = f_value0_r;
    operator[](1) = f_value1_r;
    operator[](2) = f_value2_r;
}

template <class ValueType>
inline
vfc::linalg::TVector3<ValueType>::TVector3(
    const TVector3<ValueType>& f_param_r)
    : TVectorN<ValueType, 3>(f_param_r)
{
}

template <class ValueType>
inline
vfc::linalg::TVector3<ValueType>::TVector3(
    const TVectorN<ValueType, 3>& f_param_r)
    : TVectorN<ValueType, 3>(f_param_r)
{
}


template <class ValueType>
inline
void vfc::linalg::TVector3<ValueType>::set(const ValueType& f_value0_r, const ValueType& f_value1_r, const ValueType& f_value2_r)
{
    operator[](0) = f_value0_r;
    operator[](1) = f_value1_r;
    operator[](2) = f_value2_r;
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_vector3.inl  $
//  Revision 1.1 2007/05/09 10:21:27MESZ Jaeger Thomas (CC-DA/EPV2) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
