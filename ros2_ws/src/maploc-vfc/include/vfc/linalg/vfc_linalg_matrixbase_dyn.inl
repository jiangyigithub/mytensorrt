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
///     $Source: vfc_linalg_matrixbase_dyn.inl $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (jat2hi) $
///     $Date: 2007/05/09 10:21:22MESZ $
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

template<class ValueType, class DerivedType>
inline const ValueType& 
vfc::linalg::TMatrixBase<ValueType, DerivedType, 
    vfc::linalg::CDynamicRectangle>::operator() 
    (size_type f_row, size_type f_col) const
{
    // Coding rule violation (downcasting not allowed):
    // Makes use of the "curiously recurring template pattern" to avoid
    // virtual function calls when using operators via the TVectorBase type.
    // As we know our derived_type here at compile time, it is guaranteed
    // that the static_cast casts to the correct type (i.e. derived_type).
    // By this, we achieve the effect of virtual functions without the runtime
    // cost (vtable)
    const derived_type* me = static_cast<const derived_type*>(this);
    return me->operator ()(f_row, f_col);
}

template<class ValueType, class DerivedType>
inline ValueType& 
vfc::linalg::TMatrixBase<ValueType, DerivedType, 
    vfc::linalg::CDynamicRectangle>::operator() 
    (size_type f_row, size_type f_col)
{
    // Coding rule violation (downcasting not allowed):
    // Makes use of the "curiously recurring template pattern" to avoid
    // virtual function calls when using operators via the TVectorBase type.
    // As we know our derived_type here at compile time, it is guaranteed
    // that the static_cast casts to the correct type (i.e. derived_type).
    // By this, we achieve the effect of virtual functions without the runtime
    // cost (vtable)
    derived_type* me = static_cast<derived_type*>(this);
    return me->operator ()(f_row, f_col);
}

template<class ValueType, class DerivedType>
inline vfc::int32_t
vfc::linalg::TMatrixBase<ValueType, DerivedType, 
    vfc::linalg::CDynamicRectangle>::getNbRows(void) const
{
    // Coding rule violation (downcasting not allowed):
    // Makes use of the "curiously recurring template pattern" to avoid
    // virtual function calls when using operators via the TVectorBase type.
    // As we know our derived_type here at compile time, it is guaranteed
    // that the static_cast casts to the correct type (i.e. derived_type).
    // By this, we achieve the effect of virtual functions without the runtime
    // cost (vtable)
    const derived_type* me = static_cast<const derived_type*>(this);
    return me->getNbRows();
}


template<class ValueType, class DerivedType>
inline vfc::int32_t
vfc::linalg::TMatrixBase<ValueType, DerivedType, 
    vfc::linalg::CDynamicRectangle>::getNbColumns(void) const
{
    // Coding rule violation (downcasting not allowed):
    // Makes use of the "curiously recurring template pattern" to avoid
    // virtual function calls when using operators via the TVectorBase type.
    // As we know our derived_type here at compile time, it is guaranteed
    // that the static_cast casts to the correct type (i.e. derived_type).
    // By this, we achieve the effect of virtual functions without the runtime
    // cost (vtable)
    const derived_type* me = static_cast<const derived_type*>(this);
    return me->getNbColumns();
}



//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_matrixbase_dyn.inl  $
//  Revision 1.1 2007/05/09 10:21:22MESZ Jaeger Thomas (CC-DA/EPV2) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
