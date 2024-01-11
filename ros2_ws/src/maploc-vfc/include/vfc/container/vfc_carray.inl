//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2008 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy or use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  P R O J E C T   I N F O R M A T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc
//  Target system(s):
//       Compiler(s): c++ std conformal
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
//  F I L E   C O N T E N T S   A N D   R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @brief Implementation file.
///
/// @par Revision History:
///     $Source: vfc_carray.inl $
///     $Revision: 1.2 $
///     $Author: Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) $
///     $Date: 2012/01/18 06:58:27MEZ $
///     $Locker:  $
///     $Name:  $
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

#include "vfc/core/vfc_assert.hpp"

template<class ValueType, vfc::int32_t CapacityValue> 
inline
typename vfc::TCArray<ValueType, CapacityValue>::iterator 
vfc::TCArray<ValueType, CapacityValue>::begin()
{ 
    return m_value;
}

template<class ValueType, vfc::int32_t CapacityValue> 
inline
typename vfc::TCArray<ValueType, CapacityValue>::const_iterator 
vfc::TCArray<ValueType, CapacityValue>::begin() const
{
    return m_value;
}

template<class ValueType, vfc::int32_t CapacityValue> 
inline
typename vfc::TCArray<ValueType, CapacityValue>::iterator 
vfc::TCArray<ValueType, CapacityValue>::end()
{
    return (begin() + ARRAY_SIZE);
}

template<class ValueType, vfc::int32_t CapacityValue> 
inline
typename vfc::TCArray<ValueType, CapacityValue>::const_iterator 
vfc::TCArray<ValueType, CapacityValue>::end() const
{
    return (begin() + ARRAY_SIZE);
}

template<class ValueType, vfc::int32_t CapacityValue> 
inline
typename vfc::TCArray<ValueType, CapacityValue>::reverse_iterator 
vfc::TCArray<ValueType, CapacityValue>::rbegin() 
{
    return static_cast<reverse_iterator>(end());
}

template<class ValueType, vfc::int32_t CapacityValue> 
inline
typename vfc::TCArray<ValueType, CapacityValue>::const_reverse_iterator 
vfc::TCArray<ValueType, CapacityValue>::rbegin() const
{
    return static_cast<const_reverse_iterator>(end());
}

template<class ValueType, vfc::int32_t CapacityValue> 
inline
typename vfc::TCArray<ValueType, CapacityValue>::reverse_iterator 
vfc::TCArray<ValueType, CapacityValue>::rend() 
{
    return static_cast<reverse_iterator>(begin());
}

template<class ValueType, vfc::int32_t CapacityValue> 
inline
typename vfc::TCArray<ValueType, CapacityValue>::const_reverse_iterator 
vfc::TCArray<ValueType, CapacityValue>::rend() const
{
    return static_cast<const_reverse_iterator>(begin());
}

template<class ValueType,vfc::int32_t CapacityValue>
inline
typename vfc::TCArray<ValueType,CapacityValue>::reference
vfc::TCArray<ValueType,CapacityValue>::operator[](size_type f_pos)
{
    VFC_REQUIRE((f_pos >= 0) && (f_pos < ARRAY_SIZE));
    return m_value[f_pos];
}

template<class ValueType,vfc::int32_t CapacityValue>
inline
typename vfc::TCArray<ValueType,CapacityValue>::volatile_reference
vfc::TCArray<ValueType,CapacityValue>::operator[](size_type f_pos) volatile
{
    VFC_REQUIRE((f_pos >= 0) && (f_pos < ARRAY_SIZE));
    return m_value[f_pos];
}

template<class ValueType,vfc::int32_t CapacityValue>
inline
typename vfc::TCArray<ValueType,CapacityValue>::const_reference
vfc::TCArray<ValueType,CapacityValue>::operator[](size_type f_pos) const
{
    VFC_REQUIRE((f_pos >= 0) && (f_pos < ARRAY_SIZE));
    return m_value[f_pos];
}

template<class ValueType,vfc::int32_t CapacityValue>
inline
typename vfc::TCArray<ValueType,CapacityValue>::const_volatile_reference
vfc::TCArray<ValueType,CapacityValue>::operator[](size_type f_pos) const volatile
{
    VFC_REQUIRE((f_pos >= 0) && (f_pos < ARRAY_SIZE));
    return m_value[f_pos];
}

template<class ValueType,vfc::int32_t CapacityValue>
inline
typename vfc::TCArray<ValueType,CapacityValue>::size_type
vfc::TCArray<ValueType,CapacityValue>::capacity() const
{
    return static_cast<size_type>(CapacityValue);
}

template<class ValueType,vfc::int32_t CapacityValue>
inline
typename vfc::TCArray<ValueType,CapacityValue>::size_type
vfc::TCArray<ValueType,CapacityValue>::size() const
{
    return static_cast<size_type>(CapacityValue);
}

template<class ValueType,vfc::int32_t CapacityValue>
inline
typename vfc::TCArray<ValueType,CapacityValue>::size_type
vfc::TCArray<ValueType,CapacityValue>::max_size() const
{
    return static_cast<size_type>(CapacityValue);
}

template<class ValueType,vfc::int32_t CapacityValue>
inline
bool 
vfc::TCArray<ValueType,CapacityValue>::empty() 
{
    return false;
}
//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_carray.inl  $
//  Revision 1.2 2012/01/18 06:58:27MEZ Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) 
//  - Added additional functionalities size(),maxsize(),empty() (mantis 3184)
//  Revision 1.1 2010/08/11 21:11:30IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/container/container.pj
//  Revision 1.4 2009/07/29 14:40:09MESZ Vinaykumar Setty (RBEI/ESB2) (vmr1kor) 
//  - Volatile version of the access operators are added ( volatile reference and const volatile reference)  : mantis (2842 )
//  Revision 1.3 2008/12/08 21:49:59IST Muehlmann Karsten (CC-DA/ESV2) (MUK2LR) 
//  - add capacity() (mantis2466)
//  Revision 1.2 2008/08/11 11:19:47CEST Dhananjay N (RBEI/EAE6) (dhn1kor) 
//  -ValueType is added as class template parameter,Iterator interface is added and CArray is made ROMable.(mantis:-2214)
//  Revision 1.1 2008/07/08 18:03:20IST Dilip Krishna (RBEI/EAE6) (dkn2kor) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_carray/vfc_carray.pj
//=============================================================================
