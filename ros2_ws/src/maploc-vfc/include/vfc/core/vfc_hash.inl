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
//       Projectname: vfc
//          Synopsis:
//  Target system(s): cross platform
//       Compiler(s): c++ std
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: Thomas Jaeger
//  Department: CC-DA/EPV2
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_hash.inl $
///     $Revision: 1.2 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/07/18 15:05:24MESZ $
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

#include "vfc/core/vfc_static_assert.hpp"

template<class ValueType>
void vfc::serialHash(vfc::hash_type& f_prevhash, const ValueType& f_nextObjToHash)
{
    vfc::THash<ValueType> l_hash;

    // Magic number 0x9e3779b9 is the reciprocal of the golden ratio
    // 1/phi = 0,6180339887 as hexadecimal representation. It is inserted,
    // because the bits within phi are not correlated, to have a random change
    // of each bit, independent of the input seed:
    f_prevhash ^= l_hash.operator()(f_nextObjToHash) + 0x9e3779b9U + (f_prevhash << 6) + (f_prevhash >> 2); 

    // The implementation is bound to a 32-bit hash type:
    VFC_STATIC_ASSERT(static_cast<vfc::size_t>(4) == sizeof(vfc::hash_type));
}

template<class ValueType>
vfc::hash_type vfc::intern::TGenericContainerHash<ValueType>::operator()(const ValueType& f_val) const
{
    vfc::hash_type hv = 0;
    for(typename ValueType::const_iterator it = f_val.begin(), end = f_val.end(); it != end; ++it)
    {
        vfc::serialHash(hv, *it);
    }
    return hv;
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_hash.inl  $
//  Revision 1.2 2014/07/18 15:05:24MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_hash: call theTHash function object in serializeHash() explicitly to avoid ambiguity due to associated namespace lookup (mantis0004583)
//  Revision 1.1 2014/05/09 16:04:36MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/core/core.pj
//=============================================================================
