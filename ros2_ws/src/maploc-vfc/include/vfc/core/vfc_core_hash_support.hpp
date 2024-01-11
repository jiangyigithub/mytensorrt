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
///     $Source: vfc_core_hash_support.hpp $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/05/09 16:04:06MESZ $
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

#ifndef VFC_CORE_HASH_SUPPORT_HPP_INCLUDED
#define VFC_CORE_HASH_SUPPORT_HPP_INCLUDED

#include "vfc/core/vfc_hash.hpp"

#include "vfc/core/vfc_rect.hpp"
#include "vfc/core/vfc_tuple.hpp"

namespace vfc
{

    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType, template<class, class, class> class ConvertPolicyType>
    class TSIUnits;

    //=========================================================================
    // vfc types specializations
    //-------------------------------------------------------------------------
    //  for vfc/core

    //=========================================================================
    // THash<SIUnits>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class ValueType, class UnitInfoType, class RBInfoType>
    struct THash<vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType> >
    {
        typedef vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType>            local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            return vfc::intern::hashValue(f_val.value());
        }
    };


    //=========================================================================
    // THash<CRect>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<>
    struct THash<vfc::CRect>
    {
        typedef vfc::CRect                                          local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            vfc::hash_type l_seed = 0;
            vfc::serialHash(l_seed, f_val.left());
            vfc::serialHash(l_seed, f_val.top());
            vfc::serialHash(l_seed, f_val.width());
            vfc::serialHash(l_seed, f_val.height());
            return l_seed;
        }
    };
    //=========================================================================
    // THash<CPoint>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<>
    struct THash<vfc::CPoint>
    {
        typedef vfc::CPoint                                          local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            vfc::hash_type l_seed = 0;
            vfc::serialHash(l_seed, f_val.x());
            vfc::serialHash(l_seed, f_val.y());
            return l_seed;
        }
    };
    //=========================================================================
    // THash<CSize>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<>
    struct THash<vfc::CSize>
    {
        typedef vfc::CSize                                          local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            vfc::hash_type l_seed = 0;
            vfc::serialHash(l_seed, f_val.cx());
            vfc::serialHash(l_seed, f_val.cy());
            return l_seed;
        }
    };

    //=========================================================================
    // THash<TPair>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class ValueType>
    struct THash<vfc::TPair<ValueType> >
    {
        typedef vfc::TPair<ValueType>                               local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            vfc::hash_type l_seed = 0;
            vfc::serialHash(l_seed, f_val.first());
            vfc::serialHash(l_seed, f_val.second());
            return l_seed;
        }
    };

    //=========================================================================
    // THash<TTriple>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class ValueType>
    struct THash<vfc::TTriple<ValueType> >
    {
        typedef vfc::TTriple<ValueType>                               local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            vfc::hash_type l_seed = 0;
            vfc::serialHash(l_seed, f_val.first());
            vfc::serialHash(l_seed, f_val.second());
            vfc::serialHash(l_seed, f_val.third());
            return l_seed;
        }
    };

    //=========================================================================
    // THash<TTriple>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class ValueType>
    struct THash<vfc::TQuadruple<ValueType> >
    {
        typedef vfc::TQuadruple<ValueType>                               local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            vfc::hash_type l_seed = 0;
            vfc::serialHash(l_seed, f_val.first());
            vfc::serialHash(l_seed, f_val.second());
            vfc::serialHash(l_seed, f_val.third());
            vfc::serialHash(l_seed, f_val.fourth());
            return l_seed;
        }
    };

}

#endif // VFC_CORE_HASH_SUPPORT_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_core_hash_support.hpp  $
//  Revision 1.1 2014/05/09 16:04:06MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/core/core.pj
//=============================================================================
