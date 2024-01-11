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
///     $Source: vfc_container_hash_support.hpp $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/05/09 16:04:13MESZ $
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

#ifndef VFC_CONTAINER_HASH_SUPPORT_HPP_INCLUDED
#define VFC_CONTAINER_HASH_SUPPORT_HPP_INCLUDED

#include "vfc/core/vfc_hash.hpp"

#include "vfc/core/vfc_types.hpp"

#include "vfc/container/vfc_carray.hpp"
#include "vfc/container/vfc_fixedvector.hpp"
#include "vfc/container/vfc_fixedlist.hpp"
#include "vfc/container/vfc_fixedcircularbuffer.hpp"

namespace vfc
{

    //=========================================================================
    // vfc types specializations
    //-------------------------------------------------------------------------
    //  for vfc/container

    namespace intern
    {
        template<class V, vfc::int32_t N>
        hash_type hashValue(const vfc::TCArray<V, N>& f_val)
        {
            vfc::intern::TGenericContainerHash<vfc::TCArray<V, N> > l_gch;
            return l_gch(f_val);
        }

        template<class V, vfc::int32_t N>
        hash_type hashValue(const vfc::TFixedVector<V, N>& f_val)
        {
            vfc::intern::TGenericContainerHash<vfc::TFixedVector<V, N> > l_gch;
            return l_gch(f_val);
        }

        template<class V, vfc::int32_t N>
        hash_type hashValue(const vfc::TFixedList<V, N>& f_val)
        {
            vfc::intern::TGenericContainerHash<vfc::TFixedList<V, N> > l_gch;
            return l_gch(f_val);
        }

        template<class V, vfc::int32_t N>
        hash_type hashValue(const vfc::TFixedCircularBuffer<V, N>& f_val)
        {
            vfc::intern::TGenericContainerHash<vfc::TFixedCircularBuffer<V, N> > l_gch;
            return l_gch(f_val);
        }
    }

    //=========================================================================
    // THash<TCArray>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class V, vfc::int32_t N>
    struct THash<vfc::TCArray<V, N> >
    {
        typedef vfc::TCArray<V, N>                                  local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            return vfc::intern::hashValue(f_val);
        }
    };

    //=========================================================================
    // THash<TFixedVector>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class V, vfc::int32_t N>
    struct THash<vfc::TFixedVector<V, N> >
    {
        typedef vfc::TFixedVector<V, N>                                  local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            return vfc::intern::hashValue(f_val);
        }
    };


    //=========================================================================
    // THash<TFixedList>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class V, vfc::int32_t N>
    struct THash<vfc::TFixedList<V, N> >
    {
        typedef vfc::TFixedList<V, N>                                  local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            return vfc::intern::hashValue(f_val);
        }
    };

    //=========================================================================
    // THash<TFixedCircularBuffer>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class V, vfc::int32_t N>
    struct THash<vfc::TFixedCircularBuffer<V, N> >
    {
        typedef vfc::TFixedCircularBuffer<V, N>                       local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            return vfc::intern::hashValue(f_val);
        }
    };

}

#endif // VFC_CONTAINER_HASH_SUPPORT_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_container_hash_support.hpp  $
//  Revision 1.1 2014/05/09 16:04:13MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/container/container.pj
//=============================================================================
