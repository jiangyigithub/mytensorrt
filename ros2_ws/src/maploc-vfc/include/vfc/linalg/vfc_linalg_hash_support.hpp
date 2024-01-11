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
///     $Source: vfc_linalg_hash_support.hpp $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/05/09 16:03:56MESZ $
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

#ifndef VFC_LINALG_HASH_SUPPORT_HPP_INCLUDED
#define VFC_LINALG_HASH_SUPPORT_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

#include "vfc/core/vfc_hash.hpp"

#include "vfc/linalg/vfc_linalg_vectorn.hpp"
#include "vfc/linalg/vfc_linalg_vector2.hpp"
#include "vfc/linalg/vfc_linalg_vector3.hpp"
#include "vfc/linalg/vfc_linalg_vector4.hpp"

#include "vfc/linalg/vfc_linalg_matrixmn.hpp"
#include "vfc/linalg/vfc_linalg_matrix22.hpp"
#include "vfc/linalg/vfc_linalg_matrix33.hpp"
#include "vfc/linalg/vfc_linalg_matrix44.hpp"


namespace vfc
{

    //=========================================================================
    // vfc types specializations
    //-------------------------------------------------------------------------
    //  for vfc/linalg
    namespace intern
    {
        template<class ValueType, vfc::int32_t RowValue>
        hash_type hashValue(const vfc::linalg::TVectorN<ValueType, RowValue>& f_val)
        {
            vfc::intern::TGenericContainerHash<vfc::linalg::TVectorN<ValueType, RowValue> > l_gch;
            return l_gch(f_val);
        }

        template<class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
        hash_type hashValue(const vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>& f_val)
        {
            vfc::intern::TGenericContainerHash<vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue> > l_gch;
            return l_gch(f_val);
        }

    }

    //=========================================================================
    // THash<TVectorN>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class ValueType, vfc::int32_t RowValue>
    struct THash<vfc::linalg::TVectorN<ValueType, RowValue> >
    {
        typedef vfc::linalg::TVectorN<ValueType, RowValue>            local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            return vfc::intern::hashValue(f_val);
        }
    };

    //=========================================================================
    // THash<TVector2>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class ValueType>
    struct THash<vfc::linalg::TVector2<ValueType> >
    {
        typedef vfc::linalg::TVector2<ValueType>            local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            return vfc::intern::hashValue(f_val);
        }
    };
    //=========================================================================
    // THash<TVector3>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class ValueType>
    struct THash<vfc::linalg::TVector3<ValueType> >
    {
        typedef vfc::linalg::TVector3<ValueType>            local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            return vfc::intern::hashValue(f_val);
        }
    };
    //=========================================================================
    // THash<TVector4>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class ValueType>
    struct THash<vfc::linalg::TVector4<ValueType> >
    {
        typedef vfc::linalg::TVector4<ValueType>            local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            return vfc::intern::hashValue(f_val);
        }
    };


    //=========================================================================
    // THash<TMatrixMN>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
    struct THash<vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue> >
    {
        typedef vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>            local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            return vfc::intern::hashValue(f_val);
        }
    };
    //=========================================================================
    // THash<TMatrix22>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class ValueType>
    struct THash<vfc::linalg::TMatrix22<ValueType> >
    {
        typedef vfc::linalg::TMatrix22<ValueType>            local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            return vfc::intern::hashValue(f_val);
        }
    };
    //=========================================================================
    // THash<TMatrix33>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class ValueType>
    struct THash<vfc::linalg::TMatrix33<ValueType> >
    {
        typedef vfc::linalg::TMatrix33<ValueType>            local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            return vfc::intern::hashValue(f_val);
        }
    };
    //=========================================================================
    // THash<TMatrix44>
    //-------------------------------------------------------------------------
    //! Specialisation of a hash function object.
    //=========================================================================
    template<class ValueType>
    struct THash<vfc::linalg::TMatrix44<ValueType> >
    {
        typedef vfc::linalg::TMatrix44<ValueType>            local_hash_type;

        vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            return vfc::intern::hashValue(f_val);
        }
    };

}

#endif // VFC_LINALG_HASH_SUPPORT_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_hash_support.hpp  $
//  Revision 1.1 2014/05/09 16:03:56MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/linalg/linalg.pj
//=============================================================================
