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
///     $Source: vfc_hash.hpp $
///     $Revision: 1.2 $
///     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
///     $Date: 2015/11/05 12:56:26MEZ $
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

#ifndef VFC_HASH_HPP_INCLUDED
#define VFC_HASH_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_metaprog.hpp"

namespace vfc
{

    //=========================================================================
    //  DOYGEN DEFGROUP vfc_group_core_algorithms_hash BEGIN
    //-------------------------------------------------------------------------
    /// @defgroup vfc_group_core_algorithms_hash Hash functions
    /// @ingroup vfc_group_core_algorithms
    /// @brief hashing algorithms.
    /// see 
    //    Methods for Identifying Versioned and Plagiarised Documents (2003)
    //        by Timothy C. Hoad , Justin Zobel , Timothy C. Hoad Justin Zobel
    //        Venue: Journal of the American Society for Information Science and Technology
    //
    //    M.V. Ramakrishna and J. Zobel. Performance in practice of string hashing functions. In
    //        Proc. Int. Conf. on Database Systems for Advanced Applications, pages 215-223, Melbourne,
    //        Australia, April 1997.
    /// @{
    //-------------------------------------------------------------------------

    //=========================================================================
    // hash_type
    //-------------------------------------------------------------------------
    //! Declaration of the hash type. The implementation does only work with
    //! a 32-bit hash type.
    //=========================================================================
    typedef vfc::uint32_t hash_type;

    //=========================================================================
    // serialHash
    //-------------------------------------------------------------------------
    //! Declaration of the serialHash function.
    //! This function computes contiguous hash values by taking the previous
    //! hash value as seed in the first argument, and the object, for which the
    //! next hash shall be computed, as second argument. Accumulate the previous
    //! hash value with the new one.
    //! @param f_prevhash       The previous hash value to accumulate with the next hash
    //! @param f_nextObjToHash  The next object for hash computing
    //=========================================================================
    template<class ValueType>
    void serialHash(vfc::hash_type& f_prevhash, const ValueType& f_nextObjToHash);

    namespace intern
    {

      // Declare the hash_value function overloads for different types:

      template <typename ValueType>
      inline
      typename vfc::TEnableIf<vfc::TIsSameType<vfc::uint8_t,  ValueType>::value ||
                              vfc::TIsSameType<vfc::int8_t,   ValueType>::value ||
                              vfc::TIsSameType<vfc::uint16_t, ValueType>::value ||
                              vfc::TIsSameType<vfc::int16_t,  ValueType>::value ||
                              vfc::TIsSameType<vfc::uint32_t, ValueType>::value ||
                              vfc::TIsSameType<vfc::int32_t,  ValueType>::value ,
                              vfc::hash_type>::type hashValue(ValueType f_val)
      {
          return static_cast<vfc::hash_type>(f_val);
      }

      template <typename ValueType>
      inline
      typename vfc::TEnableIf<vfc::TIsSameType<bool, ValueType>::value,
                              vfc::hash_type>::type hashValue(ValueType f_val)
      {
          return static_cast<vfc::hash_type>(f_val ? 1 : 0);
      }


#if defined (VFC_HAS_LONG_LONG) || !defined (VFC_NO_FLOAT)
        template <typename ValueType>
        inline
            vfc::hash_type hash64Bit(const ValueType& f_val)
        {
            return (hashValue(static_cast<vfc::hash_type>(f_val & 0xFFFFFFFFUL))
                ^ hashValue(static_cast<vfc::hash_type>(f_val >> 32)));
        }
#endif


#ifdef VFC_HAS_LONG_LONG
        template <typename ValueType>
        inline
        typename vfc::TEnableIf<vfc::TIsSameType<vfc::uint64_t,  ValueType>::value ||
                                vfc::TIsSameType<vfc::int64_t,   ValueType>::value,
                                vfc::hash_type>::type hashValue(ValueType f_val)
        {
            return hash64Bit(f_val);
        }
#endif

#ifndef VFC_NO_FLOAT
        template <typename ValueType>
        inline
        typename vfc::TEnableIf<vfc::TIsSameType<vfc::float32_t,  ValueType>::value,
                                vfc::hash_type>::type hashValue(ValueType f_val)
        {
            return *reinterpret_cast<vfc::hash_type*>(&f_val); 
        }

        template <typename ValueType>
        inline
        typename vfc::TEnableIf<vfc::TIsSameType<vfc::float64_t,  ValueType>::value,
                                vfc::hash_type>::type hashValue(ValueType f_val)
        {
            const vfc::uint64_t l_rawbits = *reinterpret_cast<vfc::uint64_t*>(&f_val);
            return hash64Bit(l_rawbits);
        }
#endif


        //=========================================================================
        // TGenericContainerHash<>
        //-------------------------------------------------------------------------
        //! Generic declaration of the hash function object, which iterates over
        //! all items of a container.
        //=========================================================================
        template<class ValueType>
        struct TGenericContainerHash
        {
            vfc::hash_type operator()(const ValueType& f_val) const;
        };

    } // namespace intern closed

    //=========================================================================
    // THash<>
    //-------------------------------------------------------------------------
    //! @struct vfc::THash<>
    //! Generic declaration of the hash function object.
    //! 
    //! Example of a user struct and the corresponding THash<> specialization:
    //! 
    //! \code
    //! struct CUserdata
    //! {
    //!     vfc::uint32_t                   m_member1;
    //!     vfc::TCArray<CTestdata2, 10>    m_member2;
    //!     vfc::uint32_t                   m_member3;
    //! };
    //!
    //! //! CUserdata hasher
    //! template<>
    //! struct THash<CUserdata>
    //! {
    //!     typedef CUserdata                             local_hash_type;
    //! 
    //!     vfc::hash_type operator()(const local_hash_type& f_val) const
    //!     {
    //!         vfc::hash_type l_seed = 0;
    //!         serialHash(l_seed, f_val.m_member1);
    //!         serialHash(l_seed, f_val.m_member2);
    //!         serialHash(l_seed, f_val.m_member3);
    //!         return l_seed;
    //!     }
    //! };
    //! \endcode
    //=========================================================================
    template<typename ValueType>
    struct THash
    {
        /// My own hash object type.
        typedef ValueType                               local_hash_type;

        inline vfc::hash_type operator()(const local_hash_type& f_val) const
        {
            return vfc::intern::hashValue(f_val);
        }
    };


    //=============================================================================
    //  VFC_HASH_SPECIAL_IMPL
    //-----------------------------------------------------------------------------
    //! helper macro for defining a THash<> specialized class.
    //=============================================================================
#define    VFC_HASH_SPECIAL_IMPL(TYPE)\
    template <>\
    struct THash< TYPE > \
    { inline hash_type operator()(const TYPE& f_val) const { return vfc::intern::hashValue(f_val); } };


    //=========================================================================
    //  THash<> specializations
    //-------------------------------------------------------------------------
    //! @brief Create the specializations for each basic type.
    //=========================================================================
    VFC_HASH_SPECIAL_IMPL(bool);
    VFC_HASH_SPECIAL_IMPL(vfc::uint8_t);
    VFC_HASH_SPECIAL_IMPL(vfc::int8_t);
    VFC_HASH_SPECIAL_IMPL(vfc::uint16_t);
    VFC_HASH_SPECIAL_IMPL(vfc::int16_t);
    VFC_HASH_SPECIAL_IMPL(vfc::uint32_t);
    VFC_HASH_SPECIAL_IMPL(vfc::int32_t);
#ifdef VFC_HAS_LONG_LONG
    VFC_HASH_SPECIAL_IMPL(vfc::uint64_t);
    VFC_HASH_SPECIAL_IMPL(vfc::int64_t);
#endif
#ifndef VFC_NO_FLOAT
    VFC_HASH_SPECIAL_IMPL(vfc::float32_t);
    VFC_HASH_SPECIAL_IMPL(vfc::float64_t);
#endif

    //=========================================================================
    //  DOYGEN DEFGROUP vfc_group_core_algorithms_hash END
    //-------------------------------------------------------------------------
    /// @}
    //-------------------------------------------------------------------------

}    // namespace vfc closed

#include "vfc/core/vfc_hash.inl"

#endif // VFC_HASH_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_hash.hpp  $
//  Revision 1.2 2015/11/05 12:56:26MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Remove testcase that relies on binary representation of bool (mantis0005087)
//  Revision 1.1 2014/05/09 16:05:02MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/core/core.pj
//=============================================================================
