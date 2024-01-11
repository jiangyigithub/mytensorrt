//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2006 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy, use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/core
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
//        Name: zvh2hi
//  Department: CR/AEM
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_types.hpp $
///     $Revision: 1.21 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2011/08/29 10:10:06MESZ $
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

#ifndef VFC_TYPES_HPP_INCLUDED
#define VFC_TYPES_HPP_INCLUDED

// vfc includes
#include "vfc/core/vfc_config.hpp"

//=========================================================================
// DOXYGEN ADDTOGROUP vfc_group_core_types
//-------------------------------------------------------------------------
//! @brief Typedef's based on the 1999 C Standard header <stdint.h>, wrapped in namespace vfc.
//! @par Reference:
//! http://abt-prolib.de.bosch.com:8080/pkit/process/resource/download.do?id=743019&type=Resource
//!
//! The header vfc_types.hpp provides the typedef's useful for writing
//! portable code that requires certain integer widths.
//! Use it in preference to <stdint.h> for enhanced portability
//!
//! Following typedefs are provided by vfc:
//!
//! - int8_t
//! - uint8_t
//! - int16_t
//! - uint16_t
//! - int32_t
//! - uint32_t
//! - int64_t (optional)
//! - uint64_t (optional)
//! .
//!
//! The typedef int@#_t, with @# replaced by the width, designates a signed
//! integer type of exactly @# bits.\n
//! Similarly, the typedef uint@#_t designates and unsigned integer type of
//! exactly @# bits.
//!
//! @code
//! vfc::int8_t myInt_i8; // denotes an 8-bit signed integer type.
//! vfc::uint8_t myUInt_u8; // denotes an 8-bit unsigned integer type.
//! @endcode
//!
//! The 64-bit types required by the C standard are not required in the vfc
//! header, and may not be supplied in all implementations,
//! because long long is not [yet] included in the C++ standard. If the 64bit
//! types are not available on your platform, the macro VFC_NO_INT64 is
//! defined by vfc, see also @ref vfc_group_core_config_platform .
//!
//! This implementation may #include the compiler supplied <stdint.h>,
//! if present.
//! @par Error handling:
//! In case of a manual creation of the basic types, a compiler error is given in case
//! any of the types of size 8,16 or 32 could not be identified.
//! @par Timing/Scheduling constraints:
//! On 32 bit architectures, all types up to 32 bits are supposed to be thread safe,
//! when properly aligned. 64 bit types (int and double) are not thread safe.
//! $Source: vfc_types.hpp $
//! @author zvh2hi
//! @defgroup vfc_group_core_types_c99 C99 Integral Types
//! @ingroup vfc_group_core_types
//=========================================================================

///////////////////////////////////////
// start with integral types 8..32bit
///////////////////////////////////////

// check for stdint first
#if defined VFC_HAS_STDINT_H
#   include <stdint.h>
    namespace vfc
    {    // open namespace vfc

        // signed first
        using ::int8_t;
        using ::int16_t;
        using ::int32_t;

        // unsigned
        using ::uint8_t;
        using ::uint16_t;
        using ::uint32_t;

    }    // close namespace vfc

// no stdint, so check for visualc
#elif defined VFC_MSVC
    namespace vfc
    {    // open namespace vfc

        // signed first
        typedef __int8  int8_t;
        typedef __int16 int16_t;
        typedef __int32 int32_t;

        // unsigned
        typedef unsigned __int8     uint8_t;
        typedef unsigned __int16    uint16_t;
        typedef unsigned __int32    uint32_t;

    }    // close namespace vfc

// ok, do it manually
#else
#    include <climits>
    namespace vfc
    {    // open namespace vfc

//    start with 8bit
#    if UCHAR_MAX == 0xff
        typedef signed char     int8_t;
        typedef unsigned char   uint8_t;
#    else
#        error cannot set integral 8bit type
#    endif

//    16bit goes second
#    if USHRT_MAX == 0xffff
        typedef short           int16_t;
        typedef unsigned short  uint16_t;
#    else
#        error cannot set integral 16bit type
#    endif

//    32bit
#    if ULONG_MAX == 0xffffffff
        typedef long            int32_t;
        typedef unsigned long   uint32_t;
#    elif UINT_MAX == 0xffffffff
        typedef int             int32_t;
        typedef unsigned int    uint32_t;
#    else
#        error cannot set integral 32bit type
#    endif
    }    // close namespace vfc
#endif

///////////////////////////////////////
// now 64bit (optional)
///////////////////////////////////////

#include <climits>

// first one is easy - check for visualc __int64
#if defined VFC_HAS_MS_INT64
    namespace vfc
    {    // open namespace vfc

        typedef __int64             int64_t;
        typedef unsigned __int64    uint64_t;

    }    // close namespace vfc

// check existence of long long
#elif defined VFC_HAS_LONG_LONG
//    long long is present, now check if limits are right
#    if       (defined(ULLONG_MAX) && ULLONG_MAX == 18446744073709551615ULL)\
        ||    (defined(_MSL_SIZEOF_LONG_LONG) && (_MSL_SIZEOF_LONG_LONG == 8))\
        ||    (defined(ULONG_LONG_MAX) && ULONG_LONG_MAX == 18446744073709551615ULL)\
        ||    (defined(ULONGLONG_MAX) && ULONGLONG_MAX == 18446744073709551615ULL)
        namespace vfc
        {    // open namespace vfc

            // PRQA S 47, 48 ++
            typedef long long           int64_t;
            typedef unsigned long long  uint64_t;
            // PRQA S 47, 48 --


        }    // close namespace vfc
//    limits not present or they seem to have the wrong size - so flag no int64 support
#    else
#        define VFC_NO_INT64
#    endif

// last try, is long type big enough?
#elif ULONG_MAX == 18446744073709551615u // 2^64-1
    namespace vfc
    {    // open namespace vfc

        typedef long            int64_t;
        typedef unsigned long   uint64_t;

    }    // close namespace vfc

// nothing worked, so give up and flag no int64 support
#else
     //! Defined if there are no 64-bit integral types: int64_t, uint64_t etc.
#    define VFC_NO_INT64
#endif

///////////////////////////////////////
// floating point - assuming IEEE
///////////////////////////////////////

namespace    vfc
{    // open namespace vfc

    //! single-precision (32bit) floating point type.
    //! @ingroup vfc_group_core_types
    typedef    float    float32_t;

    //! double-precision (64bit) floating point type.
    //! @ingroup vfc_group_core_types
    typedef    double   float64_t;

}    // close namespace vfc

///////////////////////////////////////
// remaining types
///////////////////////////////////////

#include <stddef.h>
//! this is vfc namespace
namespace    vfc
{    // open namespace vfc

    // promotes size_t to vfc namespace.
    // we can't document using declarations with doxygen!
    using ::size_t;

    // promotes ptrdiff_t to vfc namespace.
    // we can't document using declarations with doxygen!
    using ::ptrdiff_t;

    //! don't assume anything for char type, except sizeof(char_t)==1.
    //! use it only for string literals and compatibility with libc.
    //! @ingroup vfc_group_core_types
    typedef char    char_t;

    // !! no boolean typedef, because boolean is a fundamental type in c++ !!
    // typedef bool bool_t

    //! true type for template metaprogramming.
    //! @ingroup vfc_group_core_types vfc_group_core_metaprogram
    // identifier is intentionally not conform to coding rules
    struct true_t   {    enum { value = 1};};

    //! false type for template metaprogramming.
    //! @ingroup vfc_group_core_types vfc_group_core_metaprogram
    // identifier is intentionally not conform to coding rules
    struct false_t  {    enum { value = 0};};

}    // close namespace vfc

///////////////////////////////////////
// max aligned
///////////////////////////////////////

// PRQA S 2427,2428 ++
namespace    vfc
{    // open namespace vfc

    //-------------------------------------------------------------------------
    // conditional doxygen documentation
    //! @cond VFC_DOXY_INTERN
    //-------------------------------------------------------------------------

        namespace intern
        {   // open namespace vfc::intern

            // pointer to function
            typedef    int(*max_aligned_ptr_to_func_t)(void);

            // pointer to member
            class    CMaxAlignedDummy;    // any class decl will do it
            typedef int(CMaxAlignedDummy::*max_aligned_ptr_to_member_t);
            typedef int(CMaxAlignedDummy::*max_aligned_ptr_to_member_func_t)(void);

        }   // close namespace vfc::intern

    //-------------------------------------------------------------------------
    //! @endcond
    // of VFC_DOXY_INTERN
    //-------------------------------------------------------------------------

    //=========================================================================
    //  max_aligned_t
    //-------------------------------------------------------------------------
    //! union which is properly aligned for all built-in types.
    //! @ingroup vfc_group_core_types
    //=========================================================================

    union max_aligned_t
    {
        private:
        // we are using built-in types here, for avoiding possible conflicts

        // ptr to member func
            intern::max_aligned_ptr_to_member_func_t    m_ptr_to_member_func_p;
        // ptr to member
            intern::max_aligned_ptr_to_member_t         m_ptr_to_member_p;
        // ptr to func
            intern::max_aligned_ptr_to_func_t           m_ptr_to_func_p;

        // ptr to void
        void* m_void_p;

        // integrals
        bool    m_bool;
        char    m_char;
        short   m_short;
        int     m_int;
        long    m_long;

        // really long integrals
#        ifdef VFC_HAS_LONG_LONG
            // PRQA S 47 ++
            long long   m_longlong;
            // PRQA S 47 --
#        endif

#        ifdef VFC_HAS_MS_INT64
            __int64     m_int64;
#        endif

        // floating
        float       m_float;
        double      m_double;
        long double m_longdouble;
    };

}    // close namespace vfc
// PRQA S 2427,2428 --

#endif //VFC_TYPES_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_types.hpp  $
//  Revision 1.21 2011/08/29 10:10:06MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - corrected link to coding rules (mantis0003647)
//  Revision 1.20 2011/08/29 09:05:26MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - vfc_types.hpp: enhance the sw spec header (mantis0003647)
//  Revision 1.19 2009/02/02 15:13:32MEZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Removal of QAC++ warnings.
//  (Mantis : 0002506)
//  Revision 1.18 2007/08/02 19:17:54IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added conditional doxygen documentation generation of vfc::intern (mantis1758)
//  Revision 1.17 2007/07/23 09:36:48CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - doxygen grouping (mantis1744)
//  - added documentation
//  Revision 1.16 2007/06/15 15:15:57CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - made the max_aligned_t union members private (mantis 1685)
//  Revision 1.15 2007/06/07 14:19:16CEST Koenig Matthias (CR/AEM5) (kon3hi)
//  added gcc4.2 support
//  Revision 1.14 2007/05/22 13:49:02CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - reordered max_aligned_t members (mantis1268)
//  Revision 1.13 2007/03/30 14:36:22CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced <cstdint> with <stdint.h> (mantis1531)
//  - replaced <cstddef> with <stddef.h> (mantis1534)
//  Revision 1.12 2007/02/13 18:21:52CET Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  check for 8byte size of "long long" on non-C99 MSL , not for limits, to enable 64bit types (mantis1422)
//  Revision 1.11 2007/02/09 19:27:13CET Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  additional test for unsigned 64bit integer maximum on non-C99 Codewarrior MSL (mantis1422)
//  Revision 1.10 2006/11/24 09:51:48CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - reordered max_aligned_t members (mantis1268)
//  Revision 1.9 2006/11/16 14:41:15CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.8 2006/10/30 09:27:01CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - improved gcc compatibility (mantis1234)
//  - replaced header/footer templates
//  Revision 1.7 2005/10/28 10:26:11CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/core.pj
//  Revision 1.6 2005/10/06 16:51:57CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/vfc.pj
//  Revision 1.5 2005/04/06 15:58:33CEST zvh2hi
//  added max_aligned type
//  Revision 1.4 2005/04/06 14:29:09CEST zvh2hi
//  added docu
//  Revision 1.3 2005/04/05 17:12:24CEST zvh2hi
//  moved lost comment
//  Revision 1.2 2005/04/05 17:10:00CEST zvh2hi
//  added long long size check
//=============================================================================
