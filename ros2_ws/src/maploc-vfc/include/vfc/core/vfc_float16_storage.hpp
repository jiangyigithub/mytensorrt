//=================================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2017 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy, use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=================================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/core
//          Synopsis: storage class for storing 16bit float data
//  Target system(s): c++ compliant
//       Compiler(s): c++ compliant
//=================================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=================================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: hrp2lr
//  Department: CC-DA/ESI5
//=================================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_float16_storage.hpp $
///     $Revision: $
///     $Author: Hrenka Peter (CC-DA/ESI5) (HRP2LR) $
///     $Date: $
///     $Locker:  $
///     $Name:  $
///     $State:  $
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
//=================================================================================

#ifndef VFC_FLOAT16_STORAGE_HPP_INCLUDED
#define VFC_FLOAT16_STORAGE_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"    // used for fundamental types
#include "vfc/core/vfc_config.hpp"   // used for predefines for compiler-checks

namespace vfc
{    // namespace vfc opened

    //=============================================================================
    // E X P O R T E D   C L A S S
    //
    // CFloat16Storage
    //
    //-----------------------------------------------------------------------------
    //! @brief storage class for low-precision float values, no overloaded operators
    //!
    //! Most hardware with floating point support does support float32_t but sometimes
    //! that amount of precision is not needed for a specific application.
    //! Several architectures support the 16bit half-float format which
    //! has 10 mantissa, 5 exponent and 1 sign bit.
    //! Using CFloat16Storage the stored half-float values can be converted to
    //! float32_t values and back.
    //!
    //! @see ISO/IEC/IEEE 60559, Edition 1.0, 2011-06, Chapter 3.6, Table 3.5
    //=============================================================================

    class CFloat16Storage
    {
    public:

        /// default constructor
        CFloat16Storage() : m_value(0u)
        {
            // intentionally left blank
        }

        /// construct from float32_t, allow implicit construction
        CFloat16Storage(float32_t f_value);

        /// contructor method from integer
        static CFloat16Storage fromUint16(const uint16_t& f_value);

        /// conversion operator to float32_t
        operator float32_t() const;

        /// check if stored number is negative, mostly for tests
        bool isNegative() const;

    private:

        enum EConstant {
            NO_SIGN_MASK32    = 0x7FFFFFFFu,  //! mask for non-sign bits for float32
            NO_SIGN_MASK16    = 0x7FFFu,      //! mask for non-sign bits for float16
            SIGN_MASK32       = 0x80000000u,  //! mask for sign bit for float32
            SIGN_MASK16       = 0x8000u,      //! mask for sign bit for float16
            EXP_MASK32        = 0x7F800000u,  //! mask for exponent bits for float32
            EXP_MASK16        = 0x7C00u,      //! mask for exponent bits for float16
            MANTISSA_MASK32   = 0x007FFFFFu,  //! mask for mantissa bits for float32
            MANTISSA_MASK16   = 0x03FFu,      //! mask for mantissa bits for float16
            IMPLICIT_MASK32   = 0x00800000u,  //! implicit mantissa bit for float32
            IMPLICIT_MASK16   = 0x0400u,      //! implicit mantissa bit for float16
            INF_BITS16        = 0x7C00u,      //! signless bitpattern for infinity
            NAN32             = 0x7FC00000u,  //! NaN mask for float32
            NAN16             = 0x7E00u,      //! NaN mask for float16
            SIGN_SHIFT        = 0x10u,        //! number of bits to shift sign bit from float32 to float16
            MAX_EXP16         = 0x1Fu,        //! maximum exponent for float16
            EXP_SHIFT32  = 0x17u,             //! number of bits exponents in float32 are shifted
            EXP_SHIFT16  = 0x0Au,             //! number of bits exponents in float16 are shifted
            EXP_SHIFT_DIFF = (EXP_SHIFT32 - EXP_SHIFT16), //! difference in shift distance from float32 to float16
            EXP_BIAS32 = 0x7Fu,               //! bias of exponent value in float32
            EXP_BIAS16 = 0x0Fu,               //! bias of exponent value in float16
            EXP_BIAS_DIFF = (EXP_BIAS32 - EXP_BIAS16), //! difference in bias from float32 to float16
            ROUND_BIT32 = 0x00001000u,        //! bit which indicates rounding when converting float32 to float16
        };

        //! union for alias-safe conversions between float32_t and uint32_t
        union Ufui32_t
        {
            float32_t m_float32;
            uint32_t  m_uint32;
        };

        //! templated explicit conversion of enum
        template<typename T>
        static T as(const EConstant f_const)
        {
            return static_cast<T>(f_const);
        }

        uint16_t m_value;
    }; // class CFloat16Storage


#ifdef VFC_NATIVE_FLOAT16_TYPE
    /// wrapper to align interface to CFloat16Storage
    /// disallow arithmetic operations
    class CF16NativeStorage
    {
    public:
        inline CF16NativeStorage() : m_value(0.0f)
        {
            // intentionally left blank
        }

        inline CF16NativeStorage(float32_t f_value) : m_value(f_value)
        {
            // intentionally left blank
        }

        inline operator float32_t () const
        {
            return static_cast<float32_t>(m_value);
        }
    private:
        VFC_NATIVE_FLOAT16_TYPE m_value;
    };

    typedef CF16NativeStorage float16_storage_t;
#elif defined(VFC_TRICORE_DETECTED) && defined(VFC_COMPILER_GHS)
    /// workaround until compiler supports the real float type
    class CF16NativeStorage
    {
    public:
        inline CF16NativeStorage(float32_t f_value = 0.0f)
        {
            asm ("ftohp %1, %0" : "=r" (m_value) : "r"  (f_value));
        }

        inline operator float32_t () const
        {
            float32_t l_result;
            asm ("hptof %1, %0" : "=r" (l_result) : "r"  (m_value));
            return l_result;
        }
    private:
        uint16_t m_value;
    };
    typedef CF16NativeStorage float16_storage_t;
#else
    typedef CFloat16Storage float16_storage_t;
#endif

} // namespace vfc closed


#include "vfc/core/vfc_float16_storage.inl"

#endif //VFC_FLOAT16_STORAGE_HPP_INCLUDED


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_float16_storage.hpp  $
//=============================================================================
