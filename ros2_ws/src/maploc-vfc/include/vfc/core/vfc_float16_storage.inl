//=================================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2007 by Robert Bosch GmbH. All rights reserved.
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
///     $Source: vfc_float16_storage.inl $
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

namespace vfc
{    // namespace vfc opened

    /// construct from float32_t
    inline CFloat16Storage::CFloat16Storage(float32_t f_value)
    {
        uint16_t l_val = 0;

        Ufui32_t l_conv_instance;
        l_conv_instance.m_float32 = f_value;
        uint32_t l_x = l_conv_instance.m_uint32;

        if ( (l_x & as<uint32_t>(NO_SIGN_MASK32)) == 0 )   // Signed zero
        {
            l_val = static_cast<uint16_t>(l_x >> as<uint32_t>(SIGN_SHIFT));  // Return the signed zero
        }
        else // Not zero
        {
            const uint32_t l_xs = l_x & as<uint32_t>(SIGN_MASK32);     // Pick off sign bit
            const uint32_t l_xe = l_x & as<uint32_t>(EXP_MASK32);      // Pick off exponent bits
            uint32_t l_xm       = l_x & as<uint32_t>(MANTISSA_MASK32); // Pick off mantissa bits
            const uint16_t l_hs = static_cast<uint16_t>(l_xs >> as<uint32_t>(SIGN_SHIFT));
            if ( l_xe == 0 )
            {  // Denormal will underflow, return a signed zero
                l_val = l_hs;
            }
            else if ( l_xe == as<uint32_t>(EXP_MASK32) )
            {  // Inf or NaN (all the exponent bits are set)
                if ( l_xm == 0u )
                { // If mantissa is zero ...
                    // Signed Inf
                    l_val = (l_hs | as<uint16_t>(INF_BITS16));
                }
                else
                {
                    l_val = (l_hs | as<uint16_t>(NAN16)); // NaN, only 1st mantissa bit set
                }
            }
            else
            { // Normalized number
                const uint16_t l_hs = static_cast<uint16_t>(l_xs >> as<uint32_t>(SIGN_SHIFT)); // Sign bit
                // Exponent unbias the single, then bias the halfp
                const int16_t l_hes = (static_cast<int16_t>(l_xe >> as<uint16_t>(EXP_SHIFT32))
                                       - as<uint16_t>(EXP_BIAS_DIFF));
                uint16_t l_he; // exponent
                uint16_t l_hm; // mantissa
                if ( l_hes >= as<int16_t>(MAX_EXP16) )
                {  // Overflow
                    l_val = (l_hs | as<uint16_t>(INF_BITS16)); // Signed Inf
                }
                else if ( l_hes <= 0 )
                {  // Underflow
                    if ( l_hes < - as<int16_t>(EXP_SHIFT16) )
                    {  // Mantissa shifted all the way off & no rounding possibility
                        l_hm = static_cast<uint16_t>(0u);  // Set mantissa to zero
                    }
                    else
                    {
                        l_xm |= as<uint32_t>(IMPLICIT_MASK32);  // Add the hidden leading bit
                        l_hm = static_cast<uint16_t>(l_xm >> (as<int16_t>(EXP_SHIFT_DIFF) + 1 - l_hes)); // Mantissa
                        // Check for rounding
                        if ( (l_xm >> (as<int16_t>(EXP_SHIFT_DIFF) - l_hes)) & 0x00000001u )
                        {
                            // Round, might overflow into exp bit, but this is OK
                            l_hm += static_cast<uint16_t>(1u);
                        }
                    }
                    // Combine sign bit and mantissa bits, biased exponent is zero
                    l_val = (l_hs | l_hm);
                }
                else
                {
                    l_he = static_cast<uint16_t>(l_hes << as<uint16_t>(EXP_SHIFT16)); // Exponent
                    l_hm = static_cast<uint16_t>(l_xm >> (as<uint16_t>(EXP_SHIFT_DIFF))); // Mantissa
                    if ( l_xm & as<uint32_t>(ROUND_BIT32) ) // Check for rounding
                    {
                        // Round, might overflow to inf, this is OK
                        l_val = (l_hs | l_he | l_hm) + static_cast<uint16_t>(1u);
                    }
                    else
                    {
                        l_val = (l_hs | l_he | l_hm);  // No rounding
                    }
                }
            }
        }
        m_value = l_val;
    }

    inline CFloat16Storage
    CFloat16Storage::fromUint16(const uint16_t& f_value)
    {
        CFloat16Storage l_res;
        l_res.m_value = f_value;
        return l_res;
    }

    inline
    CFloat16Storage::operator float32_t() const
    {
        uint32_t l_x;
        uint16_t l_h = m_value;

        if ( (l_h & as<uint16_t>(NO_SIGN_MASK16)) == 0 )
        {  // Signed zero
            l_x = static_cast<uint32_t>(l_h) << as<uint16_t>(SIGN_SHIFT);  // Return the signed zero
        }
        else
        { // Not zero
            const uint16_t l_hs = l_h & as<uint16_t>(SIGN_MASK16);  // Pick off sign bit
            const uint16_t l_he = l_h & as<uint16_t>(EXP_MASK16);      // Pick off exponent bits
            uint16_t l_hm       = l_h & as<uint16_t>(MANTISSA_MASK16); // Pick off mantissa bits

            if ( l_he == 0 )
            {  // Denormal will convert to normalized
                int32_t l_e = -1; // The following loop figures out how much extra to adjust the exponent
                do {
                    ++l_e;
                    l_hm <<= 1;
                } while( (l_hm & as<uint16_t>(IMPLICIT_MASK16)) == 0 ); // Shift until leading bit overflows into exponent bit
                const uint32_t l_xs  = static_cast<uint16_t>(l_hs) << as<uint16_t>(SIGN_SHIFT); // Sign bit
                // Exponent unbias the halfp, then bias the single
                const uint16_t l_xes = (static_cast<int32_t>(l_he >> as<uint16_t>(EXP_SHIFT16))
                                        + as<uint16_t>(EXP_BIAS_DIFF) - l_e);
                const uint32_t l_xe  = static_cast<uint32_t>(l_xes << as<uint16_t>(EXP_SHIFT32)); // Exponent
                const uint32_t l_xm  = (static_cast<uint16_t>(l_hm & as<uint16_t>(MANTISSA_MASK16))
                                        << as<uint16_t>(EXP_SHIFT_DIFF)); // Mantissa
                l_x = (l_xs | l_xe | l_xm); // Combine sign bit, exponent bits, and mantissa bits
            }
            else if ( l_he == as<uint32_t>(EXP_MASK16) )
            {  // Inf or NaN (all the exponent bits are set)
                const uint32_t l_xs  = static_cast<uint16_t>(l_hs) << as<uint16_t>(SIGN_SHIFT); // Sign bit
                if ( l_hm == 0 )
                { // If mantissa is zero ...
                    l_x = (l_xs | as<uint32_t>(EXP_MASK32)); // Signed Inf
                }
                else
                {
                    l_x = (l_xs | as<uint32_t>(NAN32)); // NaN, only 1st mantissa bit set
                }
            }
            else
            { // Normalized number
                const uint32_t l_xs  = static_cast<uint32_t>(l_hs) << as<uint16_t>(SIGN_SHIFT); // Sign bit
                // Exponent unbias the halfp, then bias the single
                const int16_t l_xes = static_cast<int32_t>(l_he >> as<uint16_t>(EXP_SHIFT16)) + as<uint16_t>(EXP_BIAS_DIFF);
                const uint32_t l_xe  = static_cast<uint32_t>(l_xes << as<uint16_t>(EXP_SHIFT32)); // Exponent
                const uint32_t l_xm  = static_cast<uint32_t>(l_hm) << as<uint16_t>(EXP_SHIFT_DIFF); // Mantissa
                l_x   = (l_xs | l_xe | l_xm); // Combine sign bit, exponent bits, and mantissa bits
            }
        }

        Ufui32_t l_conv_instance;
        l_conv_instance.m_uint32 = l_x;

        return l_conv_instance.m_float32;
    }

    inline bool
    CFloat16Storage::isNegative() const
    {
        return ((m_value & as<uint16_t>(SIGN_MASK16)) != 0);
    }

}    // namespace vfc closed

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_float16_storage.inl  $
//=============================================================================
