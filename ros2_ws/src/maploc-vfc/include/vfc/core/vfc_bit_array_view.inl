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
//       Projectname:
//  Target system(s):
//       Compiler(s): VS7.10
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
///     $Source: vfc_bit_array_view.inl $
///     $Revision: 1.6 $
///     $Author: Jaeger Thomas (CC/EPV2) (JAT2HI) $
///     $Date: 2012/12/18 08:27:30MEZ $
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

#include "vfc/core/vfc_assert.hpp"    // using VFC_ASSERT
#include "vfc/core/vfc_metaprog.hpp"  // using vfc::TIf
#include "vfc/core/vfc_math.hpp"      // using vfc::isNegative
#include "vfc/core/vfc_endianess.hpp" // using convertSystemToBigEndian, convertBigEndianToSystem
                                      // convertSystemToLittleEndian, convertSystemToLittleEndian
#include "vfc/core/vfc_types.hpp"     // used for uint8_t


template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
inline
ValueType
vfc::TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType>
::getValue(const vfc::uint8_t* f_data_p)
{
    // determines if ValueType and memory reads are 32bit or 64bit
    typedef typename vfc::TInt2Boolean<READ_MORE_THAN_32BIT>::type l_needs_64bit_t;
    // determines if bits span more than 4/8 bytes (i.e. 32bit starting and ending in the middle of a byte)
    typedef typename vfc::TInt2Boolean<READ_EXTRA_BYTE>::type l_read_extra_byte_t;
    return getValueIntern(f_data_p, l_needs_64bit_t(), l_read_extra_byte_t());
}

template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
inline
void
vfc::TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType>
::setValue(ValueType f_value, vfc::uint8_t* f_data_p)
{
    // if f_value is positive or ValueType is unsigned, all bits from LengthValue upwards must be zero,
    // if f_value is negative and ValueType is signed, all bits from LengthValue up must be set
    // otherwise, the value cannot fit into LengthValue bits
//deactivated muk2lr    VFC_ASSERT((0 == (f_value & ~getMask()) || (0 == ~(f_value | getMask())))); 

    typedef typename vfc::TInt2Boolean<READ_MORE_THAN_32BIT>::type l_needs_64bit_t;
    typedef typename vfc::TInt2Boolean<READ_EXTRA_BYTE>::type l_read_extra_byte_t;
    setValueIntern(f_value, f_data_p, l_needs_64bit_t(), l_read_extra_byte_t());
}

//-----------------------------------------------------------------------------
// specializations of getValue for different reading widths
//-----------------------------------------------------------------------------

//! reads 4 bytes and puts them in a 32bit or less ValueType'd result
template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
inline
ValueType
vfc::TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType>
::getValueIntern(const vfc::uint8_t* f_data_p, vfc::false_t, vfc::false_t)
{
    vfc::uint32_t l_tmpVal_u32 = EndianessPolicyType::convToSystemOrder(
        *reinterpret_cast<const vfc::uint32_t*>(f_data_p + FIRST_BYTE_TO_READ));
    
    return shiftAndSignExtend(f_data_p, l_tmpVal_u32, typename vfc::TInt2Boolean<IS_UNSIGNED>::type());
}

//! must read 5 bytes and put them in a 32bit ValueType'd result
template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
inline
ValueType
vfc::TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType>
::getValueIntern(const vfc::uint8_t* f_data_p, vfc::false_t, vfc::true_t)
{
    // read lower bits as 4 bytes
    vfc::uint32_t l_retVal = vfc::TBitArrayView<vfc::uint32_t,    
        StartBitValue, CHAR_BIT * sizeof(vfc::uint32_t) - BITS_SHIFT, EndianessPolicyType>::getValue(f_data_p);
    // merge with upper bits from extra byte
    l_retVal |= static_cast<vfc::uint32_t>(
            vfc::TBitArrayView<
                typename vfc::TIf<vfc::TInt2Boolean<IS_UNSIGNED>::value, vfc::uint8_t, vfc::int8_t>::type,    
                vfc::TIf<vfc::TIsSameType<EndianessPolicyType, CBitArrayViewBigEndianPolicy>::value, 
                    vfc::TInt2Type<CHAR_BIT * FIRST_BYTE_TO_READ>, 
                    vfc::TInt2Type<CHAR_BIT * (FIRST_BYTE_TO_READ + sizeof(vfc::uint32_t))> >::type::value, 
                (LengthValue + BITS_SHIFT) % CHAR_BIT, 
                EndianessPolicyType>::getValue(f_data_p)
        ) << (CHAR_BIT * sizeof(vfc::uint32_t) - BITS_SHIFT);
    return l_retVal;
}

// read up to 8 bytes into 64bit return value
template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
inline
ValueType
vfc::TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType>
::getValueIntern(const vfc::uint8_t* f_data_p, vfc::true_t, vfc::false_t)
{
#if (defined(VFC_COMPILER_ARMRVCT) && defined(VFC_ARM_DETECTED))
    // RVCT 4.X emits ldrd instruction, that causes data abort exceptions on non-8byte aligned accesses 
    // (even with enabled unaligned support for 32bit reads via setting the A bit in the CP15 c1 Control Register)
    // __packed switches to slightly less efficient code, that supports unaligned accesses 
    __packed
#endif
    const vfc::uint64_t* l_unalignedRead_pu64 = reinterpret_cast<const vfc::uint64_t*>(f_data_p + FIRST_BYTE_TO_READ);
#if (defined(VFC_COMPILER_ARMRVCT) && defined(VFC_ARM_DETECTED))
    __packed
#endif
    const vfc::uint64_t  l_tmpVal_u64 = EndianessPolicyType::convToSystemOrder(*l_unalignedRead_pu64);
    return shiftAndSignExtend(f_data_p, l_tmpVal_u64, typename vfc::TInt2Boolean<IS_UNSIGNED>::type());
}

// read 9 bytes into 64bit return value
template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
inline
ValueType
vfc::TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType>
::getValueIntern(const vfc::uint8_t* f_data_p, vfc::true_t, vfc::true_t)
{
#if (defined(VFC_COMPILER_ARMRVCT) && defined(VFC_ARM_DETECTED))
    __packed
#endif
    // read lower bits as 8 bytes
    vfc::uint64_t l_retVal_u64 = vfc::TBitArrayView<vfc::uint64_t,    
        StartBitValue, CHAR_BIT * sizeof(vfc::uint64_t) - BITS_SHIFT, EndianessPolicyType>::getValue(f_data_p);
    // merge with upper bits from extra byte
    l_retVal_u64 |= static_cast<vfc::uint64_t>(
        vfc::TBitArrayView<
            typename vfc::TIf<vfc::TInt2Boolean<IS_UNSIGNED>::value, vfc::uint8_t, vfc::int8_t>::type,
            vfc::TIf<vfc::TIsSameType<EndianessPolicyType, CBitArrayViewBigEndianPolicy>::value, 
                vfc::TInt2Type<CHAR_BIT * FIRST_BYTE_TO_READ>, 
                vfc::TInt2Type<CHAR_BIT * (FIRST_BYTE_TO_READ + sizeof(vfc::uint64_t))> >::type::value, 
            (LengthValue + BITS_SHIFT) % CHAR_BIT, 
            EndianessPolicyType>::getValue(f_data_p)
        ) << (CHAR_BIT * sizeof(vfc::uint64_t) - BITS_SHIFT);
    return l_retVal_u64;
}

//-----------------------------------------------------------------------------
// specializations of setValue for different reading widths
//-----------------------------------------------------------------------------

//! write up to 4 bytes
template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
inline
void vfc::TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType>
::setValueIntern(ValueType f_value, vfc::uint8_t* f_data_p, vfc::false_t, vfc::false_t)
{
    // read old value
    vfc::uint32_t l_tmpVal_u32 = EndianessPolicyType::convToSystemOrder(
        *reinterpret_cast<const vfc::uint32_t*>(f_data_p + FIRST_BYTE_TO_READ));
    // mask out relevant bits
    l_tmpVal_u32 &= ~(static_cast<vfc::uint32_t>(getMask()) << BITS_SHIFT);
    // insert new bits in place
    *reinterpret_cast<vfc::uint32_t*>(f_data_p + FIRST_BYTE_TO_READ) = EndianessPolicyType::convFromSystemOrder(
        l_tmpVal_u32 | ((f_value & getMask()) << BITS_SHIFT) );
}

//! write 5 bytes
template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
inline
void vfc::TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType>
::setValueIntern(ValueType f_value, vfc::uint8_t* f_data_p, vfc::false_t, vfc::true_t)
{
    // set lower 32bits, omit masking f_value and use internal function to avoid assert in setValue() here
    vfc::TBitArrayView<
        vfc::uint32_t, 
        StartBitValue, 
        CHAR_BIT * sizeof(vfc::uint32_t) - BITS_SHIFT, 
        EndianessPolicyType>::setValueIntern(f_value, f_data_p, vfc::false_t(), vfc::false_t());
    // set bit in upper extra byte
    vfc::TBitArrayView<
        vfc::uint8_t,    
        vfc::TIf<vfc::TIsSameType<EndianessPolicyType, CBitArrayViewBigEndianPolicy>::value, 
            vfc::TInt2Type<CHAR_BIT * FIRST_BYTE_TO_READ>, 
            vfc::TInt2Type<CHAR_BIT * (FIRST_BYTE_TO_READ + sizeof(vfc::uint32_t))> >::type::value, 
        (LengthValue + BITS_SHIFT) % CHAR_BIT, 
        EndianessPolicyType>::setValueIntern(static_cast<vfc::uint8_t>(f_value >> (CHAR_BIT * sizeof(vfc::uint32_t) - BITS_SHIFT)), 
                                             f_data_p, 
                                             vfc::false_t(),
                                             vfc::false_t());
}

//! write 6 to 8 bytes
template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
inline
void vfc::TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType>
::setValueIntern(ValueType f_value, vfc::uint8_t* f_data_p, vfc::true_t, vfc::false_t)
{
#if (defined(VFC_COMPILER_ARMRVCT) && defined(VFC_ARM_DETECTED))
    // see comment in getValueIntern for 8byte reads, this applies to strd instructions, too
    __packed
#endif
    vfc::uint64_t* l_unalignedRead_pu64 = reinterpret_cast<vfc::uint64_t*>(f_data_p + FIRST_BYTE_TO_READ);

    vfc::uint64_t l_tmpVal_u64 = EndianessPolicyType::convToSystemOrder(*l_unalignedRead_pu64);
    l_tmpVal_u64 &= ~(static_cast<vfc::uint64_t>(getMask()) << BITS_SHIFT);
    *l_unalignedRead_pu64 = EndianessPolicyType::convFromSystemOrder(
        l_tmpVal_u64 | ((f_value & getMask()) << BITS_SHIFT) );
}

//! write 9 bytes
template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
inline
void vfc::TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType>
::setValueIntern(ValueType f_value, vfc::uint8_t* f_data_p, vfc::true_t, vfc::true_t)
{
    // set lower 64bits, omit masking f_value (will be done inside 8byte setValueIntern())
    // and use internal function to avoid assert in setValue() here
    vfc::TBitArrayView<
        vfc::uint64_t, 
        StartBitValue, 
        CHAR_BIT * sizeof(vfc::uint64_t) - BITS_SHIFT, 
        EndianessPolicyType>::setValueIntern(f_value, f_data_p, vfc::true_t(), vfc::false_t());
    // set bit in upper extra byte
    vfc::TBitArrayView<
        vfc::uint8_t,    
        vfc::TIf<vfc::TIsSameType<EndianessPolicyType, CBitArrayViewBigEndianPolicy>::value, 
            vfc::TInt2Type<CHAR_BIT * FIRST_BYTE_TO_READ>, 
            vfc::TInt2Type<CHAR_BIT * (FIRST_BYTE_TO_READ + sizeof(vfc::uint64_t))> >::type::value, 
        (LengthValue + BITS_SHIFT) % CHAR_BIT, 
        EndianessPolicyType>::setValueIntern(static_cast<vfc::uint8_t>(f_value >> (CHAR_BIT * sizeof(vfc::uint64_t) - BITS_SHIFT)), 
                                             f_data_p,
                                             vfc::false_t(),
                                             vfc::false_t());
}


//-----------------------------------------------------------------------------
// sign helper functions
//-----------------------------------------------------------------------------

// unsigned -> mask and shift
template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
inline
ValueType
vfc::TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType>
::shiftAndSignExtend(const vfc::uint8_t*, vfc::uint32_t f_rawValue_u32, vfc::true_t)
{
    return ((f_rawValue_u32 >> BITS_SHIFT) & getMask());
}

template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
inline
ValueType
vfc::TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType>
::shiftAndSignExtend(const vfc::uint8_t*, vfc::uint64_t f_rawValue_u64, vfc::true_t)
{
    return ((f_rawValue_u64 >> BITS_SHIFT) & getMask());
}

// signed -> shift and sign extend
template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
inline
ValueType
vfc::TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType>
::shiftAndSignExtend(const vfc::uint8_t* f_data_p, vfc::uint32_t f_rawValue_u32, vfc::false_t)
{
    return static_cast<ValueType>(
        static_cast<vfc::int32_t>(
        f_rawValue_u32 << (32 - MASK_LENGTH - BITS_SHIFT)
        ) >> (32 - MASK_LENGTH));
}

template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
inline
ValueType
vfc::TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType>
::shiftAndSignExtend(const vfc::uint8_t* f_data_p, vfc::uint64_t f_rawValue_u64, vfc::false_t)
{
    return static_cast<ValueType>(
        static_cast<vfc::int64_t>(
        f_rawValue_u64 << (64 - MASK_LENGTH - BITS_SHIFT)
        ) >> (64 - MASK_LENGTH));
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_bit_array_view.inl  $
//  Revision 1.6 2012/12/18 08:27:30MEZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.5 2011/06/10 15:46:21MESZ Muehlmann Karsten (CC/PJ-FA1) (MUK2LR) 
//  - add unaligned 64bit read hint for ARM compiler (mantis3809)
//  Revision 1.4 2011/06/07 17:19:46MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - added missing typename keyword (mantis3809)
//  Revision 1.3 2011/05/31 19:37:59MESZ Muehlmann Karsten (CC/ESV2) (MUK2LR) 
//  - remove extra sign bit template parameter and optimize for ARM (mantis3809)
//  Revision 1.2 2010/09/09 09:02:48MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - memory optimization adressing the "MASK" variable (mantis3405)
//  Revision 1.11 2009/06/24 13:41:00MESZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Mantis 2876
//  - Bugfix for setValueIntern for corrupting the data in the buffer, when the MSB is not byte aligned.
//  Revision 1.10 2009/02/06 09:47:35GMT+05:30 Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  -Removal of QAC++ warnings.
//  (Mantis : 0002565)
//  Revision 1.9 2008/08/25 22:24:26IST Jaeger Thomas (CC-DA/ESV2) (JAT2HI)
//  - removed unused parameter names to suppress warnings
//  Revision 1.8 2008/07/29 12:34:09CEST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  const correction (Mantis:- 2070)
//  Revision 1.7 2008/04/16 14:21:58IST Pavithra K T (RBEI/EAE6) (pkt1kor)
//  mantis issue 2070
//  Revision 1.6 2008/03/26 10:20:06IST Pavithra K T (RBEI/EAE6) (pkt1kor)
//  namespace added for TMaskGenerator
//  naming convention for variables are updated
//  Revision 1.5 2008/03/25 19:35:52IST Pavithra K T (RBEI/EAE6) (pkt1kor)
//  changes in the mask generation
//  Revision 1.4 2008/03/19 11:59:27IST Pavithra K T (RBEI/EAE6) (pkt1kor)
//  - changed to make it work correctly for big endian values
//  Revision 1.3 2008/02/27 12:04:09IST Dilip Krishna (RBIN/EAE6) (dkn2kor)
//  - typedefed vfc datatypes used. mask generator logic changed
//  Revision 1.2 2008/02/26 16:11:28IST Dilip Krishna (RBIN/EAE6) (dkn2kor)
//  - header implementation moved, mask code change to be more optimised
//  Revision 1.1 2008/02/26 14:59:20IST Dilip Krishna (RBIN/EAE6) (dkn2kor)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_bit_array_view/vfc_bit_array_view.pj
//=============================================================================
