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
/// @brief This is a brief description.
/// @par Synopsis:
///     This is the detailed description.
///
/// @par Revision History:
///     $Source: vfc_bit_array_view.hpp $
///     $Revision: 1.5 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
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

#ifndef VFC_BIT_ARRAY_VIEW_HPP_INCLUDED
#define VFC_BIT_ARRAY_VIEW_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"           // using vfc::int32, vfc::uint8
#include "vfc/core/vfc_type_traits.hpp"     // using vfc::TIsUnsignedArithmetic
#include "vfc/core/vfc_static_assert.hpp"   // using VFC_STATIC_ASSERT
#include "vfc/core/vfc_endianess.hpp"       // using convert...ToSystem
#include "vfc/core/vfc_metaprog.hpp"        // using TIf, TSigned2Unsigned

namespace
{
    template<class ValueType, vfc::int32_t LengthValue>
    struct TMaskGen
    {
        static const ValueType GENERATED_MASK 
            = (LengthValue < CHAR_BIT * sizeof(ValueType)) 
                ? (static_cast<ValueType>(1) << LengthValue) - 1 
                : ~0; // all bits set
    };

    // prevent compiler warning about integral constant overflow
    // even though the overflowing "1 << LengthValue" wouldn't be used, but anyways...
    template<class ValueType>
    struct TMaskGen<ValueType, 32>
    {
        static const ValueType GENERATED_MASK = 0xffffffff;
    };

    template<class ValueType>
    struct TMaskGen<ValueType, 64>
    {
        static const ValueType GENERATED_MASK = 0xffffffffffffffffll;
    };
}


namespace vfc
{    // namespace vfc opened

    enum 
    { 
        UNDEFINED_SIGNBIT_POS = -1
    };

    class CBitArrayViewBigEndianPolicy
    {
    public:
        template <class argType>
        static inline
        argType convToSystemOrder(const argType& f_arg)
        {
            return vfc::convertBigEndianToSystem(f_arg);
        }

        template <class argType>
        static inline
        argType convFromSystemOrder(const argType& f_arg)
        {
            return vfc::convertSystemToBigEndian(f_arg);
        }
    };

    class CBitArrayViewLittleEndianPolicy
    {
    public:
        template <class argType>
        static inline
        argType convToSystemOrder(const argType& f_arg)
        {
            return vfc::convertLittleEndianToSystem(f_arg);
        }

        template <class argType>
        static inline
        argType convFromSystemOrder(const argType& f_arg)
        {
            return vfc::convertSystemToLittleEndian(f_arg);
        }
    };
    
    template<class ValueType, vfc::int32_t StartBitValue, vfc::int32_t LengthValue, class EndianessPolicyType>
    struct TBitArrayViewConstants
    {
        enum 
        { 
            FIRST_BYTE_TO_READ   = StartBitValue / CHAR_BIT,
            // position (0-based) of bit within first byte to read
            LSB_BIT_POS_IN_BYTE  = StartBitValue % CHAR_BIT,
            NUM_BYTES_TO_READ    = ((LengthValue + LSB_BIT_POS_IN_BYTE - 1) / CHAR_BIT) + 1,
            // ValueType fits into 32 or 64bits?
            READ_MORE_THAN_32BIT = (sizeof(ValueType) >  4),
            // bits span more than 4(8) bytes even though final result fits into 4(8) bytes
            READ_EXTRA_BYTE      = READ_MORE_THAN_32BIT ? (NUM_BYTES_TO_READ == 9) : (NUM_BYTES_TO_READ == 5),
            BITS_SHIFT           = LSB_BIT_POS_IN_BYTE
        };
    };

    template<class ValueType, vfc::int32_t StartBitValue, vfc::int32_t LengthValue>
    struct TBitArrayViewConstants<ValueType, StartBitValue, LengthValue, CBitArrayViewBigEndianPolicy>
    {
        enum  
        { 
            LSB_BIT_POS_IN_BYTE  = StartBitValue % CHAR_BIT,
            // BigEndian -> subsequent bytes have _lower_ addresses than byte with lsb
            FIRST_BYTE_TO_READ   = (StartBitValue / CHAR_BIT) - ((LengthValue + LSB_BIT_POS_IN_BYTE - 1) / CHAR_BIT),
            NUM_BYTES_TO_READ    = ((LengthValue + LSB_BIT_POS_IN_BYTE - 1) / CHAR_BIT) + 1,
            // ValueType fits into 32 or 64bits?
            READ_MORE_THAN_32BIT = (sizeof(ValueType) >  4),
            // bits span more than 4(8) bytes even though final result fits into 4(8) bytes
            READ_EXTRA_BYTE      = READ_MORE_THAN_32BIT ? (NUM_BYTES_TO_READ == 9) : (NUM_BYTES_TO_READ == 5),
            BITS_SHIFT           = LSB_BIT_POS_IN_BYTE + (READ_EXTRA_BYTE ? 0 : (( (READ_MORE_THAN_32BIT ? 8 : 4) - NUM_BYTES_TO_READ) * CHAR_BIT))
        };
    };

    template<class ValueType, vfc::uint32_t StartBitValue, vfc::uint32_t LengthValue, class EndianessPolicyType>
    class TBitArrayView
    {
        typedef TBitArrayView<ValueType, StartBitValue, LengthValue, EndianessPolicyType> self_type;

        typedef typename vfc::TIf<vfc::TIsUnsignedArithmetic<ValueType>::value, ValueType, typename TSigned2Unsigned<ValueType>::type>::type mask_type;

    public:
        typedef TBitArrayViewConstants<ValueType, StartBitValue, LengthValue, EndianessPolicyType> tba_consts_t;
        enum 
        {
            IS_UNSIGNED          = vfc::TIsUnsignedArithmetic<ValueType>::value,
            FIRST_BYTE_TO_READ   = tba_consts_t::FIRST_BYTE_TO_READ,
            NUM_BYTES_TO_READ    = tba_consts_t::NUM_BYTES_TO_READ,
            MASK_LENGTH          = LengthValue,
            READ_MORE_THAN_32BIT = tba_consts_t::READ_MORE_THAN_32BIT,
            READ_EXTRA_BYTE      = tba_consts_t::READ_EXTRA_BYTE,
            BITS_SHIFT           = tba_consts_t::BITS_SHIFT
        };

        static mask_type getMask() 
        {
            return static_cast<mask_type>(TMaskGen<mask_type, LengthValue>::GENERATED_MASK);
        }

        static inline
        ValueType getValue(const vfc::uint8_t* f_data_p);

        static inline
        void setValue(ValueType f_value, vfc::uint8_t* f_data_p);

        // we do not support returning complex types
        VFC_STATIC_ASSERT(vfc::TIsPOD<ValueType>::value);

        // result must be able to fit into ValueType
        VFC_STATIC_ASSERT((sizeof(ValueType) * CHAR_BIT) >= LengthValue);

    private:
        //! read up to 4 bytes
        static inline
        ValueType getValueIntern(const vfc::uint8_t* f_data_p, vfc::false_t, vfc::false_t);

        //! read 5 bytes
        static inline
        ValueType getValueIntern(const vfc::uint8_t* f_data_p, vfc::false_t, vfc::true_t);

        //! read 6 to 8 bytes
        static inline
        ValueType getValueIntern(const vfc::uint8_t* f_data_p, vfc::true_t, vfc::false_t);

        //! read 9 bytes
        static inline
        ValueType getValueIntern(const vfc::uint8_t* f_data_p, vfc::true_t, vfc::true_t);

        //! shift and mask for unsigned values
        static inline
        ValueType shiftAndSignExtend(const vfc::uint8_t*, vfc::uint32_t f_rawValue_u32, vfc::true_t);
        static inline
        ValueType shiftAndSignExtend(const vfc::uint8_t*, vfc::uint64_t f_rawValue_u64, vfc::true_t);

        //! sign extend and shift for signed values
        static inline
        ValueType shiftAndSignExtend(const vfc::uint8_t* f_data_p, vfc::uint32_t f_rawValue_u32, vfc::false_t);
        static inline
        ValueType shiftAndSignExtend(const vfc::uint8_t* f_data_p, vfc::uint64_t f_rawValue_u64, vfc::false_t);

        //! write up to 4 bytes
        static inline
        void setValueIntern(ValueType f_value, vfc::uint8_t* f_data_p, vfc::false_t, vfc::false_t);

        //! write 5 bytes
        static inline
        void setValueIntern(ValueType f_value, vfc::uint8_t* f_data_p, vfc::false_t, vfc::true_t);

        //! write 6 to 8 bytes
        static inline
        void setValueIntern(ValueType f_value, vfc::uint8_t* f_data_p, vfc::true_t, vfc::false_t);

        //! write 9 bytes
        static inline
        void setValueIntern(ValueType f_value, vfc::uint8_t* f_data_p, vfc::true_t, vfc::true_t);

        // quirks: TBitArrayView::setValue() with other StartBit and LengthValue needs to access setValueIntern methods
        template <class OtherValueType, vfc::uint32_t OtherStartBitValue, vfc::uint32_t OtherLengthValue, class OtherEndianessPolicyType>
        friend class TBitArrayView;
    };
}


#include "vfc/core/vfc_bit_array_view.inl"

#endif //VFC_BIT_ARRAY_VIEW_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_bit_array_view.hpp  $
//  Revision 1.5 2012/12/18 08:27:30MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.4 2012/12/17 14:46:31MEZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - TBitArrayView overrides bytes outside mask in certain cases (mantis 4195)
//  Revision 1.3 2011/05/31 19:37:45MESZ Muehlmann Karsten (CC/PJ-FA1) (MUK2LR) 
//  - remove extra sign bit template parameter and optimize for ARM (mantis3809)
//  Revision 1.2 2010/09/09 09:02:47MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - memory optimization adressing the "MASK" variable (mantis3405)
//  Revision 1.8 2008/07/29 12:33:59MESZ Vinaykumar Setty (RBEI/ESB2) (vmr1kor) 
//  const correction (Mantis:- 2070)
//  Revision 1.7 2008/04/16 14:21:55IST Pavithra K T (RBEI/EAE6) (pkt1kor) 
//  mantis issue 2070
//  Revision 1.6 2008/03/26 10:16:40IST Pavithra K T (RBEI/EAE6) (pkt1kor) 
//  removed unused code
//  Revision 1.5 2008/03/25 19:35:52IST Pavithra K T (RBEI/EAE6) (pkt1kor) 
//  changes in the mask generation
//  Revision 1.4 2008/03/19 11:59:26IST Pavithra K T (RBEI/EAE6) (pkt1kor) 
//  - changed to make it work correctly for big endian values
//  Revision 1.3 2008/02/27 12:03:49IST Dilip Krishna (RBIN/EAE6) (dkn2kor) 
//  - typedefed vfc datatypes used
//  Revision 1.2 2008/02/26 16:11:05IST Dilip Krishna (RBIN/EAE6) (dkn2kor) 
//  - header implementation present moved to implementation file (.inl file)
//  Revision 1.1 2008/02/26 14:59:19IST Dilip Krishna (RBIN/EAE6) (dkn2kor) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_bit_array_view/vfc_bit_array_view.pj
//=============================================================================
