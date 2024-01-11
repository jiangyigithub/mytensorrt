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
//       Projectname: vfc
//  Target system(s):
//       Compiler(s): c++ std conformal
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: ves3kor
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
///     $Source: vfc_siunits_helper.hpp $
///     $Revision: 1.10 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/09/24 15:50:23MESZ $
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

#ifndef VFC_SIUNITS_HELPER_HPP_INCLUDED
#define VFC_SIUNITS_HELPER_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"           //vfc::int64_t
#include "vfc/core/vfc_static_assert.hpp"   //VFC_STATIC_ASSERT
#include "vfc/core/vfc_math.hpp"            //! G_DEG2RAD
#include "vfc/core/vfc_metaprog.hpp"        // TIf

namespace vfc
{

    enum ESiPrefix
    {
        YOCTO = -24,
        ZEPTO = -21,
        ATTO = -18,
        FEMTO = -15,
        PICO = -12,
        NANO = -9,
        MICRO = -6,
        MILLI = -3,
        CENTI = -2,
        DECI = -1,
        NIL = 0,
        BASE = 0,
        DECA = 1,
        HECTO = 2,
        KILO = 3,
        MEGA = 6,
        GIGA = 9,
        TERA = 12,
        PETA = 15,
        EXA = 18,
        ZETTA = 21,
        YOTTA = 24
    };

    typedef vfc::int64_t integer_type;

    struct CHasZero
    {
    };

    struct CHasOne
    {
    };

    template<integer_type InputValue, class TrailingValueType>
    struct TCountTrailingZerosIntern
    {
        enum
        {
            ZERO_COUNT = 0
        };
    };

    template<integer_type InputValue>
    struct TCountTrailingZerosIntern<InputValue, CHasZero>
    {
        enum
        {
            ZERO_COUNT = 1 + TCountTrailingZerosIntern<(InputValue >> 1),
            typename vfc::TIf<(((InputValue >> 1) & 0x1)==0), CHasZero, CHasOne>::type>::ZERO_COUNT
        };
    };

    template<integer_type InputValue>
    struct TCountTrailingZeros
    {
        enum
        {
            ZERO_COUNT = TCountTrailingZerosIntern<InputValue,
            typename vfc::TIf<((InputValue & 0x1)==0), CHasZero, CHasOne>::type>::ZERO_COUNT
        };
    };

    template<integer_type NumValue, integer_type DenValue, integer_type DivisorValue>
    struct TIsDivisible
    {
        enum
        {
            NUM_IS_DIVISIBLE = ((NumValue % DivisorValue) == 0) ? 1 : 0
        };

        enum
        {
            DEN_IS_DIVISIBLE = ((DenValue % DivisorValue) == 0) ? 1 : 0
        };

        enum
        {
            IS_DIVISIBLE = (NUM_IS_DIVISIBLE && DEN_IS_DIVISIBLE) ? 1 : 0
        };

        typedef typename vfc::TIf<(IS_DIVISIBLE==1), vfc::true_t, vfc::false_t>::type type;
    };

    // Checks if the angle unit type must disappear after multiplication
    // ex: [m] * [rad] = [m]; [1/m] * [1/rad] = [1/m]; [rad] * [1/s] = [rad/s]; [s] * [1/rad] = [s/rad];
    // The power of any unit must have the opposite sign than the power of the angle, so that the angle unit stays
    // if (a * b <= 0) then they have opposite signs
    template<class UnitInfo1Type, class UnitInfo2Type>
    struct TIsMultPowAng
    {
        enum { 
            value = (
                        (( (UnitInfo1Type::LENGTH_POWER_VALUE + UnitInfo2Type::LENGTH_POWER_VALUE) * 
                            (UnitInfo1Type::ANGLE_POWER_VALUE + UnitInfo2Type::ANGLE_POWER_VALUE) ) <= 0 ) &&
                        (( (UnitInfo1Type::MASS_POWER_VALUE + UnitInfo2Type::MASS_POWER_VALUE) * 
                            (UnitInfo1Type::ANGLE_POWER_VALUE + UnitInfo2Type::ANGLE_POWER_VALUE) ) <= 0 ) &&
                        (( (UnitInfo1Type::TIME_POWER_VALUE + UnitInfo2Type::TIME_POWER_VALUE) *
                            (UnitInfo1Type::ANGLE_POWER_VALUE + UnitInfo2Type::ANGLE_POWER_VALUE) ) <= 0 ) &&
                        (( (UnitInfo1Type::CURRENT_POWER_VALUE + UnitInfo2Type::CURRENT_POWER_VALUE) *
                            (UnitInfo1Type::ANGLE_POWER_VALUE + UnitInfo2Type::ANGLE_POWER_VALUE) ) <= 0 ) &&
                        (( (UnitInfo1Type::TEMPERATURE_POWER_VALUE + UnitInfo2Type::TEMPERATURE_POWER_VALUE) *
                            (UnitInfo1Type::ANGLE_POWER_VALUE + UnitInfo2Type::ANGLE_POWER_VALUE) ) <= 0 ) &&
                        (( (UnitInfo1Type::AMOUNTOFSUBSTANCE_POWER_VALUE + UnitInfo2Type::AMOUNTOFSUBSTANCE_POWER_VALUE) *
                            (UnitInfo1Type::ANGLE_POWER_VALUE + UnitInfo2Type::ANGLE_POWER_VALUE) ) <= 0 ) &&
                        (( (UnitInfo1Type::LUMINOUSINTENSITY_PREFIX_VALUE + UnitInfo2Type::LUMINOUSINTENSITY_PREFIX_VALUE) * 
                            (UnitInfo1Type::ANGLE_POWER_VALUE + UnitInfo2Type::ANGLE_POWER_VALUE) ) <= 0 )
            )
        };
    };
        
    // Checks if the angle unit type must disappear after division
    // ex: [s] / [rad] = [s/rad]; [rad] / [s] = [rad/s]; [rad] / [1/m] = [m]; [m] / [1/rad] = [m];
    // The power of any unit must have the opposite sign than the power of the angle, so that the angle unit stays
    // if (a * b <= 0) then they have opposite signs
    template<class UnitInfo1Type, class UnitInfo2Type>
    struct TIsDivPowAng
    {
        enum { 
            value = (
                        (( (UnitInfo1Type::LENGTH_POWER_VALUE - UnitInfo2Type::LENGTH_POWER_VALUE) * 
                            (UnitInfo1Type::ANGLE_POWER_VALUE - UnitInfo2Type::ANGLE_POWER_VALUE) ) <= 0 ) &&
                        (( (UnitInfo1Type::MASS_POWER_VALUE - UnitInfo2Type::MASS_POWER_VALUE) * 
                            (UnitInfo1Type::ANGLE_POWER_VALUE - UnitInfo2Type::ANGLE_POWER_VALUE) ) <= 0 ) &&
                        (( (UnitInfo1Type::TIME_POWER_VALUE - UnitInfo2Type::TIME_POWER_VALUE) * 
                            (UnitInfo1Type::ANGLE_POWER_VALUE - UnitInfo2Type::ANGLE_POWER_VALUE) ) <= 0 ) &&
                        (( (UnitInfo1Type::CURRENT_POWER_VALUE - UnitInfo2Type::CURRENT_POWER_VALUE) * 
                            (UnitInfo1Type::ANGLE_POWER_VALUE - UnitInfo2Type::ANGLE_POWER_VALUE) ) <= 0 ) &&
                        (( (UnitInfo1Type::TEMPERATURE_POWER_VALUE - UnitInfo2Type::TEMPERATURE_POWER_VALUE) * 
                            (UnitInfo1Type::ANGLE_POWER_VALUE - UnitInfo2Type::ANGLE_POWER_VALUE) ) <= 0 ) &&
                        (( (UnitInfo1Type::AMOUNTOFSUBSTANCE_POWER_VALUE - UnitInfo2Type::AMOUNTOFSUBSTANCE_POWER_VALUE) *
                            (UnitInfo1Type::ANGLE_POWER_VALUE - UnitInfo2Type::ANGLE_POWER_VALUE) ) <= 0 ) &&
                        (( (UnitInfo1Type::LUMINOUSINTENSITY_PREFIX_VALUE - UnitInfo2Type::LUMINOUSINTENSITY_PREFIX_VALUE) * 
                            (UnitInfo1Type::ANGLE_POWER_VALUE - UnitInfo2Type::ANGLE_POWER_VALUE) ) <= 0 )
            ) 
        };
    };

    // Checks if the unit has only negative powers
    template<class UnitInfoType>
    struct TIsNegativePower
    {
        enum { 
            value = (
                        ( UnitInfoType::LENGTH_POWER_VALUE <= 0 ) &&
                        ( UnitInfoType::MASS_POWER_VALUE <= 0 ) &&
                        ( UnitInfoType::TIME_POWER_VALUE <= 0 ) &&
                        ( UnitInfoType::CURRENT_POWER_VALUE <= 0 ) &&
                        ( UnitInfoType::TEMPERATURE_POWER_VALUE <= 0 ) &&
                        ( UnitInfoType::AMOUNTOFSUBSTANCE_POWER_VALUE <= 0 ) &&
                        ( UnitInfoType::LUMINOUSINTENSITY_PREFIX_VALUE <= 0) &&
                        ( UnitInfoType::ANGLE_POWER_VALUE <= 0 ) 
            ) 
        };
    };

    // Checks if the unit is a valid angle dominant unit (ex: deg, rad/s, rad/m...)
    template<class UnitInfoType>
    struct TIsValidAngleDominant
    {
        enum { 
            value = (
                        ( UnitInfoType::ANGLE_POWER_VALUE > 0 ) &&
                        ( UnitInfoType::LENGTH_POWER_VALUE <= 0 ) &&
                        ( UnitInfoType::MASS_POWER_VALUE <= 0 ) &&
                        ( UnitInfoType::TIME_POWER_VALUE <= 0 ) &&
                        ( UnitInfoType::CURRENT_POWER_VALUE <= 0 ) &&
                        ( UnitInfoType::TEMPERATURE_POWER_VALUE <= 0 ) &&
                        ( UnitInfoType::AMOUNTOFSUBSTANCE_POWER_VALUE <= 0 ) &&
                        ( UnitInfoType::LUMINOUSINTENSITY_PREFIX_VALUE <= 0 )
            ) 
        };
    };

    template<integer_type NumValue, integer_type DenValue, integer_type DivisorValue,
    class BooleanType = vfc::false_t>
    struct TFactoriseWithValueIntern
    {
        static const integer_type NUM = NumValue;
        static const integer_type DEN = DenValue;
    };

    template<integer_type NumValue, integer_type DenValue, integer_type DivisorValue>
    struct TFactoriseWithValueIntern<NumValue, DenValue, DivisorValue, vfc::true_t>
    {
        static const integer_type NUM = TFactoriseWithValueIntern<
            NumValue/DivisorValue, DenValue/DivisorValue, DivisorValue,
            typename TIsDivisible<NumValue/DivisorValue, DenValue/DivisorValue, DivisorValue>::type>::NUM;

        static const integer_type DEN = TFactoriseWithValueIntern<
            NumValue/DivisorValue, DenValue/DivisorValue, DivisorValue,
            typename TIsDivisible<NumValue/DivisorValue, DenValue/DivisorValue, DivisorValue>::type>::DEN;
    };


    template<integer_type NumValue, integer_type DenValue, integer_type DivisorValue>
    struct TFactorizeWithValue
    {
        static const integer_type NUM = TFactoriseWithValueIntern<NumValue, DenValue, DivisorValue,
            typename TIsDivisible<NumValue, DenValue, DivisorValue>::type>::NUM;

        static const integer_type DEN = TFactoriseWithValueIntern<NumValue, DenValue, DivisorValue,
            typename TIsDivisible<NumValue, DenValue, DivisorValue>::type>::DEN;
    };

    template<integer_type NumValue, integer_type DenValue>
    struct TFactorize
    {
        static const vfc::int32_t NUM_SHR_VALUE = TCountTrailingZeros<NumValue>::ZERO_COUNT;
        static const vfc::int32_t DEN_SHR_VALUE = TCountTrailingZeros<DenValue>::ZERO_COUNT;

        static const vfc::int32_t SHR_VALUE = (NUM_SHR_VALUE < DEN_SHR_VALUE)? NUM_SHR_VALUE : DEN_SHR_VALUE;

        static const integer_type NUM =
            ((TFactorizeWithValue<NumValue, DenValue, 5>::NUM) >> SHR_VALUE);
        static const integer_type DEN =
            ((TFactorizeWithValue<NumValue, DenValue, 5>::DEN) >> SHR_VALUE);
    };

    template<class ValueType>
    struct TMaxValue
    {

    };

    template<>
    struct TMaxValue<vfc::int32_t>
    {
        static const vfc::int32_t MAX = 0x7FFFFFFFL;
    };

    template<>
    struct TMaxValue<vfc::int64_t>
    {
        //PRQA S 474 ++
        static const vfc::int64_t MAX = 0x7FFFFFFFFFFFFFFFLL;
        //PRQA S 474 --
    };
    template<integer_type Input1Value, integer_type Input2Value>
    struct TMultiply
    {
        //Overflow due to multiply. Value too big.
        VFC_STATIC_ASSERT((Input1Value < (TMaxValue<integer_type>::MAX / Input2Value)));

        static const integer_type VALUE = (Input1Value * Input2Value);
    };

    template<
        integer_type Num1Value, integer_type Den1Value,
        integer_type Num2Value = 1, integer_type Den2Value = 1,
        integer_type Num3Value = 1, integer_type Den3Value = 1,
        integer_type Num4Value = 1, integer_type Den4Value = 1,
        integer_type Num5Value = 1, integer_type Den5Value = 1,
        integer_type Num6Value = 1, integer_type Den6Value = 1,
        integer_type Num7Value = 1, integer_type Den7Value = 1,
        integer_type Num8Value = 1, integer_type Den8Value = 1,
        integer_type Num9Value = 1, integer_type Den9Value = 1,
        integer_type Num10Value = 1, integer_type Den10Value = 1,
        integer_type Num11Value = 1, integer_type Den11Value = 1>
    struct TFactorizeInputs
    {
        //Factorizing Num1Value, Den1Value, Num2Value, Den2Value
        static const integer_type NUM_2 = TFactorize<
            TMultiply<TFactorize<Num1Value, Den2Value>::NUM,
            TFactorize<Num2Value, Den1Value>::NUM>::VALUE,
            TMultiply<TFactorize<Num1Value, Den2Value>::DEN,
            TFactorize<Num2Value, Den1Value>::DEN>::VALUE>::NUM;
        static const integer_type DEN_2 = TFactorize<
            TMultiply<TFactorize<Num1Value, Den2Value>::NUM,
            TFactorize<Num2Value, Den1Value>::NUM>::VALUE,
            TMultiply<TFactorize<Num1Value, Den2Value>::DEN,
            TFactorize<Num2Value, Den1Value>::DEN>::VALUE>::DEN;


        //Factorizing NUM_2, DEN_2, Num3Value, Den3Value
        static const integer_type NUM_3 = TFactorize<
            TMultiply<TFactorize<NUM_2, Den3Value>::NUM,
            TFactorize<Num3Value, DEN_2>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_2, Den3Value>::DEN,
            TFactorize<Num3Value, DEN_2>::DEN>::VALUE>::NUM;
        static const integer_type DEN_3 = TFactorize<
            TMultiply<TFactorize<NUM_2, Den3Value>::NUM,
            TFactorize<Num3Value, DEN_2>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_2, Den3Value>::DEN,
            TFactorize<Num3Value, DEN_2>::DEN>::VALUE>::DEN;

        //Factorizing NUM_3, DEN_3, Num4Value, Den4Value
        static const integer_type NUM_4 = TFactorize<
            TMultiply<TFactorize<NUM_3, Den4Value>::NUM,
            TFactorize<Num4Value, DEN_3>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_3, Den4Value>::DEN,
            TFactorize<Num4Value, DEN_3>::DEN>::VALUE>::NUM;
        static const integer_type DEN_4 = TFactorize<
            TMultiply<TFactorize<NUM_3, Den4Value>::NUM,
            TFactorize<Num4Value, DEN_3>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_3, Den4Value>::DEN,
            TFactorize<Num4Value, DEN_3>::DEN>::VALUE>::DEN;


        //Factorizing NUM_4, DEN_4, Num5Value, Den5Value
        static const integer_type NUM_5 = TFactorize<
            TMultiply<TFactorize<NUM_4, Den5Value>::NUM,
            TFactorize<Num5Value, DEN_4>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_4, Den5Value>::DEN,
            TFactorize<Num5Value, DEN_4>::DEN>::VALUE>::NUM;
        static const integer_type DEN_5 = TFactorize<
            TMultiply<TFactorize<NUM_4, Den5Value>::NUM,
            TFactorize<Num5Value, DEN_4>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_4, Den5Value>::DEN,
            TFactorize<Num5Value, DEN_4>::DEN>::VALUE>::DEN;

        //Factorizing NUM_5, DEN_5, Num6Value, Den6Value
        static const integer_type NUM_6 = TFactorize<
            TMultiply<TFactorize<NUM_5, Den6Value>::NUM,
            TFactorize<Num6Value, DEN_5>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_5, Den6Value>::DEN,
            TFactorize<Num6Value, DEN_5>::DEN>::VALUE>::NUM;
        static const integer_type DEN_6 = TFactorize<
            TMultiply<TFactorize<NUM_5, Den6Value>::NUM,
            TFactorize<Num6Value, DEN_5>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_5, Den6Value>::DEN,
            TFactorize<Num6Value, DEN_5>::DEN>::VALUE>::DEN;


        //Factorizing NUM_6, DEN_6, Num7Value, Den7Value
        static const integer_type NUM_7 = TFactorize<
            TMultiply<TFactorize<NUM_6, Den7Value>::NUM,
            TFactorize<Num7Value, DEN_6>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_6, Den7Value>::DEN,
            TFactorize<Num7Value, DEN_6>::DEN>::VALUE>::NUM;
        static const integer_type DEN_7 = TFactorize<
            TMultiply<TFactorize<NUM_6, Den7Value>::NUM,
            TFactorize<Num7Value, DEN_6>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_6, Den7Value>::DEN,
            TFactorize<Num7Value, DEN_6>::DEN>::VALUE>::DEN;


        //Factorizing NUM_7, DEN_7, Num8Value, Den8Value
        static const integer_type NUM_8 = TFactorize<
            TMultiply<TFactorize<NUM_7, Den8Value>::NUM,
            TFactorize<Num8Value, DEN_7>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_7, Den8Value>::DEN,
            TFactorize<Num8Value, DEN_7>::DEN>::VALUE>::NUM;
        static const integer_type DEN_8 = TFactorize<
            TMultiply<TFactorize<NUM_7, Den8Value>::NUM,
            TFactorize<Num8Value, DEN_7>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_7, Den8Value>::DEN,
            TFactorize<Num8Value, DEN_7>::DEN>::VALUE>::DEN;

        //Factorizing NUM_8, DEN_8, Num9Value, Den9Value
        static const integer_type NUM_9 = TFactorize<
            TMultiply<TFactorize<NUM_8, Den9Value>::NUM,
            TFactorize<Num9Value, DEN_8>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_8, Den9Value>::DEN,
            TFactorize<Num9Value, DEN_8>::DEN>::VALUE>::NUM;
        static const integer_type DEN_9 = TFactorize<
            TMultiply<TFactorize<NUM_8, Den9Value>::NUM,
            TFactorize<Num9Value, DEN_8>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_8, Den9Value>::DEN,
            TFactorize<Num9Value, DEN_8>::DEN>::VALUE>::DEN;

        //Factorizing NUM_9, DEN_9, Num10Value, Den10Value
        static const integer_type NUM_10 = TFactorize<
            TMultiply<TFactorize<NUM_9, Den10Value>::NUM,
            TFactorize<Num10Value, DEN_9>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_9, Den10Value>::DEN,
            TFactorize<Num10Value, DEN_9>::DEN>::VALUE>::NUM;
        static const integer_type DEN_10 = TFactorize<
            TMultiply<TFactorize<NUM_9, Den10Value>::NUM,
            TFactorize<Num10Value, DEN_9>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_9, Den10Value>::DEN,
            TFactorize<Num10Value, DEN_9>::DEN>::VALUE>::DEN;

        //Factorizing NUM_10, DEN_10, Num11Value, Den11Value
        static const integer_type NUM = TFactorize<
            TMultiply<TFactorize<NUM_10, Den11Value>::NUM,
            TFactorize<Num11Value, DEN_10>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_10, Den11Value>::DEN,
            TFactorize<Num11Value, DEN_10>::DEN>::VALUE>::NUM;
        static const integer_type DEN = TFactorize<
            TMultiply<TFactorize<NUM_10, Den11Value>::NUM,
            TFactorize<Num11Value, DEN_10>::NUM>::VALUE,
            TMultiply<TFactorize<NUM_10, Den11Value>::DEN,
            TFactorize<Num11Value, DEN_10>::DEN>::VALUE>::DEN;
    };

    struct CBasicType
    {
        struct CCompound
        {
        };

        typedef CCompound si_base_unit_type;

        enum
        {
            CONVERSION_RATIONAL = 1,
            FLOATING_OFFSET = 0
        };

        static const integer_type NUM = 1;
        static const integer_type DEN = 1;
        static const integer_type OFFSET = 0;
    };

    template<integer_type NumeratorValue, integer_type DenominatorValue,
        vfc::int32_t PowerValue, class BooleanType>
    struct TInvert
    {

    };

    template<integer_type NumeratorValue, integer_type DenominatorValue, vfc::int32_t PowerValue>
    struct TInvert<NumeratorValue, DenominatorValue, PowerValue, vfc::true_t>
    {
        enum
        {
            NUMERATOR = TPower<NumeratorValue, PowerValue>::VALUE,
            DENOMINATOR = TPower<DenominatorValue, PowerValue>::VALUE
        };
    };


    template<integer_type NumeratorValue, integer_type DenominatorValue,
        vfc::int32_t PowerValue>
    struct TInvert<NumeratorValue, DenominatorValue, PowerValue, vfc::false_t>
    {
        enum
        {
            NUMERATOR = TPower<DenominatorValue, (-PowerValue)>::VALUE,
            DENOMINATOR = TPower<NumeratorValue, (-PowerValue)>::VALUE
        };
    };

    namespace sitime
    {
        const vfc::uint32_t WEEK_IN_DAYS = 7 ;
        const vfc::uint32_t MONTH_IN_DAYS = 30 ;
        const vfc::uint32_t YEAR_IN_DAYS = 365 ;
        const vfc::uint32_t YEAR_IN_MONTHS = 12 ;
        

        enum ESiTimePrefix
        {
            NANO_SECOND             = -9,
            MICRO_SECOND            = -6,
            MILLI_SECOND            = -3,
            SECOND                  = 0,
            MINUTE                  = 60,
            HOUR                    = 3600,
            DAY                     = 3600*24,
            WEEK                    = 3600*24*WEEK_IN_DAYS,
            MONTH                   = 3600*24*MONTH_IN_DAYS,
            YEAR                    = 3600*24*YEAR_IN_DAYS
         };

        template<vfc::int32_t InUnitScaleValue, vfc::int32_t OutScaleValue>
        struct TTimeConvertUnroll
        {
        public:
            static const integer_type Y_VALUE =
                InUnitScaleValue - OutScaleValue;

            static const integer_type CONVERT_FACTOR_NUM =
                TPower<10, Y_VALUE>::VALUE;
            
            static const integer_type CONVERT_FACTOR_DEN = 1;
        };

        template<>
        struct TTimeConvertUnroll<MINUTE, MINUTE>
        {
        public:
              static const integer_type CONVERT_FACTOR_NUM =1;

              static const integer_type CONVERT_FACTOR_DEN =1;
           
        };

         template<>
        struct TTimeConvertUnroll<HOUR, HOUR>
        {
        public:
              static const integer_type CONVERT_FACTOR_NUM =1;

              static const integer_type CONVERT_FACTOR_DEN =1;
           
        };

        template<>
        struct TTimeConvertUnroll<DAY, DAY>
        {
        public:
              static const integer_type CONVERT_FACTOR_NUM =1;

              static const integer_type CONVERT_FACTOR_DEN =1;
           
        };

        template<>
        struct TTimeConvertUnroll<WEEK, WEEK>
        {
        public:
              static const integer_type CONVERT_FACTOR_NUM =1;

              static const integer_type CONVERT_FACTOR_DEN =1;
           
        };

        template<>
        struct TTimeConvertUnroll<MONTH, MONTH>
        {
        public:
              static const integer_type CONVERT_FACTOR_NUM =1;

              static const integer_type CONVERT_FACTOR_DEN =1;
           
        };

        template<>
        struct TTimeConvertUnroll<YEAR, YEAR>
        {
        public:
              static const integer_type CONVERT_FACTOR_NUM =1;

              static const integer_type CONVERT_FACTOR_DEN =1;
           
        };
        
        template<>
        struct TTimeConvertUnroll<YEAR,MONTH>
        {
            static const integer_type CONVERT_FACTOR_NUM = YEAR_IN_MONTHS  ;


            static const integer_type CONVERT_FACTOR_DEN = 1;
        };

        template<>
        struct TTimeConvertUnroll<YEAR, WEEK>
        {
            static const integer_type CONVERT_FACTOR_NUM = YEAR_IN_DAYS  ;


            static const integer_type CONVERT_FACTOR_DEN = WEEK_IN_DAYS;
        };

         template<vfc::int32_t OutScaleValue>
        struct TTimeConvertUnroll<YEAR, OutScaleValue>
        {
            static const integer_type CONVERT_FACTOR_NUM =
                YEAR/DAY * TTimeConvertUnroll<DAY, OutScaleValue>::CONVERT_FACTOR_NUM;


            static const integer_type CONVERT_FACTOR_DEN =
                1 * TTimeConvertUnroll<DAY, OutScaleValue>::CONVERT_FACTOR_DEN;
        };
        
         template<>
        struct TTimeConvertUnroll<MONTH, WEEK>
        {
            static const integer_type CONVERT_FACTOR_NUM = MONTH_IN_DAYS  ;


            static const integer_type CONVERT_FACTOR_DEN = WEEK_IN_DAYS;
        };

         template<vfc::int32_t OutScaleValue>
        struct TTimeConvertUnroll<MONTH, OutScaleValue>
        {
            static const integer_type CONVERT_FACTOR_NUM =
                (MONTH / DAY) * TTimeConvertUnroll<DAY, OutScaleValue>::CONVERT_FACTOR_NUM;


            static const integer_type CONVERT_FACTOR_DEN =
                1 * TTimeConvertUnroll<DAY, OutScaleValue>::CONVERT_FACTOR_DEN;
        };

        template<vfc::int32_t OutScaleValue>
        struct TTimeConvertUnroll<WEEK, OutScaleValue>
        {
            static const integer_type CONVERT_FACTOR_NUM =
                (WEEK / DAY) * TTimeConvertUnroll<DAY, OutScaleValue>::CONVERT_FACTOR_NUM;


            static const integer_type CONVERT_FACTOR_DEN =
                1 * TTimeConvertUnroll<DAY, OutScaleValue>::CONVERT_FACTOR_DEN;
        };

         template<vfc::int32_t OutScaleValue>
        struct TTimeConvertUnroll<DAY, OutScaleValue>
        {
            static const integer_type CONVERT_FACTOR_NUM =
                (DAY / HOUR) * TTimeConvertUnroll<HOUR, OutScaleValue>::CONVERT_FACTOR_NUM;


            static const integer_type CONVERT_FACTOR_DEN =
                1 * TTimeConvertUnroll<HOUR, OutScaleValue>::CONVERT_FACTOR_DEN;
        };

        template<>
        struct TTimeConvertUnroll<HOUR, MINUTE>
        {
            static const integer_type CONVERT_FACTOR_NUM = HOUR / MINUTE;

            static const integer_type CONVERT_FACTOR_DEN = 1;
        };

         template<vfc::int32_t OutScaleValue>
        struct TTimeConvertUnroll<HOUR, OutScaleValue>
        {
        public:
            static const integer_type CONVERT_FACTOR_NUM =
                HOUR * TTimeConvertUnroll<SECOND, OutScaleValue>::CONVERT_FACTOR_NUM;

            static const integer_type CONVERT_FACTOR_DEN =
                1 * TTimeConvertUnroll<SECOND, OutScaleValue>::CONVERT_FACTOR_DEN;

        };

                
        template<vfc::int32_t OutScaleValue>
        struct TTimeConvertUnroll<MINUTE, OutScaleValue>
        {
            static const integer_type CONVERT_FACTOR_NUM =
                MINUTE * TTimeConvertUnroll<SECOND, OutScaleValue>::CONVERT_FACTOR_NUM;


            static const integer_type CONVERT_FACTOR_DEN =
                1 * TTimeConvertUnroll<SECOND, OutScaleValue>::CONVERT_FACTOR_DEN;
        };

       template<vfc::int32_t PowerValue, vfc::int32_t InUnitPrefixValue, vfc::int32_t OutUnitPrefixValue>
        class TConvertTime
        {
        private:

            static const vfc::int32_t INUNITPREFIXVALUE_INTERNAL =
                (InUnitPrefixValue > OutUnitPrefixValue) ? InUnitPrefixValue : OutUnitPrefixValue;

            static const vfc::int32_t OUTUNITPREFIXVALUE_INTERNAL =
                (InUnitPrefixValue > OutUnitPrefixValue) ? OutUnitPrefixValue : InUnitPrefixValue;


            static const integer_type CONVERT_FACTOR_NUM =
                sitime::TTimeConvertUnroll<INUNITPREFIXVALUE_INTERNAL,
                OUTUNITPREFIXVALUE_INTERNAL>::CONVERT_FACTOR_NUM;

            static const integer_type CONVERT_FACTOR_DEN =
                sitime::TTimeConvertUnroll<INUNITPREFIXVALUE_INTERNAL,
                OUTUNITPREFIXVALUE_INTERNAL>::CONVERT_FACTOR_DEN;

            static const integer_type NUMERATOR_INTERNAL =
                (OutUnitPrefixValue > InUnitPrefixValue) ? CONVERT_FACTOR_DEN : CONVERT_FACTOR_NUM;

            static const integer_type DENOMINATOR_INTERNAL =
                (OutUnitPrefixValue < InUnitPrefixValue) ? CONVERT_FACTOR_DEN : CONVERT_FACTOR_NUM;

        public:
            static const integer_type NUM =
                TInvert<NUMERATOR_INTERNAL, DENOMINATOR_INTERNAL, PowerValue,
                typename vfc::TIf<(PowerValue >= 0), vfc::true_t, vfc::false_t>::type>::NUMERATOR;

            static const integer_type DEN =
                TInvert<NUMERATOR_INTERNAL, DENOMINATOR_INTERNAL, PowerValue,
                typename vfc::TIf<(PowerValue >= 0), vfc::true_t, vfc::false_t>::type>::DENOMINATOR;

            static const integer_type OFFSET = 0;
        };
    

    }

    template<integer_type InputValue>
    class TNumDenPower
    {
    public:
        static const integer_type NUM = (InputValue > 0)? InputValue : 0;
        static const integer_type DEN = (InputValue > 0)? 0 : -InputValue;
    };

    template<class SourceType, class DestinationType>
    struct TBasicTypeConvert
    {

        typedef TBasicTypeConvert<SourceType, DestinationType> self_type;

        static const integer_type NUM = SourceType::NUM * DestinationType::DEN;
        static const integer_type DEN = SourceType::DEN * DestinationType::NUM;

        enum
        {
            DO_MULTIPLY = SourceType::CONVERSION_RATIONAL,
            DO_DIVIDE = DestinationType::CONVERSION_RATIONAL,
            DO_OFFSET_ADDITION = SourceType::FLOATING_OFFSET,
            DO_OFFSET_SUBTRACTION = DestinationType::FLOATING_OFFSET
        };

        template<class ValueType>
        inline static void performConversion(ValueType& f_value_r)
        {

            self_type::doMultiply(f_value_r,
                typename vfc::TIf<(DO_MULTIPLY==1), vfc::false_t, vfc::true_t>::type());
            self_type::doDivide(f_value_r,
                typename vfc::TIf<(DO_DIVIDE==1), vfc::false_t, vfc::true_t>::type());
        }

        template<class ValueType>
        inline static void performOffsetConversion(ValueType& f_value_r)
        {
            self_type::doOffsetAddtion(f_value_r,
                typename vfc::TIf<(DO_OFFSET_ADDITION==1), vfc::true_t, vfc::false_t>::type());
            self_type::doOffsetSubtraction(f_value_r,
                typename vfc::TIf<(DO_OFFSET_SUBTRACTION==1), vfc::true_t, vfc::false_t>::type());
        }


        template<class ValueType>
        inline static void doMultiply(ValueType& f_value_r, vfc::true_t)
        {
            SourceType::doMultiply(f_value_r);
        }

        template<class ValueType>
        inline static void doMultiply(ValueType&, vfc::false_t)
        {
            //intentionally left blank
        }

        template<class ValueType>
        inline static void doDivide(ValueType& f_value_r, vfc::true_t)
        {
            DestinationType::doDivide(f_value_r);
        }

        template<class ValueType>
        inline static void doDivide(ValueType&, vfc::false_t)
        {
            //intentionally left blank
        }

        template<class ValueType>
        inline static void doOffsetAddtion(ValueType& f_value_r, vfc::true_t)
        {
            SourceType::doOffsetAddtion(f_value_r);
        }

        template<class ValueType>
        inline static void doOffsetAddtion(ValueType&, vfc::false_t)
        {
            //intentionally left blank
        }
        template<class ValueType>
        inline static void doOffsetSubtraction(ValueType& f_value_r, vfc::true_t)
        {
            DestinationType::doOffsetSubtraction(f_value_r);
        }

        template<class ValueType>
        inline static void doOffsetSubtraction(ValueType&, vfc::false_t)
        {
            //intentionally left blank
        }
    };


    template<class SourceType, class DestinationType>
    struct TComplexConvert
    {
        typedef TComplexConvert<SourceType, DestinationType> self_type;

        static const integer_type NUM = SourceType::NUM * DestinationType::DEN;
        static const integer_type DEN = SourceType::DEN * DestinationType::NUM;

        enum
        {
            DO_CONV = ((self_type::NUM != 1) || (self_type::DEN != 1))? 1 : 0
        };

        template<class ValueType>
        inline static void performConversion(ValueType& f_value_r)
        {
            self_type::performConversion(f_value_r,
                typename vfc::TIf<(DO_CONV==1), vfc::true_t, vfc::false_t>::type());
        }

        template<class ValueType>
        inline static void performConversion(ValueType& f_value_r, vfc::true_t)
        {
            DestinationType::performConversion(f_value_r);
        }

        template<class ValueType>
        inline static void performConversion(ValueType&, vfc::false_t)
        {
            //intentionally left blank
        }
    };

    struct CIntegralType
    {
    };

    struct CFloatingType
    {
    };

    template<class Unit1Type, class Unit2Type, template<class , class> class TConvertClass, class DataType>
    struct TUnitConversion
    {
        typedef TUnitConversion<Unit1Type, Unit2Type, TConvertClass, DataType> self_type;

        //Note if the conversion is irrational, the NUM & DEN will be 1
        enum
        {
            CONVERSION_RATIONAL = (Unit1Type::CONVERSION_RATIONAL && Unit2Type::CONVERSION_RATIONAL)? 1 : 0,
            FLOATING_OFFSET        = (Unit1Type::FLOATING_OFFSET || Unit2Type::FLOATING_OFFSET)? 1 : 0
        };

        static const integer_type NUM = (self_type::CONVERSION_RATIONAL == 1)?
            TConvertClass<Unit1Type, Unit2Type>::NUM : 1;

        static const integer_type DEN = (self_type::CONVERSION_RATIONAL == 1)?
            TConvertClass<Unit1Type, Unit2Type>::DEN : 1;

        template<class ValueType>
        inline static void performConversion(ValueType& f_value_r)
        {
            self_type::performConversion(f_value_r,
                typename vfc::TIf<(CONVERSION_RATIONAL==1), vfc::false_t, vfc::true_t>::type());

            self_type::performOffsetConversion(f_value_r,
                typename vfc::TIf<(FLOATING_OFFSET==1), vfc::true_t, vfc::false_t>::type());
        }

        template<class ValueType>
        inline static void performConversion(ValueType& f_value_r, vfc::true_t)
        {
            //comes here only if there is an irrational conversion
            TConvertClass<Unit1Type, Unit2Type>::performConversion(f_value_r);
        }

        template<class ValueType>
        inline static void performConversion(ValueType&, vfc::false_t)
        {
            //intentionall left blank
        }

        template<class ValueType>
        inline static void performOffsetConversion(ValueType& f_value_r, vfc::true_t)
        {
            //comes here only if there is an offset conversion
            TConvertClass<Unit1Type, Unit2Type>::performOffsetConversion(f_value_r);
        }

        template<class ValueType>
        inline static void performOffsetConversion(ValueType&, vfc::false_t)
        {
            //intentionall left blank
        }

    };

    template<class Unit1Type, class Unit2Type, template<class , class> class TConvertClass>
    struct TUnitConversion<Unit1Type, Unit2Type, TConvertClass, CIntegralType>
    {
        VFC_STATIC_ASSERT((vfc::TIsSameType<typename Unit1Type::si_base_unit_type,
            typename Unit2Type::si_base_unit_type>::value!=0));

        static const integer_type NUM = TConvertClass<Unit1Type, Unit2Type>::NUM;
        static const integer_type DEN = TConvertClass<Unit1Type, Unit2Type>::DEN;
    };

    template<class UnitInfo1Type, class UnitInfo2Type, class DataType>
    struct TNoConvert
    {
        //Conversion from one type to another conversion is illegal when using TNoConvert
        VFC_STATIC_ASSERT(UnitInfo1Type::LENGTH_PREFIX_VALUE == UnitInfo2Type::LENGTH_PREFIX_VALUE);
        VFC_STATIC_ASSERT(UnitInfo1Type::MASS_PREFIX_VALUE == UnitInfo2Type::MASS_PREFIX_VALUE);
        VFC_STATIC_ASSERT(UnitInfo1Type::TIME_PREFIX_VALUE == UnitInfo2Type::TIME_PREFIX_VALUE);
        VFC_STATIC_ASSERT(UnitInfo1Type::CURRENT_PREFIX_VALUE == UnitInfo2Type::CURRENT_PREFIX_VALUE);
        VFC_STATIC_ASSERT(UnitInfo1Type::TEMPERATURE_PREFIX_VALUE == UnitInfo2Type::TEMPERATURE_PREFIX_VALUE);
        VFC_STATIC_ASSERT(UnitInfo1Type::AMOUNTOFSUBSTANCE_PREFIX_VALUE == UnitInfo2Type::AMOUNTOFSUBSTANCE_PREFIX_VALUE);
        VFC_STATIC_ASSERT(UnitInfo1Type::LUMINOUSINTENSITY_PREFIX_VALUE == UnitInfo2Type::LUMINOUSINTENSITY_PREFIX_VALUE);
        VFC_STATIC_ASSERT(UnitInfo1Type::ANGLE_PREFIX_VALUE == UnitInfo2Type::ANGLE_PREFIX_VALUE);

        template<class ValueType>
        inline static void performValueConversion(ValueType&)
        {

        }
    };

    template<class Unit1Type, class Unit2Type, class DataType>
    class TDeduceOffset
    {
    public:
        //base type for both siunits should be same
        VFC_STATIC_ASSERT((vfc::TIsSameType<typename Unit1Type::si_base_unit_type,
            typename Unit2Type::si_base_unit_type>::value==1));

        //check for the interger data type
        enum { INTEGER_DATA_TYPE = vfc::TIsSameType<DataType, CIntegralType>::value };

        //calculation of interger offset
        static const integer_type OFFSET =((INTEGER_DATA_TYPE == 1) ?
            //if siunit type is interger, then calculate the difference of interger OFFSET
            (Unit1Type::OFFSET - Unit2Type::OFFSET) :
        //else check if both offsets are float, then resultant interger offset is zero
        (((Unit1Type::FLOATING_OFFSET == 1 ) && (Unit2Type::FLOATING_OFFSET == 1 )) ? 0 :
            //else check if unit1type's offset is float, then make it zero, and take the difference
            ((Unit1Type::FLOATING_OFFSET == 1 ) ? (0 - Unit2Type::OFFSET) :
            //else check if unit2type's offset is float, then make it zero, and take the difference
            ((Unit2Type::FLOATING_OFFSET == 1 ) ? (Unit1Type::OFFSET - 0) :
            //else calculate the difference of both interger OFFSET
            (Unit1Type::OFFSET - Unit2Type::OFFSET)) )));
    };

    template<class UnitInfo1Type, class UnitInfo2Type, class DataType>
    class TSumOffsets
    {
    public:
        //Summation of all interger offset
        static const integer_type OFFSET =
            (TDeduceOffset<typename UnitInfo1Type::length_unit_type,
            typename UnitInfo2Type::length_unit_type,             DataType>::OFFSET +
            TDeduceOffset<typename UnitInfo1Type::mass_unit_type,
            typename UnitInfo2Type::mass_unit_type,               DataType>::OFFSET +
            TDeduceOffset<typename UnitInfo1Type::time_unit_type,
            typename UnitInfo2Type::time_unit_type,               DataType>::OFFSET +
            TDeduceOffset<typename UnitInfo1Type::current_unit_type,
            typename UnitInfo2Type::current_unit_type,            DataType>::OFFSET +
            TDeduceOffset<typename UnitInfo1Type::temperature_unit_type,
            typename UnitInfo2Type::temperature_unit_type,        DataType>::OFFSET +
            TDeduceOffset<typename UnitInfo1Type::amountofsubstance_unit_type,
            typename UnitInfo2Type::amountofsubstance_unit_type,  DataType>::OFFSET +
            TDeduceOffset<typename UnitInfo1Type::luminousintensity_unit_type,
            typename UnitInfo2Type::luminousintensity_unit_type,  DataType>::OFFSET +
            TDeduceOffset<typename UnitInfo1Type::angle_unit_type,
            typename UnitInfo2Type::angle_unit_type,              DataType>::OFFSET +
            TDeduceOffset<typename UnitInfo1Type::unit_type,
            typename UnitInfo2Type::unit_type,                    DataType>::OFFSET);
    };


    template<class UnitInfo1Type, class UnitInfo2Type>
    class TSICompatibleCheck
    {
    public:
        static void check()
        {
            //static asserts for verifying if the powers match
            VFC_STATIC_ASSERT(UnitInfo1Type::LENGTH_POWER_VALUE == UnitInfo2Type::LENGTH_POWER_VALUE);
            VFC_STATIC_ASSERT(UnitInfo1Type::TIME_POWER_VALUE == UnitInfo2Type::TIME_POWER_VALUE);
            VFC_STATIC_ASSERT(UnitInfo1Type::MASS_POWER_VALUE == UnitInfo2Type::MASS_POWER_VALUE);
            VFC_STATIC_ASSERT(UnitInfo1Type::CURRENT_POWER_VALUE == UnitInfo2Type::CURRENT_POWER_VALUE);
            VFC_STATIC_ASSERT(UnitInfo1Type::TEMPERATURE_POWER_VALUE == UnitInfo2Type::TEMPERATURE_POWER_VALUE);
            VFC_STATIC_ASSERT(UnitInfo1Type::AMOUNTOFSUBSTANCE_POWER_VALUE ==UnitInfo2Type::AMOUNTOFSUBSTANCE_POWER_VALUE);
            VFC_STATIC_ASSERT(UnitInfo1Type::LUMINOUSINTENSITY_POWER_VALUE ==UnitInfo2Type::LUMINOUSINTENSITY_POWER_VALUE);
            VFC_STATIC_ASSERT(UnitInfo1Type::ANGLE_POWER_VALUE == UnitInfo2Type::ANGLE_POWER_VALUE);
        }
    };

    template<class UnitInfo1Type, class UnitInfo2Type, class DataType>
    class TConvert
    {
    public:
        template<class ValueType>
        inline static void performValueConversion(ValueType& f_value_r)
        {
            self_type::performValueConversionIntern(f_value_r, DataType());
        }

    private:
        typedef TConvert<UnitInfo1Type, UnitInfo2Type, DataType> self_type;

        static const integer_type RESULTANT_LENGTH = UnitInfo1Type::LENGTH_PREFIX_VALUE -
            UnitInfo2Type::LENGTH_PREFIX_VALUE;

        static const integer_type RESULTANT_MASS = UnitInfo1Type::MASS_PREFIX_VALUE -
            UnitInfo2Type::MASS_PREFIX_VALUE;

        static const integer_type RESULTANT_CURRENT = UnitInfo1Type::CURRENT_PREFIX_VALUE -
            UnitInfo2Type::CURRENT_PREFIX_VALUE;

        static const integer_type RESULTANT_TEMPERATURE = UnitInfo1Type::TEMPERATURE_PREFIX_VALUE -
            UnitInfo2Type::TEMPERATURE_PREFIX_VALUE;

        static const integer_type RESULTANT_AMOUNTOFSUBSTANCE = UnitInfo1Type::AMOUNTOFSUBSTANCE_PREFIX_VALUE -
            UnitInfo2Type::AMOUNTOFSUBSTANCE_PREFIX_VALUE;

        static const integer_type RESULTANT_LUMINOUSINTENSITY = UnitInfo1Type::LUMINOUSINTENSITY_PREFIX_VALUE -
            UnitInfo2Type::LUMINOUSINTENSITY_PREFIX_VALUE;

        static const integer_type RESULTANT_ANGLE = UnitInfo1Type::ANGLE_PREFIX_VALUE -
            UnitInfo2Type::ANGLE_PREFIX_VALUE;

        static const integer_type POWER_VALUE =
            (self_type::RESULTANT_LENGTH * UnitInfo1Type::LENGTH_POWER_VALUE) +
            (self_type::RESULTANT_MASS * UnitInfo1Type::MASS_POWER_VALUE) +
            (self_type::RESULTANT_CURRENT * UnitInfo1Type::CURRENT_POWER_VALUE) +
            (self_type::RESULTANT_TEMPERATURE * UnitInfo1Type::TEMPERATURE_POWER_VALUE) +
            (self_type::RESULTANT_AMOUNTOFSUBSTANCE * UnitInfo1Type::AMOUNTOFSUBSTANCE_POWER_VALUE) +
            (self_type::RESULTANT_LUMINOUSINTENSITY * UnitInfo1Type::LUMINOUSINTENSITY_POWER_VALUE) +
            (self_type::RESULTANT_ANGLE * UnitInfo1Type::ANGLE_POWER_VALUE);

        static const integer_type OFFSET =
            TSumOffsets<UnitInfo1Type, UnitInfo2Type, DataType>::OFFSET;

        static const integer_type NUM = TFactorizeInputs<
            TUnitConversion<typename UnitInfo1Type::length_unit_type,
            typename UnitInfo2Type::length_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::length_unit_type,
            typename UnitInfo2Type::length_unit_type, TBasicTypeConvert, DataType>::DEN,
            TUnitConversion<typename UnitInfo1Type::mass_unit_type,
            typename UnitInfo2Type::mass_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::mass_unit_type,
            typename UnitInfo2Type::mass_unit_type, TBasicTypeConvert, DataType>::DEN,
            TUnitConversion<typename UnitInfo1Type::time_unit_type,
            typename UnitInfo2Type::time_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::time_unit_type,
            typename UnitInfo2Type::time_unit_type, TBasicTypeConvert, DataType>::DEN,
            TUnitConversion<typename UnitInfo1Type::current_unit_type,
            typename UnitInfo2Type::current_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::current_unit_type,
            typename UnitInfo2Type::current_unit_type, TBasicTypeConvert, DataType>::DEN,
            TUnitConversion<typename UnitInfo1Type::temperature_unit_type,
            typename UnitInfo2Type::temperature_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::temperature_unit_type,
            typename UnitInfo2Type::temperature_unit_type, TBasicTypeConvert, DataType>::DEN,
            TUnitConversion<typename UnitInfo1Type::amountofsubstance_unit_type,
            typename UnitInfo2Type::amountofsubstance_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::amountofsubstance_unit_type,
            typename UnitInfo2Type::amountofsubstance_unit_type, TBasicTypeConvert, DataType>::DEN,
            TUnitConversion<typename UnitInfo1Type::luminousintensity_unit_type,
            typename UnitInfo2Type::luminousintensity_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::luminousintensity_unit_type,
            typename UnitInfo2Type::luminousintensity_unit_type, TBasicTypeConvert, DataType>::DEN,
            TUnitConversion<typename UnitInfo1Type::angle_unit_type,
            typename UnitInfo2Type::angle_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::angle_unit_type,
            typename UnitInfo2Type::angle_unit_type, TBasicTypeConvert, DataType>::DEN,
            sitime::TConvertTime<UnitInfo1Type::TIME_POWER_VALUE,
            UnitInfo1Type::TIME_PREFIX_VALUE, UnitInfo2Type::TIME_PREFIX_VALUE>::NUM,
            sitime::TConvertTime<UnitInfo1Type::TIME_POWER_VALUE,
            UnitInfo1Type::TIME_PREFIX_VALUE, UnitInfo2Type::TIME_PREFIX_VALUE>::DEN,
            TPower<10, TNumDenPower<self_type::POWER_VALUE>::NUM>::VALUE,
            TPower<10, TNumDenPower<self_type::POWER_VALUE>::DEN>::VALUE,
            TUnitConversion<typename UnitInfo1Type::unit_type,
            typename UnitInfo2Type::unit_type, TComplexConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::unit_type,
            typename UnitInfo2Type::unit_type, TComplexConvert, DataType>::DEN>::NUM;

        static const integer_type DEN = TFactorizeInputs<
            TUnitConversion<typename UnitInfo1Type::length_unit_type,
            typename UnitInfo2Type::length_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::length_unit_type,
            typename UnitInfo2Type::length_unit_type, TBasicTypeConvert, DataType>::DEN,
            TUnitConversion<typename UnitInfo1Type::mass_unit_type,
            typename UnitInfo2Type::mass_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::mass_unit_type,
            typename UnitInfo2Type::mass_unit_type, TBasicTypeConvert, DataType>::DEN,
            TUnitConversion<typename UnitInfo1Type::time_unit_type,
            typename UnitInfo2Type::time_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::time_unit_type,
            typename UnitInfo2Type::time_unit_type, TBasicTypeConvert, DataType>::DEN,
            TUnitConversion<typename UnitInfo1Type::current_unit_type,
            typename UnitInfo2Type::current_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::current_unit_type,
            typename UnitInfo2Type::current_unit_type, TBasicTypeConvert, DataType>::DEN,
            TUnitConversion<typename UnitInfo1Type::temperature_unit_type,
            typename UnitInfo2Type::temperature_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::temperature_unit_type,
            typename UnitInfo2Type::temperature_unit_type, TBasicTypeConvert, DataType>::DEN,
            TUnitConversion<typename UnitInfo1Type::amountofsubstance_unit_type,
            typename UnitInfo2Type::amountofsubstance_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::amountofsubstance_unit_type,
            typename UnitInfo2Type::amountofsubstance_unit_type, TBasicTypeConvert, DataType>::DEN,
            TUnitConversion<typename UnitInfo1Type::luminousintensity_unit_type,
            typename UnitInfo2Type::luminousintensity_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::luminousintensity_unit_type,
            typename UnitInfo2Type::luminousintensity_unit_type, TBasicTypeConvert, DataType>::DEN,
            TUnitConversion<typename UnitInfo1Type::angle_unit_type,
            typename UnitInfo2Type::angle_unit_type, TBasicTypeConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::angle_unit_type,
            typename UnitInfo2Type::angle_unit_type, TBasicTypeConvert, DataType>::DEN,
            sitime::TConvertTime<UnitInfo1Type::TIME_POWER_VALUE,
            UnitInfo1Type::TIME_PREFIX_VALUE, UnitInfo2Type::TIME_PREFIX_VALUE>::NUM,
            sitime::TConvertTime<UnitInfo1Type::TIME_POWER_VALUE,
            UnitInfo1Type::TIME_PREFIX_VALUE, UnitInfo2Type::TIME_PREFIX_VALUE>::DEN,
            TPower<10, TNumDenPower<self_type::POWER_VALUE>::NUM>::VALUE,
            TPower<10, TNumDenPower<self_type::POWER_VALUE>::DEN>::VALUE,
            TUnitConversion<typename UnitInfo1Type::unit_type,
            typename UnitInfo2Type::unit_type, TComplexConvert, DataType>::NUM,
            TUnitConversion<typename UnitInfo1Type::unit_type,
            typename UnitInfo2Type::unit_type, TComplexConvert, DataType>::DEN>::DEN;

        //verify & assert if types involve both OFFSET & FACTORIZATION calculation.
        //example Kilogram/Celsius to Gram/Kelvin
        VFC_STATIC_ASSERT(!((OFFSET != 0) && ((NUM!=1) || (DEN!=1))));

        //for integers
        template<class ValueType>
        inline static void performValueConversionIntern(ValueType& f_value_r, CIntegralType)
        {
            //function to perform factorization calculation
            performValueIntegralConversionIntern(f_value_r,
                typename vfc::TInt2Boolean<(self_type::NUM==self_type::DEN)>::type());
            //function to perform offset calculation
            performValueIntegralOffsetIntern(f_value_r,
                typename vfc::TInt2Boolean<(self_type::OFFSET==0)>::type());
        }

        template<class ValueType>
        inline static void performValueIntegralOffsetIntern(ValueType&,
            vfc::true_t)
        {
            //intentionally left blank
        }

        template<class ValueType>
        inline static void performValueIntegralOffsetIntern(ValueType& f_value_r,
            vfc::false_t)
        {
            f_value_r += OFFSET;
        }


        //for integers
        template<class ValueType>
        inline static void performValueIntegralConversionIntern(ValueType&,
            vfc::true_t)
        {
            //intentionally left blank
        }

        //for integers
        template<class ValueType>
        inline static void performValueIntegralConversionIntern(ValueType& f_value_r,
            vfc::false_t)
        {
            //verify & assert if there is an overflow

            VFC_ASSERT((vfc::abs(f_value_r) < ((std::numeric_limits<ValueType>::max)() /
                static_cast<ValueType>(NUM))));

            doMuliplyNum(f_value_r,
                typename vfc::TInt2Boolean<(self_type::NUM!=1)>::type());

            doDivideDen(f_value_r,
                typename vfc::TInt2Boolean<(self_type::DEN!=1)>::type());

        }

        template<class ValueType>
        inline static void doMuliplyNum(ValueType& f_value_r,
            vfc::true_t)
        {
            f_value_r *= static_cast<ValueType>(self_type::NUM);
        }

        template<class ValueType>
        inline static void doMuliplyNum(ValueType&,
            vfc::false_t)
        {
            //intentionally left blank
        }

        template<class ValueType>
        inline static void doDivideDen(ValueType& f_value_r,
            vfc::true_t)
        {
            f_value_r /= static_cast<ValueType>(self_type::DEN);
        }

        template<class ValueType>
        inline static void doDivideDen(ValueType&,
            vfc::false_t)
        {
            //intentionally left blank
        }

        //for floating
        template<class ValueType>
        inline static void performValueConversionIntern(ValueType& f_value_r, CFloatingType)
        {
            //function to perform factorization calculation
            performValueFloatingConversionIntern(f_value_r,
                typename vfc::TInt2Boolean<(self_type::NUM==self_type::DEN)>::type());

            //function to perform offset calculation
            performValueFloatingOffsetIntern(f_value_r,
                typename vfc::TInt2Boolean<(self_type::OFFSET==0)>::type());

        }

        //for integers
        template<class ValueType>
        inline static void performValueFloatingConversionIntern(ValueType& f_value_r,
            vfc::true_t)
        {
            self_type::performVaraiantConversion(f_value_r);
            self_type::performComplexConversion(f_value_r);
        }

        //for integers
        template<class ValueType>
        inline static void performValueFloatingConversionIntern(ValueType& f_value_r,
            vfc::false_t)
        {
            integer_type num = self_type::NUM;
            integer_type den = self_type::DEN;

            f_value_r *= (static_cast<ValueType>(num) /
                static_cast<ValueType>(den));

            self_type::performVaraiantConversion(f_value_r);
            self_type::performComplexConversion(f_value_r);
        }

        template<class ValueType>
        inline static void performValueFloatingOffsetIntern(ValueType&,
            vfc::true_t)
        {
            //intentionally left blank
        }

        template<class ValueType>
        inline static void performValueFloatingOffsetIntern(ValueType& f_value_r,
            vfc::false_t)
        {
            f_value_r += OFFSET;
        }

        template<class ValueType>
        inline static void performVaraiantConversion(ValueType& f_value_r)
        {
            TUnitConversion<typename UnitInfo1Type::length_unit_type,
                typename UnitInfo2Type::length_unit_type,
                TBasicTypeConvert, DataType>::performConversion(f_value_r);
            TUnitConversion<typename UnitInfo1Type::mass_unit_type,
                typename UnitInfo2Type::mass_unit_type,
                TBasicTypeConvert, DataType>::performConversion(f_value_r);
            TUnitConversion<typename UnitInfo1Type::current_unit_type,
                typename UnitInfo2Type::current_unit_type,
                TBasicTypeConvert, DataType>::performConversion(f_value_r);
            TUnitConversion<typename UnitInfo1Type::temperature_unit_type,
                typename UnitInfo2Type::temperature_unit_type,
                TBasicTypeConvert, DataType>::performConversion(f_value_r);
            TUnitConversion<typename UnitInfo1Type::amountofsubstance_unit_type,
                typename UnitInfo2Type::amountofsubstance_unit_type,
                TBasicTypeConvert, DataType>::performConversion(f_value_r);
            TUnitConversion<typename UnitInfo1Type::luminousintensity_unit_type,
                typename UnitInfo2Type::luminousintensity_unit_type,
                TBasicTypeConvert, DataType>::performConversion(f_value_r);
            TUnitConversion<typename UnitInfo1Type::angle_unit_type,
                typename UnitInfo2Type::angle_unit_type,
                TBasicTypeConvert, DataType>::performConversion(f_value_r);
        }

        template<class ValueType>
        inline static void performComplexConversion(ValueType& f_value_r)
        {
            TUnitConversion<typename UnitInfo1Type::unit_type, typename UnitInfo2Type::unit_type,
                TComplexConvert, DataType>::performConversion(f_value_r);
        }

    };

    template<class RBInfo1Type, class RBInfo2Type, class DataType>
    class TRBConvert
    {
    public:

        typedef TRBConvert<RBInfo1Type, RBInfo2Type, DataType> self_type;

        //static assserts for verifying if the powers match
        VFC_STATIC_ASSERT(RBInfo1Type::PIXEL_POWER_VALUE == RBInfo2Type::PIXEL_POWER_VALUE);
        VFC_STATIC_ASSERT(RBInfo1Type::PERCENTAGE_POWER_VALUE == RBInfo2Type::PERCENTAGE_POWER_VALUE);

        static const integer_type RESULTANT_PIXLE = RBInfo1Type::PIXEL_PREFIX_VALUE -
            RBInfo2Type::PIXEL_PREFIX_VALUE;

        static const integer_type RESULTANT_PERCENTAGE = RBInfo1Type::PERCENTAGE_PREFIX_VALUE -
            RBInfo2Type::PERCENTAGE_PREFIX_VALUE;

        static const integer_type POWER_VALUE =(self_type::RESULTANT_PIXLE * RBInfo1Type::PIXEL_POWER_VALUE)
            + (self_type::RESULTANT_PERCENTAGE * RBInfo1Type::PERCENTAGE_POWER_VALUE) ;

        template<class ValueType>
        inline static void performValueConversion(ValueType& f_value_r)
        {
        }

    };

    template<
        class           LengthUnitType,
            vfc::int32_t    LengthPrefixValue,
            vfc::int32_t    LengthPowerValue,
        class           MassUnitType,
            vfc::int32_t    MassPrefixValue,
            vfc::int32_t    MassPowerValue,
        class           TimeUnitType,
            vfc::int32_t    TimePrefixValue,
            vfc::int32_t    TimePowerValue,
        class           CurrentUnitType,
            vfc::int32_t    CurrentPrefixValue,
            vfc::int32_t    CurrentPowerValue,
        class           TemperatureUnitType,
            vfc::int32_t    TemperaturePrefixValue,
            vfc::int32_t    TemperaturePowerValue,
        class           AmountOfSubstanceUnitType,
            vfc::int32_t    AmountOfSubstancePrefixValue,
            vfc::int32_t    AmountOfSubstancePowerValue,
        class           LuminousIntensityUnitType,
            vfc::int32_t    LuminousIntensityPrefixValue,
            vfc::int32_t    LuminousIntensityPowerValue,
        class           AngleUnitType,
            vfc::int32_t    AnglePrefixValue,
            vfc::int32_t    AnglePowerValue,
        class           UnitType = CBasicType
    >
    class TUnitInfoType
    {
    public:
        typedef TUnitInfoType<
            LengthUnitType, LengthPrefixValue, LengthPowerValue,
            MassUnitType, MassPrefixValue, MassPowerValue,
            TimeUnitType, TimePrefixValue, TimePowerValue,
            CurrentUnitType, CurrentPrefixValue, CurrentPowerValue,
            TemperatureUnitType, TemperaturePrefixValue, TemperaturePowerValue,
            AmountOfSubstanceUnitType, AmountOfSubstancePrefixValue, AmountOfSubstancePowerValue,
            LuminousIntensityUnitType, LuminousIntensityPrefixValue, LuminousIntensityPowerValue,
            AngleUnitType, AnglePrefixValue, AnglePowerValue
        >                                                           self_unit_type;

        typedef TUnitInfoType<
            LengthUnitType, LengthPrefixValue, LengthPowerValue,
            MassUnitType, MassPrefixValue, MassPowerValue,
            TimeUnitType, TimePrefixValue, TimePowerValue,
            CurrentUnitType, CurrentPrefixValue, CurrentPowerValue,
            TemperatureUnitType, TemperaturePrefixValue, TemperaturePowerValue,
            AmountOfSubstanceUnitType, AmountOfSubstancePrefixValue, AmountOfSubstancePowerValue,
            LuminousIntensityUnitType, LuminousIntensityPrefixValue, LuminousIntensityPowerValue,
            AngleUnitType, AnglePrefixValue, AnglePowerValue,
            UnitType>                                                   base_unit_type;

        enum
        {
            SI_UNIT = 1
        };

        enum
        {
            SI_FUNDAMENTAL = vfc::TIsSameType<self_unit_type, base_unit_type>::value
        };

        typedef LengthUnitType              length_unit_type;
        typedef MassUnitType                mass_unit_type;
        typedef TimeUnitType                time_unit_type;
        typedef CurrentUnitType             current_unit_type;
        typedef TemperatureUnitType         temperature_unit_type;
        typedef AmountOfSubstanceUnitType   amountofsubstance_unit_type;
        typedef LuminousIntensityUnitType   luminousintensity_unit_type;
        typedef AngleUnitType               angle_unit_type;

        typedef UnitType        unit_type;

        static const vfc::int32_t LENGTH_PREFIX_VALUE = LengthPrefixValue;
        static const vfc::int32_t LENGTH_POWER_VALUE = LengthPowerValue;

        static const vfc::int32_t MASS_PREFIX_VALUE = MassPrefixValue;
        static const vfc::int32_t MASS_POWER_VALUE = MassPowerValue;

        static const vfc::int32_t TIME_PREFIX_VALUE = TimePrefixValue;
        static const vfc::int32_t TIME_POWER_VALUE = TimePowerValue;

        static const vfc::int32_t CURRENT_PREFIX_VALUE = CurrentPrefixValue;
        static const vfc::int32_t CURRENT_POWER_VALUE = CurrentPowerValue;

        static const vfc::int32_t TEMPERATURE_PREFIX_VALUE = TemperaturePrefixValue;
        static const vfc::int32_t TEMPERATURE_POWER_VALUE = TemperaturePowerValue;

        static const vfc::int32_t AMOUNTOFSUBSTANCE_PREFIX_VALUE = AmountOfSubstancePrefixValue;
        static const vfc::int32_t AMOUNTOFSUBSTANCE_POWER_VALUE = AmountOfSubstancePowerValue;

        static const vfc::int32_t LUMINOUSINTENSITY_PREFIX_VALUE = LuminousIntensityPrefixValue;
        static const vfc::int32_t LUMINOUSINTENSITY_POWER_VALUE = LuminousIntensityPowerValue;

        static const vfc::int32_t ANGLE_PREFIX_VALUE = AnglePrefixValue;
        static const vfc::int32_t ANGLE_POWER_VALUE = AnglePowerValue;


    };

        class CPercentage { };


    template<
        class           PixelUnitType,
            vfc::int32_t    PixelPrefixValue,
            vfc::int32_t    PixelPowerValue,
        class           PercentageUnitType,
            vfc::int32_t    PercentagePrefixValue,
            vfc::int32_t    PercentagePowerValue,
        class           UnitType = CBasicType
    >
    class TRBInfoType
    {
    public:
        typedef TRBInfoType<
            PixelUnitType, PixelPrefixValue, PixelPowerValue,
            PercentageUnitType,PercentagePrefixValue,PercentagePowerValue
        >                                                           self_unit_type;

        typedef TRBInfoType<
            PixelUnitType, PixelPrefixValue, PixelPowerValue,
            PercentageUnitType,PercentagePrefixValue,PercentagePowerValue,
            UnitType>                                                   base_unit_type;

        enum
        {
            SI_UNIT = 1
        };

        enum
        {
            SI_FUNDAMENTAL = vfc::TIsSameType<self_unit_type, base_unit_type>::value
        };

        typedef PixelUnitType              pixel_unit_type;
        typedef PercentageUnitType         percentage_unit_type;
        typedef UnitType        unit_type;

        static const vfc::int32_t PIXEL_PREFIX_VALUE = PixelPrefixValue;
        static const vfc::int32_t PIXEL_POWER_VALUE = PixelPowerValue;
        static const vfc::int32_t PERCENTAGE_PREFIX_VALUE = PercentagePrefixValue;
        static const vfc::int32_t PERCENTAGE_POWER_VALUE = PercentagePowerValue;
    };

    struct CBar
    {
        enum
        {
            CONVERSION_RATIONAL = 1,
            FLOATING_OFFSET = 0
        };

        static const integer_type NUM = 10000;
        static const integer_type DEN = 1;
        static const integer_type OFFSET = 0;
    };

    struct CGForce
    {
        enum
        {
            CONVERSION_RATIONAL = 1,
            FLOATING_OFFSET = 0
        };

        static const integer_type NUM = 981;
        static const integer_type DEN = 100;
        static const integer_type OFFSET = 0;

    };

    // si-unit operation sine and cosine must use unit promotion
    template<class RBInfoType>
    struct TSIUnitRBSinCosPromote
    {
        typedef vfc::TRBInfoType<
            typename RBInfoType::pixel_unit_type,
            RBInfoType::PIXEL_PREFIX_VALUE,
            (RBInfoType::PIXEL_POWER_VALUE),
            typename RBInfoType::percentage_unit_type,
             RBInfoType::PERCENTAGE_PREFIX_VALUE,
            (RBInfoType::PERCENTAGE_POWER_VALUE),
            typename RBInfoType::unit_type>  unit_info_type;
    };

    // si-unit operation sine and cosine must use unit promotion
    template<class UnitInfo1Type>
    struct TSIUnitSinCosPromote
    {
        typedef vfc::TUnitInfoType<
            typename UnitInfo1Type::length_unit_type,
            UnitInfo1Type::LENGTH_PREFIX_VALUE,
            (UnitInfo1Type::LENGTH_POWER_VALUE),
            typename UnitInfo1Type::mass_unit_type,
            UnitInfo1Type::MASS_PREFIX_VALUE,
            (UnitInfo1Type::MASS_POWER_VALUE),
            typename UnitInfo1Type::time_unit_type,
            UnitInfo1Type::TIME_PREFIX_VALUE,
            (UnitInfo1Type::TIME_POWER_VALUE),
            typename UnitInfo1Type::current_unit_type,
            UnitInfo1Type::CURRENT_PREFIX_VALUE,
            (UnitInfo1Type::CURRENT_POWER_VALUE),
            typename UnitInfo1Type::temperature_unit_type,
            UnitInfo1Type::TEMPERATURE_PREFIX_VALUE,
            (UnitInfo1Type::TEMPERATURE_POWER_VALUE),
            typename UnitInfo1Type::amountofsubstance_unit_type,
            UnitInfo1Type::AMOUNTOFSUBSTANCE_PREFIX_VALUE,
            (UnitInfo1Type::AMOUNTOFSUBSTANCE_POWER_VALUE),
            typename UnitInfo1Type::luminousintensity_unit_type,
            UnitInfo1Type::LUMINOUSINTENSITY_PREFIX_VALUE,
            (UnitInfo1Type::LUMINOUSINTENSITY_POWER_VALUE),
            typename UnitInfo1Type::angle_unit_type,
            UnitInfo1Type::ANGLE_PREFIX_VALUE,
            (UnitInfo1Type::ANGLE_POWER_VALUE - 1),             // turns from angle to slope
            typename UnitInfo1Type::unit_type>  unit_info_type;
    };

    // si-unit operation arcus tangent must use unit promotion
    template<class RBInfoType>
    struct TSIUnitRBAtanPromote
    {
        typedef vfc::TRBInfoType<
            typename RBInfoType::pixel_unit_type,
            RBInfoType::PIXEL_PREFIX_VALUE,
            (RBInfoType::PIXEL_POWER_VALUE),
            typename RBInfoType::percentage_unit_type,
             RBInfoType::PERCENTAGE_PREFIX_VALUE,
            (RBInfoType::PERCENTAGE_POWER_VALUE),
            typename RBInfoType::unit_type> unit_info_type;
    };

    // si-unit operation arcus tangent must use unit promotion
    template<class UnitInfo1Type>
    struct TSIUnitAtanPromote
    {
        typedef vfc::TUnitInfoType<
            typename UnitInfo1Type::length_unit_type,
            UnitInfo1Type::LENGTH_PREFIX_VALUE,
            (UnitInfo1Type::LENGTH_POWER_VALUE),
            typename UnitInfo1Type::mass_unit_type,
            UnitInfo1Type::MASS_PREFIX_VALUE,
            (UnitInfo1Type::MASS_POWER_VALUE),
            typename UnitInfo1Type::time_unit_type,
            UnitInfo1Type::TIME_PREFIX_VALUE,
            (UnitInfo1Type::TIME_POWER_VALUE),
            typename UnitInfo1Type::current_unit_type,
            UnitInfo1Type::CURRENT_PREFIX_VALUE,
            (UnitInfo1Type::CURRENT_POWER_VALUE),
            typename UnitInfo1Type::temperature_unit_type,
            UnitInfo1Type::TEMPERATURE_PREFIX_VALUE,
            (UnitInfo1Type::TEMPERATURE_POWER_VALUE),
            typename UnitInfo1Type::amountofsubstance_unit_type,
            UnitInfo1Type::AMOUNTOFSUBSTANCE_PREFIX_VALUE,
            (UnitInfo1Type::AMOUNTOFSUBSTANCE_POWER_VALUE),
            typename UnitInfo1Type::luminousintensity_unit_type,
            UnitInfo1Type::LUMINOUSINTENSITY_PREFIX_VALUE,
            (UnitInfo1Type::LUMINOUSINTENSITY_POWER_VALUE),
            typename UnitInfo1Type::angle_unit_type,
            UnitInfo1Type::ANGLE_PREFIX_VALUE,
            (UnitInfo1Type::ANGLE_POWER_VALUE + 1), // turns from slope to angle
            typename UnitInfo1Type::unit_type> unit_info_type;
    }; 

    template<class UnitInfo1Type, class UnitInfo2Type>
    struct TSIUnitMuliplicationPromote
    {
        typedef vfc::TUnitInfoType<
            typename UnitInfo1Type::length_unit_type,
            UnitInfo1Type::LENGTH_PREFIX_VALUE,
            (UnitInfo1Type::LENGTH_POWER_VALUE + UnitInfo2Type::LENGTH_POWER_VALUE),
            typename UnitInfo1Type::mass_unit_type,
            UnitInfo1Type::MASS_PREFIX_VALUE,
            (UnitInfo1Type::MASS_POWER_VALUE + UnitInfo2Type::MASS_POWER_VALUE),
            typename UnitInfo1Type::time_unit_type,
            UnitInfo1Type::TIME_PREFIX_VALUE,
            (UnitInfo1Type::TIME_POWER_VALUE + UnitInfo2Type::TIME_POWER_VALUE),
            typename UnitInfo1Type::current_unit_type,
            UnitInfo1Type::CURRENT_PREFIX_VALUE,
            (UnitInfo1Type::CURRENT_POWER_VALUE + UnitInfo2Type::CURRENT_POWER_VALUE),
            typename UnitInfo1Type::temperature_unit_type,
            UnitInfo1Type::TEMPERATURE_PREFIX_VALUE,
            (UnitInfo1Type::TEMPERATURE_POWER_VALUE + UnitInfo2Type::TEMPERATURE_POWER_VALUE),
            typename UnitInfo1Type::amountofsubstance_unit_type,
            UnitInfo1Type::AMOUNTOFSUBSTANCE_PREFIX_VALUE,
            (UnitInfo1Type::AMOUNTOFSUBSTANCE_POWER_VALUE + UnitInfo2Type::AMOUNTOFSUBSTANCE_POWER_VALUE),
            typename UnitInfo1Type::luminousintensity_unit_type,
            UnitInfo1Type::LUMINOUSINTENSITY_PREFIX_VALUE,
            (UnitInfo1Type::LUMINOUSINTENSITY_POWER_VALUE + UnitInfo2Type::LUMINOUSINTENSITY_POWER_VALUE),
            typename vfc::TIf<(UnitInfo1Type::ANGLE_POWER_VALUE == 0), typename UnitInfo2Type::angle_unit_type,
                                                                       typename UnitInfo1Type::angle_unit_type>::type,
            UnitInfo1Type::ANGLE_PREFIX_VALUE,
            (UnitInfo1Type::ANGLE_POWER_VALUE + UnitInfo2Type::ANGLE_POWER_VALUE) * (vfc::TIsMultPowAng<UnitInfo1Type, UnitInfo2Type>::value) ,
            typename UnitInfo1Type::unit_type>  unit_info_type;
    };

    template<class UnitInfo1Type, class UnitInfo2Type>
    struct TSIUnitDivisionPromote
    {
        typedef vfc::TUnitInfoType<
            typename UnitInfo1Type::length_unit_type,
            UnitInfo1Type::LENGTH_PREFIX_VALUE,
            (UnitInfo1Type::LENGTH_POWER_VALUE - UnitInfo2Type::LENGTH_POWER_VALUE),
            typename UnitInfo1Type::mass_unit_type,
            UnitInfo1Type::MASS_PREFIX_VALUE,
            (UnitInfo1Type::MASS_POWER_VALUE - UnitInfo2Type::MASS_POWER_VALUE),
            typename UnitInfo1Type::time_unit_type,
            UnitInfo1Type::TIME_PREFIX_VALUE,
            (UnitInfo1Type::TIME_POWER_VALUE - UnitInfo2Type::TIME_POWER_VALUE),
            typename UnitInfo1Type::current_unit_type,
            UnitInfo1Type::CURRENT_PREFIX_VALUE,
            (UnitInfo1Type::CURRENT_POWER_VALUE - UnitInfo2Type::CURRENT_POWER_VALUE),
            typename UnitInfo1Type::temperature_unit_type,
            UnitInfo1Type::TEMPERATURE_PREFIX_VALUE,
            (UnitInfo1Type::TEMPERATURE_POWER_VALUE - UnitInfo2Type::TEMPERATURE_POWER_VALUE),
            typename UnitInfo1Type::amountofsubstance_unit_type,
            UnitInfo1Type::AMOUNTOFSUBSTANCE_PREFIX_VALUE,
            (UnitInfo1Type::AMOUNTOFSUBSTANCE_POWER_VALUE - UnitInfo2Type::AMOUNTOFSUBSTANCE_POWER_VALUE),
            typename UnitInfo1Type::luminousintensity_unit_type,
            UnitInfo1Type::LUMINOUSINTENSITY_PREFIX_VALUE,
            (UnitInfo1Type::LUMINOUSINTENSITY_POWER_VALUE - UnitInfo2Type::LUMINOUSINTENSITY_POWER_VALUE),
            typename vfc::TIf<(UnitInfo1Type::ANGLE_POWER_VALUE == 0), typename UnitInfo2Type::angle_unit_type, 
                                                                       typename UnitInfo1Type::angle_unit_type>::type,
            UnitInfo1Type::ANGLE_PREFIX_VALUE,
            (UnitInfo1Type::ANGLE_POWER_VALUE - UnitInfo2Type::ANGLE_POWER_VALUE) * (vfc::TIsDivPowAng<UnitInfo1Type, UnitInfo2Type>::value) ,
            typename UnitInfo1Type::unit_type>  unit_info_type;
    };

    template<class UnitInfoType>
    struct TSIUnitSqrPromote
    {
        typedef vfc::TUnitInfoType<
            typename UnitInfoType::length_unit_type,
            UnitInfoType::LENGTH_PREFIX_VALUE,
            (UnitInfoType::LENGTH_POWER_VALUE * 2),
            typename UnitInfoType::mass_unit_type,
            UnitInfoType::MASS_PREFIX_VALUE,
            (UnitInfoType::MASS_POWER_VALUE * 2),
            typename UnitInfoType::time_unit_type,
            UnitInfoType::TIME_PREFIX_VALUE,
            (UnitInfoType::TIME_POWER_VALUE * 2),
            typename UnitInfoType::current_unit_type,
            UnitInfoType::CURRENT_PREFIX_VALUE,
            (UnitInfoType::CURRENT_POWER_VALUE * 2),
            typename UnitInfoType::temperature_unit_type,
            UnitInfoType::TEMPERATURE_PREFIX_VALUE,
            (UnitInfoType::TEMPERATURE_POWER_VALUE * 2),
            typename UnitInfoType::amountofsubstance_unit_type,
            UnitInfoType::AMOUNTOFSUBSTANCE_PREFIX_VALUE,
            (UnitInfoType::AMOUNTOFSUBSTANCE_POWER_VALUE * 2),
            typename UnitInfoType::luminousintensity_unit_type,
            UnitInfoType::LUMINOUSINTENSITY_PREFIX_VALUE,
            (UnitInfoType::LUMINOUSINTENSITY_POWER_VALUE * 2),
            typename UnitInfoType::angle_unit_type,
            UnitInfoType::ANGLE_PREFIX_VALUE,
            (UnitInfoType::ANGLE_POWER_VALUE * 2),
            typename UnitInfoType::unit_type>  unit_info_type;
    };

    template<class UnitInfoType>
    struct TSIUnitSqrtPromote
    {
        typedef vfc::TUnitInfoType<
            typename UnitInfoType::length_unit_type,
            UnitInfoType::LENGTH_PREFIX_VALUE,
            (UnitInfoType::LENGTH_POWER_VALUE / 2),
            typename UnitInfoType::mass_unit_type,
            UnitInfoType::MASS_PREFIX_VALUE,
            (UnitInfoType::MASS_POWER_VALUE / 2),
            typename UnitInfoType::time_unit_type,
            UnitInfoType::TIME_PREFIX_VALUE,
            (UnitInfoType::TIME_POWER_VALUE / 2),
            typename UnitInfoType::current_unit_type,
            UnitInfoType::CURRENT_PREFIX_VALUE,
            (UnitInfoType::CURRENT_POWER_VALUE / 2),
            typename UnitInfoType::temperature_unit_type,
            UnitInfoType::TEMPERATURE_PREFIX_VALUE,
            (UnitInfoType::TEMPERATURE_POWER_VALUE / 2),
            typename UnitInfoType::amountofsubstance_unit_type,
            UnitInfoType::AMOUNTOFSUBSTANCE_PREFIX_VALUE,
            (UnitInfoType::AMOUNTOFSUBSTANCE_POWER_VALUE / 2),
            typename UnitInfoType::luminousintensity_unit_type,
            UnitInfoType::LUMINOUSINTENSITY_PREFIX_VALUE,
            (UnitInfoType::LUMINOUSINTENSITY_POWER_VALUE / 2),
            typename UnitInfoType::angle_unit_type,
            UnitInfoType::ANGLE_PREFIX_VALUE,
            (UnitInfoType::ANGLE_POWER_VALUE / 2),
            typename UnitInfoType::unit_type>  unit_info_type;
    };

    template<class RBInfo1Type, class RBInfo2Type>
    struct TSIUnitRBMuliplicationPromote
    {
        typedef vfc::TRBInfoType<
            typename RBInfo1Type::pixel_unit_type,
            RBInfo1Type::PIXEL_PREFIX_VALUE,
            (RBInfo1Type::PIXEL_POWER_VALUE + RBInfo2Type::PIXEL_POWER_VALUE),
            typename RBInfo1Type::percentage_unit_type,
             RBInfo1Type::PERCENTAGE_PREFIX_VALUE,
            (RBInfo1Type::PERCENTAGE_POWER_VALUE + RBInfo2Type::PERCENTAGE_POWER_VALUE),
            typename RBInfo1Type::unit_type>  unit_info_type;
    };

    template<class RBInfo1Type, class RBInfo2Type>
    struct TSIUnitRBDivisionPromote
    {
        typedef vfc::TRBInfoType<
            typename RBInfo1Type::pixel_unit_type,
            RBInfo1Type::PIXEL_PREFIX_VALUE,
            (RBInfo1Type::PIXEL_POWER_VALUE - RBInfo2Type::PIXEL_POWER_VALUE),
            typename RBInfo1Type::percentage_unit_type,
            RBInfo1Type::PERCENTAGE_PREFIX_VALUE,
            (RBInfo1Type::PERCENTAGE_POWER_VALUE - RBInfo2Type::PERCENTAGE_POWER_VALUE),
            typename RBInfo1Type::unit_type>  unit_info_type;
    };

    template<class RBInfoType>
    struct TSIUnitRBSqrPromote
    {
        typedef vfc::TRBInfoType<
            typename RBInfoType::pixel_unit_type,
            RBInfoType::PIXEL_PREFIX_VALUE,
            (RBInfoType::PIXEL_POWER_VALUE * 2),
            typename RBInfoType::percentage_unit_type,
            RBInfoType::PERCENTAGE_PREFIX_VALUE,
            (RBInfoType::PERCENTAGE_POWER_VALUE * 2),
            typename RBInfoType::unit_type>  unit_info_type;
    };

    template<class RBInfoType>
    struct TSIUnitRBSqrtPromote
    {
        typedef vfc::TRBInfoType<
            typename RBInfoType::pixel_unit_type,
            RBInfoType::PIXEL_PREFIX_VALUE,
            (RBInfoType::PIXEL_POWER_VALUE / 2),
            typename RBInfoType::percentage_unit_type,
            RBInfoType::PERCENTAGE_PREFIX_VALUE,
            (RBInfoType::PERCENTAGE_POWER_VALUE / 2),
            typename RBInfoType::unit_type>  unit_info_type;
    };

    template<class SIBaseUnitType>
    struct TSIType
    {
        //typedef TSIType<SIBaseUnitType> base_type;
        typedef SIBaseUnitType  si_base_unit_type;

        enum
        {
            CONVERSION_RATIONAL = 1,
            FLOATING_OFFSET = 0
        };
        static const vfc::int32_t NUM = 1;
        static const vfc::int32_t DEN = 1;
        static const vfc::int32_t OFFSET = 0;

    };

    //SI base units
    class CLength { };
    class CMass { };
    class CTime { };
    class CCurrent { };
    class CTemperature { };
    class CAmountOfSubstance { };
    class CLuminousIntensity { };
    class CAngle { };

    //RB Types
    class CPixel { };

    //User type for DIN70k coordinate system with base in middle of front axle
    class CDinFrontType { };
    //User type for DIN70k coordinate system with base in middle of rear axle
    class CDinRearType  { };
    //User type for DIN70k coordinate system with base in projection of middle of rear axle to the ground
    class CDinWorldType { };
    //User type for DIN70k coordinate system with base in principle point of camera
    class CDinCamType   { };
    //User type for CV coordinate system with base in middle of front axle
    class CCvFrontType  { };
    //User type for CV coordinate system with base in middle of rear axle
    class CCvRearType   { };
    //User type for CV coordinate system with base in projection of middle of rear axle to the ground
    class CCvWorldType  { };
    //User type for CV coordinate system with base in principle point of camera
    class CCvCamType    { };

    typedef TSIType<CLength>            CLengthType;
    typedef TSIType<CMass>              CMassType;
    typedef TSIType<CTime>              CTimeType;
    typedef TSIType<CCurrent>           CCurrentType;
    typedef TSIType<CTemperature>       CTemperatureType;
    typedef TSIType<CAmountOfSubstance> CAmountOfSubstanceType;
    typedef TSIType<CLuminousIntensity> CLuminousIntensityType;
    typedef TSIType<CAngle>             CAngleType;

    typedef TSIType<CPixel>             CPixelType;
    typedef TSIType<CPercentage>        CPercentageType;

    typedef vfc::TUnitInfoType<
        vfc::CLengthType, vfc::BASE, 0,
        vfc::CMassType, vfc::BASE, 0,
        vfc::CTimeType, vfc::BASE, 0,
        vfc::CCurrentType, vfc::BASE, 0,
        vfc::CTemperatureType, vfc::BASE, 0,
        vfc::CAmountOfSubstanceType, vfc::BASE, 0,
        vfc::CLuminousIntensityType, vfc::BASE, 0,
        vfc::CAngleType, vfc::BASE, 0,
        vfc::CBasicType>    info_nil_type;

    typedef vfc::TRBInfoType<
        vfc::CPixelType, vfc::BASE, 0,             //pixle
        vfc::CPercentageType, vfc::BASE, 0,        //Percentage
        vfc::CBasicType>                            info_rbnil_type;

    struct CDefaultType
    {
    };

    template<class UnitInfo1Type, class UnitInfo2Type, class DataType>
    class TNoConvertConvert
    {
    public:

        //static assserts for verifying if the prefixes match
        VFC_STATIC_ASSERT(UnitInfo1Type::LENGTH_PREFIX_VALUE == UnitInfo2Type::LENGTH_PREFIX_VALUE);
        VFC_STATIC_ASSERT(UnitInfo1Type::MASS_PREFIX_VALUE == UnitInfo2Type::MASS_PREFIX_VALUE);
        VFC_STATIC_ASSERT(UnitInfo1Type::TIME_PREFIX_VALUE == UnitInfo2Type::TIME_PREFIX_VALUE);
        VFC_STATIC_ASSERT(UnitInfo1Type::CURRENT_PREFIX_VALUE == UnitInfo2Type::CURRENT_PREFIX_VALUE);
        VFC_STATIC_ASSERT(UnitInfo1Type::TEMPERATURE_PREFIX_VALUE == UnitInfo2Type::TEMPERATURE_PREFIX_VALUE);
        VFC_STATIC_ASSERT(UnitInfo1Type::AMOUNTOFSUBSTANCE_PREFIX_VALUE == UnitInfo2Type::AMOUNTOFSUBSTANCE_PREFIX_VALUE);
        VFC_STATIC_ASSERT(UnitInfo1Type::LUMINOUSINTENSITY_PREFIX_VALUE == UnitInfo2Type::LUMINOUSINTENSITY_PREFIX_VALUE);
        VFC_STATIC_ASSERT(UnitInfo1Type::ANGLE_PREFIX_VALUE == UnitInfo2Type::ANGLE_PREFIX_VALUE);

        template<class ValueType>
        inline static void performValueConversion(ValueType&)
        {

        }
    };
}



#endif //SIUNITS_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_siunits_helper.hpp  $
//  Revision 1.10 2014/09/24 15:50:23MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_siunits: correct the Degree/Radian handling (mantis0004533)
//  Revision 1.9 2014/08/12 16:42:02MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Use an extra set of parentheses for std::min/max() function calls (mantis0004452)
//  Revision 1.8 2014/05/15 16:19:43MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - SI-Units: kg=cm!? (mantis0004478)
//  Revision 1.7 2014/05/13 14:20:50MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - SI-Units: kg=cm!? (mantis0004478)
//  Revision 1.6 2014/03/27 13:44:04MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Move TPower implementation from siunits to metaprog (mantis0003445)
//  Revision 1.5 2013/01/16 12:09:58MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - replaced tabs by 4 spaces
//  Revision 1.4 2013/01/15 09:54:14MEZ Gaurav Jain (RBEI/ESD1) (gaj2kor) 
//  -Incorporated all the proposed code under SIUnit (mantis 4184)
//  Revision 1.3 2012/12/18 12:57:32IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.2 2010/09/13 16:43:49MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - si units: add support for units used in Volkswagen AG DBC files (mantis3370)
//  Revision 1.1 2010/08/11 21:27:08IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/core/core.pj
//  Revision 1.14 2010/07/14 09:33:00MESZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Addidtion of typepromotion for atan function. (mantis 0003347)
//  Revision 1.13 2010/06/14 13:32:26CEST Gaurav Jain (RBEI/EAS3) (gaj2kor) 
//  - Inclusion of TypePromotion for sqr and sqrt functions. (mantis : 0003341)
//  Revision 1.12 2010/06/02 13:34:04CEST Gaurav Jain (RBEI/EAS3) (gaj2kor) 
//  -Introduction of RB Types as new template parameter. (Mantis :0003259)
//  Revision 1.11 2010/03/12 19:17:24GMT+05:30 Jaeger Thomas (CC/ESV2) (JAT2HI)
//  - fixed compiler warning (mantis3253)
//  Revision 1.10 2010/03/11 06:23:04CET Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Inclusion of new SIUnits UserType (like "pixel", "coord system").
//  Revision 1.9 2009/07/31 18:41:46GMT+05:30 Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Support for different offsets in the siunits implementation, for units like temperature Kelvin and °C. (mantis 2975)
//  Revision 1.8 2009/03/06 17:10:01GMT+05:30 Dilip Krishna (RBEI/EAC1) (dkn2kor)
//  - compiler issues in cw85 resolved (mantis 2542)
//  Revision 1.7 2009/02/03 12:18:52IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Removal of QAC++ warnings.
//  (Mantis : 0002549)
//  Revision 1.6 2009/01/23 16:00:39IST Dilip Krishna (RBEI/EAC1) (dkn2kor)
//  - qacpp warnings fixed
//  Revision 1.5 2009/01/09 14:14:10IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  - No convert feature added
//  Revision 1.4 2008/12/11 17:01:09IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  - major redesign to accomodate the conversion between different types
//  Revision 1.3 2008/10/09 17:12:01IST Jaeger Thomas (CC-DA/ESV1) (JAT2HI)
//  - updates
//  Revision 1.2 2008/09/17 07:46:58CEST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  - new InfoTypes for Ampere and Candela are added ( mantis :- 0002340 , 0002328)
//  Revision 1.1 2008/08/20 18:24:51IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_siunits/vfc_siunits.pj
//=============================================================================
