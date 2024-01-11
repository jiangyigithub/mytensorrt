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
///     $Source: vfc_siunits.hpp $
///     $Revision: 1.10 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/07/21 09:22:21MESZ $
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

#ifndef SIUNITS_HPP_INCLUDED
#define SIUNITS_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"           // vfc::int32_t
#include "vfc/core/vfc_metaprog.hpp"        // TIf
#include "vfc/core/vfc_type_traits.hpp"     // TIsFundamental
#include "vfc/core/vfc_math.hpp"            // vfc::divide
#include "vfc/core/vfc_siunits_helper.hpp"  // For UnitInfoType & conversions

namespace vfc
{   // open namespace vfc
     

    
    //=============================================================================
    //  TSIUnits <>
    //-----------------------------------------------------------------------------
    //! TSIUnits class
    //! The TSIUnits class is a generic template class that can be used to create
    //! many siunit types with a desired datatype
    //! $Source: vfc_siunits.hpp $
    //! @param ValueType            Defines the DataType
    //! @param UnitInfoType         Defines the type of SIUnit
    //! @param RBInfoType           New Bosch specific Info type
    //! @param UserType             Defines a specific UserType SIUnit
    //! @author                     dkn2kor
    //! @ingroup                    vfc_core
    //=============================================================================
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,     // new Bosch specific Info type
        class UserType = vfc::CDefaultType,
        template<class, class, class> class ConvertPolicyType = vfc::TConvert>
    class TSIUnits
    {

    public:
        // typdefes
        typedef ValueType                                                       value_type;
        typedef UserType                                                        user_type;
        typedef TSIUnits<ValueType, UnitInfoType, RBInfoType,  UserType, ConvertPolicyType>  self_type;
        typedef UnitInfoType                                                    unit_type;    
        typedef RBInfoType                                                      rbunit_type; 
        typedef ConvertPolicyType<UnitInfoType, UnitInfoType, ValueType>        convert_type;

        typedef TSIUnits<ValueType, typename vfc::TSIUnitSqrtPromote<UnitInfoType>::unit_info_type,
                                    typename vfc::TSIUnitRBSqrtPromote<RBInfoType>::unit_info_type,  UserType, ConvertPolicyType>  sqrt_promoted_type;

        typedef TSIUnits<ValueType, typename vfc::TSIUnitSqrPromote<UnitInfoType>::unit_info_type,
                                    typename vfc::TSIUnitRBSqrPromote<RBInfoType>::unit_info_type,  UserType, ConvertPolicyType>  sqr_promoted_type;

        typedef TSIUnits<ValueType, typename vfc::TSIUnitSinCosPromote<UnitInfoType>::unit_info_type, RBInfoType,  UserType, ConvertPolicyType>  trig_promoted_type;



        //---------------------------------------------------------------------
        //! Default constructor
        //! $Source: vfc_siunits.hpp $
        //! @author     dkn2kor
        //---------------------------------------------------------------------
        TSIUnits() 
        // intentionally no m_value initialization, to mimic POD behaviour
        {
           //intentionally left blank
        }

        //---------------------------------------------------------------------
        //! Explicit    Constructs with the specified value
        //! @param  f_value_r   Data to be stored
        //! $Source: vfc_siunits.hpp $
        //! @author     ves3kor
        //---------------------------------------------------------------------
        explicit TSIUnits(const ValueType& f_value_r) : m_value(f_value_r)
        {
            //intentionally left blank
        }

        //---------------------------------------------------------------------
        //! Explicit    Constructs with the SIUnits type
        //! @param  f_value_r   Data to be stored
        //! $Source: vfc_siunits.hpp $
        //! @author     ves3kor
        //---------------------------------------------------------------------
        template<class OtherUnitInfoType>
        TSIUnits(const vfc::TSIUnits<ValueType,
            OtherUnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_value_r) :
            m_value(f_value_r.value())
        {
            TSICompatibleCheck<UnitInfoType, OtherUnitInfoType>::check();

            ConvertPolicyType<OtherUnitInfoType, UnitInfoType,
                typename vfc::TIf<(vfc::TIsFloating<ValueType>::value==1),
                    CFloatingType, CIntegralType>::type>::performValueConversion(m_value);
        }

        //---------------------------------------------------------------------
        //! Explicit    Constructs with the SIUnits type
        //! @param  f_value_r   Data to be stored
        //! $Source: vfc_siunits.hpp $
        //! @author     ves3kor
        //---------------------------------------------------------------------
        template<class OtherRBInfoType>
        TSIUnits(const vfc::TSIUnits<ValueType,
            UnitInfoType, OtherRBInfoType, UserType, ConvertPolicyType>& f_value_r) :
            m_value(f_value_r.value())
        {
            ConvertPolicyType<OtherRBInfoType, RBInfoType,
                typename vfc::TIf<(vfc::TIsFloating<ValueType>::value==1),
                    CFloatingType, CIntegralType>::type>::performValueConversion(m_value);
        }

        //---------------------------------------------------------------------
        //! Returns the contained value
        //! $Source: vfc_siunits.hpp $
        //! @return     returns value of data type
        //! @author     ves3kor
        //---------------------------------------------------------------------
        ValueType value() const
        {
            return m_value;
        }

        //---------------------------------------------------------------------
        //! Returns the reference of the contained value
        //! $Source: vfc_siunits.hpp $
        //! @return     returns reference of the value
        //! @author     ves3kor
        //---------------------------------------------------------------------
        ValueType& value()
        {
            return m_value;
        }

        //---------------------------------------------------------------------
        //! Right shift assignment operator
        //! @param     f_shift_i    shift bits (Numeric expression of a data type that widens to Integer)
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        TSIUnits& operator >>=(vfc::int32_t f_shift_i)
        {
            VFC_STATIC_ASSERT(!TIsFloating<ValueType>::value);
            m_value >>= f_shift_i;
            return *this;
        }

        //---------------------------------------------------------------------
        //! Left shift assignment operator
        //! @param     f_shift_i    shift bits (Numeric expression of a data type that widens to Integer)
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        TSIUnits& operator <<=(vfc::int32_t f_shift_i)
        {
             // only Integers are accepted
            VFC_STATIC_ASSERT(!TIsFloating<ValueType>::value);
            m_value <<= f_shift_i;
            return *this;
        }

        //---------------------------------------------------------------------
        //! Right shift operator
        //! @param     f_shift_i    shift bits (Numeric expression of a data type that widens to Integer)
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        const TSIUnits operator >>(vfc::int32_t f_shift_i) const
        {
            self_type result(*this);
            result >>= f_shift_i;
            return result;
        }


        //---------------------------------------------------------------------
        //! Left shift operator
        //! @param     f_shift_i    shift bits (Numeric expression of a data type that widens to Integer)
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        const TSIUnits operator <<(vfc::int32_t f_shift_i) const
        {
            self_type result(*this);
            result <<= f_shift_i;
            return result;
        }

        //---------------------------------------------------------------------
        //! Negation operator
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        TSIUnits operator -() const
        {
            self_type result(-(this->m_value));
            return result;
        }

        //---------------------------------------------------------------------
        //! Decrement operator
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        TSIUnits& operator --()
        {
            --m_value;
            return *this;
        }

        //---------------------------------------------------------------------
        //! Post decrement operator
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        //PRQA S 2427 ++
        TSIUnits operator --(int)
        {
            self_type result(*this);
            --(*this);
            return result;
        }
        //PRQA S 2427 --

        //---------------------------------------------------------------------
        //! Increment operator
        //! $Source: vfc_siunits.hpp $
        //! @return    returns result of siunit_type types.
        //! @author    ves3kor
        //---------------------------------------------------------------------
        TSIUnits& operator ++()
        {
            ++m_value;
            return *this;
        }


        //---------------------------------------------------------------------
        //! Post increment operator
        //! $Source: vfc_siunits.hpp $
        //! @return    returns result of siunit_type types.
        //! @author    ves3kor
        //---------------------------------------------------------------------
        //PRQA S 2427 ++
        TSIUnits operator ++(int)
        {
            self_type result(*this);
            ++(*this);
            return result;
        }
        //PRQA S 2427 --
        //---------------------------------------------------------------------
        //! Comparision Operator
        //! @param     f_rhs_r  data to be comapared
        //! $Source: vfc_siunits.hpp $
        //! @return    returns if the data contained within is equal to that of f_rhs_r
        //! @author    ves3kor
        //---------------------------------------------------------------------
        bool operator ==(const TSIUnits& f_rhs_r) const
        {
            return (this->m_value == f_rhs_r.m_value);
        }


        //---------------------------------------------------------------------
        //! Is not equal to comparision Operator
        //! @param     f_rhs_r  data to be comapared
        //! $Source: vfc_siunits.hpp $
        //! @return    returns if the data contained within is inequal to that of f_rhs_r
        //! @author    ves3kor
        //---------------------------------------------------------------------
        bool operator !=(const TSIUnits& f_rhs_r) const
        {
            return (!((*this) == f_rhs_r));
        }


        //---------------------------------------------------------------------
        //! Lesser than Operator
        //! @param     f_rhs_r  data to be comapared
        //! $Source: vfc_siunits.hpp $
        //! @return    returns if the data contained within is less than that of f_rhs_r
        //! @author    ves3kor
        //---------------------------------------------------------------------
        bool operator <(const TSIUnits& f_rhs_r) const
        {
            return (this->m_value < f_rhs_r.m_value);
        }


        //---------------------------------------------------------------------
        //! Greater than Operator
        //! @param     f_rhs_r  data to be comapared
        //! $Source: vfc_siunits.hpp $
        //! @return    returns if the data contained within is greater than that of f_rhs_r
        //! @author    ves3kor
        //---------------------------------------------------------------------
        bool operator >(const TSIUnits& f_rhs_r) const
        {
            return (f_rhs_r < (*this));
        }

        //---------------------------------------------------------------------
        //! Lesser than Or Equal to Operator
        //! @param     f_rhs_r  data to be comapared
        //! $Source: vfc_siunits.hpp $
        //! @return    returns if the data contained within is less than or equal to that of f_rhs_r
        //! @author    ves3kor
        //---------------------------------------------------------------------
        bool operator <=(const TSIUnits& f_rhs_r) const
        {
            return (!(f_rhs_r < (*this)));
        }

        //---------------------------------------------------------------------
        //! Greater than Or Equal to Operator
        //! @param     f_rhs_r  data to be comapared
        //! $Source: vfc_siunits.hpp $
        //! @return    returns if the data contained within is greater than or equal to that of f_rhs_r
        //! @author    ves3kor
        //---------------------------------------------------------------------
        bool operator >=(const TSIUnits& f_rhs_r) const
        {
            return (!((*this) < f_rhs_r));
        }

        //---------------------------------------------------------------------
        //! Addition Assignment Operator
        //! @param     f_rhs_r  data to be added
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        template<class OtherUnitInfoType>
        TSIUnits& operator +=(
                const TSIUnits<ValueType, OtherUnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
        {
            TSICompatibleCheck<UnitInfoType, OtherUnitInfoType>::check();

            ValueType value = f_rhs_r.value();
            ConvertPolicyType<OtherUnitInfoType, UnitInfoType,
                typename vfc::TIf<(vfc::TIsIntegral<ValueType>::value==1),
                    CIntegralType, CFloatingType>::type>::performValueConversion(value);

            m_value += value;
            return (*this);
        }


        //---------------------------------------------------------------------
        //! Subtraction Assignment Operator
        //! @param     f_rhs_r  data to be subtracted
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        template<class OtherUnitInfoType>
        TSIUnits& operator -=(
                const TSIUnits<ValueType, OtherUnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
        {
            TSICompatibleCheck<UnitInfoType, OtherUnitInfoType>::check();

            ValueType value = f_rhs_r.value();
            ConvertPolicyType<OtherUnitInfoType, UnitInfoType,
                typename vfc::TIf<(vfc::TIsFloating<ValueType>::value==1),
                    CFloatingType, CIntegralType>::type>::performValueConversion(value);

            m_value -= value;
            return (*this);
        }

        //---------------------------------------------------------------------
        //! Multiplication Assignment Operator
        //! @param     f_rhs_r  data to be multiplied
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        TSIUnits& operator *=(const value_type& f_rhs_r)
        {
            m_value *= f_rhs_r;
            return (*this);
        }

        //---------------------------------------------------------------------
        //! Multiplication Assignment Operator with unitless TSIUnits as RHS.
        //! @param     f_rhs_r  data to be multiplied
        //! $Source: vfc_siunits.hpp $
        //! @author    gaj2kor
        //---------------------------------------------------------------------
        TSIUnits& operator *=(
                const TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
        {
            VFC_STATIC_ASSERT(UnitInfoType::LENGTH_POWER_VALUE == 0);
            VFC_STATIC_ASSERT(UnitInfoType::MASS_POWER_VALUE == 0);
            VFC_STATIC_ASSERT(UnitInfoType::TIME_POWER_VALUE == 0);
            VFC_STATIC_ASSERT(UnitInfoType::CURRENT_POWER_VALUE == 0);
            VFC_STATIC_ASSERT(UnitInfoType::TEMPERATURE_POWER_VALUE == 0);
            VFC_STATIC_ASSERT(UnitInfoType::AMOUNTOFSUBSTANCE_POWER_VALUE == 0);
            VFC_STATIC_ASSERT(UnitInfoType::LUMINOUSINTENSITY_POWER_VALUE == 0);
            VFC_STATIC_ASSERT(UnitInfoType::ANGLE_POWER_VALUE == 0);

            this->operator *=(f_rhs_r.value());
            return (*this);
        }

        //---------------------------------------------------------------------
        //! Multiplication Assignment Operator with unitless TSIUnits as RHS.
        //! @param     f_rhs_r  data to be multiplied
        //! $Source: vfc_siunits.hpp $
        //! @author    gaj2kor
        //---------------------------------------------------------------------
        template<class OtherRBInfoType>
        TSIUnits& operator *=(
                const TSIUnits<ValueType, UnitInfoType, OtherRBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
        {
            //Static assert for RBInfoType is a Nil type
            VFC_STATIC_ASSERT(OtherRBInfoType::PIXEL_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherRBInfoType::PIXEL_POWER_VALUE == 0);

            this->operator *=(f_rhs_r.value());
            return (*this);
        }

        //---------------------------------------------------------------------
        //! Multiplication Assignment Operator with unitless TSIUnits as RHS.
        //! @param     f_rhs_r  data to be multiplied
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        template<class OtherUnitInfoType>
        TSIUnits& operator *=(
                const TSIUnits<ValueType, OtherUnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
        {
            //Static assert for InfoType is a Nil type
            VFC_STATIC_ASSERT(OtherUnitInfoType::LENGTH_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::LENGTH_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::MASS_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::MASS_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::TIME_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::TIME_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::CURRENT_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::CURRENT_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::TEMPERATURE_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::TEMPERATURE_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::AMOUNTOFSUBSTANCE_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::AMOUNTOFSUBSTANCE_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::LUMINOUSINTENSITY_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::LUMINOUSINTENSITY_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::ANGLE_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::ANGLE_POWER_VALUE == 0);

            this->operator *=(f_rhs_r.value());
            return (*this);
        }

        //---------------------------------------------------------------------
        //! Division Assignment Operator
        //! @param     f_rhs_r  divisor data
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        TSIUnits& operator /=(const value_type& f_rhs_r)
        {
            m_value /= f_rhs_r;
            return (*this);
        }

        //---------------------------------------------------------------------
        //! Division Assignment Operator with other RBInfoType TSIUnits as RHS.
        //! @param     f_rhs_r  divisor data
        //! $Source: vfc_siunits.hpp $
        //! @author    gaj2kor
        //---------------------------------------------------------------------
        template<class OtherRBInfoType>
        TSIUnits& operator /=(
                const TSIUnits<ValueType, UnitInfoType, OtherRBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
        {
            VFC_STATIC_ASSERT(OtherRBInfoType::PIXEL_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherRBInfoType::PIXEL_POWER_VALUE == 0);

            this->operator /=(f_rhs_r.value());
            return (*this);
        }

        //---------------------------------------------------------------------
        //! Division Assignment Operator with TSIUnits as RHS.
        //! @param     f_rhs_r  divisor data
        //! $Source: vfc_siunits.hpp $
        //! @author    gaj2kor
        //---------------------------------------------------------------------
        TSIUnits& operator /=(
                const TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
        {
            this->operator /=(f_rhs_r.value());
            return (*this);
        }

        //---------------------------------------------------------------------
        //! Division Assignment Operator with unitless TSIUnits as RHS.
        //! @param     f_rhs_r  divisor data
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        template<class OtherUnitInfoType>
        TSIUnits& operator /=(
                const TSIUnits<ValueType, OtherUnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
        {
            //Static assert for InfoType is a Nil type
            VFC_STATIC_ASSERT(OtherUnitInfoType::LENGTH_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::LENGTH_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::MASS_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::MASS_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::TIME_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::TIME_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::CURRENT_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::CURRENT_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::TEMPERATURE_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::TEMPERATURE_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::AMOUNTOFSUBSTANCE_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::AMOUNTOFSUBSTANCE_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::LUMINOUSINTENSITY_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::LUMINOUSINTENSITY_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::ANGLE_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::ANGLE_POWER_VALUE == 0);

            this->operator /=(f_rhs_r.value());
            return (*this);
        }


        //---------------------------------------------------------------------
        //! Division Assignment Operator with other RBInfoType and other UnitInfoType TSIUnits as RHS.
        //! @param     f_rhs_r  divisor data
        //! $Source: vfc_siunits.hpp $
        //! @author    gaj2kor
        //---------------------------------------------------------------------
        template<class OtherUnitInfoType, class OtherRBInfoType>
        TSIUnits& operator /=(
                const TSIUnits<ValueType, OtherUnitInfoType, OtherRBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
        {
            //Static assert for InfoType is a Nil type
            VFC_STATIC_ASSERT(OtherUnitInfoType::LENGTH_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::LENGTH_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::MASS_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::MASS_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::TIME_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::TIME_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::CURRENT_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::CURRENT_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::TEMPERATURE_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::TEMPERATURE_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::AMOUNTOFSUBSTANCE_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::AMOUNTOFSUBSTANCE_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::LUMINOUSINTENSITY_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::LUMINOUSINTENSITY_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherUnitInfoType::ANGLE_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherUnitInfoType::ANGLE_POWER_VALUE == 0);

            VFC_STATIC_ASSERT(OtherRBInfoType::PIXEL_PREFIX_VALUE == vfc::BASE);
            VFC_STATIC_ASSERT(OtherRBInfoType::PIXEL_POWER_VALUE == 0);

            this->operator /=(f_rhs_r.value());
            return (*this);
        }


        //---------------------------------------------------------------------
        //! Mod Assignment Operator
        //! @param     f_rhs_r  data on rhs
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        template<class OtherUnitInfoType>
        TSIUnits& operator %=(
                const TSIUnits<ValueType, OtherUnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
        {
            TSICompatibleCheck<UnitInfoType, OtherUnitInfoType>::check();

            // only Integers are accepted
            VFC_STATIC_ASSERT((TIsFloating<ValueType>::value==0));

            this->m_value %= f_rhs_r.m_value;
            return (*this);
        }

        //---------------------------------------------------------------------
        //! Bitwise OR Assignment Operator
        //! @param     f_rhs_r  data to be comapared
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        TSIUnits& operator |=(const TSIUnits& f_rhs_r)
        {
            // only Integers are accepted
            VFC_STATIC_ASSERT(!TIsFloating<ValueType>::value);

            this->m_value |= f_rhs_r.m_value;
            return *this;
        }

        //---------------------------------------------------------------------
        //! Bitwise AND Assignment Operator
        //! @param     f_rhs_r  data to be comapared
        //! $Source: vfc_siunits.hpp $
        //! @author    ves3kor
        //---------------------------------------------------------------------
        TSIUnits& operator &=(const TSIUnits& f_rhs_r)
        {
            // only Integers are accepted
            VFC_STATIC_ASSERT(!TIsFloating<ValueType>::value);

            this->m_value &= f_rhs_r.m_value;
            return *this;
        }

    private:
        value_type  m_value; //!< Member variable used store the data value
    };

    //-----------------------------------------------------------------------------
    //! TSIUnits + operator
    //! computes the sum of the two operands.
    //! $Source: vfc_siunits.hpp $
    //! @param f_lhs_r              TSIUnits expression.
    //! @param f_rhs_r              TSIUnits expression.
    //! @return returns the sum of the two operands.
    //! @note f_lhs_r and f_rhs_r should belong the same SI unit type
    //! @author                     ves3kor
    //! @ingroup                    vfc_core
    //=============================================================================

    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
                                        template<class, class, class> class ConvertPolicyType>
    const TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>
    operator +(
        const TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_lhs_r,
        const TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
    {
        vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> result(f_lhs_r);
        result.operator+=(f_rhs_r);
        return result;
    }

    //-----------------------------------------------------------------------------
    //! TSIUnits - operator
    //! computes the difference between the two operands.
    //! $Source: vfc_siunits.hpp $
    //! @param f_lhs_r              TSIUnits expression.
    //! @param f_rhs_r              TSIUnits expression.
    //! @return returns the difference of the two operands.
    //! @note f_lhs_r and f_rhs_r should belong the same SI unit type
    //! @author                     ves3kor
    //! @ingroup                    vfc_core
    //=============================================================================
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
                                        template<class, class, class> class ConvertPolicyType>
    const TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>
    operator -(
        const TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_lhs_r,
        const TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
    {
        vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> result(f_lhs_r);
        result.operator-=(f_rhs_r);
        return result;
    }

    //-----------------------------------------------------------------------------
    //! TSIUnits * operator.
    //! computes the multiplication of two operands.
    //! $Source: vfc_siunits.hpp $
    //! @param f_lhs_r              TSIUnits expression.
    //! @param f_rhs_r              TSIUnits expression.
    //! @return returns the multiplication of the two operands.
    //! @author                     ves3kor
    //! @ingroup                    vfc_core
    //=============================================================================
    // (Msg Disable 4222:This binary operator is not implemented in terms of its assignment version. )
    // PRQA S 4222 ++
    template<class ValueType, class UnitInfo1Type, class UnitInfo2Type, class RBInfo1Type, class RBInfo2Type,
                                        class UserType, template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType,
        typename vfc::TSIUnitMuliplicationPromote<UnitInfo1Type, UnitInfo2Type>::unit_info_type,
        typename vfc::TSIUnitRBMuliplicationPromote<RBInfo1Type, RBInfo2Type>::unit_info_type,
        UserType,
        ConvertPolicyType>
    operator *(
        const vfc::TSIUnits<ValueType, UnitInfo1Type, RBInfo1Type, UserType, ConvertPolicyType>& f_val1_r,
        const vfc::TSIUnits<ValueType, UnitInfo2Type, RBInfo2Type, UserType, ConvertPolicyType>& f_val2_r)
    {

        ValueType l_value2 = f_val2_r.value();
        ConvertPolicyType<UnitInfo2Type, 
            vfc::TUnitInfoType< 
                               typename UnitInfo2Type::length_unit_type,
                               UnitInfo1Type::LENGTH_PREFIX_VALUE,
                               UnitInfo2Type::LENGTH_POWER_VALUE,
                               typename UnitInfo2Type::mass_unit_type,
                               UnitInfo1Type::MASS_PREFIX_VALUE,
                               UnitInfo2Type::MASS_POWER_VALUE,
                               typename UnitInfo2Type::time_unit_type,
                               UnitInfo1Type::TIME_PREFIX_VALUE,
                               UnitInfo2Type::TIME_POWER_VALUE,
                               typename UnitInfo2Type::current_unit_type,
                               UnitInfo1Type::CURRENT_PREFIX_VALUE,
                               UnitInfo2Type::CURRENT_POWER_VALUE,
                               typename UnitInfo2Type::temperature_unit_type,
                               UnitInfo1Type::TEMPERATURE_PREFIX_VALUE,
                               UnitInfo2Type::TEMPERATURE_POWER_VALUE,
                               typename UnitInfo2Type::amountofsubstance_unit_type,
                               UnitInfo1Type::AMOUNTOFSUBSTANCE_PREFIX_VALUE,
                               UnitInfo2Type::AMOUNTOFSUBSTANCE_POWER_VALUE,
                               typename UnitInfo2Type::luminousintensity_unit_type,
                               UnitInfo1Type::LUMINOUSINTENSITY_PREFIX_VALUE,
                               UnitInfo2Type::LUMINOUSINTENSITY_POWER_VALUE,
                               typename UnitInfo2Type::angle_unit_type,
                               UnitInfo1Type::ANGLE_PREFIX_VALUE,
                               UnitInfo2Type::ANGLE_POWER_VALUE,
                               typename UnitInfo2Type::unit_type
                        >,
            typename vfc::TIf<(vfc::TIsIntegral<ValueType>::value==1),
            CIntegralType, CFloatingType>::type>::performValueConversion(l_value2);

        return (vfc::TSIUnits<ValueType,
            typename TSIUnitMuliplicationPromote<UnitInfo1Type, UnitInfo2Type>::unit_info_type,
            typename TSIUnitRBMuliplicationPromote<RBInfo1Type, RBInfo2Type>::unit_info_type,
            UserType, ConvertPolicyType>(f_val1_r.value() * l_value2));
    }

    // PRQA S 4222 --
    // (Msg Enable 4222:This binary operator is not implemented in terms of its assignment version. )

    //-----------------------------------------------------------------------------
    //! TSIUnits * operator.
    //! computes the multiplication of two operands (siunits_type * pod_type)
    //! $Source: vfc_siunits.hpp $
    //! @param f_lhs_r              TSIUnits expression.
    //! @param f_rhs_r              POD      expression
    //! @return returns the multiplication of the two operands.
    //! @author                     ves3kor
    //! @ingroup                    vfc_core
    //=============================================================================
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
                                        template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>
    operator *(
        const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_lhs_r,
        const ValueType& f_rhs_r)
    {
        vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> result(f_lhs_r);
        result.operator*=(f_rhs_r);
        return result;
    }


    //-----------------------------------------------------------------------------
    //! TSIUnits * operator.
    //! computes the multiplication of two operands (pod_type * siunit_type)
    //! $Source: vfc_siunits.hpp $
    //! @param f_lhs_r              POD      expression
    //! @param f_rhs_r              TSIUnits expression.
    //! @return returns the multiplication of the two operands.
    //! @author                     ves3kor
    //! @ingroup                    vfc_core
    //=============================================================================
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
                                        template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>
    operator *(
        const ValueType& f_lhs_r,
        const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
    {
        vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> result(f_rhs_r);
        result.operator*=(f_lhs_r);
        return result;
    }

    //-----------------------------------------------------------------------------
    //! TSIUnits / operator.
    //! computes the division of two operands  (siunits_type / siunits_type)
    //! $Source: vfc_siunits.hpp $
    //! @param f_lhs_r              TSIUnits expression.
    //! @param f_rhs_r              TSIUnits expression.
    //! @return returns the division of the two operands.
    //! @author                     ves3kor
    //! @ingroup                    vfc_core
    //=============================================================================
    // (Msg Disable 4222:This binary operator is not implemented in terms of its assignment version. )
    // PRQA S 4222 ++
    template<class ValueType, class UnitInfo1Type, class UnitInfo2Type, class RBInfo1Type,
             class RBInfo2Type, class UserType, template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType,
        typename TSIUnitDivisionPromote<UnitInfo1Type, UnitInfo2Type>::unit_info_type,
        typename TSIUnitRBDivisionPromote<RBInfo1Type, RBInfo2Type>::unit_info_type,
        UserType,
        ConvertPolicyType>
    operator /(
        const vfc::TSIUnits<ValueType, UnitInfo1Type, RBInfo1Type, UserType, ConvertPolicyType>& f_val1_r,
        const vfc::TSIUnits<ValueType, UnitInfo2Type, RBInfo2Type, UserType, ConvertPolicyType>& f_val2_r)
    {
        ValueType l_value2 = f_val2_r.value();
        ConvertPolicyType<UnitInfo2Type, 
            vfc::TUnitInfoType< 
            typename UnitInfo2Type::length_unit_type,
            UnitInfo1Type::LENGTH_PREFIX_VALUE,
            UnitInfo2Type::LENGTH_POWER_VALUE,
            typename UnitInfo2Type::mass_unit_type,
            UnitInfo1Type::MASS_PREFIX_VALUE,
            UnitInfo2Type::MASS_POWER_VALUE,
            typename UnitInfo2Type::time_unit_type,
            UnitInfo1Type::TIME_PREFIX_VALUE,
            UnitInfo2Type::TIME_POWER_VALUE,
            typename UnitInfo2Type::current_unit_type,
            UnitInfo1Type::CURRENT_PREFIX_VALUE,
            UnitInfo2Type::CURRENT_POWER_VALUE,
            typename UnitInfo2Type::temperature_unit_type,
            UnitInfo1Type::TEMPERATURE_PREFIX_VALUE,
            UnitInfo2Type::TEMPERATURE_POWER_VALUE,
            typename UnitInfo2Type::amountofsubstance_unit_type,
            UnitInfo1Type::AMOUNTOFSUBSTANCE_PREFIX_VALUE,
            UnitInfo2Type::AMOUNTOFSUBSTANCE_POWER_VALUE,
            typename UnitInfo2Type::luminousintensity_unit_type,
            UnitInfo1Type::LUMINOUSINTENSITY_PREFIX_VALUE,
            UnitInfo2Type::LUMINOUSINTENSITY_POWER_VALUE,
            typename UnitInfo2Type::angle_unit_type,
            UnitInfo1Type::ANGLE_PREFIX_VALUE,
            UnitInfo2Type::ANGLE_POWER_VALUE,
            typename UnitInfo2Type::unit_type
            >,
            typename vfc::TIf<(vfc::TIsIntegral<ValueType>::value==1),
            CIntegralType, CFloatingType>::type>::performValueConversion(l_value2);


        return vfc::TSIUnits<ValueType,
            typename TSIUnitDivisionPromote<UnitInfo1Type, UnitInfo2Type>::unit_info_type,
            typename TSIUnitRBDivisionPromote<RBInfo1Type, RBInfo2Type>::unit_info_type,
            UserType, ConvertPolicyType>(vfc::divide(f_val1_r.value(), l_value2));
    }
    // PRQA S 4222 --
    // (Msg Enable 4222:This binary operator is not implemented in terms of its assignment version. )

    //-----------------------------------------------------------------------------
    //! TSIUnits / operator.
    //! computes the division of two operands ( siunits_type / pod_type )
    //! $Source: vfc_siunits.hpp $
    //! @param f_lhs_r              TSIUnits expression.
    //! @param f_rhs_r              POD      expression.
    //! @return returns the division of the two operands.
    //! @author                     ves3kor
    //! @ingroup                    vfc_core
    //=============================================================================
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
                                        template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>
    operator /(
        const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_lhs_r,
        const ValueType& f_rhs_r)
    {
        vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType,  UserType, ConvertPolicyType> result(f_lhs_r);
        result.operator/=(f_rhs_r);
        return result;
    }

    //-----------------------------------------------------------------------------
    //! TSIUnits / operator.
    //! computes the division of two operands (pod_type / siunits_type )
    //! $Source: vfc_siunits.hpp $
    //! @param f_lhs_r              POD      expression.
    //! @param f_rhs_r              TSIUnits expression.
    //! @return returns the division of the two operands.
    //! @author                     ves3kor
    //! @ingroup                    vfc_core
    //=============================================================================
    // (Msg Disable 4222:This binary operator is not implemented in terms of its assignment version. )
    // PRQA S 4222 ++
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
                                        template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType,
        typename TSIUnitDivisionPromote<info_nil_type, UnitInfoType>::unit_info_type,
        typename TSIUnitRBDivisionPromote<info_rbnil_type, RBInfoType>::unit_info_type,
        UserType,
        ConvertPolicyType>
    operator /(
    const ValueType& f_lhs_r,
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
    {
        ValueType l_rhsValue = f_rhs_r.value();
        typedef vfc::TUnitInfoType<
            vfc::CLengthType,
            vfc::BASE,
            (0 - UnitInfoType::LENGTH_POWER_VALUE),
            vfc::CMassType,
            vfc::BASE,
            (0 - UnitInfoType::MASS_POWER_VALUE),
            vfc::CTimeType,
            vfc::BASE,
            (0 - UnitInfoType::TIME_POWER_VALUE),
            vfc::CCurrentType,
            vfc::BASE,
            (0 - UnitInfoType::CURRENT_POWER_VALUE),
            vfc::CTemperatureType,
            vfc::BASE,
            (0 - UnitInfoType::TEMPERATURE_POWER_VALUE),
            vfc::CAmountOfSubstanceType,
            vfc::BASE,
            (0 - UnitInfoType::AMOUNTOFSUBSTANCE_POWER_VALUE),
            vfc::CLuminousIntensityType,
            vfc::BASE,
            (0 - UnitInfoType::LUMINOUSINTENSITY_POWER_VALUE),
            vfc::CAngleType,
            vfc::BASE,
            (0 - UnitInfoType::ANGLE_POWER_VALUE),
            vfc::CBasicType>  my3_unit_info_type;
        ConvertPolicyType<UnitInfoType, my3_unit_info_type,
            typename vfc::TIf<(vfc::TIsIntegral<ValueType>::value==1),
            CIntegralType, CFloatingType>::type>::performValueConversion(l_rhsValue);

        vfc::TSIUnits<ValueType,
            typename TSIUnitDivisionPromote<info_nil_type, UnitInfoType>::unit_info_type,
            typename TSIUnitRBDivisionPromote<info_rbnil_type, RBInfoType>::unit_info_type,
            UserType,
            ConvertPolicyType> l_result((f_lhs_r / l_rhsValue));

        return l_result;
    }
    // PRQA S 4222 --
    // (Msg Enable 4222:This binary operator is not implemented in terms of its assignment version. )

    //-----------------------------------------------------------------------------
    //! TSIUnits % operator
    //! computes the modulus of two operands
    //! $Source: vfc_siunits.hpp $
    //! @param f_lhs_r              TSIUnits expression.
    //! @param f_rhs_r              TSIUnits expression.
    //! @return returns the modulus of the two operands.
    //! @author                     ves3kor
    //! @ingroup                    vfc_core
    //=============================================================================
    // (Msg Disable 4222:This binary operator is not implemented in terms of its assignment version. )
    // PRQA S 4222 ++
    template<class ValueType, class UnitInfo1Type, class UnitInfo2Type,class RBInfo1Type,
             class RBInfo2Type, class UserType, template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType, UnitInfo1Type, RBInfo1Type, UserType, ConvertPolicyType>
    operator %(
    const vfc::TSIUnits<ValueType, UnitInfo1Type, RBInfo1Type, UserType, ConvertPolicyType>& f_lhs_r,
    const vfc::TSIUnits<ValueType, UnitInfo2Type, RBInfo2Type, UserType, ConvertPolicyType>& f_rhs_r)
    {
        // only Integers are accepted
        VFC_STATIC_ASSERT((TIsFloating<ValueType>::value==0));

        return vfc::TSIUnits<ValueType, UnitInfo1Type, RBInfo1Type, UserType, ConvertPolicyType>(
            f_lhs_r.value() % f_rhs_r.value());
    }
    // PRQA S 4222 --
    // (Msg Enable 4222:This binary operator is not implemented in terms of its assignment version. )

    //-----------------------------------------------------------------------------
    //! Bitwise-inclusive-OR operator compares each bit of its first operand
    //! to the corresponding bit of its second operand. If either bit is 1,
    //! the corresponding result bit is set to 1. Otherwise, the corresponding
    //! result bit is set to 0.
    //! $Source: vfc_siunits.hpp $
    //! @param f_lhs_r              TSIUnits expression.
    //! @param f_rhs_r              TSIUnits expression.
    //! @return                     returns the OR of the two operands.
    //! @author                     ves3kor
    //! @ingroup                    vfc_core
    //=============================================================================
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
                                        template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>
    operator |(
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_lhs_r,
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
    {
        vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> result(f_lhs_r);
        result.operator|=(f_rhs_r);
        return result;
    }

    //-----------------------------------------------------------------------------
    //! Bitwise AND operator (&) compares each bit of the first operand
    //! to the corresponding bit of the second operand. If both bits are 1,
    //! the corresponding result bit is set to 1. Otherwise, the corresponding
    //! result bit is set to 0.
    //! $Source: vfc_siunits.hpp $
    //! @param f_lhs_r              TSIUnits expression.
    //! @param f_rhs_r              TSIUnits expression.
    //! @return                     returns the & of the two operands.
    //! @author                     ves3kor
    //! @ingroup                    vfc_core
    //=============================================================================
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
                                        template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>
    operator &(
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_lhs_r,
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
    {
        vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> result(f_lhs_r);
        result.operator&=(f_rhs_r);
        return result;
    }

    //-----------------------------------------------------------------------------
    //! Conditional-AND operator (&&) performs a logical-AND of its bool operands.
    //! $Source: vfc_siunits.hpp $
    //! @param f_lhs_r              TSIUnits expression.
    //! @param f_rhs_r              TSIUnits expression.
    //! @return                     returns the && of the two operands.
    //! @author                     ves3kor
    //! @ingroup                    vfc_core
    //=============================================================================
    // (Msg Disable 2078:Avoid overloading operator 'and' (&&).)
    // PRQA S 2078 ++
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
                                        template<class, class, class> class ConvertPolicyType>
    bool
    operator &&(
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_lhs_r,
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
    {
        return ((vfc::notZero(f_lhs_r.value()) &&
            vfc::notZero(f_rhs_r.value())));
    }
    // PRQA S 2078 --
    // (Msg Enable 2078:Avoid overloading operator 'and' (&&).)

    //-----------------------------------------------------------------------------
    //! Conditional-OR operator (||) performs a logical-OR of its bool operands.
    //! $Source: vfc_siunits.hpp $
    //! @param f_lhs_r              TSIUnits expression.
    //! @param f_rhs_r              TSIUnits expression.
    //! @return                     returns the || of the two operands.
    //! @author                     ves3kor
    //! @ingroup                    vfc_core
    //=============================================================================
    // (Msg Disable 2079:Avoid overloading operator 'or' (||).)
    // PRQA S 2079 ++
    template<class ValueType, class UnitInfoType,class RBInfoType, class UserType,
                                        template<class, class, class> class ConvertPolicyType>
    bool
    operator ||(
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_lhs_r,
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_rhs_r)
    {
        return ((vfc::notZero(f_lhs_r.value()) ||
            vfc::notZero(f_rhs_r.value())));
    }
    // PRQA S 2079 --
    // (Msg Enable 2079:Avoid overloading operator 'and' (&&).)

    // Made Fundamental type
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
                                        template<class, class, class> class ConvertPolicyType>
    struct TIsFundamental<vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> >
    {
        enum { value = TIsFundamental<ValueType>::value };
    };



    //---------------------------------------------------------------------
    //! To calculate tan
    //! @param     f_val_r  to calculate tan of
    //! $Source: vfc_siunits.hpp $
    //! @author    gaj2kor
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    const vfc::TSIUnits<ValueType, typename vfc::TSIUnitSinCosPromote<UnitInfoType>::unit_info_type,
           RBInfoType, UserType, ConvertPolicyType> tan(
             vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> f_val);

    //---------------------------------------------------------------------
    //! To calculate sin
    //! @param     f_val_r  to calculate sin of
    //! $Source: vfc_siunits.hpp $
    //! @author    gaj2kor
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    const vfc::TSIUnits<ValueType, typename vfc::TSIUnitSinCosPromote<UnitInfoType>::unit_info_type,
           RBInfoType, UserType, ConvertPolicyType> sin(
             vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> f_val);

    //---------------------------------------------------------------------
    //! To calculate cos
    //! @param     f_val_r  to calculate cos of
    //! $Source: vfc_siunits.hpp $
    //! @author    gaj2kor
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    const vfc::TSIUnits<ValueType, typename vfc::TSIUnitSinCosPromote<UnitInfoType>::unit_info_type,
          RBInfoType, UserType, ConvertPolicyType> cos(
            vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> f_val);

    //---------------------------------------------------------------------
    //! To calculate atan
    //! @param f_val_r to calculate atan of
    //! $Source: vfc_siunits.hpp $
    //! @author sst1lr
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    const vfc::TSIUnits<ValueType, typename vfc::TSIUnitAtanPromote<UnitInfoType>::unit_info_type,
           RBInfoType, UserType, ConvertPolicyType> atan(
             vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> f_val);

    //---------------------------------------------------------------------
    //! To calculate floor
    //! @param     f_val_r  to calculate floor of
    //! $Source: vfc_siunits.hpp $
    //! @author    gaj2kor
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> floor
                    ( vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> f_val_r);

    //---------------------------------------------------------------------
    //! To calculate min
    //! @param     f_val1_r  data to be compared with
    //! @param     f_val2_r  to calculate floor of
    //! $Source: vfc_siunits.hpp $
    //! @author    gaj2kor
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> (min)
                    (const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_val1_r,
                     const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_val2_r);

    //---------------------------------------------------------------------
    //! To calculate max
    //! @param     f_val1_r  data to be compared with
    //! @param     f_val2_r  to calculate floor of
    //! $Source: vfc_siunits.hpp $
    //! @author    gaj2kor
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> (max)
                    (const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_val1_r,
                     const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_val2_r);

    //---------------------------------------------------------------------
    //! To calculate sqr
    //! @param     f_val_r  to calculate square of
    //! $Source: vfc_siunits.hpp $
    //! @author    gaj2kor
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    const vfc::TSIUnits<ValueType, typename vfc::TSIUnitSqrPromote<UnitInfoType>::unit_info_type,
            typename vfc::TSIUnitRBSqrPromote<RBInfoType>::unit_info_type, UserType, ConvertPolicyType> sqr
                    ( const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_val_r);

    //---------------------------------------------------------------------
    //! To calculate sqrt
    //! @param     f_val_r  to calculate square root of
    //! $Source: vfc_siunits.hpp $
    //! @author    gaj2kor
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    const vfc::TSIUnits<ValueType, typename vfc::TSIUnitSqrtPromote<UnitInfoType>::unit_info_type,
            typename vfc::TSIUnitRBSqrtPromote<RBInfoType>::unit_info_type, UserType, ConvertPolicyType> sqrt
                    ( vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> f_val_r);

    //---------------------------------------------------------------------
    //! To calculate abs
    //! @param     f_val_r  to calculate absolute of
    //! $Source: vfc_siunits.hpp $
    //! @author    gaj2kor
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> abs
                    ( const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_val_r);

    
    //---------------------------------------------------------------------
    //! To Find NAN
    //! @param     f_NANValue  to find not a number
    //! $Source: vfc_siunits.hpp $
    //! @return    returns true if given value is NAN, false otherwise
    //! @author    nva1cob
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    bool isNAN( const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_nanValue_r);


    //---------------------------------------------------------------------
    //! To find INF
    //! @param     f_INFValue  to find infinity or not
    //! $Source: vfc_siunits.hpp $
    //! @return    returns true if given value is +INF or -INF, false otherwise.
    //! @author    nva1cob
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    bool isINF( const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_infValue_r);
    

    //---------------------------------------------------------------------
    //! To find value is zero or not
    //! @param     f_zeroValue  to find  value is zero or not
    //! $Source: vfc_siunits.hpp $
    //! @return    returns true if specified value equals zero, false otherwise.
    //! @author    nva1cob
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    bool isZero( const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_zeroValue_r);


    //---------------------------------------------------------------------
    //! To find value is notzero 
    //! @param     f_notzeroValue  to find  value is notzero 
    //! $Source: vfc_siunits.hpp $
    //! @return    returns true if specified value is NOT zero.
    //! @author    nva1cob
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    bool notZero( const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_notzeroValue_r);


    //---------------------------------------------------------------------
    //! To find value is positive 
    //! @param     f_positiveValue  to find  value is positive
    //! $Source: vfc_siunits.hpp $
    //! @return    returns true if specified value is greater zero 
    //! @author    nva1cob
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    bool isPositive( const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_positiveValue_r);

    //---------------------------------------------------------------------
    //! To find value is notpositive 
    //! @param     f_notpositiveValue  to find  value is notpositive
    //! $Source: vfc_siunits.hpp $
    //! @return    returns true if specified value is zero or less 
    //! @author    nva1cob
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    bool notPositive( const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_notpositiveValue_r);


    //---------------------------------------------------------------------
    //! To find value is negative
    //! @param     f_negativeValue  to find  value is negative
    //! $Source: vfc_siunits.hpp $
    //! @return    returns true, if specified value is less zero
    //! @author    nva1cob
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    bool isNegative( const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_negativeValue_r);


    //---------------------------------------------------------------------
    //! To find value is notnegative
    //! @param     f_notnegativeValue  to find  value is not negative
    //! $Source: vfc_siunits.hpp $
    //! @return    returns true, if specified value is zero or greater
    //! @author    nva1cob
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    bool notNegative( const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_notnegativeValue_r);

    
    //---------------------------------------------------------------------
    //! To find values are equal
    //! @param     f_testValueA  first value for comparision
    //! @param     f_testValueB  second value for comparision
    //! $Source: vfc_siunits.hpp $
    //! @return    returns true if specified values are equal, false otherwise
    //! @author    nva1cob
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    bool isEqual( const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_testValueA_r,
                  const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_testValueB_r);

    
    //---------------------------------------------------------------------
    //! To find values are equal with epsilon
    //! @param     f_testValueA  first value for comparision
    //! @param     f_testValueB  second value for comparision
    //! @param     f_testEpsilon  epsilon value for comparision
    //! $Source: vfc_siunits.hpp $
    //! @return    returns true if specified values are equal within given epsilon
    //! @author    nva1cob
    //---------------------------------------------------------------------
    template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    bool isEqual( const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_testValueA_r,
                  const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_testValueB_r,
                  const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_testEpsilon_r);


    //---------------------------------------------------------------------
    //! To find values are not equal
    //! @param     f_testValueA  first value for comparision
    //! @param     f_testValueB  second value for comparision
    //! $Source: vfc_siunits.hpp $
    //! @return    returns true if specified values are NOT equal, false otherwise
    //! @author    nva1cob
    //---------------------------------------------------------------------
   template<class ValueType,
        class UnitInfoType,
        class RBInfoType,
        class UserType ,
        template<class, class, class> class ConvertPolicyType >
    bool notEqual( const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_testValueA_r,
                   const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_testValueB_r);


      

}// closed namespace vfc

#include "vfc/core/vfc_siunits.inl"

#endif //SIUNITS_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_siunits.hpp  $
//  Revision 1.10 2014/07/21 09:22:21MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Use an extra set of parentheses for std::min/max() function calls (mantis0004452)
//  Revision 1.9 2014/07/18 09:55:47MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - si units: resolve QAC++ warning 3336: This call may be hijacked via template argument to one of the types. (mantis0004602)
//  Revision 1.8 2014/05/12 10:40:50MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - SI-Units: kg=cm!? (mantis0004478)
//  Revision 1.7 2014/01/28 14:21:56MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - SI Units default-ctor performance issue (mantis0004266)
//  Revision 1.6 2013/01/15 08:49:08MEZ Gaurav Jain (RBEI/ESD4) (gaj2kor) 
//  -Incorporated all the proposed code under SIUnit (mantis 4184)
//  Revision 1.5 2012/12/18 12:57:31IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.4 2012/10/23 11:41:03MESZ Sudhakar Nannapaneni (RBEI/ESD1) (SNU5KOR) 
//  -Added support for the new module TRect under vfc::core (mantis3349)
//  Revision 1.3 2011/04/14 13:37:10MESZ Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) 
//  - Functional -- -- Additional functions for SI-units.
//  Revision 1.2 2010/09/22 13:30:13IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - Siunits fail with the cw85 compiler (mantis3389)
//  Revision 1.1 2010/08/11 17:57:07MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/core/core.pj
//  Revision 1.19 2010/07/14 09:33:30MESZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Addidtion of atan function. (mantis 0003347)
//  Revision 1.18 2010/07/12 12:35:30CEST Pavithra K T (RBEI/ESB2) (pkt1kor) 
//  - Global functions put into vfc namespace.(mantis : 0003348)
//  Revision 1.17 2010/06/14 17:02:27IST Gaurav Jain (RBEI/ESB2) (gaj2kor)
//  - Inclusion of TypePromotion for sqr and sqrt functions. (mantis : 0003341)
//  Revision 1.15 2010/06/02 13:34:07CEST Gaurav Jain (RBEI/EAS3) (gaj2kor)
//  -Introduction of RB Types as new template parameter. (Mantis :0003259)
//  Revision 1.14 2010/03/11 10:53:25GMT+05:30 Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Inclusion of new SIUnits UserType (like "pixel", "coord system").
//  Revision 1.13 2009/03/06 17:12:12GMT+05:30 Dilip Krishna (RBEI/EAS6) (dkn2kor)
//  - conversion ctor made non explicit to be backward compatible with earlier version (mantis 2586)
//  Revision 1.12 2009/02/05 10:20:45IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Removal of QAC++ warnings.
//  (Mantis : 0002564)
//  Revision 1.11 2009/01/23 16:00:41IST Dilip Krishna (RBEI/EAC1) (dkn2kor)
//  - qacpp warnings fixed
//  Revision 1.10 2008/12/30 09:14:28IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  - compilation problem due to policy type in gcc corrected
//  Revision 1.9 2008/12/11 17:01:14IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  - major redesign to accomodate the conversion between different types
//  Revision 1.8 2008/10/09 17:12:03IST Jaeger Thomas (CC-DA/ESV1) (JAT2HI)
//  - updates
//  Revision 1.7 2008/09/22 15:22:43CEST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  - usertype typedef added while working on (Mantis :- 0002352)
//  Revision 1.6 2008/09/17 11:18:29IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  -add support for electric current (A) (mantis:- 2328)
//  -add support for Candela (mantis:- 2340)
//  Revision 1.5 2008/08/20 18:26:37IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  SiUnit new Design implementation
//  - Implicit conversion etc.
//  Revision 1.4 2008/06/11 19:51:08IST Venkatesh Somanahalli (RBEI/EAE6) (ves3kor)
//  copy ctor and assignment operator has been removed and UserType added in the operator&
//  Revision 1.3 2008/06/03 18:32:08IST Venkatesh Somanahalli (RBEI/EAE6) (ves3kor)
//  -Default value "0" for all the ValueTypes has been used,The ValueType2 has been removed for POD operations,use of std 'int' been replaced with vfc::int32_t types
//  Revision 1.2 2008/05/09 17:42:42IST Venkatesh Somanahalli (RBEI/EAE6) (ves3kor)
//  added the free operator * and operator /
//  Revision 1.1 2008/04/28 17:24:18IST Venkatesh Somanahalli (RBEI/EAE6) (ves3kor)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_siunits/vfc_siunits.pj
//  Revision 1.2 2008/03/19 18:45:35IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  - implemented missing operators, added generic type capability to siunits
//  Revision 1.1 2008/03/07 19:56:29IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/private_workspace/jat2hi/test/appl_sw/versatile/pc_ivs2_siunits/devl/include/siunits/siunits_user_include.pj
//=============================================================================


