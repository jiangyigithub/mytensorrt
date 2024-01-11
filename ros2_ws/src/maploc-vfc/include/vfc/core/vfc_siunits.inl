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
/// @brief Implementation file.
///
/// @par Revision History:
///     $Source: vfc_siunits.inl $
///     $Revision: 1.5 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/09/24 14:20:32MESZ $
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
#include "vfc/core/vfc_siunits_helper.hpp"  // For UnitInfoType & conversions
#include "vfc/core/vfc_math.hpp"            // For sqr, abs
#include <cmath>                            // For floor, sin, cos


 // tan function for si-unit
    template<class ValueType, class UnitInfo1Type, class RBInfoType, class UserType,
             template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType,typename vfc::TSIUnitSinCosPromote<UnitInfo1Type>::unit_info_type,
                        RBInfoType, UserType, ConvertPolicyType>
                        vfc::tan (vfc::TSIUnits<ValueType, UnitInfo1Type, RBInfoType, UserType, ConvertPolicyType> f_val)
    {
        VFC_STATIC_ASSERT(( (vfc::TIsValidAngleDominant<UnitInfo1Type>::value) || (UnitInfo1Type::SI_UNIT == 0) ));
        vfc::TBasicTypeConvert<typename UnitInfo1Type::angle_unit_type, vfc::CAngleType>::performConversion(f_val.value());
        return (vfc::TSIUnits<ValueType,
                              typename vfc::TSIUnitSinCosPromote<UnitInfo1Type>::unit_info_type,
                              RBInfoType, UserType,
                              ConvertPolicyType>(stlalias::tan(f_val.value())));
    }

    // sin function for si-unit
    template<class ValueType, class UnitInfo1Type, class RBInfoType, class UserType,
             template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType, typename vfc::TSIUnitSinCosPromote<UnitInfo1Type>::unit_info_type,
                        RBInfoType, UserType, ConvertPolicyType>
                        vfc::sin (vfc::TSIUnits<ValueType, UnitInfo1Type, RBInfoType, UserType, ConvertPolicyType> f_val)
    {
        VFC_STATIC_ASSERT(( (vfc::TIsValidAngleDominant<UnitInfo1Type>::value) || (UnitInfo1Type::SI_UNIT == 0) ));
        vfc::TBasicTypeConvert<typename UnitInfo1Type::angle_unit_type, vfc::CAngleType>::performConversion(f_val.value());
        return (vfc::TSIUnits<ValueType,
                              typename vfc::TSIUnitSinCosPromote<UnitInfo1Type>::unit_info_type,
                              RBInfoType, UserType,
                              ConvertPolicyType>(stlalias::sin(f_val.value())));
    }

    // cos function for si-unit
    template<class ValueType, class UnitInfo1Type, class RBInfoType, class UserType,
             template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType,typename vfc::TSIUnitSinCosPromote<UnitInfo1Type>::unit_info_type,
                        RBInfoType, UserType, ConvertPolicyType>
        vfc::cos (vfc::TSIUnits<ValueType, UnitInfo1Type, RBInfoType, UserType, ConvertPolicyType> f_val)
    {
        VFC_STATIC_ASSERT(( (vfc::TIsValidAngleDominant<UnitInfo1Type>::value) || (UnitInfo1Type::SI_UNIT == 0) ));
        vfc::TBasicTypeConvert<typename UnitInfo1Type::angle_unit_type, vfc::CAngleType>::performConversion(f_val.value());
        return (vfc::TSIUnits<ValueType,
                              typename vfc::TSIUnitSinCosPromote<UnitInfo1Type>::unit_info_type,
                              RBInfoType, UserType,
                              ConvertPolicyType>(stlalias::cos(f_val.value())));
    }

    // atan function for si-unit
    template<class ValueType, class UnitInfo1Type, class RBInfoType, class UserType,
             template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType,typename vfc::TSIUnitAtanPromote<UnitInfo1Type>::unit_info_type,
                        RBInfoType, UserType, ConvertPolicyType>
                        vfc::atan (vfc::TSIUnits<ValueType, UnitInfo1Type, RBInfoType, UserType, ConvertPolicyType> f_val)
    {
        VFC_STATIC_ASSERT(( (UnitInfo1Type::ANGLE_POWER_VALUE == 0) && (vfc::TIsNegativePower<UnitInfo1Type>::value) ));
        return (vfc::TSIUnits<ValueType,
                              typename vfc::TSIUnitAtanPromote<UnitInfo1Type>::unit_info_type,
                              RBInfoType, UserType,
                              ConvertPolicyType>(stlalias::atan(f_val.value())));
    } 

    // floor function for si-unit
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
             template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>
        vfc::floor (vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> f_val)
    {
        return (vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>(
                                                                        stlalias::floor(f_val.value())));
    }

    // min function for si-unit
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
             template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>
        vfc::min (
        const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_val_l,
        const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_val_r)
    {
        return (vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>(
                                                        stlalias::min(f_val_l.value(), f_val_r.value())));
    }

    // max function for si-unit
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
             template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>
        vfc::max (
        const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_val1_r,
        const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_val2_r)
    {
        return (vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>(
                                                       stlalias::max(f_val1_r.value(), f_val2_r.value())));
    }

    // sqr function for si-unit
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
             template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType, typename vfc::TSIUnitSqrPromote<UnitInfoType>::unit_info_type,
            typename vfc::TSIUnitRBSqrPromote<RBInfoType>::unit_info_type, UserType, ConvertPolicyType>
        vfc::sqr (const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_val)
    {

        return (vfc::TSIUnits<ValueType,
                typename vfc::TSIUnitSqrPromote<UnitInfoType>::unit_info_type,
                typename vfc::TSIUnitRBSqrPromote<RBInfoType>::unit_info_type, UserType, ConvertPolicyType>(
                                                                                vfc::sqr(f_val.value())));
    }

    // sqrt function for si-unit
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
             template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType, typename vfc::TSIUnitSqrtPromote<UnitInfoType>::unit_info_type,
            typename vfc::TSIUnitRBSqrtPromote<RBInfoType>::unit_info_type, UserType, ConvertPolicyType>
        vfc::sqrt (vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType> f_val)
    {
        return (vfc::TSIUnits<ValueType,
                typename vfc::TSIUnitSqrtPromote<UnitInfoType>::unit_info_type,
                typename vfc::TSIUnitRBSqrtPromote<RBInfoType>::unit_info_type, UserType, ConvertPolicyType>(
                                                                               vfc::sqrt(f_val.value())));
    }


    // abs function for si-unit
    template<class ValueType, class UnitInfoType, class RBInfoType, class UserType,
             template<class, class, class> class ConvertPolicyType>
    const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>
        vfc::abs (const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_val)
    {
        return (vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>(
                                                                                vfc::abs(f_val.value())));
    }


    // isNAN function for si-unit
    template<class ValueType,class UnitInfoType,class RBInfoType,class UserType ,
    template<class, class, class> class ConvertPolicyType >
    bool vfc::isNAN (const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_nanValue_r)
    {
        ValueType l_nanValue = f_nanValue_r.value();
        return vfc::isNAN(l_nanValue);
    }


    // isINF function for si-unit
    template<class ValueType, class UnitInfoType,class RBInfoType,class UserType ,
    template<class, class, class> class ConvertPolicyType >
    bool vfc::isINF (const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_infValue_r)
    {
        ValueType l_infValue = f_infValue_r.value();
        return vfc::isINF(l_infValue);
    }


    // isZero function for si-unit
    template<class ValueType,class UnitInfoType,class RBInfoType,class UserType ,
    template<class, class, class> class ConvertPolicyType >
    bool vfc::isZero(const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_zeroValue_r)
    {
        ValueType l_zeroValue = f_zeroValue_r.value();
        return vfc::isZero(l_zeroValue);
    }


    // notZero function for si-unit
    template<class ValueType,class UnitInfoType,class RBInfoType,class UserType ,
    template<class, class, class> class ConvertPolicyType >
    bool vfc::notZero(const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_notzeroValue_r)
    {
        ValueType l_notzeroValue = f_notzeroValue_r.value();
        return vfc::notZero(l_notzeroValue);
    }


    // ispositive function for si-unit
    template<class ValueType,class UnitInfoType,class RBInfoType,class UserType ,
    template<class, class, class> class ConvertPolicyType >
    bool vfc::isPositive(const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_positiveValue_r)
    {
        ValueType l_positiveValue = f_positiveValue_r.value();
        return vfc::isPositive(l_positiveValue);
    }



    // notPositive function for si-unit
    template<class ValueType,class UnitInfoType,class RBInfoType,class UserType ,
    template<class, class, class> class ConvertPolicyType >
    bool vfc::notPositive(const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_notpositiveValue_r)
    {
        ValueType l_notpositiveValue = f_notpositiveValue_r.value();
        return vfc::notPositive(l_notpositiveValue);
    }


    // isNegative function for si-unit
    template<class ValueType,class UnitInfoType,class RBInfoType,class UserType ,
    template<class, class, class> class ConvertPolicyType >
    bool vfc::isNegative(const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_negativeValue_r)
    {
        ValueType l_negativeValue = f_negativeValue_r.value();
        return vfc::isNegative(l_negativeValue);
    }


    // notNegative function for si-unit
    template<class ValueType,class UnitInfoType,class RBInfoType,class UserType ,
    template<class, class, class> class ConvertPolicyType >
    bool vfc::notNegative(const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_notnegativeValue_r)
    {
        ValueType l_notnegativeValue = f_notnegativeValue_r.value();
        return vfc::notNegative(l_notnegativeValue);
    }


   
    // isEqual function for si-unit
    template<class ValueType,class UnitInfoType,class RBInfoType,class UserType ,
    template<class, class, class> class ConvertPolicyType >
    bool vfc::isEqual(const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_testValueA_r,
                      const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_testValueB_r )
    {
        ValueType l_valueA = f_testValueA_r.value();
        ValueType l_valueB = f_testValueB_r.value();
        return vfc::isEqual(l_valueA,l_valueB);
    }


    // isEqual function for si-unit
    template<class ValueType,class UnitInfoType,class RBInfoType,class UserType ,
    template<class, class, class> class ConvertPolicyType >
    bool vfc::isEqual(const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_testValueA_r,
                      const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_testValueB_r,
                      const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_testEpsilonValue_r )
    {
        ValueType l_valueA = f_testValueA_r.value();
        ValueType l_valueB = f_testValueB_r.value();
        ValueType l_epilisonValue = f_testEpsilonValue_r.value();
        return vfc::isEqual(l_valueA,l_valueB,l_epilisonValue);
    }


    // notEqual function for si-unit
    template<class ValueType,class UnitInfoType,class RBInfoType,class UserType ,
    template<class, class, class> class ConvertPolicyType >
    bool vfc::notEqual(const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_testValueA_r,
                       const vfc::TSIUnits<ValueType, UnitInfoType, RBInfoType, UserType, ConvertPolicyType>& f_testValueB_r)
    {
        ValueType l_valueA = f_testValueA_r.value();
        ValueType l_valueB = f_testValueB_r.value();
        return vfc::notEqual(l_valueA,l_valueB);
    }


    
//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_siunits.inl  $
//  Revision 1.5 2014/09/24 14:20:32MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_siunits: correct the Degree/Radian handling (mantis0004533)
//  Revision 1.4 2012/12/18 08:27:31MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.3 2011/04/14 13:43:03MESZ Vanitha Nagarajan (RBEI/ESB3) (NVA1COB) 
//  - Functional - -Mantis3670 -- Additional functions for SI-units
//  Revision 1.2 2010/09/22 13:30:15IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - Siunits fail with the cw85 compiler (mantis3389)
//  Revision 1.1 2010/08/11 17:57:07MESZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/core/core.pj
//  Revision 1.14 2010/07/14 09:33:10MESZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Addidtion of atan function. (mantis 0003347)
//  Revision 1.13 2010/07/12 12:35:28CEST Pavithra K T (RBEI/ESB2) (pkt1kor) 
//  - Global functions put into vfc namespace.(mantis : 0003348)
//  Revision 1.12 2010/06/14 17:02:27IST Gaurav Jain (RBEI/ESB2) (gaj2kor)
//  - Inclusion of TypePromotion for sqr and sqrt functions. (mantis : 0003341)
//  Revision 1.10 2010/06/02 13:34:07CEST Gaurav Jain (RBEI/EAS3) (gaj2kor)
//  -Introduction of RB Types as new template parameter. (Mantis :0003259)
//  Revision 1.9 2010/03/11 10:53:11GMT+05:30 Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Inclusion of new SIUnits UserType (like "pixel", "coord system").
//  Revision 1.8 2008/12/11 17:01:14GMT+05:30 Dilip Krishna (RBEI/EAS6) (dkn2kor)
//  - major redesign to accomodate the conversion between different types
//  Revision 1.7 2008/10/09 17:12:02IST Jaeger Thomas (CC-DA/ESV1) (JAT2HI)
//  - updates
//  Revision 1.6 2008/09/17 07:48:35CEST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  -add support for electric current (A) (mantis:- 2328)
//  -add support for Candela (mantis:- 2340)
//  Revision 1.5 2008/08/20 18:26:38IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  SiUnit new Design implementation
//  - Implicit conversion etc.
//  Revision 1.4 2008/06/11 19:56:18IST Venkatesh Somanahalli (RBEI/EAE6) (ves3kor)
//  copy ctor and assignment operator has been removed and UserType has been added in operator&
//  Revision 1.3 2008/06/03 18:44:18IST Venkatesh Somanahalli (RBEI/EAE6) (ves3kor)
//  -Default value "0" for all the ValueTypes has been used.
//  -The ValueType2 has been removed for POD operations.
//  -Use of std 'int' been replaced with vfc::int32_t types.
//  -All the implementation part is  done in this file.
//  Revision 1.2 2008/05/09 17:42:52IST Venkatesh Somanahalli (RBEI/EAE6) (ves3kor)
//  added the free operator * and operator /
//  Revision 1.1 2008/04/28 17:24:19IST Venkatesh Somanahalli (RBEI/EAE6) (ves3kor)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_siunits/vfc_siunits.pj
//  Revision 1.2 2008/03/19 18:45:42IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  - implemented missing operators, added generic type capability to siunits
//  Revision 1.1 2008/03/07 19:56:31IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/private_workspace/jat2hi/test/appl_sw/versatile/pc_ivs2_siunits/devl/include/siunits/siunits_user_include.pj
//=============================================================================



