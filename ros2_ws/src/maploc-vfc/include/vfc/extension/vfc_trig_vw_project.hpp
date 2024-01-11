//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2007 by Robert Bosch GmbH. All rights reserved.
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
///     $Source: vfc_trig.hpp $
///     $Revision: 1.15 $
///     $Author: gaj2kor $
///     $Date: 2009/02/09 06:15:42MEZ $
///     $Locker:  $
///     $Name: 0032 RC1  $
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

/// version used in the cc-da vw gen5 project with templated radian and degree types

#ifndef VFC_TRIG_VW_PROJECT_HPP_INCLUDED
#define VFC_TRIG_VW_PROJECT_HPP_INCLUDED

#include <math.h>                        // used for trig funcs
#include "vfc/core/vfc_static_assert.hpp"   // used for VFC_STATIC_ASSERT()
#include "vfc/core/vfc_types.hpp"        // used for fundamental types
#include "vfc/core/vfc_type_traits.hpp"    // used for VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL
#include "vfc/core/vfc_math.hpp"        // used for G_DEG2RAD and G_RAD2DEG constants, isEqual()

namespace vfc
{    // namespace vfc opened

    ///////////////////////////////////////////////////////////////////////////
    // based on ideas for safe radian/degree utilization
    // from CR/AEM4-Jaeger
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    // some forward declarations
    ///////////////////////////////////////////////////////////////////////////

    template<class FloatType>
    class TDegree;

    //=========================================================================
    // CRadian
    //-------------------------------------------------------------------------
    //! Type-Safe angle representation in radians.
    //! @sa CDegree
    //! @ingroup vfc_group_core_types
    //=========================================================================

    template<class FloatType>
    class TRadian
    {
    public:
        VFC_STATIC_ASSERT((TIsFloating<FloatType>::value));

        // enable access to underlying type in algorithms
        typedef FloatType value_type;

        //! c'tor, takes angle in radians
        explicit
        TRadian(value_type f_angleInRadians = typedZero<value_type>()) : m_value(f_angleInRadians) {}

        //PRQA S 2180 ++
        //! implicit conversion from degree to radian angle
        TRadian(const TDegree<value_type>& angle);
        //PRQA S 2180 --

        //! returns angle value in radians
        value_type value(void) const { return m_value;}
        //! add-assign operator
        const TRadian&  operator+= (const TRadian& rhs) { m_value+=rhs.m_value; return *this;}
        //! subtract-assign operator
        const TRadian&  operator-= (const TRadian& rhs) { m_value-=rhs.m_value; return *this;}
        //! multiply-assign operator
        const TRadian&  operator*= (value_type f_rhs)      { m_value*=f_rhs; return *this;}
        //! divide-assign operator
        const TRadian&  operator/= (value_type f_rhs)      { m_value/=f_rhs; return *this;}

        /// extend to a type with higher precision
        template<class OtherFloatType>
        TRadian<OtherFloatType> extend_to() const {
          VFC_STATIC_ASSERT((TIsFloating<OtherFloatType>::value));
          VFC_STATIC_ASSERT(sizeof(OtherFloatType) > sizeof(value_type));
          return TRadian<OtherFloatType>(numeric_cast<OtherFloatType>(value()));
        }

        /// truncate to a type with lower precision
        template<class OtherFloatType>
        TRadian<OtherFloatType> truncate_to() const {
          VFC_STATIC_ASSERT((TIsFloating<OtherFloatType>::value));
          VFC_STATIC_ASSERT(sizeof(value_type) > sizeof(OtherFloatType));
          return TRadian<OtherFloatType>(numeric_cast<OtherFloatType>(value()));
        }

    private:
        value_type    m_value;
    };

    /// typedef for backward compatibility/transition
    typedef TRadian<float32_t> CRadian32;
    typedef TRadian<float64_t> CRadian64;

    // traits specialization
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsPOD,CRadian64,true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsPOD,CRadian32,true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating,CRadian64,true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating,CRadian32,true);

    //! returns angle differnce in rad. @relates CRadian
    template<class FloatType>
    inline
    const   TRadian<FloatType> operator-(const TRadian<FloatType>& lhs, const TRadian<FloatType>& rhs)
    {
        return TRadian<FloatType>(lhs).operator -= (rhs);
    }

    //! returns angle sum in rad. @relates TRadian
    template<class FloatType>
    inline
    const   TRadian<FloatType> operator+(const TRadian<FloatType>& lhs, const TRadian<FloatType>& rhs)
    {
        return TRadian<FloatType>(lhs).operator += (rhs);
    }

    //! returns angle times given factor. @relates TRadian
    template<class FloatType>
    inline
    const   TRadian<FloatType>    operator*(const TRadian<FloatType>& lhs, FloatType f_rhs)
    {
        return TRadian<FloatType>(lhs).operator *= (f_rhs);
    }

    //! returns angle times given factor. @relates TRadian
    template<class FloatType>
    inline
    const   TRadian<FloatType>    operator*(FloatType f_lhs, const TRadian<FloatType>& rhs)
    {
        return TRadian<FloatType>(rhs).operator *= (f_lhs);
    }

    //! returns angle divided by denom. @relates TRadian
    template<class FloatType>
    inline
    const   TRadian<FloatType>    operator/(const TRadian<FloatType>& lhs, FloatType f_rhs)
    {
        return TRadian<FloatType>(lhs).operator /= (f_rhs);
    }

    //! unary neg operator. @relates TRadian
    template<class FloatType>
    inline
    const   TRadian<FloatType>    operator-(const TRadian<FloatType>& angle_rad)              {    return TRadian<FloatType>(-angle_rad.value());}

    // comparison operators

    //! comparison operator for equality. @relates TRadian
    template<class FloatType>
    inline
    bool operator== (const TRadian<FloatType>& lhs, const TRadian<FloatType>& rhs)    {    return isEqual(lhs.value(), rhs.value());}

    //! comparison operator for inequality. @relates TRadian
    template<class FloatType>
    inline
    bool operator!= (const TRadian<FloatType>& lhs, const TRadian<FloatType>& rhs)    {    return notEqual(lhs.value(), rhs.value());}

    //! comparison operator for less. @relates TRadian
    template<class FloatType>
    inline
    bool operator<  (const TRadian<FloatType>& lhs, const TRadian<FloatType>& rhs)    {    return lhs.value()<rhs.value();}

    //! comparison operator for greater. @relates TRadian
    template<class FloatType>
    inline
    bool operator>  (const TRadian<FloatType>& lhs, const TRadian<FloatType>& rhs)    {    return lhs.value()>rhs.value();}

    //! comparison operator for less-equal. @relates TRadian
    template<class FloatType>
    inline
    bool operator<= (const TRadian<FloatType>& lhs, const TRadian<FloatType>& rhs)    {    return lhs.value()<=rhs.value();}

    //! comparison operator for greater-equal. @relates TRadian
    template<class FloatType>
    inline
    bool operator>= (const TRadian<FloatType>& lhs, const TRadian<FloatType>& rhs)    {    return lhs.value()>=rhs.value();}

    //=========================================================================
    // TDegree
    //-------------------------------------------------------------------------
    //! Type-Safe angle representation in degrees.
    //! @sa TRadian
    //! @ingroup vfc_group_core_types
    //=========================================================================

    template<class FloatType>
    class TDegree
    {
    public:
        VFC_STATIC_ASSERT((TIsFloating<FloatType>::value));

        // enable access to underlying type in algorithms
        typedef FloatType value_type;

        //! c'tor, takes angle in degrees
        explicit
        TDegree(value_type f_angleInDegrees = typedZero<value_type>()) : m_value(f_angleInDegrees) {}

        //PRQA S 2180 ++
        //! implicit conversion from radian to degree angle
        TDegree(const TRadian<value_type>& angle);
        //PRQA S 2180 --

        //! returns angle value in degrees
        value_type value(void) const { return m_value;}
        //! add-assign operator
        const TDegree&  operator+= (const TDegree& rhs) { m_value+=rhs.m_value; return *this;}
        //! subtract-assign operator
        const TDegree&  operator-= (const TDegree& rhs) { m_value-=rhs.m_value; return *this;}
        //! multiply-assign operator
        const TDegree&  operator*= (value_type f_rhs) { m_value*=f_rhs; return *this;}
        //! divide-assign operator
        const TDegree&  operator/= (value_type f_rhs) { m_value/=f_rhs; return *this;}

        /// extend to a type with higher precision
        template<class OtherFloatType>
        TDegree<OtherFloatType> extend_to() const {
          VFC_STATIC_ASSERT((TIsFloating<OtherFloatType>::value));
          VFC_STATIC_ASSERT(sizeof(OtherFloatType) > sizeof(value_type));
          return TDegree<OtherFloatType>(numeric_cast<OtherFloatType>(value()));
        }

        /// truncate to a type with lower precision
        template<class OtherFloatType>
        TDegree<OtherFloatType> truncate_to() const {
          VFC_STATIC_ASSERT((TIsFloating<OtherFloatType>::value));
          VFC_STATIC_ASSERT(sizeof(value_type) > sizeof(OtherFloatType));
          return TDegree<OtherFloatType>(numeric_cast<OtherFloatType>(value()));
        }
    private:
        value_type    m_value;
    };

    /// typedef for backward compatibility/transition
    typedef TDegree<float32_t> CDegree32;
    typedef TDegree<float64_t> CDegree64;

    // conversion c'tor implementations
    template<class FloatType>
    inline
    TRadian<FloatType>::TRadian(const TDegree<FloatType>& angle) : m_value(angle.value() * static_cast<FloatType>(G_DEG2RAD)) {}

    template<class FloatType>
    inline
    TDegree<FloatType>::TDegree(const TRadian<FloatType>& angle) : m_value(angle.value() * static_cast<FloatType>(G_RAD2DEG)) {}

    //
    // full specializations of numeric_cast for changing precision
    //

    /// numeric_cast TDegree<float64_t> to TDegree<float32_t>
    template<>
    inline
    TDegree<float32_t> numeric_cast<TDegree<float32_t> >(const TDegree<float64_t>& f_degree)
    {
        return TDegree<float32_t>(numeric_cast<float32_t>(f_degree.value()));
    }

    /// numeric_cast TDegree<float32_t> to TDegree<float_64_t>
    template<>
    inline
    TDegree<float64_t> numeric_cast<TDegree<float64_t> >(const TDegree<float32_t>& f_degree)
    {
        return TDegree<float64_t>(numeric_cast<float64_t>(f_degree.value()));
    }

    /// numeric_cast TRadian<float64_t> to TRadian<float32_t>
    template<>
    inline
    TRadian<float32_t> numeric_cast<TRadian<float32_t> >(const TRadian<float64_t>& f_radian)
    {
        return TRadian<float32_t>(numeric_cast<float32_t>(f_radian.value()));
    }

    /// numeric_cast TRadian<float32_t> to TRadian<float_64_t>
    template<>
    inline
    TRadian<float64_t> numeric_cast<TRadian<float64_t> >(const TRadian<float32_t>& f_radian)
    {
        return TRadian<float64_t>(numeric_cast<float64_t>(f_radian.value()));
    }

    // traits specialization
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsPOD,CDegree64,true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsPOD,CDegree32,true);

    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating,CDegree64,true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsFloating,CDegree32,true);

    //! returns angle differnce in degrees. @relates TDegree
    template<class FloatType>
    inline
    const TDegree<FloatType> operator-(const TDegree<FloatType>& lhs, const TDegree<FloatType>& rhs)
    {
        return TDegree<FloatType>(lhs).operator -= (rhs);
    }

    //! returns angle sum in degrees. @relates TDegree
    template<class FloatType>
    inline
    const TDegree<FloatType> operator+(const TDegree<FloatType>& lhs, const TDegree<FloatType>& rhs)
    {
        return TDegree<FloatType>(lhs).operator += (rhs);
    }

    //! returns angle times given factor. @relates TDegree
    template<class FloatType>
    inline
    const TDegree<FloatType>    operator*(const TDegree<FloatType>& lhs, FloatType f_rhs)
    {
        return TDegree<FloatType>(lhs).operator *= (f_rhs);
    }

    //! returns angle times given factor. @relates TDegree
    template<class FloatType>
    inline
    const TDegree<FloatType>    operator*(FloatType f_lhs, const TDegree<FloatType>& rhs)
    {
        return TDegree<FloatType>(rhs).operator *= (f_lhs);
    }

    //! returns angle divided by denom. @relates TDegree
    template<class FloatType>
    inline
    const TDegree<FloatType>    operator/(const TDegree<FloatType>& lhs, FloatType f_rhs)
    {
        return TDegree<FloatType>(lhs).operator /= (f_rhs);
    }

    //! unary neg operator. @relates TDegree
    template<class FloatType>
    inline
    const TDegree<FloatType>    operator-(const TDegree<FloatType>& angle_rad)                {    return TDegree<FloatType>(-angle_rad.value());}

    //! comparison operator for equality
    template<class FloatType>
    inline
    bool operator== (const TDegree<FloatType>& lhs, const TDegree<FloatType>& rhs)    {    return isEqual(lhs.value(), rhs.value());}

    //! comparison operator for inequality. @relates TDegree
    template<class FloatType>
    inline
    bool operator!= (const TDegree<FloatType>& lhs, const TDegree<FloatType>& rhs)    {    return notEqual(lhs.value(), rhs.value());}

    //! comparison operator for less. @relates TDegree
    template<class FloatType>
    inline
    bool operator<  (const TDegree<FloatType>& lhs, const TDegree<FloatType>& rhs)    {    return lhs.value()<rhs.value();}

    //! comparison operator for greater. @relates TDegree
    template<class FloatType>
    inline
    bool operator>  (const TDegree<FloatType>& lhs, const TDegree<FloatType>& rhs)    {    return lhs.value()>rhs.value();}

    //! comparison operator for less-equal. @relates TDegree
    template<class FloatType>
    inline
    bool operator<= (const TDegree<FloatType>& lhs, const TDegree<FloatType>& rhs)    {    return lhs.value()<=rhs.value();}

    //! comparison operator for greater-equal. @relates TDegree
    template<class FloatType>
    inline
    bool operator>= (const TDegree<FloatType>& lhs, const TDegree<FloatType>& rhs)    {    return lhs.value()>=rhs.value();}

    ///////////////////////////////////////////////////////////////////////////
    // trig function wrappers
    ///////////////////////////////////////////////////////////////////////////

    //! sin function wrapper. @relates TRadian
    inline
    float64_t    sin    (const CRadian64& angle)    {   return ::sin(angle.value());}

    //! sin function wrapper, float32. @relates TRadian
    inline
    float32_t    sin    (const CRadian32& angle)    {   return ::sinf(angle.value());}

    //! cos function wrapper. @relates TRadian
    inline
    float64_t   cos    (const CRadian64& angle)    {   return ::cos(angle.value());}

    //! cos function wrapper, float32. @relates TRadian
    inline
    float32_t   cos    (const CRadian32& angle)    {   return ::cosf(angle.value());}

    //! tan function wrapper. @relates TRadian
    inline
    float64_t   tan    (const CRadian64& angle)    {   return ::tan(angle.value());}

    //! tan function wrapper, float32. @relates TRadian
    inline
    float32_t   tan    (const CRadian32& angle)    {   return ::tanf(angle.value());}

    //! sinh function wrapper. @relates TRadian
    inline
    float64_t   sinh(const CRadian64& angle)    {   return ::sinh(angle.value());}

    //! sinh function wrapper, float32. @relates TRadian
    inline
    float32_t   sinh(const CRadian32& angle)    {   return ::sinhf(angle.value());}

    //! cosh function wrapper. @relates TRadian
    inline
    float64_t   cosh(const CRadian64& angle)    {   return ::cosh(angle.value());}

    //! cosh function wrapper, float32. @relates TRadian
    inline
    float32_t   cosh(const CRadian32& angle)    {   return ::coshf(angle.value());}

    //! tanh function wrapper. @relates TRadian
    inline
    float64_t   tanh(const CRadian64& angle)    {   return ::tanh(angle.value());}

    //! tanh function wrapper, float32. @relates TRadian
    inline
    float32_t   tanh(const CRadian32& angle)    {   return ::tanhf(angle.value());}

    //! templated arcus sin function, must specify return type as template parameter
    template<template<typename> class AngleClass, typename T>
    AngleClass<T> asin(const T& value);

    template<> inline
    CRadian64 asin<TRadian,float64_t>(const float64_t& value) { return CRadian64(::asin(value));}

    template<> inline
    CRadian32 asin<TRadian,float32_t>(const float32_t& value) { return CRadian32(::asinf(value));}

    template<> inline
    CDegree64 asin<TDegree,float64_t>(const float64_t& value) { return CDegree64(asin<TRadian>(value));}

    template<> inline
    CDegree32 asin<TDegree,float32_t>(const float32_t& value) { return CDegree32(asin<TRadian>(value));}

    //! templated arcus cos function, must specify return type as template parameter
    template<template<typename> class AngleClass, typename T>
    AngleClass<T> acos(const T& value);

    template<> inline
    CRadian64 acos<TRadian,float64_t>(const float64_t& value) { return CRadian64(::acos(value));}

    template<> inline
    CRadian32 acos<TRadian,float32_t>(const float32_t& value) { return CRadian32(::acosf(value));}

    template<> inline
    CDegree64 acos<TDegree,float64_t>(const float64_t& value) { return CDegree64(acos<TRadian>(value));}

    template<> inline
    CDegree32 acos<TDegree,float32_t>(const float32_t& value) { return CDegree32(acos<TRadian>(value));}

    //! templated arcus tangens function, must specify return type as template parameter
    template<template<typename> class AngleClass, typename T>
    AngleClass<T> atan(const T& value);

    template<> inline
    CRadian64 atan<TRadian,float64_t>(const float64_t& value) { return CRadian64(::atan(value));}

    template<> inline
    CRadian32 atan<TRadian,float32_t>(const float32_t& value) { return CRadian32(::atanf(value));}

    template<> inline
    CDegree64 atan<TDegree,float64_t>(const float64_t& value) { return CDegree64(atan<TRadian>(value));}

    template<> inline
    CDegree32 atan<TDegree,float32_t>(const float32_t& value) { return CDegree32(atan<TRadian>(value));}

    //! templated arcus tangens function with 2 parameters giving the correct quadrant
    template<template<typename> class AngleClass, typename T>
    AngleClass<T> atan2(const T& y, const T& x);

    template<> inline
    CRadian64 atan2<TRadian,float64_t>(const float64_t& y, const float64_t& x) { return CRadian64(::atan2(y, x));}

    template<> inline
    CRadian32 atan2<TRadian,float32_t>(const float32_t& y, const float32_t& x) { return CRadian32(::atan2f(y, x));}

    template<> inline
    CDegree64 atan2<TDegree,float64_t>(const float64_t& y, const float64_t& x) { return CDegree64(atan2<TRadian>(y, x));}

    template<> inline
    CDegree32 atan2<TDegree,float32_t>(const float32_t& y, const float32_t& x) { return CDegree32(atan2<TRadian>(y, x));}


}    // namespace vfc closed

#endif //VFC_TRIG_VW_PROJECT_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_trig.hpp  $
//  Revision 1.15 2009/02/09 06:15:42MEZ gaj2kor
//  -Removal of QAC++ warnings. (mantis2570)
//  Revision 1.14 2009/02/03 11:14:18IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Removal of QAC++ warnings.
//  (Mantis : 0002503)
//  Revision 1.13 2008/07/21 11:17:06IST Vinaykumar Setty (RBEI/EAC1) (vmr1kor)
//  Operator + ,-,*,/ are implemented interms of its assignment version for CRadian and CDegree(Mantis : 1703)
//  Revision 1.12 2007/08/24 19:57:48IST Muehlmann Karsten (AE-DA/ESV1) (muk2lr)
//  As there are no "Angels In My Room", we use angles (mantisNoNrForThis)
//  Revision 1.11 2007/07/18 16:32:37CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - doxygen documentation grouping (mantis1744)
//  Revision 1.10 2007/03/29 13:12:51CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - access libc functions with global scope using c headers (mantis1534)
//  Revision 1.9 2007/03/12 13:28:14CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - changed qualifier of arithmetic operators return values (mantis1488)
//  Revision 1.8 2006/11/16 14:41:08CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.7 2006/02/20 10:52:28CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -changed floating point comparison in op==() and op!=() (mantis1011)
//  Revision 1.6 2005/12/19 11:21:50CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -fixed last (and incomplete) check-in
//  Revision 1.5 2005/12/19 11:08:18CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added more operators
//  -removed *.inl file
//  Revision 1.4 2005/11/08 17:58:07CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added TIsPOD<> traits specialization
//  Revision 1.3 2005/11/04 16:58:31CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added missing unary operator-()
//  Revision 1.2 2005/11/03 09:28:29CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added arithmetic operators (+,-,*,/)
//  -removed ambiguous arcus functions
//=============================================================================
