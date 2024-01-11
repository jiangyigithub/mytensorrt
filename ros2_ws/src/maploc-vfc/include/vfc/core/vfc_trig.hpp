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

#ifndef VFC_TRIG_HPP_INCLUDED
#define VFC_TRIG_HPP_INCLUDED

#include <math.h>                        // used for trig funcs
#include "vfc/core/vfc_types.hpp"        // used for fundamental types
#include "vfc/core/vfc_type_traits.hpp"    // used for VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL
#include "vfc/core/vfc_math.hpp"        // used for G_DEG2RAD and G_RAD2DEG constants, isEqual()
#include <vfc/extension/vfc_trig_vw_project.hpp>        // used for templated Radian and Degree types

namespace vfc
{    // namespace vfc opened

    ///////////////////////////////////////////////////////////////////////////
    // based on ideas for safe radian/degree utilization
    // from CR/AEM4-Jaeger
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    // some forward declarations
    ///////////////////////////////////////////////////////////////////////////

    class CRadian;
    class CDegree;

    //=========================================================================
    // CRadian
    //-------------------------------------------------------------------------
    //! Type-Safe angle representation in radians.
    //! @sa CDegree
    //! @ingroup vfc_group_core_types
    //=========================================================================

    class CRadian
    {
    public:
        //! c'tor, takes angle in radians
        explicit
        CRadian(float64_t f_angleInRadians = 0.) : m_value(f_angleInRadians) {}

        //PRQA S 2180 ++
        //! implicit conversion from degree to radian angle
        CRadian(const CDegree& angle);
        //PRQA S 2180 --

        //! returns angle value in radians
        float64_t value(void) const { return m_value;}
        //! add-assign operator
        const CRadian&  operator+= (const CRadian& rhs) { m_value+=rhs.m_value; return *this;}
        //! subtract-assign operator
        const CRadian&  operator-= (const CRadian& rhs) { m_value-=rhs.m_value; return *this;}
        //! multiply-assign operator
        const CRadian&  operator*= (float64_t f_rhs)      { m_value*=f_rhs; return *this;}
        //! divide-assign operator
        const CRadian&  operator/= (float64_t f_rhs)      { m_value/=f_rhs; return *this;}

    private:
        float64_t    m_value;
    };

    // traits specialization
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsPOD,CRadian,true);

    //! returns angle differnce in rad. @relates CRadian
    inline
    const   CRadian operator-(const CRadian& lhs, const CRadian& rhs)   {    return CRadian(lhs) -= rhs ;}

    //! returns angle sum in rad. @relates CRadian
    inline
    const   CRadian operator+(const CRadian& lhs, const CRadian& rhs)   {    return CRadian(lhs) += rhs;}

    //! returns angle times given factor. @relates CRadian
    inline
    const   CRadian    operator*(const CRadian& lhs, float64_t f_rhs_f64)     {    return CRadian(lhs) *= f_rhs_f64;}

    //! returns angle times given factor. @relates CRadian
    inline
    const   CRadian    operator*(float64_t f_lhs, const CRadian& rhs)     {    return CRadian(rhs) *= f_lhs ;}

    //! returns angle divided by denom. @relates CRadian
    inline
    const   CRadian    operator/(const CRadian& lhs, float64_t f_rhs_f64)     {    return CRadian(lhs) /= f_rhs_f64;}

    //! unary neg operator. @relates CRadian
    inline
    const   CRadian    operator-(const CRadian& angle_rad)              {    return CRadian(-angle_rad.value());}

    // comparison operators

    //! comparison operator for equality. @relates CRadian
    inline
    bool operator== (const CRadian& lhs, const CRadian& rhs)    {    return isEqual(lhs.value(), rhs.value());}

    //! comparison operator for inequality. @relates CRadian
    inline
    bool operator!= (const CRadian& lhs, const CRadian& rhs)    {    return notEqual(lhs.value(), rhs.value());}

    //! comparison operator for less. @relates CRadian
    inline
    bool operator<  (const CRadian& lhs, const CRadian& rhs)    {    return lhs.value()<rhs.value();}

    //! comparison operator for greater. @relates CRadian
    inline
    bool operator>  (const CRadian& lhs, const CRadian& rhs)    {    return lhs.value()>rhs.value();}

    //! comparison operator for less-equal. @relates CRadian
    inline
    bool operator<= (const CRadian& lhs, const CRadian& rhs)    {    return lhs.value()<=rhs.value();}

    //! comparison operator for greater-equal. @relates CRadian
    inline
    bool operator>= (const CRadian& lhs, const CRadian& rhs)    {    return lhs.value()>=rhs.value();}

    //=========================================================================
    // CDegree
    //-------------------------------------------------------------------------
    //! Type-Safe angle representation in degrees.
    //! @sa CRadian
    //! @ingroup vfc_group_core_types
    //=========================================================================

    class CDegree
    {
    public:
        //! c'tor, takes angle in degrees
        explicit
        CDegree(float64_t f_angleInDegrees = 0.) : m_value(f_angleInDegrees) {}

        //PRQA S 2180 ++
        //! implicit conversion from radian to degree angle
        CDegree(const CRadian& angle);
        //PRQA S 2180 --

        //! returns angle value in degrees
        float64_t value(void) const { return m_value;}
        //! add-assign operator
        const CDegree&  operator+= (const CDegree& rhs) { m_value+=rhs.m_value; return *this;}
        //! subtract-assign operator
        const CDegree&  operator-= (const CDegree& rhs) { m_value-=rhs.m_value; return *this;}
        //! multiply-assign operator
        const CDegree&  operator*= (float64_t f_rhs) { m_value*=f_rhs; return *this;}
        //! divide-assign operator
        const CDegree&  operator/= (float64_t f_rhs) { m_value/=f_rhs; return *this;}

    private:
        float64_t    m_value;
    };

    // conversion c'tor implementations
    inline
    CRadian::CRadian(const CDegree& angle) : m_value(angle.value() * G_DEG2RAD) {}
    inline
    CDegree::CDegree(const CRadian& angle) : m_value(angle.value() * G_RAD2DEG) {}

    // traits specialization
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TIsPOD,CDegree,true);

    //! returns angle differnce in degrees. @relates CDegree
    inline
    const CDegree operator-(const CDegree& lhs, const CDegree& rhs)     {    return CDegree(lhs) -= rhs;}

    //! returns angle sum in degrees. @relates CDegree
    inline
    const CDegree operator+(const CDegree& lhs, const CDegree& rhs)     {    return CDegree(lhs) += rhs;}

    //! returns angle times given factor. @relates CDegree
    inline
    const CDegree    operator*(const CDegree& lhs, float64_t f_rhs_f64)       {    return CDegree(lhs) *= f_rhs_f64;}

    //! returns angle times given factor. @relates CDegree
    inline
    const CDegree    operator*(float64_t f_lhs, const CDegree& rhs)       {    return CDegree(rhs) *= f_lhs;}

    //! returns angle divided by denom. @relates CDegree
    inline
    const CDegree    operator/(const CDegree& lhs, float64_t f_rhs_f64)       {    return CDegree(lhs) /= f_rhs_f64;}

    //! unary neg operator. @relates CDegree
    inline
    const CDegree    operator-(const CDegree& angle_rad)                {    return CDegree(-angle_rad.value());}

        //! comparison operator for equality
    inline
    bool operator== (const CDegree& lhs, const CDegree& rhs)    {    return isEqual(lhs.value(), rhs.value());}

    //! comparison operator for inequality. @relates CDegree
    inline
    bool operator!= (const CDegree& lhs, const CDegree& rhs)    {    return notEqual(lhs.value(), rhs.value());}

    //! comparison operator for less. @relates CDegree
    inline
    bool operator<  (const CDegree& lhs, const CDegree& rhs)    {    return lhs.value()<rhs.value();}

    //! comparison operator for greater. @relates CDegree
    inline
    bool operator>  (const CDegree& lhs, const CDegree& rhs)    {    return lhs.value()>rhs.value();}

    //! comparison operator for less-equal. @relates CDegree
    inline
    bool operator<= (const CDegree& lhs, const CDegree& rhs)    {    return lhs.value()<=rhs.value();}

    //! comparison operator for greater-equal. @relates CDegree
    inline
    bool operator>= (const CDegree& lhs, const CDegree& rhs)    {    return lhs.value()>=rhs.value();}

    ///////////////////////////////////////////////////////////////////////////
    // trig function wrappers
    ///////////////////////////////////////////////////////////////////////////

    //! sin function wrapper. @relates CRadian
    inline
    float64_t    sin    (const CRadian& angle)    {   return ::sin(angle.value());}

    //! cos function wrapper. @relates CRadian
    inline
    float64_t   cos    (const CRadian& angle)    {   return ::cos(angle.value());}

    //! tan function wrapper. @relates CRadian
    inline
    float64_t   tan    (const CRadian& angle)    {   return ::tan(angle.value());}

    //! sinh function wrapper. @relates CRadian
    inline
    float64_t   sinh(const CRadian& angle)    {   return ::sinh(angle.value());}

    //! cosh function wrapper. @relates CRadian
    inline
    float64_t   cosh(const CRadian& angle)    {   return ::cosh(angle.value());}

    //! tanh function wrapper. @relates CRadian
    inline
    float64_t   tanh(const CRadian& angle)    {   return ::tanh(angle.value());}

}    // namespace vfc closed

#endif //VFC_TRIG_HPP_INCLUDED

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
