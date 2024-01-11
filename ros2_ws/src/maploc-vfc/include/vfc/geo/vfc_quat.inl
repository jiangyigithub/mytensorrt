//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2006 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorised copy, use or 
//  distribution is an offensive act against international law and may be 
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/geo
//          Synopsis: Quaternion implementation
//  Target system(s): 
//       Compiler(s): VC_71
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
///     $Source: vfc_quat.inl $
///     $Revision: 1.7 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/08/19 17:21:04MESZ $
///     $Locker:  $
///     $Name:  $
///     $State: in_work $
///
/// @par Review Information:
/// - Reviewed version: 
/// - Type (use "X" to mark):
///        - [ ] Formal Review
///        - [ ] Walkthrough
///        - [ ] Inspection
/// - State including date (DD.MM.YYYY)
///        - [--.--.----] Preparation
///        - [--.--.----] Review audit 
///        - [--.--.----] Integration of findings
///        - [--.--.----] Test
///        - [--.--.----] Verification of integration of findings
///        - [--.--.----] Review release
///    - Responsible:
///    - Review-Document:        
//=============================================================================

#include "vfc/core/vfc_assert.hpp"
#include "vfc/core/vfc_type_traits.hpp"
#include "vfc/core/vfc_metaprog.hpp"
#include "vfc/core/vfc_math.hpp"

template <class T>    template <class U> inline
vfc::TQuaternion<T>::TQuaternion    (const TQuaternion<U>& f_rhs)                                        
{    
    set(    static_cast<T>(f_rhs.x()),
            static_cast<T>(f_rhs.y()),
            static_cast<T>(f_rhs.z()),
            static_cast<T>(f_rhs.w())
        );
}

template <class T>    inline
const vfc::TQuaternion<T>&    vfc::TQuaternion<T>::operator*= (const T& f_rhs)
{
    m_data[0] *= f_rhs;
    m_data[1] *= f_rhs;
    m_data[2] *= f_rhs;
    m_data[3] *= f_rhs;

    return *this;
}

template <class T>    inline
const vfc::TQuaternion<T>&    vfc::TQuaternion<T>::divide_assign(const T& f_rhs, true_t)
{
    const T fac = static_cast<T>(1)/f_rhs;

    return this->operator*=(fac);
}

template <class T>    inline
const vfc::TQuaternion<T>&    vfc::TQuaternion<T>::divide_assign (const T& f_rhs, false_t)
{
    m_data[0] /= f_rhs;
    m_data[1] /= f_rhs;
    m_data[2] /= f_rhs;
    m_data[3] /= f_rhs;

    return *this;
}

template <class T>    inline
const vfc::TQuaternion<T>&    vfc::TQuaternion<T>::operator/= (const T& f_rhs)
{
    VFC_REQUIRE2(!vfc::isZero(f_rhs), "Division by zero");

    return divide_assign( f_rhs, typename vfc::TInt2Boolean<vfc::TIsFloating<T>::value>::type());
}

template <class T>    inline
const vfc::TQuaternion<T>&    vfc::TQuaternion<T>::operator+= (const TQuaternion<T>& f_rhs)
{
    m_data[0] += f_rhs.m_data[0];
    m_data[1] += f_rhs.m_data[1];
    m_data[2] += f_rhs.m_data[2];
    m_data[3] += f_rhs.m_data[3];

    return *this;
}

template <class T>    inline
const vfc::TQuaternion<T>&    vfc::TQuaternion<T>::operator-= (const TQuaternion<T>& f_rhs)
{
    m_data[0] -= f_rhs.m_data[0];
    m_data[1] -= f_rhs.m_data[1];
    m_data[2] -= f_rhs.m_data[2];
    m_data[3] -= f_rhs.m_data[3];

    return *this;
}

template <class T>    inline
void    vfc::TQuaternion<T>::set    (    const T& f_qx, const T& f_qy, const T& f_qz, const T& f_qw)    
{    
    m_data[0] = f_qx; 
    m_data[1] = f_qy; 
    m_data[2] = f_qz;
    m_data[3] = f_qw;
}

template <class T>    inline
void    vfc::TQuaternion<T>::set    (const T& f_qxyzw)                                    
{    
    m_data[0] = m_data[1] = m_data[2] = m_data[3] = f_qxyzw;
}

template <class T>    inline
const    T&    vfc::TQuaternion<T>::operator[](int32_t f_idx)    const    
{    
    VFC_ASSERT(0<=f_idx && 3>=f_idx); 
    return m_data[f_idx];
}

template <class T>    inline
T&    vfc::TQuaternion<T>::operator[](int32_t f_idx)            
{    
    VFC_ASSERT(0<=f_idx && 3>=f_idx); 
    return m_data[f_idx];
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_quat.inl  $
//  Revision 1.7 2014/08/19 17:21:04MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_quat uses isZero, but does not #include vfc_math (mantis0004657)
//  Revision 1.6 2011/01/21 12:53:31MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - sometimes .hpp or namespace qualifiers are missing (mantis3599)
//  Revision 1.5 2007/03/16 11:35:07MEZ Muehlmann Karsten (CC/ESV2) (muk2lr) 
//  replaced exception by precondition (mantis1420)
//  Revision 1.4 2006/11/16 14:41:21CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.3 2006/10/13 10:00:25CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed header/footer template    
//  - added documentation
//  Revision 1.2 2006/07/27 10:08:35CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -added documentation
//  -following coding conventions
//=============================================================================
