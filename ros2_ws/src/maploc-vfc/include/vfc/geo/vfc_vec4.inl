/********************************************************************
* C O P Y R I G H T
*--------------------------------------------------------------------
* Copyright (c) 2005 by Robert Bosch GmbH. All rights reserved.
*
* This file is property of Robert Bosch GmbH. Any unauthorised copy,
* use or distribution is an offensive act against international law
* and may me prosecuted under federal law. Its content is company
* confidential.
*--------------------------------------------------------------------
* D E S C R I P T I O N
*--------------------------------------------------------------------
*   Projectname: vfc/geo
*      Synopsis: 
* Target system: 
*      Compiler: VS7.1
*--------------------------------------------------------------------
* N O T E S
*--------------------------------------------------------------------
* Notes: 
*--------------------------------------------------------------------
* I N I T I A L   A U T H O R   I D E N T I T Y
*--------------------------------------------------------------------
*       Name: 
* Department: 
*--------------------------------------------------------------------
* R E V I S I O N   I N F O R M A T I O N
*------------------------------------------------------------------*/
/*!   \file
*     \par Revision History
*     $Source: vfc_vec4.inl $
*     $Revision: 1.6 $
*     $Author: Muehlmann Karsten (CC/EYN2) (muk2lr) $
*     $Date: 2007/03/16 11:35:08MEZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/


#include "vfc/core/vfc_assert.hpp"
#include "vfc/core/vfc_math.hpp"

template <class T>    template <class U> inline
vfc::TVector4<T>::TVector4    (const TVector4<U>& rhs)                                        
{    
    set(    static_cast<T>(rhs.x()),
            static_cast<T>(rhs.y()),
            static_cast<T>(rhs.z()),
            static_cast<T>(rhs.w())
        );
}

template <class T>    inline
const vfc::TVector4<T>&    vfc::TVector4<T>::operator*= (const T& rhs)
{
    m_data[0] *= rhs;
    m_data[1] *= rhs;
    m_data[2] *= rhs;
    m_data[3] *= rhs;

    return *this;
}

template <class T>    inline
const vfc::TVector4<T>&    vfc::TVector4<T>::divide_assign(const T& rhs, true_t)
{
    const T fac = static_cast<T>(1)/rhs;

    return this->operator*=(fac);
}

template <class T>    inline
const vfc::TVector4<T>&    vfc::TVector4<T>::divide_assign (const T& rhs, false_t)
{
    m_data[0] /= rhs;
    m_data[1] /= rhs;
    m_data[2] /= rhs;
    m_data[3] /= rhs;

    return *this;
}

template <class T>    inline
const vfc::TVector4<T>&    vfc::TVector4<T>::operator/= (const T& rhs)
{
    VFC_REQUIRE2(!isZero(rhs), "Division by zero");

    return divide_assign( rhs, typename TInt2Boolean<TIsFloating<T>::value>::type());
}

template <class T>    inline
const vfc::TVector4<T>&    vfc::TVector4<T>::operator+= (const TVector4<T>& rhs)
{
    m_data[0] += rhs.m_data[0];
    m_data[1] += rhs.m_data[1];
    m_data[2] += rhs.m_data[2];
    m_data[3] += rhs.m_data[3];

    return *this;
}

template <class T>    inline
const vfc::TVector4<T>&    vfc::TVector4<T>::operator-= (const TVector4<T>& rhs)
{
    m_data[0] -= rhs.m_data[0];
    m_data[1] -= rhs.m_data[1];
    m_data[2] -= rhs.m_data[2];
    m_data[3] -= rhs.m_data[3];

    return *this;
}

template <class T>    inline
void    vfc::TVector4<T>::set    (const T& val1, const T& val2, const T& val3, const T& val4)    
{    
    m_data[0] = val1; 
    m_data[1] = val2; 
    m_data[2] = val3;
    m_data[3] = val4;
}

template <class T>    inline
void    vfc::TVector4<T>::set    (const T& val)                                    
{    
    m_data[0] = m_data[1] = m_data[2] = m_data[3] = val;
}

template <class T>    inline
const    T&    vfc::TVector4<T>::operator[](int32_t idx)    const    
{    
    VFC_ASSERT(0<=idx && 3>=idx); 
    return m_data[idx];
}

template <class T>    inline
T&    vfc::TVector4<T>::operator[](int32_t idx)            
{    
    VFC_ASSERT(0<=idx && 3>=idx); 
    return m_data[idx];
}

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_vec4.inl  $
Revision 1.6 2007/03/16 11:35:08MEZ Muehlmann Karsten (CC/EYN2) (muk2lr) 
replaced exception by precondition (mantis1420)
Revision 1.5 2006/11/16 14:41:12CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.4 2006/01/11 11:37:53CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-added missing include file
Revision 1.3 2005/11/30 11:13:53CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-changed implementation of generalized copy c'tor
Revision 1.2 2005/11/07 18:21:03CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-changed project name to vfc/geo
********************************************************************/
