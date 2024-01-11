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
*     $Source: vfc_mat4.inl $
*     $Revision: 1.6 $
*     $Author: gaj2kor $
*     $Date: 2009/02/09 06:29:36MEZ $
*     $Locker:  $
*     $Name: 0032 RC1  $
*     $State: in_work $
*/
/*******************************************************************/

#include "vfc/core/vfc_assert.hpp"
#include "vfc/core/vfc_type_traits.hpp"
#include "vfc/core/vfc_math.hpp"

#ifndef VFC_ASSERT_MAT4
#    define VFC_ASSERT_MAT4(col,row) VFC_ASSERT2( (0<=(col) && 0<=(row) && 3>=(col) && 3>= (row)),"mat4 idx out of bounds")
#endif

///////////////////////////////////////////////////////////////////////////////
// c'tors
///////////////////////////////////////////////////////////////////////////////

template <class T>    inline
vfc::TMatrix4<T>::TMatrix4
(    const T& m00, const T& m01, const T& m02, const T& m03,
    const T& m10, const T& m11, const T& m12, const T& m13,
    const T& m20, const T& m21, const T& m22, const T& m23,
    const T& m30, const T& m31, const T& m32, const T& m33
)
{
    set    (    m00, m01, m02, m03,
            m10, m11, m12, m13,
            m20, m21, m22, m23,
            m30, m31, m32, m33
        );
}

template <class T> template <class U>    inline
vfc::TMatrix4<T>::TMatrix4    (const TMatrix4<U>& rhs)
{
    set    (    static_cast<T>(rhs(0,0)), static_cast<T>(rhs(0,1)), static_cast<T>(rhs(0,2)), static_cast<T>(rhs(0,3)),
            static_cast<T>(rhs(1,0)), static_cast<T>(rhs(1,1)), static_cast<T>(rhs(1,2)), static_cast<T>(rhs(1,3)),
            static_cast<T>(rhs(2,0)), static_cast<T>(rhs(2,1)), static_cast<T>(rhs(2,2)), static_cast<T>(rhs(2,3)),
            static_cast<T>(rhs(3,0)), static_cast<T>(rhs(3,1)), static_cast<T>(rhs(3,2)), static_cast<T>(rhs(3,3))
        );
}

///////////////////////////////////////////////////////////////////////////////
// set
///////////////////////////////////////////////////////////////////////////////

template <class T>    inline
void    vfc::TMatrix4<T>::set (    const T& val)
{
    set    (    val, val, val, val,
            val, val, val, val,
            val, val, val, val,
            val, val, val, val
        );
}

template <class T>    inline
void    vfc::TMatrix4<T>::set
(    const T& m00, const T& m01, const T& m02, const T& m03,
    const T& m10, const T& m11, const T& m12, const T& m13,
    const T& m20, const T& m21, const T& m22, const T& m23,
    const T& m30, const T& m31, const T& m32, const T& m33
)
{
    m_data[0][0] = m00; m_data[0][1] = m01; m_data[0][2] = m02; m_data[0][3] = m03;
    m_data[1][0] = m10; m_data[1][1] = m11; m_data[1][2] = m12; m_data[1][3] = m13;
    m_data[2][0] = m20; m_data[2][1] = m21; m_data[2][2] = m22; m_data[2][3] = m23;
    m_data[3][0] = m30; m_data[3][1] = m31; m_data[3][2] = m32; m_data[3][3] = m33;
}

///////////////////////////////////////////////////////////////////////////////
// operator()
///////////////////////////////////////////////////////////////////////////////

template <class T>    inline
const    T&    vfc::TMatrix4<T>::operator()(int32_t row, int32_t col)    const
{
    VFC_ASSERT_MAT4(col,row);
    return m_data[row][col];
}

template <class T>    inline
T&    vfc::TMatrix4<T>::operator()(int32_t row, int32_t col)
{
    VFC_ASSERT_MAT4(col,row);
    return m_data[row][col];
}

///////////////////////////////////////////////////////////////////////////////
// scalar ops
///////////////////////////////////////////////////////////////////////////////

template <class T>    inline
const vfc::TMatrix4<T>&    vfc::TMatrix4<T>::operator*= (const T& rhs)
{
    m_data[0][0] *= rhs; m_data[0][1] *= rhs; m_data[0][2] *= rhs; m_data[0][3] *= rhs;
    m_data[1][0] *= rhs; m_data[1][1] *= rhs; m_data[1][2] *= rhs; m_data[1][3] *= rhs;
    m_data[2][0] *= rhs; m_data[2][1] *= rhs; m_data[2][2] *= rhs; m_data[2][3] *= rhs;
    m_data[3][0] *= rhs; m_data[3][1] *= rhs; m_data[3][2] *= rhs; m_data[3][3] *= rhs;

    return *this;
}

template <class T>    inline
const vfc::TMatrix4<T>&    vfc::TMatrix4<T>::divide_assign(const T& rhs, true_t)
{
    const T    fac = static_cast<T>(1)/rhs;

    return this->operator*=(fac);
}

template <class T>    inline
const vfc::TMatrix4<T>&    vfc::TMatrix4<T>::divide_assign(const T& rhs, false_t)
{
    m_data[0][0] /= rhs; m_data[0][1] /= rhs; m_data[0][2] /= rhs; m_data[0][3] /= rhs;
    m_data[1][0] /= rhs; m_data[1][1] /= rhs; m_data[1][2] /= rhs; m_data[1][3] /= rhs;
    m_data[2][0] /= rhs; m_data[2][1] /= rhs; m_data[2][2] /= rhs; m_data[2][3] /= rhs;
    m_data[3][0] /= rhs; m_data[3][1] /= rhs; m_data[3][2] /= rhs; m_data[3][3] /= rhs;

    return *this;
}

template <class T>    inline
const vfc::TMatrix4<T>&    vfc::TMatrix4<T>::operator/= (const T& rhs)
{
    VFC_REQUIRE2(!isZero(rhs), "Division by zero");

    return divide_assign( rhs, typename TInt2Boolean<TIsFloating<T>::value>::type());
}

///////////////////////////////////////////////////////////////////////////////
// matrix ops
///////////////////////////////////////////////////////////////////////////////

template <class T>    inline
const vfc::TMatrix4<T>&    vfc::TMatrix4<T>::operator+= (const TMatrix4<T>& rhs)
{
    m_data[0][0] += rhs.m_data[0][0]; m_data[0][1] += rhs.m_data[0][1]; m_data[0][2] += rhs.m_data[0][2];  m_data[0][3] += rhs.m_data[0][3];
    m_data[1][0] += rhs.m_data[1][0]; m_data[1][1] += rhs.m_data[1][1]; m_data[1][2] += rhs.m_data[1][2];  m_data[1][3] += rhs.m_data[1][3];
    m_data[2][0] += rhs.m_data[2][0]; m_data[2][1] += rhs.m_data[2][1]; m_data[2][2] += rhs.m_data[2][2];  m_data[2][3] += rhs.m_data[2][3];
    m_data[3][0] += rhs.m_data[3][0]; m_data[3][1] += rhs.m_data[3][1]; m_data[3][2] += rhs.m_data[3][2];  m_data[3][3] += rhs.m_data[3][3];

    return *this;
}
template <class T>    inline
const vfc::TMatrix4<T>&    vfc::TMatrix4<T>::operator-= (const TMatrix4<T>& rhs)
{
    m_data[0][0] -= rhs.m_data[0][0]; m_data[0][1] -= rhs.m_data[0][1]; m_data[0][2] -= rhs.m_data[0][2];  m_data[0][3] -= rhs.m_data[0][3];
    m_data[1][0] -= rhs.m_data[1][0]; m_data[1][1] -= rhs.m_data[1][1]; m_data[1][2] -= rhs.m_data[1][2];  m_data[1][3] -= rhs.m_data[1][3];
    m_data[2][0] -= rhs.m_data[2][0]; m_data[2][1] -= rhs.m_data[2][1]; m_data[2][2] -= rhs.m_data[2][2];  m_data[2][3] -= rhs.m_data[2][3];
    m_data[3][0] -= rhs.m_data[3][0]; m_data[3][1] -= rhs.m_data[3][1]; m_data[3][2] -= rhs.m_data[3][2];  m_data[3][3] -= rhs.m_data[3][3];

    return *this;
}


/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_mat4.inl  $
Revision 1.6 2009/02/09 06:29:36MEZ gaj2kor 
-Removal of QAC++ warnings. (mantis2568)
Revision 1.5 2007/03/16 16:05:07IST Muehlmann Karsten (AE-DA/ESV1) (muk2lr)
replaced exception by precondition (mantis1420)
Revision 1.4 2006/11/16 14:41:17CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
- replaced tabs with 4 spaces (mantis1294)
Revision 1.3 2006/04/10 15:35:07CEST Dilipkrishna Natesan (RBIN/EAE3 AE-DA/ESA3) * (DIN1LR)
-included vfc_math.hpp and vfc_type_traits.hpp (mantis1050)
Revision 1.2 2005/11/07 18:20:59CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-changed project name to vfc/geo
********************************************************************/
