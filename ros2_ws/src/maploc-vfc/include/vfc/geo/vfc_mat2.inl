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
*     $Source: vfc_mat2.inl $
*     $Revision: 1.6 $
*     $Author: gaj2kor $
*     $Date: 2009/02/02 05:47:13MEZ $
*     $Locker:  $
*     $Name: 0032 RC1  $
*     $State: in_work $
*/
/*******************************************************************/

#include "vfc/core/vfc_assert.hpp"
#include "vfc/core/vfc_math.hpp"

#ifndef VFC_ASSERT_MAT2
#    define    VFC_ASSERT_MAT2(col,row) VFC_ASSERT2( (0<=(col) && 0<=(row) && 1>=(col) && 1>= (row)),"mat2 idx out of bounds")
#endif

///////////////////////////////////////////////////////////////////////////////
// c'tors
///////////////////////////////////////////////////////////////////////////////

template <class T> inline
vfc::TMatrix2<T>::TMatrix2    (    const T& m00, const T& m01,
                const T& m10, const T& m11)
{
    set    (    m00, m01,
            m10, m11);
}

template <class T> template <class U>  inline
vfc::TMatrix2<T>::TMatrix2    (const TMatrix2<U>& rhs)
{
    set    (    static_cast<T>(rhs(0,0)), static_cast<T>(rhs(0,1)),
            static_cast<T>(rhs(1,0)), static_cast<T>(rhs(1,1)));
}

///////////////////////////////////////////////////////////////////////////////
// set
///////////////////////////////////////////////////////////////////////////////

template <class T> inline
void    vfc::TMatrix2<T>::set (const T& val)
{
    set    (    val, val,
            val, val);
}

template <class T> inline
void    vfc::TMatrix2<T>::set
(    const T& m00, const T& m01,
    const T& m10, const T& m11
)
{
    m_data[0][0] = m00; m_data[0][1] = m01;
    m_data[1][0] = m10; m_data[1][1] = m11;
}

///////////////////////////////////////////////////////////////////////////////
// operator()
///////////////////////////////////////////////////////////////////////////////

template <class T> inline
const    T&    vfc::TMatrix2<T>::operator()(int32_t row, int32_t col)    const
{
    VFC_ASSERT_MAT2(col,row);
    return m_data[row][col];
}

template <class T> inline
T&    vfc::TMatrix2<T>::operator()(int32_t row, int32_t col)
{
    VFC_ASSERT_MAT2(col,row);
    return m_data[row][col];
}

///////////////////////////////////////////////////////////////////////////////
// scalar ops
///////////////////////////////////////////////////////////////////////////////

template <class T> inline
const vfc::TMatrix2<T>&    vfc::TMatrix2<T>::operator*= (const T& rhs)
{
    m_data[0][0] *= rhs; m_data[0][1] *= rhs;
    m_data[1][0] *= rhs; m_data[1][1] *= rhs;

    return *this;
}

template <class T> inline
const vfc::TMatrix2<T>&    vfc::TMatrix2<T>::divide_assign(const T& rhs, true_t)
{
    const T    fac = static_cast<T>(1)/rhs;

    return this->operator*=(fac);
}

template <class T> inline
const vfc::TMatrix2<T>&    vfc::TMatrix2<T>::divide_assign(const T& rhs, false_t)
{
    m_data[0][0] /= rhs; m_data[0][1] /= rhs;
    m_data[1][0] /= rhs; m_data[1][1] /= rhs;

    return *this;
}

template <class T> inline
const vfc::TMatrix2<T>&    vfc::TMatrix2<T>::operator/= (const T& rhs)
{
    VFC_REQUIRE2(!isZero(rhs), "Division by zero");

    return divide_assign( rhs, typename TInt2Boolean<TIsFloating<T>::value>::type());
}

///////////////////////////////////////////////////////////////////////////////
// matrix ops
///////////////////////////////////////////////////////////////////////////////

template <class T> inline
const vfc::TMatrix2<T>&    vfc::TMatrix2<T>::operator+= (const TMatrix2<T>& rhs)
{
    m_data[0][0] += rhs.m_data[0][0]; m_data[0][1] += rhs.m_data[0][1];
    m_data[1][0] += rhs.m_data[1][0]; m_data[1][1] += rhs.m_data[1][1];

    return *this;
}

template <class T> inline
const vfc::TMatrix2<T>&    vfc::TMatrix2<T>::operator-= (const TMatrix2<T>& rhs)
{
    m_data[0][0] -= rhs.m_data[0][0]; m_data[0][1] -= rhs.m_data[0][1];
    m_data[1][0] -= rhs.m_data[1][0]; m_data[1][1] -= rhs.m_data[1][1];

    return *this;
}

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_mat2.inl  $
Revision 1.6 2009/02/02 05:47:13MEZ gaj2kor 
-Removal of QAC++ warnings.
(Mantis : 0002494)
Revision 1.5 2007/03/16 16:05:08IST Muehlmann Karsten (CC-DA/ESV2) (muk2lr) 
replaced exception by precondition (mantis1420)
Revision 1.4 2006/11/16 14:41:06CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
- replaced tabs with 4 spaces (mantis1294)
Revision 1.3 2006/04/10 15:34:13CEST Dilipkrishna Natesan (RBIN/EAE3 AE-DA/ESA3) * (DIN1LR)
-included vfc_math.hpp (mantis1049)
Revision 1.2 2005/11/07 18:20:56CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-changed project name to vfc/geo
********************************************************************/
