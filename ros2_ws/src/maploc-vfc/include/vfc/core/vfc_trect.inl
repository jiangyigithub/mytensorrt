//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2011 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy, use or 
//  distribution is an offensive act against international law and may be 
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  P R O J E C T   I N F O R M A T I O N
//-----------------------------------------------------------------------------
//       Projectname: MPC2
//  Target system(s): zync
//       Compiler(s): VC, GCC, RVCT, GHS
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: Gaurav Jain (RBEI/ESD1)
//  Department: RBEI/ESD1
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @brief add short description here
/// @par Revision History:
///     $Source: vfc_trect.inl $
///     $Revision: 1.3 $
///     $Author: gaj2kor
///     $Date: 2014/07/29 11:20:26MESZ $
///     $State: in_work $
//=============================================================================

#include <algorithm>             // used for ::std::min(), ::std::max()
#include "vfc/core/vfc_math.hpp" // used for vfc::isZero    

template<class ValueType>
inline
vfc::TPoint<ValueType>::TPoint (void)
    :    m_x(static_cast<ValueType>(0)), m_y(static_cast<ValueType>(0))
{
}

template<class ValueType>
inline
vfc::TPoint<ValueType>::TPoint (typename vfc::TPoint<ValueType>::value_type f_x, typename vfc::TPoint<ValueType>::value_type f_y)
    :    m_x(f_x), m_y(f_y)
{
}

template<class ValueType>
inline
const vfc::TPoint<ValueType>& vfc::TPoint<ValueType>::operator += (const TPoint<ValueType>& f_rhs)
{
    m_x += f_rhs.m_x;
    m_y += f_rhs.m_y;
    return *this;
}

template<class ValueType>
inline
const vfc::TPoint<ValueType>& vfc::TPoint<ValueType>::operator -= (const TPoint<ValueType>& f_rhs)
{
    m_x -= f_rhs.m_x;
    m_y -= f_rhs.m_y;
    return *this;
}

template<class ValueType>
inline
const vfc::TPoint<ValueType>    vfc::operator+ (const TPoint<ValueType>& f_op1, const TPoint<ValueType>& f_op2)
{
    return TPoint<ValueType>(f_op1).operator+=(f_op2);
}

template<class ValueType>
inline
const vfc::TPoint<ValueType>    vfc::operator- (const TPoint<ValueType>& f_op1, const TPoint<ValueType>& f_op2)
{
    return TPoint<ValueType>(f_op1).operator-=(f_op2);
}

template<class ValueType>
inline
vfc::TSize<ValueType>::TSize    (void)
    :    m_cx(static_cast<typename vfc::TPoint<ValueType>::value_type>(0)), m_cy(static_cast<typename vfc::TPoint<ValueType>::value_type>(0))
{
}


template<class ValueType>
inline
vfc::TSize<ValueType>::TSize    (typename vfc::TSize<ValueType>::value_type f_cx, typename vfc::TSize<ValueType>::value_type f_cy)
    :    m_cx(f_cx), m_cy(f_cy)
{
}

template<class ValueType>
inline
const vfc::TSize<ValueType>& vfc::TSize<ValueType>::operator += (const TSize<ValueType>& f_rhs)
{
    m_cx += f_rhs.m_cx;
    m_cy += f_rhs.m_cy;
    return *this;
}

template<class ValueType>
inline
const vfc::TSize<ValueType>& vfc::TSize<ValueType>::operator -= (const TSize<ValueType>& f_rhs)
{
    m_cx -= f_rhs.m_cx;
    m_cy -= f_rhs.m_cy;
    return *this;
}

template<class ValueType>
inline
const vfc::TSize<ValueType>    vfc::operator+ (const TSize<ValueType>& f_op1, const TSize<ValueType>& f_op2)
{
    return TSize<ValueType>(f_op1).operator+=(f_op2);
}

template<class ValueType>
inline
const vfc::TSize<ValueType>    vfc::operator- (const TSize<ValueType>& f_op1, const TSize<ValueType>& f_op2)
{
    return TSize<ValueType>(f_op1).operator-=(f_op2);
}

template<class ValueType>
inline
vfc::TRect<ValueType>::TRect    (void)
    : m_left    (static_cast<typename vfc::TPoint<ValueType>::value_type>(0)),
    m_top     (static_cast<typename vfc::TPoint<ValueType>::value_type>(0)),
    m_width   (static_cast<typename vfc::TPoint<ValueType>::value_type>(0)),
    m_height  (static_cast<typename vfc::TPoint<ValueType>::value_type>(0))
{
}

template<class ValueType>
inline
vfc::TRect<ValueType>::TRect    (ValueType f_left, ValueType f_top, ValueType f_width, ValueType f_height)
    :   m_left  (static_cast<typename vfc::TRect<ValueType>::value_type>(f_left)),
    m_top   (static_cast<typename vfc::TRect<ValueType>::value_type>(f_top)),
    m_width (static_cast<typename vfc::TRect<ValueType>::value_type>(f_width)),
    m_height(static_cast<typename vfc::TRect<ValueType>::value_type>(f_height))
{
}

template<class ValueType>
inline
vfc::TRect<ValueType>::TRect    (const TPoint<ValueType>& f_topLeft, const TSize<ValueType>& f_size)
    :    m_left  (f_topLeft.x()),
    m_top   (f_topLeft.y()),
    m_width (f_size.cx()),
    m_height(f_size.cy())
{
}

template<class ValueType>
inline
vfc::TRect<ValueType>::TRect    (const TPoint<ValueType>& f_topLeft, const TPoint<ValueType>& f_bottomRight)
    :    m_left  (f_topLeft.x()),
    m_top   (f_topLeft.y()),
    m_width (f_bottomRight.x()-f_topLeft.x()),
    m_height(f_bottomRight.y()-f_topLeft.y())
{
}

template<class ValueType>
inline
vfc::TPoint<ValueType>    vfc::TRect<ValueType>::topLeft        (void)    const
{
    return TPoint<ValueType>(left(),top());
}

template<class ValueType>
inline
vfc::TPoint<ValueType>    vfc::TRect<ValueType>::bottomRight    (void)    const
{
    return TPoint<ValueType>(right(), bottom());
}

template<class ValueType>
inline
vfc::TPoint<ValueType>    vfc::TRect<ValueType>::center        (void)    const
{
    return centerIntern(typename vfc::TIf<(vfc::TIsIntegral<ValueType>::value || vfc::TIsFloating<ValueType>::value), vfc::true_t,vfc::false_t>::type());
}

template<class ValueType>
inline
vfc::TPoint<ValueType>    vfc::TRect<ValueType>::centerIntern        (vfc::true_t)    const
{
    return TPoint<ValueType>(static_cast<ValueType>(left()+right())/static_cast<ValueType>(2),static_cast<ValueType>(top()+bottom())/static_cast<ValueType>(2));
}

template<class ValueType>
inline
vfc::TPoint<ValueType>    vfc::TRect<ValueType>::centerIntern        (vfc::false_t)    const
{
    return TPoint<ValueType>((left()+right())/static_cast<typename ValueType::value_type>(2),static_cast<ValueType>(top()+bottom())/static_cast<typename ValueType::value_type>(2));
}


template<class ValueType>
inline
vfc::TSize<ValueType>    vfc::TRect<ValueType>::size        (void)    const
{
    return TSize<ValueType>(width(),height());
}

template<class ValueType>
inline
typename vfc::TRect<ValueType>::area_type  vfc::TRect<ValueType>::area        (void)    const
{
    return (m_width * m_height);
}

template<class ValueType>
inline
bool    vfc::TRect<ValueType>::isEmpty        (void)    const
{
    return ( (static_cast<ValueType>(0) >= width()) || (static_cast<ValueType>(0) >= height()) );
}

template<class ValueType>
inline
bool    vfc::TRect<ValueType>::contains    (const TRect<ValueType>& f_otherRect)    const
{
    return (f_otherRect.left()   >= left())
        && (f_otherRect.top()    >= top())
        && (f_otherRect.right()  <= right())
        && (f_otherRect.bottom() <= bottom());
}

template<class ValueType>
inline
bool    vfc::TRect<ValueType>::contains    (const TPoint<ValueType>& f_point)    const
{
    return contains(f_point.x(),f_point.y());
}

template<class ValueType>
inline
bool    vfc::TRect<ValueType>::contains    (ValueType f_px, ValueType f_py)    const
{
    return (    (f_px>=left())    &&    (f_px<right())
            &&    (f_py>=top())    &&    (f_py<bottom()));
}

template<class ValueType>
inline
void    vfc::TRect<ValueType>::resize        (ValueType f_cx, ValueType f_cy)
{
    m_width        = f_cx;
    m_height    = f_cy;
}

template<class ValueType>
inline
void    vfc::TRect<ValueType>::resize        (const TSize<ValueType>& f_newSize)
{
    resize(f_newSize.cx(),f_newSize.cy());
}

template<class ValueType>
inline
void    vfc::TRect<ValueType>::normalize    (void)
{
    if (static_cast<ValueType>(0)>m_width)
    {
        m_left  += m_width;
        m_width  = -m_width;
    }
    if (static_cast<ValueType>(0)>m_height)
    {
        m_top   += m_height;
        m_height = -m_height;
    }
}

template<class ValueType>
inline
void    vfc::TRect<ValueType>::inflate    (ValueType f_dx, ValueType f_dy)
{
    inflate(f_dx,f_dy,f_dx,f_dy);
}

template<class ValueType>
inline
void    vfc::TRect<ValueType>::inflate    (ValueType f_dleft, ValueType f_dtop, ValueType f_dright, ValueType f_dbottom)
{
    m_left    -= f_dleft;
    m_top    -= f_dtop;

    m_width    += f_dleft+f_dright;
    m_height+= f_dtop +f_dbottom;
}

template<class ValueType>
inline
void    vfc::TRect<ValueType>::deflate    (ValueType f_dx, ValueType f_dy)
{
    inflate(-f_dx,-f_dy,-f_dx,-f_dy);
}

template<class ValueType>
inline
void    vfc::TRect<ValueType>::deflate    (ValueType f_dleft, ValueType f_dtop, ValueType f_dright, ValueType f_dbottom)
{
    inflate(-f_dleft,-f_dtop,-f_dright,-f_dbottom);
}

template<class ValueType>
inline
void    vfc::TRect<ValueType>::offset    (ValueType f_dx, ValueType f_dy)
{
    m_left        += f_dx;
    m_top        += f_dy;
}

template<class ValueType>
inline
void    vfc::TRect<ValueType>::moveTo    (ValueType f_x, ValueType f_y)
{
    m_left  = f_x;
    m_top   = f_y;
}

template<class ValueType>
inline
void    vfc::TRect<ValueType>::moveTo    (const TPoint<ValueType>& f_point)
{
moveTo(f_point.x(),f_point.y());
}

template<class ValueType>
inline
vfc::TRect<ValueType>    vfc::union_rect    (const TRect<ValueType>& f_op1, const TRect<ValueType>& f_op2)
{
    return TRect<ValueType>(    TPoint<ValueType>( (stlalias::min)(f_op1.left(),  f_op2.left()),
        (stlalias::min)(f_op1.top(),   f_op2.top())),
        TPoint<ValueType>( (stlalias::max)(f_op1.right(), f_op2.right()),
        (stlalias::max)(f_op1.bottom(),f_op2.bottom())));
}

template<class ValueType>
inline
vfc::TRect<ValueType>    vfc::intersect_rect    (const TRect<ValueType>& f_op1, const TRect<ValueType>& f_op2)
{
    return TRect<ValueType>(   TPoint<ValueType>( (stlalias::max)(f_op1.left(),  f_op2.left()),
        (stlalias::max)(f_op1.top(),   f_op2.top())),
        TPoint<ValueType>( (stlalias::min)(f_op1.right(), f_op2.right()),
        (stlalias::min)(f_op1.bottom(),f_op2.bottom())));
}

template<class ValueType>
inline
bool vfc::operator==(const TRect<ValueType>& f_lhs, const TRect<ValueType>& f_rhs)
{
    return ( vfc::isZero(f_lhs.width()  - f_rhs.width()) &&
             vfc::isZero(f_lhs.height() - f_rhs.height()) &&
             vfc::isZero(f_lhs.left()   - f_rhs.left()) &&
             vfc::isZero(f_lhs.top()    - f_rhs.top()) );
}

template<class ValueType>
inline
bool vfc::operator!=(const TRect<ValueType>& f_lhs, const TRect<ValueType>& f_rhs)
{
    return !(f_lhs==f_rhs);
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_trect.inl  $
//  Revision 1.3 2014/07/29 11:20:26MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - TRect<float> contains issue (mantis0004233)
//  Revision 1.2 2014/07/18 14:30:49MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_hash: call theTHash function object in serializeHash() explicitly to avoid ambiguity due to associated namespace lookup (mantis0004583)
//  Revision 1.1 2012/10/23 10:28:50MESZ SNU5KOR 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/core/core.pj
//=============================================================================

