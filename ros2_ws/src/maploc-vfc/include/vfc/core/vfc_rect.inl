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
///     $Source: vfc_rect.inl $
///     $Revision: 1.18 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (ZVH2HI) $
///     $Date: 2008/08/11 14:32:56MESZ $
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


#include <algorithm>             // used for ::std::min(), ::std::max()
#include "vfc/core/vfc_math.hpp" // used for vfc::isZero

inline    
vfc::CPoint::CPoint (void) 
:    m_x(0), m_y(0) 
{
}

inline    
vfc::CPoint::CPoint (int32_t f_x_i32, int32_t f_y_i32) 
:    m_x(f_x_i32), m_y(f_y_i32) 
{
}

inline
const vfc::CPoint& vfc::CPoint::operator += (const CPoint& f_rhs) 
{ 
    m_x += f_rhs.m_x; 
    m_y += f_rhs.m_y; 
    return *this;
}

inline
const vfc::CPoint& vfc::CPoint::operator -= (const CPoint& f_rhs) 
{ 
    m_x -= f_rhs.m_x; 
    m_y -= f_rhs.m_y; 
    return *this;
}

inline
const vfc::CPoint    vfc::operator+ (const CPoint& f_op1, const CPoint& f_op2)    
{    
    return CPoint(f_op1) += f_op2;
}

inline
const vfc::CPoint    vfc::operator- (const CPoint& f_op1, const CPoint& f_op2)    
{    
    return CPoint(f_op1) -= f_op2 ;
}

inline
vfc::CSize::CSize    (void)
:    m_cx(0), m_cy(0) 
{
}


inline
vfc::CSize::CSize    (int32_t f_cx_i32, int32_t f_cy_i32)
:    m_cx(f_cx_i32), m_cy(f_cy_i32) 
{
}

inline
const vfc::CSize& vfc::CSize::operator += (const CSize& f_rhs) 
{ 
    m_cx += f_rhs.m_cx; 
    m_cy += f_rhs.m_cy; 
    return *this;
}

inline
const vfc::CSize& vfc::CSize::operator -= (const CSize& f_rhs) 
{ 
    m_cx -= f_rhs.m_cx; 
    m_cy -= f_rhs.m_cy; 
    return *this;
}

inline
const vfc::CSize    vfc::operator+ (const CSize& f_op1, const CSize& f_op2)    
{    
    return CSize(f_op1) += f_op2;
}
    
inline
const vfc::CSize    vfc::operator- (const CSize& f_op1, const CSize& f_op2)    
{    
    return CSize(f_op1) -= f_op2 ;
}


inline
vfc::CRect::CRect    (void) 
: m_left(0), m_top(0), m_width(0), m_height(0) 
{
}

inline
vfc::CRect::CRect    (int32_t f_left, int32_t f_top, int32_t f_width, int32_t f_height)
:    m_left  (f_left), 
    m_top   (f_top), 
    m_width (f_width), 
    m_height(f_height)
{
}

inline
vfc::CRect::CRect    (const CPoint& f_topLeft, const CSize& f_size) 
:    m_left  (f_topLeft.x()), 
    m_top   (f_topLeft.y()), 
    m_width (f_size.cx()), 
    m_height(f_size.cy())
{
}

inline
vfc::CRect::CRect    (const CPoint& f_topLeft, const CPoint& f_bottomRight)
:    m_left  (f_topLeft.x()), 
    m_top   (f_topLeft.y()), 
    m_width (f_bottomRight.x()-f_topLeft.x()), 
    m_height(f_bottomRight.y()-f_topLeft.y())
{
}

inline
vfc::CPoint    vfc::CRect::topLeft        (void)    const    
{    
    return CPoint(left(),top());
}

inline
vfc::CPoint    vfc::CRect::bottomRight    (void)    const    
{    
    return CPoint(right(), bottom());
}

inline
vfc::CPoint    vfc::CRect::center        (void)    const    
{    
    return CPoint((left()+right())/2,(top()+bottom())/2);
}

inline
vfc::CSize    vfc::CRect::size        (void)    const    
{    
    return CSize(width(),height());
}

inline
vfc::int32_t    vfc::CRect::area        (void)    const    
{    
    return width()*height();
}

inline
bool    vfc::CRect::isEmpty        (void)    const        
{    
    return ( (0 >= width()) || (0 >= height()) );
}

inline
bool    vfc::CRect::contains    (const CRect& f_otherRect)    const        
{    
    return (   contains(f_otherRect.topLeft()) 
            && contains(f_otherRect.right()-1,f_otherRect.bottom()-1));
}

inline
bool    vfc::CRect::contains    (const CPoint& f_point)    const        
{    
    return contains(f_point.x(),f_point.y());
}

inline
bool    vfc::CRect::contains    (int32_t f_px, int32_t f_py)    const    
{    
    return (    (f_px>=left())    &&    (f_px<right()) 
            &&    (f_py>=top())    &&    (f_py<bottom()));
}

inline
void    vfc::CRect::resize        (int32_t f_cx, int32_t f_cy)    
{    
    m_width        = f_cx;
    m_height    = f_cy;
}

inline
void    vfc::CRect::resize        (const CSize& f_newSize)            
{    
    resize(f_newSize.cx(),f_newSize.cy());
}

inline
void    vfc::CRect::normalize    (void)    
{    
    if (0>m_width) 
    {
        m_left  += m_width;
        m_width  = -m_width;
    }
    if (0>m_height) 
    {
        m_top   += m_height;
        m_height = -m_height;
    }
}

inline
void    vfc::CRect::inflate    (int32_t f_dx, int32_t f_dy)                            
{ 
    inflate(f_dx,f_dy,f_dx,f_dy);
}

inline
void    vfc::CRect::inflate    (int32_t f_dleft, int32_t f_dtop, int32_t f_dright, int32_t f_dbottom)    
{ 
    m_left    -= f_dleft; 
    m_top    -= f_dtop; 
    
    m_width    += f_dleft+f_dright; 
    m_height+= f_dtop +f_dbottom;
}

inline
void    vfc::CRect::deflate    (int32_t f_dx, int32_t f_dy)                            
{ 
    inflate(-f_dx,-f_dy,-f_dx,-f_dy);
}

inline
void    vfc::CRect::deflate    (int32_t f_dleft, int32_t f_dtop, int32_t f_dright, int32_t f_dbottom)    
{ 
    inflate(-f_dleft,-f_dtop,-f_dright,-f_dbottom);
}

inline
void    vfc::CRect::offset    (int32_t f_dx, int32_t f_dy)    
{ 
    m_left        += f_dx; 
    m_top        += f_dy; 
}

inline
void    vfc::CRect::moveTo    (int32_t f_x, int32_t f_y)        
{ 
    m_left  = f_x;
    m_top   = f_y;
}

inline
void    vfc::CRect::moveTo    (const CPoint& f_point)            
{
    moveTo(f_point.x(),f_point.y());
}

inline
vfc::CRect    vfc::union_rect    (const CRect& f_op1, const CRect& f_op2)
{
    return CRect(    CPoint( (stlalias::min)(f_op1.left(),  f_op2.left()), 
                            (stlalias::min)(f_op1.top(),   f_op2.top())), 
                    CPoint( (stlalias::max)(f_op1.right(), f_op2.right()), 
                            (stlalias::max)(f_op1.bottom(),f_op2.bottom())));
}

inline
vfc::CRect    vfc::intersect_rect    (const CRect& f_op1, const CRect& f_op2)
{
    return CRect(   CPoint( (stlalias::max)(f_op1.left(),  f_op2.left()), 
                            (stlalias::max)(f_op1.top(),   f_op2.top())), 
                    CPoint( (stlalias::min)(f_op1.right(), f_op2.right()), 
                            (stlalias::min)(f_op1.bottom(),f_op2.bottom())));
}

inline 
bool vfc::operator==(const CRect& f_lhs, const CRect& f_rhs)
{
    return ( (f_lhs.width() == f_rhs.width()) &&
             (f_lhs.height() == f_rhs.height()) &&
             (f_lhs.left() == f_rhs.left()) &&
             (f_lhs.top() == f_rhs.top()) );
}

inline 
bool vfc::operator!=(const CRect& f_lhs, const CRect& f_rhs)
{
    return !(f_lhs==f_rhs);
} 


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_rect.inl  $
//  Revision 1.18 2008/08/11 14:32:56MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - undone changes in intersect_rect (mantis1419)
//  - added more documentation to intersect_rect
//  Revision 1.17 2008/07/21 07:52:58CEST Vinaykumar Setty (RBEI/EAE6) (vmr1kor) 
//  operator - implemented in terms of there assignment version( Mantis :- 1704)
//  Revision 1.16 2007/11/22 20:21:13IST Dilip Krishna (RBEI/EAE6) (dkn2kor) 
//  - writen operators in terms of their assignment implementation (mantis 1704)
//  Revision 1.15 2007/10/31 18:10:13IST vmr1kor 
//  precedence confusion in line 331 with '>', '||' solved
//  Matis Id:- 0001705 
//  Revision 1.14 2007/10/30 15:57:48IST Koenig Matthias (CR/AEM5) (kon3hi) 
//  added comparison operators (mantis 0001753)
//  Revision 1.13 2007/07/23 09:33:13CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - moved documentation to header (mantis1744)
//  Revision 1.12 2007/03/29 13:13:23CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replace ::std with stlalias (mantis1534)
//  Revision 1.11 2007/03/14 17:13:44CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced c'tor with default arguments with two c'tors (mantis1501)
//  Revision 1.10 2007/03/12 13:28:15CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed qualifier of arithmetic operators return values (mantis1488)
//  Revision 1.9 2007/02/20 16:28:16CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed intersect_rect() implementation (mantis1419)
//  - replaced old header/footer
//  - comment reformatting
//  Revision 1.8 2006/11/16 14:41:13CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.7 2006/06/22 13:36:46CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -reformatted code
//  Revision 1.6 2006/06/21 19:11:04CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -changed rect implementation and interface (mantis1047)
//  Revision 1.5 2006/03/09 10:48:01CET Muehlmann Karsten (AE-DA/ESA3) (muk2lr) 
//  - added CRect.contains(const CRect&) (mantis 1012)
//  - changed CRect.isInside to CRect.contains for points to be consistent
//  - changed isEmpty to be checked with isZero
//  Revision 1.4 2005/11/08 17:44:37CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -replaced structs by classes
//  Revision 1.2 2005/11/08 14:56:01CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added missing includes
//=============================================================================
