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
///     $Source: vfc_rect.hpp $
///     $Revision: 1.20 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/08/13 09:33:27MESZ $
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


#ifndef VFC_RECT_HPP_INCLUDED
#define VFC_RECT_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"            // used for int32_t
#include "vfc/core/vfc_type_traits.hpp"        // VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL()

namespace vfc
{    // namespace vfc opened

    //=========================================================================
    // CPoint
    //-------------------------------------------------------------------------
    //! The CPoint structure defines the integer x- and y-coordinates of a 
    //! 2d point.
    //! @author zvh2hi
    //! @ingroup vfc_group_core_types
    //=========================================================================

    class    CPoint    
    {    
    public:
        //! default c'tor, initializes x and y member variables with zero
        CPoint (void);

        //! c'tor, initializes x and y member variables
        CPoint (int32_t f_x_i32, int32_t f_y_i32);

        //! offsets CPoint by adding specified position
        const CPoint& operator += (const CPoint& f_rhs);
        //! offsets CPoint by subtracting specified position
        const CPoint& operator -= (const CPoint& f_rhs);

        int32_t&    x    (void)         {    return m_x;}
        int32_t     x    (void) const   {    return m_x;}

        int32_t&    y    (void)         {    return m_y;}
        int32_t     y    (void) const   {    return m_y;}

    private:
        int32_t m_x,    //!< x-coordinate 
                m_y;    //!< y-coordinate
    };

    // traits specialization
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(THasTrivialDTor, CPoint,true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(THasTrivialCopy, CPoint,true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(THasTrivialSetZero, CPoint,true);

    //! Returns the sum of two points. @relates CPoint
    const CPoint    operator+   (const CPoint& f_op1, const CPoint& f_op2);
    //! Returns the difference of two points. @relates CPoint
    const CPoint    operator-   (const CPoint& f_op1, const CPoint& f_op2);

    //=========================================================================
    // CSize
    //-------------------------------------------------------------------------
    //! The CSize class implements a relative coordinate or extent (eg the 
    //! width and height of a rectangle).
    //! @sa CRect
    //! @author zvh2hi
    //! @ingroup vfc_group_core_types
    //=========================================================================

    class    CSize    
    {    
    public:
        
        //! default c'tor, initializes cx and cy member variables with zero
        CSize    (void);
        
        //! c'tor, initializes cx and cy member variables
        CSize    (int32_t f_cx_i32, int32_t f_cy_i32);

        //! Adds a size to CSize.
        const CSize& operator += (const CSize& f_rhs);
        //! Subtracts a size from CSize.
        const CSize& operator -= (const CSize& f_rhs);

        int32_t&    cx  (void)          {   return m_cx;}
        int32_t     cx  (void)  const   {   return m_cx;}

        int32_t&    cy  (void)          {   return m_cy;}
        int32_t     cy  (void)  const   {   return m_cy;}

    private:
        int32_t m_cx,    //!< extent in x-direction 
                m_cy;    //!< extent in y-direction
    };

    // traits specialization
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(THasTrivialDTor, CSize,true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(THasTrivialCopy, CSize,true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(THasTrivialSetZero, CSize,true);

    //! returns the sum of two sizes. @relates CSize
    const CSize    operator+ (const CSize& f_op1, const CSize& f_op2);
    //! returns the difference of two sizes. @relates CSize
    const CSize    operator- (const CSize& f_op1, const CSize& f_op2);

    //=========================================================================
    // CRect
    //-------------------------------------------------------------------------
    //! A CRect defines the coordinates of the (inside) upper-left and (outside) 
    //! lower-right corners of a rectangle.
    //! Keep in mind that the upper left corner is inside the rectangle, while 
    //! the lower right is 1 pixel to the right and below the last pixel in the 
    //! rectangle and therefore outside of the rectangle area.
    //! @par
    //! @image html core-rect.png
    //!
    //! When specifying a CRect, you must be careful to construct it so that it 
    //! is normalized — in other words, such that the value of the left coordinate 
    //! is less than the right and the top is less than the bottom. 
    //! For example, a top left of (10,10) and bottom right of (20,20) defines a 
    //! normalized rectangle but a top left of (20,20) and bottom right of (10,10) 
    //! defines a non-normalized rectangle. 
    //! If the rectangle is not normalized, many CRect member functions may 
    //! return incorrect results. 
    //! Especially an isEmpty() for a not normalized CRect will return true, as
    //! a not normalized rect is interpreted as "not existing" aka "a hole".
    //! Before you call a function that requires normalized rectangles, you can 
    //! normalize non-normalized rectangles by calling the normalize() function.
    //! @sa 
    //! - CPoint
    //! - CSize
    //! @author zvh2hi
    //! @ingroup vfc_group_core_types
    //=========================================================================

    class    CRect    
    {
    public:
        //! default, c'tor - sets top-left to origin and width and height to zero.
        CRect   (void);

        //! constructs CRect with top-left corner and specified extent.
        //! @param  f_left      Specifies the left position of CRect.
        //! @param  f_top       Specifies the top of CRect. 
        //! @param  f_width     Specifies the extent in x-direction.
        //! @param  f_height    Specifies the extent in y-direction.

        CRect   (   int32_t f_left,     int32_t f_top,      
                    int32_t f_width,    int32_t f_height);

        //! constructs CRect with top-left corner and specified extent.
        //! @param  f_topLeft   Specifies the origin point for the rectangle. 
        //!                     Corresponds to the top-left corner. 
        //! @param  f_size      Specifies the displacement from the top-left 
        //!                     corner to the bottom-right corner. 

        CRect   (   const CPoint& f_topLeft, const CSize& f_size);

        //! constructs CRect with top-left (inside) and bottom-right (outside) 
        //! corners.
        //! @param  f_topLeft       Specifies the top-left (inside) position
        //!                         of CRect.  
        //! @param  f_bottomRight   Specifies the bottom-right (outside) 
        //!                         position of CRect.

        CRect   (   const CPoint&   f_topLeft, const CPoint&   f_bottomRight);

        //! returns the left position (inside) of CRect (read only).
        int32_t     left    (void)  const   {    return m_left;}
        //! returns the left position (inside) of CRect (read/write).
        int32_t&    left    (void)          {    return m_left;}

        //! returns the top (inside) of CRect (read only).
        int32_t     top     (void)  const   {    return m_top;}
        //! returns the top (inside) of CRect (read/write).
        int32_t&    top     (void)          {    return m_top;}

        //! returns the width of CRect (read only).
        int32_t     width   (void) const   {    return m_width;}
        //! returns the width of CRect (read/write).
        int32_t&    width   (void)         {    return m_width;}

        //! returns the height of CRect (read only).
        int32_t     height  (void)  const   {    return m_height;}
        //! returns the height of CRect (read/write).
        int32_t&    height  (void)          {    return m_height;}

        //! returns the top-left position of the rect.
        CPoint      topLeft     (void)  const;

        //! returns the bottom-right position of the rect.
        CPoint      bottomRight (void)  const;

        //! returns the centerpoint C((right+left)/2,(bottom+top)/2) of the 
        //! rectangle .
        CPoint      center  (void)  const;

        //! returns the right position (outside) of CRect.
        int32_t     right   (void)  const   {   return m_left+m_width;}

        //! returns the bottom (outside) of CRect.
        int32_t     bottom  (void)  const   {   return m_top+m_height;}

        //! returns the size of the rect .
        CSize       size    (void)  const;

        //! returns the rect's area area = width*height.
        int32_t     area    (void)  const;

        //! determines whether CRect is empty. CRect is empty if the 
        //! width and/or height are 0, or if the rect is not normalized,
        //! i.e. width and/or height are negative.
        bool    isEmpty     (void)  const;

        //! determines whether the specified rect lies inside this rect, 
        //! identical coordinates are treated as inside.
        bool    contains    (const CRect& f_otherRect)  const;

        //! determines whether the specified point lies inside or outside 
        //! of this rect.
        bool    contains    (const CPoint& f_point)     const;

        //! determines whether the specified point p(x,y) lies inside or 
        //! outside of this rect.
        bool    contains    (int32_t f_px, int32_t f_py)    const;

        //! changes the size of the rect without changing the position of the 
        //! top-left corner
        void    resize      (int32_t f_cx, int32_t f_cy);

        //! changes the size of the rect without changing the position of the 
        //! top-left corner.
        void    resize      (const CSize& f_newSize);

        //! Standardizes the height and width of CRect.
        //! NormalizeRect compares the m_top and m_bottom values, and swaps them 
        //! if the m_top is greater than the m_bottom. 
        //! Similarly, it swaps the m_left and m_right values if the m_left is 
        //! greater than the m_right. 
        //! This function is useful when dealing with inverted rectangles.
        void    normalize   (void);

        //! Inflate rectangle's width and height by dx units to the left and 
        //! right ends of the rectangle and dy units to the top and bottom.
        //! @param  f_dx    Specifies the number of units to inflate the left and 
        //!                 right sides of CRect. 
        //! @param  f_dy    Specifies the number of units to inflate the top and 
        //!                 bottom of CRect. 
        void    inflate     (   int32_t f_dx, int32_t f_dy );

        //! Inflate rectangle's width and height by moving individual sides.
        //! Left side is moved to the left, right side is moved to the right,
        //! top is moved up and bottom is moved down.
        //! @param  f_dleft     Specifies the number of units to inflate the 
        //!                     left side of CRect.  
        //! @param  f_dtop      Specifies the number of units to inflate the 
        //!                     top side of CRect. 
        //! @param  f_dright    Specifies the number of units to inflate the 
        //!                     right side of CRect. 
        //! @param  f_dbottom   Specifies the number of units to inflate the 
        //!                     bottom side of CRect.

        void    inflate     (   int32_t f_dleft,    int32_t f_dtop,     
                                int32_t f_dright,   int32_t f_dbottom);

        //! Deflate rectangle's width and height by dx units to the left and 
        //! right ends of the rectangle and dy units to the top and bottom.
        //! @param  f_dx    Specifies the number of units to deflate the left 
        //!                 and right sides of CRect.
        //! @param  f_dy    Specifies the number of units to deflate the top 
        //!                 and bottom of CRect. 

        void    deflate    (    int32_t f_dx, int32_t f_dy );

        //! Deflate rectangle's width and height by moving individual sides.
        //! Left side is moved to the right, right side is moved to the left,
        //! top is moved down and bottom is moved up.
        //! @param  f_dleft     Specifies the number of units to deflate the 
        //!                     left side of CRect. 
        //! @param  f_dtop      Specifies the number of units to deflate the 
        //!                     top side of CRect. 
        //! @param  f_dright    Specifies the number of units to deflate the 
        //!                     right side of CRect. 
        //! @param  f_dbottom   Specifies the number of units to deflate the 
        //!                     bottom side of CRect.

        void    deflate    (    int32_t f_dleft,    int32_t f_dtop,                        
                                int32_t f_dright,   int32_t f_dbottom );

        //! Moves CRect by the specified offsets.
        void    offset    (int32_t f_dx, int32_t f_dy);

        //! Moves CRect to the specified x- and y-coordinates.
        //! @param  f_x The absolute x-coordinate for the upper-left corner 
        //!             of the rectangle.
        //! @param  f_y The absolute y-coordinate for the upper-left corner 
        //!             of the rectangle.

        void    moveTo    (    int32_t f_x, int32_t f_y );

        //! Moves CRect to the specified x- and y-coordinates.
        //! @param   f_point    A CPoint specifying the absolute upper-left 
        //!                     corner of the rectangle. 
        void    moveTo    (    const CPoint& f_point );

    private:
        int32_t m_left, 
                m_top, 
                m_width, 
                m_height;
    };

    // traits specialization
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(THasTrivialDTor, CRect,true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(THasTrivialCopy, CRect,true);
    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(THasTrivialSetZero, CRect,true);

    //-------------------------------------------------------------------------
    //! returns a rect which is equal to the union of the two specified 
    //! rectangles. 
    //! The union is the smallest rectangle that contains both source 
    //! rectangles. 
    //! @relates CRect
    //! @author zvh2hi
    //-------------------------------------------------------------------------
    CRect    union_rect        (const CRect& f_op1, const CRect& f_op2);

    //-------------------------------------------------------------------------
    //! Returns a rect which is equal to the intersection of the two 
    //! specified rectangles.
    //! The intersection is the largest rectangle contained in both existing 
    //! rectangles. 
    //! If the two specified rectangles don't intersect, the resulting 
    //! rectangle may become denormalized.
    //! This can be checked with CRect::isEmpty(). 
    //! Please keep in mind that a check with CRect::area() == 0 may fail.
    //! @note always check the resulting rectangle for denormalization!
    //! @relates CRect
    //! @author zvh2hi
    //-------------------------------------------------------------------------
    CRect    intersect_rect    (const CRect& f_op1, const CRect& f_op2);

    //-------------------------------------------------------------------------
    //! returns true if every of the four values (top, left, width, height)   
    //! of one rectangle is equal the its corresponding value in the other
    //! rectangle.
    //! @relates CRect
    //! @author kon3hi
    //-------------------------------------------------------------------------
    bool operator==(const CRect& f_lhs, const CRect& f_rhs);

    //-------------------------------------------------------------------------
    //! returns true if any of the four values (top, left, width, height) of  
    //! one of the rectangles is not equal to the corresponding value in 
    //! the other rectangle.
    //! @relates CRect
    //! @author kon3hi
    //-------------------------------------------------------------------------
    bool operator!=(const CRect& f_lhs, const CRect& f_rhs);

}    // namespace vfc closed

#include "vfc/core/vfc_rect.inl"

#endif //VFC_RECT_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_rect.hpp  $
//  Revision 1.20 2014/08/13 09:33:27MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - CRect and TRect:: isEmpty does not work correct on non-normalized rects (mantis0004644)
//  Revision 1.19 2008/08/11 14:32:57MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - undone changes in intersect_rect (mantis1419)
//  - added more documentation to intersect_rect
//  Revision 1.18 2008/08/11 14:00:02CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed type traits specializations (mantis2198)
//  Revision 1.17 2007/10/30 11:27:49CET Koenig Matthias (CR/AEM5) (kon3hi) 
//  added comparison operators (mantis 0001753)
//  Revision 1.16 2007/07/23 09:33:34CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  - added documentation
//  Revision 1.15 2007/06/22 15:12:25CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - corrected redeclared function parameter names (mantis 1691)
//  Revision 1.14 2007/03/14 17:13:41CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced c'tor with default arguments with two c'tors (mantis1501)
//  Revision 1.13 2007/03/12 13:28:16CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed qualifier of arithmetic operators return values (mantis1488)
//  Revision 1.12 2007/02/20 16:28:15CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed intersect_rect() implementation (mantis1419)
//  - replaced old header/footer
//  - comment reformatting
//  Revision 1.11 2006/11/16 14:41:08CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.10 2006/06/22 13:37:25CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -fixed and added docu
//  -changed identifier
//  -reformatted code
//  Revision 1.9 2006/06/21 19:11:04CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -changed rect implementation and interface (mantis1047)
//  Revision 1.8 2006/03/09 10:47:24CET Muehlmann Karsten (AE-DA/ESA3) * (muk2lr) 
//  - added CRect.contains(const CRect&) (mantis 1012)
//  - changed CRect.isInside to CRect.contains for points to be consistent
//  Revision 1.7 2005/11/08 17:58:08CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added TIsPOD<> traits specialization
//  Revision 1.6 2005/11/08 17:44:37CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -replaced structs by classes
//  Revision 1.5 2005/11/08 17:30:08CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added more docu
//  Revision 1.4 2005/11/08 17:09:21CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -small docu update
//  Revision 1.3 2005/11/08 17:07:14CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added CPoint, CSize and CRect docu ( thx to MSDN ;)
//=============================================================================
