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
///     $Source: vfc_trect.hpp $
///     $Revision: 1.4 $
///     $Author: gaj2kor
///     $Date: 2014/09/24 13:46:42MESZ $
///     $State: in_work $
//=============================================================================

#ifndef VFC_TRECT_HPP_INCLUDED  
#define VFC_TRECT_HPP_INCLUDED

//-----------------------------------------------------------------------------
// system includes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// includes for identifiers needed to be known by definition
//-----------------------------------------------------------------------------
#include "vfc/core/vfc_types.hpp"               // used for int32_t
#include "vfc/core/vfc_type_traits.hpp"         // used for VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL()

//-----------------------------------------------------------------------------
// forward declarations for identifiers needed to be known by name only
//-----------------------------------------------------------------------------

//  === BEGIN_SPEC =============================================================================
/// vfc templated rectangular class
//  --------------------------------------------------------------------------------------------
//! @brief vfc templated rectangular class
//! @par Reference (CS-CRM / DOORS ID / EA ...):
//!   link(s) to related requirement(s), work package(s), test case(s), architecture etc
//! @par Description:
//!   A TRect defines the coordinates of the (inside) upper-left and (outside)
//!      lower-right corners of a rectangle.
//! @par Error handling:
//! @par Timing/Scheduling constraints:
//! @pre               
//! @post              
//! @par Input:        
//! @par Output:       
//! @par Return Value: 
//! $Source: vfc_trect.hpp $
//! @author            gaj2kor
//! @ingroup           vfc_group_core_types
//! @todo          
//  === END_SPEC ================================================================================

//-----------------------------------------------------------------------------
// function/class definitions
//-----------------------------------------------------------------------------

namespace vfc
{
    // namespace vfc opened


    //======================================================
    // TPoint
    //------------------------------------------------------
    /// The TPoint structure defines the integer x- and y-coordinates 
    /// of a 2d point.
    /// @par Reference (CS-CRM / DOORS ID / EA):
    /// $Source: vfc_trect.hpp $
    /// @author gaj2kor
    /// @ingroup vfc_group_core_types
    //======================================================
    template<class ValueType>
    class    TPoint
    {
    public:
        typedef ValueType        value_type;

        //---------------------------------------------------------------------
        /// Default constructor
        /// default c'tor, initializes x and y member variables with zero.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param 
        /// @return 
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        TPoint (void);

        //---------------------------------------------------------------------
        /// Constructor
        /// c'tor, initializes x and y member variables .
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param value for x-axis
        /// @param value for y-axis
        /// @return 
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        TPoint (value_type f_x, value_type f_y);


        //---------------------------------------------------------------------
        /// operator +=
        /// Offsets TPoint by adding specified position
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param Object of TPoint
        /// @return Object of TPoint
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        const TPoint& operator += (const TPoint& f_rhs);

        //---------------------------------------------------------------------
        /// operator -=
        /// Offsets TPoint by subtracting specified position
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param Object of TPoint
        /// @return Object of TPoint
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        const TPoint& operator -= (const TPoint& f_rhs);

        //---------------------------------------------------------------------
        /// Set value of x axis
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param 
        /// @return value for x-axis
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type&    x    (void)         {    return m_x;}

        //---------------------------------------------------------------------
        /// Set value of x axis. const interface
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param 
        /// @return value for x-axis
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type     x    (void) const   {    return m_x;}

        //---------------------------------------------------------------------
        /// Set value of y axis
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param 
        /// @return value for y-axis
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type&    y    (void)         {    return m_y;}

        //---------------------------------------------------------------------
        /// Set value of y axis. const interface
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param 
        /// @return value for y-axis
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type     y    (void) const   {    return m_y;}

    private:
        value_type m_x,    //!< x-coordinate
                   m_y;    //!< y-coordinate
    };

    // traits specialization
    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(THasTrivialDTor   , TPoint<T>,THasTrivialDTor<T>::value   );
    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(THasTrivialCopy   , TPoint<T>,THasTrivialCopy<T>::value   );
    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(THasTrivialSetZero, TPoint<T>,THasTrivialSetZero<T>::value);

    //---------------------------------------------------------------------
    /// Returns the sum of two points. @relates TPoint
    /// @par Reference (CS-CRM / DOORS ID / EA):
    /// @param Object of TPoint
    /// @param Object of TPoint
    /// @return Object of TPoint
    /// $Source: vfc_trect.hpp $
    /// @author gaj2kor
    /// @ingroup vfc_group_core_types
    //---------------------------------------------------------------------    
    template<class ValueType>
    const TPoint<ValueType>    operator+   (const TPoint<ValueType>& f_op1, const TPoint<ValueType>& f_op2);

    //---------------------------------------------------------------------
    /// Returns the difference of two points. @relates TPoint
    /// @par Reference (CS-CRM / DOORS ID / EA):
    /// @param Object of TPoint
    /// @param Object of TPoint
    /// @return Object of TPoint
    /// $Source: vfc_trect.hpp $
    /// @author gaj2kor
    /// @ingroup vfc_group_core_types
    //---------------------------------------------------------------------    
    template<class ValueType>
    const TPoint<ValueType>    operator-   (const TPoint<ValueType>& f_op1, const TPoint<ValueType>& f_op2);

    //=========================================================================
    // TSize
    //-------------------------------------------------------------------------
    /// The TSize class implements a relative coordinate or extent (eg the
    /// width and height of a rectangle).
    /// @sa TRect
    /// @par Reference (CS-CRM / DOORS ID / EA):
    /// $Source: vfc_trect.hpp $
    /// @author gaj2kor
    /// @ingroup vfc_group_core_types
    //=========================================================================
    template<class ValueType>
    class    TSize
    {
    public:
        typedef ValueType        value_type;

        //---------------------------------------------------------------------
        /// Default constructor
        /// default c'tor, initializes cx and cy member variables with zero
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param 
        /// @return 
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        TSize    (void);

        //---------------------------------------------------------------------
        /// Default constructor
        /// default c'tor, initializes cx and cy member variables.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param value for x-axis
        /// @param value for y-axis
        /// @return 
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        TSize    (value_type f_cx, value_type f_cy);

        //---------------------------------------------------------------------
        /// operator += 
        /// Adds a size to TSize.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param Object of TSize
        /// @return Object of TSize
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        const TSize& operator += (const TSize& f_rhs);

        //---------------------------------------------------------------------
        /// operator -= 
        /// Subtracts a size to TSize.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param Object of TSize
        /// @return Object of TSize
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        const TSize& operator -= (const TSize& f_rhs);

        //---------------------------------------------------------------------
        /// Set value of x axis
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param 
        /// @return value for x-axis
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type&    cx  (void)          {   return m_cx;}

        //---------------------------------------------------------------------
        /// Set value of x axis. const interface
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param 
        /// @return value for x-axis
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type     cx  (void)  const   {   return m_cx;}

        //---------------------------------------------------------------------
        /// Set value of y axis. 
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param 
        /// @return value for y-axis
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type&    cy  (void)          {   return m_cy;}

        //---------------------------------------------------------------------
        /// Set value of y axis. const interface
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param 
        /// @return value for y-axis
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type     cy  (void)  const   {   return m_cy;}

    private:
        value_type m_cx,    ///< extent in x-direction
                      m_cy;    ///< extent in y-direction
    };

    // traits specialization
    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(THasTrivialDTor   , TSize<T>, THasTrivialDTor<T>::value);
    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(THasTrivialCopy   , TSize<T>, THasTrivialCopy<T>::value);
    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(THasTrivialSetZero, TSize<T>, THasTrivialSetZero<T>::value);

    //---------------------------------------------------------------------
    /// returns the sum of two sizes. @relates TSize
    /// @par Reference (CS-CRM / DOORS ID / EA):
    /// @param Object of TSize
    /// @param Object of TSize
    /// @return Object of TSize
    /// $Source: vfc_trect.hpp $
    /// @author gaj2kor
    /// @ingroup vfc_group_core_types
    //---------------------------------------------------------------------    
    template<class ValueType>
    const TSize<ValueType>    operator+ (const TSize<ValueType>& f_op1, const TSize<ValueType>& f_op2);

    //---------------------------------------------------------------------
    /// returns the difference of two sizes. @relates TSize
    /// @par Reference (CS-CRM / DOORS ID / EA):
    /// @param Object of TSize
    /// @param Object of TSize
    /// @return Object of TSize
    /// $Source: vfc_trect.hpp $
    /// @author gaj2kor
    /// @ingroup vfc_group_core_types
    //---------------------------------------------------------------------    
    template<class ValueType>
    const TSize<ValueType>    operator- (const TSize<ValueType>& f_op1, const TSize<ValueType>& f_op2);


    //=========================================================================
    // TRect
    //-------------------------------------------------------------------------
    /// A TRect defines the coordinates of the (inside) upper-left and (outside)
    /// lower-right corners of a rectangle.
    /// Keep in mind that the upper left corner is inside the rectangle, while
    /// the lower right is 1 pixel to the right and below the last pixel in the
    /// rectangle and therefore outside of the rectangle area.
    /// @par
    /// @image html core-rect.png
    ///
    /// When specifying a TRect, you must be careful to construct it so that it
    /// is normalized — in other words, such that the value of the left coordinate
    /// is less than the right and the top is less than the bottom.
    /// For example, a top left of (10,10) and bottom right of (20,20) defines a
    /// normalized rectangle but a top left of (20,20) and bottom right of (10,10)
    /// defines a non-normalized rectangle.
    /// If the rectangle is not normalized, many TRect member functions may
    /// return incorrect results.
    //! Especially an isEmpty() for a not normalized TRect will return true, as
    //! a not normalized rect is interpreted as "not existing" aka "a hole".
    /// Before you call a function that requires normalized rectangles, you can
    /// normalize non-normalized rectangles by calling the normalize() function.
    /// @sa
    /// - TPoint
    /// - TSize
    /// @par Reference (CS-CRM / DOORS ID / EA):
    /// $Source: vfc_trect.hpp $
    /// @author gaj2kor
    /// @ingroup vfc_group_core_types
    //=========================================================================
    template<class ValueType>
    class    TRect
    {
    public:
        typedef ValueType                   value_type;

        typedef typename TRectAreaTypeTraits<ValueType>::area_type area_type;

        //---------------------------------------------------------------------
        /// Default constructor
        /// default, c'tor - sets top-left to origin and width and height to zero.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param 
        /// @return 
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        TRect   (void);

        //---------------------------------------------------------------------
        /// Constructs 
        /// constructs TRect with top-left corner and specified extent.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  f_left      Specifies the left position of TRect
        /// @param  f_top       Specifies the top of TRect.
        /// @param  f_width     Specifies the extent in x-direction.
        /// @param  f_height    Specifies the extent in y-direction.
        /// @return 
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        TRect   (   value_type f_left,     value_type f_top,
        value_type f_width,    value_type f_height);

        //---------------------------------------------------------------------
        /// Constructs 
        /// constructs TRect with top-left corner and specified extent.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  f_topLeft   Specifies the origin point for the rectangle.
        ///                     Corresponds to the top-left corner.
        /// @param  f_size      Specifies the displacement from the top-left
        ///                     corner to the bottom-right corner.
        /// @return 
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        TRect   (   const TPoint<ValueType>& f_topLeft, const TSize<ValueType>& f_size);

        //---------------------------------------------------------------------
        /// Constructs 
        /// constructs TRect with top-left (inside) and bottom-right (outside)
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  f_topLeft       Specifies the top-left (inside) position
        ///                         of TRect.
        /// @param  f_bottomRight   Specifies the bottom-right (outside)
        ///                         position of TRect.
        /// @return 
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        TRect   (   const TPoint<ValueType>&   f_topLeft, const TPoint<ValueType>&   f_bottomRight);

        //---------------------------------------------------------------------
        /// Returns the left position (inside) of TRect (read only). const interface.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return        Specifies the left position of TRect        
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type     left    (void)  const   {    return m_left;}

        //---------------------------------------------------------------------
        /// Returns the left position (inside) of TRect (read/write).
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return        Specifies the left position of TRect        
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type&    left    (void)          {    return m_left;}

        //---------------------------------------------------------------------
        /// returns the top (inside) of TRect (read only). const interface.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return        Specifies the top position of TRect        
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type     top     (void)  const   {    return m_top;}

        //---------------------------------------------------------------------
        /// returns the top (inside) of TRect (read/write).
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return        Specifies the top position of TRect        
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type&    top     (void)          {    return m_top;}

        //---------------------------------------------------------------------
        /// returns the width of TRect (read only).
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return        Specifies the width position of TRect        
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type     width   (void) const   {    return m_width;}

        //---------------------------------------------------------------------
        /// returns the width of TRect (read/write).
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return        Specifies the width position of TRect        
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type&    width   (void)         {    return m_width;}

        //---------------------------------------------------------------------
        /// returns the height of TRect (read only).
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return        Specifies the height position of TRect        
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type     height  (void)  const   {    return m_height;}

        //---------------------------------------------------------------------
        /// returns the height of TRect (read/write).
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return        Specifies the height position of TRect        
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type&    height  (void)          {    return m_height;}

        //---------------------------------------------------------------------
        /// returns the top-left position of TRect (read only).
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return        Specifies the top-left position of TRect        
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        TPoint<ValueType>      topLeft     (void)  const;

        //---------------------------------------------------------------------
        /// returns the bottom-right position of TRect (read only).
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return        Specifies the bottom-right position of TRect        
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        TPoint<ValueType>      bottomRight (void)  const;

        //---------------------------------------------------------------------
        /// returns the centerpoint C((right+left)/2,(bottom+top)/2) of the
        /// rectangle .
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return     Object of TPoint which specify center of rect
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        TPoint<ValueType>      center  (void)  const;

        //---------------------------------------------------------------------
        /// returns the right position (outside) of TRect.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return     right position (outside) of TRect.
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type     right   (void)  const   {   return m_left+m_width;}

        //---------------------------------------------------------------------
        /// returns the bottom (outside) of TRect.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return     bottom position (outside) of TRect.
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        value_type     bottom  (void)  const   {   return m_top+m_height;}

        //---------------------------------------------------------------------
        /// returns the size of the rect .
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return     Object of TSize which specify size of the rect .
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------    
        TSize<ValueType>       size    (void)  const;

        //---------------------------------------------------------------------
        /// returns the rect's area. area = width*height.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return     area of the rect .
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------            
        area_type area    (void)  const;

        //---------------------------------------------------------------------
        /// determines whether TRect is empty. TRect is empty if the
        /// width and/or height are 0, or if the rect is not normalized,
        /// i.e. width and/or height are negative.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return     bool true to indicate rect is empty and false otherwise.
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------            
        bool    isEmpty     (void)  const;    

        //---------------------------------------------------------------------
        /// determines whether the specified rect lies inside this rect,
        ///    identical coordinates are treated as inside.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return     bool true to indicate rect contains rect and false otherwise.
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------            
        bool    contains    (const TRect& f_otherRect)  const;

        //---------------------------------------------------------------------
        /// determines whether the specified point lies inside or outside
        ///    of this rect.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return     bool true to indicate rect contains point and false otherwise.
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------            
        bool    contains    (const TPoint<ValueType>& f_point)     const;

        //---------------------------------------------------------------------
        /// determines whether the specified point p(x,y) lies inside or
        ///    outside of this rect.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  f_px     Specifies the extent in x-direction.
        /// @param  f_py    Specifies the extent in y-direction.
        /// @return     bool true to indicate rect contains point and false otherwise.
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------            
        bool    contains    (value_type f_px, value_type f_py)    const;

        //---------------------------------------------------------------------
        /// changes the size of the rect without changing the position of the
        ///    top-left corner
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  f_cx     Specifies the extent in x-direction.
        /// @param  f_cy     Specifies the extent in y-direction.
        /// @return     
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------            
        void    resize      (value_type f_cx, value_type f_cy);

        //---------------------------------------------------------------------
        /// changes the size of the rect without changing the position of the
        ///    top-left corner.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  f_cx     Specifies the extent in x-direction.
        /// @return     
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------            
        void    resize      (const TSize<ValueType>& f_newSize);

        //---------------------------------------------------------------------
        /// Standardizes the height and width of TRect.
        /// NormalizeRect compares the m_top and m_bottom values, and swaps them
        /// if the m_top is greater than the m_bottom.
        /// Similarly, it swaps the m_left and m_right values if the m_left is
        /// greater than the m_right.
        /// This function is useful when dealing with inverted rectangles.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  
        /// @return     
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------            
        void    normalize   (void);

        //---------------------------------------------------------------------
        /// Inflate rectangle's width and height by dx units to the left and
        /// right ends of the rectangle and dy units to the top and bottom.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  f_dx    Specifies the number of units to inflate the left and
        ///                 right sides of TRect.
        /// @param  f_dy    Specifies the number of units to inflate the top and
        ///                 bottom of TRect.
        /// @return     
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------            
        void    inflate     (   value_type f_dx, value_type f_dy );

        //---------------------------------------------------------------------
        /// Inflate rectangle's width and height by moving individual sides.
        /// Left side is moved to the left, right side is moved to the right,
        /// top is moved up and bottom is moved down.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  f_dleft     Specifies the number of units to inflate the
        ///                     left side of TRect.
        /// @param  f_dtop      Specifies the number of units to inflate the
        ///                     top side of TRect.
        /// @param  f_dright    Specifies the number of units to inflate the
        ///                     right side of TRect.
        /// @param  f_dbottom   Specifies the number of units to inflate the
        ///                     bottom side of TRect.
        /// @return     
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------            
        void    inflate     (   value_type f_dleft,    value_type f_dtop,
                                value_type f_dright,   value_type f_dbottom);

        //---------------------------------------------------------------------
        /// Deflate rectangle's width and height by dx units to the left and
        /// right ends of the rectangle and dy units to the top and bottom.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  f_dx    Specifies the number of units to deflate the left
        ///                 and right sides of TRect.
        /// @param  f_dy    Specifies the number of units to deflate the top
        ///                 and bottom of TRect.
        /// @return     
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------            
        void    deflate    (    value_type f_dx, value_type f_dy );

        //---------------------------------------------------------------------
        /// Deflate rectangle's width and height by moving individual sides.
        /// Left side is moved to the right, right side is moved to the left,
        /// top is moved down and bottom is moved up.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  f_dleft     Specifies the number of units to deflate the
        ///                     left side of TRect.
        /// @param  f_dtop      Specifies the number of units to deflate the
        ///                     top side of TRect.
        /// @param  f_dright    Specifies the number of units to deflate the
        ///                     right side of TRect.
        /// @param  f_dbottom   Specifies the number of units to deflate the
        ///                     bottom side of TRect.
        /// @return     
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------            
        void    deflate    (    value_type f_dleft,    value_type f_dtop,
                                value_type f_dright,   value_type f_dbottom );

        //---------------------------------------------------------------------
        /// Moves TRect by the specified offsets.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  f_dx      Specifies the number of units to shift in x dir
        /// @param  f_dy      Specifies the number of units to shift in y dir
        /// @return     
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------                    
        void    offset    (value_type f_dx, value_type f_dy);

        //---------------------------------------------------------------------
        /// Moves TRect to the specified x- and y-coordinates.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param  f_x The absolute x-coordinate for the upper-left corner
        ///             of the rectangle.
        /// @param  f_y The absolute y-coordinate for the upper-left corner
        ///             of the rectangle.
        /// @return     
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------                    
        void    moveTo    (    value_type f_x, value_type f_y );

        //---------------------------------------------------------------------
        /// Moves TRect to the specified x- and y-coordinates.
        /// @par Reference (CS-CRM / DOORS ID / EA):
        /// @param   f_point    A TPoint specifying the absolute upper-left
        ///                     corner of the rectangle.
        /// @return     
        /// $Source: vfc_trect.hpp $
        /// @author gaj2kor
        /// @ingroup vfc_group_core_types
        //---------------------------------------------------------------------                    
        void    moveTo    (    const TPoint<ValueType>& f_point );

    private:

        TPoint<ValueType>      centerIntern        (vfc::true_t)    const;
        TPoint<ValueType>      centerIntern        (vfc::false_t)    const;

        value_type m_left,
                   m_top,
                   m_width,
                   m_height;
    };

    // traits specialization
    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(THasTrivialDTor   , TRect<T>, THasTrivialDTor<T>::value);
    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(THasTrivialCopy   , TRect<T>, THasTrivialCopy<T>::value);
    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(THasTrivialSetZero, TRect<T>, THasTrivialSetZero<T>::value);

    //---------------------------------------------------------------------
    /// Returns a rect which is equal to the union of the two specified
    /// rectangles.
    /// The union is the smallest rectangle that contains both source
    /// rectangles.
    /// @par Reference (CS-CRM / DOORS ID / EA):
    /// @param   f_op1    Object of first TRect
    /// @param   f_op2    Object of second TRect
    /// @return     Returns a rect which is equal to the union
    /// $Source: vfc_trect.hpp $
    /// @author gaj2kor
    /// @ingroup vfc_group_core_types
    //---------------------------------------------------------------------                    
    template<class ValueType>
    TRect<ValueType>    union_rect        (const TRect<ValueType>& f_op1, const TRect<ValueType>& f_op2);

    //---------------------------------------------------------------------
    /// Returns a rect which is equal to the intersection of the two
    /// specified rectangles.
    /// The intersection is the largest rectangle contained in both existing
    /// rectangles.
    /// If the two specified rectangles don't intersect, the resulting
    /// rectangle may become denormalized.
    /// This can be checked with TRect::isEmpty().
    /// Please keep in mind that a check with TRect::area() == 0 may fail.
    /// @par Reference (CS-CRM / DOORS ID / EA):
    /// @param   f_op1    Object of first TRect
    /// @param   f_op2    Object of second TRect
    /// @return     Returns a rect which is equal to the intersection
    /// $Source: vfc_trect.hpp $
    /// @author gaj2kor
    /// @ingroup vfc_group_core_types
    //---------------------------------------------------------------------                    
    template<class ValueType>
    TRect<ValueType>    intersect_rect    (const TRect<ValueType>& f_op1, const TRect<ValueType>& f_op2);

    //---------------------------------------------------------------------
    /// returns true if every of the four values (top, left, width, height)
    /// of one rectangle is equal the its corresponding value in the other
    /// rectangle.
    /// @par Reference (CS-CRM / DOORS ID / EA):
    /// @param   f_op1    Object of first TRect
    /// @param   f_op2    Object of second TRect
    /// @return     Returns true if rects are equal and false otherwise.
    /// $Source: vfc_trect.hpp $
    /// @author gaj2kor
    /// @ingroup vfc_group_core_types
    //---------------------------------------------------------------------                    
    template<class ValueType>
    bool operator==(const TRect<ValueType>& f_lhs, const TRect<ValueType>& f_rhs);

    //---------------------------------------------------------------------
    /// returns true if any of the four values (top, left, width, height) of
    /// one of the rectangles is not equal to the corresponding value in
    /// the other rectangle.
    /// @par Reference (CS-CRM / DOORS ID / EA):
    /// @param   f_op1    Object of first TRect
    /// @param   f_op2    Object of second TRect
    /// @return     Returns true if rects are NOT equal and false otherwise.
    /// $Source: vfc_trect.hpp $
    /// @author gaj2kor
    /// @ingroup vfc_group_core_types
    //---------------------------------------------------------------------                    
    template<class ValueType>
    bool operator!=(const TRect<ValueType>& f_lhs, const TRect<ValueType>& f_rhs);

}    // namespace vfc closed

#include "vfc/core/vfc_trect.inl"

#endif //VFC_TRECT_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_trect.hpp  $
//  Revision 1.4 2014/09/24 13:46:42MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_trect: wrong indendation (mantis0004729)
//  Revision 1.3 2014/08/13 09:34:10MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - CRect and TRect:: isEmpty does not work correct on non-normalized rects (mantis0004644)
//  Revision 1.2 2012/12/18 08:27:32MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.1 2012/10/23 10:27:46MESZ Sudhakar Nannapaneni (RBEI/ESD1) (SNU5KOR) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/core/core.pj
//=============================================================================

