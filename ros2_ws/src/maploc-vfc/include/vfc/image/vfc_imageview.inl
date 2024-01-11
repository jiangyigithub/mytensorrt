//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2006 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy, use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/image
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
///     $Source: vfc_imageview.inl $
///     $Revision: 1.19 $
///     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
///     $Date: 2016/01/14 11:17:32MEZ $
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

#include "vfc/core/vfc_assert.hpp"

template <class PixelType> inline
vfc::TImageView<PixelType>::TImageView(void)
: m_data_p(0), m_width(0), m_height(0), m_stride(0)
{
    // do nothing
}

template <class PixelType> inline
vfc::TImageView<PixelType>::TImageView(pointer f_data_p, int32_t f_width, int32_t f_height, int32_t f_stride)
: m_data_p(f_data_p), m_width (f_width), m_height (f_height), m_stride(f_stride)
{
    VFC_REQUIRE(0 <= f_width);
    VFC_REQUIRE(0 <= f_height);
    VFC_REQUIRE(f_width <= f_stride);
}

template <class PixelType> inline
vfc::TImageView<PixelType>::TImageView(pointer f_data_p, int32_t f_width, int32_t f_height)
: m_data_p(f_data_p), m_width (f_width), m_height (f_height), m_stride(f_width)
{
    VFC_REQUIRE(0 <= f_width);
    VFC_REQUIRE(0 <= f_height);
}

// PRQA S 2180 ++
template <class PixelType>
template <class OtherValueType> inline
vfc::TImageView<PixelType>::TImageView(const TImageView<OtherValueType>& f_other)
:   m_data_p(f_other.getData()),
    m_width(f_other.getWidth()),
    m_height(f_other.getHeight()),
    m_stride(f_other.getStride())
{

}
// PRQA S 2180 --

template <class PixelType>
template <class OtherValueType>
inline
vfc::TImageView<PixelType>&
vfc::TImageView<PixelType>::operator=(const TImageView<OtherValueType>& f_other)
{
    m_data_p = f_other.getData();
    m_width = f_other.getWidth();
    m_height = f_other.getHeight();
    m_stride = f_other.getStride();
    return *this ;
}


template <class PixelType> inline
void
vfc::TImageView<PixelType>::set(pointer f_data_p, int32_t f_width, int32_t f_height, int32_t f_stride)
{
    VFC_REQUIRE(0 <= f_width);
    VFC_REQUIRE(0 <= f_height);
    VFC_REQUIRE(f_width <= f_stride);

    m_data_p= f_data_p;
    m_width = f_width;
    m_height= f_height;
    m_stride= f_stride;
}


template <class PixelType> inline
typename vfc::TImageView<PixelType>::reference
vfc::TImageView<PixelType>::operator() (int32_t f_col, int32_t f_row) const
{
    VFC_REQUIRE(0 <= f_col);
    VFC_REQUIRE(0 <= f_row);
    VFC_REQUIRE(f_col < m_width);
    VFC_REQUIRE(f_row < m_height);

    // ImageViews have been designed to have pointer semantics because they are only references to image data. 
    // Like pointers there are 4 combinations to express pointer/data constness:
    //
    // T * - non-const pointer to non-const data
    // T const * - non-const pointer to const data
    // T * const - const pointer to non-const data
    // T const * const - const pointer to const data
    //
    // this maps to imageviews:
    // TImageView<T>
    // TImageView<const T>
    // const TImageView<T>
    // const TImageView<const T>
    //
    // Please use the template argument to express const-ness of image data. This is similar to STL iterator 
    // and const_iterator but we use only one class declaration for this. 
    //
    // The QAC++ warning can be safely suppressed here:

    return m_data_p[f_row*m_stride+f_col];
}

// object creation
//! returns an image container with given type and metrics
template <class T>    inline
vfc::TImageView<T>    vfc::makeImageView (T* f_data_p, int32_t f_width, int32_t f_height, int32_t f_stride)
{
    return TImageView<T>( f_data_p, f_width, f_height, f_stride);
}

//! returns an image container with given type and metrics
template <class T>    inline
vfc::TImageView<T>    vfc::makeImageView (T* f_data_p, int32_t f_width, int32_t f_height)
{
    return TImageView<T>( f_data_p, f_width, f_height, f_width);
}

//! returns reference to subimage - out of bounds conditions are NOT checked
template <class T>    inline
vfc::TImageView<T>    vfc::makeSubImageView    (    const TImageView<T>& f_image,
                                                    int32_t f_xstart, int32_t f_ystart,
                                                    int32_t f_width, int32_t f_height)
{
    VFC_REQUIRE(0 <= f_xstart);
    VFC_REQUIRE(0 <= f_ystart);
    VFC_REQUIRE(f_image.getWidth() >= (f_xstart+f_width));
    VFC_REQUIRE(f_image.getHeight() >= (f_ystart+f_height));

    return TImageView<T>(   f_image.getData()+f_ystart*f_image.getStride()+f_xstart,
                            f_width,
                            f_height,
                            f_image.getStride());
}

//! returns reference to subimage
template <class T>    inline
vfc::TImageView<T>    vfc::makeSubImageView    (    const TImageView<T>& f_image,
                                                    const vfc::CRect& f_rect)
{
    return vfc::makeSubImageView(f_image, f_rect.left(), f_rect.top(), f_rect.width(), f_rect.height());
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_imageview.inl  $
//  Revision 1.19 2016/01/14 11:17:32MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - add version of vfc::makeImageView() with CRect parameter (mantis0002707)
//  Revision 1.18 2013/09/19 14:34:48MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc::TImageView<PixelType>::operator() const returns non-const reference (mantis0004268)
//  Revision 1.17 2009/02/04 13:19:01MEZ Gaurav Jain (RBEI/ESD4) (gaj2kor) 
//  -Removal of QAC++ warnings.
//  (Mantis : 0002486)
//  Revision 1.16 2008/09/11 17:31:33IST Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  Replaced combined assertions by single ones.
//  (Mantis : 2052)
//  Revision 1.15 2007/11/26 20:49:41IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - removed dependencies to TPixelTraits (mantis1969)
//  Revision 1.14 2007/10/31 13:26:33CET vmr1kor
//  copy ctor and assop added
//  Mantis Id:- 0001709
//  Revision 1.13 2007/10/04 12:42:35IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added inline declarators (mantis1804)
//  Revision 1.12 2007/06/22 15:28:10CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - corrected redeclared function parameter names (mantis 1695)
//  - added _p suffix to pointer types (mantsi 1695)
//  Revision 1.11 2007/02/19 13:32:44CET Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - removed #include of vfc_except.hpp (mantis 1444)
//  Revision 1.10 2007/02/19 10:45:55CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - TImageView and TImage refactoring (mantis1432)
//  - adaption of image submodule to new error-handling (mantis1431)
//  Revision 1.9 2006/11/16 14:41:05CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.8 2006/10/26 10:00:02CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added assertions to TImageView::operator() (mantis1242)
//  - replaced header/footer
//  Revision 1.7 2006/01/26 12:35:28CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  -fixed error in c'tor implementation (mantis:986)
//  Revision 1.6 2005/10/28 10:32:10CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -changed directory of core files to vfc\include\core\...
//  Revision 1.5 2005/10/21 16:22:52CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -fixed bug in subimage generation
//  Revision 1.4 2005/10/12 13:16:31CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -fixed type in makeSubImageView
//  Revision 1.3 2005/10/12 10:36:04CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -changed subimage creation
//  Revision 1.2 2005/10/06 17:02:39CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/image/image.pj
//  Revision 1.1 2005/10/06 09:49:48CEST zvh2hi
//  Initial revision
//  Member added to project /import/mks/data/projects/cv/vfc/include/vfc/image/image.pj
//=============================================================================
