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
///     $Source: vfc_imageview.hpp $
///     $Revision: 1.18 $
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

#ifndef VFC_IMAGEVIEW_HPP_INCLUDED
#define VFC_IMAGEVIEW_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_rect.hpp"   // used for makeSubImageView with CRect parameter

namespace vfc
{    // namespace vfc opened

    //=========================================================================
    //  TImageView<>
    //-------------------------------------------------------------------------
    /// Image View class.
    /// @author zvh2hi
    /// @ingroup vfc_group_image_types
    //=========================================================================
    // PRQA S 2113 ++
    template <class PixelType>
    class  TImageView
    {
    public:

        // template typedef's
        typedef PixelType           value_type;

        typedef PixelType const *   const_pointer;
        typedef PixelType*          pointer;

        typedef PixelType const &   const_reference;
        typedef PixelType&          reference;

        typedef const_pointer       const_iterator;
        typedef pointer             iterator;

        typedef const_pointer       const_raw_iterator;
        typedef pointer             raw_iterator;

        typedef TImageView<PixelType>           view_type;
        typedef TImageView<const PixelType>     const_view_type;

    public:
        // c'tors

        /// default c'tor initalizes all elements with 0
        TImageView(void);

        /// stores a reference to an image data block with given image metrics (data isn't copied!)
        TImageView(pointer f_data_p, int32_t f_width, int32_t f_height, int32_t f_stride);

        /// stores a reference to an image data block with given image metrics (data isn't copied!)
        TImageView(pointer f_data_p, int32_t f_width, int32_t f_height);

        /// template conversion c'tor, works with compatible value_types only
        template <class OtherValueType>
        TImageView(const TImageView<OtherValueType>& f_other);

        /// template conversion c'tor, works with compatible value_types only
        template <class OtherValueType>
        TImageView& operator=(const TImageView<OtherValueType>& f_other);

        /// returns a reference to the element at a specified location in the image.
        /// \note image bounds are not checked!
        reference   operator() (int32_t f_col, int32_t f_row) const;

        /// returns image width in pixel
        int32_t     getWidth    (void)  const    {    return m_width;}
        /// returns image height in pixel
        int32_t     getHeight   (void)  const    {    return m_height;}
        /// returns the stride of the image in pixel (pixel offset from one row to the next)
        int32_t     getStride   (void)  const    {    return m_stride;}
        /// returns the raw size of the image (height*stride)
        int32_t     getSize     (void)  const    {    return getHeight()*getStride();}

        /// returns a pointer to the begin of the image data
        pointer     getData     (void)  const    {    return m_data_p;}

        /// returns a random-access iterator to the first pixel in the unpadded (raw) image.
        raw_iterator    begin_raw   (void)  const   {   return getData();}

        /// returns a random-access iterator that points just beyond the last pixel of the unpadded (raw) image.
        raw_iterator    end_raw     (void)  const   {   return getData()+getSize();}

        /// sets image view parameters
        void set(pointer f_data_p, int32_t f_width, int32_t f_height)   { set( f_data_p, f_width, f_height, f_width);}

        /// sets image view parameters
        void set(pointer f_data_p, int32_t f_width, int32_t f_height, int32_t f_stride);

    private:
        // attributes
        pointer     m_data_p;       ///< reference image data block
        int32_t     m_width,        ///< image width
                    m_height;       ///< image heigth
        int32_t     m_stride;       ///< actual row width (may differ from image width)
    };
    // PRQA S 2113 --

    // object creation

    //-------------------------------------------------------------------------
    //! returns an image view to a continguous block of pixeldata of specified
    //! type and metric.
    //! @ingroup vfc_group_image_types
    //-------------------------------------------------------------------------

    template <class T>
    TImageView<T>    makeImageView (T* f_data_p, int32_t f_width, int32_t f_height, int32_t f_stride);

    //-------------------------------------------------------------------------
    //! returns an image view to a continguous block of pixeldata of specified
    //! type and metric.
    //! @ingroup vfc_group_image_types
    //-------------------------------------------------------------------------

    template <class T>
    TImageView<T>    makeImageView (T* f_data_p, int32_t f_width, int32_t f_height);

    //-------------------------------------------------------------------------
    //! returns an image view to a subimage with specified position and extent.
    //! @relates TImageView
    //-------------------------------------------------------------------------
    template <class T>
    TImageView<T>    makeSubImageView   (   const TImageView<T>& f_image,
                                            int32_t f_xstart, int32_t f_ystart,
                                            int32_t f_width, int32_t f_height);
    
    //-------------------------------------------------------------------------
    //! returns an image view to a subimage with specified position and extent.
    //! @relates TImageView
    //-------------------------------------------------------------------------
    template <class T>
    TImageView<T>    makeSubImageView   (   const TImageView<T>& f_image,
                                            const vfc::CRect& f_rect);

}    // namespace vfc closed

#include "vfc/image/vfc_imageview.inl"

#endif //VFC_IMAGEVIEW_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_imageview.hpp  $
//  Revision 1.18 2016/01/14 11:17:32MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - add version of vfc::makeImageView() with CRect parameter (mantis0002707)
//  Revision 1.17 2009/02/04 13:19:01MEZ gaj2kor 
//  -Removal of QAC++ warnings.
//  (Mantis : 0002486)
//  Revision 1.16 2007/11/26 20:49:39IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - removed dependencies to TPixelTraits (mantis1969)
//  Revision 1.15 2007/10/31 13:26:31CET vmr1kor
//  copy ctor and assop added
//  Mantis Id:- 0001709
//  Revision 1.14 2007/10/13 14:28:38IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added view_type and const_view_type (mantis1589)
//  Revision 1.13 2007/07/23 13:50:22CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - doxygen grouping (mantis1744)
//  Revision 1.12 2007/07/23 09:53:47CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - doxygen grouping (mantis1744)
//  Revision 1.11 2007/06/22 15:28:10CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - corrected redeclared function parameter names (mantis 1695)
//  - added _p suffix to pointer types (mantsi 1695)
//  Revision 1.10 2007/02/19 10:45:55CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - TImageView and TImage refactoring (mantis1432)
//  - adaption of image submodule to new error-handling (mantis1431)
//  Revision 1.9 2006/11/16 14:41:17CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.8 2006/10/26 10:00:02CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added assertions to TImageView::operator() (mantis1242)
//  - replaced header/footer
//  Revision 1.7 2006/08/10 14:19:53CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  -added begin_raw(), end_raw() for STL compatibility (mantis1135)
//  Revision 1.6 2005/10/28 10:32:10CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -changed directory of core files to vfc\include\core\...
//  Revision 1.5 2005/10/12 10:36:03CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -changed subimage creation
//  Revision 1.4 2005/10/06 17:03:17CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/image/image.pj
//  Revision 1.3 2005/09/30 15:28:50CEST zvh2hi
//  -added construction functions
//  Revision 1.2 2005/09/13 17:14:50CEST zvh2hi
//  -several changes
//=============================================================================
