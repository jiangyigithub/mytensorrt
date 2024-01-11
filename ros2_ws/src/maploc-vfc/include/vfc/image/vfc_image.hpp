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
//       Projectname: ..\..\..\..\vfc\dummy\win32\vc_71\vfc_dummy
//          Synopsis: 
//  Target system(s): 
//       Compiler(s): VS7.10
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes: 
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: zvh2hi
//  Department: 
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_image.hpp $
///     $Revision: 1.22 $
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

#ifndef VFC_IMAGE_HPP_INCLUDED
#define VFC_IMAGE_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_metaprog.hpp"
#include "vfc/core/vfc_type_traits.hpp"
#include "vfc/core/vfc_rect.hpp"   // used for makeSubImageView with CRect parameter
#include "vfc/image/vfc_imageview.hpp"    // TImageView<T>
#include "vfc/memory/vfc_freestore_allocator.hpp"   // used for TFreeStoreAllocator

namespace vfc
{    // namespace vfc opened

    //=========================================================================
    //  TImage<>
    //-------------------------------------------------------------------------
    /// Image Container class.
    /// @par
    /// @image html image-image-01.png
    ///
    /// @author zvh2hi
    /// @ingroup vfc_group_image_types
    //=========================================================================

    template <class PixelType, class AllocatorType = TFreeStoreAllocator<PixelType> >
    class TImage 
    {
        typedef AllocatorType            alloc_type;
        
        typedef typename TInt2Boolean<THasTrivialCTor<PixelType>::value>::type    hasTrivialCTor_t;
        typedef typename TInt2Boolean<THasTrivialDTor<PixelType>::value>::type    hasTrivialDTor_t;

    public:

        typedef TImageView<PixelType>           view_type;
        typedef TImageView<const PixelType>     const_view_type;

        typedef typename view_type::value_type          value_type;
        typedef typename view_type::pointer             pointer;
        typedef typename view_type::const_pointer       const_pointer;
        typedef typename view_type::reference           reference;
        typedef typename view_type::const_reference     const_reference;
        typedef typename view_type::iterator            iterator;
        typedef typename view_type::const_iterator      const_iterator;
        typedef typename view_type::raw_iterator        raw_iterator;
        typedef typename view_type::const_raw_iterator  const_raw_iterator;
        
        //---------------------------------------------------------------------
        //! default c'tor, constructs image object with zero width, height and stride
        //---------------------------------------------------------------------
        TImage(void);

        //---------------------------------------------------------------------
        //! c'tor, constructs image object with zero width, height and stride
        //! Memory will be allocated via specified allocator object.
        //---------------------------------------------------------------------
        explicit TImage(const alloc_type& f_alloc);

        //---------------------------------------------------------------------
        //! C'tor, constructs image object with specified width and height (stride == width).
        //! @note
        //! Pixel values of POD types (more specific: types for which THasTrivialCTor 
        //! evalutes to true) will not be initialized for performance reasons 
        //! (so don't expect zeros for fundamental types!). 
        //! All other types will be initialized by their default c'tor.
        //---------------------------------------------------------------------
        TImage(int32_t f_width, int32_t f_height);

        //---------------------------------------------------------------------
        //! C'tor, constructs image object with specified width and height (stride == width).
        //! Memory is allocated via specified allocator object (optional parameter).
        //! @note
        //! Pixel values of POD types (more specific: types for which THasTrivialCTor 
        //! evalutes to true) will not be initialized for performance reasons 
        //! (so don't expect zeros for fundamental types!). 
        //! All other types will be initialized by their default c'tor.
        //---------------------------------------------------------------------
        TImage(int32_t f_width, int32_t f_height, const alloc_type& f_alloc);

        //---------------------------------------------------------------------
        //! C'tor, constructs image object with specified width, height and stride.
        //! @note
        //! Pixel values of POD types (more specific: types for which THasTrivialCTor 
        //! evalutes to true) will not be initialized for performance reasons 
        //! (so don't expect zeros for fundamental types!). 
        //! All other types will be initialized by their default c'tor.
        //---------------------------------------------------------------------
        TImage(int32_t f_width, int32_t f_height, int32_t f_stride);

        //---------------------------------------------------------------------
        //! C'tor, constructs image object with specified width, height and stride.
        //! Memory is allocated via specified allocator object.
        //! @note
        //! Pixel values of POD types (more specific: types for which THasTrivialCTor 
        //! evalutes to true) will not be initialized for performance reasons 
        //! (so don't expect zeros for fundamental types!). 
        //! All other types will be initialized by their default c'tor.
        //---------------------------------------------------------------------
        TImage(int32_t f_width, int32_t f_height, int32_t f_stride, const alloc_type& f_alloc);

    
        //! copy c'tor.
        TImage (const TImage<PixelType,AllocatorType>& f_other);

        //! conversion c'tor.
        explicit
        TImage (const TImageView<PixelType>& f_view);

        //! d'tor - frees memory.
        ~TImage(void);

        //! assignment op.
        const TImage<PixelType,AllocatorType>& operator=(const TImage<PixelType,AllocatorType>& f_other);

        //! assignment op.
        const TImage<PixelType,AllocatorType>& operator=(const TImageView<PixelType>& f_view);

        //! returns current image capacity.
        int32_t    getCapacity() const    {    return m_capacity;}

        //! resizes image, allocates new memory if neccessary.
        bool resize (int32_t f_width, int32_t f_height, int32_t f_stride);

        //! resizes image, allocates memory via allocator object if neccessary.
        bool resize (int32_t f_width, int32_t f_height) { return resize(f_width,f_height,f_width);}

        /// returns a reference to the element at a specified location in the image. 
        /// \note image bounds are not checked!
        const_reference operator()(int32_t f_col, int32_t f_row) const { return m_view(f_col,f_row);}

        /// returns a reference to the element at a specified location in the image. 
        /// \note image bounds are not checked!
        reference       operator()(int32_t f_col, int32_t f_row) { return m_view(f_col,f_row);}

        /// returns image width in pixel.
        int32_t     getWidth    (void)  const    {    return m_view.getWidth();}
        
        /// returns image height in pixel.
        int32_t     getHeight   (void)  const    {    return m_view.getHeight();}
        
        /// returns the stride of the image in pixel (pixel offset from one row to the next).
        int32_t     getStride   (void)  const    {    return m_view.getStride();}
        
        /// returns the raw size of the image (height*stride).
        int32_t     getSize     (void)  const    {    return m_view.getSize();}

        /// returns a pointer to the begin of the image data .
        const_pointer   getData     (void)  const     {    return m_view.getData();}

        /// returns a pointer to the begin of the image data .
        pointer         getData     (void)            {    return m_view.getData();}

        /// returns a random-access iterator to the first pixel in the unpadded (raw) image.
        const_raw_iterator  begin_raw   (void)  const   {   return m_view.begin_raw();}

         /// returns a random-access iterator to the first pixel in the unpadded (raw) image.
        raw_iterator        begin_raw   (void)          {   return m_view.begin_raw();}
        
        /// returns a random-access iterator that points just beyond the last pixel of the unpadded (raw) image.
        const_raw_iterator  end_raw     (void)  const   {   return m_view.end_raw();}
      
        /// returns a random-access iterator that points just beyond the last pixel of the unpadded (raw) image.
        raw_iterator        end_raw     (void)          {   return m_view.end_raw();}

        view_type           getView     (void)          {   return m_view;}
        const_view_type     getView     (void) const    {   return m_view;}

        void swap (TImage<PixelType, AllocatorType>& f_other);

    private:
        
        //! calls allocator::destroy() for each object in sequence [begin,end[.
        void    destroy        (pointer , pointer , true_t);
        void    destroy        (pointer begin, pointer end, false_t);
        
        //! calls allocator::construct for each object in sequence [begin,end[.
        void    construct    (pointer , pointer , true_t);
        void    construct    (pointer begin, pointer end, false_t);

        bool    resize_and_copy (const view_type& f_view);

        // attributes   
        alloc_type  m_alloc;
        view_type   m_view;
        int32_t     m_capacity;
    };

    //-------------------------------------------------------------------------
    //! swaps specified images.
    //! @relates TImage
    //-------------------------------------------------------------------------
    template <class PixelType, class AllocatorType>
    void swap ( TImage<PixelType,AllocatorType>& f_obj1, TImage<PixelType,AllocatorType>& f_obj2);

    // image view creation

    //-------------------------------------------------------------------------
    //! returns an image view to const pixel data of specified image.
    //! @relates TImage
    //-------------------------------------------------------------------------
    
    template <class PixelType, class AllocatorType>
    TImageView<const PixelType> makeImageView (const TImage<PixelType, AllocatorType>& f_image);

    //-------------------------------------------------------------------------
    //! returns an image view to mutable pixel data of specified image.
    //! @relates TImage
    //-------------------------------------------------------------------------

    template <class PixelType, class AllocatorType>
    TImageView<PixelType>       makeImageView (TImage<PixelType, AllocatorType>& f_image);

    //-------------------------------------------------------------------------
    //! returns an image view to a const subimage with specified position and 
    //! extent.
    //! @relates TImage
    //-------------------------------------------------------------------------
    template <class PixelType, class AllocatorType>
    TImageView<const PixelType> makeSubImageView    
        (   const TImage<PixelType, AllocatorType>& f_image, 
            int32_t f_xstart, int32_t f_ystart, 
            int32_t f_width, int32_t f_height);

    //-------------------------------------------------------------------------
    //! returns an image view to a mutable subimage with specified position and 
    //! extent.
    //! @relates TImage
    //-------------------------------------------------------------------------
    template <class PixelType, class AllocatorType>
    TImageView<PixelType>       makeSubImageView   
        (   TImage<PixelType, AllocatorType>& f_image, 
            int32_t f_xstart, int32_t f_ystart, 
            int32_t f_width, int32_t f_height);

    //-------------------------------------------------------------------------
    //! returns an image view to a const subimage with specified position and 
    //! extent from CRect parameter
    //! @relates TImage
    //-------------------------------------------------------------------------
    template <class PixelType, class AllocatorType>
    TImageView<const PixelType> makeSubImageView    
        (   const TImage<PixelType, AllocatorType>& f_image, 
            const vfc::CRect& f_rect);

    //-------------------------------------------------------------------------
    //! returns an image view to a mutable subimage with specified position and 
    //! extent from CRect parameter
    //! @relates TImage
    //-------------------------------------------------------------------------
    template <class PixelType, class AllocatorType>
    TImageView<PixelType>       makeSubImageView   
        (   TImage<PixelType, AllocatorType>& f_image, 
            const vfc::CRect& f_rect);


}    // namespace vfc closed

#include "vfc/image/vfc_image.inl"

#endif //VFC_IMAGE_HPP_INCLUDED



//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_image.hpp  $
//  Revision 1.22 2016/01/14 11:17:32MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - add version of vfc::makeImageView() with CRect parameter (mantis0002707)
//  Revision 1.21 2013/05/14 09:41:17MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - rework TImage c'tors and memory layout (mantis4237)
//  Revision 1.20 2008/08/22 19:45:15MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added swap method/function (mantis 2296)
//  Revision 1.19 2008/08/11 10:59:34CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added image to docs (mantis2182)
//  Revision 1.18 2008/01/21 16:30:28CET Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - fixed bug in makeSubImageView and makeImageView (mantis2019)
//  Revision 1.17 2007/10/31 13:57:58CET vmr1kor 
//  made one argument constructor as 'explicit'
//  Revision 1.16 2007/08/10 14:26:09IST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - fixed comments (mantis1760)
//  Revision 1.15 2007/07/23 13:55:32CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.14 2007/07/23 13:50:22CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.13 2007/07/23 09:53:24CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.12 2007/07/09 10:30:32CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - replaced old c templates with c++
//  Revision 1.11 2007/02/19 10:45:54CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - TImageView and TImage refactoring (mantis1432)
//  - adaption of image submodule to new error-handling (mantis1431)
//  Revision 1.10 2006/11/16 14:41:11CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.9 2005/10/28 10:32:09CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -changed directory of core files to vfc\include\core\...
//  Revision 1.8 2005/10/13 10:03:01CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added typedefs for avoiding cw errors
//  Revision 1.7 2005/10/06 17:03:52CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/image/image.pj
//  Revision 1.6 2005/09/30 15:26:58CEST zvh2hi 
//  -added type dependant element construction/destruction
//  Revision 1.5 2005/09/13 17:14:49CEST zvh2hi 
//  -several changes
//  Revision 1.4 2005/09/13 11:11:41CEST zvh2hi 
//  -added TImage<T> class (manages own memory)
//  Revision 1.3 2005/09/13 09:25:06CEST zvh2hi 
//  -renamed TImage<T> to TImageView<T>
//  -changed algorithm interface from TImage<T> to generic ImageType
//  Revision 1.2 2005/09/12 09:39:04CEST zvh2hi 
//  -changed datatype from size_t to int32_t
//=============================================================================
