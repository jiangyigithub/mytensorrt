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
//       Projectname: vfc/image
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
///     $Source: vfc_image.inl $
///     $Revision: 1.21 $
///     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
///     $Date: 2016/11/28 09:12:16MEZ $
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


#include "vfc/core/vfc_algorithm.hpp"    // contiguous_copy_n()
#include "vfc/core/vfc_assert.hpp"        // VFC_ASSERT()

#include "vfc/image/vfc_image_traits.hpp"   // used for TImageRawIteratorTraits

namespace vfc
{   // namespace vfc opened

    //! @cond VFC_DOXY_SPECIALIZATIONS
        //========================================================================
        //  TImageRawIteratorTraits
        //-------------------------------------------------------------------------
        //! Specialization for const TImage<>.
        //! $Source: vfc_image.inl $
        //! @author zvh2hi
        //! @ingroup vfc_group_image_types
        //========================================================================
        template <class PixelType, class AllocatorType> 
        struct TImageRawIteratorTraits< const TImage<PixelType, AllocatorType> >
        {
            typedef typename TImage<PixelType,AllocatorType>::const_raw_iterator raw_iterator;
        };
    //! @endcond

}   // namespace vfc closed

template <class PixelType, class AllocatorType> inline
vfc::TImage<PixelType,AllocatorType>::TImage(void)
:   m_alloc(), m_view(), m_capacity(0)   
{
    // do nothing
}

template <class PixelType, class AllocatorType> inline
vfc::TImage<PixelType,AllocatorType>::TImage(const alloc_type& f_alloc)
:   m_alloc(f_alloc), m_view(), m_capacity(0)   
{
    // do nothing
}

template <class PixelType, class AllocatorType> inline
vfc::TImage<PixelType,AllocatorType>::TImage(int32_t f_width, int32_t f_height) 
:   m_alloc(), m_view(), m_capacity(0)   
{ 
    resize( f_width, f_height);
}

template <class PixelType, class AllocatorType> inline
vfc::TImage<PixelType,AllocatorType>::TImage(int32_t f_width, int32_t f_height, const alloc_type& f_alloc) 
:   m_alloc(f_alloc), m_view(), m_capacity(0)   
{ 
    resize( f_width, f_height);
}

template <class PixelType, class AllocatorType> inline
vfc::TImage<PixelType,AllocatorType>::TImage(int32_t f_width, int32_t f_height, int32_t f_stride) 
:   m_alloc(), m_view(), m_capacity(0)   
{ 
    resize( f_width, f_height, f_stride);
}

template <class PixelType, class AllocatorType> inline
vfc::TImage<PixelType,AllocatorType>::TImage(int32_t f_width, int32_t f_height, int32_t f_stride, const alloc_type& f_alloc) 
:   m_alloc(f_alloc), m_view(), m_capacity(0)   
{ 
    resize( f_width, f_height, f_stride);
}


template <class PixelType, class AllocatorType>    inline
vfc::TImage<PixelType,AllocatorType>::TImage (const TImage<PixelType,AllocatorType>& f_other) 
:   m_alloc(), m_view(), m_capacity(0)
{    
    resize_and_copy(f_other.m_view);
}

template <class PixelType, class AllocatorType>    inline
vfc::TImage<PixelType,AllocatorType>::TImage (const TImageView<PixelType>& f_view) 
:   m_alloc(), m_view(), m_capacity(0)
{  
    // should be safe
    resize_and_copy(f_view);
}

template <class PixelType, class AllocatorType>    inline
const vfc::TImage<PixelType,AllocatorType>& 
vfc::TImage<PixelType,AllocatorType>::operator=(const TImage<PixelType,AllocatorType>& f_other)
{
    if (&f_other != this)
    {
        resize_and_copy(f_other.m_view);
    }
    return *this;
}

template <class PixelType, class AllocatorType>    inline
const vfc::TImage<PixelType,AllocatorType>& 
vfc::TImage<PixelType,AllocatorType>::operator=(const TImageView<PixelType>& f_view)
{
    // create temporary to be safe if specified view references into this image
    TImage  tmp(f_view);
    resize_and_copy(tmp.getView());
    return *this;
}

template <class PixelType, class AllocatorType>
vfc::TImage<PixelType,AllocatorType>::~TImage(void) 
{ 
    // destroy and deallocate if data has been allocated previously

    const pointer data_begin = this->getData();
    if ( 0 != data_begin )
    {
        const int32_t oldCapacity = this->getCapacity(); 
        const pointer data_end    = this->end_raw();

        destroy(data_begin, data_end, hasTrivialDTor_t()); 
        m_alloc.deallocate(data_begin, oldCapacity);
    }
}

template <class PixelType, class AllocatorType>
bool    vfc::TImage<PixelType,AllocatorType>::resize_and_copy (const view_type& f_view)
{
    if ( resize( f_view.getWidth(), f_view.getHeight(), f_view.getStride()) )
    {
        const int32_t newSize = this->getSize();
        const pointer data    = this->getData();

        contiguous_copy_n( f_view.getData(), data, newSize);
        return true;
    }
    else
    {
        return false;
    }
}

template <class PixelType, class AllocatorType>
bool vfc::TImage<PixelType,AllocatorType>::resize(int32_t f_width, int32_t f_height, int32_t f_stride)
{
    VFC_REQUIRE( (0<=f_width) && (0<=f_height) && (f_width<=f_stride) );

    int32_t newSize     = f_height * f_stride;
    
    int32_t oldSize     = this->getSize(); 
    int32_t oldCapacity = this->getCapacity();
    pointer data        = this->getData();
    
    bool success = false;
        
    if (newSize <= getCapacity())
    {
        if (newSize <= oldSize)
        {   // destroy not needed objects
            
            // set image to smaller dimensions first
            m_view.set( data, f_width, f_height, f_stride);

            // now destroy unneeded content
            destroy     (data+newSize, data+oldSize, hasTrivialDTor_t());
        }
        else
        {    // construct new objects
            construct   (data+oldSize, data+newSize, hasTrivialCTor_t());

            // construction succesfull, set new image dimensions
            m_view.set( data, f_width, f_height, f_stride);
        }

        success = true;
    }
    else
    {
        // for smaller memory footprint: destroy first, allocate second
        // but current image is lost if allocation throws an exception

        // first reset image container state
        m_view.set(0,0,0,0);
        m_capacity = 0;

        // destroy and deallocate if data has been allocated previously
        if (0 != data)
        {    
            // destroy old objects
            destroy(data, data+oldSize, hasTrivialDTor_t());

            // free old memory
            m_alloc.deallocate(data, oldCapacity);
        }
                
        // check allocator capacity
        if ( m_alloc.max_size() >= static_cast<typename AllocatorType::size_type>(newSize) )
        {
            data    = m_alloc.allocate(newSize);
     
            // check if allocations was succesfull
            if (0 != data)
            {   
                // construct new objects
                construct(data, data+newSize, hasTrivialCTor_t());

                // update state
                m_capacity    = newSize;
                m_view.set(data, f_width, f_height, f_stride);

                success = true;
            }
        }
    }

    VFC_ENSURE (    ((0==getData()) && (0==getWidth()) && (0==getHeight()) && (0==getStride())) \
                ||  ((0!=getData()) && (f_width==getWidth()) && (f_height==getHeight()) && (f_stride == getStride())));

    return success;
}

template <class PixelType, class AllocatorType> inline
void    vfc::TImage<PixelType,AllocatorType>::destroy        (pointer , pointer , true_t) 
{ 
    // do nothing - fundamental type
}

template <class PixelType, class AllocatorType> inline
void    vfc::TImage<PixelType,AllocatorType>::destroy        (pointer begin, pointer end, false_t) 
{ 
    for (;begin!=end;++begin) 
    {
        m_alloc.destroy(begin);
    }
}

template <class PixelType, class AllocatorType> inline
void    vfc::TImage<PixelType,AllocatorType>::construct    (pointer , pointer , true_t) 
{ 
    // do nothing - fundamental type
}

template <class PixelType, class AllocatorType> inline
void    vfc::TImage<PixelType,AllocatorType>::construct    (pointer begin, pointer end, false_t) 
{ 
    for (;begin!=end;++begin) 
    {
        m_alloc.construct(begin, PixelType());
    }
}

template <class PixelType, class AllocatorType>
void vfc::TImage<PixelType,AllocatorType>::swap (TImage<PixelType, AllocatorType>& f_other)
{
    // do we have the same allocator strategy here?
    if (this->m_alloc == f_other.m_alloc)
    {
        // we do, so just do a shallow swap by swapping control information
        view_type tmpView(m_view);
        m_view = f_other.m_view;
        f_other.m_view = tmpView;

        vfc::int32_t tmpCapacity(m_capacity);
        m_capacity = f_other.m_capacity;
        f_other.m_capacity = tmpCapacity;
    }
    else
    {
        // we don't, so we need a deep swap
        TImage<PixelType, AllocatorType> tmpImage(*this);
        *this = f_other;
        f_other = tmpImage;
    }
}

template <class PixelType, class AllocatorType> inline
void vfc::swap ( vfc::TImage<PixelType,AllocatorType>& f_obj1, vfc::TImage<PixelType,AllocatorType>& f_obj2)
{
    f_obj1.swap(f_obj2);
}

template <class PixelType, class AllocatorType> inline
vfc::TImageView<const PixelType> vfc::makeImageView (const TImage<PixelType, AllocatorType>& f_image)
{
    return f_image.getView();
}


template <class PixelType, class AllocatorType> inline
vfc::TImageView<PixelType>       vfc::makeImageView (TImage<PixelType, AllocatorType>& f_image)
{
    return f_image.getView();
}


template <class PixelType, class AllocatorType> inline
vfc::TImageView<const PixelType> vfc::makeSubImageView    
(   const TImage<PixelType, AllocatorType>& f_image, 
    int32_t f_xstart, int32_t f_ystart, 
    int32_t f_width, int32_t f_height
)
{
    VFC_REQUIRE ( (0 <= f_xstart) && (0 <= f_ystart));
    VFC_REQUIRE ( (f_image.getWidth() >= (f_xstart+f_width)) && (f_image.getHeight() >= (f_ystart+f_height)));

    return TImageView<const PixelType>(     f_image.getData()+f_ystart*f_image.getStride()+f_xstart,
                                    f_width,
                                    f_height,
                                    f_image.getStride());
}


template <class PixelType, class AllocatorType> inline
vfc::TImageView<PixelType>       vfc::makeSubImageView   
(   TImage<PixelType, AllocatorType>& f_image, 
    int32_t f_xstart, int32_t f_ystart, 
    int32_t f_width, int32_t f_height
)
{
    VFC_REQUIRE ( (0 <= f_xstart) && (0 <= f_ystart));
    VFC_REQUIRE ( (f_image.getWidth() >= (f_xstart+f_width)) && (f_image.getHeight() >= (f_ystart+f_height)));


    const int32_t stride = f_image.getStride();
    const typename TImage<PixelType, AllocatorType>::pointer data = f_image.getData();

    return TImageView<PixelType>((data+(f_ystart*stride))+f_xstart,
                                  f_width,
                                  f_height,
                                  stride);
}


template <class PixelType, class AllocatorType> inline
vfc::TImageView<const PixelType> vfc::makeSubImageView    
(   const TImage<PixelType, AllocatorType>& f_image, 
    const vfc::CRect& f_rect
)
{
    return vfc::makeSubImageView(f_image, f_rect.left(), f_rect.top(), f_rect.width(), f_rect.height());
}


template <class PixelType, class AllocatorType> inline
vfc::TImageView<PixelType>       vfc::makeSubImageView   
(   TImage<PixelType, AllocatorType>& f_image, 
    const vfc::CRect& f_rect
)
{
    return vfc::makeSubImageView(f_image, f_rect.left(), f_rect.top(), f_rect.width(), f_rect.height());
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_image.inl  $
//  Revision 1.21 2016/11/28 09:12:16MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - rule violation in vfc_image: modification between sequence points (mantis0005381)
//  Revision 1.20 2016/01/14 11:17:32MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - add version of vfc::makeImageView() with CRect parameter (mantis0002707)
//  Revision 1.19 2013/05/14 09:45:46MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - rework TImage c'tors and memory layout (mantis4237)
//  Revision 1.18 2008/08/22 19:45:15MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added swap method/function (mantis 2296)
//  Revision 1.17 2008/02/14 10:52:21CET Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - fixed bug in TImageRawIteratorTraits (mantis2062)
//  Revision 1.16 2008/01/21 16:30:29CET Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - fixed bug in makeSubImageView and makeImageView (mantis2019)
//  Revision 1.15 2007/11/26 16:55:10CET Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - fixed constness bug (mantis1961)
//  Revision 1.14 2007/07/09 11:23:09CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - only call allocator's deallocate if memory has been allocted previously (mantis1735)
//  Revision 1.13 2007/02/23 11:29:46CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added missing typename (mantis1466)
//  Revision 1.12 2007/02/22 16:57:33CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - fixed warning (mantis1466)
//  Revision 1.11 2007/02/19 10:45:55CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - TImageView and TImage refactoring (mantis1432)
//  - adaption of image submodule to new error-handling (mantis1431)
//  Revision 1.10 2006/11/16 14:41:17CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.9 2006/10/24 11:47:14CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - fixed bug in TImage::construct() method (mantis1236)
//  Revision 1.8 2006/06/29 10:16:53CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -added one more missing "this->"
//  Revision 1.7 2005/12/06 15:36:21CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -fixed missing this->() for method calls of template base class (thx to ti comp and jat2hi)
//  Revision 1.6 2005/10/28 10:32:10CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -changed directory of core files to vfc\include\core\...
//  Revision 1.5 2005/10/13 10:02:33CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -removed wrong ctor default parameter
//  -added nop code for avoiding cw warnings
//  Revision 1.4 2005/10/06 17:03:34CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/image/image.pj
//  Revision 1.3 2005/09/30 15:27:56CEST zvh2hi 
//  -added type dependant construction/destruction
//  Revision 1.2 2005/09/13 17:14:50CEST zvh2hi 
//  -several changes
//  Revision 1.1 2005/09/13 11:11:08CEST zvh2hi 
//  Initial revision
//  Member added to project /import/mks/data/projects/cv/vfc/include/vfc/image/image.pj
//=============================================================================
