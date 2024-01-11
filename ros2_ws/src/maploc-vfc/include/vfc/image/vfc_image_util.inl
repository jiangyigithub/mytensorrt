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
*   Projectname: vfc
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
*     $Source: vfc_image_util.inl $
*     $Revision: 1.17 $
*     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
*     $Date: 2016/11/28 14:58:45MEZ $
*     $Locker:  $
*     $Name:  $
*     $State: in_work $
*/
/*******************************************************************/


#include "vfc/core/vfc_math.hpp" // isEqual

template <class ImageType>    inline  
void    vfc::zeroImage (ImageType& img)   
{
    const int32_t width  = img.getWidth();
    const int32_t stride = img.getStride();
    const int32_t size   = img.getSize();
    const typename ImageType::pointer data   = img.getData();

    // fast implementation for full images
    if (width == stride)
    {
        contiguous_fill_zero_n(data, size);
    }
    // implementation for subimages
    else
    {    
        typename ImageType::iterator             rowit_img  = data;
        const typename ImageType::const_iterator rowend_img = img.end_raw();
        
        while (rowit_img!=rowend_img)
        {
            contiguous_fill_zero_n(rowit_img, width);    

            rowit_img += stride;
        }
    }
}

template <class ImageType, class ValueType> inline 
void    vfc::fillImage (ImageType& img, const ValueType& value)   
{
    const int32_t width  = img.getWidth();
    const int32_t stride = img.getStride();
    const int32_t size   = img.getSize();
    const typename ImageType::pointer data = img.getData();

    // fast implementation for full images
    if (width == stride)
    {
        contiguous_fill_n(data, size, value);
    }
    // implementation for subimages
    else
    {    
        typename ImageType::iterator             rowit_img  = data;
        const typename ImageType::const_iterator rowend_img = img.end_raw();
        
        while(rowit_img!=rowend_img)
        {
            contiguous_fill_n(rowit_img, width, value);    
            
            rowit_img += stride;
        }
    }
}

template <class SrcImageType, class DestImageType>  inline 
void    vfc::copyImage  (const SrcImageType& src, DestImageType& dest)
{
    // check image dimensions
    VFC_REQUIRE (( src.getWidth() == dest.getWidth()) &&  (src.getHeight() == dest.getHeight()));

    // full image    
    if ((src.getWidth() == src.getStride()) && (dest.getWidth() == dest.getStride()))
    {
        contiguous_copy_n(src.getData(), dest.getData(), src.getHeight()*src.getStride());
    
    }
    // subimages
    else
    {    
        typename SrcImageType::const_iterator  rowit_src   = src.getData();
        typename SrcImageType::const_iterator  rowend_src  = src.getData()+src.getHeight()*src.getStride();
        
        typename DestImageType::iterator  rowit_dest = dest.getData();
        
        while(rowit_src!=rowend_src)
        {
            contiguous_copy_n(rowit_src, rowit_dest, src.getWidth());    

            rowit_src   += src.getStride();
            rowit_dest  += dest.getStride();
        }
    }   

}

template <class SrcImageType, class DestImageType>    inline    
void    vfc::moveImage    (const SrcImageType& src, DestImageType& dest)
{
    // check image dimensions
    VFC_REQUIRE ( (src.getWidth() == dest.getWidth()) &&  (src.getHeight() == dest.getHeight()));

    // full image    
    if ( (src.getWidth() == src.getStride()) && (dest.getWidth() == dest.getStride()))
    {
        contiguous_move_n(src.getData(),dest.getData(),src.getHeight()*src.getStride());
    
    }
    // subimages
    else
    {    
        typename SrcImageType::const_iterator  rowit_src   = src.getData();
        typename SrcImageType::const_iterator  rowend_src  = src.getData()+src.getHeight()*src.getStride();
        
        typename DestImageType::iterator  rowit_dest = dest.getData();
        
        while (rowit_src!=rowend_src)
        {
            contiguous_move_n(rowit_src, rowit_dest, src.getWidth());    

            rowit_src   += src.getStride();
            rowit_dest  += dest.getStride();
        }
    }   

}

template <class Image1Type, class Image2Type> inline
bool vfc::isImageDataEqual(const Image1Type& f_img1, const Image2Type& f_img2)
{
    // images of same size?
    if ((f_img1.getWidth() != f_img2.getWidth()) || (f_img1.getHeight() != f_img2.getHeight()))
    {
        return false; // exit 
    }

    typename Image1Type::const_iterator l_image1It  = f_img1.getData();
    typename Image1Type::const_iterator l_image1LineEnd = l_image1It + f_img1.getWidth();
    typename Image1Type::const_iterator l_image1End = l_image1It + f_img1.getHeight()*f_img1.getStride();
    typename Image2Type::const_iterator l_image2It  = f_img2.getData();
    
    const int32_t l_image1Offset = f_img1.getStride()-f_img1.getWidth();
    const int32_t l_image2Offset = f_img2.getStride()-f_img2.getWidth();

    // each line
    while (l_image1It != l_image1End)
    {
        // each pixel of line
        while (l_image1It != l_image1LineEnd)
        {
            if (!isEqual(*l_image1It, *l_image2It)) 
            {
                return false; // exit
            }
            ++l_image1It;
            ++l_image2It;
        }
        l_image1It += l_image1Offset;
        l_image2It += l_image2Offset;
        l_image1LineEnd = l_image1It + f_img1.getWidth();
    }
    return true; // exit
}

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_image_util.inl  $
Revision 1.17 2016/11/28 14:58:45MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
- rule violation in vfc_imageutil: modification between sequence points (mantis0005394)
Revision 1.16 2008/09/01 15:59:45MESZ Dhananjay N (RBEI/ESD1) (dhn1kor) 
Precedence confusion in QAC++ 2.5.Warning Rule 8.0.3 is removed.(mantis2218)
Revision 1.15 2008/08/25 18:11:40IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- added paranthesis (mantis 1710)
- replaced for-loops with while-loops (mantis 1710)
Revision 1.14 2007/11/23 17:01:41CET Muehlmann Karsten (CC-DA/ESV1) (muk2lr) 
- missing include (mantis1960)
Revision 1.13 2007/08/14 11:01:22CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
- replaced true/false return value check with VFC_REQUIRE (mantis1690)
Revision 1.12 2007/03/21 13:32:53CET Koenig Matthias (CR/AEM5) (KON3HI) 
MANTIS 0001137: added 'isImageDataEqual'
Revision 1.11 2006/11/16 14:41:19CET ZVH2HI 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.10 2006/11/08 08:37:04CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
- removed ";" (mantis 1267)
Revision 1.9 2005/10/06 17:04:17CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/image/image.pj
Revision 1.8 2005/09/30 15:21:07CEST zvh2hi 
-moved creation function
Revision 1.7 2005/09/13 17:14:49CEST zvh2hi 
-several changes
Revision 1.6 2005/09/13 11:11:41CEST zvh2hi 
-added TImage<T> class (manages own memory)
Revision 1.5 2005/09/13 09:25:06CEST zvh2hi 
-renamed TImage<T> to TImageView<T>
-changed algorithm interface from TImage<T> to generic ImageType
Revision 1.4 2005/09/12 13:56:47CEST zvh2hi 
-changed safe subimage creation
Revision 1.3 2005/09/12 09:54:19CEST zvh2hi 
-renamed several funcs
Revision 1.2 2005/09/12 09:39:04CEST zvh2hi 
-changed datatype from size_t to int32_t
********************************************************************/
