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
*     $Source: vfc_image_util.hpp $
*     $Revision: 1.14 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
*     $Date: 2007/08/14 11:01:21MESZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

#ifndef VFC_IMAGE_UTIL_HPP_INCLUDED
#define VFC_IMAGE_UTIL_HPP_INCLUDED

#include "vfc/image/vfc_imageview.hpp"
#include "vfc/core/vfc_algorithm.hpp"

namespace vfc
{    // namespace vfc opened
    
    //-------------------------------------------------------------------------
    //! assigns zero to all pixels of specified image.
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <class ImageType>  
    void    zeroImage    (ImageType& img);

    //-------------------------------------------------------------------------
    //! assigns a new value to all pixels of specified image.
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <class ImageType, class ValueType>  
    void    fillImage    (ImageType& img, const ValueType& value);

    //-------------------------------------------------------------------------
    //! assigns the pixel values from the source image to the destination image.
    //! If the source and destination image overlap (e.g. views), the behavior 
    //! of copyImage is undefined. Use moveImage to handle overlapping regions.
    //! @pre specified images must have the same size.
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <class SrcImageType, class DestImageType>  
    void    copyImage    (const SrcImageType& src, DestImageType& dest);

    //-------------------------------------------------------------------------
    //! assigns the pixel values from the source image to the destination image.
    //! If some region of the source image and the destination image overlap, 
    //! moveImage ensures that the original elements in the overlapping 
    //! region are copied before being overwritten.
    //! @pre specified images must have the same size.
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <class SrcImageType, class DestImageType>    inline    
    void    moveImage    (const SrcImageType& src, DestImageType& dest);

    //-------------------------------------------------------------------------
    //! compares the pixel values (pixel by pixel) of image1 and image2
    //! @return true if images are equal, false otherwise
    //! @author kon3hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <class Image1Type, class Image2Type2>
    bool isImageDataEqual(const Image1Type& f_img1, const Image2Type2& f_img2);
    
}    // namespace vfc closed

#include "vfc/image/vfc_image_util.inl"

#endif //VFC_IMAGE_UTIL_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_image_util.hpp  $
Revision 1.14 2007/08/14 11:01:21MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
- replaced true/false return value check with VFC_REQUIRE (mantis1690)
Revision 1.13 2007/07/23 09:53:05CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
- doxygen grouping (mantis1744)
Revision 1.12 2007/03/21 13:32:53CET Koenig Matthias (CR/AEM5) (KON3HI) 
MANTIS 0001137: added 'isImageDataEqual'
Revision 1.11 2006/11/16 14:41:13CET ZVH2HI 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.10 2005/10/28 10:32:08CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
-changed directory of core files to vfc\include\core\...
Revision 1.9 2005/10/06 17:04:36CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/image/image.pj
Revision 1.8 2005/09/30 15:20:15CEST zvh2hi 
-moved creation functions
Revision 1.7 2005/09/13 17:14:48CEST zvh2hi 
-several changes
Revision 1.6 2005/09/13 11:11:40CEST zvh2hi 
-added TImage<T> class (manages own memory)
Revision 1.5 2005/09/13 09:25:06CEST zvh2hi 
-renamed TImage<T> to TImageView<T>
-changed algorithm interface from TImage<T> to generic ImageType
Revision 1.4 2005/09/12 13:56:46CEST zvh2hi 
-changed safe subimage creation
Revision 1.3 2005/09/12 09:54:19CEST zvh2hi 
-renamed several funcs
Revision 1.2 2005/09/12 09:39:03CEST zvh2hi 
-changed datatype from size_t to int32_t
********************************************************************/
