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
*     $Source: vfc_image_ranaccess.inl $
*     $Revision: 1.9 $
*     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
*     $Date: 2012/12/18 08:27:33MEZ $
*     $Locker:  $
*     $Name:  $
*     $State: in_work $
*/
/*******************************************************************/

template <class PixelType, vfc::int32_t MaxRowValue> template <class ImageType>
void vfc::TRandomPixelAccess<PixelType,MaxRowValue>::init (ImageType& image)
{    
    // cache image height
    const int32_t height = image.getHeight();
    
    // check bounds
    VFC_ASSERT(height <= MAX_ROWS);
    
    // grab first row
    int32_t    rowiter = 0;
    pointer imgiter = image.getData();
    
    // fill row array with rowstarts
    for (rowiter=0; rowiter < height; ++rowiter, imgiter += image.getStride())
    {
        m_rows[rowiter] = imgiter;
    }
    
    // reset remaining rows
    vfc::contiguous_fill_zero_n(&(m_rows[height]),MAX_ROWS-height);
}

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_image_ranaccess.inl  $
Revision 1.9 2012/12/18 08:27:33MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
- Replace tabs by 4 spaces (mantis 4199)
Revision 1.8 2007/10/31 14:22:03MEZ vmr1kor 
Instead of using concatenenation of two for-loops, changed to use vfc::contiguous_fill_zero_n
Mantis Id:- 0001716
Revision 1.7 2007/04/02 13:08:30IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- removed statement (mantis1527)
Revision 1.6 2006/11/16 14:41:19CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.5 2005/10/06 17:04:59CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/image/image.pj
Revision 1.4 2005/09/13 11:11:40CEST zvh2hi 
-added TImage<T> class (manages own memory)
Revision 1.3 2005/09/13 09:25:05CEST zvh2hi 
-renamed TImage<T> to TImageView<T>
-changed algorithm interface from TImage<T> to generic ImageType
Revision 1.2 2005/09/12 09:54:18CEST zvh2hi 
-renamed several funcs
********************************************************************/
