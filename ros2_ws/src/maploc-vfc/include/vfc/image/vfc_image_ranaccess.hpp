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
*     $Source: vfc_image_ranaccess.hpp $
*     $Revision: 1.11 $
*     $Author: gaj2kor $
*     $Date: 2009/02/02 08:53:04MEZ $
*     $Locker:  $
*     $Name: 0032 RC1  $
*     $State: in_work $
*/
/*******************************************************************/

#ifndef VFC_IMAGE_RANACCESS_HPP_INCLUDED
#define VFC_IMAGE_RANACCESS_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"        // int32_t
#include "vfc/core/vfc_algorithm.hpp"    // contiguous_fill_zero_n()
#include "vfc/image/vfc_image.hpp"    // TImageView<T>


namespace vfc
{    // namespace vfc opened

    //=========================================================================
    //  TRandomPixelAccess<>
    //-------------------------------------------------------------------------
    /// Image Adapter for fast random pixel access.
    /// @param PixelType type of pixel values.
    /// @param MaxRowValue maximum size of internal rowstart pointer array.
    /// @ingroup vfc_group_image_types
    //=========================================================================

    template <class PixelType, vfc::int32_t MaxRowValue = 512>
    class TRandomPixelAccess
    {
    public:
        enum { MAX_ROWS = MaxRowValue};

        typedef PixelType            value_type;
        typedef PixelType*            pointer;
        typedef const PixelType*    const_pointer;
        typedef PixelType&            reference;
        typedef const PixelType&    const_reference;

        TRandomPixelAccess(void)                { contiguous_fill_zero_n(m_rows,MAX_ROWS);}

        // suppress warning 4054 here as it is not possible to initialise a c-array in the ctor's init list
        // PRQA S 4054 ++
        template <class ImageType>
        explicit
        TRandomPixelAccess(ImageType& image)    { init(image);}
        // PRQA S 4054 --

        reference        operator()(int32_t col, int32_t row)        { return m_rows[row][col];}
        const_reference    operator()(int32_t col, int32_t row) const    { return m_rows[row][col];}

        template <class ImageType>
        void init (ImageType& image);

    private:
        pointer    m_rows[MAX_ROWS];
    };

    //-------------------------------------------------------------------------
    //! convenience function for TRandomPixelAccess creation.
    //! @relates TRandomPixelAccess
    //-------------------------------------------------------------------------

    template <class T>    inline
    TRandomPixelAccess<T>    makeRandomPixelAccess (TImageView<T>& img)
    {
        return TRandomPixelAccess<T>(img);
    }

}    // namespace vfc closed

#include "vfc/image/vfc_image_ranaccess.inl"

#endif //VFC_IMAGE_RANACCESS_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_image_ranaccess.hpp  $
Revision 1.11 2009/02/02 08:53:04MEZ gaj2kor 
-Addtion of comment related to the QAC++ msg.
(Mantis : 0002487)
Revision 1.10 2009/02/02 11:38:10IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
-Removal of QAC++ warnings.
(Mantis : 0002487)
Revision 1.9 2007/07/23 17:20:22IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
- doxygen grouping (mantis1744)
Revision 1.8 2007/07/23 09:52:26CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
- doxygen grouping (mantis1744)
Revision 1.7 2006/11/16 14:41:13CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
- replaced tabs with 4 spaces (mantis1294)
Revision 1.6 2006/04/10 15:36:58CEST DIN1LR
-added vfc namespace name to resolve compilation error in rational realtime (mantis1051)
Revision 1.5 2005/10/28 10:32:08CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
-changed directory of core files to vfc\include\core\...
Revision 1.4 2005/10/06 17:05:28CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/image/image.pj
Revision 1.3 2005/09/13 09:25:05CEST zvh2hi
-renamed TImage<T> to TImageView<T>
-changed algorithm interface from TImage<T> to generic ImageType
Revision 1.2 2005/09/12 09:54:18CEST zvh2hi
-renamed several funcs
********************************************************************/
