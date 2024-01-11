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
///     $Source: vfc_image_algorithm.hpp $
///     $Revision: 1.20 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/07/23 13:22:49MESZ $
///     $Locker:  $
///     $Name: 0032 RC1 Hello KW07  $
///     $State: In_Development $
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

#ifndef VFC_IMAGE_ALGORITHM_HPP_INCLUDED
#define VFC_IMAGE_ALGORITHM_HPP_INCLUDED

// vfc/core includes
#include "vfc/core/vfc_types.hpp"   // used for int32_t

namespace vfc
{    // namespace vfc opened

    ////////////////////////////////////////
    // for_each_pixel()
    ////////////////////////////////////////


    //-------------------------------------------------------------------------
    //! applies a specified function object to each pixel adress (pointer) of 
    //! specified image in a forward order (row-major).
    //! @par Implicit Functor Interface:
    //! @code
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() ( ptr1)
    //! @endcode
    //! @sa for_each_pixel
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <  class ImageType, 
                class FunctorType>
    void    for_each_pixel    ( ImageType& f_img, 
                                FunctorType& f_func);

    //-------------------------------------------------------------------------
    //! applies a specified function object to corresponding pixel 
    //! adresses (pointers) of specified images in a forward order (row-major).
    //! @par Implicit Functor Interface:
    //! @code
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() ( ptr1, ptr2)
    //! @endcode
    //! @sa for_each_pixel
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <  class Image1Type, class Image2Type, 
                class FunctorType>
    void    for_each_pixel    ( Image1Type& f_img1, Image2Type& f_img2, 
                                FunctorType& f_func);

    //-------------------------------------------------------------------------
    //! applies a specified function object to corresponding pixel 
    //! adresses (pointers) of specified images in a forward order (row-major).
    //! @par Implicit Functor Interface:
    //! @code
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() ( ptr1, ptr2, ptr3)
    //! @endcode
    //! @sa for_each_pixel
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <  class Image1Type, class Image2Type, class Image3Type, 
                class FunctorType>
    void    for_each_pixel    ( Image1Type& f_img1, Image2Type& f_img2, Image3Type& f_img3, 
                                FunctorType& f_func);

    //-------------------------------------------------------------------------
    //! applies a specified function object to corresponding pixel 
    //! adresses (pointers) of specified images in a forward order (row-major).
    //! @par Implicit Functor Interface:
    //! @code
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() ( ptr1, ptr2, ptr3, ptr4);
    //! @endcode
    //! @sa for_each_pixel
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <  class Image1Type, class Image2Type, class Image3Type, 
                class Image4Type, 
                class FunctorType>
    void    for_each_pixel    ( Image1Type& f_img1, Image2Type& f_img2, Image3Type& f_img3, 
                                Image4Type& f_img4, 
                                FunctorType& f_func);

    //-------------------------------------------------------------------------
    //! applies a specified function object to corresponding pixel 
    //! adresses (pointers) of specified images in a forward order (row-major).
    //! @par Implicit Functor Interface:
    //! @code
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() ( ptr1, ptr2, ptr3, ptr4, ptr5);
    //! @endcode
    //! @sa for_each_pixel
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <  class Image1Type, class Image2Type, class Image3Type, 
                class Image4Type, class Image5Type, 
                class FunctorType>
    void    for_each_pixel    ( Image1Type& f_img1, Image2Type& f_img2, Image3Type& f_img3, 
                                Image4Type& f_img4, Image5Type& f_img5, 
                                FunctorType& f_func);

    //-------------------------------------------------------------------------
    //! applies a specified function object to corresponding pixel 
    //! adresses (pointers) of specified images in a forward order (row-major).
    //! @par Implicit Functor Interface:
    //! @code
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() ( ptr1, ptr2, ptr3, ptr4, ptr5, ptr6);
    //! @endcode
    //! @sa for_each_pixel
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <  class Image1Type, class Image2Type, class Image3Type, 
                class Image4Type, class Image5Type, class Image6Type, 
                class FunctorType>
    void    for_each_pixel    ( Image1Type& f_img1, Image2Type& f_img2, Image3Type& f_img3, 
                                Image4Type& f_img4, Image5Type& f_img5, Image6Type& f_img6, 
                                FunctorType& f_func);

    ////////////////////////////////////////
    // for_each_pixel_backward()
    ////////////////////////////////////////

    //-------------------------------------------------------------------------
    //! applies a specified function object to each pixel adress (pointer) of 
    //! specified image in a backward order (row-major).
    //! @par Implicit Functor Interface:
    //! @code
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() ( ptr1)
    //! @endcode
    //! @sa for_each_pixel
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <  class ImageType, 
                class FunctorType>
    void    for_each_pixel_backward    (    ImageType& f_img, 
                                            FunctorType& f_func);
    //-------------------------------------------------------------------------
    //! applies a specified function object to corresponding pixel 
    //! adresses (pointers) of specified images in a backward order (row-major).
    //! @par Implicit Functor Interface:
    //! @code
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() (     ptr1, ptr2)
    //! @endcode
    //! @sa for_each_pixel
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <  class Image1Type, class Image2Type, 
                class FunctorType>
    void    for_each_pixel_backward    (    Image1Type& f_img1, Image2Type& f_img2, 
                                            FunctorType& f_func);

    ////////////////////////////////////////
    // for_each_nth_pixel()
    ////////////////////////////////////////

    //-------------------------------------------------------------------------
    //! applies a specified function object to each nth pixel adress (pointer),
    //! in a forward order (row major) by skipping SkipValue columns and rows.
    //! @par Implicit Functor Interface:
    //! @code
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() (     ptr1)
    //! @endcode
    //! @sa for_each_pixel
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <  int32_t SkipValue, 
                class ImageType, 
                class FunctorType>
    void    for_each_nth_pixel    ( ImageType& f_img, 
                                    FunctorType& f_func);

    //-------------------------------------------------------------------------
    //! applies a specified function object to each nth corresponding pixel 
    //! adresses (pointer) in a forward order (row major) by skipping SkipValue 
    //! columns and rows.
    //! @par Implicit Functor Interface:
    //! @code
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() ( ptr1, ptr2)
    //! @endcode
    //! @sa for_each_pixel
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <  int32_t SkipValue, 
                class Image1Type, class Image2Type, 
                class FunctorType>
    void    for_each_nth_pixel    ( Image1Type& f_img1, Image2Type& f_img2, 
                                    FunctorType& f_func);

    //-------------------------------------------------------------------------
    //! applies a specified function object to each nth corresponding pixel 
    //! adresses (pointer) in a forward order (row major) by skipping SkipValue 
    //! columns and rows.
    //! @par Implicit Functor Interface:
    //! @code
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() ( ptr1, ptr2, ptr3)
    //! @endcode
    //! @sa for_each_pixel
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <  int32_t SkipValue, 
                class Image1Type, class Image2Type, class Image3Type, 
                class FunctorType>
    void    for_each_nth_pixel    ( Image1Type& f_img1, Image2Type& f_img2, Image3Type& f_img3, 
                                    FunctorType& f_func);

    //-------------------------------------------------------------------------
    //! applies a specified function object to each nth corresponding pixel 
    //! adresses (pointer) in a forward order (row major) by skipping SkipValue 
    //! columns and rows.
    //! @par Implicit Functor Interface:
    //! @code
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() ( ptr1, ptr2, ptr3, ptr4)
    //! @endcode
    //! @sa for_each_pixel
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <  int32_t SkipValue, 
                class Image1Type, class Image2Type, class Image3Type, 
                class Image4Type, 
                class FunctorType>
    void    for_each_nth_pixel    ( Image1Type& f_img1, Image2Type& f_img2, Image3Type& f_img3, 
                                    Image4Type& f_img4, 
                                    FunctorType& f_func);

    ////////////////////////////////////////
    // for_each_pixel_xy()
    ////////////////////////////////////////

    //-------------------------------------------------------------------------
    //! applies a specified function object to each pixel adress (pointer) of 
    //! specified image in a forward order (row-major) with row and column 
    //! indices belonging to them.
    //! @par Implicit Functor Interface:
    //! @code
    //! // xposn, yposn    values of type int32_t
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() (  xpos1, ypos1, 
    //!                    ptr1)
    //! @endcode
    //! @sa for_each_pixel
    //! @author zvh2hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <  class ImageType, 
                class FunctorType>
    void    for_each_pixel_xy    (  ImageType& f_img, 
                                    FunctorType& f_func, 
                                    int32_t f_xOffset_i32 = 0, int32_t f_yOffset_i32 = 0);

    //-------------------------------------------------------------------------
    //! applies a specified function object to corresponding pixel 
    //! adresses (pointers) of specified images in a forward order (row-major)
    //! with row and column indices belonging to them.
    //! @par Implicit Functor Interface:
    //! @code
    //! // xposn, yposn    values of type int32_t
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() (  xpos1, ypos1, 
    //!                    xpos2, ypos2,   
    //!                    ptr1,
    //!                    ptr2)
    //! @endcode
    //! @sa for_each_pixel
    //! @author kon3hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

    template <  class Image1Type, class Image2Type, 
                class FunctorType>
    void    for_each_pixel_xy    (  Image1Type& f_img1, Image2Type& f_img2,
                                    FunctorType& f_func, 
                                    int32_t f_xOffset1_i32 = 0, int32_t f_yOffset1_i32 = 0,
                                    int32_t f_xOffset2_i32 = 0, int32_t f_yOffset2_i32 = 0);

    //-------------------------------------------------------------------------
    //! applies a specified function object to corresponding pixel 
    //! adresses (pointers) of specified images in a forward order (row-major)
    //! with row and column indices belonging to them.
    //! @par Implicit Functor Interface:
    //! @code
    //! // xposn, yposn    values of type int32_t
    //! // ptrn            values of ImageNType::raw_iterator type
    //!
    //! func.operator() (  xpos1, ypos1, 
    //!                    xpos2, ypos2,   
    //!                    xpos3, ypos3,  
    //!                    ptr1,
    //!                    ptr2,
    //!                    ptr3)
    //! @endcode
    //! @sa for_each_pixel
    //! @author kon3hi
    //! @ingroup vfc_group_image_algorithms
    //-------------------------------------------------------------------------

   template <  class Image1Type, class Image2Type, class Image3Type,
                class FunctorType>
    void    for_each_pixel_xy    (  Image1Type& f_img1, Image2Type& f_img2, Image3Type& f_img3,
                                    FunctorType& f_func, 
                                    int32_t f_xOffset1_i32 = 0, int32_t f_yOffset1_i32 = 0,
                                    int32_t f_xOffset2_i32 = 0, int32_t f_yOffset2_i32 = 0,
                                    int32_t f_xOffset3_i32 = 0, int32_t f_yOffset3_i32 = 0);

}    // namespace vfc closed

#include "vfc/image/vfc_image_algorithm.inl"

#endif //VFC_IMAGE_ALGORITHM_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
// $Log: vfc_image_algorithm.hpp  $
// Revision 1.20 2007/07/23 13:22:49MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
// - added implicit functor interface  (mantis1744)
// Revision 1.19 2007/07/23 09:51:59CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
// - doxygen grouping (mantis1744)
// Revision 1.18 2007/04/24 15:42:46CEST dkn2kor 
// - VFC_REQUIRE precondition checks used instead of if statement.  
//   the function interface has changed.  returns void instead of bool (mantis1578)
//   
// Revision 1.17 2007/04/02 19:02:46IST Koenig Matthias (CR/AEM5) (kon3hi) 
// Added for_each_pixel_xy for two and three images (MANTIS 0001389)
// Revision 1.16 2007/01/29 14:45:46CET ZVH2HI 
// - replaced header + footer with c++ version
// - added for_each_pixel() with 5 and 6 images (mantis1396)
// Revision 1.15 2006/11/16 14:41:10CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
// - replaced tabs with 4 spaces (mantis1294)
// Revision 1.14 2006/03/03 10:34:02CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
// -added missing include "vfc_types.hpp" (mantis1016)
// Revision 1.13 2006/02/01 17:24:34CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -added for_each_pixel_backward() function for two images
// Revision 1.12 2006/02/01 16:27:03CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -added for_each_pixel_backward()
// Revision 1.11 2005/12/15 17:37:58CET Muehlmann Karsten (AE-DA/ESA3) * (MUK2LR) 
// addition to implicit interface in documentation block
// Revision 1.10 2005/12/02 10:05:39CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -fixed implicit interface documentation
// Revision 1.9 2005/12/01 17:27:01CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -cosmetics
// Revision 1.8 2005/12/01 16:58:39CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -added for_each_pixel_xy()
// Revision 1.7 2005/11/16 11:46:02CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -moved for_each_nmth_pixel() to vfc_image_algorithm_ex.hpp (function ist still experimental and only used for RSR)
// Revision 1.6 2005/11/15 09:53:09CET Alaa El-Din Omar (AE-DA/ESA3-Hi) * (ALO2HI) 
// - added function for_each_nmth_pixel to allow writing output image with different size than input image (packed subampled output)
// Revision 1.5 2005/11/01 10:31:06CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -added four image for_each_...
// -fixed bug in for_each_nth_pixel()
// Revision 1.4 2005/10/11 15:15:39CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// -experimental nth pixel algo
// Revision 1.3 2005/10/06 17:06:23CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
// Initial revision
// Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/image/image.pj
// Revision 1.2 2005/09/13 17:14:50CEST zvh2hi 
// -several changes
//=============================================================================
