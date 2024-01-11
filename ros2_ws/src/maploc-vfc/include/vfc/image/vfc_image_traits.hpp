//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2007 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy or use or
//  distribution is an offensive act against international law and may be 
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  P R O J E C T   I N F O R M A T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/image
//  Target system(s): 
//       Compiler(s): VS8.0
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
//  F I L E   C O N T E N T S   A N D   R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @brief traits classes for image operations
///
/// @par Revision History:
///     $Source: vfc_image_traits.hpp $
///     $Revision: 1.1 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/11/26 16:54:06MEZ $
///     $Locker:  $
///     $Name: 0032 RC1 Hello  $
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

#ifndef VFC_IMAGE_TRAITS_HPP_INCLUDED
#define VFC_IMAGE_TRAITS_HPP_INCLUDED

namespace vfc
{
    //=========================================================================
    //  TImageRawIteratorTraits  
    //-------------------------------------------------------------------------
    //! Traits class for raw_iterator type.
    //! The general implementation just propagates the internal typedef.
    //! Add a specialization for your own image class if you want to use
    //! for_each_pixel and the general implementation doesn't fit.
    //!
    //! $Source: vfc_image_traits.hpp $
    //! @author zvh2hi
    //! @ingroup vfc_group_image_types
    //=========================================================================

    template <class ImageType> 
    struct TImageRawIteratorTraits
    {
        typedef typename ImageType::raw_iterator raw_iterator;
    };
}


#endif //VFC_IMAGE_TRAITS_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_image_traits.hpp  $
//  Revision 1.1 2007/11/26 16:54:06MEZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/image/image.pj
//=============================================================================
