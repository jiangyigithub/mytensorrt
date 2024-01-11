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
//       Projectname: vfc/memory
//          Synopsis: memory management utils
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
///     $Source: vfc_memory_util.hpp $
///     $Revision: 1.3 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/07/23 09:57:41MESZ $
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

#ifndef VFC_MEMORY_UTIL_HPP_INCLUDED
#define VFC_MEMORY_UTIL_HPP_INCLUDED

namespace vfc
{   // namespace vfc opened

    //=========================================================================
    //  DOYGEN ADDTOGROUP vfc_group_memory BEGIN
    //-------------------------------------------------------------------------
    /// @addtogroup vfc_group_memory
    /// @{
    //-------------------------------------------------------------------------


    //! calls delete on specified pointer and sets pointer to '0'
    template <class ValueType>
    void    deleteAndNull (ValueType*& f_ptr_p);

    //! calls delete [] (array delete) on specified pointer and sets pointer to '0'
    template <class ValueType>
    void    deleteArrayAndNull (ValueType*& f_ptr_p);

    //=========================================================================
    //  DOYGEN ADDTOGROUP vfc_group_memory END
    //-------------------------------------------------------------------------
    /// @}
    //-------------------------------------------------------------------------

}   // namespace vfc closed

#include "vfc/memory/vfc_memory_util.inl"

#endif //VFC_MEMORY_UTIL_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_memory_util.hpp  $
//  Revision 1.3 2007/07/23 09:57:41MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.2 2006/10/23 13:56:29CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - renamed functions (mantis1211)
//  - replaced header/footer templates
//=============================================================================
