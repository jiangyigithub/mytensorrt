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
//       Projectname: 
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
//        Name: 
//  Department: 
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_algorithm_helper.hpp $
///     $Revision: 1.3 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/08/02 15:52:33MESZ $
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

#ifndef VFC_ALGORITHM_HELPER_HPP_INCLUDED
#define VFC_ALGORITHM_HELPER_HPP_INCLUDED

namespace vfc
{    // namespace vfc opened

    //-------------------------------------------------------------------------
    // conditional doxygen documentation
    //! @cond VFC_DOXY_INTERN
    //-------------------------------------------------------------------------

    namespace intern
    {   
        // doxygen conditional documentation
        //! @cond VFC_DOXY_INTERN

        //! unroll definitions
        struct CUnroll4
        {
            enum {OFFSET = 0x4};
            enum {BITMASK = OFFSET - 1};
        };

        //! @endcond
        // of VFC_DOXY_INTERN

    }    // namespace intern closed

    //-------------------------------------------------------------------------
    //! @endcond 
    // of VFC_DOXY_INTERN
    //-------------------------------------------------------------------------

}    // namespace vfc closed

#endif //VFC_ALGORITHM_HELPER_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_algorithm_helper.hpp  $
//  Revision 1.3 2007/08/02 15:52:33MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - added conditional doxygen documentation generation of vfc::intern (mantis1758)
//  Revision 1.2 2007/07/23 09:24:45CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.1 2007/02/23 13:36:11CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/core.pj
//=============================================================================
