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
*     $Source: vfc_static_assert.hpp $
*     $Revision: 1.8 $
*     $Author: gaj2kor $
*     $Date: 2009/02/03 07:06:36MEZ $
*     $Locker:  $
*     $Name: 0032 RC1  $
*     $State: in_work $
*/
/*******************************************************************/

#ifndef VFC_STATIC_ASSERT_HPP_INCLUDED
#define VFC_STATIC_ASSERT_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_preprocessor.hpp"

namespace vfc
{    // namespace vfc opened

    //-----------------------------------------------------------------------------
    //  doxygen conditional documentation
    /// @cond VFC_DOXY_INTERN
    //-----------------------------------------------------------------------------

    namespace intern
    {
        //=============================================================================
        //  TStaticAssertionFailure
        //-----------------------------------------------------------------------------
        /// helper struct for static asserts.
        /// @sa VFC_STATIC_ASSERT
        /// @ingroup vfc_group_core_error
        //=============================================================================

        template <bool BooleanValue>
        struct TStaticAssertionFailure;

        /// @cond VFC_DOXY_SPECIALIZATIONS

            template <>
            struct TStaticAssertionFailure<true>
            {
                enum { value = 1 };
            };

        /// @endcond

        //=============================================================================
        //  TStatic_Assert_Helper
        //-----------------------------------------------------------------------------
        /// helper struct for static asserts.
        /// @sa VFC_STATIC_ASSERT
        /// @ingroup vfc_group_core_error
        //=============================================================================

        template<int32_t x>
        struct TStatic_Assert_Helper
        {};
    }

    //-----------------------------------------------------------------------------
    /// @endcond
    //  of VFC_DOXY_INTERN
    //-----------------------------------------------------------------------------

}    // namespace vfc closed


//=============================================================================
//  VFC_STATIC_ASSERT
//-----------------------------------------------------------------------------
/// macro for static (compile-time) assertions.
/// @ingroup vfc_group_core_error
//=============================================================================
#define VFC_STATIC_ASSERT(EXP)\
    typedef ::vfc::intern::TStatic_Assert_Helper<\
    sizeof(::vfc::intern::TStaticAssertionFailure< (bool)( EXP ) >)>\
         VFC_JOIN(vfc_static_assert_typedef_, __LINE__)


#endif //VFC_STATIC_ASSERT_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_static_assert.hpp  $
Revision 1.8 2009/02/03 07:06:36MEZ gaj2kor 
-Removal of QAC++ warnings.
(Mantis : 0002502)
Revision 1.7 2007/08/02 19:18:58IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
- added conditional doxygen documentation generation of vfc::intern (mantis1758)
Revision 1.6 2007/07/23 09:22:54CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
- added comments (mantis1744)
Revision 1.5 2007/07/18 16:33:41CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
- doxygen documentation grouping (mantis1744)
Revision 1.4 2006/11/16 14:41:14CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
- replaced tabs with 4 spaces (mantis1294)
Revision 1.2 2005/10/06 16:53:09CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/vfc.pj
********************************************************************/
