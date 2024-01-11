//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2006 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorised copy, use or
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
//        Name: dkn2kor
//  Department:
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_linalg_alias_wrapper.hpp $
///     $Revision: 1.4 $
///     $Author: gaj2kor $
///     $Date: 2009/03/25 14:48:10MEZ $
///     $Locker:  $
///     $Name: 0032 RC1  $
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

#ifndef VFC_LINALG_ALIAS_WRAPPER_HPP_INCLUDED
#define VFC_LINALG_ALIAS_WRAPPER_HPP_INCLUDED

namespace vfc
{   // namespace vfc opened

    namespace linalg
    {   // namespace linalg opened

        //-------------------------------------------------------------------------
        // conditional doxygen documentation
        //! @cond VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

        namespace intern
        {   // namespace intern opened
            template<class ExpressionType>
            class TAliasWrapper;
        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg BEGIN
        //-------------------------------------------------------------------------
        /// @addtogroup vfc_group_linalg
        /// @{
        //-------------------------------------------------------------------------

        //---------------------------------------------------------------------
        //! The alias function is intended to be used for math expressions
        //! which are not alias-free (source and destination memory are
        //! overlapping).
        //! The alias problem is handled by this alias function by generating
        //! an implicit temporary which stores the result of the expression.
        //! The temporary is copied to the result object after expression
        //! evaluation is done.
        //! \li Note: @par
        //! Think carefully if alias is needed when writing an expression.
        //! If it is needed but left out, the results are wrong.
        //! If it is added but not needed, there is a performance hit due
        //! to the unnecessary temporary.
        //! \li Examples: @par
        //! The expression "A = A * B" has overlapping memory,
        //! so one must write
        //! \code vfc::linalg::alias(A) = A * B; \endcode.
        //! The expression "A = B * C" has no overlapping memory, thus does
        //! not need alias.
        //! \li Exception: @par
        //! Element wise operations, e.g. the expression
        //! "A = A + B" has overlapping memory for source and destination,
        //! but does not need the alias function.
        //! @par
        //! @param[in] f_expr_r The expression to be aliased.
        //! @return A TAliasWrapper object.
        //! @author jat2hi
        //! $Source: vfc_linalg_alias_wrapper.hpp $
        //---------------------------------------------------------------------
        template<class ExpressionType>
        intern::TAliasWrapper<ExpressionType>
        alias(ExpressionType& f_expr_r);

        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg END
        //-------------------------------------------------------------------------
        /// @}
        //-------------------------------------------------------------------------


    }   // namespace linalg closed

}   // namespace vfc closed

#include "vfc/linalg/vfc_linalg_alias_wrapper.inl"
#endif //VFC_LINALG_ALIAS_WRAPPER_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_alias_wrapper.hpp  $
//  Revision 1.4 2009/03/25 14:48:10MEZ gaj2kor 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.3 2008/08/29 18:34:47IST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.2 2008/07/31 14:08:11IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:18IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
