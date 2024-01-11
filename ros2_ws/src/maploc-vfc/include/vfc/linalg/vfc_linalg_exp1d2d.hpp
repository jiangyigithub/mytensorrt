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
///     $Source: vfc_linalg_exp1d2d.hpp $
///     $Revision: 1.4 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2011/01/21 12:53:36MEZ $
///     $Locker:  $
///     $Name:  $
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

#ifndef VFC_LINALG_EXP1D2D_HPP_INCLUDED
#define VFC_LINALG_EXP1D2D_HPP_INCLUDED

#include "vfc/linalg/vfc_linalg_exp1d2d_common.hpp"
#include "vfc/linalg/vfc_linalg_shapepromotion.hpp"

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

            //=============================================================================
            // TExp2D1DMulOpImpl
            //-----------------------------------------------------------------------------
            //! 2D * 1D implementation for a Dynamic expression
            //! @param ValueType          Data Type
            //! @param Expression1Type    Data Type
            //! @param Expression2Type    Data Type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp1d2d.hpp $
            //=============================================================================
            template<class ValueType, class Expression1Type, class Expression2Type>
            class TExp2D1DMulOpImpl<ValueType, Expression1Type, Expression2Type, vfc::linalg::CDynamicRectangle>
            {

            public:

                //---------------------------------------------------------------------
                //! Returns 2D * 1D for a Dynamic expression
                //! @param f_operand1_r        Expression Data of type Expression1Type
                //! @param f_operand2_r        Expression Data of type Expression2Type
                //! @param f_row               Row number.
                //! @return  Returns 2D * 1D for a Dynamic expression
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d.hpp $
                //---------------------------------------------------------------------
                inline
                static ValueType eval (
                    const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r,
                    vfc::int32_t f_row)
                {
                    ValueType value = static_cast<ValueType>(0);
                    for (vfc::int32_t i=0; i < f_operand1_r.getNbColumns(); ++i)
                    {
                        value += f_operand1_r(f_row, i) * f_operand2_r[i];
                    }
                    return value;
                }
            };

        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed

#endif //VFC_LINALG_EXP1D2D_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_exp1d2d.hpp  $
//  Revision 1.4 2011/01/21 12:53:36MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - sometimes .hpp or namespace qualifiers are missing (mantis3599)
//  Revision 1.3 2009/03/25 14:48:17MEZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.2 2008/07/31 14:08:18IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:19IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
