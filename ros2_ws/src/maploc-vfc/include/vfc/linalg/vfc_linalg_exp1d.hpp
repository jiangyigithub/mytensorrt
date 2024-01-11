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
///     $Source: vfc_linalg_exp1d.hpp $
///     $Revision: 1.4 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/03/11 16:42:28MEZ $
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

#ifndef VFC_LINALG_EXP1D_HPP_INCLUDED
#define VFC_LINALG_EXP1D_HPP_INCLUDED

#include <functional>

#include "vfc/linalg/vfc_linalg_exp1d_common.hpp"

namespace vfc
{   // namespace vfc opened

    namespace linalg
    {   // namespace linalg opened

        //-------------------------------------------------------------------------
        // conditional doxygen documentation
        //! @cond VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

        namespace intern
        {  // namespace intern opened

            //=============================================================================
            // TExp1DDotProductImpl
            //---------------------------------------------------------------------
            //!  Dot product implementation for a Dynamic Vector/1D Expression
            //! @param ValueType        Data Type
            //! @param Expression1Type  Data Type for dynamic Vector/1D Expression
            //! @param Expression2Type  Data Type for dynamic Vector/1D Expression
            //! @ingroup    vfc_group_linalg
            //! @author     jat2hi
            //! $Source: vfc_linalg_exp1d.hpp $
            //=============================================================================
            template<class ValueType, class Expression1Type, class Expression2Type>
            class TExp1DDotProductImpl<ValueType, Expression1Type, Expression2Type, CDynamicRectangle>
            {
            public:
                /// Value Type
                typedef ValueType           value_type;
                /// Shape Type
                typedef CDynamicRectangle   shape_type;
                /// Size Type
                typedef vfc::int32_t        size_type;

                //---------------------------------------------------------------------
                //!  Evaluates the dot product of a dynamic vector/1D expression
                //! @param f_operand1_r       Expression Data of type Expression1Type
                //! @param f_operand2_r       Expression Data of type Expression2Type
                //! @return  Returns scalar value of type ValueType
                //! @author     jat2hi
                //! $Source: vfc_linalg_exp1d.hpp $
                //---------------------------------------------------------------------
                static
                ValueType eval (
                    const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r);
            };


            //=============================================================================
            // TExp1DSumImpl
            //---------------------------------------------------------------------
            //!  Sum implementation for Dynamic vectors and 1d expressions
            //! @param ValueType        Data Type
            //! @param ExpressionType   Data Type for Dynamic Vector/1D Expression
            //! @ingroup    vfc_group_linalg
            //! @author     jat2hi
            //! $Source: vfc_linalg_exp1d.hpp $
            //=============================================================================
            template<class ValueType, class ExpressionType>
            class TExp1DSumImpl<ValueType, ExpressionType, CDynamicRectangle>
            {
            public:
                /// Value Type
                typedef ValueType           value_type;
                /// Shape Type
                typedef CDynamicRectangle   shape_type;
                /// Size Type
                typedef vfc::int32_t        size_type;

                //---------------------------------------------------------------------
                //!  Evaluates the sum of a dynamic vector/1D expression
                //! @param f_operand_r       Expression Data of type ExpressionType
                //! @return  Returns scalar value of type ValueType
                //! @author     jat2hi
                //! $Source: vfc_linalg_exp1d.hpp $
                //---------------------------------------------------------------------
                static
                ValueType eval (const ExpressionType& f_operand_r);
            };


            //=============================================================================
            // TExp1DNormLinfOpImpl
            //---------------------------------------------------------------------
            //!  Norm implementation for Dynamic vectors and 1d expressions
            //! @param ValueType        Data Type
            //! @param ExpressionType   Data Type for Dynamic Vector/1D Expression
            //! @ingroup    vfc_group_linalg
            //! @author     jat2hi
            //! $Source: vfc_linalg_exp1d.hpp $
            //=============================================================================
            template<class ValueType, class ExpressionType>
            class TExp1DNormLinfOpImpl<ValueType, ExpressionType, CDynamicRectangle>
            {
            public:
                /// Value Type
                typedef ValueType           value_type;
                /// Shape Type
                typedef CDynamicRectangle   shape_type;
                /// Size Type
                typedef vfc::int32_t        size_type;

                //---------------------------------------------------------------------
                //!  Evaluates the norm of a dynamic vector/1D expression
                //! @param f_operand_r       Expression Data of type ExpressionType
                //! @return  Returns scalar value of type ValueType
                //! @author     jat2hi
                //! $Source: vfc_linalg_exp1d.hpp $
                //---------------------------------------------------------------------
                static
                ValueType eval (const ExpressionType& f_operand_r);
            };

        } // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed


#include "vfc/linalg/vfc_linalg_exp1d.inl"

#endif //VFC_LINALG_EXP1D_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_exp1d.hpp  $
//  Revision 1.4 2014/03/11 16:42:28MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - The linalg _ops headers lack the <functional> include (mantis0004401)
//  Revision 1.3 2009/03/25 14:48:15MEZ Gaurav Jain (RBEI/ESD4) (gaj2kor) 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.2 2008/07/31 14:08:15IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:18IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
