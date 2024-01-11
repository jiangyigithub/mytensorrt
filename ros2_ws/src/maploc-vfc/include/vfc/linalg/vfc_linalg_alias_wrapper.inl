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
//        Name: prd3kor
//  Department:
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_linalg_alias_wrapper.inl $
///     $Revision: 1.4 $
///     $Author: Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) $
///     $Date: 2009/05/28 09:19:02MESZ $
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
#include <functional>

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
            class TAliasWrapper
            {
            public:
                TAliasWrapper(ExpressionType& f_expr_r) : m_expr(f_expr_r) { }

                TAliasWrapper(const TAliasWrapper& f_other) : m_expr(f_other.m_expr) {}

                template<class OtherExpressionType>
                ExpressionType& operator=(const OtherExpressionType& f_expr_r)
                {
                    m_expr.assign(f_expr_r);
                    return m_expr;
                }

                template<class OtherExpressionType>
                ExpressionType& operator+=(const OtherExpressionType& f_expr_r)
                {
                    m_expr.opassign(f_expr_r, stlalias::plus<typename OtherExpressionType::value_type>());
                    return m_expr;
                }

                template<class OtherExpressionType>
                ExpressionType& operator-=(const OtherExpressionType& f_expr_r)
                {
                    m_expr.opassign(f_expr_r, stlalias::minus<typename OtherExpressionType::value_type>());
                    return m_expr;
                }

                template<class OtherValueType>
                ExpressionType& operator*=(const OtherValueType& f_value_r)
                {
                    m_expr.unaryOpassign(f_value_r, stlalias::multiplies<OtherValueType>());
                    return m_expr;
                }

                template<class OtherValueType>
                ExpressionType& operator/=(const OtherValueType& f_value_r)
                {
                    m_expr.unaryOpassign(f_value_r, stlalias::divides<OtherValueType>());
                    return m_expr;
                }
            private:
                ExpressionType&           m_expr;

                //! No assignment operator.
                const TAliasWrapper& operator=(const TAliasWrapper&);
            };

        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed

template<class ExpressionType>
vfc::linalg::intern::TAliasWrapper<ExpressionType>
vfc::linalg::alias(ExpressionType& f_expr_r)
{
    return intern::TAliasWrapper<ExpressionType>(f_expr_r);
}
//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_alias_wrapper.inl  $
//  Revision 1.4 2009/05/28 09:19:02MESZ Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) 
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.3 2009/03/25 14:48:11CET Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.2 2007/11/12 21:58:24IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  - remove unnecessary iostream include (mantis 1906)
//  Revision 1.1 2007/05/09 10:21:18CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
