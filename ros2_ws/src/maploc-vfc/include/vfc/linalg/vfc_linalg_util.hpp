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
///     $Source: vfc_linalg_util.hpp $
///     $Revision: 1.4 $
///     $Author: gaj2kor $
///     $Date: 2009/03/25 14:48:24MEZ $
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

#ifndef VFC_LINALG_UTIL_HPP_INCLUDED
#define VFC_LINALG_UTIL_HPP_INCLUDED


#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_type_traits.hpp"
#include "vfc/core/vfc_metaprog.hpp"

namespace vfc
{  // namespace vfc opened

    namespace linalg
    {  // namespace linalg opened

        //-------------------------------------------------------------------------
        // conditional doxygen documentation
        //! @cond VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

        namespace intern
        {  // namespace intern opened

            //=====================================================================
            // TMetaprogUnroll
            //---------------------------------------------------------------------
            //! Based on the given Shape Type, type is set to true_t or false_t
            //! This class is used to decide if unrolling is required or not
            //! @param RowValue      Number of rows
            //! @param ColValue      Number of columns
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_util.hpp $
            //=====================================================================
            template<vfc::int32_t RowValue, vfc::int32_t ColValue>
            struct TMetaprogUnroll
            {
            private:
                static const vfc::int32_t MAX_METAPROG_UNROLL_SIZE = 16;
            public:
                enum { value = RowValue*ColValue <= MAX_METAPROG_UNROLL_SIZE };
                typedef typename vfc::TIf<value, vfc::true_t,
                    vfc::false_t>::type type;
            };

            //=====================================================================
            // TScalarWrapper
            //---------------------------------------------------------------------
            //! Wrapper class to wrap the scalar value
            //! used for scalar multiplication/division
            //! @param ValueType      Data type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_util.hpp $
            //=====================================================================
            template<class ValueType>
            struct TScalarWrapper
            {
                typedef vfc::int32_t     size_type;
                typedef ValueType        value_type;
                inline TScalarWrapper(const ValueType& f_value_r)
                    : m_value(f_value_r) { }

                inline
                const value_type& operator()(size_type, size_type) const
                {
                    return m_value;
                }

                inline
                const value_type& operator[](size_type) const
                {
                    return m_value;
                }
            private:
                ValueType m_value;

                //Dummy function for suppression of QAC++ msg 2141
                //Msg 2141 : The subscript 'operator []' is not available in a non-const version.
                value_type& operator[](size_type);

            };

            //=====================================================================
            // TCopyFunctor
            //---------------------------------------------------------------------
            //! Used to copy single element from Source to Destination
            //! @param Value1Type      Data type
            //! @param Value2Type      Data type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_util.hpp $
            //=====================================================================
            template<class Value1Type, class Value2Type>
            struct TCopyFunctor
            {
                inline TCopyFunctor() { }
                inline
                Value1Type operator()(const Value2Type& f_value_r)
                {
                    return f_value_r;
                }
            };
        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed


#endif //VFC_LINALG_UTIL_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_util.hpp  $
//  Revision 1.4 2009/03/25 14:48:24MEZ gaj2kor 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.3 2009/01/07 10:19:44IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  QAC++ Warning (2141) Removal.
//  (Mantis : 0002479)
//  Revision 1.2 2008/07/31 14:08:45IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:26IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
