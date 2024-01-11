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
///     $Source: vfc_linalg_algorithm1d.hpp $
///     $Revision: 1.5 $
///     $Author: gaj2kor $
///     $Date: 2009/03/25 14:48:08MEZ $
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

#ifndef VFC_LINALG_ALGORITHM1D_HPP_INCLUDED
#define VFC_LINALG_ALGORITHM1D_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_algorithm_fixed.hpp"
#include "vfc/core/vfc_algorithm.hpp"
#include "vfc/core/vfc_static_assert.hpp"
#include "vfc/linalg/vfc_linalg_util.hpp"

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

            //=========================================================================
            //  DOYGEN ADDTOGROUP vfc_group_linalg BEGIN
            //-------------------------------------------------------------------------
            /// @addtogroup vfc_group_linalg
            /// @{
            //-------------------------------------------------------------------------

            //---------------------------------------------------------------------
            //! Copies a static Vector/1D expression to the destination using template unrolling
            //! @param  f_src_r         Source value of type SourceType
            //! @param  f_to_r          Destinaltion value of type DestinationType
            //! @param  true_t
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm1d.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, class SourceType, class DestinationType>
            inline
            void copy_vector_fixed_n(const SourceType& f_src_r,
                DestinationType& f_to_r, true_t)
            {
                vfc::transform_square_fixed_n<RowValue>(f_src_r, f_to_r,
                    TCopyFunctor<typename SourceType::value_type,
                        typename DestinationType::value_type>());
            }

            //---------------------------------------------------------------------
            //! Copies a static Vector/1D expression to the destination without using template unrolling
            //! @param  f_src_r         Source value of type SourceType
            //! @param  f_to_r          Destinaltion value of type DestinationType
            //! @param  false_t
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm1d.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, class SourceType, class DestinationType>
            inline
            void copy_vector_fixed_n(const SourceType& f_src_r,
                DestinationType& f_to_r, false_t)
            {
                vfc::transform_square_n(f_src_r, f_to_r,
                    TCopyFunctor<typename SourceType::value_type,
                        typename DestinationType::value_type>(),
                    RowValue);
            }

            //---------------------------------------------------------------------
            //! Performs operation OperationFunctorType using template unrolling
            //! on static Vectors/1D Expressions InArg1Type & InArg2Type and
            //! copies the resultant to the destination OutArgType
            //! @param  f_inArg1_r          static Vectors/1D Expressions
            //! @param  f_inArg2_r          static Vectors/1D Expressions
            //! @param  f_outArg            static Vectors/1D Expressions
            //! @param  f_functor_r         Function pointer of type OperationFunctorType
            //! @param  true_t
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm1d.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, class OperationFunctorType, class InArg1Type,
                class InArg2Type, class OutArgType>
            inline
            void do_operation_vector_fixed_n(const InArg1Type& f_inArg1_r, const InArg2Type& f_inArg2_r,
                OutArgType& f_outArg, OperationFunctorType f_functor_r,true_t)
            {
                vfc::transform_square_fixed_n<RowValue>(
                    f_inArg1_r, f_inArg2_r, f_outArg, f_functor_r);
            }

            //---------------------------------------------------------------------
            //! Performs operation OperationFunctorType without using template unrolling
            //! on static Vectors/1D Expressions InArg1Type & InArg2Type and
            //! copies the resultant to the destination OutArgType
            //! @param  f_inArg1_r          static Vectors/1D Expressions
            //! @param  f_inArg2_r          static Vectors/1D Expressions
            //! @param  f_outArg            static Vectors/1D Expressions
            //! @param  f_functor_r         Function pointer of type OperationFunctorType
            //! @param  true_t
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm1d.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, class OperationFunctorType, class InArg1Type,
                class InArg2Type, class OutArgType>
            inline
            void do_operation_vector_fixed_n(const InArg1Type& f_inArg1_r, const InArg2Type& f_inArg2_r,
                OutArgType& f_outArg, OperationFunctorType f_functor_r,false_t)
            {
                vfc::transform_square_n(f_inArg1_r, f_inArg2_r, f_outArg,
                    f_functor_r, RowValue);
            }

            //---------------------------------------------------------------------
            //! Calls the intern function for a static vector/ 1D expression
            //! switching between with/without using template unrolling
            //! @param  f_from_r         Source value of type SourceType
            //! @param  f_to_r          Destinaltion value of type DestinationType
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm1d.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, class SourceType, class DestinationType>
            inline
            void copy_vector_fixed_n(const SourceType& f_from_r,
                    DestinationType& f_to_r)
            {
                intern::copy_vector_fixed_n<RowValue>(f_from_r, f_to_r,
                    typename intern::TMetaprogUnroll<RowValue, 1>::type());
            }

            //---------------------------------------------------------------------
            //! Copies a dynamic Vector/1D expression to the destination
            //! @param  f_from_r        Source value of type SourceType
            //! @param  f_to_r          Destinaltion value of type DestinationType
            //! @param  f_nbRows        Row number
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm1d.hpp $
            //---------------------------------------------------------------------
            template<class SourceType, class DestinationType>
            inline
            void copy_vector_n(const SourceType& f_from_r, DestinationType& f_to_r,
                vfc::int32_t f_nbRows)
            {
                vfc::transform_square_n(f_from_r, f_to_r,
                        intern::TCopyFunctor<typename SourceType::value_type,
                            typename DestinationType::value_type>(),
                        f_nbRows);
            }

            //---------------------------------------------------------------------
            //! Performs operation OperationFunctorType
            //! on dynamic size vector / 1D Expressions InArg1Type & InArg2Type and
            //! copies the resultant to the destination OutArgType
            //! @param  f_inArg1_r          static Vectors/1D Expressions
            //! @param  f_inArg2_r          static Vectors/1D Expressions
            //! @param  f_outArg            static Vectors/1D Expressions
            //! @param  f_functor_r         Function pointer of type OperationFunctorType
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm1d.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, class OperationFunctorType, class InArg1Type,
                class InArg2Type, class OutArgType>
            inline
            void do_operation_vector_fixed_n(const InArg1Type& f_inArg1_r, const InArg2Type& f_inArg2_r,
                OutArgType& f_outArg, OperationFunctorType f_functor_r)
            {
                intern::do_operation_vector_fixed_n<RowValue>(f_inArg1_r, f_inArg2_r, f_outArg, f_functor_r,
                    typename intern::TMetaprogUnroll<RowValue, 1>::type());
            }

            //---------------------------------------------------------------------
            //! Performs operation OperationFunctorType
            //! on dynamic size vector / 1D Expressions InArg1Type & InArg2Type and
            //! copies the resultant to the destination OutArgType
            //! @param  f_inArg1_r          static Vectors/1D Expressions
            //! @param  f_inArg2_r          static Vectors/1D Expressions
            //! @param  f_outArg            static Vectors/1D Expressions
            //! @param  f_functor_r         Function pointer of type OperationFunctorType
            //! @param  f_nbRows            Row number
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm1d.hpp $
            //---------------------------------------------------------------------
            template<class InArg1Type, class InArg2Type, class OutArgType, class OperationFunctorType>
            inline
            void do_operation_vector_n(const InArg1Type& f_inArg1_r, const InArg2Type& f_inArg2_r,
                OutArgType& f_outArg_r, const OperationFunctorType f_functor_r,
                vfc::int32_t f_nbRows)
            {
                vfc::transform_square_n(f_inArg1_r, f_inArg2_r, f_outArg_r,
                        f_functor_r, f_nbRows);
            }

            //=========================================================================
            //  DOYGEN ADDTOGROUP vfc_group_linalg END
            //-------------------------------------------------------------------------
            /// @}
            //-------------------------------------------------------------------------

        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed


#endif //VFC_LINALG_ALGORITHM1D_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_algorithm1d.hpp  $
//  Revision 1.5 2009/03/25 14:48:08MEZ gaj2kor 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.4 2008/08/29 18:34:46IST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.3 2008/08/08 14:41:10IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  Removed exclamation marks in comments ( Mantis :- 1744 , note:-3110)
//  Revision 1.2 2008/07/31 14:08:09IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:18IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
