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
///     $Source: vfc_linalg_algorithm2d.hpp $
///     $Revision: 1.3 $
///     $Author: gaj2kor $
///     $Date: 2009/03/25 14:48:09MEZ $
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

#ifndef VFC_LINALG_ALGORITHM2D_HPP_INCLUDED
#define VFC_LINALG_ALGORITHM2D_HPP_INCLUDED


#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_algorithm2d_fixed.hpp"
#include "vfc/core/vfc_algorithm2d.hpp"
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

            //=====================================================================
            //  TIdentityFunctor
            //---------------------------------------------------------------------
            //! Used while generating the identity of a matrix
            //! Sets the matrix element m(i,j) to 1 if i = j
            //! else to 0.
            //! @param ValueType        Data Type
            //! @author jat2hi
            //! @ingroup                vfc_group_linalg
            //! $Source: vfc_linalg_algorithm2d.hpp $
            //=====================================================================
            template<class ValueType>
            struct TIdentityFunctor
            {
                typedef vfc::int32_t     size_type;
                typedef ValueType        value_type;
                inline TIdentityFunctor() { }
                inline
                value_type  operator()(size_type f_row, size_type f_col) const
                {
                    return (f_row==f_col)?static_cast<ValueType>(1):static_cast<ValueType>(0);
                }
            };

            //---------------------------------------------------------------------
            //! Copies a static Matrix/2D expression to the destination using template unrolling
            //! @param  f_src_r         Source value of type SourceType
            //! @param  f_to_r          Destinaltion value of type DestinationType
            //! @param  true_t
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm2d.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, vfc::int32_t ColValue,
                class SourceType, class DestinationType>
            inline
            void copy_matrix_fixed_mn(const SourceType& f_src_r,
                DestinationType& f_to_r, true_t)
            {
                vfc::transform_2d_round_fixed_n<RowValue, ColValue>(f_src_r, f_to_r,
                    TCopyFunctor<typename SourceType::value_type,
                        typename DestinationType::value_type>());
            }

            //---------------------------------------------------------------------
            //! Copies a static Matrix/2D expression to the destination without using template unrolling
            //! @param  f_src_r         Source value of type SourceType
            //! @param  f_to_r          Destinaltion value of type DestinationType
            //! @param  false_t
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm2d.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, vfc::int32_t ColValue,
                class SourceType, class DestinationType>
            inline
            void copy_matrix_fixed_mn(const SourceType& f_src_r,
                DestinationType& f_to_r, false_t)
            {
                vfc::transform_2d_round_n(f_src_r, f_to_r,
                    TCopyFunctor<typename SourceType::value_type,
                        typename DestinationType::value_type>(),
                    RowValue, ColValue);
            }

            //---------------------------------------------------------------------
            //! Performs operation OperationFunctorType using template unrolling
            //! on static Matrices/2D Expressions InArg1Type & InArg2Type and
            //! copies the resultant to the destination OutArgType
            //! @param  f_inArg1_r          static Vectors/1D Expressions
            //! @param  f_inArg2_r          static Vectors/1D Expressions
            //! @param  f_outArg            static Vectors/1D Expressions
            //! @param  f_functor_r         Function pointer of type OperationFunctorType
            //! @param  true_t
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm2d.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, vfc::int32_t ColValue,
                class OperationFunctor, class InArg1Type, class InArg2Type,
                class OutArgType>
            inline
            void do_operation_matrix_fixed_mn(const InArg1Type& f_inArg1_r, const InArg2Type& f_inArg2_r,
                OutArgType& f_outArg_r, const OperationFunctor& f_functor_r, true_t)
            {
                vfc::transform_2d_round_fixed_n<RowValue, ColValue>(
                    f_inArg1_r, f_inArg2_r, f_outArg_r, f_functor_r);
            }

            //---------------------------------------------------------------------
            //! Performs operation OperationFunctorType without using
            //! template unrolling on static Matrices/2D Expressions
            //! InArg1Type & InArg2Type and copies the resultant to the destination OutArgType
            //! @param  f_inArg1_r          static Vectors/1D Expressions
            //! @param  f_inArg2_r          static Vectors/1D Expressions
            //! @param  f_outArg            static Vectors/1D Expressions
            //! @param  f_functor_r         Function pointer of type OperationFunctorType
            //! @param  false_t
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm2d.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, vfc::int32_t ColValue,
                class OperationFunctor, class InArg1Type, class InArg2Type,
                class OutArgType>
            inline
            void do_operation_matrix_fixed_mn(const InArg1Type& f_inArg1_r, const InArg2Type& f_inArg2_r,
                OutArgType& f_outArg_r, const OperationFunctor& f_functor_r, false_t)
            {
                vfc::transform_2d_round_n(f_inArg1_r, f_inArg2_r, f_outArg_r,
                    f_functor_r, RowValue, ColValue);
            }

            //---------------------------------------------------------------------
            //! Performs operation OperationFunctorType using template unrolling
            //! on static Matrices/2D Expression InArg1Type and
            //! copies the resultant to the destination OutArgType
            //! @param  f_inArg_r           static Vectors/1D Expressions
            //! @param  f_outArg            static Vectors/1D Expressions
            //! @param  f_functor_r         Function pointer of type OperationFunctorType
            //! @param  true_t
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm2d.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, vfc::int32_t ColValue, class OperationFunctor,
                class InArgType, class OutArgType>
            inline
            void do_operation_matrix_fixed_mn(const InArgType& f_inArg_r,
                OutArgType& f_outArg_r, const OperationFunctor& f_functor_r, true_t)
            {
                vfc::transform_2d_round_fixed_n<RowValue, ColValue>(f_inArg_r, f_outArg_r,
                    f_functor_r);
            }

            //---------------------------------------------------------------------
            //! Performs operation OperationFunctorType without
            //! using template unrolling on static Matrices/2D Expression
            //! InArg1Type and copies the resultant to the destination OutArgType
            //! @param  f_inArg_r           static Vectors/1D Expressions
            //! @param  f_outArg            static Vectors/1D Expressions
            //! @param  f_functor_r         Function pointer of type OperationFunctorType
            //! @param  false_t
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm2d.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, vfc::int32_t ColValue, class OperationFunctor,
                class InArgType, class OutArgType>
            inline
            void do_operation_matrix_fixed_mn(const InArgType& f_inArg_r,
                OutArgType& f_outArg_r, const OperationFunctor& f_functor_r, false_t)
            {
                vfc::transform_2d_round_n(f_inArg_r, f_outArg_r,
                    f_functor_r, RowValue, ColValue);
            }

            //---------------------------------------------------------------------
            //! Copies a source fixed size matrix to the destination fixed size matrix
            //! @param  f_from_r        Source value of type SourceType
            //! @param  f_to_r          Destinaltion value of type DestinationType
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm2d.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, vfc::int32_t ColValue, class SourceType,
                class DestinationType>
            inline
            void copy_matrix_fixed_mn(const SourceType& f_from_r,
                    DestinationType& f_to_r)
            {
                intern::copy_matrix_fixed_mn<RowValue, ColValue>(f_from_r, f_to_r,
                    typename intern::TMetaprogUnroll<RowValue, ColValue>::type());
            }

            //---------------------------------------------------------------------
            //! Copies a source dynamic size matrix to the destination dynamic size matrix
            //! @param  f_from_r        Source value of type SourceType
            //! @param  f_to_r          Destinaltion value of type DestinationType
            //! @param  f_nbRows        Row number
            //! @param  f_nbCols        Column number
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm2d.hpp $
            //---------------------------------------------------------------------
            template<class SourceType, class DestinationType>
            inline
            void copy_matrix_mn(const SourceType& f_from_r, DestinationType& f_to_r,
                vfc::int32_t f_nbRows, vfc::int32_t f_nbCols)
            {
                vfc::transform_2d_round_n(f_from_r, f_to_r,
                    intern::TCopyFunctor<typename SourceType::value_type,
                            typename DestinationType::value_type>(),
                        f_nbRows, f_nbCols);
            }

            //---------------------------------------------------------------------
            //! Performs operation OperationFunctorType
            //! on fixed size matrix / 2D Expressions InArg1Type & InArg2Type and
            //! copies the resultant to the destination OutArgType
            //! @param  f_inArg1_r          static Vectors/1D Expressions
            //! @param  f_inArg2_r          static Vectors/1D Expressions
            //! @param  f_outArg            static Vectors/1D Expressions
            //! @param  f_functor_r         Function pointer of type OperationFunctorType
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm2d.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, vfc::int32_t ColValue, class OperationFunctor,
                class InArg1Type, class InArg2Type, class OutArgType>
            inline
            void do_operation_matrix_fixed_mn(const InArg1Type& f_inArg1_r, const InArg2Type& f_inArg2_r,
                OutArgType& f_outArg_r, const OperationFunctor& f_functor_r)
            {
                intern::do_operation_matrix_fixed_mn<RowValue, ColValue>(f_inArg1_r, f_inArg2_r,
                    f_outArg_r, f_functor_r, typename intern::TMetaprogUnroll<RowValue, ColValue>::type());
            }

            //---------------------------------------------------------------------
            //! Performs operation OperationFunctorType
            //! on dynamic size matrix / 2D Expressions InArg1Type & InArg2Type and
            //! copies the resultant to the destination OutArgType
            //! @param  f_inArg1_r          static Vectors/1D Expressions
            //! @param  f_inArg2_r          static Vectors/1D Expressions
            //! @param  f_outArg            static Vectors/1D Expressions
            //! @param  f_functor_r         Function pointer of type OperationFunctorType
            //! @param  f_nbRows            Row number
            //! @param  f_nbCols            Column number
            //! @author jat2hi
            //! $Source: vfc_linalg_algorithm2d.hpp $
            //---------------------------------------------------------------------
            template<class OperationFunctor, class InArg1Type,
                class InArg2Type, class OutArgType>
            inline
            void do_operation_matrix_mn(const InArg1Type& f_inArg1_r, const InArg2Type& f_inArg2_r,
                OutArgType& f_outArg_r, const OperationFunctor& f_functor_r,
                vfc::int32_t f_nbRows, vfc::int32_t f_nbCols)
            {
                vfc::transform_2d_round_n(f_inArg1_r, f_inArg2_r, f_outArg_r,
                        f_functor_r, f_nbRows, f_nbCols);
            }
        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed


#endif //VFC_LINALG_ALGORITHM2D_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_algorithm2d.hpp  $
//  Revision 1.3 2009/03/25 14:48:09MEZ gaj2kor 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.2 2008/07/31 14:08:10IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:18IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
