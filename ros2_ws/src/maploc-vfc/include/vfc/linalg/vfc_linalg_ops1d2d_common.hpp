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
///     $Source: vfc_linalg_ops1d2d_common.hpp $
///     $Revision: 1.4 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2011/01/21 12:53:29MEZ $
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

#ifndef VFC_LINALG_OPS1D2D_COMMON_HPP_INCLUDED
#define VFC_LINALG_OPS1D2D_COMMON_HPP_INCLUDED

#include "vfc/linalg/vfc_linalg_shapepromotion.hpp"
#include "vfc/linalg/vfc_linalg_exp1d_common.hpp"
#include "vfc/linalg/vfc_linalg_exp2d_common.hpp"
#include "vfc/linalg/vfc_linalg_exp1d2d_common.hpp"

// forward declarations
namespace vfc
{
    namespace linalg
    {
        template<class ValueType, class DerivedType, class ShapeType>
        class TVectorBase;

        template<class ValueType, class DerivedType, class ShapeType>
        class TMatrixBase;

        template <class ValueType, class DerivedType, class ShapeType>
        class TVectorConstRef;

        template <class ValueType, class DerivedType, class ShapeType>
        class TMatrixConstRef;
    }
}

namespace vfc
{
    namespace linalg
    {
        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg BEGIN
        //-------------------------------------------------------------------------
        /// @addtogroup vfc_group_linalg
        /// @{
        //-------------------------------------------------------------------------
        //---------------------------------------------------------------------
        //! * operator overload to perform matrix * vector
        //! Asserts if invalid operands are provided
        //! @param f_op1_r     Object of TMatrixBase
        //! @param f_op2_r     Object of TVectorBase
        //! @return  Returns matrix * vector.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class MatrixType, class VectorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
        intern::TExp2D1DMulOp<ValueType,
                TMatrixConstRef<ValueType, MatrixType, Shape1Type>,
                TVectorConstRef<ValueType, VectorType, Shape2Type> >,
            typename TExp2D1DMulShapePromotion<Shape1Type, Shape2Type>::type >
            operator* (const TMatrixBase<ValueType, MatrixType, Shape1Type>& f_op1_r,
            const TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //! * operator overload to perform matrix * 1D Expression
        //! Asserts if invalid operands are provided
        //! @param f_op1_r     Object of TMatrixBase
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns matrix * 1D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class MatrixType, class OperatorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp2D1DMulOp<ValueType,
                TMatrixConstRef<ValueType, MatrixType, Shape1Type>,
                intern::TExp1D<ValueType, OperatorType, Shape2Type> >,
            typename TExp2D1DMulShapePromotion<Shape1Type, Shape2Type>::type >
        operator* (const TMatrixBase<ValueType, MatrixType, Shape1Type>& f_op1_r,
            const intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //! * operator overload to perform 2D Expression * vector
        //! Asserts if invalid operands are provided
        //! @param f_op1_r     Object of TExp2D
        //! @param f_op2_r     Object of TMatrixBase
        //! @return  Returns 2D Expression * vector.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp2D1DMulOp<ValueType,
                intern::TExp2D<ValueType, OperatorType, Shape1Type>,
                TVectorConstRef<ValueType, VectorType, Shape2Type> >,
            typename TExp2D1DMulShapePromotion<Shape1Type, Shape2Type>::type >
        operator* (const intern::TExp2D<ValueType, OperatorType, Shape1Type>& f_op1_r,
            const TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //! * operator overload to perform 2D Expression * 1D Expression
        //! Asserts if invalid operands are provided
        //! @param f_op1_r     Object of TExp2D
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns 2D Expression * 1D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Operator1Type, class Operator2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp1D<ValueType,
            intern::TExp2D1DMulOp<ValueType,
                intern::TExp2D<ValueType, Operator1Type, Shape1Type>,
                intern::TExp1D<ValueType, Operator2Type, Shape2Type> >,
            typename TExp2D1DMulShapePromotion<Shape1Type, Shape2Type>::type >
        operator* (const intern::TExp2D<ValueType, Operator1Type, Shape1Type>& f_op1_r,
            const intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //! dyad function overload:
        //! Generates dyad product of Vector and Vector
        //! and returns 2D Expression
        //! @param f_op1_r     Object of TVectorBase
        //! @param f_op2_r     Object of TVectorBase
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Vector1Type, class Vector2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DDyadProductOp<ValueType,
                TVectorConstRef<ValueType, Vector1Type, Shape1Type>,
                TVectorConstRef<ValueType, Vector2Type, Shape2Type> >,
            typename TVectorDyadProductShapePromotion<Shape1Type, Shape2Type>::type >
        dyad (
            const TVectorBase<ValueType, Vector1Type, Shape1Type>& f_op1_r,
            const TVectorBase<ValueType, Vector2Type, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //! dyad function overload:
        //! Generates dyad product of Vector and 1D Expression
        //! and returns 2D Expression
        //! @param f_op1_r     Object of TVectorBase
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DDyadProductOp<ValueType,
                TVectorConstRef<ValueType, VectorType, Shape1Type>,
                intern::TExp1D<ValueType, OperatorType, Shape2Type> >,
            typename TVectorDyadProductShapePromotion<Shape1Type, Shape2Type>::type >
        dyad (
            const TVectorBase<ValueType, VectorType, Shape1Type>& f_op1_r,
            const intern::TExp1D<ValueType, OperatorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //! dyad function overload:
        //! Generates dyad product of 1D Expression and Vector
        //! and returns 2D Expression
        //! @param f_op1_r     Object of TExp1D
        //! @param f_op2_r     Object of TVectorBase
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class VectorType, class OperatorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DDyadProductOp<ValueType,
                intern::TExp1D<ValueType, OperatorType, Shape1Type>,
                TVectorConstRef<ValueType, VectorType, Shape2Type> >,
            typename TVectorDyadProductShapePromotion<Shape1Type, Shape2Type>::type >
        dyad (
            const intern::TExp1D<ValueType, OperatorType, Shape1Type>& f_op1_r,
            const TVectorBase<ValueType, VectorType, Shape2Type>& f_op2_r);

        //---------------------------------------------------------------------
        //! dyad function overload:
        //! Generates dyad product of 1D Expression and 1D Expression
        //! and returns 2D Expression
        //! @param f_op1_r     Object of TExp1D
        //! @param f_op2_r     Object of TExp1D
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops1d2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Operator1Type, class Operator2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DDyadProductOp<ValueType,
                intern::TExp1D<ValueType, Operator1Type, Shape1Type>,
                intern::TExp1D<ValueType, Operator2Type, Shape2Type> >,
            typename TVectorDyadProductShapePromotion<Shape1Type, Shape2Type>::type >
        dyad (
            const intern::TExp1D<ValueType, Operator1Type, Shape1Type>& f_op1_r,
            const intern::TExp1D<ValueType, Operator2Type, Shape2Type>& f_op2_r);

        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg END
        //-------------------------------------------------------------------------
        /// @}
        //-------------------------------------------------------------------------

    }
}
#include "vfc/linalg/vfc_linalg_ops1d2d_common.inl"

#endif //VFC_LINALG_OPS1D2D_COMMON_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_ops1d2d_common.hpp  $
//  Revision 1.4 2011/01/21 12:53:29MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - sometimes .hpp or namespace qualifiers are missing (mantis3599)
//  Revision 1.3 2008/08/29 15:04:57MESZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.2 2008/07/31 14:08:39IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:23IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
