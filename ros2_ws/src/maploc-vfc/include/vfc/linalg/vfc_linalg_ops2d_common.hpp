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
///     $Source: vfc_linalg_ops2d_common.hpp $
///     $Revision: 1.7 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2011/01/21 12:53:27MEZ $
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

#ifndef VFC_LINALG_OPS2D_COMMON_HPP_INCLUDED
#define VFC_LINALG_OPS2D_COMMON_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/linalg/vfc_linalg_shapepromotion.hpp"
#include "vfc/linalg/vfc_linalg_exp2d_common.hpp"
#include <functional>

// forward declarations
namespace vfc
{
    namespace linalg
    {
        template<class ValueType, class DerivedType, class ShapeType>
        class TMatrixBase;

        template <class ValueType, class DerivedType, class ShapeType>
        class TMatrixConstRef;
    }
}

namespace vfc
{   // namespace vfc opened

    namespace linalg
    {   // namespace linalg opened

        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg BEGIN
        //-------------------------------------------------------------------------
        /// @addtogroup vfc_group_linalg
        /// @{
        //-------------------------------------------------------------------------

        //---------------------------------------------------------------------
        //! Operator overloading for matrix + matrix addition, returns resulting 2d expression.
        //! asserts if matrix shapes does not match.
        //! @param f_operand1_r     Object of TMatrixBase
        //! @param f_operand2_r     Object of TMatrixBase
        //! @return  Returns 2D Expression of matrix + matrix addition .
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Matrix1Type, class Matrix2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DAddOp<ValueType,
                TMatrixConstRef<ValueType, Matrix1Type, Shape1Type>,
                TMatrixConstRef<ValueType, Matrix2Type, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator+ (const TMatrixBase<ValueType, Matrix1Type, Shape1Type>& f_operand1_r,
            const TMatrixBase<ValueType, Matrix2Type, Shape2Type>& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading for matrix + expression addition, returns resulting 2d expression
        //! asserts if matrix shapes does not match.
        //! @param f_operand1_r     Object of TMatrixBase
        //! @param f_operand2_r     Object of TExp2D
        //! @return  Returns 2D Expression of matrix + 2D expression addition .
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class MatrixType, class OperatorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DAddOp<ValueType,
                TMatrixConstRef<ValueType, MatrixType, Shape1Type>,
                intern::TExp2D<ValueType, OperatorType, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator+ (const TMatrixBase<ValueType, MatrixType, Shape1Type>& f_operand1_r,
            const intern::TExp2D<ValueType, OperatorType, Shape2Type>& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading for expression + matrix addition, returns resulting 2d expression
        //! asserts if matrix shapes does not match.
        //! @param f_operand1_r     Object of TExp2D
        //! @param f_operand2_r     Object of TMatrixBase
        //! @return  Returns 2D Expression of 2D expression + matrix addition .
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class MatrixType, class OperatorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DAddOp<ValueType,
                intern::TExp2D<ValueType, OperatorType, Shape1Type>,
                TMatrixConstRef<ValueType, MatrixType, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator+ (const intern::TExp2D<ValueType, OperatorType, Shape1Type>& f_operand1_r,
            const TMatrixBase<ValueType, MatrixType, Shape2Type>& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading for expression + expression addition, returns resulting 2d expression
        //! asserts if matrix shapes does not match.
        //! @param f_operand1_r     Object of TExp2D
        //! @param f_operand2_r     Object of TExp2D
        //! @return  Returns 2D Expression of 2D expression + 2D expression addition .
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Operator1Type, class Operator2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DAddOp<ValueType,
                intern::TExp2D<ValueType, Operator1Type, Shape1Type>,
                intern::TExp2D<ValueType, Operator2Type, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator+ (const intern::TExp2D<ValueType, Operator1Type, Shape1Type>& f_operand1_r,
            const intern::TExp2D<ValueType, Operator2Type, Shape2Type>& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading for matrix - matrix subtraction, returns resulting 2d expression
        //! asserts if matrix shapes does not match.
        //! @param f_operand1_r     Object of TMatrixBase
        //! @param f_operand2_r     Object of TMatrixBase
        //! @return  Returns 2D Expression of matrix - matrix subtraction .
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Matrix1Type, class Matrix2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DSubOp<ValueType,
                TMatrixConstRef<ValueType, Matrix1Type, Shape1Type>,
                TMatrixConstRef<ValueType, Matrix2Type, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator- (const TMatrixBase<ValueType, Matrix1Type, Shape1Type>& f_operand1_r,
            const TMatrixBase<ValueType, Matrix2Type, Shape2Type>& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading for matrix - expression subtraction, returns resulting 2d expression
        //! asserts if matrix shapes does not match.
        //! @param f_operand1_r     Object of TMatrixBase
        //! @param f_operand2_r     Object of TExp2D
        //! @return  Returns 2D Expression of matrix - 2D expression subtraction .
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class MatrixType, class OperatorType, class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DSubOp<ValueType,
                TMatrixConstRef<ValueType, MatrixType, Shape1Type>,
                intern::TExp2D<ValueType, OperatorType, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator- (const TMatrixBase<ValueType, MatrixType, Shape1Type>& f_operand1_r,
            const intern::TExp2D<ValueType, OperatorType, Shape2Type>& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading for expression - matrix subtraction, returns resulting 2d expression
        //! asserts if matrix shapes does not match.
        //! @param f_operand1_r     Object of TExp2D
        //! @param f_operand2_r     Object of TMatrixBase
        //! @return  Returns 2D Expression of 2D expression - matrix subtraction .
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
       template <class ValueType, class MatrixType, class OperatorType,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DSubOp<ValueType,
                intern::TExp2D<ValueType, OperatorType, Shape1Type>,
                TMatrixConstRef<ValueType, MatrixType, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator- (const intern::TExp2D<ValueType, OperatorType, Shape1Type>& f_operand1_r,
            const TMatrixBase<ValueType, MatrixType, Shape2Type>& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading for expression - expression subtraction, returns resulting 2d expression
        //! asserts if matrix shapes does not match.
        //! @param f_operand1_r     Object of TMatrixBase
        //! @param f_operand2_r     Object of TMatrixBase
        //! @return  Returns 2D Expression of 2D expression - 2D expression subtraction .
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class Operator1Type, class Operator2Type,
            class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DSubOp<ValueType,
                intern::TExp2D<ValueType, Operator1Type, Shape1Type>,
                intern::TExp2D<ValueType, Operator2Type, Shape2Type> >,
            typename TBinaryShapePromotion<Shape1Type, Shape2Type>::type >
        operator- (const intern::TExp2D<ValueType, Operator1Type, Shape1Type>& f_operand1_r,
            const intern::TExp2D<ValueType, Operator2Type, Shape2Type>& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading for unary operator- (matrix negate), returns resulting 2d expression
        //! @param f_operand_r     Object of TMatrixBase
        //! @return  Returns 2D Expression of matrix negate.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
       template <class ValueType, class MatrixType, class ShapeType>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DNegateOp<ValueType,
                TMatrixConstRef<ValueType, MatrixType, ShapeType> >,
            typename MatrixType::shape_type>
        operator- (const TMatrixBase<ValueType, MatrixType, ShapeType>& f_operand_r);

        //---------------------------------------------------------------------
        //! Operator overloading for unary operator- (expression negate), returns resulting 2d expression
        //! @param f_operand_r     Object of TExp2D
        //! @return  Returns 2D Expression of 2D expression negate.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DNegateOp<ValueType,
                intern::TExp2D<ValueType, OperatorType, ShapeType> >,
            ShapeType>
        operator- (const intern::TExp2D<ValueType, OperatorType, ShapeType>& f_operand_r);

        //---------------------------------------------------------------------
        //! Transpose operation on matrix, returns resulting 2d expression
        //! @param f_operand_r     Object of TMatrixBase
        //! @return  Returns 2D Expression .
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class MatrixType, class ShapeType>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DTransposeOp<ValueType,
                TMatrixConstRef<ValueType, MatrixType, ShapeType> >,
            typename TTransposeMatrixShapePromotion<typename MatrixType::shape_type>::shape_type>
        transpose (const TMatrixBase<ValueType, MatrixType, ShapeType>& f_operand_r);

        //---------------------------------------------------------------------
        //! Transpose operation on expression, returns resulting 2d expression
        //! @param f_operand_r     Object of TExp2D
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DTransposeOp<ValueType,
                intern::TExp2D<ValueType, OperatorType, ShapeType> >,
            typename TTransposeMatrixShapePromotion<ShapeType>::shape_type>
        transpose (const intern::TExp2D<ValueType, OperatorType, ShapeType>& f_operand_r);

        //---------------------------------------------------------------------
        //! Operator overloading for mat*scalar multiplication, returns resulting 2d expression
        //! @param f_operand1_r     Object of TMatrixBase
        //! @param f_operand2_r     Scalar value for multiplication
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
       template <class ValueType, class MatrixType, class ShapeType>
        const
        intern::TExp2D <ValueType,
            intern::TExp2DScalarOp<ValueType,
                TMatrixConstRef<ValueType, MatrixType, ShapeType>,
                stlalias::multiplies<ValueType> > ,
            typename MatrixType::shape_type>
        operator* (const TMatrixBase<ValueType, MatrixType,ShapeType>& f_operand1_r,
            const ValueType& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading for scalar*mat multiplication, returns resulting 2d expression
        //! @param f_operand1_r     Scalar value for multiplication
        //! @param f_operand2_r     Object of TMatrixBase
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class MatrixType, class ShapeType>
        const
        intern::TExp2D <ValueType,
            intern::TExp2DScalarOp<ValueType,
                TMatrixConstRef<ValueType, MatrixType, ShapeType>,
                stlalias::multiplies<ValueType> > ,
            typename MatrixType::shape_type>
        operator* (const ValueType& f_operand1_r,
            const TMatrixBase<ValueType, MatrixType,ShapeType>& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading for exp*scalar multiplication, returns resulting 2d expression
        //! @param f_operand1_r     Object of TExp2D
        //! @param f_operand2_r     Scalar value for multiplication
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        const
        intern::TExp2D <ValueType,
            intern::TExp2DScalarOp<ValueType,
                intern::TExp2D<ValueType, OperatorType, ShapeType>,
                stlalias::multiplies<ValueType> > ,
            ShapeType>
        operator* (const intern::TExp2D<ValueType, OperatorType, ShapeType>& f_operand1_r,
            const ValueType& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading for scalar*exp multiplication, returns resulting 2d expression
        //! @param f_operand1_r     Scalar value for multiplication
        //! @param f_operand2_r     Object of TExp2D
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        const
        intern::TExp2D <ValueType,
            intern::TExp2DScalarOp<ValueType,
                intern::TExp2D<ValueType, OperatorType, ShapeType>,
                stlalias::multiplies<ValueType> > ,
            ShapeType>
        operator* (const ValueType& f_operand1_r,
            const intern::TExp2D<ValueType, OperatorType, ShapeType>& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading for mat/scalar division, returns resulting 2d expression
        //! @param f_operand1_r     Object of TMatrixBase
        //! @param f_operand2_r     Scalar value for division
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
       template <class ValueType, class MatrixType, class ShapeType>
        const
        intern::TExp2D <ValueType,
            intern::TExp2DDivScalarOp<ValueType,
                TMatrixConstRef<ValueType, MatrixType, ShapeType>,
                stlalias::divides<ValueType> > ,
            typename MatrixType::shape_type>
        operator/ (const TMatrixBase<ValueType, MatrixType,ShapeType>& f_operand1_r,
            const ValueType& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading for exp/scalar division, returns resulting 2d expression
        //! @param f_operand1_r     Object of TExp2D
        //! @param f_operand2_r     Scalar value for divission
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class OperatorType, class ShapeType>
        const
        intern::TExp2D <ValueType,
            intern::TExp2DDivScalarOp<ValueType,
                intern::TExp2D<ValueType, OperatorType, ShapeType>,
                stlalias::divides<ValueType> > ,
            ShapeType>
        operator/ (const intern::TExp2D<ValueType, OperatorType, ShapeType>& f_operand1_r,
            const ValueType& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading performs mat*mat multiplication, returns resulting 2d expression
        //! asserts if matrix shapes does not match.
        //! @param f_operand1_r     Object of TMatrixBase
        //! @param f_operand2_r     Object of TMatrixBase
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
       template <class ValueType, class Matrix1Type, class Matrix2Type, class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DMulOp<ValueType,
                TMatrixConstRef<ValueType, Matrix1Type, Shape1Type>,
                TMatrixConstRef<ValueType, Matrix2Type, Shape2Type> >,
            typename TMulShapePromotion<Shape1Type, Shape2Type>::type>
        operator* (const TMatrixBase<ValueType ,Matrix1Type, Shape1Type>& f_operand1_r,
            const TMatrixBase<ValueType ,Matrix2Type, Shape2Type>& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading performs mat*expr multiplication, returns resulting 2d expression
        //! asserts if matrix shapes does not match.
        //! @param f_operand1_r     Object of TMatrixBase
        //! @param f_operand2_r     Object of TExp2D
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class MatrixType, class OperatorType, class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DMulOp<ValueType,
                TMatrixConstRef<ValueType, MatrixType, Shape1Type>,
                intern::TExp2D<ValueType, OperatorType, Shape2Type> >,
            typename TMulShapePromotion<Shape1Type, Shape2Type>::type>
        operator* (const TMatrixBase<ValueType, MatrixType, Shape1Type>& f_operand1_r,
            const intern::TExp2D<ValueType, OperatorType, Shape2Type>& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading performs mat*expr multiplication, returns resulting 2d expression
        //! asserts if matrix shapes does not match.
        //! @param f_operand1_r     Object of TExp2D
        //! @param f_operand2_r     Object of TMatrixBase
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
        template <class ValueType, class MatrixType, class OperatorType, class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DMulOp<ValueType,
                intern::TExp2D<ValueType, OperatorType, Shape1Type>,
                TMatrixConstRef<ValueType, MatrixType, Shape2Type> >,
            typename TMulShapePromotion<Shape1Type, Shape2Type>::type>
        operator* (const intern::TExp2D<ValueType, OperatorType, Shape1Type>& f_operand1_r,
            const TMatrixBase<ValueType, MatrixType, Shape2Type>& f_operand2_r);

        //---------------------------------------------------------------------
        //! Operator overloading performs expr*expr multiplication, returns resulting 2d expression
        //! asserts if matrix shapes does not match.
        //! @param f_operand1_r     Object of TExp2D
        //! @param f_operand2_r     Object of TExp2D
        //! @return  Returns 2D Expression.
        //! @author  jat2hi
        //! $Source: vfc_linalg_ops2d_common.hpp $
        //---------------------------------------------------------------------
      template <class ValueType, class Operator1Type, class Operator2Type, class Shape1Type, class Shape2Type>
        const
        intern::TExp2D<ValueType,
            intern::TExp2DMulOp<ValueType,
                intern::TExp2D<ValueType, Operator1Type, Shape1Type>,
                intern::TExp2D<ValueType, Operator2Type, Shape2Type> >,
            typename TMulShapePromotion<Shape1Type, Shape2Type>::type>
        operator* (const intern::TExp2D<ValueType, Operator1Type, Shape1Type>& f_operand1_r,
            const intern::TExp2D<ValueType, Operator2Type, Shape2Type>& f_operand2_r);

        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg END
        //-------------------------------------------------------------------------
        /// @}
        //-------------------------------------------------------------------------

    }   // namespace linalg closed


}   // namespace vfc closed

#include "vfc/linalg/vfc_linalg_ops2d_common.inl"

#endif //VFC_LINALG_OPS2D_COMMON_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_ops2d_common.hpp  $
//  Revision 1.7 2011/01/21 12:53:27MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - sometimes .hpp or namespace qualifiers are missing (mantis3599)
//  Revision 1.6 2009/05/28 09:19:03MESZ Muehlmann Karsten (CC/ESV2) (MUK2LR) 
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.5 2008/08/29 15:04:59CEST Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.4 2008/07/31 14:08:42IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.3 2007/07/02 16:43:22IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  - moved definition of the dynamic submatrix() op from the common files to the dynamic specific files (mantis 1726)
//  Revision 1.2 2007/06/22 15:10:00CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - corrected redeclared function parameter names (mantis 1692)
//  Revision 1.1 2007/05/09 10:21:25CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
