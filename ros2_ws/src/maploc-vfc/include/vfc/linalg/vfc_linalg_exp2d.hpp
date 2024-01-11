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
///     $Source: vfc_linalg_exp2d.hpp $
///     $Revision: 1.5 $
///     $Author: gaj2kor $
///     $Date: 2009/03/25 14:48:21MEZ $
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

#ifndef VFC_LINALG_EXP2D_HPP_INCLUDED
#define VFC_LINALG_EXP2D_HPP_INCLUDED

#include "vfc/linalg/vfc_linalg_exp2d_common.hpp"
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

            //=====================================================================
            // TExp2DMulOpImpl
            //---------------------------------------------------------------------
            //! 2D expression matrix multiplication operator implementation for dynamic matrices
            //! @param ValueType           Data Type
            //! @param Expression1Type     Data Type
            //! @param Expression2Type     Data Type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d.hpp $
            //=====================================================================
            template<class ValueType, class Expression1Type, class Expression2Type>
            class TExp2DMulOpImpl<ValueType, Expression1Type, Expression2Type,
                CDynamicRectangle>
            {
            public:
                /// Value Type
                typedef ValueType           value_type;
                /// Shape type
                typedef CDynamicRectangle   shape_type;
                /// Size type
                typedef vfc::int32_t         size_type;

                //---------------------------------------------------------------------
                //! Evaluates multiplication value single element
                //! @param f_operand1_r         Expression Data of type Expression2Type
                //! @param f_operand1_r         Expression Data of type Expression2Type
                //! @param f_row                Row number
                //! @param f_col                Column number
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d.hpp $
                //---------------------------------------------------------------------
                static
                ValueType eval (
                    const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r,
                    size_type f_row, size_type f_col);
            };

            //=====================================================================
            // TExp2DSubMatrixOp
            //---------------------------------------------------------------------
            //! 2D expression subamtrix operator
            //! @param ValueType            Data Type
            //! @param ExpressionType       Data Type
            //! @param ShapeType            Data Type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d.hpp $
            //=====================================================================
            template <class ValueType, class ExpressionType, class ShapeType>
            class TExp2DSubMatrixOp
            {
            public:
                /// Value Type
                typedef ValueType                       value_type;
                /// Shape type of the sub matrix
                typedef ShapeType                       shape_type;
                /// Size type
                typedef vfc::int32_t                     size_type;

                //---------------------------------------------------------------------
                //! Saves references to the operands
                //! Asserts if the inputs do not have valid dimensions
                //! @param f_expression_r            Expression of type ExpressionType
                //! @param f_row0                    Number of rows
                //! @param f_column0                 Number of columns
                //! @param f_row1                    Number of rows
                //! @param f_column1                 Number of columns
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d.hpp $
                //---------------------------------------------------------------------
                inline TExp2DSubMatrixOp(const ExpressionType& f_expression_r,
                    size_type f_row0, size_type f_column0,
                    size_type f_row1, size_type f_column1)
                        : m_expression(f_expression_r), m_nbRowZero(f_row0),
                        m_nbColumnZero(f_column0), m_nbRows(f_row1-f_row0+1),
                        m_nbColumns(f_column1-f_column0+1)
                {
                    VFC_REQUIRE( f_row0 >= 0 && f_column0 >= 0 && \
                        f_row1 < f_expression_r.getNbRows() && \
                        f_column1 < f_expression_r.getNbColumns() );
                }

                //---------------------------------------------------------------------
                //! Evaluates the value of the submatrix for single element.
                //! Asserts if the dimension is not OK
                //! @param f_row        Number of rows
                //! @param f_column     Number of column
                //! @returns Returns the value of the submatrix for single element.
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_row, size_type f_col)  const
                {
                    VFC_REQUIRE(f_row < m_nbRows && f_col < m_nbColumns);
                    return m_expression(f_row + m_nbRowZero, f_col + m_nbColumnZero);
                }

                //---------------------------------------------------------------------
                //! Returns column dimension of resulting 2D expression
                //! @param void
                //! @returns Returns column dimension of resulting 2D expression
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbColumns() const { return m_nbColumns;}

                //---------------------------------------------------------------------
                //! Returns row dimension of resulting 2D expression
                //! @param void
                //! @return Returns row dimension of resulting 2D expression
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbRows() const { return m_nbRows;}
            private:
                const ExpressionType m_expression;  //!< reference to expression
                size_type m_nbRowZero;              //!< 0th row position
                size_type m_nbColumnZero;           //!< 0th column position
                size_type m_nbRows;                 //!< Number of rows
                size_type m_nbColumns;              //!< Number of columns

            };

        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace linalg closed

#include "vfc/linalg/vfc_linalg_exp2d.inl"

#endif //VFC_LINALG_EXP2D_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_exp2d.hpp  $
//  Revision 1.5 2009/03/25 14:48:21MEZ gaj2kor 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.4 2008/08/26 09:34:45IST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  -Range check of TExp2DSubMatrixOp updated
//  (Mantis 2272)
//  Revision 1.3 2008/07/31 14:08:23IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.2 2007/07/02 16:25:24IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  - moved the definition of the dynamic TExp2DSubMatrixOp to the dynamic specific file vfc_linalg_exp2d.hpp (mantis 1725)
//  Revision 1.1 2007/05/09 10:21:19CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
