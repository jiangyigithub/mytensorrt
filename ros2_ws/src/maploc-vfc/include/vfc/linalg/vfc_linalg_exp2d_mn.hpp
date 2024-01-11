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
///     $Source: vfc_linalg_exp2d_mn.hpp $
///     $Revision: 1.5 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2013/05/16 11:22:28MESZ $
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

#ifndef VFC_LINALG_EXP2D_MN_HPP_INCLUDED
#define VFC_LINALG_EXP2D_MN_HPP_INCLUDED

#include "vfc/linalg/vfc_linalg_exp2d_common.hpp"
#include "vfc/linalg/vfc_linalg_shapepromotion.hpp"
#include "vfc/linalg/vfc_linalg_exp2d.hpp"

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
            // TExp2D
            //---------------------------------------------------------------------
            //! TExp2D class specialized for static matrices
            //! @param ValueType     Data Type
            //! @param OperatorType  Data Type
            //! @param RowValue      Number of rows
            //! @param ColValue      Number of column
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d_mn.hpp $
            //=====================================================================
            template <class ValueType, class OperatorType,
                    vfc::int32_t RowValue,  vfc::int32_t ColValue>
            class TExp2D<ValueType, OperatorType,
                TStaticRectangle<RowValue, ColValue> >
            {
            public:
                /// Value Type
                typedef ValueType                               value_type;
                /// Shape type
                typedef TStaticRectangle<RowValue, ColValue>    shape_type;
                /// Size type

                typedef vfc::int32_t                             size_type;

                enum { NB_ROWS = shape_type::NB_ROWS };
                enum { NB_COLUMNS = shape_type::NB_COLUMNS };

                //---------------------------------------------------------------------
                //! Constructs expression from given operator
                //! @param f_operand_r     2D Expresstion of type Expression1Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_mn.hpp $
                //---------------------------------------------------------------------
                inline TExp2D(const OperatorType& f_operator_r)
                    : m_operator(f_operator_r) { }

                //---------------------------------------------------------------------
                //! Evaluates operator for single element
                //! @param f_row     Number of rows
                //! @param f_col     Number of column
                //! @return Returns value at specific row and column
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_mn.hpp $
                //---------------------------------------------------------------------
                inline value_type operator()(size_type f_row, size_type f_col) const
                {
                    return m_operator.eval(f_row, f_col);
                }

                //---------------------------------------------------------------------
                //! Returns column dimension of resulting 2D expression
                //! @param void
                //! @return Returns column dimension of resulting 2D expression
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_mn.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbColumns() const { return NB_COLUMNS; }

                //---------------------------------------------------------------------
                //! Returns row dimension of resulting 2D expression
                //! @param void
                //! @return Returns row dimension of resulting 2D expression
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_mn.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbRows() const { return NB_ROWS; }

            private:
                TExp2D& operator=(const TExp2D<ValueType, OperatorType,
                    TStaticRectangle<RowValue, ColValue> >&);
                const OperatorType m_operator;  //!< operator wrapped in the expression to be evaluated
            };

            //=====================================================================
            // TExp2DMulOpImpl
            //---------------------------------------------------------------------
            //! 2D expression matrix multiplication operator implementation for static matrices
            //! @param ValueType        Data Type
            //! @param Expression1Type  Data Type
            //! @param Expression2Type  Data Type
            //! @param ShapeType        Data Type
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d_mn.hpp $
            //=====================================================================
            template<class ValueType, class Expression1Type,
                class Expression2Type, class ShapeType>
            class TExp2DMulOpImpl
            {
            public:
                /// Value Type
                typedef ValueType           value_type;
                /// Shape type
                typedef ShapeType           shape_type;
                /// Size type
                typedef vfc::int32_t         size_type;

                //---------------------------------------------------------------------
                //! Evaluates multiplication value single element
                //! @param f_operand1_r     Expression of type Expression1Type
                //! @param f_operand2_r     Expression of type Expression2Type
                //! @param f_row            Row number
                //! @param f_col            Column number
                //! @return Returns multiplication of f_operand1_r with f_operand2_r
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_mn.hpp $
                //---------------------------------------------------------------------
                 static ValueType eval (
                    const Expression1Type& f_operand1_r,
                    const Expression2Type& f_operand2_r,
                    size_type f_row, size_type f_col);
            };

            //=====================================================================
            // TExp2DSubMatrixOp
            //---------------------------------------------------------------------
            //! Subamtrix expression operator for static matrices
            //! @param ValueType        Data Type
            //! @param ExpressionType   Data Type
            //! @param RowValue         Numbre of rows
            //! @param ColValue         Number of columns
            //! @author  jat2hi
            //! @ingroup vfc_group_linalg
            //! $Source: vfc_linalg_exp2d_mn.hpp $
            //=====================================================================
            template <class ValueType, class ExpressionType,
                vfc::int32_t RowValue, vfc::int32_t ColValue>
            class TExp2DSubMatrixOp<ValueType, ExpressionType,
                TStaticRectangle<RowValue, ColValue> >
            {
            public:
                /// Value Type
                typedef ValueType                               value_type;
                /// Shape type
                typedef TStaticRectangle<RowValue, ColValue>    shape_type;
                /// Size type
                typedef vfc::int32_t                             size_type;

                enum { NB_ROWS = shape_type::NB_ROWS };
                enum { NB_COLUMNS = shape_type::NB_COLUMNS };

                //---------------------------------------------------------------------
                //! Saves refernce to the operand, asserts if operand dimensions does not fit
                //! @param f_operand_r     2D Expresstion of type Expression1Type
                //! @param f_row0          Number of rows
                //! @param f_column0       Number of columns
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_mn.hpp $
                //---------------------------------------------------------------------
                inline TExp2DSubMatrixOp(const ExpressionType& f_operand_r,
                    size_type f_row0, size_type f_column0)
                        : m_expression(f_operand_r), m_nbRowZero(f_row0),
                            m_nbColumnZero(f_column0)
                {
                    VFC_REQUIRE( (f_row0 + NB_ROWS <= f_operand_r.getNbRows()) && \
                        (f_column0 + NB_COLUMNS <= f_operand_r.getNbColumns()) );
                }

                //---------------------------------------------------------------------
                //! Evaluates the value of the submatrix for single element. asserts if the dimension is not OK
                //! @param f_row          Number of rows
                //! @param f_column       Number of columns
                //! @return Returns multiplication of f_operand1_r with f_operand2_r
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_mn.hpp $
                //---------------------------------------------------------------------
                inline value_type eval (size_type f_row, size_type f_col)  const
                {
                    VFC_REQUIRE(f_row < NB_ROWS && f_col < NB_COLUMNS);
                    return m_expression(f_row + m_nbRowZero,
                        f_col + m_nbColumnZero);
                }

                //---------------------------------------------------------------------
                //! Returns number of rows
                //! @param void
                //! @return Returns number of rows
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_mn.hpp $
                //---------------------------------------------------------------------
                inline size_type  getNbRows(void) const { return NB_ROWS; }

                //---------------------------------------------------------------------
                //! Returns number of columns
                //! @param void
                //! @return Returns number of columns
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp2d_mn.hpp $
                //---------------------------------------------------------------------
                inline size_type  getNbColumns(void) const { return NB_COLUMNS; }

            private:
                const ExpressionType m_expression;  //!< reference to expression
                size_type m_nbRowZero;              //!< 0th row position
                size_type m_nbColumnZero;           //!< 0th column position

            };

        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed


}   // namespace vfc closed

#include "vfc/linalg/vfc_linalg_exp2d_mn.inl"

#endif //VFC_LINALG_EXP2D_MN_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_exp2d_mn.hpp  $
//  Revision 1.5 2013/05/16 11:22:28MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc linalg submatrix: assertion is too strict (mantis4254)
//  Revision 1.4 2011/01/21 12:53:28MEZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - sometimes .hpp or namespace qualifiers are missing (mantis3599)
//  Revision 1.3 2009/03/25 14:48:20MEZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.2 2008/07/31 14:08:22IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:19IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
