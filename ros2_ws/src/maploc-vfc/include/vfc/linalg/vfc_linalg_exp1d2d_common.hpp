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
///     $Source: vfc_linalg_exp1d2d_common.hpp $
///     $Revision: 1.4 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2011/01/21 12:53:33MEZ $
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

#ifndef VFC_LINALG_EXP1D2D_COMMON_HPP_INCLUDED
#define VFC_LINALG_EXP1D2D_COMMON_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_assert.hpp"
#include "vfc/linalg/vfc_linalg_shapepromotion.hpp"

namespace vfc
{   // namespace vfc open

    namespace linalg
    {   // namespace linalg open

        //-------------------------------------------------------------------------
        // conditional doxygen documentation
        //! @cond VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

        namespace intern
        {   // namespace intern open

            template<class ValueType, class Expression1Type, class Expression2Type, class ShapeType>
            class TExp2D1DMulOpImpl;

            //=============================================================================
            // TExp2D1DMulOp
            //-----------------------------------------------------------------------------
            //! Operator class to perform 2D * 1D
            //! @param ValueType          Data Type
            //! @param Expression1Type    Data Type
            //! @param Expression2Type    Data Type
            //! @ingroup vfc_group_linalg
            //! @author  jat2hi
            //! $Source: vfc_linalg_exp1d2d_common.hpp $
            //=============================================================================
            template<class ValueType, class Expression1Type, class Expression2Type>
            class TExp2D1DMulOp
            {
            public:

                //---------------------------------------------------------------------
                //! Constructor for the class TExp2D1DMulOp
                //! @param f_operand1_r    2D Expression of type Expression1Type
                //! @param f_operand2_r    1D Expression of type Expression2Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d_common.hpp $
                //---------------------------------------------------------------------
                inline
                TExp2D1DMulOp(const Expression1Type& f_operand1_r, const Expression2Type& f_operand2_r)
                :   m_operand1(f_operand1_r),
                    m_operand2(f_operand2_r)
                {
                    VFC_REQUIRE(m_operand1.getNbColumns() == m_operand2.getDim());
                }

                //---------------------------------------------------------------------
                //! Function for multiplication of 2D and 1D expression
                //! @param f_row    Number of rows
                //! @return  Returns multiplication of 2D and 1D expression
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d_common.hpp $
                //---------------------------------------------------------------------
                inline
                ValueType eval(vfc::int32_t f_row) const
                {
                    return TExp2D1DMulOpImpl<ValueType, Expression1Type, Expression2Type,
                        typename vfc::linalg::TExp2D1DMulShapePromotion<typename Expression1Type::shape_type,
                                                        typename Expression2Type::shape_type>::type >
                                                        ::eval(m_operand1,
                                                                m_operand2,
                                                                f_row);
                }

                //---------------------------------------------------------------------
                //! Function to get number of rows
                //! @param void
                //! @return  Returns number of rows
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d_common.hpp $
                //---------------------------------------------------------------------
                inline vfc::int32_t getDim() const
                {
                    return m_operand1.getNbRows();
                }
            private:
                const Expression1Type m_operand1;
                const Expression2Type m_operand2;
            };

            //=============================================================================
            // TExp2DDyadProductOp
            //---------------------------------------------------------------------
            //! Dyad Product operator for 1D Expressions,Generates a 2D expression
            //! @param ValueType          Data Type
            //! @param Expression1Type    Data Type
            //! @param Expression2Type    Data Type
            //! @ingroup vfc_group_linalg
            //! @author  jat2hi
            //! $Source: vfc_linalg_exp1d2d_common.hpp $
            //=============================================================================
            template <class ValueType, class Expression1Type, class Expression2Type>
            class TExp2DDyadProductOp
            {
            public:
                /// Value Type
                typedef ValueType                                           value_type;
                /// Shape Type
                typedef typename vfc::linalg::TVectorDyadProductShapePromotion<
                            typename Expression1Type::shape_type,
                            typename Expression2Type::shape_type>::type     shape_type;

                /// Size Type
                typedef vfc::int32_t                                        size_type;

                //---------------------------------------------------------------------
                //! Constructor for the class TExp2DDyadProductOp
                //! Saves references to the operands
                //! Asserts if the inputs do not have valid dimensions
                //! @param f_operand1_r    Expression of type Expression1Type
                //! @param f_operand2_r    Expression of type Expression2Type
                //! $Source: vfc_linalg_exp1d2d_common.hpp $
                //---------------------------------------------------------------------
                inline TExp2DDyadProductOp(const Expression1Type& f_operand1_r, const Expression2Type& f_operand2_r)
                    : m_operand1(f_operand1_r), m_operand2(f_operand2_r)
                {
                }

                //---------------------------------------------------------------------
                //! Evaluates the Dyad product of the vector to generate
                //! a single element element in the Resultant matrix
                //! @param f_row    Number of rows
                //! @param f_col    Number of columns
                //! @return  Returns Dyad produc of expressions
                //! $Source: vfc_linalg_exp1d2d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_row, size_type f_col) const
                {
                    return m_operand1[f_row] * m_operand2[f_col];
                }

                //---------------------------------------------------------------------
                //! Returns the number of rows in the Resultant matrix
                //! @param void
                //! @return  Returns number of rows
                //! $Source: vfc_linalg_exp1d2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbRows() const
                {
                    return m_operand1.getDim();
                }

                //---------------------------------------------------------------------
                //! Returns the number of columns in the resultant matrix
                //! @param void
                //! @return  Returns number of columns
                //! $Source: vfc_linalg_exp1d2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getNbColumns() const
                {
                    return m_operand2.getDim();
                }
            private:
                TExp2DDyadProductOp& operator=(
                    const TExp2DDyadProductOp<ValueType, Expression1Type, Expression2Type>&);
                const Expression1Type m_operand1;
                const Expression2Type m_operand2;
            };

            //=============================================================================
            // TExp1DSubViewFrom2DOp
            //---------------------------------------------------------------------
            //! SubView operator for 2D Expressions
            //! @param ValueType          Data Type
            //! @param ExpressionType     Data Type
            //! @param ShapeType          Data Type
            //! @ingroup vfc_group_linalg
            //! @author  jat2hi
            //! $Source: vfc_linalg_exp1d2d_common.hpp $
            //=============================================================================
            template <class ValueType, class ExpressionType, class ShapeType>
            class TExp1DSubViewFrom2DOp
            {
            public:

                /// Value Type
                typedef ValueType                                           value_type;
                /// Shape Type
                typedef typename ExpressionType::shape_type                 shape_type;

                /// Size Type
                typedef vfc::int32_t                                        size_type;

                //---------------------------------------------------------------------
                //! Constructor for the class TExp1DSubViewFrom2DOp
                //! Saves references to the operands
                //! Asserts if the inputs do not have valid dimensions
                //! @param f_operand_r    Expression of type ExpressionType
                //! @param f_row0         Number of rows
                //! @param f_column0      Number of columns
                //! @param f_length
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d_common.hpp $
                //---------------------------------------------------------------------
                inline
                TExp1DSubViewFrom2DOp(const ExpressionType& f_operand_r,
                    size_type f_row0, size_type f_column0, size_type f_length) :
                    m_operand(f_operand_r),
                    m_nbStartPos(f_row0*f_operand_r.getNbColumns()+f_column0),
                    m_length(f_length)
                {
                    VFC_REQUIRE(0 <= f_row0 && 0 <= f_column0);

                    //check if the max size required is acceptanle
                    VFC_REQUIRE(f_length <= ((f_operand_r.getNbRows() *
                        f_operand_r.getNbColumns()) - m_nbStartPos));
                }

                //---------------------------------------------------------------------
                //! Returns a single element at the given position in the subview
                //! @param f_pos    Position of single element
                //! @return  Returns a single element at the given position in the subview
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_pos) const
                {
                    VFC_REQUIRE(f_pos < m_length && 0 <= f_pos);
                    size_type row = (m_nbStartPos + f_pos)/m_operand.getNbColumns();
                    size_type col = (m_nbStartPos + f_pos)%m_operand.getNbColumns();
                    return m_operand(row, col);
                }

                //---------------------------------------------------------------------
                //! Returns the dimension of the subview expression
                //! @param void
                //! @return  Returns the dimension of the subview expression
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d2d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getDim(void) const   { return m_length; }

            private:
                TExp1DSubViewFrom2DOp& operator=(
                    const TExp1DSubViewFrom2DOp<ValueType, ExpressionType, ShapeType>&);

                /// Operand
                const ExpressionType m_operand;     //!< reference to expression
                size_type m_nbStartPos;             //!< start position in the expression
                size_type m_length;                 //!< size of the 1d expression
            };

        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed

#endif //VFC_LINALG_EXP1D2D_COMMON_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_exp1d2d_common.hpp  $
//  Revision 1.4 2011/01/21 12:53:33MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - sometimes .hpp or namespace qualifiers are missing (mantis3599)
//  Revision 1.3 2009/03/25 14:48:16MEZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.2 2008/07/31 14:08:16IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:19IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
