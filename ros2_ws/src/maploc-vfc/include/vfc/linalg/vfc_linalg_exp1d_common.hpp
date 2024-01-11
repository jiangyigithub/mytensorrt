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
///     $Source: vfc_linalg_exp1d_common.hpp $
///     $Revision: 1.6 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2011/01/21 12:53:34MEZ $
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

#ifndef VFC_LINALG_EXP1D_COMMON_HPP_INCLUDED
#define VFC_LINALG_EXP1D_COMMON_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_assert.hpp"
#include "vfc/core/vfc_math.hpp"
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

            //=============================================================================
            // TExp1D
            //-----------------------------------------------------------------------------
            //! Template container for 1d expression, wraps 1D operators
            //! @param ValueType        Data Type
            //! @param OperatorType     Data Type for dynamic 1D Expression
            //! @param ShapeType        Data Type for dynamic 1D Expression
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_common.hpp $
            //=============================================================================
            template <class ValueType, class OperatorType, class ShapeType>
            class TExp1D
            {
            public:
                /// Value Type
                typedef ValueType       value_type;
                /// Shape Type
                typedef ShapeType       shape_type;
                /// Size Type
                typedef vfc::int32_t    size_type;

                /// Number of elements in the Vector
                enum { NB_ROWS = ShapeType::NB_ROWS };

                //---------------------------------------------------------------------
                //! Constructor to construct an expression from an operator
                //! @param f_operator_r       Data of type OperatorType
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline
                TExp1D(const OperatorType& f_operator_r) : m_operator(f_operator_r) {}

                //---------------------------------------------------------------------
                //! Evaluates [] operator for an element of a vector
                //! @param f_row       Row number
                //! @return  Returns the value of vector at index number
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type operator[](size_type f_row) const
                {
                    return m_operator.eval(f_row);
                }

                //---------------------------------------------------------------------
                //! Function which returns the dimension of the 1D expression
                //! @param void
                //! @return  Returns the dimension of the 1D expression
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getDim() const
                {
                    return m_operator.getDim();
                }

            private:
                /// Operation to be evaluated
                const OperatorType m_operator;
                //Dummy function for suppression of QAC++ msg 2141
                //Msg 2141 : The subscript 'operator []' is not available in a non-const version.
                value_type operator[](size_type f_row);

            };

            //=============================================================================
            // TExp1DAddOp
            //-----------------------------------------------------------------------------
            //! Add operator for 1D expressions
            //! @param ValueType         Data Type
            //! @param Expression1Type   Data Type for 1D Expression
            //! @param Expression2Type   Data Type for 1D Expression
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_common.hpp $
            //=============================================================================
            template <class ValueType, class Expression1Type, class Expression2Type>
            class TExp1DAddOp
            {
            public:
                /// Value Type
                typedef ValueType                                   value_type;

                /// Shape Type after shape promotion
                typedef typename vfc::linalg::TBinaryShapePromotion<
                    typename Expression1Type::shape_type,
                    typename Expression2Type::shape_type>::type     shape_type;

                /// Size Type
                typedef vfc::int32_t                                size_type;

                //---------------------------------------------------------------------
                //! Constructor
                //! Saves reference to the operands
                //! Asserts if the Dimension of the inputs is invalid.
                //! @param f_operand1_r       Expression Data of type Expression1Type
                //! @param f_operand2_r       Expression Data of type Expression2Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline
                TExp1DAddOp(const Expression1Type& f_operand1_r, const Expression2Type& f_operand2_r)
                    : m_operand1(f_operand1_r), m_operand2(f_operand2_r)
                {
                    VFC_REQUIRE(m_operand1.getDim() == m_operand2.getDim());
                }

                //---------------------------------------------------------------------
                //! Performs addition operation for a single element.
                //! @param f_row       Row number
                //! @return  Returns the addition of m_operand1 & m_operand2 at index number
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_row) const
                {
                    return m_operand1[f_row] + m_operand2[f_row];
                }

                //---------------------------------------------------------------------
                //! Returns the Dimension of the Operands
                //! @param void
                //! @return  Returns the Dimension of the Operands
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getDim() const
                {
                    return m_operand1.getDim();
                }
            private:

                TExp1DAddOp& operator=(
                    const TExp1DAddOp<ValueType, Expression1Type, Expression2Type>&);

                /// Operand 1
                const Expression1Type m_operand1;
                /// Operand 2
                const Expression2Type m_operand2;
            };

            //=============================================================================
            // TExp1DSubOp
            //-----------------------------------------------------------------------------
            //! Sub operator for 1D expressions
            //! @param ValueType         Data Type
            //! @param Expression1Type   Data Type for 1D Expression
            //! @param Expression2Type   Data Type for 1D Expression
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_common.hpp $
            //=============================================================================
            template <class ValueType, class Expression1Type, class Expression2Type>
            class TExp1DSubOp
            {
            public:

                /// Value Type
                typedef ValueType                                   value_type;
                /// Shape Type after type promotion
                typedef typename vfc::linalg::TBinaryShapePromotion<
                    typename Expression1Type::shape_type,
                    typename Expression2Type::shape_type>::type     shape_type;

                /// Size Type
                typedef vfc::int32_t                                size_type;

                //---------------------------------------------------------------------
                //! Constructor
                //! Saves reference to the operands
                //! Asserts if the Dimension of the inputs is invalid.
                //! @param f_operand1_r       Expression Data of type Expression1Type
                //! @param f_operand2_r       Expression Data of type Expression2Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline
                TExp1DSubOp(const Expression1Type& f_operand1_r, const Expression2Type& f_operand2_r)
                    : m_operand1(f_operand1_r), m_operand2(f_operand2_r)
                {
                    VFC_REQUIRE(m_operand1.getDim() == m_operand2.getDim());
                }

                //---------------------------------------------------------------------
                //! Performs subtraction operation for a single element
                //! @param f_row       Row number
                //! @return  Returns the subtraction of m_operand1 & m_operand2 at index number
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_row) const
                {
                    return m_operand1[f_row] - m_operand2[f_row];
                }

                //---------------------------------------------------------------------
                //! Returns the dimension of the operands
                //! @param void
                //! @return  Returns the dimension of the operands
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getDim() const
                {
                    return m_operand1.getDim();
                }
            private:
                TExp1DSubOp& operator=(
                    const TExp1DSubOp<ValueType, Expression1Type, Expression2Type>&);

                /// Operand 1
                const Expression1Type m_operand1;
                /// Operand 2
                const Expression2Type m_operand2;
            };

            //=============================================================================
            // TExp1DScalarOp
            //-----------------------------------------------------------------------------
            //! Scalar operator for 1D expressions
            //! @param ValueType            Data Type
            //! @param ExpressionType       Data Type for 1D Expression
            //! @param OperationFunctorType Data Type for function pointer
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_common.hpp $
            //=============================================================================
            template <class ValueType, class ExpressionType, class OperationFunctorType>
            class TExp1DScalarOp
            {
            public:

                /// Value Type
                typedef ValueType                   value_type;

                /// Shape Type
                typedef typename ExpressionType::shape_type shape_type;

                /// Size type
                typedef vfc::int32_t                size_type;

                //---------------------------------------------------------------------
                //! Constructor
                //! Saves reference to the operands
                //! @param f_operand1_r       Expression Data of type Expression1Type
                //! @param f_operand2_r       Expression Data of type Expression2Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline
                TExp1DScalarOp(
                    const ExpressionType& f_operand1_r,
                    const ValueType& f_operand2_r)
                    : m_operand1(f_operand1_r), m_operand2(f_operand2_r) { }

                //---------------------------------------------------------------------
                //! Evaluates OperationFunctorType for a single element
                //! @param f_row       Row number
                //! @return  Returns the OperationFunctorType of m_operand1 at index number
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_row) const
                {
                    return OperationFunctorType()(m_operand1[f_row], m_operand2);
                }

                //---------------------------------------------------------------------
                //! Returns dimension of the operand 1
                //! @param void
                //! @return  Returns dimension of the operand 1
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getDim(void) const
                {
                    return m_operand1.getDim();
                }
            private:
                TExp1DScalarOp&
                    operator=(
                    const TExp1DScalarOp<ValueType,
                    ExpressionType,
                    OperationFunctorType>&);

                /// First operand
                const ExpressionType m_operand1;
                /// Second Operand
                ValueType            m_operand2;    //!< second operand
            };

            //=============================================================================
            // TExp1DNegateOp
            //-----------------------------------------------------------------------------
            //! Negation operator for 1D Expressions
            //! @param ValueType            Data Type
            //! @param ExpressionType       Data Type for 1D Expression
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_common.hpp $
            //=============================================================================
            template <class ValueType, class ExpressionType>
            class TExp1DNegateOp
            {
                public:

                    /// Value Type
                typedef ValueType                                           value_type;
                /// Shape Type
                typedef typename ExpressionType::shape_type                 shape_type;

                /// Size Type
                typedef vfc::int32_t                                        size_type;

                //---------------------------------------------------------------------
                //! Constructor
                //! Saves references to the operands
                //! Asserts if the inputs do not have valid dimensions
                //! @param f_operand_r       Expression Data of type ExpressionType
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline
                TExp1DNegateOp(const ExpressionType& f_operand_r) : m_operand(f_operand_r) { }

                //---------------------------------------------------------------------
                //! Returns negation of single element
                //! @param f_row       Row number
                //! @return  Returns negation of single element
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_row) const
                {
                    return -(m_operand[f_row]);
                }

                //---------------------------------------------------------------------
                //! Returns the dimension of the operand
                //! @param void
                //! @return  Returns the dimension of the operand
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getDim(void) const   { return m_operand.getDim(); }

            private:
                TExp1DNegateOp& operator=(const TExp1DNegateOp<ValueType, ExpressionType>&);

                /// Operand
                const ExpressionType m_operand;
            };

            template<class ValueType, class Expression1Type, class Expression2Type, class ShapeType>
            class TExp1DDotProductImpl;


            //=============================================================================
            // TExp1DAbsOp
            //-----------------------------------------------------------------------------
            //! Abs operator for 1D Expressions
            //! @param ValueType            Data Type
            //! @param ExpressionType       Data Type for 1D Expression
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_common.hpp $
            //=============================================================================
            template <class ValueType, class ExpressionType>
            class TExp1DAbsOp
            {
                public:

                    /// Value Type
                typedef ValueType                                           value_type;
                /// Shape Type
                typedef typename ExpressionType::shape_type                 shape_type;

                /// Size Type
                typedef vfc::int32_t                                        size_type;

                //---------------------------------------------------------------------
                //! Constructor
                //! Saves references to the operands
                //! Asserts if the inputs do not have valid dimensions
                //! @param f_operand_r       Expression Data of type ExpressionType
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline
                TExp1DAbsOp(const ExpressionType& f_operand_r) : m_operand(f_operand_r) { }

                //---------------------------------------------------------------------
                //! Returns abs of single element
                //! @param f_row       Row number
                //! @return  Returns abs of single element at index number
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
               inline value_type eval(size_type f_row) const
                {
                    return vfc::abs(m_operand[f_row]);
                }

                //---------------------------------------------------------------------
                //! Returns the dimension of the operand
                //! @param void
                //! @return  Returns the dimension of the operand
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getDim(void) const   { return m_operand.getDim(); }

            private:
                TExp1DAbsOp& operator=(const TExp1DAbsOp<ValueType, ExpressionType>&);

                /// Operand
                const ExpressionType m_operand;
            };

            //=============================================================================
            // TExp1DMinOp
            //-----------------------------------------------------------------------------
            //! Min operator for 1D expressions
            //! @param ValueType            Data Type
            //! @param Expression1Type      Data Type for 1D Expression
            //! @param Expression2Type      Data Type for 1D Expression
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_common.hpp $
            //=============================================================================
            template <class ValueType, class Expression1Type, class Expression2Type>
            class TExp1DMinOp
            {
            public:
                /// Value Type
                typedef ValueType                                   value_type;

                /// Shape Type after shape promotion
                typedef typename TBinaryShapePromotion<
                    typename Expression1Type::shape_type,
                    typename Expression2Type::shape_type>::type     shape_type;

                /// Size Type
                typedef vfc::int32_t                                size_type;

                //---------------------------------------------------------------------
                //! Constructor
                //! Saves references to the operands
                //! Asserts if the inputs do not have valid dimensions
                //! @param f_operand1_r       Expression Data of type Expression1Type
                //! @param f_operand2_r       Expression Data of type Expression2Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline
                TExp1DMinOp(const Expression1Type& f_operand1_r, const Expression2Type& f_operand2_r)
                    : m_operand1(f_operand1_r), m_operand2(f_operand2_r)
                {
                    VFC_REQUIRE(m_operand1.getDim() == m_operand2.getDim());
                }

                //---------------------------------------------------------------------
                //! Performs min operation for a single element
                //! @param f_row       Row number
                //! @return  Returns the min among m_operand1 and m_operand2 at index number
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_row) const
                {
                    return stlalias::min(m_operand1[f_row], m_operand2[f_row]);
                }

                //---------------------------------------------------------------------
                //! Returns the Dimension of the Operands
                //! @param void
                //! @return  Returns the Dimension of the Operands
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getDim() const
                {
                    return m_operand1.getDim();
                }
            private:

                TExp1DMinOp& operator=(
                    const TExp1DMinOp<ValueType, Expression1Type, Expression2Type>&);

                /// Operand 1
                const Expression1Type m_operand1;
                /// Operand 2
                const Expression2Type m_operand2;
            };

            //=============================================================================
            // TExp1DMaxOp
            //-----------------------------------------------------------------------------
            //! Max operator for 1D expressions
            //! @param ValueType            Data Type
            //! @param Expression1Type      Data Type for 1D Expression
            //! @param Expression2Type      Data Type for 1D Expression
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_common.hpp $
            //=============================================================================
            template <class ValueType, class Expression1Type, class Expression2Type>
            class TExp1DMaxOp
            {
            public:
                /// Value Type
                typedef ValueType                                   value_type;

                /// Shape Type after shape promotion
                typedef typename TBinaryShapePromotion<
                    typename Expression1Type::shape_type,
                    typename Expression2Type::shape_type>::type     shape_type;

                /// Size Type
                typedef vfc::int32_t                                size_type;

                //---------------------------------------------------------------------
                //! Constructor
                //! Saves references to the operands
                //! Asserts if the Dimension of the inputs is invalid.
                //! @param f_operand1_r       Expression Data of type Expression1Type
                //! @param f_operand2_r       Expression Data of type Expression2Type
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline
                TExp1DMaxOp(const Expression1Type& f_operand1_r, const Expression2Type& f_operand2_r)
                    : m_operand1(f_operand1_r), m_operand2(f_operand2_r)
                {
                    VFC_REQUIRE(m_operand1.getDim() == m_operand2.getDim());
                }

                //---------------------------------------------------------------------
                //! Performs max operation for a single element
                //! @param f_row       Row number
                //! @return  Returns the max among m_operand1 and m_operand2 at index number
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline value_type eval(size_type f_row) const
                {
                    return stlalias::max(m_operand1[f_row], m_operand2[f_row]);
                }

                //---------------------------------------------------------------------
                //! Returns the Dimension of the Operands
                //! @param void
                //! @return  Returns the Dimension of the Operands
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getDim() const
                {
                    return m_operand1.getDim();
                }
            private:

                TExp1DMaxOp& operator=(
                    const TExp1DMaxOp<ValueType, Expression1Type, Expression2Type>&);

                /// Operand 1
                const Expression1Type m_operand1;
                /// Operand 2
                const Expression2Type m_operand2;
            };

            template<class ValueType, class ExpressionType, class ShapeType>
            class TExp1DSumImpl;

            template<class ValueType, class ExpressionType, class ShapeType>
            class TExp1DNormLinfOpImpl;

            //=============================================================================
            // TExp1DSubViewFrom1DOp
            //-----------------------------------------------------------------------------
            //! SubView operator for 1D Expressions
            //! @param ValueType            Data Type
            //! @param ExpressionType       Data Type for 1D Expression
            //! @param ShapeType            Data Type
            //! @ingroup  vfc_group_linalg
            //! @author   jat2hi
            //! $Source: vfc_linalg_exp1d_common.hpp $
            //=============================================================================
            template <class ValueType, class ExpressionType, class ShapeType>
            class TExp1DSubViewFrom1DOp
            {
            public:

                /// Value Type
                typedef ValueType                                           value_type;
                /// Shape Type
                typedef typename ExpressionType::shape_type                 shape_type;

                /// Size Type
                typedef vfc::int32_t                                        size_type;

                //---------------------------------------------------------------------
                //! Constructor
                //! Saves references to the operands
                //! Asserts if the inputs do not have valid dimensions
                //! @param f_operand_r      Expression Data of type ExpressionType
                //! @param f_pos0           Index Position
                //! @param f_length         Vector length
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline
                TExp1DSubViewFrom1DOp(const ExpressionType& f_operand_r,
                    size_type f_pos0, size_type f_length) :
                    m_operand(f_operand_r), m_nbStartPos(f_pos0), m_length(f_length)
                {
                    VFC_REQUIRE(0 <= f_pos0);
                    //check if the max size required is acceptanle
                    VFC_REQUIRE(f_length <= (m_operand.getDim() - f_pos0));
                }

                //---------------------------------------------------------------------
                //! Returns a single element at the given position in the subview.
                //! @param f_pos       Index Position
                //! @return  Returns a single element at the given position in the subview
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
               inline value_type eval(size_type f_pos) const
                {
                    VFC_REQUIRE(f_pos < m_length && 0 <= f_pos);
                    return m_operand[m_nbStartPos+f_pos];
                }

                //---------------------------------------------------------------------
                //! Returns the dimension of the expression
                //! @param void
                //! @return  Returns the dimension of the expression
                //! @author  jat2hi
                //! $Source: vfc_linalg_exp1d_common.hpp $
                //---------------------------------------------------------------------
                inline size_type getDim(void) const   { return m_length; }

            private:
                TExp1DSubViewFrom1DOp& operator=(
                    const TExp1DSubViewFrom1DOp<ValueType, ExpressionType, ShapeType>&);

                /// Operand
                const ExpressionType m_operand;     //!< reference to expression
                size_type m_nbStartPos;             //!< start position in the expression
                size_type m_length;                 //!< size of the 1d expression
            };

        } // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed

#endif //VFC_LINALG_EXP1D_COMMON_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_exp1d_common.hpp  $
//  Revision 1.6 2011/01/21 12:53:34MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - sometimes .hpp or namespace qualifiers are missing (mantis3599)
//  Revision 1.5 2009/05/28 09:19:05MESZ Muehlmann Karsten (CC/ESV2) (MUK2LR) 
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.4 2009/03/25 14:48:12CET Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.3 2009/01/07 10:19:16IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  QAC++ Warning (2141) Removal.
//  (Mantis : 0002479)
//  Revision 1.2 2008/07/31 14:08:13IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:19IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
