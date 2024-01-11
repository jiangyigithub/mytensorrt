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
///     $Source: vfc_linalg_matrixmn.hpp $
///     $Revision: 1.4 $
///     $Author: gaj2kor $
///     $Date: 2009/03/25 14:48:22MEZ $
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

#ifndef VFC_LINALG_MATRIXMN_HPP_INCLUDED
#define VFC_LINALG_MATRIXMN_HPP_INCLUDED

#include <iterator>
#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_assert.hpp"
#include "vfc/core/vfc_static_assert.hpp"
#include "vfc/linalg/vfc_linalg_matrixbase.hpp"

// forward declarations
namespace vfc
{   // namespace vfc opened

    namespace linalg
    {   // namespace linalg opened

        template <vfc::int32_t RowValue, vfc::int32_t ColValue>
        struct TStaticRectangle;

        //-------------------------------------------------------------------------
        // conditional doxygen documentation
        //! @cond VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

        namespace intern
        {   // namespace intern opened

            template <class ValueType, class OperatorType, class ShapeType>
            class TExp2D;

        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed

namespace vfc
{   // namespace vfc opened

    namespace linalg
    {   // namespace linalg opened

        //=====================================================================
        // TMatrixMN
        //---------------------------------------------------------------------
        //! Static matrix class
        //! Makes use of the "curiously recurring template pattern" to avoid
        //! virtual function calls when using operators via the TMatrixBase type.
        //! @param ValueType     Data Type
        //! @param RowValue      Number of rows
        //! @param ColValue      Number of columns
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg_fixed
        //! $Source: vfc_linalg_matrixmn.hpp $
        //=====================================================================
        template <class ValueType, vfc::int32_t RowValue, vfc::int32_t ColValue>
        class TMatrixMN : public TMatrixBase<ValueType,
            TMatrixMN<ValueType, RowValue, ColValue>,
            TStaticRectangle<RowValue, ColValue> >
        {

            VFC_STATIC_ASSERT( (0 <= RowValue) && (0 <= ColValue) );

        public:

            typedef TMatrixBase<ValueType,
                    TMatrixMN<ValueType, RowValue, ColValue>,
                    TStaticRectangle<RowValue, ColValue> >      base_type;
            /// Data type of the linalg::TMatrixMN.
            typedef typename base_type::value_type              value_type;
            /// Reference type of the linalg::TMatrixMN data elements.
            typedef typename base_type::reference               reference;
            /// const reference type of the linalg::TMatrixMN data elements.
            typedef typename base_type::const_reference         const_reference;
            /// STL pointer interface.
            typedef typename base_type::pointer                 pointer;
            /// STL const_pointer interface.
            typedef typename base_type::const_pointer           const_pointer;
            /// STL iterator interface.
            typedef typename base_type::iterator                iterator;
            /// STL const_iterator interface.
            typedef typename base_type::const_iterator          const_iterator;
            /// STL reverse iterator interface.
            typedef typename base_type::reverse_iterator        reverse_iterator;
            /// STL const reverse iterator interface.
            typedef typename base_type::const_reverse_iterator  const_reverse_iterator;
            /// Shape type
            typedef typename base_type::shape_type              shape_type;
            /// Size type
            typedef typename base_type::size_type               size_type;

            enum { NB_ROWS = shape_type::NB_ROWS };
            enum { NB_COLUMNS = shape_type::NB_COLUMNS };

            //---------------------------------------------------------------------
            //! Default constructor. Does not do any operation
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            inline TMatrixMN(void) { }

            //---------------------------------------------------------------------
            //! Constructor which takes two ints.
            //! For interface combatibility with the dynamic matrix.
            //! Does nothing, asserts that the dimensions match.
            //! @param f_rows       Number of rows
            //! @param f_cols       Number of columns
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            inline TMatrixMN(vfc::int32_t f_rows, vfc::int32_t f_cols)
            {
                VFC_REQUIRE( (NB_ROWS == f_rows) && (NB_COLUMNS == f_cols) );
            }

            //---------------------------------------------------------------------
            //! Constructor which takes two ints and a default value.
            //! For interface combatibility with the dynamic matrix.
            //! Constructor initializes the matrix with the value provided.
            //! @param f_rows       Number of rows
            //! @param f_cols       Number of columns
            //! @param f_value_r    Data for initialization of matrix
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            inline TMatrixMN(vfc::int32_t f_rows, vfc::int32_t f_cols, const ValueType& f_value_r);

            //---------------------------------------------------------------------
            //! Copy constructor
            //! @param f_param_r    Object of TMatrixMN
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            TMatrixMN(const TMatrixMN<ValueType, RowValue, ColValue>& f_param_r);

            //---------------------------------------------------------------------
            //! Constructor initializes the matrix with the value provided.
            //! @param f_value_r    Data for initialization of matrix
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            explicit
            TMatrixMN(const ValueType& f_value_r);

            //---------------------------------------------------------------------
            //! Constructor copies from given 2D expression.
            //! @param f_param_r    Object of TExp2D
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType>
            explicit
            TMatrixMN(const intern::TExp2D<ValueType, OperatorType,
                TStaticRectangle<RowValue, ColValue> >& f_param_r);

            //---------------------------------------------------------------------
            //! Returns const ref to element(row,col)). asserts if invalid dimension is provided
            //! @param f_row    Number of rows
            //! @param f_col    Number of columns
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            inline const_reference operator() (size_type f_row, size_type f_col) const
            {
                VFC_REQUIRE( f_row < NB_ROWS && \
                    f_col < NB_COLUMNS );
                return m_data[f_row*NB_COLUMNS+f_col];
            }

            //---------------------------------------------------------------------
            //! Returns ref to element(row,col)). asserts if invalid dimension is provided
            //! @param f_row    Number of rows
            //! @param f_col    Number of columns
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            inline reference operator() (size_type f_row, size_type f_col)
            {
                VFC_REQUIRE( f_row < NB_ROWS && \
                    f_col < NB_COLUMNS );
                return m_data[f_row*NB_COLUMNS+f_col];
            }

            //---------------------------------------------------------------------
            //! Returns number of rows
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            inline size_type getNbRows(void) const
            {
                return NB_ROWS;
            }

            //---------------------------------------------------------------------
            //! Returns number of columns
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            inline size_type getNbColumns(void) const
            {
                return NB_COLUMNS;
            }

            //---------------------------------------------------------------------
            //! Returns value at matrix(0,0)
            //! @param void
            //! @return Returns value at matrix(0,0)
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            const_iterator  begin   (void) const    {   return &m_data[0]; }
            //---------------------------------------------------------------------
            //! Returns value at matrix(0,0)
            //! @param void
            //! @return Returns value at matrix(0,0)
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            iterator        begin   (void)          {   return &m_data[0]; }

            //---------------------------------------------------------------------
            //! Returns value at last index of matrix
            //! @param void
            //! @return Returns value at last index of matrix
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            const_iterator  end     (void) const    {   return begin()+DATASIZE; }
            //---------------------------------------------------------------------
            //! Returns value at last index of matrix
            //! @param void
            //! @return Returns value at last index of matrix
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            iterator        end     (void)          {   return begin()+DATASIZE; }

            //---------------------------------------------------------------------
            //! Assignment operator, assigns a static matrix
            //! @param f_rhs_r      Object of TMatrixMN
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            const TMatrixMN<ValueType, RowValue, ColValue>&
            operator=(const TMatrixMN<ValueType, RowValue, ColValue>& f_rhs_r);

            //---------------------------------------------------------------------
            //! assignment operator, assigns a 2D expression matrix.
            //! This assignment operator can not be used to write expressions, which
            //! resemble this example:
            //! \code vfc::linalg::TMatrixMN<vfc::float64_t,4,4> A = B * C; \endcode
            //! Such code will give a compiler error. @par
            //! The reason is that this is not a constructor call followed by an
            //! assignment operator call. As this is not an assignment, but an initialisation,
            //! the constructor taking an expression type as argument is called by the compiler:
            //! vfc::linalg::TMatrixMN<ValueType, RowValue, ColValue>::TMatrixMN(const TExp2D<ValueType, OperatorType,
            //! TStaticRectangle<RowValue, ColValue> >&)
            //! The compiler generates an error here, as the implicitly needed constructor
            //! is made explicit (by design).
            //! @par
            //! To use an initialisation with an expression, do not use the assignment operator,
            //! but explicitly call the constructor instead:
            //! \code vfc::linalg::TMatrixMN<vfc::float64_t,4,4> A(B * C); \endcode.
            //! @param f_rhs_r      Object of TExp2D
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            template <class OperatorType>
            const TMatrixMN<ValueType, RowValue, ColValue>&
            operator=(const intern::TExp2D<ValueType, OperatorType,
                TStaticRectangle<RowValue, ColValue> >& f_rhs_r);

            //---------------------------------------------------------------------
            //! Copies the contents of a given static matrix to itself after a temporary copy
            //! @param f_rhs_r      Object of TMatrixMN
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            void assign(const TMatrixMN<ValueType,
                RowValue, ColValue>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Copies the contents of a 2D expression to iteself after a temporary copy
            //! @param f_rhs_r      Object of TExp2D
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            template <class OperatorType>
            void assign(const intern::TExp2D<ValueType, OperatorType,
                TStaticRectangle<RowValue, ColValue> >& f_rhs_r);

            //---------------------------------------------------------------------
            //! Copies the contents of a 2D expression to iteself after a temporary copy
            //! @param f_rhs_r      Object of TMatrixMN
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            void assign_to(TMatrixMN<ValueType,
                RowValue, ColValue>& f_rhs_r) const;

            //---------------------------------------------------------------------
            //! Performs the binary operation specified with the given matrix to itself
            //! @param f_rhs_r      Object of TMatrixMN
            //! @param f_functor_r  Function pointer of type OperationFunctor
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            template <class OperationFunctor>
            void opassign(const TMatrixMN<ValueType,
                RowValue, ColValue>& f_rhs_r,
                const OperationFunctor& f_functor_r);

            //---------------------------------------------------------------------
            //! Performs the operation specified with the given 2D expression to itself
            //! @param f_rhs_r      Object of TExp2D
            //! @param f_functor_r  Function pointer of type OperationFunctor
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            template <class OperatorType, class OperationFunctor>
            void opassign(const intern::TExp2D<ValueType, OperatorType,
                TStaticRectangle<RowValue, ColValue> >& f_rhs_r,
                const OperationFunctor& f_functor_r);

            //---------------------------------------------------------------------
            //! Performs the unary operation specified with the given value to itself
            //! @param f_rhs_r      Value for intialization
            //! @param f_functor_r  Function pointer of type OperationFunctor
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            template <class OperationFunctor>
            void unaryOpassign(const ValueType& f_rhs_r,
                const OperationFunctor& f_functor_r);

            //---------------------------------------------------------------------
            //! Makes the matrix an identity matrix. does not check for square matrices
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            void identity();

            //---------------------------------------------------------------------
            //! Fills the matrix with 0's
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            void zero();

            //---------------------------------------------------------------------
            //! Set every element
            //! @param f_value      Value for intialization
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            template<class T>
            void set(T f_value);

            //---------------------------------------------------------------------
            //! Adds a given matrix to itself
            //! @param f_rhs_r      Object of TMatrixMN for addition
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            TMatrixMN<ValueType, RowValue, ColValue>&
            operator+= (const TMatrixMN<ValueType, RowValue, ColValue>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Adds a given 2D expression to itself
            //! @param f_rhs_r      Object of TExp2D for addition
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType>
            TMatrixMN<ValueType, RowValue, ColValue>&
            operator+= (const intern::TExp2D<ValueType, OperatorType,
                TStaticRectangle<RowValue, ColValue> >& f_rhs_r);

            //---------------------------------------------------------------------
            //! Subtracts a given matrix from itself
            //! @param f_rhs_r      Object of TMatrixMN for Subtraction
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            TMatrixMN<ValueType, RowValue, ColValue>&
            operator-= (const TMatrixMN<ValueType, RowValue, ColValue>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Subtracts a given 2D expression from itself
            //! @param f_rhs_r      Object of TExp2D for Subtraction
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType>
            TMatrixMN<ValueType, RowValue, ColValue>&
            operator-= (const intern::TExp2D<ValueType, OperatorType,
                TStaticRectangle<RowValue, ColValue> >& f_rhs_r);

            //---------------------------------------------------------------------
            //! Multiplies a given scalar to itself
            //! @param f_value_r      Scalar Value for multiplication
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            TMatrixMN<ValueType, RowValue, ColValue>&
            operator*= (const ValueType& f_value_r);

            //---------------------------------------------------------------------
            //! Divides a given scalar to itself. asserts if f_value_r is 0
            //! @param f_value_r      Scalar Value for divission
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixmn.hpp $
            //---------------------------------------------------------------------
            TMatrixMN<ValueType, RowValue, ColValue>&
            operator/= (const ValueType& f_value_r);

        private:
            enum { DATASIZE = RowValue * ColValue };
            value_type m_data[DATASIZE]; //!< array containing the matrix data
        };

    }   // namespace linalg closed


}   // namespace vfc closed

#include "vfc/linalg/vfc_linalg_matrixmn.inl"

#endif //VFC_LINALG_MATRIXMN_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_matrixmn.hpp  $
//  Revision 1.4 2009/03/25 14:48:22MEZ gaj2kor 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.3 2008/08/29 18:34:53IST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.2 2008/07/31 14:08:33IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:22IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
