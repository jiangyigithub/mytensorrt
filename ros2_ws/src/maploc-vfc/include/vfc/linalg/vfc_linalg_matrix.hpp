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
///     $Source: vfc_linalg_matrix.hpp $
///     $Revision: 1.8 $
///     $Author: Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) $
///     $Date: 2009/05/28 09:19:05MESZ $
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

#ifndef VFC_LINALG_MATRIX_HPP_INCLUDED
#define VFC_LINALG_MATRIX_HPP_INCLUDED


// libc
#include <iterator>
#include <vector>

// core
#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_assert.hpp"

// memory
#include "vfc/memory/vfc_freestore_allocator.hpp"
// linalg
#include "vfc/linalg/vfc_linalg_shapepromotion.hpp"
#include "vfc/linalg/vfc_linalg_matrixbase_dyn.hpp"
#include "vfc/linalg/vfc_linalg_matrixmn.hpp" // for ctor/assignment from static matrix

namespace vfc
{   // namespace vfc opened

    namespace linalg
    {   // namespace linalg opened

        //=====================================================================
        // TMatrix : public TMatrixBase
        //---------------------------------------------------------------------
        //! Dynamic matrix class
        //! Makes use of the "curiously recurring template pattern" to avoid
        //! virtual function calls when using operators via the TMatrixBase type.
        //! @param ValueType     Data Type
        //! @param StorageType   std::vector<ValueType>
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg_dynamic
        //! $Source: vfc_linalg_matrix.hpp $
        //=====================================================================
        template <class ValueType, class StorageType = stlalias::vector< ValueType, vfc::TFreeStoreAllocator<ValueType> > >
        class TMatrix : public TMatrixBase<ValueType, TMatrix<ValueType, StorageType>,
            CDynamicRectangle>
        {
        public:

            /// Data type of the linalg::TMatrix.
            typedef ValueType                                   value_type;
            /// Reference type of the linalg::TMatrix data elements.
            typedef ValueType&                                  reference;
            /// const reference type of the linalg::TMatrix data elements.
            typedef const ValueType&                            const_reference;
            /// STL pointer interface.
            typedef ValueType*                                  pointer;
            /// STL const_pointer interface.
            typedef const ValueType*                            const_pointer;
            /// STL iterator interface.
            typedef typename StorageType::iterator              iterator;
            /// STL const_iterator interface.
            typedef typename StorageType::const_iterator        const_iterator;
            /// STL reverse iterator interface.
            typedef stlalias::reverse_iterator<iterator>             reverse_iterator;
            /// STL const reverse iterator interface.
            typedef stlalias::reverse_iterator<const_iterator>       const_reverse_iterator;
            /// Shape type
            typedef CDynamicRectangle                           shape_type;
            /// Size type
            typedef vfc::int32_t                                size_type;
            /// Storage type
            typedef StorageType                                    storage_type;

            enum { NB_ROWS = 0 };
            enum { NB_COLUMNS = 0 };

            //---------------------------------------------------------------------
            //! Default constructor. Matrix size will be 0, 0. Does not initialize the values to 0.
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            inline TMatrix(void) : m_nbRows(0), m_nbColumns(0)
            {
            }

            //---------------------------------------------------------------------
            //! Constructor resizes the matrix to the specifed size and initialized with the value provided.
            //! @param f_nbRows     Number of rows
            //! @param f_nbCols     Number of columns
            //! @param f_val        ValueType()
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            inline TMatrix(size_type f_nbRows, size_type f_nbCols, const ValueType& f_val = ValueType())
            {
                resize(f_nbRows, f_nbCols, f_val);
            }

            //---------------------------------------------------------------------
            //! Copy constructor
            //! @param f_param_r     Object of TMatrix
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            TMatrix(const TMatrix<ValueType, StorageType>& f_param_r);

            //---------------------------------------------------------------------
            //! Constructor copies from given static matrix
            //! @param f_param_r     Object of TMatrixMN
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, vfc::int32_t ColValue>
            TMatrix(const TMatrixMN<ValueType, RowValue, ColValue>& f_param_r);

            //---------------------------------------------------------------------
            //! Constructor copies from given 2D expression
            //! @param f_param_r     Object of TExp2D
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType, class ShapeType>
            explicit
            TMatrix(const intern::TExp2D<ValueType, OperatorType, ShapeType>& f_param_r);

            //---------------------------------------------------------------------
            //! Resizes the matrix
            //! @param f_nbRows     Number of rows
            //! @param f_nbCols     Number of columns
            //! @note               If the container's size is less than the requested size,
            //!                     elements are added to the container until it reaches
            //!                     the requested size. If the container's size is larger than
            //!                     the requested size, the elements closest to the end of the
            //!                     container are deleted until the container reaches the size.
            //!                     If the present size of the container is the same
            //!                     as the requested size, no action is taken.
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            void resize (size_type f_nbRows, size_type f_nbCols);

            //---------------------------------------------------------------------
            //! Resizes the matrix and assigns them to the specified value.
            //! @param f_nbRows     Number of rows
            //! @param f_nbCols     Number of columns
            //! @param f_val        Specified scalar value
            //! @note               The f_val is value of new elements added to the vector,
            //!                     if the new size is larger that the original size.
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            void resize (size_type f_nbRows, size_type f_nbCols,
                const value_type& f_val);

            //---------------------------------------------------------------------
            //! Returns const ref to element(row,col)). asserts if invalid dimension is provided
            //! @param f_nbRows     Number of rows
            //! @param f_nbCols     Number of columns
            //! @return Returns const ref to element(row,col)). asserts if invalid dimension is provided
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            inline const_reference operator() (size_type f_row, size_type f_col) const
            {
                VFC_REQUIRE( f_row < m_nbRows && \
                    f_col < m_nbColumns );
                return m_data[f_row*m_nbColumns+f_col];
            }

            //---------------------------------------------------------------------
            //! Returns ref to element(row,col)). asserts if invalid dimension is provided
            //! @param f_nbRows     Number of rows
            //! @param f_nbCols     Number of columns
            //! @return Returns ref to element(row,col)). asserts if invalid dimension is provided
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            inline reference operator() (size_type f_row, size_type f_col)
            {
                VFC_REQUIRE( f_row < m_nbRows && \
                    f_col < m_nbColumns );
                return m_data[f_row*m_nbColumns+f_col];
            }

            //---------------------------------------------------------------------
            //! Returns number of rows
            //! @param void
            //! @return Returns number of rows
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            inline size_type getNbRows(void) const
            {
                return m_nbRows;
            }

            //---------------------------------------------------------------------
            //! Returns number of columns
            //! @param void
            //! @return Returns number of columns
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            inline size_type getNbColumns(void) const
            {
                return m_nbColumns;
            }

            //---------------------------------------------------------------------
            //! Returns value at Matrix(0,0)
            //! @param void
            //! @return Returns value at Matrix(0,0)
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            const_iterator  begin   (void) const    {   return m_data.begin(); }

            //---------------------------------------------------------------------
            //! Returns value at Matrix(0,0)
            //! @param void
            //! @return Returns value at Matrix(0,0)
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            iterator        begin   (void)          {   return m_data.begin(); }

            //---------------------------------------------------------------------
            //! Returns value at last Matrix index
            //! @param void
            //! @return Returns value at last Matrix index
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            const_iterator  end     (void) const    {   return m_data.end(); }

            //---------------------------------------------------------------------
            //! Returns value at last Matrix index
            //! @param void
            //! @return Returns value at last Matrix index
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            iterator        end     (void)          {   return m_data.end(); }

            //---------------------------------------------------------------------
            //! Assignment operator, assigns a dynamic matrix
            //! @param f_rhs_r      Object of TMatrix
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            const TMatrix<ValueType, StorageType>&
            operator=(const TMatrix<ValueType, StorageType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Assignment operator, assigns a static matrix
            //! @param f_rhs_r      Object of TMatrixMN
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, vfc::int32_t ColValue>
            const TMatrix<ValueType, StorageType>&
            operator=(const TMatrixMN<ValueType, RowValue, ColValue>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Assignment operator, assigns a 2D expression
            //! @param f_rhs_r      Object of TExp2D
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            template <class OperatorType, class ShapeType>
            const TMatrix<ValueType, StorageType>&
            operator=(const intern::TExp2D<ValueType, OperatorType, ShapeType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Copies the contents of a given dynamic matrix to itself after a temporary copy
            //! @param f_rhs_r      Object of TMatrix
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            void assign(const TMatrix<ValueType, StorageType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Copies the contents of a given static matrix to itself after a temporary copy
            //! @param f_rhs_r      Object of TMatrixMN
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue, vfc::int32_t ColValue>
            void assign(const TMatrixMN<ValueType, RowValue,
                ColValue>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Copies the contents of a given 2d expression to itself after a temporary copy
            //! @param f_rhs_r      Object of TExp2D
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            template <class OperatorType, class ShapeType>
            void assign(const intern::TExp2D<ValueType, OperatorType,
                ShapeType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Copies its contents to a given dynamic matrix
            //! @param f_rhs_r      Object of TMatrix
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            void assign_to(TMatrix<ValueType, StorageType>& f_rhs_r) const;

            //---------------------------------------------------------------------
            //! Performs the binary operation specified with the given dynamic matrix to itself
            //! @param f_rhs_r      Object of TMatrix
            //! @param f_functor_r  Function pointer of type OperationFunctor
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            template <class OperationFunctor>
            void opassign(const TMatrix<ValueType, StorageType>& f_rhs_r,
                const OperationFunctor& f_functor_r);

            //---------------------------------------------------------------------
            //! Performs the binary operation specified with the given static matrix to itself
            //! @param f_rhs_r      Object of TMatrixMN
            //! @param f_functor_r  Function pointer of type OperationFunctor
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            template <vfc::int32_t RowValue, vfc::int32_t ColValue, class OperationFunctor>
            void opassign(const TMatrixMN<ValueType, RowValue,
                ColValue>& f_rhs_r, const OperationFunctor& f_functor_r);

            //---------------------------------------------------------------------
            //! Performs the binary operation specified with the given 2dexpression to itself
            //! @param f_rhs_r      Object of TExp2D
            //! @param f_functor_r  Function pointer of type OperationFunctor
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            template <class OperatorType, class ShapeType, class OperationFunctor>
            void opassign(const intern::TExp2D<ValueType, OperatorType,
                ShapeType>& f_rhs_r, const OperationFunctor& f_functor_r);

            //---------------------------------------------------------------------
            //! Performs the unary operation specified with the given value to itself
            //! @param f_rhs_r          Scalar value
            //! @param f_functor_r      Function pointer of type OperationFunctor
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            template <class OperationFunctor>
            void unaryOpassign(const ValueType& f_rhs_r,
                const OperationFunctor& f_functor_r);

            //---------------------------------------------------------------------
            //! Makes the matrix an identity matrix. does not check for square matrices
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            void identity();

            //---------------------------------------------------------------------
            //! Fills the matrix with 0's
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            void zero();

            //---------------------------------------------------------------------
            //! Set the Matrix with specific user said value
            //! @param f_value_r      Scalar value
            //! @author  vmr1kor
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            void set(const ValueType& f_value_r);

            //---------------------------------------------------------------------
            //! Adds a given static / dynamic matrix to itself. asserts if shape is invalid
            //! @param f_rhs_r      object of TMatrixBase
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            template<class MatrixType, class ShapeType>
            TMatrix<ValueType, StorageType>&
            operator+= (const TMatrixBase<ValueType, MatrixType, ShapeType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Adds a given 2D expression to itself. asserts if shape is invalid
            //! @param f_rhs_r      object of TExp2D
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType, class ShapeType>
            TMatrix<ValueType, StorageType>&
            operator+= (const intern::TExp2D<ValueType, OperatorType,
                ShapeType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Subtracts a given static / dynamic matrix from itself. asserts if shape is invalid
            //! @param f_rhs_r      object of TMatrixBase
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            template<class MatrixType, class ShapeType>
            TMatrix<ValueType, StorageType>&
            operator-= (const TMatrixBase<ValueType, MatrixType, ShapeType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Subtracts a given 2D expression from itself. asserts if shape is invalid
            //! @param f_rhs_r      object of TExp2D
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType, class ShapeType>
            TMatrix<ValueType, StorageType>&
            operator-= (const intern::TExp2D<ValueType, OperatorType,
                ShapeType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Multiplies a given scalar to itself
            //! @param f_value_r      Scalar value
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            TMatrix<ValueType, StorageType>&
            operator*= (const ValueType& f_value_r);

            //---------------------------------------------------------------------
            //! Divides a given scalar to itself. asserts if value is 0.
            //! @param f_value_r      Scalar value
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix.hpp $
            //---------------------------------------------------------------------
            TMatrix<ValueType, StorageType>&
            operator/= (const ValueType& f_value_r);

        protected:
            size_type m_nbRows;              //!< Number of rows
            size_type m_nbColumns;           //!< Number of columns
            StorageType m_data;  //!< Vector containing the matrix data
        };

    }   // namespace linalg closed

}   // namespace vfc closed

#include "vfc/linalg/vfc_linalg_matrix.inl"

#endif //VFC_LINALG_MATRIX_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_matrix.hpp  $
//  Revision 1.8 2009/05/28 09:19:05MESZ Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) 
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.7 2008/09/11 13:55:05CEST Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  -Documentation related to the functionality of vfc::linalg::TMatrix, vfc::linalg::TVector : resize()
//  (Mantis : 1727)
//  Revision 1.6 2008/08/29 18:32:39IST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  Removal of template parameter from set functionality (Mantis :2176)
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.5 2008/08/11 16:33:05IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - changed std::vector default allocator (mantis2270)
//  Revision 1.4 2008/07/31 13:13:34CEST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  set function doxy comment added(Mantis :- 1744 )
//  Revision 1.3 2008/07/31 14:08:24IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.2 2008/07/21 11:06:22IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  set() function added ( Mantis : 2176 )
//  Revision 1.1 2007/05/09 13:51:20IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
