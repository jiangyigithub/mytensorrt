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
///     $Source: vfc_linalg_matrixbase.hpp $
///     $Revision: 1.3 $
///     $Author: Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) $
///     $Date: 2009/05/28 09:19:04MESZ $
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

#ifndef VFC_LINALG_MATRIXBASE_HPP_INCLUDED
#define VFC_LINALG_MATRIXBASE_HPP_INCLUDED

#include <iterator>
#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_assert.hpp"

namespace vfc
{   // namespace vfc opened

    namespace linalg
    {   // namespace linalg opened

        //=====================================================================
        // TMatrixBase
        //---------------------------------------------------------------------
        //!  Matrix base class. Class used only for inheritance.
        //! @param ValueType     Data Type
        //! @param DerivedType   Data Type
        //! @param ShapeType     Data Type
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_matrixbase.hpp $
        //=====================================================================
        template<class ValueType, class DerivedType, class ShapeType>
        class TMatrixBase
        {
        public:

            /// Data type of the linalg::TMatrixBase.
            typedef ValueType                               value_type;
            /// Reference type of the linalg::TMatrixBase data elements.
            typedef ValueType&                              reference;
            /// const reference type of the linalg::TMatrixBase data elements.
            typedef const ValueType&                        const_reference;
            /// STL pointer interface.
            typedef ValueType*                              pointer;
            /// STL const_pointer interface.
            typedef const ValueType*                        const_pointer;
            /// STL iterator interface.
            typedef ValueType*                              iterator;
            /// STL const_iterator interface.
            typedef const ValueType*                        const_iterator;
            /// STL reverse iterator interface.
            typedef stlalias::reverse_iterator<iterator>    reverse_iterator;
            /// STL const reverse iterator interface.
            typedef stlalias::reverse_iterator<const_iterator> const_reverse_iterator;
            /// Derived class type.
            typedef DerivedType                             derived_type;
            /// Shape type
            typedef ShapeType                               shape_type;
            /// Size type
            typedef vfc::int32_t                             size_type;

            enum { NB_ROWS = shape_type::NB_ROWS };
            enum { NB_COLUMNS = shape_type::NB_COLUMNS };

            //---------------------------------------------------------------------
            //!   Returns const ref to element(row,col). asserts if arguments are invalid.
            //!         Coding rule violation for the sake of speed.
            //! @param f_row        Number of rows
            //! @param f_col        Number of coloumns
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixbase.hpp $
            //---------------------------------------------------------------------
            const_reference operator() (size_type f_row, size_type f_col) const;

            //---------------------------------------------------------------------
            //!   Returns ref to element(row,col). asserts if arguments are invalid.
            //!         Coding rule violation for the sake of speed
            //! @param f_row        Number of rows
            //! @param f_col        Number of coloumns
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixbase.hpp $
            //---------------------------------------------------------------------
            reference operator() (size_type f_row, size_type f_col);

            //---------------------------------------------------------------------
            //!   Returns number of rows
            //! @param void
            //! @return  Returns number of rows
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixbase.hpp $
            //---------------------------------------------------------------------
            inline size_type getNbRows(void) const
            {
                return shape_type::NB_ROWS;
            }

            //---------------------------------------------------------------------
            //!   Returns number of columns
            //! @param void
            //! @return  Returns number of columns
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixbase.hpp $
            //---------------------------------------------------------------------
            inline size_type getNbColumns(void) const
            {
                return shape_type::NB_COLUMNS;
            }


        protected:
            inline
            explicit
            TMatrixBase() { }

            inline
            explicit
            TMatrixBase(const TMatrixBase<ValueType, DerivedType, ShapeType>&) { }

            const TMatrixBase<ValueType, DerivedType, ShapeType>&
            operator=(const TMatrixBase<ValueType, DerivedType, ShapeType>& f_rhs_r);

            inline
            ~TMatrixBase(){}
        };

        //=====================================================================
        // TMatrixBase
        //---------------------------------------------------------------------
        //!  Provides a constant reference to a matrix class. The class is used by operators
        //! @param ValueType     Data Type
        //! @param DerivedType   Data Type
        //! @param ShapeType     Data Type
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_matrixbase.hpp $
        //=====================================================================
        template <class ValueType, class DerivedType, class ShapeType>
        class TMatrixConstRef
        {
        public:

            /// Data type of the linalg::TMatrixBase.
            typedef ValueType                               value_type;
            /// Reference type of the linalg::TMatrixBase data elements.
            typedef ValueType&                              reference;
            /// const reference type of the linalg::TMatrixBase data elements.
            typedef const ValueType&                        const_reference;
            /// STL iterator interface.
            typedef ValueType*                              iterator;
            /// STL const_iterator interface.
            typedef const ValueType*                        const_iterator;
            /// STL reverse iterator interface.
            typedef stlalias::reverse_iterator<iterator>         reverse_iterator;
            /// STL const reverse iterator interface.
            typedef stlalias::reverse_iterator<const_iterator>   const_reverse_iterator;
            /// Derived class type.
            typedef DerivedType                             derived_type;
            /// Shape type
            typedef ShapeType                               shape_type;
            /// Size type
            typedef vfc::int32_t                             size_type;

            enum { NB_ROWS = shape_type::NB_ROWS};
            enum { NB_COLUMNS = shape_type::NB_COLUMNS};

            //---------------------------------------------------------------------
            //!   Saves refernces of the matrix class
            //! @param f_param_r    Object of TMatrixBase
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixbase.hpp $
            //---------------------------------------------------------------------
            inline TMatrixConstRef(const TMatrixBase<ValueType, DerivedType, ShapeType>& f_param_r) :
                m_data(&f_param_r(0,0))
            { }

            //---------------------------------------------------------------------
            //!   Returns const ref to element(row,col). asserts if arguments are invalid.
            //! @param f_row    Number of rows
            //! @param f_col    Number of columns
            //! @return  Returns const ref to element(row,col). asserts if arguments are invalid.
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixbase.hpp $
            //---------------------------------------------------------------------
            inline const value_type& operator()(size_type f_row, size_type f_col) const
            {
                VFC_REQUIRE( f_row < NB_ROWS && \
                    f_col < NB_COLUMNS );
                return m_data[f_row * NB_COLUMNS + f_col];
            }

            //---------------------------------------------------------------------
            //!   Returns number of rows
            //! @param void
            //! @return  Returns number of rows
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixbase.hpp $
            //---------------------------------------------------------------------
            inline size_type getNbRows(void) const
            {
                return shape_type::NB_ROWS;
            }

            //---------------------------------------------------------------------
            //!   Returns number of columns
            //! @param void
            //! @return  Returns number of columns
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixbase.hpp $
            //---------------------------------------------------------------------
            inline size_type getNbColumns(void) const
            {
                return shape_type::NB_COLUMNS;
            }

            //---------------------------------------------------------------------
            //!  Saves refernces of the matrix class.
            //! @param f_rhs_r      Object of TMatrixConstRef
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrixbase.hpp $
            //---------------------------------------------------------------------
            inline
            explicit
            TMatrixConstRef(const TMatrixConstRef<ValueType, DerivedType, ShapeType>& f_rhs_r)
                : m_data(&f_rhs_r(0,0))
            {
            }

        private:
            const value_type* m_data; //!< pointer to the matrix data
            explicit
            TMatrixConstRef();

            TMatrixConstRef operator=(const TMatrixConstRef<ValueType, DerivedType, ShapeType>& );

        };


    }   // namespace linalg closed


}   // namespace vfc closed

#include "vfc/linalg/vfc_linalg_matrixbase.inl"

#endif //VFC_LINALG_MATRIXBASE_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_matrixbase.hpp  $
//  Revision 1.3 2009/05/28 09:19:04MESZ Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) 
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.2 2008/07/31 10:38:32CEST Vinaykumar Setty (RBEI/EAC1) (vmr1kor) 
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:22IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
