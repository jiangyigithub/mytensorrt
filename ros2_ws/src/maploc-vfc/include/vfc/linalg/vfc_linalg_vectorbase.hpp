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
///     $Source: vfc_linalg_vectorbase.hpp $
///     $Revision: 1.4 $
///     $Author: Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) $
///     $Date: 2009/03/16 17:32:39MEZ $
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

#ifndef VFC_LINALG_VECTORBASE_HPP_INCLUDED
#define VFC_LINALG_VECTORBASE_HPP_INCLUDED

#include <iterator>
#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_assert.hpp"

namespace vfc
{   // namespace vfc opened

    namespace linalg
    {   // namespace linalg opened

        //=====================================================================
        // TVectorBase
        //---------------------------------------------------------------------
        //!  Vector Base class
        //! @param ValueType      Data Type
        //! @param DerivedType    Data Type
        //! @param ShapeType      Data Type
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_vectorbase.hpp $
        //=====================================================================
        template<class ValueType, class DerivedType, class ShapeType>
        class TVectorBase
        {
        public:

            ///  Data type of the linalg::TVectorBase.
            typedef ValueType                               value_type;
            ///  Reference type of the linalg::TVectorBase data elements.
            typedef ValueType&                              reference;
            ///  const reference type of the linalg::TVectorBase data elements.
            typedef const ValueType&                        const_reference;
            ///  STL pointer interface.
            typedef ValueType*                              pointer;
            ///  STL const_pointer interface.
            typedef const ValueType*                        const_pointer;
            ///  STL iterator interface.
            typedef ValueType*                              iterator;
            ///  STL const_iterator interface.
            typedef const ValueType*                        const_iterator;
            ///  STL reverse iterator interface.
            typedef stlalias::reverse_iterator<iterator>       reverse_iterator;
            ///  STL const reverse iterator interface.
            typedef stlalias::reverse_iterator<const_iterator> const_reverse_iterator;
            ///  Derived class type.
            typedef DerivedType                             derived_type;
            ///  Shape type
            typedef ShapeType                               shape_type;
            ///  Size type
            typedef vfc::int32_t                            size_type;

            enum { NB_ROWS = shape_type::NB_ROWS };

            //---------------------------------------------------------------------
            //!  Returns const ref to element[f_row]
            //!        Coding rule violation for the sake of speed
            //! @param f_row        Row number
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorbase.hpp $
            //---------------------------------------------------------------------
            const_reference operator[] (size_type f_row) const;

            //---------------------------------------------------------------------
            //!  Returns ref to element[row]
            //!        Coding rule violation for the sake of speed
            //! @param f_row        Row number
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorbase.hpp $
            //---------------------------------------------------------------------
            reference operator[] (size_type f_row);

            //---------------------------------------------------------------------
            //!  Returns number of rows
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorbase.hpp $
            //---------------------------------------------------------------------
            inline size_type getDim(void) const
            {
                return shape_type::NB_ROWS;
            }

        protected:
            //---------------------------------------------------------------------
            //!  Default constructor
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorbase.hpp $
            //---------------------------------------------------------------------
            inline
            explicit
            TVectorBase() { }

            //---------------------------------------------------------------------
            //!  Constructor
            //! @param TVectorBase      Object for TVectorBase
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorbase.hpp $
            //---------------------------------------------------------------------
            inline
            explicit
            TVectorBase(const TVectorBase<ValueType, DerivedType, ShapeType>&) { }

            //---------------------------------------------------------------------
            //!  Assignment operation
            //! @param f_rhs_r          Object for TVectorBase
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorbase.hpp $
            //---------------------------------------------------------------------
            const TVectorBase<ValueType, DerivedType, ShapeType>&
            operator=(const TVectorBase<ValueType, DerivedType, ShapeType>& f_rhs_r);

            //---------------------------------------------------------------------
            //!  Destroy object of TVectorBase
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorbase.hpp $
            //---------------------------------------------------------------------
            inline
            ~TVectorBase(){}
        };

        //=====================================================================
        // TVectorConstRef
        //---------------------------------------------------------------------
        //!  Provides a constant reference to the Vector
        //! @param ValueType      Data Type
        //! @param DerivedType    Data Type
        //! @param ShapeType      Data Type
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_vectorbase.hpp $
        //=====================================================================
        template <class ValueType, class DerivedType, class ShapeType>
        class TVectorConstRef
        {
        public:

            ///  Data type of the linalg::TVectorBase.
            typedef ValueType                               value_type;
            ///  Reference type of the linalg::TVectorBase data elements.
            typedef ValueType&                              reference;
            ///  const reference type of the linalg::TVectorBase data elements.
            typedef const ValueType&                        const_reference;
            ///  STL iterator interface.
            typedef ValueType*                              iterator;
            ///  STL const_iterator interface.
            typedef const ValueType*                        const_iterator;
            ///  STL reverse iterator interface.
            typedef stlalias::reverse_iterator<iterator>       reverse_iterator;
            ///  STL const reverse iterator interface.
            typedef stlalias::reverse_iterator<const_iterator> const_reverse_iterator;
            ///  Derived class type.
            typedef DerivedType                             derived_type;
            ///  Shape type
            typedef ShapeType                               shape_type;
            ///  Size type
            typedef vfc::int32_t                            size_type;

            enum { NB_ROWS = shape_type::NB_ROWS};

            //---------------------------------------------------------------------
            //!  Constructor, saves refernces of the vector class
            //! @param f_param_r            Object for TVectorBase
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorbase.hpp $
            //---------------------------------------------------------------------
            inline
            TVectorConstRef(const TVectorBase<ValueType, DerivedType, ShapeType>& f_param_r) :
                m_data(&f_param_r[0])
            { }

            //---------------------------------------------------------------------
            //!  Returns const ref to element(row,col). asserts if arguments are invalid.
            //! @param f_row            Row number
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorbase.hpp $
            //---------------------------------------------------------------------
            inline
            const_reference operator[](size_type f_row) const
            {
                VFC_REQUIRE(f_row < NB_ROWS);
                return m_data[f_row];
            }

            //---------------------------------------------------------------------
            //!  Returns number of rows
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorbase.hpp $
            //---------------------------------------------------------------------
            inline size_type getDim(void) const
            {
                return shape_type::NB_ROWS;
            }

            //---------------------------------------------------------------------
            //!  Default constructor
            //! @param f_rhs_r          Object for TVectorConstRef
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorbase.hpp $
            //---------------------------------------------------------------------
            inline
            explicit
            TVectorConstRef(const TVectorConstRef<ValueType, DerivedType, ShapeType>& f_rhs_r)
                : m_data(&f_rhs_r[0])
            {
            }

        private:
            const value_type* m_data; /// pointer to the vector data
            explicit
            TVectorConstRef();
            TVectorConstRef operator=(const TVectorConstRef<ValueType, DerivedType, ShapeType>& );

            //Dummy function for suppression of QAC++ msg 2141
            //Msg 2141 : The subscript 'operator []' is not available in a non-const version.
            reference operator[](size_type f_row);

        };

    }   // namespace linalg closed

}   // namespace vfc closed

#include "vfc/linalg/vfc_linalg_vectorbase.inl"

#endif //VFC_LINALG_VECTORBASE_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_vectorbase.hpp  $
//  Revision 1.4 2009/03/16 17:32:39MEZ Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) 
//  use stalalias:: to enable STL switching (mantis2723)
//  Revision 1.3 2009/01/07 05:50:14CET Gaurav Jain (RBEI/EAE5) (gaj2kor) 
//  QAC++ Warning (2141) Removal.
//  (Mantis : 0002479)
//  Revision 1.2 2008/07/31 14:08:55IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:29IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
