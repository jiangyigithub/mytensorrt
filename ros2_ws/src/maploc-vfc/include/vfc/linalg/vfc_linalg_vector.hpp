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
///     $Source: vfc_linalg_vector.hpp $
///     $Revision: 1.10 $
///     $Author: Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) $
///     $Date: 2009/05/28 09:19:06MESZ $
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

#ifndef VFC_LINALG_VECTOR_HPP_INCLUDED
#define VFC_LINALG_VECTOR_HPP_INCLUDED

// libc
#include <vector>
#include <iterator>

// core
#include "vfc/core/vfc_types.hpp"

// memory
#include "vfc/memory/vfc_freestore_allocator.hpp"

// linalg
#include "vfc/linalg/vfc_linalg_vectorbase_dyn.hpp"

// forward declarations
namespace vfc
{  // namespace vfc opened

    namespace linalg
    {  // namespace linalg opened

        struct CDynamicRectangle;

        //-------------------------------------------------------------------------
        // conditional doxygen documentation
        //! @cond VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

        namespace intern
        {  // namespace intern opened

            template <class ValueType, class OperatorType, class ShapeType>
            class TExp1D;

        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

        template <class ValueType, vfc::int32_t RowValue>
        class TVectorN;

    }   // namespace linalg closed

}   // namespace vfc closed



namespace vfc
{
    namespace linalg
    {
        //=====================================================================
        // TVector
        //---------------------------------------------------------------------
        //! Dynamic Vector class
        //! Makes use of the "curiously recurring template pattern" to avoid
        //! virtual function calls when using operators via the TVectorBase type
        //! @param ValueType      Data type
        //! @param StorageType    std::vector<ValueType>
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg_dynamic
        //! $Source: vfc_linalg_vector.hpp $
        //=====================================================================
        template<typename ValueType, typename StorageType = stlalias::vector< ValueType, vfc::TFreeStoreAllocator<ValueType> > >
        class TVector : public TVectorBase<ValueType,
            TVector<ValueType, StorageType>, CDynamicRectangle>
        {
            public:

            typedef ValueType                                   value_type;
            /// Reference type of the linalg::TMatrixMN data elements.
            typedef ValueType&                                  reference;
            /// const reference type of the linalg::TMatrixMN data elements.
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
            typedef StorageType                                 storage_type;

            enum { NB_ROWS = 0 };

            //---------------------------------------------------------------------
            //! Default constructor, Initialize m_nbRows=0
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            inline TVector() : m_nbRows(0) { }

            //---------------------------------------------------------------------
            //! Constructor - initialized Vector elements to f_val
            //! @param f_nbRows     Number of rows
            //! @param f_val_r      ValueType()
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            explicit
            inline TVector(size_type f_nbRows,
                const ValueType& f_val_r = ValueType())
            {
                resize(f_nbRows, f_val_r);
            }

            //---------------------------------------------------------------------
            //! Copy constructor
            //! @param f_param_r        Object of TVector
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            TVector(const TVector<ValueType, StorageType>& f_param_r);

            //---------------------------------------------------------------------
            //! Constructor copies from given static vector
            //! @param f_param_r        Object of TVectorN
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue>
            explicit
            TVector(const TVectorN<ValueType, RowValue>& f_param_r);

            //---------------------------------------------------------------------
            //! Constructor - copies from an expression
            //! @param f_param_r        Object of TExp1D
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType, class ShapeType>
            explicit
            TVector(const intern::TExp1D<ValueType,OperatorType,ShapeType>& f_param_r);

            //---------------------------------------------------------------------
            //! Resizes the vector to size f_nbRows
            //! @param f_nbRows     Number of rows
            //! @note               If the container's size is less than the requested size,
            //!                     elements are added to the container until it reaches
            //!                     the requested size. If the container's size is larger than
            //!                     the requested size, the elements closest to the end of the
            //!                     container are deleted until the container reaches the size.
            //!                     If the present size of the container is the same
            //!                     as the requested size, no action is taken.
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            void resize (size_type f_nbRows);

            //---------------------------------------------------------------------
            //! Resizes the vector to size f_nbRows and sets the elements to f_val
            //! @param f_nbRows     Number of rows
            //! @param f_val_r      Value for initialization
            //! @note               The f_val_r is value of new elements added to the vector,
            //!                     if the new size is larger that the original size.
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            void resize (size_type f_nbRows,
                const value_type& f_val_r);

            //---------------------------------------------------------------------
            //! Returns reference to single element from the Vector
            //! @param pos       Index value
            //! @return  Returns reference to single element from the Vector
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            inline reference operator[](size_type pos)
            {
                VFC_REQUIRE(pos < m_nbRows);
                return m_data[pos];
            }

            //---------------------------------------------------------------------
            //! Returns const reference to single element from the Vector
            //! @param pos       Index value
            //! @return  Returns const reference to single element from the Vector
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            inline const_reference operator[](size_type pos) const
            {
                VFC_REQUIRE(pos < m_nbRows);
                return m_data[pos];
            }

            //---------------------------------------------------------------------
            //! Returns the number of elements in the vector
            //! @param void
            //! @return  Returns the number of elements in the vector
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            inline size_type getDim(void) const
            {
                return m_nbRows;
            }

            //---------------------------------------------------------------------
            //! Returns value at vector(0)
            //! @param void
            //! @return  Returns value at vector(0)
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            const_iterator  begin   (void) const    {   return m_data.begin(); }

            //---------------------------------------------------------------------
            //! Returns value at vector(0)
            //! @param void
            //! @return  Returns value at vector(0)
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            iterator        begin   (void)          {   return m_data.begin(); }

            //---------------------------------------------------------------------
            //! Returns value at last index of vector
            //! @param void
            //! @return  Returns value at last index of vector
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            const_iterator  end     (void) const    {   return m_data.end(); }

            //---------------------------------------------------------------------
            //! Returns value at last index of  vector
            //! @param void
            //! @return  Returns value at last index of vector
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            iterator        end     (void)          {   return m_data.end(); }

            //---------------------------------------------------------------------
            //! Assignment operator, assigns a dynamic vector
            //! @param f_rhs_r       Object for TVector
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            const TVector<ValueType, StorageType>&
            operator=(const TVector<ValueType, StorageType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Assignment operator, assigns a static vector
            //! @param f_rhs_r       Object for TVectorN
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            template<vfc::int32_t RowValue>
            const TVector<ValueType, StorageType>&
            operator=(const TVectorN<ValueType, RowValue>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Assignment operator, assigns a 1D Expression
            //! @param f_rhs_r       Object for TExp1D
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType, class ShapeType>
            const TVector<ValueType, StorageType>& operator=(const intern::TExp1D<ValueType,
                OperatorType,ShapeType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Copies the contents of a given dynamic vector
            //! @param f_rhs_r          Object for TVector
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            void assign(const TVector<ValueType, StorageType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Copies the contents of a given static vector
            //! @param f_rhs_r       Object for TVectorN
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            template <vfc::int32_t RowValue>
            void assign(const TVectorN<ValueType, RowValue>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Copies the contents of a given 1D expression
            //! @param f_rhs_r       Object for TExp1D
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            template <class OperatorType, class ShapeType>
            void assign(const intern::TExp1D<ValueType, OperatorType,
                ShapeType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Copies its contents to a given dynamic vector
            //! @param f_rhs_r       Object for TVector
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            void assign_to(TVector<ValueType, StorageType>& f_rhs_r) const;

            //---------------------------------------------------------------------
            //! opassign
            //! @param f_rhs_r          Object for TVector
            //! @param f_functor_r      Function pointer
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            template <class OperationFunctor>
            void opassign(const TVector<ValueType, StorageType> & f_rhs_r,
                const OperationFunctor& f_functor_r);

            //---------------------------------------------------------------------
            //! opassign
            //! @param f_rhs_r          Object for TVectorN
            //! @param f_functor_r      Function pointer
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            template <vfc::int32_t RowValue, class OperationFunctor>
            void opassign(const TVectorN<ValueType,
                    RowValue> & f_rhs_r,
                const OperationFunctor& f_functor_r);

            //---------------------------------------------------------------------
            //! opassign
            //! @param f_rhs_r          Object for TExp1D
            //! @param f_functor_r      Function pointer
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            template <class OperatorType, class ShapeType, class OperationFunctor>
            void opassign(const intern::TExp1D<ValueType, OperatorType,
                    ShapeType>& f_rhs_r,
                const OperationFunctor& f_functor_r);

            //---------------------------------------------------------------------
            //! unaryOpassign
            //! @param f_rhs_r           Scalar value
            //! @param f_functor_r       Function pointer
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            template <class OperationFunctor>
            void unaryOpassign(const ValueType& f_rhs_r,
                const OperationFunctor& f_functor_r);

            //---------------------------------------------------------------------
            //! Normalizes itself
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            void normalize();

            //---------------------------------------------------------------------
            //! Resets the vector elements to 0
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            void zero();

            //---------------------------------------------------------------------
            //! Set vector elements to user passed value
            //! @param f_value_r    Scalar Value
            //! @author  vmr1kor
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            void set(const ValueType& f_value_r);

            //---------------------------------------------------------------------
            //! Adds a given static / dynamic vector to itself. asserts if shape is invalid
            //! @param f_rhs_r       Object for TVectorBase
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            template<class VectorType, class ShapeType>
            TVector<ValueType, StorageType>&
            operator+= (const TVectorBase<ValueType, VectorType, ShapeType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Adds a given 1D expression to itself. asserts if shape is invalid
            //! @param f_rhs_r       Object for TExp1D
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType, class ShapeType>
            TVector<ValueType, StorageType>&
            operator+= (const intern::TExp1D<ValueType, OperatorType,
                ShapeType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Subtracts a given static / dynamic vector from itself. asserts if shape is invalid
            //! @param f_rhs_r       Object for TVectorBase
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            template<class VectorType, class ShapeType>
            TVector<ValueType, StorageType>&
            operator-= (const TVectorBase<ValueType, VectorType, ShapeType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Subtracts a given 2D expression from itself. asserts if shape is invalid
            //! @param f_rhs_r       Object for TExp1D
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType, class ShapeType>
            TVector<ValueType, StorageType>&
            operator-= (const intern::TExp1D<ValueType, OperatorType,
                ShapeType>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Multiplies a given scalar to itself
            //! @param f_value_r       Scalar value for multiplication
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            TVector<ValueType, StorageType>&
            operator*= (const ValueType& f_value_r);

            //---------------------------------------------------------------------
            //! Divides itself by a given scalar . asserts if value is 0.
            //! @param f_value_r       Scalar value for Division
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            TVector<ValueType, StorageType>&
            operator/= (const ValueType& f_value_r);

        protected:

            //---------------------------------------------------------------------
            //! Returns the lenght of itself
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector.hpp $
            //---------------------------------------------------------------------
            ValueType getLength() const;

        private:
            /// Saves the number of rows of the vector
            size_type m_nbRows;
            /// Vector Data
            StorageType m_data;
        };
    }
}
#include "vfc/linalg/vfc_linalg_vector.inl"

#endif //VFC_LINALG_VECTOR_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_vector.hpp  $
//  Revision 1.10 2009/05/28 09:19:06MESZ Muehlmann Karsten (CC/EYV2 CC/PJ-FA1) (MUK2LR) 
//  - replace std:: with stlalias:: (mantis2720)
//  Revision 1.9 2009/03/25 14:48:26CET Gaurav Jain (RBEI/EAC1) (gaj2kor) 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.8 2008/09/11 17:25:06IST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  -Documentation related to the functionality of vfc::linalg::TMatrix, vfc::linalg::TVector : resize()
//  (Mantis : 1727)
//  Revision 1.7 2008/08/29 18:32:40IST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  Removal of template parameter from set functionality (Mantis :2176)
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.6 2008/08/11 16:33:08IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - changed std::vector default allocator (mantis2270)
//  Revision 1.5 2008/07/31 13:13:33CEST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  set function doxy comment added(Mantis :- 1744 )
//  Revision 1.4 2008/07/31 14:08:48IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.3 2008/07/21 11:06:23IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  set() function added ( Mantis : 2176 )
//  Revision 1.2 2007/06/22 18:40:01IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  - corrected redeclared function parameter names (mantis 1692)
//  Revision 1.1 2007/05/09 10:21:26CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
