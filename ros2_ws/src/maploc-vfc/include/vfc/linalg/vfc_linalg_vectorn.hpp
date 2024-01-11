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
///     $Source: vfc_linalg_vectorn.hpp $
///     $Revision: 1.6 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/08/18 16:25:04MESZ $
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

#ifndef VFC_LINALG_VECTORN_HPP_INCLUDED
#define VFC_LINALG_VECTORN_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_assert.hpp"
#include "vfc/linalg/vfc_linalg_shapepromotion.hpp"
#include "vfc/linalg/vfc_linalg_vectorbase.hpp"
#include <iterator>

// forward declarations
namespace vfc
{  // namespace vfc opened

    namespace linalg
    {  // namespace linalg opened

        template <vfc::int32_t RowValue, vfc::int32_t ColValue>
        struct TStaticRectangle;

        //-------------------------------------------------------------------------
        // conditional doxygen documentation
        //! @cond VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

        namespace intern
        {   // namespace intern opened

            template <class ValueType, class OperatorType, class ShapeType>
            class TExp1D;

        }   // namespace intern closed

        //-------------------------------------------------------------------------
        //! @endcond
        // of VFC_DOXY_INTERN
        //-------------------------------------------------------------------------

    }   // namespace linalg closed

}   // namespace vfc closed


namespace vfc
{   //namespace vfc opened

    namespace linalg
    {   //namespace linalg opened

        //=====================================================================
        // TVectorN
        //---------------------------------------------------------------------
        //! Static vector class.
        //! Makes use of the "curiously recurring template pattern" to avoid
        //! virtual function calls when using operators via the TVectorBase type.
        //! @param ValueType      Data Type
        //! @param DerivedType    Data Type
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg_fixed
        //! $Source: vfc_linalg_vectorn.hpp $
        //=====================================================================
        template <class ValueType, vfc::int32_t RowValue>
        class TVectorN : public TVectorBase<ValueType,
            TVectorN<ValueType, RowValue>,
            TStaticRectangle<RowValue, 1> >
        {
        public:
            typedef TVectorBase<ValueType,
                    TVectorN<ValueType, RowValue>,
                    TStaticRectangle<RowValue, 1> >             base_type;
            /// Data type of the linalg::TVectorN.
            typedef typename base_type::value_type              value_type;
            /// Reference type of the linalg::TVectorN data elements.
            typedef typename base_type::reference               reference;
            /// const reference type of the linalg::TVectorN data elements.
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

            //---------------------------------------------------------------------
            //! Default constructor. Does not do any operation
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            TVectorN(void) {}

            //---------------------------------------------------------------------
            //! Defaults the elements of the Vector to f_default while construction
            //! @param f_nbRows            Number fo rows
            //! @param f_default           ValueType()
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            explicit
            TVectorN(size_type f_nbRows,
                const ValueType& f_default = ValueType());

            //---------------------------------------------------------------------
            //! Constructor copies from given 1D expression
            //! @param f_param_r            Object for TExp1D
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType>
            explicit
            TVectorN(const intern::TExp1D<ValueType, OperatorType,
                TStaticRectangle<RowValue, 1 > >& f_param_r);

            //---------------------------------------------------------------------
            //! Copy constructor
            //! @param f_param_r            Object for TVectorN
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            inline TVectorN(const TVectorN<ValueType, RowValue>& f_param_r)
                : TVectorBase<ValueType, TVectorN<ValueType, RowValue>, TStaticRectangle<RowValue, 1> >()
            {
                operator =(f_param_r);
            }

            //---------------------------------------------------------------------
            //! Returns const ref to element[row]. asserts if invalid dimension is provided
            //! @param pos            Index value
            //! @return  Returns const ref to element[row].
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            inline const_reference operator[](size_type pos)  const
            {
                VFC_REQUIRE(pos < NB_ROWS);
                return m_data[pos];
            }

            //---------------------------------------------------------------------
            //! Returns ref to element[row]. asserts if invalid dimension is provided
            //! @param pos            Index value
            //! @return  Returns ref to element[row].
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            inline reference operator[](size_type pos)
            {
                VFC_REQUIRE(pos < NB_ROWS);
                return m_data[pos];
            }

            //---------------------------------------------------------------------
            //! Returns the number of elements in the Vector.
            //! @param void
            //! @return  Returns the number of elements in the Vector.
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            inline size_type getDim(void) const
            {
                return NB_ROWS;
            }

            //---------------------------------------------------------------------
            //! Returns value at vector(0)
            //! @param void
            //! @return Returns value at vector(0)
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            const_iterator  begin   (void) const    {   return &m_data[0]; }
            //---------------------------------------------------------------------
            //! Returns value at vector(0)
            //! @param void
            //! @return Returns value at vector(0)
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            iterator        begin   (void)          {   return &m_data[0]; }

            //---------------------------------------------------------------------
            //! Returns value at last index of vector
            //! @param void
            //! @return Returns value at last index of vector
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            const_iterator  end     (void) const    {   return begin()+NB_ROWS; }
            //---------------------------------------------------------------------
            //! Returns value at last index of vector
            //! @param void
            //! @return Returns value at last index of vector
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            iterator        end     (void)          {   return begin()+NB_ROWS; }

            //---------------------------------------------------------------------
            //! Assignment operator, assigns a static vector
            //! @param f_rhs_r            Object for TExp1D
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            template <class OperatorType>
            const TVectorN& operator=(
                const intern::TExp1D<ValueType, OperatorType,
                    TStaticRectangle<RowValue, 1> >& f_rhs_r);

            //---------------------------------------------------------------------
            //! Assignment operator, assigns a 1D expression vector
            //! @param f_rhs_r            Object for TVectorN
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            const TVectorN& operator=(const TVectorN<ValueType,
                RowValue>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Copies the contents of a static vector to itself
            //! @param f_rhs_r            Object for TVectorN
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            void assign(const TVectorN<ValueType, RowValue>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Copies the contents of a 1D Expression to itself
            //! @param f_rhs_r            Object for TExp1D
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            template <class OperatorType>
            void assign(const intern::TExp1D<ValueType, OperatorType,
                TStaticRectangle<RowValue, 1> >& f_rhs_r);

            //---------------------------------------------------------------------
            //! Copies the contents of itself to a Static Vector
            //! @param f_rhs_r            Object for TVectorN
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            void assign_to(
            TVectorN<value_type, RowValue >& f_rhs_r) const;

            //---------------------------------------------------------------------
            //! Performs the binary operation specified with the given vector to itself
            //! @param f_rhs_r            Object for TVectorN
            //! @param f_func_r           Function pointer
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            template <class OperationFunctor>
            void opassign(
                const TVectorN<ValueType, RowValue>& f_rhs_r,
                const OperationFunctor& f_func_r);

            //---------------------------------------------------------------------
            //! Performs the binary operation specified with the given 1d expr to itself
            //! @param f_rhs_r            Object for TExp1D
            //! @param f_func_r           Function pointer
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            template <class OperatorType, class OperationFunctor>
            void opassign(
                const intern::TExp1D<ValueType, OperatorType,
                    TStaticRectangle<RowValue, 1> >& f_rhs_r,
                const OperationFunctor& f_func_r);

            //---------------------------------------------------------------------
            //! Performs the unary operation specified with the given value to itself
            //! @param f_rhs_r            Scalar value
            //! @param f_func_r           Function pointer
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            template <class OperationFunctor>
            void unaryOpassign(const ValueType& f_rhs_r,
                const OperationFunctor& f_func_r);

            //---------------------------------------------------------------------
            //! Normalizes itself
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            void normalize();

            //---------------------------------------------------------------------
            //!  Resets every element to 0
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            void zero();

            //---------------------------------------------------------------------
            //! Set every element
            //! @param f_value            Scalar value for initialization
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            template<class T>
            void set(const T& f_value);

            //---------------------------------------------------------------------
            //! Adds a static vector to itself
            //! @param f_rhs_r            Object for TVectorN
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            const TVectorN& operator+=(const TVectorN<ValueType,
                RowValue>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Adds a 1D Expression to itself
            //! @param f_rhs_r            Object for TExp1D
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType>
            TVectorN<ValueType, RowValue>&
            operator+= (const intern::TExp1D<ValueType, OperatorType,
                TStaticRectangle<RowValue, 1> >& f_rhs_r);

            //---------------------------------------------------------------------
            //! Subtracts a Static Vector from itself
            //! @param f_rhs_r            Object for TVectorN
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            const TVectorN& operator-=(const TVectorN<ValueType,
                RowValue>& f_rhs_r);

            //---------------------------------------------------------------------
            //! Subtracts a 1D Expression form itself
            //! @param f_rhs_r            Object for TExp1D
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType>
            TVectorN<ValueType, RowValue>&
            operator-= (const intern::TExp1D<ValueType, OperatorType,
                TStaticRectangle<RowValue, 1> >& f_rhs_r);

            //---------------------------------------------------------------------
            //! Multiplies a Scalar to itself
            //! @param f_value_r            Scalar value for multiplication
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            TVectorN<ValueType, RowValue>&
            operator*= (
                const ValueType& f_value_r)  ;

            //---------------------------------------------------------------------
            //! Divides itself by a Scalar
            //! @param f_value_r            Scalar value for division
            //! @author  jat2hi
            //! $Source: vfc_linalg_vectorn.hpp $
            //---------------------------------------------------------------------
            TVectorN<ValueType, RowValue>&
            operator/= (
                const ValueType& f_value_r)  ;

        private:
            /// Returns the Length of the Vector
            value_type getLength() const;

            /// Array of the vector elements
            value_type m_data[RowValue];
        };

    }   //namespace linalg closed

}   //namespace vfc closed

#include "vfc/linalg/vfc_linalg_vectorn.inl"

#endif //VFC_LINALG_VECTORN_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_vectorn.hpp  $
//  Revision 1.6 2014/08/18 16:25:04MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - ...should be explicitly initialized in the copy constructor (mantis0004594)
//  Revision 1.5 2009/03/25 14:48:29MEZ gaj2kor 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.4 2008/08/29 18:35:08IST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.3 2008/07/31 14:08:57IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.2 2007/06/22 18:40:01IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  - corrected redeclared function parameter names (mantis 1692)
//  Revision 1.1 2007/05/09 10:21:30CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
