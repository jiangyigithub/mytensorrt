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
///     $Source: vfc_linalg_matrix33.hpp $
///     $Revision: 1.4 $
///     $Author: gaj2kor $
///     $Date: 2008/08/29 15:04:50MESZ $
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

#ifndef VFC_LINALG_MATRIX33_HPP_INCLUDED
#define VFC_LINALG_MATRIX33_HPP_INCLUDED

#include "vfc/linalg/vfc_linalg_matrixmn.hpp"

namespace vfc
{   // namespace vfc opened

    namespace linalg
    {   // namespace linalg opened

        //=====================================================================
        // TMatrix33
        //---------------------------------------------------------------------
        //! Special types Matrix class for a 3X3 matrix
        //! @param ValueType     Data Type
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg_tiny
        //! $Source: vfc_linalg_matrix33.hpp $
        //=====================================================================
        template <class ValueType>
        class TMatrix33 : public TMatrixMN<ValueType, 3, 3>
        {
        public:

            typedef TMatrixMN<ValueType, 3, 3>                  base_type;
            /// Data type of the linalg::TMatrixMN.
            typedef typename base_type::value_type              value_type;
            /// Reference type of the linalg::TMatrix22 data elements.
            typedef typename base_type::reference               reference;
            /// const reference type of the linalg::TMatrix22 data elements.
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
            //! Default constructor. Matrix size will be 3,3. Does not initialize the values to 0.
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix33.hpp $
            //---------------------------------------------------------------------
            inline TMatrix33(void) : TMatrixMN<ValueType, 3, 3> () { }

            //---------------------------------------------------------------------
            //! Default constructor. Matrix size will be 3,3. Initialize the values to f_value_r.
            //! @param f_value_r        Scalar value
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix33.hpp $
            //---------------------------------------------------------------------
            inline
            explicit
            TMatrix33(const ValueType& f_value_r)
                : TMatrixMN<ValueType, 3, 3>(f_value_r){ }

            //---------------------------------------------------------------------
            //! Constructor takes 9 prameters and initializes all data
            //! @param f_value00_r      Marix Value at location 00, of type ValueType
            //! @param f_value01_r      Marix Value at location 01, of type ValueType
            //! @param f_value02_r      Marix Value at location 02, of type ValueType
            //! @param f_value10_r      Marix Value at location 10, of type ValueType
            //! @param f_value11_r      Marix Value at location 11, of type ValueType
            //! @param f_value12_r      Marix Value at location 12, of type ValueType
            //! @param f_value20_r      Marix Value at location 20, of type ValueType
            //! @param f_value21_r      Marix Value at location 21, of type ValueType
            //! @param f_value22_r      Marix Value at location 22, of type ValueType
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix33.hpp $
            //---------------------------------------------------------------------
            TMatrix33(const ValueType& f_value00_r, const ValueType& f_value01_r,
                      const ValueType& f_value02_r, const ValueType& f_value10_r,
                      const ValueType& f_value11_r, const ValueType& f_value12_r,
                      const ValueType& f_value20_r, const ValueType& f_value21_r,
                      const ValueType& f_value22_r);

            //---------------------------------------------------------------------
            //! Copy constructor
            //! @param f_param_r      Object of TMatrix33
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix33.hpp $
            //---------------------------------------------------------------------
            TMatrix33(const TMatrix33<ValueType>& f_param_r);

            //---------------------------------------------------------------------
            //! Contructor copies from the given 2D expression
            //! @param f_param_r      Object of TExp2D
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix33.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType>
            inline
            explicit
            TMatrix33(const intern::TExp2D<ValueType, OperatorType,
                TStaticRectangle<3, 3> >& f_param_r)
                : TMatrixMN<ValueType, 3, 3>(f_param_r){ }

            //---------------------------------------------------------------------
            //! Constructor copies from TMatrixMN
            //! @param f_param_r      Object of TMatrixMN
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix33.hpp $
            //---------------------------------------------------------------------
            explicit
            TMatrix33(const TMatrixMN<ValueType, 3, 3>& f_param_r);

            using TMatrixMN<ValueType,3,3>::operator=;
            using TMatrixMN<ValueType,3,3>::operator();

            using TMatrixMN<ValueType,3,3>::set;
            //---------------------------------------------------------------------
            //! Set functionality for initializing the Matrix with given value
            //! @param f_value00_r      Marix Value at location 00, of type ValueType
            //! @param f_value01_r      Marix Value at location 01, of type ValueType
            //! @param f_value02_r      Marix Value at location 02, of type ValueType
            //! @param f_value10_r      Marix Value at location 10, of type ValueType
            //! @param f_value11_r      Marix Value at location 11, of type ValueType
            //! @param f_value12_r      Marix Value at location 12, of type ValueType
            //! @param f_value20_r      Marix Value at location 20, of type ValueType
            //! @param f_value21_r      Marix Value at location 21, of type ValueType
            //! @param f_value22_r      Marix Value at location 22, of type ValueType
            //! @author  jat2hi
            //! $Source: vfc_linalg_matrix33.hpp $
            //---------------------------------------------------------------------
            void set(const ValueType& f_value00_r, const ValueType& f_value01_r,
                     const ValueType& f_value02_r, const ValueType& f_value10_r,
                     const ValueType& f_value11_r, const ValueType& f_value12_r,
                     const ValueType& f_value20_r, const ValueType& f_value21_r,
                     const ValueType& f_value22_r);


        };

        // convenience typedefs
        typedef linalg::TMatrix33<vfc::int32_t>   CMatrix33i;
        typedef linalg::TMatrix33<vfc::float32_t> CMatrix33f;
        typedef linalg::TMatrix33<vfc::float64_t> CMatrix33d;

    }   // namespace linalg closed

}   // namespace vfc closed

#include "vfc/linalg/vfc_linalg_matrix33.inl"

#endif //VFC_LINALG_MATRIX33_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_matrix33.hpp  $
//  Revision 1.4 2008/08/29 15:04:50MESZ gaj2kor 
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.3 2008/07/31 14:08:28IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.2 2007/06/22 18:40:00IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  - corrected redeclared function parameter names (mantis 1692)
//  Revision 1.1 2007/05/09 10:21:20CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
