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
///     $Source: vfc_linalg_vector4.hpp $
///     $Revision: 1.4 $
///     $Author: gaj2kor $
///     $Date: 2008/08/29 15:05:06MESZ $
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

#ifndef VFC_LINALG_VECTOR4_HPP_INCLUDED
#define VFC_LINALG_VECTOR4_HPP_INCLUDED

#include "vfc/linalg/vfc_linalg_vectorn.hpp"

namespace vfc
{   // namespace vfc opened

    namespace linalg
    {   // namespace linalg opened

        //=====================================================================
        // TVector4
        //---------------------------------------------------------------------
        //!  Special types Vector class for a 4 element Vector
        //! @param ValueType      Data type
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg_tiny
        //! $Source: vfc_linalg_vector4.hpp $
        //=====================================================================
        template <class ValueType>
        class TVector4 : public TVectorN<ValueType, 4>
        {
        public:
            typedef TVectorN<ValueType, 4>                      base_type;
            /// Data type of the linalg::TVector4.
            typedef typename base_type::value_type              value_type;
            /// Reference type of the linalg::TVector4 data elements.
            typedef typename base_type::reference               reference;
            /// const reference type of the linalg::TVector4 data elements.
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
            //!  Default constructor
            //! @param void
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
            inline
            TVector4() : TVectorN<ValueType, 4>() {}

            //---------------------------------------------------------------------
            //!  Defaults the elements of the Vector to f_value_r while construction
            //! @param f_value_r        Scalra value for initialization
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
            inline
            explicit
            TVector4(const ValueType& f_value_r)
            : TVectorN<ValueType, 4>(4, f_value_r){ }

            //---------------------------------------------------------------------
            //!  Constructor takes 4 prameters and initializes all data
            //! @param f_value0_r        Scalra value for initialization
            //! @param f_value1_r        Scalra value for initialization
            //! @param f_value2_r        Scalra value for initialization
            //! @param f_value3_r        Scalra value for initialization
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
            TVector4(const ValueType& f_value0_r, const ValueType& f_value1_r,
                    const ValueType& f_value2_r, const ValueType& f_value3_r);

            //---------------------------------------------------------------------
            //!  Copy constructor
            //! @param f_param_r        Object for TVector4
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
            TVector4(const TVector4<ValueType>& f_param_r);

            //---------------------------------------------------------------------
            //!  Constructor copies from given 1D expression
            //! @param f_param_r        Object for TExp1D
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
            template<class OperatorType>
            inline
            explicit
            TVector4(const intern::TExp1D<value_type,OperatorType,TStaticRectangle<4,1 > >& f_param_r)
            : TVectorN<ValueType, 4>(f_param_r)
            {
            }

            //---------------------------------------------------------------------
            //!  Constructor copies from TVectorN
            //! @param f_param_r        Object for TVectorN
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
            explicit
            TVector4(const TVectorN<ValueType,4>& f_param_r);

            //---------------------------------------------------------------------
            //!  Returns value at vector(0)
            //! @param void
            //! @return Returns value at vector(0)
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
            const    value_type&    x(void)    const    {    return operator [](0);}
            //---------------------------------------------------------------------
            //!  Returns value at vector(0)
            //! @param void
            //! @return Returns value at vector(0)
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
                    value_type&    x(void)            {    return operator [](0);}

            //---------------------------------------------------------------------
            //!  Returns value at vector(1)
            //! @param void
            //! @return Returns value at vector(1)
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
            const    value_type&    y(void)    const    {    return operator [](1);}
            //---------------------------------------------------------------------
            //!  Returns value at vector(1)
            //! @param void
            //! @return Returns value at vector(1)
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
                    value_type&    y(void)            {    return operator [](1);}

            //---------------------------------------------------------------------
            //!  Returns value at vector(2)
            //! @param void
            //! @return Returns value at vector(2)
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
            const    value_type&    z(void)    const    {    return operator [](2);}
            //---------------------------------------------------------------------
            //!  Returns value at vector(2)
            //! @param void
            //! @return Returns value at vector(2)
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
                    value_type&    z(void)            {    return operator [](2);}

            //---------------------------------------------------------------------
            //!  Returns value at vector(3)
            //! @param void
            //! @return Returns value at vector(3)
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
           const    value_type&    w(void)    const    {    return operator [](3);}
            //---------------------------------------------------------------------
            //!  Returns value at vector(3)
            //! @param void
            //! @return Returns value at vector(3)
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
                    value_type&    w(void)            {    return operator [](3);}

            using TVectorN<ValueType,4>::operator=;
            using TVectorN<ValueType,4>::operator[];

            using TVectorN<ValueType,4>::set;
            //---------------------------------------------------------------------
            //!  Set function for initiializing vector
            //! @param f_value0_r        Value for initialization at 0
            //! @param f_value1_r        Value for initialization at 1
            //! @param f_value2_r        Value for initialization at 2
            //! @param f_value3_r        Value for initialization at 3
            //! @author  jat2hi
            //! $Source: vfc_linalg_vector4.hpp $
            //---------------------------------------------------------------------
            void set(const ValueType& f_value0_r, const ValueType& f_value1_r,
                    const ValueType& f_value2_r, const ValueType& f_value3_r);

        };

        // convenience typedefs
        typedef linalg::TVector4<vfc::int32_t>   CVector4i;
        typedef linalg::TVector4<vfc::float32_t> CVector4f;
        typedef linalg::TVector4<vfc::float64_t> CVector4d;

    }   // namespace linalg closed

}   // namespace vfc closed

#include "vfc/linalg/vfc_linalg_vector4.inl"

#endif //VFC_LINALG_VECTOR4_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_vector4.hpp  $
//  Revision 1.4 2008/08/29 15:05:06MESZ gaj2kor 
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.3 2008/07/31 14:08:53IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.2 2007/06/22 18:40:01IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  - corrected redeclared function parameter names (mantis 1692)
//  Revision 1.1 2007/05/09 10:21:28CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
