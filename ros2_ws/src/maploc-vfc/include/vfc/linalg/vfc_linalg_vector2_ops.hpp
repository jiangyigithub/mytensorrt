//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2007 by Robert Bosch GmbH. All rights reserved.
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
///     $Source: vfc_linalg_vector2_ops.hpp $
///     $Revision: 1.5 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2011/01/21 12:53:32MEZ $
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

#ifndef VFC_LINALG_VECTOR2_OPS_HPP_INCLUDED
#define VFC_LINALG_VECTOR2_OPS_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

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

        template<class ValueType, class DerivedType, class ShapeType>
        class TVectorBase;

    }   // namespace linalg closed

}   // namespace vfc closed


namespace vfc
{// namespace vfc opened

    namespace linalg
    {// namespace linalg opened

        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg BEGIN
        //-------------------------------------------------------------------------
        /// @addtogroup vfc_group_linalg
        /// @{
        //-------------------------------------------------------------------------

        //---------------------------------------------------------------------
        //! cross function overload for vector with Size = 2:
        //! Calculates cross product of a two static vectors
        //! returns the cross product value
        //! @param f_op1_r      Object for TVectorBase
        //! @param f_op2_r      Object for TVectorBase
        //! @return Returns scalar value of type ValueType
        //! @author  jat2hi
        //! $Source: vfc_linalg_vector2_ops.hpp $
        //---------------------------------------------------------------------
        template<typename ValueType, class Vector1Type, class Vector2Type>
        ValueType
        cross (const TVectorBase<ValueType, Vector1Type,
                TStaticRectangle<2, 1> >& f_op1_r,
            const TVectorBase<ValueType, Vector2Type,
                TStaticRectangle<2, 1> >& f_op2_r);

        //---------------------------------------------------------------------
        //! cross function overload for vector with Size = 2:
        //! Calculates cross product of a static vector and a 1d expression
        //! returns the cross product value
        //! @param f_op1_r      Object for TVectorBase
        //! @param f_op2_r      Object for TExp1D
        //! @return Returns scalar value of type ValueType
        //! @author  jat2hi
        //! $Source: vfc_linalg_vector2_ops.hpp $
        //---------------------------------------------------------------------
        template<typename ValueType, class VectorType, class OperatorType>
        ValueType
        cross (const TVectorBase<ValueType, VectorType,
                TStaticRectangle<2, 1> >& f_op1_r,
            const intern::TExp1D<ValueType, OperatorType,
                TStaticRectangle<2, 1> >& f_op2_r);

        //---------------------------------------------------------------------
        //! cross function overload for vector with Size = 2:
        //! Calculates cross product of a 1d expression and a static vector
        //! returns the cross product value
        //! @param f_op1_r      Object for TExp1D
        //! @param f_op2_r      Object for TVectorBase
        //! @return Returns scalar value of type ValueType
        //! @author  jat2hi
        //! $Source: vfc_linalg_vector2_ops.hpp $
        //---------------------------------------------------------------------
        template<typename ValueType, class VectorType, class OperatorType>
        ValueType
        cross (const intern::TExp1D<ValueType, OperatorType,
                TStaticRectangle<2, 1> >& f_op1_r,
            const TVectorBase<ValueType, VectorType,
                TStaticRectangle<2, 1> >& f_op2_r);

        //---------------------------------------------------------------------
        //! cross function overload for vector with Size = 2:
        //! Calculates cross product of two 1d expressions
        //! returns the cross product value
        //! @param f_op1_r      Object for TExp1D
        //! @param f_op2_r      Object for TExp1D
        //! @return Returns scalar value of type ValueType
        //! @author  jat2hi
        //! $Source: vfc_linalg_vector2_ops.hpp $
        //---------------------------------------------------------------------
        template<typename ValueType, class Operator1Type, class Operator2Type>
        ValueType
        cross (const intern::TExp1D<ValueType, Operator1Type,
                TStaticRectangle<2, 1> >& f_op1_r,
            const intern::TExp1D<ValueType, Operator2Type,
                TStaticRectangle<2, 1> >& f_op2_r);

        //---------------------------------------------------------------------
        //! area function overload for vector with Size = 2:
        //! @param f_op1_r      Object for TVectorBase
        //! @param f_op2_r      Object for TVectorBase
        //! @return Returns scalar value of type ValueType
        //! @author  jat2hi
        //! $Source: vfc_linalg_vector2_ops.hpp $
        //---------------------------------------------------------------------
        template<typename ValueType, class Vector1Type, class Vector2Type>
        ValueType
        area (const TVectorBase<ValueType, Vector1Type,
                TStaticRectangle<2, 1> >& f_op1_r,
            const TVectorBase<ValueType, Vector2Type,
                TStaticRectangle<2, 1> >& f_op2_r);

        //---------------------------------------------------------------------
        //! area function overload for vector with Size = 2:
        //! @param f_op1_r      Object for TVectorBase
        //! @param f_op2_r      Object for TExp1D
        //! @return Returns scalar value of type ValueType
        //! @author  jat2hi
        //! $Source: vfc_linalg_vector2_ops.hpp $
        //---------------------------------------------------------------------
        template<typename ValueType, class VectorType, class OperatorType>
        ValueType
        area (const TVectorBase<ValueType, VectorType,
                TStaticRectangle<2, 1> >& f_op1_r,
            const intern::TExp1D<ValueType, OperatorType,
                TStaticRectangle<2, 1> >& f_op2_r);

        //---------------------------------------------------------------------
        //! area function overload for vector with Size = 2:
        //! @param f_op1_r      Object for TExp1D
        //! @param f_op2_r      Object for TVectorBase
        //! @return Returns scalar value of type ValueType
        //! @author  jat2hi
        //! $Source: vfc_linalg_vector2_ops.hpp $
        //---------------------------------------------------------------------
        template<typename ValueType, class VectorType, class OperatorType>
        ValueType
        area (const intern::TExp1D<ValueType, OperatorType,
                TStaticRectangle<2, 1> >& f_op1_r,
            const TVectorBase<ValueType, VectorType,
                TStaticRectangle<2, 1> >& f_op2_r);

        //---------------------------------------------------------------------
        //! area function overload for vector with Size = 2:
        //! @param f_op1_r      Object for TExp1D
        //! @param f_op2_r      Object for TExp1D
        //! @return Returns scalar value of type ValueType
        //! @author  jat2hi
        //! $Source: vfc_linalg_vector2_ops.hpp $
        //---------------------------------------------------------------------
        template<typename ValueType, class Operator1Type, class Operator2Type>
        ValueType
        area (const intern::TExp1D<ValueType, Operator1Type,
                TStaticRectangle<2, 1> >& f_op1_r,
            const intern::TExp1D<ValueType, Operator2Type,
                TStaticRectangle<2, 1> >& f_op2_r);
        //=========================================================================
        //  DOYGEN ADDTOGROUP vfc_group_linalg END
        //-------------------------------------------------------------------------
        /// @}
        //-------------------------------------------------------------------------

    }   // namespace linalg closed


}   // namespace vfc closed

#include "vfc/linalg/vfc_linalg_vector2_ops.inl"

#endif //VFC_LINALG_VECTOR2_OPS_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_vector2_ops.hpp  $
//  Revision 1.5 2011/01/21 12:53:32MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - sometimes .hpp or namespace qualifiers are missing (mantis3599)
//  Revision 1.4 2009/03/25 14:48:27MEZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Wrap vfc::linalg::intern members with doxygen statements for conditional documentation generation. (mantis 1758)
//  Revision 1.3 2008/08/29 18:35:02IST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  Addition of doxygen ingroup comment  (Mantis :2269)
//  Revision 1.2 2008/07/31 14:08:49IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:27IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
