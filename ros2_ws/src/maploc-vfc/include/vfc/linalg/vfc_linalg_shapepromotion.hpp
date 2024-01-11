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
///     $Source: vfc_linalg_shapepromotion.hpp $
///     $Revision: 1.3 $
///     $Author: vmr1kor $
///     $Date: 2008/08/12 12:23:13MESZ $
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

#ifndef VFC_LINALG_SHAPEPROMOTION_HPP_INCLUDED
#define VFC_LINALG_SHAPEPROMOTION_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_static_assert.hpp"

namespace vfc
{   // namespace vfc opened

    namespace linalg
    {   // namespace linalg opened

        //=====================================================================
        // TStaticRectangle
        //---------------------------------------------------------------------
        //! Generic shape class used by Static matrixs/vectors & 1D/2D Expressions
        //! @param RowValue      Number of rows
        //! @param ColValue      Number of columns
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_shapepromotion.hpp $
        //=====================================================================
        template <vfc::int32_t RowValue, vfc::int32_t ColValue>
        struct TStaticRectangle
        {
            enum { NB_ROWS = RowValue};
            enum { NB_COLUMNS = ColValue};
        };

        //=====================================================================
        // CDynamicRectangle
        //---------------------------------------------------------------------
        //! Generic shape class used by dynamic matrixs/vectors & 1D/2D Expressions
        //! @param void
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_shapepromotion.hpp $
        //=====================================================================
        struct CDynamicRectangle
        {
            enum { NB_ROWS = 0};
            enum { NB_COLUMNS = 0};
        };

        //=====================================================================
        // TBinaryShapePromotion
        //---------------------------------------------------------------------
        //! Class used by operators that require both the operands to have same Dimensions
        //! @param Shape1Type     Data type
        //! @param Shape2Type     Data type
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_shapepromotion.hpp $
        //=====================================================================
        template <class Shape1Type, class Shape2Type>
        struct TBinaryShapePromotion
        {
            typedef CDynamicRectangle type;
        };

        //=====================================================================
        // TBinaryShapePromotion
        //---------------------------------------------------------------------
        //! Specialization for static matrixs/vectors & 1D/2D Expressions
        //! used by operators that require both the operands to have same Dimensions
        //! @param Row1Value      Number of rows
        //! @param Col1Value      Number of columns
        //! @param Row2Value      Number of rows
        //! @param Col2Value      Number of columns
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_shapepromotion.hpp $
        //=====================================================================
        template <vfc::int32_t Row1Value, vfc::int32_t Col1Value,
            vfc::int32_t Row2Value, vfc::int32_t Col2Value>
        struct TBinaryShapePromotion<TStaticRectangle<Row1Value, Col1Value>,
            TStaticRectangle<Row2Value, Col2Value> >
        {
            VFC_STATIC_ASSERT((Row1Value == Row2Value) && (Col1Value == Col2Value));
            typedef TStaticRectangle<Row1Value, Col1Value> type;
        };

        //=====================================================================
        // TTransposeMatrixShapePromotion
        //---------------------------------------------------------------------
        //! Used by transpose functions
        //! interchanges row number and column numbers of a matrix
        //! @param ShapeType     Data type
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_shapepromotion.hpp $
        //=====================================================================
        template<class ShapeType>
        struct TTransposeMatrixShapePromotion
        {
            typedef vfc::linalg::CDynamicRectangle shape_type;
        };

        //=====================================================================
        // TTransposeMatrixShapePromotion
        //---------------------------------------------------------------------
        //! Static specialization used for transpose functions
        //! interchanges row number and column numbers of a matrix
        //! @param RowValue      Number of rows
        //! @param ColValue      Number of columns
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_shapepromotion.hpp $
        //=====================================================================
        template<vfc::int32_t RowValue, vfc::int32_t ColValue>
        struct TTransposeMatrixShapePromotion<vfc::linalg::TStaticRectangle<RowValue, ColValue> >
        {
            typedef vfc::linalg::TStaticRectangle<ColValue, RowValue> shape_type;
        };

        //=====================================================================
        // TMulShapePromotion
        //---------------------------------------------------------------------
        //! Used while multiplying 2D Expressions
        //! @param Shape1Type      Data type
        //! @param Shape2Type      Data type
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_shapepromotion.hpp $
        //=====================================================================
        template <class Shape1Type, class Shape2Type>
        struct TMulShapePromotion
        {
            typedef CDynamicRectangle type;
        };

        //=====================================================================
        // TMulShapePromotion
        //---------------------------------------------------------------------
        //! Used while multiplying 2D Expressions
        //! For static operands
        //! @param Row1Value      Number of rows
        //! @param Col1Value      Number of columns
        //! @param Row2Value      Number of rows
        //! @param Col2Value      Number of columns
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_shapepromotion.hpp $
        //=====================================================================
        template <vfc::int32_t Row1Value, vfc::int32_t Col1Value, vfc::int32_t Row2Value, vfc::int32_t Col2Value>
        struct TMulShapePromotion<TStaticRectangle<Row1Value, Col1Value>,
            TStaticRectangle<Row2Value, Col2Value> >
        {
            VFC_STATIC_ASSERT(Col1Value == Row2Value);
            typedef TStaticRectangle<Row1Value, Col2Value> type;
        };

        //=====================================================================
        // TVectorDyadProductShapePromotion
        //---------------------------------------------------------------------
        //! Used by dyad function
        //! Sets the resultant Matrix dimension based on input vectors
        //! @param Shape1Type      Data type
        //! @param Shape2Type      Data type
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_shapepromotion.hpp $
        //=====================================================================
        template <class Shape1Type, class Shape2Type>
        struct TVectorDyadProductShapePromotion
        {
            typedef CDynamicRectangle type;
        };

        //=====================================================================
        // TVectorDyadProductShapePromotion
        //---------------------------------------------------------------------
        //! Used by dyad function (Template Specialization)
        //! Sets the resultant Matrix dimension based on input vectors
        //! @param Row1Value      No of rows
        //! @param Row2Value      No of rows
        //! @author  dkn2kor
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_shapepromotion.hpp $
        //=====================================================================
        template <vfc::int32_t Row1Value, vfc::int32_t Row2Value>
        struct TVectorDyadProductShapePromotion<TStaticRectangle<Row1Value, 1>,
            TStaticRectangle<Row2Value, 1> >
        {

            typedef TStaticRectangle<Row1Value, Row2Value> type;
        };

        //=====================================================================
        // TExp2D1DMulShapePromotion
        //---------------------------------------------------------------------
        //! Shape promotion while performing 2D * 1D
        //! @param Shape1Type      Data type
        //! @param Shape2Type      Data type
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_shapepromotion.hpp $
        //=====================================================================
        template <class Shape1Type, class Shape2Type>
        struct TExp2D1DMulShapePromotion
        {
            typedef CDynamicRectangle type;
        };

        //=====================================================================
        // TExp2D1DMulShapePromotion
        //---------------------------------------------------------------------
        //! TExp2D1DMulShapePromotion class specialization
        //! @param Row1Value      Number of rows
        //! @param Col1Value      Number of columns
        //! @param Row2Value      Number of rows
        //! @author  jat2hi
        //! @ingroup vfc_group_linalg
        //! $Source: vfc_linalg_shapepromotion.hpp $
        //=====================================================================
        template <vfc::int32_t Row1Value, vfc::int32_t Col1Value,
            vfc::int32_t Row2Value>
        struct TExp2D1DMulShapePromotion<TStaticRectangle<Row1Value, Col1Value>,
            TStaticRectangle<Row2Value, 1> >
        {
            VFC_STATIC_ASSERT((Col1Value == Row2Value) );
            typedef TStaticRectangle<Row1Value, 1> type;
        };

    }   // namespace linalg closed

}   // namespace vfc closed


#endif //VFC_LINALG_SHAPEPROMOTION_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_shapepromotion.hpp  $
//  Revision 1.3 2008/08/12 12:23:13MESZ vmr1kor 
//  - missing function template ( TVectorDyadProductShapePromotion) added . (mantis :- 2275)
//  Revision 1.2 2008/07/31 14:08:45IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor) 
//  doxy comments added ( Mantis :- 1744 )
//  Revision 1.1 2007/05/09 13:51:25IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
