//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2006 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy, use or 
//  distribution is an offensive act against international law and may be 
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: dummy_test
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
//        Name: zvh2hi
//  Department: 
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_algorithm2d.hpp $
///     $Revision: 1.3 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/07/23 09:27:05MESZ $
///     $Locker:  $
///     $Name: 0032 RC1 Hello KW07  $
///     $State: In_Development $
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

#ifndef VFC_ALGORITHM2D_HPP_INCLUDED
#define VFC_ALGORITHM2D_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

namespace vfc
{   // namespace vfc opened

    //=========================================================================
    //  DOYGEN ADDTOGROUP vfc_group_core_algorithms_stl BEGIN
    //-------------------------------------------------------------------------
    /// @addtogroup vfc_group_core_algorithms_stl STL Enhancements
    /// @ingroup vfc_group_core_algorithms
    /// @brief STL enhancements.
    /// @{
    //-------------------------------------------------------------------------

    //=============================================================================
    // modified STL for 2D random-access iterators (it[n1][n2] double square brackets)
    //=============================================================================
    
    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of elements from 
    /// a 2D range in a forward order.
    //-------------------------------------------------------------------------
    template <class InArgType, class FuncType, class CountType>
    FuncType    for_each_2d_square_n (InArgType& f_inArg,
                                      FuncType   f_func,
                                      CountType  f_count1, 
                                      CountType  f_count2);
    
    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of elements from 
    /// a 2D range in a forward order.
    //-------------------------------------------------------------------------
    template <class IOArg1Type, class IOArg2Type, class FuncType, class CountType>
    FuncType    for_each_2d_square_n (IOArg1Type& f_ioArg1,
                                      IOArg2Type& f_ioArg2,
                                      FuncType    f_func,
                                      CountType   f_count1,
                                      CountType   f_count2);
    
    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of elements from
    /// a 2D in a forward order.
    //-------------------------------------------------------------------------
    template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType, 
        class CountType>
    FuncType    for_each_2d_square_n (IOArg1Type& f_ioArg1,
                                      IOArg2Type& f_ioArg2,
                                      IOArg3Type& f_ioArg3,
                                      FuncType    f_func,
                                      CountType   f_count1,
                                      CountType   f_count2);

    //-------------------------------------------------------------------------
    /// Assigns the values generated by a function object to a specified number
    /// of elements in a 2D range.
    //-------------------------------------------------------------------------
    template <class FwArgType, class FuncType, class CountType>
    void        generate_2d_square_n (FwArgType& f_fwArg,
                                      FuncType   f_func,
                                      CountType  f_count1,
                                      CountType  f_count2);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of element
    /// in a 2D range and copies the return values of the function object into a destination
    /// range.
    //-------------------------------------------------------------------------
    template <class InArgType, class OutArgType, class FuncType, class CountType>
    void        transform_2d_square_n (const InArgType& f_inArg,
                                       OutArgType& f_destArg,
                                       FuncType    f_func,
                                       CountType   f_count1,
                                       CountType   f_count2);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of element
    /// in a 2D range and copies the return values of the function object into a
    /// destination range.
    //-------------------------------------------------------------------------
    template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType, 
        class CountType>
    void        transform_2d_square_n (const InArg1Type& f_inArg1,
                                       const InArg2Type& f_inArg2,
                                       OutArgType&       f_destArg,
                                       FuncType          f_func,
                                       CountType         f_count1,
                                       CountType         f_count2);

    //=============================================================================
    // modified STL for 2D random-access iterators (op()(n1,n2) round brackets)
    //=============================================================================

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of elements from a
    /// 2D range in a forward order.
    //-------------------------------------------------------------------------
    template <class InArgType, class FuncType, class CountType>
    FuncType    for_each_2d_round_n (InArgType& f_inArg,
                                     FuncType   f_func,
                                     CountType  f_count1,
                                     CountType  f_count2);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of elements from
    /// a 2D range in a forward order.
    //-------------------------------------------------------------------------
    template <class IOArg1Type, class IOArg2Type, class FuncType, class CountType>
    FuncType    for_each_2d_round_n (IOArg1Type& f_ioArg1,
                                     IOArg2Type& f_ioArg2,
                                     FuncType    f_func,
                                     CountType   f_count1,
                                     CountType   f_count2);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of elements from
    /// a 2D range in a forward order.
    //-------------------------------------------------------------------------
    template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType, 
        class CountType>
    FuncType    for_each_2d_round_n (IOArg1Type& f_ioArg1,
                                     IOArg2Type& f_ioArg2,
                                     IOArg3Type& f_ioArg3,
                                     FuncType    f_func,
                                     CountType   f_count1,
                                     CountType   f_count2);

    //-------------------------------------------------------------------------
    /// Assigns the values generated by a function object to a specified number
    /// of elements in a 2D range.
    //-------------------------------------------------------------------------
    template <class FwArgType, class FuncType, class CountType>
    void        generate_2d_round_n (FwArgType& f_fwArg,
                                     FuncType   f_func,
                                     CountType  f_count1,
                                     CountType  f_count2);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of element
    /// in a 2D range and copies the return values of the function object into a destination range.
    //-------------------------------------------------------------------------
    template <class InArgType, class OutArgType, class FuncType, class CountType>
    void        transform_2d_round_n (const InArgType&  f_inArg,
                                      OutArgType&       f_destArg,
                                      FuncType          f_func,
                                      CountType         f_count1,
                                      CountType         f_count2);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of element
    /// in a 2D range and copies the return values of the function object into a destination range.
    //-------------------------------------------------------------------------
    template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType, 
        class CountType>
    void        transform_2d_round_n (const InArg1Type& f_inArg1,
                                      const InArg2Type& f_inArg2,
                                      OutArgType&       f_destArg,
                                      FuncType          f_func,
                                      CountType         f_count1,
                                      CountType         f_count2);

    //=========================================================================
    //  DOYGEN ADDTOGROUP vfc_group_core_algorithms_stl END
    //-------------------------------------------------------------------------
    /// @}
    //-------------------------------------------------------------------------

}   // namespace vfc closed

#include "vfc/core/vfc_algorithm2d.inl"

#endif //VFC_ALGORITHM2D_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_algorithm2d.hpp  $
//  Revision 1.3 2007/07/23 09:27:05MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.2 2007/06/11 10:53:46CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - corrected include path (mantis 1670)
//  Revision 1.1 2007/02/23 13:36:10CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/core.pj
//  Revision 1.3 2007/01/18 08:52:59CET dkn2kor 
//  - Replaced size_t with a template parameter CountType (mantis1252)
//  Revision 1.2 2006/12/12 20:09:03IST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - wrapped long lines
//  Revision 1.1 2006/11/16 16:46:13CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/algorithm_looped/algorithm_looped.pj
//  Revision 1.2 2006/11/16 15:55:35CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - changed the count parameter types from size_t to signed int
//  Revision 1.1 2006/10/19 16:01:45CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/algorithm_looped/algorithm_looped.pj
//  Revision 1.2 2006/10/06 09:34:37CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - made the functions STL conform
//  - removed one indirection
//  Revision 1.1 2006/10/05 09:20:39CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/algorithm_looped/algorithm_looped.pj
//  Revision 1.3 2006/09/29 14:18:46CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - corrected #include statement
//  Revision 1.2 2006/09/29 13:07:35CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added the square loops
//  - renamed "Unroll" to "Loop"
//  Revision 1.1 2006/09/29 12:48:23CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/internal-projects/ipj_ivs_AlgoDevelop/ipj_ivs_LinAlgBenchmark/matrixeval2/extern/uwnfu/uwnfu.pj
//  Revision 1.3 2006/09/28 18:06:42CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -changed comments
//  Revision 1.2 2006/09/28 18:02:25CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -fixed and added docu
//=============================================================================
