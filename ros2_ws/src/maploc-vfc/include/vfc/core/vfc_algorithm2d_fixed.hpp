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
//       Projectname: vfc/core
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
//  Department: CR/AEM
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_algorithm2d_fixed.hpp $
///     $Revision: 1.4 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/07/23 09:26:25MESZ $
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

#ifndef VFC_ALGORITHM2D_FIXED_HPP_INCLUDED
#define VFC_ALGORITHM2D_FIXED_HPP_INCLUDED

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
    template <  int32_t Count1Value, int32_t Count2Value, 
                class InArgType, 
                class FuncType>
    
    FuncType    for_each_2d_square_fixed_n (    InArgType& f_inArg, 
                                                FuncType f_func);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of elements from 
    /// a 2D range in a forward order.
    //-------------------------------------------------------------------------
    template <  int32_t Count1Value, int32_t Count2Value, 
                class IOArg1Type, class IOArg2Type, 
                class FuncType>
    
    FuncType    for_each_2d_square_fixed_n (    IOArg1Type& f_ioArg1, 
                                                IOArg2Type& f_ioArg2, 
                                                FuncType f_func);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of elements from 
    /// a 2D in a forward order.
    //-------------------------------------------------------------------------
    template <  int32_t Count1Value, int32_t Count2Value, 
                class IOArg1Type, class IOArg2Type, class IOArg3Type, 
                class FuncType>
    
    FuncType    for_each_2d_square_fixed_n (    IOArg1Type& f_ioArg1, 
                                                IOArg2Type& f_ioArg2, 
                                                IOArg3Type& f_ioArg3, 
                                                FuncType f_func);

    //-------------------------------------------------------------------------
    /// Assigns the values generated by a function object to a specified number 
    /// of elements in a 2D range.
    //-------------------------------------------------------------------------
    template <  int32_t Count1Value, int32_t Count2Value, 
                class FwArgType, 
                class FuncType>
    
    void        generate_2d_square_fixed_n (    FwArgType& f_fwArg, 
                                                FuncType f_func);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of element in 
    /// a 2D range and copies the return values of the function object into a 
    /// destination range.
    //-------------------------------------------------------------------------
    template <  int32_t Count1Value, int32_t Count2Value, 
                class InArgType, class OutArgType, class FuncType>
    
    void        transform_2d_square_fixed_n (   const InArgType& f_inArg, 
                                                OutArgType& f_destArg, 
                                                FuncType f_func);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of element in a 
    /// 2D range and copies the return values of the function object into a 
    /// destination range.
    //-------------------------------------------------------------------------
    template <  int32_t Count1Value, int32_t Count2Value, 
                class InArg1Type, class InArg2Type, class OutArgType, 
                class FuncType>

    void        transform_2d_square_fixed_n (   const InArg1Type& f_inArg1, 
                                                const InArg2Type& f_inArg2, 
                                                OutArgType& f_destArg, 
                                                FuncType f_func);

    //=============================================================================
    // modified STL for 2D random-access iterators (op()(n1,n2) round brackets)
    //=============================================================================

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of elements from 
    /// a 2D range in a forward order.
    //-------------------------------------------------------------------------
    template <  int32_t Count1Value, int32_t Count2Value, 
                class InArgType, 
                class FuncType>

    FuncType    for_each_2d_round_fixed_n ( InArgType& f_inArg, 
                                            FuncType f_func);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of elements from 
    /// a 2D range in a forward order.
    //-------------------------------------------------------------------------
    template <  int32_t Count1Value, int32_t Count2Value, 
                class IOArg1Type, class IOArg2Type, 
                class FuncType>

    FuncType    for_each_2d_round_fixed_n ( IOArg1Type& f_ioArg1, 
                                            IOArg2Type& f_ioArg2, 
                                            FuncType f_func);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of elements from 
    /// a 2D range in a forward order.
    //-------------------------------------------------------------------------
    template <  int32_t Count1Value, int32_t Count2Value, 
                class IOArg1Type, class IOArg2Type, class IOArg3Type, 
                class FuncType>

    FuncType    for_each_2d_round_fixed_n ( IOArg1Type& f_ioArg1, 
                                            IOArg2Type& f_ioArg2, 
                                            IOArg3Type& f_ioArg3, 
                                            FuncType f_func);

    //-------------------------------------------------------------------------
    /// Assigns the values generated by a function object to a specified number of 
    /// elements in a 2D range.
    //-------------------------------------------------------------------------
    template <  int32_t Count1Value, int32_t Count2Value, 
                class FwArgType, 
                class FuncType>

    void        generate_2d_round_fixed_n ( FwArgType& f_fwArg, 
                                            FuncType f_func);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of element in a 
    /// 2D range and copies the return values of the function object into a 
    /// destination range.
    //-------------------------------------------------------------------------
    template <  int32_t Count1Value, int32_t Count2Value, 
                class InArgType, class OutArgType, 
                class FuncType>

    void        transform_2d_round_fixed_n (    const InArgType& f_inArg, 
                                                OutArgType& f_destArg, 
                                                FuncType f_func);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of element in a 
    /// 2D range and copies the return values of the function object into a 
    /// destination range.
    //-------------------------------------------------------------------------
    template <  int32_t Count1Value, int32_t Count2Value, 
                class InArg1Type, class InArg2Type, class OutArgType, 
                class FuncType>

    void        transform_2d_round_fixed_n (    const InArg1Type& f_inArg1, 
                                                const InArg2Type& f_inArg2, 
                                                OutArgType& f_destArg, 
                                                FuncType f_func);

    //=========================================================================
    //  DOYGEN ADDTOGROUP vfc_group_core_algorithms_stl END
    //-------------------------------------------------------------------------
    /// @}
    //-------------------------------------------------------------------------

}   // namespace vfc closed

#include "vfc/core/vfc_algorithm2d_fixed.inl"

#endif //VFC_ALGORITHM2D_FIXED_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_algorithm2d_fixed.hpp  $
//  Revision 1.4 2007/07/23 09:26:25MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.3 2007/01/09 12:24:26CET dkn2kor 
//  - Used signed type for count values, changed size_t to int32_t (mantis1376)
//  Revision 1.2 2006/11/06 15:48:53IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - cosmetics
//=============================================================================
