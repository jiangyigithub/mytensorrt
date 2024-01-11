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
///     $Source: vfc_algorithm_fixed.hpp $
///     $Revision: 1.5 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/07/23 09:24:17MESZ $
///     $Locker:  $
///     $Name: 0032 RC1 Hello KW07  $
///     $State: defective $
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

#ifndef VFC_ALGORITHM_FIXED_HPP_INCLUDED
#define VFC_ALGORITHM_FIXED_HPP_INCLUDED

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
    // original STL algorithms with metaprogramming loop-unrolling
    //=============================================================================

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of elements from 
    /// a range in a forward order. 
    //-------------------------------------------------------------------------

    template <  int32_t CountValue, 
                class IOItType, 
                class FuncType>

    FuncType    for_each_fixed_n (  IOItType f_inIt, 
                                    FuncType f_func);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of pairs from two 
    /// ranges in a forward order.
    //-------------------------------------------------------------------------
    template <  int32_t CountValue, 
                class IOIt1Type, class IOIt2Type, 
                class FuncType>

    FuncType    for_each_fixed_n (  IOIt1Type f_ioIt1, 
                                    IOIt2Type f_ioIt2, 
                                    FuncType f_func);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of triples from 
    /// three ranges in a forward order.
    //-------------------------------------------------------------------------
    template <  int32_t CountValue, 
                class IOIt1Type, class IOIt2Type, class IOIt3Type, 
                class FuncType>

    FuncType    for_each_fixed_n (  IOIt1Type f_ioIt1, 
                                    IOIt2Type f_ioIt2, 
                                    IOIt3Type f_ioIt3, 
                                    FuncType f_func);

    //-------------------------------------------------------------------------
    /// Assigns the values generated by a function object to a specified number of 
    /// element in a range.
    //-------------------------------------------------------------------------

    template <  int32_t CountValue, 
                class FwItType, 
                class FuncType>

    void        generate_fixed_n (  FwItType f_fwIt, 
                                    FuncType f_func);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of element in a range
    /// and copies the return values of the function object into a destination range.
    //-------------------------------------------------------------------------
    template <  int32_t CountValue, 
                class InItType, class OutItType, 
                class FuncType>

    OutItType   transform_fixed_n ( InItType f_inIt, 
                                    OutItType f_destIt, 
                                    FuncType f_func);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of pairs from two 
    /// source ranges and copies the return values of the function object into a destination range.
    //-------------------------------------------------------------------------
    template <  int32_t CountValue, 
                class InIt1Type, class InIt2Type, class OutItType, 
                class FuncType>

    OutItType   transform_fixed_n ( InIt1Type f_inIt1, 
                                    InIt2Type f_inIt2, 
                                    OutItType f_destIt, 
                                    FuncType f_func);

    //=============================================================================
    // modified STL for 1D random-access iterators (op[](n) square brackets)
    //=============================================================================

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of elements from 
    /// a range in a forward order.
    //-------------------------------------------------------------------------

    template <  int32_t CountValue, 
                class InArgType, 
                class FuncType>

    FuncType    for_each_square_fixed_n (   InArgType& f_inArg, 
                                            FuncType f_func);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of triples from three 
    /// ranges in a forward order.
    //-------------------------------------------------------------------------
    template <  int32_t CountValue, 
                class IOArg1Type, class IOArg2Type, 
                class FuncType>

    FuncType    for_each_square_fixed_n (   IOArg1Type& f_ioArg1, 
                                            IOArg2Type& f_ioArg2, 
                                            FuncType f_func);

    //-------------------------------------------------------------------------
    /// Applies a specified function object to each element in a forward order to a 
    /// specified number of element in a range. 
    //-------------------------------------------------------------------------
    template <  int32_t CountValue, 
                class IOArg1Type, class IOArg2Type, class IOArg3Type, 
                class FuncType>

    FuncType    for_each_square_fixed_n (   IOArg1Type& f_ioArg1, 
                                            IOArg2Type& f_ioArg2, 
                                            IOArg3Type& f_ioArg3, 
                                            FuncType f_func);

    //-------------------------------------------------------------------------
    /// Assigns the values generated by a function object to a specified number 
    /// of element in a range.
    //-------------------------------------------------------------------------
    template <  int32_t CountValue, 
                class OutArgType, 
                class FuncType>

    void        generate_square_fixed_n (   OutArgType& f_outArg, 
                                            FuncType f_func);
    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of element in a 
    /// range and copies the return values of the function object into a destination range.
    //-------------------------------------------------------------------------
    template <  int32_t CountValue, 
                class InArgType, class OutArgType, 
                class FuncType>

    void        transform_square_fixed_n (  const InArgType& f_inArg, 
                                            OutArgType& f_destArg, 
                                            FuncType f_func);
    
    //-------------------------------------------------------------------------
    /// Applies a specified function object to a specified number of pairs from two source 
    /// ranges and copies the return values of the function object into a destination range.
    //-------------------------------------------------------------------------
    template <  int32_t CountValue, 
                class InArg1Type, class InArg2Type, class OutArgType, 
                class FuncType>

    void        transform_square_fixed_n (  const InArg1Type& f_inArg1, 
                                            const InArg2Type& f_inArg2, 
                                            OutArgType& f_destArg, 
                                            FuncType f_func);

    //=========================================================================
    //  DOYGEN ADDTOGROUP vfc_group_core_algorithms_stl END
    //-------------------------------------------------------------------------
    /// @}
    //-------------------------------------------------------------------------

}   // namespace vfc closed

#include "vfc/core/vfc_algorithm_fixed.inl"

#endif //VFC_ALGORITHM_FIXED_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_algorithm_fixed.hpp  $
//  Revision 1.5 2007/07/23 09:24:17MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.4 2007/06/22 15:12:24CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - corrected redeclared function parameter names (mantis 1691)
//  Revision 1.3 2006/11/16 14:28:58CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced size_t with int32_t (mantis1305)
//  - replaced tabs with spaces (mantis1294)
//  Revision 1.2 2006/11/06 11:17:58CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - cosmetics
//=============================================================================
