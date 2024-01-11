/********************************************************************
* C O P Y R I G H T
*--------------------------------------------------------------------
* Copyright (c) 2005 by Robert Bosch GmbH. All rights reserved.
*
* This file is property of Robert Bosch GmbH. Any unauthorised copy,
* use or distribution is an offensive act against international law
* and may me prosecuted under federal law. Its content is company
* confidential.
*--------------------------------------------------------------------
* D E S C R I P T I O N
*--------------------------------------------------------------------
*   Projectname: ..\..\..\..\vfc\dummy\win32\vc_71\vfc_dummy
*      Synopsis: 
* Target system: 
*      Compiler: VS7.1
*--------------------------------------------------------------------
* N O T E S
*--------------------------------------------------------------------
* Notes: 
*--------------------------------------------------------------------
* I N I T I A L   A U T H O R   I D E N T I T Y
*--------------------------------------------------------------------
*       Name: 
* Department: 
*--------------------------------------------------------------------
* R E V I S I O N   I N F O R M A T I O N
*------------------------------------------------------------------*/
/*!   \file
*     \par Revision History
*     $Source: vfc_bitalgo.hpp $
*     $Revision: 1.4 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
*     $Date: 2007/07/23 09:28:46MESZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

#ifndef VFC_BITALGO_HPP_INCLUDED
#define VFC_BITALGO_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

namespace vfc
{    // namespace vfc opened
    
    //=========================================================================
    //  DOYGEN DEFGROUP vfc_group_core_algorithms_bitalgos BEGIN
    //-------------------------------------------------------------------------
    /// @defgroup vfc_group_core_algorithms_bitalgos Bit-level Algos
    /// @ingroup vfc_group_core_algorithms
    /// @brief bit-level algorithms.
    /// @{
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    //! returns the number of leading 0-bits in given 32bit value
    //-------------------------------------------------------------------------
    uint32_t countLeadingZeros(register uint32_t f_x_u32);

    //-------------------------------------------------------------------------
    //! returns the number of 1-bits in given 32bit value (population count)
    //-------------------------------------------------------------------------
    uint32_t countOnes (register uint32_t f_x_u32);

    //=========================================================================
    //  DOYGEN DEFGROUP vfc_group_core_algorithms_bitalgos END
    //-------------------------------------------------------------------------
    /// @}
    //-------------------------------------------------------------------------

}    // namespace vfc closed

#include "vfc/core/vfc_bitalgo.inl"

#endif //VFC_BITALGO_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_bitalgo.hpp  $
Revision 1.4 2007/07/23 09:28:46MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
- doxygen grouping (mantis1744)
Revision 1.3 2006/11/16 14:41:17CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.2 2005/11/02 14:57:59CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
-added ansi c fallback for countLeadingZeros()
-added ansi c countOnes()
********************************************************************/
