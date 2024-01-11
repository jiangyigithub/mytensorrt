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
*     $Source: vfc_bitalgo.inl $
*     $Revision: 1.10 $
*     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
*     $Date: 2012/12/18 08:27:31MEZ $
*     $Locker:  $
*     $Name:  $
*     $State: in_work $
*/
/*******************************************************************/

#ifdef VFC_COMPILER_MWERKS
#    include <algorithm>
#endif

#include "vfc/core/vfc_assert.hpp"

inline
vfc::uint32_t vfc::countLeadingZeros(register vfc::uint32_t f_x_u32)
{
#ifdef VFC_COMPILER_MWERKS
// PRQA S 60, 4101, 4200 ++
#    ifdef VFC_PLATFORM_EPPC

    register uint32_t zcnt_u32;
    asm
    {
        //cntlzw (Count Leading Zeros Word) Instruction.
        //Places the number of leading zeros from a source general-purpose register
        //in a general-purpose register.
        cntlzw zcnt_u32, f_x_u32
    }
    return zcnt_u32;
#    else
    // this is inlines on X86, why not on PPC???
    return Metrowerks::count_leading_zero(f_x_u32);
#    endif
// PRQA S 60, 4101, 4200 --
#elif defined(VFC_COMPILER_ARMRVCT) && defined(VFC_ARM_DETECTED)
    vfc::uint32_t l_retVal_u32;
    __asm
    {
        CLZ l_retVal_u32, f_x_u32
    }
    return l_retVal_u32;
#else
    // ansi c solution
    // algo taken from "Hacker's Delight" chapter 5-3
    f_x_u32 |= (f_x_u32 >> 1);
    f_x_u32 |= (f_x_u32 >> 2);
    f_x_u32 |= (f_x_u32 >> 4);
    f_x_u32 |= (f_x_u32 >> 8);
    f_x_u32 |= (f_x_u32 >> 16);
    return countOnes(~f_x_u32);
#endif
}

inline
vfc::uint32_t vfc::countOnes (register uint32_t f_x_u32)
{
    // ansi c solution
    // algo taken from "Hacker's Delight" chapter 5-1
    f_x_u32 -= ((f_x_u32 >> 1) & 0x55555555);
    f_x_u32 = (((f_x_u32 >> 2) & 0x33333333) + (f_x_u32 & 0x33333333));
    f_x_u32 = (((f_x_u32 >> 4) + f_x_u32) & 0x0f0f0f0f);
    f_x_u32 += (f_x_u32 >> 8);
    f_x_u32 += (f_x_u32 >> 16);
    return(f_x_u32 & 0x0000003f);
}

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_bitalgo.inl  $
Revision 1.10 2012/12/18 08:27:31MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
- Replace tabs by 4 spaces (mantis 4199)
Revision 1.9 2011/05/31 12:00:45MESZ Muehlmann Karsten (CC/PJ-FA1) (MUK2LR) 
- added ARM optimized version of count_leading_zeros (mantis3249)
Revision 1.8 2009/05/28 10:54:27CEST Gaurav Jain (RBEI/EAC1) (gaj2kor) 
-Removal of initialisation with zero. (mantis : 2495)
Revision 1.7 2009/02/03 10:36:49IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
-Removal of QAC++ warnings.
(Mantis : 0002495)
Revision 1.6 2006/11/16 19:11:06IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
- replaced tabs with 4 spaces (mantis1294)
Revision 1.5 2006/10/09 09:52:04CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
- changes to use VFC_COMPILER/PLATFORM_XXX instead of platform/compiler specific defines (mantis 1166)
Revision 1.4 2006/10/06 13:15:14CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
- changed VFC_EPPC define to VFC_EPPC_DETECTED (mantis 1149)
Revision 1.3 2005/11/02 14:58:00CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
-added ansi c fallback for countLeadingZeros()
-added ansi c countOnes()
Revision 1.2 2005/11/02 09:41:30CET Muehlmann Karsten (AE-DA/ESA3) * (MUK2LR)
include <algorithm> whenn needed to find Metrowerks::count_leading_zero()
Revision 1.1 2005/10/28 11:00:38CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/core.pj
********************************************************************/
