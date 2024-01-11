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
*   Projectname: vfc
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
*     $Source: vfc_win32.hpp $
*     $Revision: 1.4 $
*     $Author: Jaeger Thomas (CC-DA/ENV1) (jat2hi) $
*     $Date: 2006/10/06 13:14:31MESZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: InProcess $
*/
/*******************************************************************/

#ifndef VFC_WIN32_HPP_INCLUDED
#define VFC_WIN32_HPP_INCLUDED

////////////////////////////////////
// win32 specific config options
////////////////////////////////////

#define VFC_PLATFORM_STRING "win32"
#define VFC_WINDOWS 1

#ifndef VFC_PLATFORM_WIN32
#   define VFC_PLATFORM_WIN32
#endif


// add more config code here ...

#endif //VFC_WIN32_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_win32.hpp  $
Revision 1.4 2006/10/06 13:14:31MESZ Jaeger Thomas (CC-DA/ENV1) (jat2hi) 
- changed VFC_PLATFORM define to VFC_PLATFORM_STRING
- added define VFC_PLATFORM_WIN32
(mantis 1149)
Revision 1.3 2005/10/28 10:17:29CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/config/platform/platform.pj
Revision 1.2 2005/10/06 17:00:34CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/config/platform/platform.pj
********************************************************************/
