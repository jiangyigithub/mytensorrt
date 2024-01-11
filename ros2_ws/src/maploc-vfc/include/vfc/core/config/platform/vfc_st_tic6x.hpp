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
*   Projectname: vfc/core/config/platform
*      Synopsis: 
* Target system: 
*      Compiler: 
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
*     $Source: vfc_st_tic6x.hpp $
*     $Revision: 1.1 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (ZVH2HI) $
*     $Date: 2008/08/11 17:38:10MESZ $
*     $Locker:  $
*     $Name: 0032 RC1  $
*     $State: in_work $
*/
/*******************************************************************/

#ifndef VFC_ST_TIC6X_HPP_INCLUDED
#define VFC_ST_TIC6X_HPP_INCLUDED

////////////////////////////////////
// ST Ti C6X specific config options
////////////////////////////////////

#define VFC_PLATFORM_STRING "ST-TiC6X"

#ifndef VFC_PLATFORM_ST_TIC6X
#   define VFC_PLATFORM_ST_TIC6X
#endif

// use vstl STL implementation
#undef VFC_STL_ALIAS
#define VFC_STL_ALIAS vstl

// disable vfc checks for non-debug targets
#ifndef _DEBUG
#   undef VFC_NDEBUG
#   define VFC_NDEBUG
#endif

// add more config code here ...

#endif //VFC_ST_TIC6X_HPP_INCLUDED


/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_st_tic6x.hpp  $
Revision 1.1 2008/08/11 17:38:10MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/config/platform/platform.pj
Revision 1.1 2008/07/17 14:21:30CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/mantis2067/mantis2067.pj
********************************************************************/


