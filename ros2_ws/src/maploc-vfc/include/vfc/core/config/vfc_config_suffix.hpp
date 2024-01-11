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
*     $Source: vfc_config_suffix.hpp $
*     $Revision: 1.6 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (ZVH2HI) $
*     $Date: 2007/03/29 11:16:16MESZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

#ifndef VFC_CONFIG_SUFFIX_HPP_INCLUDED
#define VFC_CONFIG_SUFFIX_HPP_INCLUDED

#ifndef VFC_DECL_FORCEINLINE
#    define VFC_DECL_FORCEINLINE inline
#endif

#ifdef VFC_STL_ALIAS
    namespace VFC_STL_ALIAS {}
    namespace stlalias = VFC_STL_ALIAS ;
#else
    namespace std {}
    namespace stlalias = ::std; 
#endif

#endif //VFC_CONFIG_SUFFIX_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_config_suffix.hpp  $
Revision 1.6 2007/03/29 11:16:16MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- added namespace alias for stl (mantis1534)
Revision 1.5 2006/11/16 14:43:57CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.3 2005/10/06 16:58:43CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/config/config.pj
Revision 1.2 2005/04/06 16:42:41CEST zvh2hi 
added forceinline support
********************************************************************/
