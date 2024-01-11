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
*     $Source: vfc_preprocessor.hpp $
*     $Revision: 1.5 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
*     $Date: 2007/07/18 16:34:37MESZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

#ifndef VFC_PREPROCESSOR_HPP_INCLUDED
#define VFC_PREPROCESSOR_HPP_INCLUDED

//=============================================================================
//  VFC_JOIN
//-----------------------------------------------------------------------------
/// helper macro, joins two macro arguments together.
/// The following piece of macro magic joins the two 
/// arguments together, even when one of the arguments is
/// itself a macro (see 16.3.1 in C++ standard).  The key
/// is that macro expansion of macro arguments does not
/// occur in VFC_DO_JOIN2 but does in VFC_DO_JOIN.
/// @sa VFC_DO_JOIN
/// @sa VFC_DO_JOIN2
/// @ingroup vfc_group_core_misc
//=============================================================================
#define VFC_JOIN(ARG1,ARG2) VFC_DO_JOIN(ARG1,ARG2)
//! @sa VFC_JOIN
#define VFC_DO_JOIN(ARG1,ARG2) VFC_DO_JOIN2(ARG1,ARG2)
//! @sa VFC_JOIN
#define VFC_DO_JOIN2(ARG1,ARG2) ARG1##ARG2

//=============================================================================
//  VFC_STRINGIZE
//-----------------------------------------------------------------------------
/// helper macro, converts the parameter X to a string after macro replacement
/// on X has been performed.
/// @sa VFC_DO_STRINGIZE
/// @ingroup vfc_group_core_misc
//=============================================================================
#define VFC_STRINGIZE(ARG) VFC_DO_STRINGIZE(ARG)
//! @sa VFC_STRINGIZE
#define VFC_DO_STRINGIZE(ARG) #ARG

#endif //VFC_PREPROCESSOR_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_preprocessor.hpp  $
Revision 1.5 2007/07/18 16:34:37MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
- doxygen documentation grouping (mantis1744)
Revision 1.4 2006/11/16 14:41:18CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.2 2005/10/06 16:53:32CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/vfc.pj
********************************************************************/
