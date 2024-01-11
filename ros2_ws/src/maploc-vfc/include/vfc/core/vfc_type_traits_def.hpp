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
*     $Source: vfc_type_traits_def.hpp $
*     $Revision: 1.5 $
*     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
*     $Date: 2007/07/23 09:35:10MESZ $
*     $Locker:  $
*     $Name: 0032 RC1 Hello KW07  $
*     $State: In_Development $
*/
/*******************************************************************/

#ifndef VFC_TYPE_TRAITS_DEF_HPP_INCLUDED
#define VFC_TYPE_TRAITS_DEF_HPP_INCLUDED

//=============================================================================
//  VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL
//-----------------------------------------------------------------------------
//! helper macro for defining a traits class with a default value.
//! See @ref vfc_group_core_generic_typetraits for more details.
//! @ingroup vfc_group_core_generic_typetraits
//=============================================================================

#define    VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL(TRAIT,VALUE)\
    template < typename T >\
    struct TRAIT\
    {    enum { value = (VALUE) };}

//=============================================================================
//  VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL
//-----------------------------------------------------------------------------
//! helper macro for defining a traits class full specialization.
//! See @ref vfc_group_core_generic_typetraits for more details.
//! @ingroup vfc_group_core_generic_typetraits
//=============================================================================

#define    VFC_TYPE_TRAITS_VALUE_SPECIAL_IMPL(TRAIT,TYPE,VALUE)\
    template <>\
    struct TRAIT < TYPE >\
    {    enum { value = (VALUE) };}

//=============================================================================
//  VFC_TYPE_TRAITS_VALUE_DEFAULT_IMPL
//-----------------------------------------------------------------------------
//! helper macro for defining a traits class partial specialization.
//! See @ref vfc_group_core_generic_typetraits for more details.
//! @ingroup vfc_group_core_generic_typetraits
//=============================================================================

#define    VFC_TYPE_TRAITS_VALUE_PART_SPECIAL_IMPL(TRAIT,TYPE,VALUE)\
    template < typename T >\
    struct TRAIT < TYPE >\
    {    enum { value = (VALUE) };}

#endif //VFC_TYPE_TRAITS_DEF_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_type_traits_def.hpp  $
Revision 1.5 2007/07/23 09:35:10MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
- doxygen grouping (mantis1744)
- added documentation
Revision 1.4 2006/11/16 14:41:20CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.2 2005/10/06 16:52:47CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/vfc.pj
Revision 1.1 2005/04/18 11:58:30CEST zvh2hi 
Initial revision
Member added to project /import/mks/data/projects/cv/vfc/include/vfc/vfc.pj
********************************************************************/
