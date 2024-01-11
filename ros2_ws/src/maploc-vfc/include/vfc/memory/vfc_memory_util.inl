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
//       Projectname: vfc/memory
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
///     $Source: vfc_memory_util.inl $
///     $Revision: 1.2 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (ZVH2HI) $
///     $Date: 2006/10/23 13:56:29MESZ $
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

//! calls delete on specified pointer and sets pointer to '0'
template <class ValueType>  inline
void    vfc::deleteAndNull (ValueType*& f_ptr_p)
{
    delete f_ptr_p;
    f_ptr_p = 0;
}

//! calls delete [] (array delete) on specified pointer and sets pointer to '0'
template <class ValueType>  inline
void    vfc::deleteArrayAndNull (ValueType*& f_ptr_p)
{
    delete [] f_ptr_p;
    f_ptr_p = 0;
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_memory_util.inl  $
//  Revision 1.2 2006/10/23 13:56:29MESZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - renamed functions (mantis1211)
//  - replaced header/footer templates
//=============================================================================
