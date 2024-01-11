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
//       Projectname: vfc/geo
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
//        Name: Christian Renner (rec1lr)
//  Department: AE-DA/ESA3
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_vec4_ops.inl $
///     $Revision: 1.6 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (ZVH2HI) $
///     $Date: 2006/11/16 14:41:18MEZ $
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

template <class T>    inline
vfc::TVector4<T>    vfc::operator-    (const TVector4<T>& f_a_vec4, const TVector4<T>& f_b_vec4)
{
    return TVector4<T>    (    f_a_vec4.x()-f_b_vec4.x(),
                            f_a_vec4.y()-f_b_vec4.y(),
                            f_a_vec4.z()-f_b_vec4.z(),
                            f_a_vec4.w()-f_b_vec4.w()
                        );
}


template <class T>    inline
vfc::TVector4<T>    vfc::operator+    (const TVector4<T>& f_a_vec4, const TVector4<T>& f_b_vec4)
{
    return TVector4<T>    (    f_a_vec4.x()+f_b_vec4.x(),
                            f_a_vec4.y()+f_b_vec4.y(),
                            f_a_vec4.z()+f_b_vec4.z(),
                            f_a_vec4.w()+f_b_vec4.w()
                        );
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_vec4_ops.inl  $
//  Revision 1.6 2006/11/16 14:41:18MEZ Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.5 2006/11/06 11:29:13CET Jaeger Thomas (AE-DA/ESA3) (jat2hi) 
//  - added empty last line (mantis 1258)
//  Revision 1.4 2006/10/30 08:34:52CET Renner Christian (AE-DA/ESA3) * (rec1lr) 
//  (mantis ID 1221)
//  deleted unnessesary unary-() operator
//  Revision 1.3 2006/10/20 10:20:12CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced header/footer templates
//  - removed unnecessary includes
//  Revision 1.2 2006/10/19 11:11:21CEST Renner Christian (AE-DA/ESA3) * (rec1lr) 
//  - deleted unneeded include
//  Revision 1.1 2006/10/18 16:00:51CEST Renner Christian (AE-DA/ESA3) * (rec1lr) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-projects/ppj_ivs_ldw/IntegrationTestWin/vfc/geo/geo.pj
//=============================================================================
