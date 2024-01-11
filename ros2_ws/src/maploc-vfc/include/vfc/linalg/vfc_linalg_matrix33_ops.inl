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
//       Projectname: 
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
//        Name: dkn2kor
//  Department: 
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_linalg_matrix33_ops.inl $
///     $Revision: 1.1 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (jat2hi) $
///     $Date: 2007/05/09 10:21:21MESZ $
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

#include "vfc/core/vfc_static_assert.hpp"   // used for VFC_STATIC_ASSERT()
#include "vfc/core/vfc_math.hpp"            // used for isZero()
#include "vfc/core/vfc_assert.hpp"   // used for VFC_ASSERT2()

template <class T>    inline 
T    vfc::linalg::det    (const TMatrix33<T>& f_mat)
{    
    return        (    f_mat(2,0) * ( f_mat(0,1)*f_mat(1,2)-f_mat(1,1)*f_mat(0,2)) )
            -   (    f_mat(2,1) * ( f_mat(0,0)*f_mat(1,2)-f_mat(1,0)*f_mat(0,2)) )
            +   (    f_mat(2,2) * ( f_mat(0,0)*f_mat(1,1)-f_mat(1,0)*f_mat(0,1)) );
}

template <class T>    inline 
T    vfc::linalg::trace    (const TMatrix33<T>& f_mat)
{    
    return    ( f_mat(0,0) + f_mat(1,1) + f_mat(2,2) );
}

template <class T>    inline  
vfc::linalg::TMatrix33<T>    vfc::linalg::inverse (const TMatrix33<T>& f_mat)
{    
    // only support inverse for floating types
    VFC_STATIC_ASSERT(vfc::TIsFloating<T>::value);
    
    const    T   d    = det(f_mat);
    
    VFC_REQUIRE2(!isZero(d),"TMatrix33<T> is singular");
        
    const    T    oneOverDet = static_cast<T>(1)/d;
    
    return TMatrix33<T>    (    oneOverDet * ( f_mat(1,1)*f_mat(2,2) - f_mat(2,1)*f_mat(1,2) ),
                            oneOverDet * ( f_mat(2,1)*f_mat(0,2) - f_mat(2,2)*f_mat(0,1) ),
                            oneOverDet * ( f_mat(0,1)*f_mat(1,2) - f_mat(1,1)*f_mat(0,2) ),
                            oneOverDet * ( f_mat(2,0)*f_mat(1,2) - f_mat(1,0)*f_mat(2,2) ),
                            oneOverDet * ( f_mat(0,0)*f_mat(2,2) - f_mat(2,0)*f_mat(0,2) ),
                            oneOverDet * ( f_mat(1,0)*f_mat(0,2) - f_mat(0,0)*f_mat(1,2) ),
                            oneOverDet * ( f_mat(1,0)*f_mat(2,1) - f_mat(2,0)*f_mat(1,1) ),
                            oneOverDet * ( f_mat(2,0)*f_mat(0,1) - f_mat(0,0)*f_mat(2,1) ),
                            oneOverDet * ( f_mat(0,0)*f_mat(1,1) - f_mat(1,0)*f_mat(0,1) )
                        );
}


template <class ValueType>
void vfc::linalg::fromXAxisRotation (TMatrix33<ValueType>& f_mat, const CRadian& f_angleX)
{
    ValueType    cx = static_cast<ValueType>(cos(f_angleX));
    ValueType    sx = static_cast<ValueType>(sin(f_angleX));

    f_mat.set(static_cast<ValueType>(1),
            static_cast<ValueType>(0),
            static_cast<ValueType>(0),
            static_cast<ValueType>(0), 
            cx, 
            -sx,
            static_cast<ValueType>(0), 
            sx, 
            cx);
}


template <class ValueType>
void    vfc::linalg::fromYAxisRotation    (TMatrix33<ValueType>& f_mat, const CRadian& f_angleY)
{
    ValueType    cy = static_cast<ValueType>(cos(f_angleY));
    ValueType    sy = static_cast<ValueType>(sin(f_angleY));

    f_mat.set(cy,
            static_cast<ValueType>(0),
            sy,
            static_cast<ValueType>(0), 
            static_cast<ValueType>(1), 
            static_cast<ValueType>(0),
            -sy, 
            static_cast<ValueType>(0), 
            cy);
}

template <class ValueType>
void    vfc::linalg::fromZAxisRotation    (TMatrix33<ValueType>& f_mat, const CRadian& f_angleZ)
{
    ValueType    cz = static_cast<ValueType>(cos(f_angleZ));
    ValueType    sz = static_cast<ValueType>(sin(f_angleZ));

    f_mat.set( cz,
            -sz, 
            static_cast<ValueType>(0),
            sz, 
            cz, 
            static_cast<ValueType>(0),
            static_cast<ValueType>(0), 
            static_cast<ValueType>(0), 
            static_cast<ValueType>(1));
}

template <class ValueType>
void    vfc::linalg::fromZYXAxisRotation    (TMatrix33<ValueType>& f_mat, const CRadian& f_angleX, const CRadian& f_angleY, const CRadian& f_angleZ)
{
    ValueType    cx = static_cast<ValueType>(cos(f_angleX));
    ValueType    sx = static_cast<ValueType>(sin(f_angleX));

    ValueType    cy = static_cast<ValueType>(cos(f_angleY));
    ValueType    sy = static_cast<ValueType>(sin(f_angleY));

    ValueType    cz = static_cast<ValueType>(cos(f_angleZ));
    ValueType    sz = static_cast<ValueType>(sin(f_angleZ));

    f_mat.set(cy * cz,
            sx * sy * cz - cx * sz,
            cx * sy * cz + sx * sz,
            cy * sz,
            sx * sy * sz + cx * cz,       
            cx * sy * sz - sx * cz,
            - sy,
            sx * cy,
            cx * cy);
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_linalg_matrix33_ops.inl  $
//  Revision 1.1 2007/05/09 10:21:21MESZ Jaeger Thomas (CC-DA/EPV2) (jat2hi) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/linalg/linalg.pj
//=============================================================================
