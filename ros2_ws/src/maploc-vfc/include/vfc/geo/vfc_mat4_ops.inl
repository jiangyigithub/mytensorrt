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
//        Name: zvh2hi
//  Department: CR/AEM
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_mat4_ops.inl $
///     $Revision: 1.10 $
///     $Author: gaj2kor $
///     $Date: 2008/09/11 14:25:48MESZ $
///     $Locker:  $
///     $Name: 0032 RC1  $
///     $State: in_work $
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


#include "vfc/core/vfc_math.hpp"            // used for isZero()
#include "vfc/core/vfc_static_assert.hpp"   // used for VFC_STATIC_ASSERT()
#include "vfc/core/vfc_metaprog.hpp"        // used for TIsFloating<>

namespace vfc
{    // namespace vfc opened

    //-------------------------------------------------------------------------
    // conditional doxygen documentation
    //! @cond VFC_DOXY_INTERN
    //-------------------------------------------------------------------------

    namespace intern
    {    // namespace intern opened
        template <class T, class U>    inline
        TMatrix4<T>    divide    (const TMatrix4<T>& mat, const U& denom, true_t)
        {
            const U    fac = static_cast<U>(1)/denom;
            return (mat*fac);
        }

        template <class T, class U>    inline
        TMatrix4<T>    divide    (const TMatrix4<T>& mat, const U& denom, false_t)
        {
            return TMatrix4<T>    (    mat(0,0)/denom, mat(0,1)/denom, mat(0,2)/denom, mat(0,3)/denom,
                                    mat(1,0)/denom, mat(1,1)/denom, mat(1,2)/denom, mat(1,3)/denom,
                                    mat(2,0)/denom, mat(2,1)/denom, mat(2,2)/denom, mat(2,3)/denom,
                                    mat(3,0)/denom, mat(3,1)/denom, mat(3,2)/denom, mat(3,3)/denom
                                );
        }
    }    // namespace intern closed

    //-----------------------------------------------------------------------------
    /// @endcond
    //  of VFC_DOXY_INTERN
    //-----------------------------------------------------------------------------

}    // namespace vfc closed



template <class T>    inline
T    vfc::det    (const TMatrix4<T>& mat)
{
    const T da = mat(0,2)*mat(1,3) - mat(1,2)*mat(0,3);
    const T db = mat(0,1)*mat(1,3) - mat(1,1)*mat(0,3);
    const T dc = mat(0,1)*mat(1,2) - mat(1,1)*mat(0,2);
    const T dd = mat(0,0)*mat(1,3) - mat(1,0)*mat(0,3);
    const T de = mat(0,0)*mat(1,2) - mat(1,0)*mat(0,2);
    const T df = mat(0,0)*mat(1,1) - mat(1,0)*mat(0,1);

    return    (    -    mat(3,0)*( mat(2,1)*da - mat(2,2)*db + mat(2,3)*dc)
                +    mat(3,1)*( mat(2,0)*da - mat(2,2)*dd + mat(2,3)*de)
                -    mat(3,2)*( mat(2,0)*db - mat(2,1)*dd + mat(2,3)*df)
                +    mat(3,3)*( mat(2,0)*dc - mat(2,1)*de + mat(2,2)*df)
            );
}

template <class T>    inline
T    vfc::trace    (const TMatrix4<T>& mat)
{
    return    ( mat(0,0) + mat(1,1) + mat(2,2) + mat(3,3) );
}

template <class T>    inline
vfc::TMatrix4<T>    vfc::inverse (const TMatrix4<T>& mat)
{
    // only support inverse for floating types
    VFC_STATIC_ASSERT(vfc::TIsFloating<T>::value);

    const    T deta = mat(0,2)*mat(1,3) - mat(0,3)*mat(1,2);
    const    T detb = mat(0,1)*mat(1,3) - mat(0,3)*mat(1,1);
    const    T detc = mat(0,1)*mat(1,2) - mat(0,2)*mat(1,1);
    const    T detd = mat(0,0)*mat(1,3) - mat(1,0)*mat(0,3);
    const    T dete = mat(0,0)*mat(1,2) - mat(1,0)*mat(0,2);
    const    T detf = mat(0,0)*mat(1,1) - mat(1,0)*mat(0,1);

    const T d =    -    mat(3,0)*( mat(2,1)*deta - mat(2,2)*detb + mat(2,3)*detc)
                +    mat(3,1)*( mat(2,0)*deta - mat(2,2)*detd + mat(2,3)*dete)
                -    mat(3,2)*( mat(2,0)*detb - mat(2,1)*detd + mat(2,3)*detf)
                +    mat(3,3)*( mat(2,0)*detc - mat(2,1)*dete + mat(2,2)*detf);

    VFC_REQUIRE2(!isZero(d), "TMatrix4<T> is singular");

    const    T    oneOverDet = static_cast<T>(1)/d;

    const    T detg = mat(2,2)*mat(3,3) - mat(2,3)*mat(3,2);
    const    T deth = mat(1,2)*mat(3,3) - mat(1,3)*mat(3,2);
    const    T deti = mat(1,2)*mat(2,3) - mat(1,3)*mat(2,2);
    const    T detj = mat(2,1)*mat(3,3) - mat(2,3)*mat(3,1);
    const    T detk = mat(1,1)*mat(3,3) - mat(1,3)*mat(3,1);
    const    T detl = mat(1,1)*mat(2,3) - mat(1,3)*mat(2,1);
    const    T detm = mat(2,1)*mat(3,2) - mat(2,2)*mat(3,1);
    const    T detn = mat(1,1)*mat(3,2) - mat(1,2)*mat(3,1);
    const    T deto = mat(1,1)*mat(2,2) - mat(1,2)*mat(2,1);
    const    T detp = mat(0,2)*mat(3,3) - mat(0,3)*mat(3,2);
    const    T detq = mat(0,2)*mat(2,3) - mat(0,3)*mat(2,2);
    const    T detr = mat(0,1)*mat(3,3) - mat(0,3)*mat(3,1);
    const    T dets = mat(0,1)*mat(2,3) - mat(0,3)*mat(2,1);
    const    T dett = mat(0,1)*mat(3,2) - mat(0,2)*mat(3,1);
    const    T detu = mat(0,1)*mat(2,2) - mat(0,2)*mat(2,1);

    return TMatrix4<T> (    oneOverDet*(    mat(1,1)*detg - mat(2,1)*deth + mat(3,1)*deti),
                            oneOverDet*(-    mat(0,1)*detg + mat(2,1)*detp - mat(3,1)*detq),
                            oneOverDet*(    mat(0,1)*deth - mat(1,1)*detp + mat(3,1)*deta),
                            oneOverDet*(-    mat(0,1)*deti + mat(1,1)*detq - mat(2,1)*deta),

                            oneOverDet*(-    mat(1,0)*detg + mat(2,0)*deth - mat(3,0)*deti),
                            oneOverDet*(    mat(0,0)*detg - mat(2,0)*detp + mat(3,0)*detq),
                            oneOverDet*(-    mat(0,0)*deth + mat(1,0)*detp - mat(3,0)*deta),
                            oneOverDet*(    mat(0,0)*deti - mat(1,0)*detq + mat(2,0)*deta),

                            oneOverDet*(    mat(1,0)*detj - mat(2,0)*detk + mat(3,0)*detl),
                            oneOverDet*(-    mat(0,0)*detj + mat(2,0)*detr - mat(3,0)*dets),
                            oneOverDet*(    mat(0,0)*detk - mat(1,0)*detr + mat(3,0)*detb),
                            oneOverDet*(-    mat(0,0)*detl + mat(1,0)*dets - mat(2,0)*detb),

                            oneOverDet*(-    mat(1,0)*detm + mat(2,0)*detn - mat(3,0)*deto),
                            oneOverDet*(    mat(0,0)*detm - mat(2,0)*dett + mat(3,0)*detu),
                            oneOverDet*(-    mat(0,0)*detn + mat(1,0)*dett - mat(3,0)*detc),
                            oneOverDet*(    mat(0,0)*deto - mat(1,0)*detu + mat(2,0)*detc)
                        );
}

template <class T>    inline
vfc::TMatrix4<T>    vfc::transpose (const TMatrix4<T>& mat)
{
    return TMatrix4<T>    (    mat(0,0), mat(1,0), mat(2,0), mat(3,0),
                            mat(0,1), mat(1,1), mat(2,1), mat(3,1),
                            mat(0,2), mat(1,2), mat(2,2), mat(3,2),
                            mat(0,3), mat(1,3), mat(2,3), mat(3,3)
                        );

}

template <class T>    inline
void    vfc::identity (TMatrix4<T>& mat)
{
    mat.set    (    static_cast<T>(1), static_cast<T>(0), static_cast<T>(0), static_cast<T>(0),
                static_cast<T>(0), static_cast<T>(1), static_cast<T>(0), static_cast<T>(0),
                static_cast<T>(0), static_cast<T>(0), static_cast<T>(1), static_cast<T>(0),
                static_cast<T>(0), static_cast<T>(0), static_cast<T>(0), static_cast<T>(1)
            );
}

template <class T>    inline
vfc::TMatrix4<T>        vfc::operator-    (const TMatrix4<T>& mat)
{
    return TMatrix4<T>    (    -mat(0,0), -mat(0,1), -mat(0,2), -mat(0,3),
                            -mat(1,0), -mat(1,1), -mat(1,2), -mat(1,3),
                            -mat(2,0), -mat(2,1), -mat(2,2), -mat(2,3),
                            -mat(3,0), -mat(3,1), -mat(3,2), -mat(3,3)
                        );
}

// binary

template <class T>    inline
vfc::TMatrix4<T>        vfc::mul_scalar    (const TMatrix4<T>& mat, const T& fac)
{
    return TMatrix4<T>    (    static_cast<T>(fac*mat(0,0)), static_cast<T>(fac*mat(0,1)), static_cast<T>(fac*mat(0,2)), static_cast<T>(fac*mat(0,3)),
                            static_cast<T>(fac*mat(1,0)), static_cast<T>(fac*mat(1,1)), static_cast<T>(fac*mat(1,2)), static_cast<T>(fac*mat(1,3)),
                            static_cast<T>(fac*mat(2,0)), static_cast<T>(fac*mat(2,1)), static_cast<T>(fac*mat(2,2)), static_cast<T>(fac*mat(2,3)),
                            static_cast<T>(fac*mat(3,0)), static_cast<T>(fac*mat(3,1)), static_cast<T>(fac*mat(3,2)), static_cast<T>(fac*mat(3,3))
                        );
}

template <class T>    inline
vfc::TMatrix4<T>        vfc::div_scalar    (const TMatrix4<T>& mat, const T& fac)
{
    VFC_REQUIRE2(!isZero(fac), "Division by zero");
    return vfc::intern::divide( mat, fac, typename TInt2Boolean<TIsFloating<T>::value>::type());

}

template <class T>    inline
vfc::TMatrix4<T>        vfc::operator*    (const TMatrix4<T>& op1, const TMatrix4<T>& op2)
{
    return TMatrix4<T> (    op1(0,0)*op2(0,0)+op1(0,1)*op2(1,0)+op1(0,2)*op2(2,0)+op1(0,3)*op2(3,0),
                            op1(0,0)*op2(0,1)+op1(0,1)*op2(1,1)+op1(0,2)*op2(2,1)+op1(0,3)*op2(3,1),
                            op1(0,0)*op2(0,2)+op1(0,1)*op2(1,2)+op1(0,2)*op2(2,2)+op1(0,3)*op2(3,2),
                            op1(0,0)*op2(0,3)+op1(0,1)*op2(1,3)+op1(0,2)*op2(2,3)+op1(0,3)*op2(3,3),

                            op1(1,0)*op2(0,0)+op1(1,1)*op2(1,0)+op1(1,2)*op2(2,0)+op1(1,3)*op2(3,0),
                            op1(1,0)*op2(0,1)+op1(1,1)*op2(1,1)+op1(1,2)*op2(2,1)+op1(1,3)*op2(3,1),
                            op1(1,0)*op2(0,2)+op1(1,1)*op2(1,2)+op1(1,2)*op2(2,2)+op1(1,3)*op2(3,2),
                            op1(1,0)*op2(0,3)+op1(1,1)*op2(1,3)+op1(1,2)*op2(2,3)+op1(1,3)*op2(3,3),

                            op1(2,0)*op2(0,0)+op1(2,1)*op2(1,0)+op1(2,2)*op2(2,0)+op1(2,3)*op2(3,0),
                            op1(2,0)*op2(0,1)+op1(2,1)*op2(1,1)+op1(2,2)*op2(2,1)+op1(2,3)*op2(3,1),
                            op1(2,0)*op2(0,2)+op1(2,1)*op2(1,2)+op1(2,2)*op2(2,2)+op1(2,3)*op2(3,2),
                            op1(2,0)*op2(0,3)+op1(2,1)*op2(1,3)+op1(2,2)*op2(2,3)+op1(2,3)*op2(3,3),

                            op1(3,0)*op2(0,0)+op1(3,1)*op2(1,0)+op1(3,2)*op2(2,0)+op1(3,3)*op2(3,0),
                            op1(3,0)*op2(0,1)+op1(3,1)*op2(1,1)+op1(3,2)*op2(2,1)+op1(3,3)*op2(3,1),
                            op1(3,0)*op2(0,2)+op1(3,1)*op2(1,2)+op1(3,2)*op2(2,2)+op1(3,3)*op2(3,2),
                            op1(3,0)*op2(0,3)+op1(3,1)*op2(1,3)+op1(3,2)*op2(2,3)+op1(3,3)*op2(3,3));
}

template <class T>    inline
vfc::TMatrix4<T>        vfc::operator+    (const TMatrix4<T>& op1, const TMatrix4<T>& op2)
{
    return TMatrix4<T> (    op1(0,0) + op2(0,0), op1(0,1) + op2(0,1), op1(0,2) + op2(0,2), op1(0,3) + op2(0,3),
                            op1(1,0) + op2(1,0), op1(1,1) + op2(1,1), op1(1,2) + op2(1,2), op1(1,3) + op2(1,3),
                            op1(2,0) + op2(2,0), op1(2,1) + op2(2,1), op1(2,2) + op2(2,2), op1(2,3) + op2(2,3),
                            op1(3,0) + op2(3,0), op1(3,1) + op2(3,1), op1(3,2) + op2(3,2), op1(3,3) + op2(3,3)
                        );
}

template <class T>    inline
vfc::TMatrix4<T>        vfc::operator-    (const TMatrix4<T>& op1, const TMatrix4<T>& op2)
{
    return TMatrix4<T> (    op1(0,0) - op2(0,0), op1(0,1) - op2(0,1), op1(0,2) - op2(0,2), op1(0,3) - op2(0,3),
                            op1(1,0) - op2(1,0), op1(1,1) - op2(1,1), op1(1,2) - op2(1,2), op1(1,3) - op2(1,3),
                            op1(2,0) - op2(2,0), op1(2,1) - op2(2,1), op1(2,2) - op2(2,2), op1(2,3) - op2(2,3),
                            op1(3,0) - op2(3,0), op1(3,1) - op2(3,1), op1(3,2) - op2(3,2), op1(3,3) - op2(3,3)
                        );
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_mat4_ops.inl  $
//  Revision 1.10 2008/09/11 14:25:48MESZ gaj2kor 
//  -Resolution of QAC++ warning related to Rule 9.0.3
//  (Mantis : 2221)
//  Revision 1.9 2007/08/02 19:17:14IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added conditional doxygen documentation generation of vfc::intern (mantis1758)
//  Revision 1.8 2007/03/16 11:35:07CET Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  replaced exception by precondition (mantis1420)
//  Revision 1.7 2006/11/16 14:41:07CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.6 2006/10/13 15:15:10CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added some more msiing includes (mantis1205)
//  - changed header/footer templates
//  Revision 1.5 2006/10/13 10:06:18CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  -added missing include file (mantis1205)
//  Revision 1.4 2006/07/07 15:23:26CEST Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  allow inverse only for floating point types (mantis1110)
//  Revision 1.3 2005/12/07 16:11:47CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -restructuring
//  Revision 1.2 2005/11/30 11:10:02CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -fixed implementation bug
//  Revision 1.1 2005/11/16 17:45:14CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/geo/geo.pj
//=============================================================================
