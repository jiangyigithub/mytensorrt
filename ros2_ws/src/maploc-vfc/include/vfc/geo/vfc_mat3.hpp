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
//  Department: 
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_mat3.hpp $
///     $Revision: 1.11 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/07/23 09:42:36MESZ $
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

#ifndef VFC_MAT3_HPP_INCLUDED
#define VFC_MAT3_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

namespace vfc
{    // namespace vfc opened

    //=========================================================================
    //  TMatrix3<>  
    //-------------------------------------------------------------------------
    //! 3-dim matrix class.
    //! @author zvh2hi
    //! @ingroup vfc_group_geo_linalg
    //=========================================================================

    template <class T>
    class    TMatrix3
    {
    public:
        typedef T*       iterator;
        typedef T const* const_iterator;
        typedef T value_type;

    public:

        TMatrix3    (void)    {}

        explicit
        TMatrix3    (const T& val)    {    set(val);}
        
        TMatrix3    (    const T& m00, const T& m01, const T& m02, 
                        const T& m10, const T& m11, const T& m12, 
                        const T& m20, const T& m21, const T& m22    );
        
        template <class U> explicit 
        TMatrix3    (const TMatrix3<U>& rhs);

        const TMatrix3<T>&    operator*= (const T& rhs);

        const TMatrix3<T>&    operator/= (const T& rhs);
        
        const TMatrix3<T>&    operator+= (const TMatrix3<T>& rhs);

        const TMatrix3<T>&    operator-= (const TMatrix3<T>& rhs);

        void    set (    const T& val);

        void    set (    const T& m00, const T& m01, const T& m02, 
                        const T& m10, const T& m11, const T& m12, 
                        const T& m20, const T& m21, const T& m22    );

        const    T&    operator()    (int32_t row, int32_t col)    const;

                T&    operator()    (int32_t row, int32_t col);

        const_iterator  begin   (void) const    {   return &m_data[0][0];}
        iterator        begin   (void)          {   return &m_data[0][0];}

        const_iterator  end     (void) const    {   return begin()+9;}
        iterator        end     (void)          {   return begin()+9;}

        friend TMatrix3<T>    operator* (const T& f_scalar, TMatrix3<T> f_mat3) { return f_mat3*=f_scalar;}
        friend TMatrix3<T>    operator* (TMatrix3<T> f_mat3, const T& f_scalar) { return f_mat3*=f_scalar;}
        friend TMatrix3<T>    operator/ (TMatrix3<T> f_mat3, const T& f_scalar) { return f_mat3/=f_scalar;}

    private:
        const TMatrix3<T>&    divide_assign(const T& rhs, true_t);
        const TMatrix3<T>&    divide_assign(const T& rhs, false_t);
    
    private:
        T    m_data[3][3];
    };

    typedef TMatrix3<int32_t>    CMatrix3i;
    typedef TMatrix3<float32_t> CMatrix3f;
    typedef TMatrix3<float64_t> CMatrix3d;

}    // namespace vfc closed

#include "vfc/geo/vfc_mat3.inl"

#endif //VFC_MAT3_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_mat3.hpp  $
//  Revision 1.11 2007/07/23 09:42:36MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.10 2006/11/16 14:41:14CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.9 2006/10/26 10:17:57CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - removed explicit template instantiations (mantis1239)
//  - replaced header/footer templates
//  Revision 1.8 2006/07/07 10:37:16CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -added begin(), end() methods (mantis1121)
//  Revision 1.7 2006/07/07 10:04:57CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -cosmetics
//  Revision 1.6 2006/02/14 16:16:22CET Muehlmann Karsten (AE-DA/ESA3) * (muk2lr) 
//  moved explicit instantiations at very end of file, removes compiler warnings (mantis943)
//  Revision 1.5 2005/12/07 16:17:25CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added explicit c'tor for setting all elements
//  Revision 1.4 2005/12/07 16:10:07CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added mat-scalar friend functions
//  Revision 1.3 2005/11/08 08:32:49CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added explicit template instantiation for int32, float32 and float64 types
//  -added typedefs for int32, float32 and float64 types
//  Revision 1.2 2005/11/07 18:20:58CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -changed project name to vfc/geo
//=============================================================================
