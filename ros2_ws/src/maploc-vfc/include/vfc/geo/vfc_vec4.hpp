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
///     $Source: vfc_vec4.hpp $
///     $Revision: 1.14 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/07/23 09:50:32MESZ $
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

#ifndef VFC_VEC4_HPP_INCLUDED
#define VFC_VEC4_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

namespace vfc
{    // namespace vfc opened

    //=========================================================================
    //  TVector4<>  
    //-------------------------------------------------------------------------
    //! 4-dim vector class.
    //! @author zvh2hi
    //! @ingroup vfc_group_geo_linalg
    //=========================================================================

    template <class T>
    class TVector4
    {
    public:
        typedef T value_type;
        
        typedef T*       iterator;
        typedef T const* const_iterator;

    public:
        TVector4    (void)        {}

        explicit
        TVector4    (const T& val)    {    set(val);}

        TVector4    (const T& val1, const T& val2, const T& val3, const T& val4 = static_cast<T>(1))    {    set(val1,val2,val3,val4);}
        
        template <class U> explicit
        TVector4    (const TVector4<U>& rhs);

        const TVector4<T>&    operator*= (const T& rhs);

        const TVector4<T>&    operator/= (const T& rhs);
        
        const TVector4<T>&    operator+= (const TVector4<T>& rhs);

        const TVector4<T>&    operator-= (const TVector4<T>& rhs);

        void    set    (    const T& val1, const T& val2, const T& val3, const T& val4);

        void    set    (const T& val);

        const    T&    operator[](int32_t idx)    const;
                T&    operator[](int32_t idx);

        const    T&    x(void)    const    {    return m_data[0];}
                T&    x(void)            {    return m_data[0];}

        const    T&    y(void)    const    {    return m_data[1];}
                T&    y(void)            {    return m_data[1];}

        const    T&    z(void)    const    {    return m_data[2];}
                T&    z(void)            {    return m_data[2];}

        const    T&    w(void)    const    {    return m_data[3];}
                T&    w(void)            {    return m_data[3];}

        const_iterator  begin   (void) const    {   return m_data;}
        iterator        begin   (void)          {   return m_data;}

        const_iterator  end     (void) const    {   return begin()+4;}
        iterator        end     (void)          {   return begin()+4;}

    friend TVector4<T>    operator* (const T& f_scalar, TVector4<T> f_vec4) { return f_vec4*=f_scalar;}
    friend TVector4<T>    operator* (TVector4<T> f_vec4, const T& f_scalar) { return f_vec4*=f_scalar;}
    friend TVector4<T>    operator/ (TVector4<T> f_vec4, const T& f_scalar) { return f_vec4/=f_scalar;}

    private:
        const TVector4<T>&    divide_assign(const T& rhs, true_t);
        const TVector4<T>&    divide_assign(const T& rhs, false_t);
    
    private:
        T    m_data[4];
    };


    //-------------------------------------------------------------------------
    //! convenience function for generating a 4-dim vector with specified value 
    //! type.
    //! @relates TVector4
    //! @author zvh2hi
    //-------------------------------------------------------------------------

    template <class T>    inline    
    TVector4<T>    make_vec (const T& val1, const T& val2, const T& val3, const T& val4)
    {
        return TVector4<T>(val1,val2,val3,val4);
    }    
    
    typedef TVector4<int32_t>    CVector4i;
    typedef TVector4<float32_t> CVector4f;
    typedef TVector4<float64_t> CVector4d;

}    // namespace vfc closed

#include "vfc/geo/vfc_vec4.inl"

#endif //VFC_VEC4_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_vec4.hpp  $
//  Revision 1.14 2007/07/23 09:50:32MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.13 2006/11/16 14:41:08CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.12 2006/10/26 10:17:57CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - removed explicit template instantiations (mantis1239)
//  - replaced header/footer templates
//  Revision 1.11 2006/07/07 10:37:16CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -added begin(), end() methods (mantis1121)
//  Revision 1.10 2006/07/07 10:05:41CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -cosmetics
//  Revision 1.9 2006/02/14 16:16:22CET Muehlmann Karsten (AE-DA/ESA3) * (muk2lr) 
//  moved explicit instantiations at very end of file, removes compiler warnings (mantis943)
//  Revision 1.8 2006/01/11 09:51:11CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -fixed bug in the w() component access methods
//  Revision 1.7 2005/12/07 16:17:26CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added explicit c'tor for setting all elements
//  Revision 1.6 2005/12/07 16:11:48CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -restructuring
//  Revision 1.5 2005/11/08 13:02:09CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added make_vec() func
//  Revision 1.4 2005/11/08 08:32:49CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added explicit template instantiation for int32, float32 and float64 types
//  -added typedefs for int32, float32 and float64 types
//  Revision 1.3 2005/11/07 18:21:02CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -changed project name to vfc/geo
//  Revision 1.2 2005/11/07 18:15:47CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  -added default value for homogenous component
//=============================================================================
