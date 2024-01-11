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
//       Projectname: vfc/geo
//          Synopsis: Quaternion implementation
//  Target system(s): 
//       Compiler(s): VC_71
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
///     $Source: vfc_quat.hpp $
///     $Revision: 1.8 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2011/01/21 12:53:30MEZ $
///     $Locker:  $
///     $Name:  $
///     $State: in_work $
///
/// @par Review Information:
/// - Reviewed version: 
/// - Type (use "X" to mark):
///        - [ ] Formal Review
///        - [ ] Walkthrough
///        - [ ] Inspection
/// - State including date (DD.MM.YYYY)
///        - [--.--.----] Preparation
///        - [--.--.----] Review audit 
///        - [--.--.----] Integration of findings
///        - [--.--.----] Test
///        - [--.--.----] Verification of integration of findings
///        - [--.--.----] Review release
///    - Responsible:
///    - Review-Document:        
//=============================================================================

#ifndef VFC_QUAT_HPP_INCLUDED
#define VFC_QUAT_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

namespace vfc
{    // namespace vfc opened

    //=========================================================================
    //    TQuaternion<>
    //-------------------------------------------------------------------------
    /// Quaternion class, representing a quaternion as a four-component vector.
    /// @par Description:
    /// In the 19th century, Sir William Hamilton developed the quaternion as 
    /// an extension to complex numbers.\n 
    /// Instead of having one real component and only one imaginary component, 
    /// quaternions have one real component and three imaginary components: 
    /// q = iqx+jqy+kqz+qw, with i2=j2=k2=-1.\n
    /// The main practical application of a quaternion is to represent 
    /// 3D rotations.\n
    /// The traditional method for representing 3D rotations has always been 
    /// with Euler angles - rotations about the three coordinate axes. This 
    /// method suffers from a dependence between the angles which means that 
    /// rotation with Euler angles is prone to effects such as Gimbal Lock.\n 
    /// Quaternions offer a much better method of representing rotations than 
    /// the traditional Euler angles because the extra, fourth parameter 
    /// removes this dependence. 
    /// @author zvh2hi
    /// @ingroup vfc_group_geo_linalg
    //=========================================================================

    template <class T>
    class TQuaternion
    {
    public:
        typedef T*       iterator;
        typedef T const* const_iterator;
        typedef T value_type;

    public:
        /// default c'tor, sets quaternion to identity.
        TQuaternion    (void)    { set(static_cast<T>(0),static_cast<T>(0),static_cast<T>(0),static_cast<T>(1)); }

        /// assigns specified value to all four quaternion components.
        explicit
        TQuaternion    (const T& f_qxyzw)    {    set(f_qxyzw);}

        /// assigns specified values to quaternion components.
        TQuaternion    (const T& f_qx, const T& f_qy, const T& f_qz, const T& f_qw)    {    set(f_qx,f_qy,f_qz,f_qw);}
        
        /// template copy c'tor for converting quaternions with compatible value type.
        template <class U> explicit
        TQuaternion    (const TQuaternion<U>& f_rhs);

        /// multiply-assign operator for scalar values (q = q*s).
        const TQuaternion<T>&    operator*= (const T& f_rhs);

        /// divide-assign operator for scalar values (q=q/s).
        const TQuaternion<T>&    operator/= (const T& f_rhs);
        
        /// add-assign operator for quaternions. 
        const TQuaternion<T>&    operator+= (const TQuaternion<T>& f_rhs);

        /// subtract-assign operator for quaternions. 
        const TQuaternion<T>&    operator-= (const TQuaternion<T>& f_rhs);

        /// sets all quaternion components to specified values.
        void    set    (    const T& f_qx, const T& f_qy, const T& f_qz, const T& f_qw);

        /// sets all quaternion components to specified value.
        void    set    (const T& f_qxyzw);

        /// returns quaternion component with specified index (0=iqx, 1=jqy, 2=kqz, 3=qw), asserts if index is out of range.
        const    T&    operator[](int32_t f_idx)    const;
        /// returns quaternion component with specified index (0=iqx, 1=jqy, 2=kqz, 3=qw), asserts if index is out of range.
                T&    operator[](int32_t f_idx);

        /// returns imaginary quaternion component x. 
        const    T&    x(void)    const    {    return m_data[0];}
        /// returns imaginary quaternion component x. 
                T&    x(void)            {    return m_data[0];}
        /// returns imaginary quaternion component y. 
        const    T&    y(void)    const    {    return m_data[1];}
        /// returns imaginary quaternion component y. 
                T&    y(void)            {    return m_data[1];}
        /// returns imaginary quaternion component z. 
        const    T&    z(void)    const    {    return m_data[2];}
        /// returns imaginary quaternion component z .
                T&    z(void)            {    return m_data[2];}
        /// returns real quaternion component w. 
        const    T&    w(void)    const    {    return m_data[3];}
        /// returns real quaternion component w.
                T&    w(void)            {    return m_data[3];}

        /// returns a const iterator to first element in quaternion sequence.
        const_iterator  begin   (void) const    {   return m_data;}
        /// returns an iterator to first element in quaternion sequence.
        iterator        begin   (void)          {   return m_data;}

        /// returns a const iterator that points just beyond the end of the quaternion sequence.
        const_iterator  end     (void) const    {   return begin()+4;}
        /// returns a iterator that points just beyond the end of the quaternion sequence.
        iterator        end     (void)          {   return begin()+4;}

        /// returns the product of a quaternion and a scalar. 
        friend 
        TQuaternion<T>    operator* (const T& f_scalar, TQuaternion<T> f_quat) { return f_quat*=f_scalar;}
        /// returns the product of a quaternion and a scalar. 
        friend 
        TQuaternion<T>    operator* (TQuaternion<T> f_quat, const T& f_scalar) { return f_quat*=f_scalar;}
        /// returns the quotient of a quaternion and a scalar.
        friend 
        TQuaternion<T>    operator/ (TQuaternion<T> f_quat, const T& f_scalar) { return f_quat/=f_scalar;}

    private:
        const TQuaternion<T>&    divide_assign(const T& f_rhs, true_t);
        const TQuaternion<T>&    divide_assign(const T& f_rhs, false_t);
    
    private:
        T    m_data[4];
    };

    //-------------------------------------------------------------------------
    /// a template helper function used to construct TQuaternion objects,
    /// the value type is based on the data type of the passed parameters.
    /// @relates TQuaternion
    //-------------------------------------------------------------------------

    template <class T>    inline    
    TQuaternion<T>    make_quat (    const T& f_qx, const T& f_qy, const T& f_qz, const T& f_qw)
    {
        return TQuaternion<T>(f_qx,f_qy,f_qz,f_qw);
    }    
    
    // convenience typedefs
    typedef TQuaternion<float32_t>    CQuaternionf;
    typedef TQuaternion<float64_t>    CQuaterniond;

}    // namespace vfc closed

#include "vfc/geo/vfc_quat.inl"

#endif //VFC_QUAT_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_quat.hpp  $
//  Revision 1.8 2011/01/21 12:53:30MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - sometimes .hpp or namespace qualifiers are missing (mantis3599)
//  Revision 1.7 2007/07/23 09:47:33MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen grouping (mantis1744)
//  Revision 1.6 2006/11/16 14:41:14CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.5 2006/10/26 10:17:56CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - removed explicit template instantiations (mantis1239)
//  - replaced header/footer templates
//  Revision 1.4 2006/10/13 10:00:25CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - changed header/footer template    
//  - added documentation
//  Revision 1.3 2006/07/27 10:08:35CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -added documentation
//  -following coding conventions
//  Revision 1.2 2006/07/07 10:37:16CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  -added begin(), end() methods (mantis1121)
//=============================================================================
