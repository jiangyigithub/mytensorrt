//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2007 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy, use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/core
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
///     $Source: vfc_tuple.hpp $
///     $Revision: 1.4 $
///     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
///     $Date: 2016/04/12 16:37:58MESZ $
///     $Locker:  $
///     $Name:  $
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

#ifndef VFC_TUPLE_HPP_INCLUDED
#define VFC_TUPLE_HPP_INCLUDED


namespace vfc
{   // namespace vfc opened

    //=========================================================================
    //  TPair
    //-------------------------------------------------------------------------
    //! A class that provides for the ability to treat two objects of arbitrary 
    //! types as a single object.
    //! The template class stores a pair of objects of type FirstType and SecondType, 
    //! respectively. 
    //! The type first_type is the same as the template parameter FirstType and 
    //! the type second_type is the same as the template parameter SecondType. 
    //! FirstType and SecondType each need supply only a default constructor, 
    //! a single-argument constructor, and a destructor. 
    //!
    //! The most common use for a TPair is as return types for functions that 
    //! return two values.
    //! @sa
    //! - TTriple
    //! - TQuadruple
    //! @ingroup vfc_group_core_types
    //! @author zvh2hi
    //=========================================================================

    template <class FirstType, class SecondType=FirstType>
    class TPair
    {
    public:
        typedef TPair<FirstType,SecondType> tuple_type;

        typedef FirstType   first_type;
        typedef SecondType  second_type;

    public:
        //! c'tor, initializes all members with their default c'tor
        TPair (void);
        //! c'tor, initializes members with specified values
        TPair (const first_type& f_first, const second_type& f_second);
        //! Copy Constructor
        TPair (const TPair<FirstType,SecondType>& f_other);
        //! Copy Assignment Operator
        TPair& operator=(const TPair& f_other);

        //! returns the first object (read-only)
        const first_type&   first    (void) const    { return m_first;}
        //! returns the second object (read-only)
        const second_type&  second   (void) const    { return m_second;}
        
        //! returns the first object (read-write)
        first_type&   first    (void)    { return m_first;}
        //! returns the second object (read-write)
        second_type&  second   (void)    { return m_second;}

    private:
        first_type  m_first;    //! first object
        second_type m_second;   //! second object
    };

    // creation functions

    //-------------------------------------------------------------------------
    //! A template helper function used to construct objects of type TPair<T1,T2>, 
    //! where the component types are based on the data types passed as parameters.
    //! @relates TPair
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType>   
    TPair<FirstType,SecondType> make_pair   (const FirstType& f_first, const SecondType& f_second);

    //-------------------------------------------------------------------------
    //! A template helper function used to construct objects of type TPair<T1,T2>, 
    //! where the component types are based on the data types passed as parameters.
    //! @relates TPair
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType>   
    TPair<FirstType,SecondType> make_tuple  (const FirstType& f_first, const SecondType& f_second);

    // template logical operators
    
    //-------------------------------------------------------------------------
    //! returns true if two TPair objects are identical.
    //! @relates TPair
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType>   
    bool    operator==  (const TPair<FirstType,SecondType>& f_lhs, const TPair<FirstType,SecondType>& f_rhs);

    //-------------------------------------------------------------------------
    //! returns true if two TPair objects are not identical.
    //! @relates TPair
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType>   
    bool    operator!=  (const TPair<FirstType,SecondType>& f_lhs, const TPair<FirstType,SecondType>& f_rhs);

    //-------------------------------------------------------------------------
    //! returns true for (A < B) if TPair object A is less than TPair object B.
    //! @relates TPair
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType>   
    bool    operator<   (const TPair<FirstType,SecondType>& f_lhs, const TPair<FirstType,SecondType>& f_rhs);

    //-------------------------------------------------------------------------
    //! returns true for (A > B) if TPair object A is greater than TPair object B.
    //! @relates TPair
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType>   
    bool    operator>   (const TPair<FirstType,SecondType>& f_lhs, const TPair<FirstType,SecondType>& f_rhs);

    //-------------------------------------------------------------------------
    //! returns true for (A <= B) if TPair object A is less than or equal to TPair object B.
    //! @relates TPair
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType>   
    bool    operator<=  (const TPair<FirstType,SecondType>& f_lhs, const TPair<FirstType,SecondType>& f_rhs);

    //-------------------------------------------------------------------------
    //! returns true for (A >= B) if TPair object A is greater than or equal to TPair object B.
    //! @relates TPair
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType>   
    bool    operator>=  (const TPair<FirstType,SecondType>& f_lhs, const TPair<FirstType,SecondType>& f_rhs);


    //=========================================================================
    //  TTriple
    //-------------------------------------------------------------------------
    //! a class that provides for the ability to treat three objects of 
    //! arbitrary types as a single object.
    //! The template class stores a triple of objects of type FirstType, 
    //! SecondType and ThirdType, respectively. 
    //! The type first_type is the same as the template parameter FirstType and 
    //! the type second_type is the same as the template parameter SecondType 
    //! and so on.
    //! Each specified type needs to supply only a default constructor, 
    //! a single-argument constructor, and a destructor. 
    //! 
    //! The most common use for a TTriple is as return types for functions that 
    //! return three values.
    //! @sa
    //! - TPair
    //! - TQuadruple
    //! @ingroup vfc_group_core_types
    //! @author zvh2hi
    //=========================================================================

    template <class FirstType, class SecondType=FirstType, class ThirdType=FirstType>
    class TTriple
    {
    public:
        typedef TTriple<FirstType,SecondType,ThirdType> tuple_type;

        typedef FirstType   first_type;
        typedef SecondType  second_type;
        typedef ThirdType   third_type;

    public:
        //! c'tor, initializes all members with their default c'tor
        TTriple (void);
        //! c'tor, initializes members with specified values
        TTriple (const first_type& f_first, const second_type& f_second, const third_type& f_third);
        //! copy constructor
        TTriple (const TTriple<FirstType,SecondType,ThirdType>& f_other); 
        //! Copy Assignment Operator
        TTriple& operator=(const TTriple& f_other);

        //! returns the first object (read-only)
        const first_type&   first    (void) const    { return m_first;}
        //! returns the second object (read-only)
        const second_type&  second   (void) const    { return m_second;}
        //! returns the third object (read-only)
        const third_type&   third    (void) const    { return m_third;}
        
        //! returns the first object (read-write)
        first_type&         first    (void)          { return m_first;}
        //! returns the second object (read-write)
        second_type&        second   (void)          { return m_second;}
        //! returns the third object (read-write)
        third_type&         third    (void)          { return m_third;}

    private:
        first_type  m_first;    //! first object
        second_type m_second;   //! second object
        third_type  m_third;    //! third object
    };

    // creation functions

    //-------------------------------------------------------------------------
    //! A template helper function used to construct objects of type TTriple<T1,T2,T3>, 
    //! where the component types are based on the data types passed as parameters.
    //! @relates TTriple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType>  
    TTriple<FirstType,SecondType,ThirdType> make_triple (const FirstType& f_first, const SecondType& f_second, const ThirdType& f_third);

    //-------------------------------------------------------------------------
    //! A template helper function used to construct objects of type TTriple<T1,T2,T3>, 
    //! where the component types are based on the data types passed as parameters.
    //! @relates TTriple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType>  
    TTriple<FirstType,SecondType,ThirdType> make_tuple  (const FirstType& f_first, const SecondType& f_second, const ThirdType& f_third);

    // template comparison operators

    //-------------------------------------------------------------------------
    //! returns true if two TTriple objects are identical.
    //! @relates TTriple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType>     
    bool    operator==  (   const TTriple<FirstType,SecondType,ThirdType>& f_lhs, 
                            const TTriple<FirstType,SecondType,ThirdType>& f_rhs);

    //-------------------------------------------------------------------------
    //! returns true if two TTriple objects are not identical.
    //! @relates TTriple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType>  
    bool    operator!=  (   const TTriple<FirstType,SecondType,ThirdType>& f_lhs, 
                            const TTriple<FirstType,SecondType,ThirdType>& f_rhs);

    //-------------------------------------------------------------------------
    //! returns true for (A < B) if TTriple object A is less than TTriple object B.
    //! @relates TTriple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType>  
    bool    operator<   (   const TTriple<FirstType,SecondType,ThirdType>& f_lhs, 
                            const TTriple<FirstType,SecondType,ThirdType>& f_rhs);

    //-------------------------------------------------------------------------
    //! returns true for (A > B) if TTriple object A is greater than TTriple object B.
    //! @relates TTriple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType>  
    bool    operator>   (   const TTriple<FirstType,SecondType,ThirdType>& f_lhs, 
                            const TTriple<FirstType,SecondType,ThirdType>& f_rhs);

    //-------------------------------------------------------------------------
    //! returns true for (A <= B) if TTriple object A is less than or equal to TTriple object B.
    //! @relates TTriple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType>  
    bool    operator<=  (   const TTriple<FirstType,SecondType,ThirdType>& f_lhs, 
                            const TTriple<FirstType,SecondType,ThirdType>& f_rhs);

    //-------------------------------------------------------------------------
    //! returns true for (A >= B) if TTriple object A is greater than or equal to TTriple object B.
    //! @relates TTriple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType>  
    bool    operator>=  (   const TTriple<FirstType,SecondType,ThirdType>& f_lhs, 
                            const TTriple<FirstType,SecondType,ThirdType>& f_rhs);

    //=========================================================================
    //  TQuadruple
    //-------------------------------------------------------------------------
    //! a class that provides for the ability to treat four objects of 
    //! arbitrary types as a single object.
    //! The template class stores a quadruple of objects of type FirstType, 
    //! SecondType, ThirdType and FourthType, respectively. 
    //! The type first_type is the same as the template parameter FirstType and 
    //! the type second_type is the same as the template parameter SecondType 
    //! and so on.
    //! Each specified type needs to supply only a default constructor, 
    //! a single-argument constructor, and a destructor. 
    //!
    //! The most common use for a TQuadruple is as return types for functions that 
    //! return four values.
    //! @sa
    //! - TPair
    //! - TTriple
    //! @ingroup vfc_group_core_types
    //! @author zvh2hi
    //=========================================================================

    template <class FirstType, class SecondType=FirstType, class ThirdType=FirstType, class FourthType=FirstType>
    class TQuadruple
    {
    public:
        typedef TQuadruple<FirstType,SecondType,ThirdType,FourthType> tuple_type;

        typedef FirstType   first_type;
        typedef SecondType  second_type;
        typedef ThirdType   third_type;
        typedef FourthType  fourth_type;

    public:
        //! c'tor, initializes all members with their default c'tor
        TQuadruple (void);
        //! c'tor, initializes members with specified values
        TQuadruple (    const first_type&   f_first, const second_type&  f_second, 
                        const third_type&   f_third, const fourth_type&  f_fourth);
        //! copy constructor
        TQuadruple (const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_other);
        //! Copy Assignment Operator
        TQuadruple& operator=(const TQuadruple& f_other);

        //! returns the first object (read-only)
        const first_type&   first    (void) const    { return m_first;}
        //! returns the second object (read-only)
        const second_type&  second   (void) const    { return m_second;}
        //! returns the third object (read-only)
        const third_type&   third    (void) const    { return m_third;}
        //! returns the fourth object (read-only)
        const fourth_type&  fourth   (void) const    { return m_fourth;}
        
        //! returns the first object (read-write)
        first_type&         first    (void)          { return m_first;}
        //! returns the second object (read-write)
        second_type&        second   (void)          { return m_second;}
        //! returns the third object (read-write)
        third_type&         third    (void)          { return m_third;}
        //! returns the fourth object (read-write)
        fourth_type&        fourth   (void)          { return m_fourth;}

    private:
        first_type  m_first;    //! first object
        second_type m_second;   //! second object
        third_type  m_third;    //! third object
        fourth_type m_fourth;   //! fourth object
    };

    // creation functions

    //-------------------------------------------------------------------------
    //! A template helper function used to construct objects of type TQuadruple<T1,T2,T3,T4>, 
    //! where the component types are based on the data types passed as parameters.
    //! @relates TQuadruple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType, class FourthType> 
    TQuadruple<FirstType,SecondType,ThirdType,FourthType> make_quadruple(   const FirstType&    f_first, 
                                                                            const SecondType&   f_second, 
                                                                            const ThirdType&    f_third, 
                                                                            const FourthType&   f_fourth);

    //-------------------------------------------------------------------------
    //! A template helper function used to construct objects of type TQuadruple<T1,T2,T3,T4>, 
    //! where the component types are based on the data types passed as parameters.
    //! @relates TQuadruple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType, class FourthType>
    TQuadruple<FirstType,SecondType,ThirdType,FourthType> make_tuple    (   const FirstType&    f_first, 
                                                                            const SecondType&   f_second, 
                                                                            const ThirdType&    f_third, 
                                                                            const FourthType&   f_fourth);

    // template comparison operators
    
    //-------------------------------------------------------------------------
    //! returns true if two TQuadruple objects are identical.
    //! @relates TQuadruple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType, class FourthType>     
    bool    operator==  (   const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_lhs, 
                            const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_rhs);

    //-------------------------------------------------------------------------
    //! returns true if two TQuadruple objects are not identical.
    //! @relates TQuadruple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType, class FourthType> 
    bool    operator!=  (   const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_lhs, 
                            const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_rhs);
    
    //-------------------------------------------------------------------------
    //! returns true for (A < B) if TQuadruple object A is less than TQuadruple object B.
    //! @relates TQuadruple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType, class FourthType>
    bool    operator<   (   const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_lhs, 
                            const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_rhs);

    //-------------------------------------------------------------------------
    //! returns true for (A > B) if TQuadruple object A is greater than TQuadruple object B.
    //! @relates TQuadruple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType, class FourthType> 
    bool    operator>   (   const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_lhs, 
                            const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_rhs);

    //-------------------------------------------------------------------------
    //! returns true for (A <= B) if TQuadruple object A is less than or equal to TQuadruple object B.
    //! @relates TQuadruple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType, class FourthType> 
    bool    operator<=  (   const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_lhs, 
                            const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_rhs);
    
    //-------------------------------------------------------------------------
    //! returns true for (A >= B) if TQuadruple object A is greater than or equal to TQuadruple object B.
    //! @relates TQuadruple
    //-------------------------------------------------------------------------
    template <class FirstType, class SecondType, class ThirdType, class FourthType> 
    bool    operator>=  (   const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_lhs, 
                            const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_rhs);
    

}   // namespace vfc closed

#include "vfc/core/vfc_tuple.inl"

#endif //VFC_TUPLE_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_tuple.hpp  $
//  Revision 1.4 2016/04/12 16:37:58MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - vfc_tuple violates the rule of three: copy assignment operator is missing (mantis0005191)
// Revision 1.3 2007/08/24 13:37:26MESZ Dilip Krishna (RBEI/ESE) (dkn2kor) 
// - removed generalized copy c'tor (mantis 1702)
// Revision 1.2 2007/07/23 13:04:27IST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
// - doxygen grouping (mantis1744)
// - added documentation
// Revision 1.1 2006/06/27 17:49:15CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
// Initial revision
// Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/core.pj
//=============================================================================
