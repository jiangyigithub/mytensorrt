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
///     $Source: vfc_tuple.inl $
///     $Revision: 1.6 $
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

///////////////////////////////////////////////////////////////////////////////
// TPair
///////////////////////////////////////////////////////////////////////////////

template <class FirstType, class SecondType>    inline
vfc::TPair<FirstType,SecondType>::TPair (void)
:   m_first (first_type()),
    m_second(second_type())
{
    // intentionally left blank
}

template <class FirstType, class SecondType>    inline
vfc::TPair<FirstType,SecondType>::TPair (const FirstType& f_first, const SecondType& f_second)
:   m_first (f_first),
    m_second(f_second)
{
    // intentionally left blank
}

template <class FirstType, class SecondType>    inline
vfc::TPair<FirstType,SecondType>::TPair(const TPair<FirstType,SecondType>& f_other)
:   m_first (f_other.first()),
    m_second(f_other.second())
{}

// To be conformal with the std::pair implementation, the test for self-assignment is skipped
// PRQA S 4072 ++
template <class FirstType, class SecondType>    inline
vfc::TPair<FirstType,SecondType>& vfc::TPair<FirstType,SecondType>::operator=(const TPair<FirstType,SecondType>& f_other)
{
    m_first  = f_other.first();
    m_second = f_other.second();
    return *this;
}
// PRQA S 4072 --

template <class FirstType, class SecondType>    inline
vfc::TPair<FirstType,SecondType> vfc::make_pair(const FirstType& f_first, const SecondType& f_second)
{
    return TPair<FirstType,SecondType>(f_first,f_second);
}

template <class FirstType, class SecondType>    inline
vfc::TPair<FirstType,SecondType> vfc::make_tuple(const FirstType& f_first, const SecondType& f_second)
{
    return TPair<FirstType,SecondType>(f_first,f_second);
}

template <class FirstType, class SecondType>    inline
bool    vfc::operator==  (const TPair<FirstType,SecondType>& f_lhs, const TPair<FirstType,SecondType>& f_rhs)
{
    return (    (f_lhs.first()  ==  f_rhs.first())
            &&  (f_lhs.second() ==  f_rhs.second()));
}

template <class FirstType, class SecondType>    inline
bool    vfc::operator!=  (const TPair<FirstType,SecondType>& f_lhs, const TPair<FirstType,SecondType>& f_rhs)
{
    return (!(f_lhs == f_rhs));
}

template <class FirstType, class SecondType>    inline
bool    vfc::operator<  (const TPair<FirstType,SecondType>& f_lhs, const TPair<FirstType,SecondType>& f_rhs)
{
    //the return value is true if the first element of x is less than the first element of y, 
    //and false if the first element of y is less than the first element of x. 
    //If neither of these is the case, then operator< returns the result of comparing the second elements of x and y. This operator may only be used if both T1 and T2 are LessThanComparable
    return ((f_lhs.first() < f_rhs.first()) || 
        ((!(f_rhs.first() < f_lhs.first())) && (f_lhs.second() < f_rhs.second())));
}

template <class FirstType, class SecondType>    inline
bool    vfc::operator>  (const TPair<FirstType,SecondType>& f_lhs, const TPair<FirstType,SecondType>& f_rhs)
{
    return (f_rhs < f_lhs);
}

template <class FirstType, class SecondType>    inline
bool    vfc::operator<=  (const TPair<FirstType,SecondType>& f_lhs, const TPair<FirstType,SecondType>& f_rhs)
{
    return (!(f_rhs < f_lhs));
}

template <class FirstType, class SecondType>    inline
bool    vfc::operator>=  (const TPair<FirstType,SecondType>& f_lhs, const TPair<FirstType,SecondType>& f_rhs)
{
    return (!(f_lhs < f_rhs));
}

///////////////////////////////////////////////////////////////////////////////
// TTriple
///////////////////////////////////////////////////////////////////////////////

template <class FirstType, class SecondType, class ThirdType>   inline
vfc::TTriple<FirstType,SecondType,ThirdType>::TTriple (void)
:   m_first(FirstType()), m_second(SecondType()), m_third(ThirdType())
{
    // intentionally left blank
}

template <class FirstType, class SecondType, class ThirdType>   inline
vfc::TTriple<FirstType,SecondType,ThirdType>::TTriple (const FirstType& f_first, const SecondType& f_second, const ThirdType& f_third)
:   m_first(f_first), m_second(f_second), m_third(f_third)
{
    // intentionally left blank
}

template <class FirstType, class SecondType, class ThirdType>   inline
vfc::TTriple<FirstType,SecondType,ThirdType>::TTriple (const TTriple<FirstType,SecondType,ThirdType>& f_other)
:   m_first (f_other.first()),
    m_second(f_other.second()),
    m_third (f_other.third())
{
    // intentionally left blank
}

// To be conformal with the std::pair implementation, the test for self-assignment is skipped
// PRQA S 4072 ++
template <class FirstType, class SecondType, class ThirdType>    inline
vfc::TTriple<FirstType,SecondType,ThirdType>& vfc::TTriple<FirstType,SecondType,ThirdType>::operator=(const TTriple<FirstType,SecondType,ThirdType>& f_other)
{
    m_first  = f_other.first();
    m_second = f_other.second();
    m_third = f_other.third();
    return *this;
}
// PRQA S 4072 --

template <class FirstType, class SecondType, class ThirdType>   inline
vfc::TTriple<FirstType,SecondType,ThirdType> vfc::make_triple(const FirstType& f_first, const SecondType& f_second, const ThirdType& f_third)
{
    return TTriple<FirstType,SecondType,ThirdType>(f_first,f_second,f_third);
}

template <class FirstType, class SecondType, class ThirdType>   inline
vfc::TTriple<FirstType,SecondType,ThirdType> vfc::make_tuple(const FirstType& f_first, const SecondType& f_second, const ThirdType& f_third)
{
    return TTriple<FirstType,SecondType,ThirdType>(f_first,f_second,f_third);
}

template <class FirstType, class SecondType, class ThirdType>  inline
bool    vfc::operator==  (  const TTriple<FirstType,SecondType,ThirdType>& f_lhs,
                            const TTriple<FirstType,SecondType,ThirdType>& f_rhs)
{
    return (    (f_lhs.first()  ==  f_rhs.first())
            &&  (f_lhs.second() ==  f_rhs.second())
            &&  (f_lhs.third()  ==  f_rhs.third()));
}

template <class FirstType, class SecondType, class ThirdType>  inline
bool    vfc::operator!=  (  const TTriple<FirstType,SecondType,ThirdType>& f_lhs,
                            const TTriple<FirstType,SecondType,ThirdType>& f_rhs)
{
    return (!(f_lhs == f_rhs));
}

template <class FirstType, class SecondType, class ThirdType>  inline
bool    vfc::operator<   (  const TTriple<FirstType,SecondType,ThirdType>& f_lhs,
                            const TTriple<FirstType,SecondType,ThirdType>& f_rhs)
{
    if ( f_lhs.first() == f_rhs.first() )
    {
        if ( f_lhs.second() == f_rhs.second() )
        {
            return (f_lhs.third() < f_rhs.third());
        }
        else
        {
            return (f_lhs.second() < f_rhs.second());
        }
    }
    else
    {
        return (f_lhs.first() < f_rhs.first());
    }
}

template <class FirstType, class SecondType, class ThirdType>  inline
bool    vfc::operator>   (  const TTriple<FirstType,SecondType,ThirdType>& f_lhs,
                            const TTriple<FirstType,SecondType,ThirdType>& f_rhs)
{
    return (f_rhs < f_lhs);
}

template <class FirstType, class SecondType, class ThirdType>  inline
bool    vfc::operator<=  (  const TTriple<FirstType,SecondType,ThirdType>& f_lhs,
                            const TTriple<FirstType,SecondType,ThirdType>& f_rhs)
{
    return (!(f_rhs < f_lhs));
}

template <class FirstType, class SecondType, class ThirdType>  inline
bool    vfc::operator>=  (  const TTriple<FirstType,SecondType,ThirdType>& f_lhs,
                            const TTriple<FirstType,SecondType,ThirdType>& f_rhs)
{
    return (!(f_lhs < f_rhs));
}


///////////////////////////////////////////////////////////////////////////////
// TQuadruple
///////////////////////////////////////////////////////////////////////////////

template <class FirstType, class SecondType, class ThirdType, class FourthType> inline
vfc::TQuadruple<FirstType,SecondType,ThirdType,FourthType>::TQuadruple (void)
:   m_first(FirstType()), m_second(SecondType()),
    m_third(ThirdType()), m_fourth(FourthType())
{
    // intentionally left blank
}

template <class FirstType, class SecondType, class ThirdType, class FourthType> inline
vfc::TQuadruple<FirstType,SecondType,ThirdType,FourthType>::TQuadruple
(   const FirstType&   f_first,    const SecondType&  f_second,
    const ThirdType&   f_third,    const FourthType&  f_fourth)
:   m_first(f_first), m_second(f_second),
    m_third(f_third), m_fourth(f_fourth)
{
    // intentionally left blank
}

template <class FirstType, class SecondType, class ThirdType, class FourthType> inline
vfc::TQuadruple<FirstType,SecondType,ThirdType,FourthType> :: TQuadruple(const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_other)
    :   m_first (f_other.first()),
    m_second(f_other.second()),
    m_third (f_other.third()),
    m_fourth(f_other.fourth())
{
    // intentionally left blank
}

// To be conformal with the std::pair implementation, the test for self-assignment is skipped
// PRQA S 4072 ++
template <class FirstType, class SecondType, class ThirdType, class FourthType>    inline
vfc::TQuadruple<FirstType,SecondType,ThirdType,FourthType>&
vfc::TQuadruple<FirstType,SecondType,ThirdType,FourthType>::operator=(const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_other)
{

    m_first  = f_other.first();
    m_second = f_other.second();
    m_third = f_other.third();
    m_fourth = f_other.fourth();
    return *this;
}
// PRQA S 4072 --

template <class FirstType, class SecondType, class ThirdType, class FourthType> inline
vfc::TQuadruple<FirstType,SecondType,ThirdType,FourthType> vfc::make_quadruple(const FirstType& f_first, const SecondType& f_second, const ThirdType& f_third, const FourthType& f_fourth)
{
    return TQuadruple<FirstType,SecondType,ThirdType,FourthType>(f_first,f_second,f_third,f_fourth);
}

template <class FirstType, class SecondType, class ThirdType, class FourthType> inline
vfc::TQuadruple<FirstType,SecondType,ThirdType,FourthType> vfc::make_tuple(const FirstType& f_first, const SecondType& f_second, const ThirdType& f_third, const FourthType& f_fourth)
{
    return TQuadruple<FirstType,SecondType,ThirdType,FourthType>(f_first,f_second,f_third,f_fourth);
}

template <class FirstType, class SecondType, class ThirdType, class FourthType>  inline
bool    vfc::operator==  (  const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_lhs,
                            const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_rhs)
{
    return (    (f_lhs.first()  ==  f_rhs.first())
            &&  (f_lhs.second() ==  f_rhs.second())
            &&  (f_lhs.third()  ==  f_rhs.third())
            &&  (f_lhs.fourth() ==  f_rhs.fourth()));

}

template <class FirstType, class SecondType, class ThirdType, class FourthType> inline
bool    vfc::operator!=  (  const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_lhs,
                            const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_rhs)
{
    return (!(f_lhs == f_rhs));
}

template <class FirstType, class SecondType, class ThirdType, class FourthType> inline
bool    vfc::operator<   (  const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_lhs,
                            const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_rhs)
{
    if ( f_lhs.first() == f_rhs.first() )
    {
        if ( f_lhs.second() == f_rhs.second() )
        {
            if (f_lhs.third() == f_rhs.third() )
            {
                return (f_lhs.fourth() < f_rhs.fourth());
            }
            else
            {
                return (f_lhs.third() < f_rhs.third());
            }
        }
        else
        {
            return (f_lhs.second() < f_rhs.second());
        }
    }
    else
    {
        return (f_lhs.first() < f_rhs.first());
    }
}

template <class FirstType, class SecondType, class ThirdType, class FourthType> inline
bool    vfc::operator>   (  const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_lhs,
                            const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_rhs)
{
    return (f_rhs < f_lhs);
}

template <class FirstType, class SecondType, class ThirdType, class FourthType> inline
bool    vfc::operator<=  (  const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_lhs,
                            const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_rhs)
{
    return (!(f_rhs < f_lhs));
}

template <class FirstType, class SecondType, class ThirdType, class FourthType> inline
bool    vfc::operator>=  (  const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_lhs,
                            const TQuadruple<FirstType,SecondType,ThirdType,FourthType>& f_rhs)
{
    return (!(f_lhs < f_rhs));
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_tuple.inl  $
//  Revision 1.6 2016/04/12 16:37:58MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - vfc_tuple violates the rule of three: copy assignment operator is missing (mantis0005191)
//  Revision 1.5 2008/09/01 16:01:54MESZ Dhananjay N (RBEI/ESD1) (dhn1kor) 
//  Precedence confusion in QAC++ 2.5.Warning Rule 8.0.3 is removed.(mantis2220)
//  Revision 1.4 2007/08/24 17:12:10IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  - removed generalized copy c'tor and missing parenthesis added (mantis 1701, 1702)
//  Revision 1.3 2007/05/08 13:30:04IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added header / footer template (mantis1631)
//=============================================================================

