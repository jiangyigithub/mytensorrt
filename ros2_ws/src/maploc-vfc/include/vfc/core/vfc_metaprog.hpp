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
//  Department:
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_metaprog.hpp $
///     $Revision: 1.24 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/08/18 15:57:15MESZ $
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

#ifndef VFC_METAPROG_HPP_INCLUDED
#define VFC_METAPROG_HPP_INCLUDED

//vfc includes
#include "vfc/core/vfc_types.hpp"            // used for int32_t
#include "vfc/core/vfc_static_assert.hpp"    // used for VFC_STATIC_ASSERT()

namespace vfc
{    // namespace vfc opened

    //=============================================================================
    //  DOYGEN DEFGROUP vfc_group_core_generic_metaprogram BEGIN
    //-----------------------------------------------------------------------------
    /// @defgroup vfc_group_core_generic_metaprogram Template Metaprograms
    /// @ingroup vfc_group_core_generic
    /// @brief template metaprogramming with vfc.
    /// Metaprogramming consists of “programming a program.” In other words, we lay
    /// out code that the programming system executes to generate new code that
    /// implements the functionality we really want. Usually the term metaprogramming
    /// implies a reflexive attribute: The metaprogramming component is part of the
    /// program for which it generates a bit of code/program.
    ///
    /// Why would metaprogramming be desirable?
    /// As with most other programming techniques, the goal is to achieve more
    /// functionality with less effort, where effort can be measured as code size,
    /// maintenance cost, and so forth. What characterizes metaprogramming is that
    /// some user-defined computation happens at translation time.
    /// The underlying motivation is often performance (things computed at translation
    /// time can frequently be optimized away) or interface simplicity
    /// (a metapro-gram is generally shorter than what it expands to) or both.
    ///
    /// Metaprogramming often relies on the concepts of traits and type functions, see
    /// also @ref vfc_group_core_generic_typetraits. We therefore recommend getting
    /// familiar with this technique prior to delving into this one.
    ///
    /// In 1994 during a meeting of the C++ standardization committee, Erwin Unruh
    /// discovered that templates can be used to compute something at compile time.
    /// He wrote a program that produced prime numbers.
    /// The intriguing part of this exercise, however, was that the production of the
    /// prime numbers was performed by the compiler during the compilation process
    /// and not at run time. Specifically, the compiler produced a sequence of error
    /// messages with all prime numbers from two up to a certain configurable value.
    /// Although this program wasn’t strictly portable (error messages aren’t
    /// standardized), the program did show that the template instantiation mechanism
    /// is a primitive recursive language that can perform nontrivial computations at
    /// compile time. This sort of compile-time computation that occurs through
    /// template instantiation is commonly called template metaprogramming.
    /// @{
    //=============================================================================

    //=============================================================================
    //    TIf<>
    //-----------------------------------------------------------------------------
    /// Compile time if-then-else statement.
    /// @param PredicateValue predicate, evaluates to true or false
    /// @param ThenType type selected if PredicateValue evaluates to true
    /// @param ElseType type selected if PredicateValue evaluates to false
    /// @author zvh2hi
    //=============================================================================

    template <bool PredicateValue, class ThenType, class ElseType>
    struct TIf
    {
        typedef ElseType type;
    };

    /// @cond VFC_DOXY_SPECIALIZATIONS

    template <class ThenType, class ElseType>
    struct TIf< true, ThenType, ElseType>
    {
        typedef ThenType type;
    };

    /// @endcond

    //=============================================================================
    //    TEvalIf<>
    //-----------------------------------------------------------------------------
    /// Compile time if-then-else statement with extra indirection.
    /// @sa TIf
    /// @param PredicateValue predicate, evaluates to true or false
    /// @param ThenType ThenType::type is selected if PredicateValue evaluates to true
    /// @param ElseType ElseType::type is selected if PredicateValue evaluates to false
    /// @author zvh2hi
    //=============================================================================

    template <bool PredicateValue, class ThenType, class ElseType>
    struct TEvalIf
    {
        typedef typename ElseType::type type;
    };

    /// @cond VFC_DOXY_SPECIALIZATIONS

    template <class ThenType, class ElseType>
    struct TEvalIf< true, ThenType, ElseType>
    {
        typedef typename ThenType::type type;
    };

    /// @endcond

    //=============================================================================
    //    TEnableIf<>
    //-----------------------------------------------------------------------------
    /// Compile time typedef, depending on the predicate value. 
    /// Used to add or remove function overloads, e.g. depending on type traits
    /// evaluating to true or false.
    /// @param PredicateValue predicate, evaluates to true or false
    /// @param ValueType ValueType is typedefed if PredicateValue evaluates to true
    /// @author jat2hi
    //=============================================================================

    template<bool PredicateValue, class ValueType>
    struct TEnableIf
    {
    };

    /// @cond VFC_DOXY_SPECIALIZATIONS

    template<class ValueType>
    struct TEnableIf<true, ValueType>
    {
        typedef ValueType type;
    };

    /// @endcond

    //=============================================================================
    //    TRemoveCV<>
    //-----------------------------------------------------------------------------
    /// Template Metaprogram for removing const and volatile qualifiers of
    /// specified type.
    /// @param T Type whose qualifiers should be removed
    /// @author zvh2hi
    //=============================================================================

    template <class T>
    struct  TRemoveCV
    {
        typedef T type;
    };

    /// @cond VFC_DOXY_SPECIALIZATIONS

    template <class T>
    struct  TRemoveCV<T const>
    {
        typedef T type;
    };

    template <class T>
    struct  TRemoveCV<T volatile>
    {
        typedef T type;
    };

    template <class T>
    struct  TRemoveCV<T const volatile>
    {
        typedef T type;
    };

    /// @endcond


    //=============================================================================
    //    TIsSameTypeCV<>
    //-----------------------------------------------------------------------------
    /// Template Metaprogram for comparing two types for equality.
    /// If both types are equal TIsSameTypeCV<>::value evaluates to true.
    /// @note result depends on qualifiers as well, e.g.
    /// TIsSameTypeCV<const int32_t, int32_t>::value evaluates to @b false.
    /// @param T First type for comparison
    /// @param U Second type for comparison
    /// @author zvh2hi
    //=============================================================================

    template <class FirstType, class SecondType>
    struct    TIsSameTypeCV
    {
        enum { value = false };
    };

    /// @cond VFC_DOXY_SPECIALIZATIONS

    template <class SameType>
    struct TIsSameTypeCV<SameType,SameType>
    {
        enum { value = true };
    };

    /// @endcond

    //=============================================================================
    //    TIsSameType<>
    //-----------------------------------------------------------------------------
    /// Template Metaprogram for comparing two types for equality.
    /// If both types are equal TIsSameType<>::value evaluates to true.
    /// @note result @b doesn't depend on qualifiers, e.g.
    /// TIsSameType<const int32_t, int32_t>::value evaluates to @b true.
    /// @param T First type for comparison
    /// @param U Second type for comparison
    /// @author zvh2hi
    //=============================================================================

    template <class FirstType, class SecondType>
    struct    TIsSameType
    {
        enum    { value =   TIsSameTypeCV
                            <   typename TRemoveCV<FirstType>::type,
                                typename TRemoveCV<SecondType>::type
                            >::value
                };
    };

    //=============================================================================
    // TIsDerivedFrom<>
    //-----------------------------------------------------------------------------
    /// Template Metaprogram for testing if one type is derived from another.
    /// If DerivedType is derived from BaseType,
    /// TIsDerivedFrom<>::value evaluates to true.
    /// @param DerivedType
    /// @param BaseType
    /// @author muk2lr
    //=============================================================================

    template<class DerivedType, class BaseType>
    class TIsDerivedFrom
    {
    private:
        // Make sure, the two sizes are not accidentally identical, because of aligment issues
        class CYes { vfc::uint8_t m_a[1]; };
        class CNo  { vfc::uint8_t m_a[10]; };

        static CYes test( typename vfc::TRemoveCV<BaseType>::type * );
        // Deviation of coding rule 9.6,
        // We do not define the memberfunction and no code is actually called.
        static CNo  test( ... );
    public:
        enum
        {
            //PRQA S 3249 ++
            // If DerivedType* can be converted to BaseType*,
            // the first specialization of Test() is chosen
            value = (((sizeof(test(static_cast<typename vfc::TRemoveCV<DerivedType>::type *>(0))))
                    == sizeof(CYes)) ? true : false)
            //PRQA S 3249 --
        };
    };

    //=============================================================================
    //    TType2Type<>
    //-----------------------------------------------------------------------------
    /// Simple Template Metaprogram for storing specified type in typedef.
    /// TType2Type<> is used for building larger metaprograms.
    /// @sa TEvalIf
    /// @author zvh2hi
    //=============================================================================

    template <class T>
    struct TType2Type
    {
        typedef T type;
    };

    //=============================================================================
    //    TInt2Boolean<>
    //-----------------------------------------------------------------------------
    /// Simple Template Metaprogram for converting an integral value to a
    /// boolean type.
    /// @par Description:
    /// If Value equals false (zero), the typedef evaluates to false_t. All other
    /// values evaluate to true_t.
    /// @param Value integral value for evaluation
    /// @author zvh2hi
    //=============================================================================

    template <int32_t Value>
    struct TInt2Boolean
    {
        enum { value = true};
        typedef true_t    type;
    };

    /// @cond VFC_DOXY_SPECIALIZATIONS

    template <>
    struct TInt2Boolean<false>
    {
        enum { value = false};
        typedef false_t    type;
    };

    /// @endcond

    //=============================================================================
    //    TInt2Type<>
    //-----------------------------------------------------------------------------
    /// Simple Template Metaprogram for converting an integral value to a unique type.
    /// @author zvh2hi
    //=============================================================================

    template <int32_t Value>
    struct TInt2Type
    {
        enum { value = Value };
    };


    //=============================================================================
    //    TSigned2Unsigned<>
    //-----------------------------------------------------------------------------
    /// Simple Template Metaprogram for converting a signed type to its unsigned
    /// counterpart type.
    /// @author jat2hi
    //=============================================================================

    template<class ValueType>
    struct TSigned2Unsigned
    {
        typedef ValueType type;
    };

    /// @cond VFC_DOXY_SPECIALIZATIONS

    template<>
    struct TSigned2Unsigned<int8_t>
    {
        typedef uint8_t type;
    };

    template<>
    struct TSigned2Unsigned<int16_t>
    {
        typedef uint16_t type;
    };

    template<>
    struct TSigned2Unsigned<int32_t>
    {
        typedef uint32_t type;
    };

#ifndef VFC_NO_INT64
    template<>
    struct TSigned2Unsigned<int64_t>
    {
        typedef uint64_t type;
    };
#endif

    /// @endcond

    //=============================================================================
    //    TAbs<>
    //-----------------------------------------------------------------------------
    /// Template Metaprogram which calculates at compile time the absolute value
    /// corresponding to the specified integer parameter value.
    /// @author zvh2hi
    //=============================================================================

    template <int32_t Value>
    struct TAbs
    {
        enum { value = (Value < 0) ? (-Value) : Value };
    };

    //=============================================================================
    //    TMin<>
    //-----------------------------------------------------------------------------
    /// Template Metaprogram which calculates at compile time the minimum of two
    /// specified integer values.
    /// @author zvh2hi
    //=============================================================================

    template <int32_t Op1Value, int32_t Op2Value>
    struct TMin
    {
        enum { value = (Op1Value < Op2Value) ? Op1Value : Op2Value };
    };

    //=============================================================================
    //    TMin<>
    //-----------------------------------------------------------------------------
    /// Template Metaprogram which calculates at compile time the maximum of two
    /// specified integer values.
    /// @author zvh2hi
    //=============================================================================

    template <int32_t Op1Value, int32_t Op2Value>
    struct TMax
    {
        enum { value = (Op1Value > Op2Value) ? Op1Value : Op2Value };
    };

    //=============================================================================
    //    TNextPow2<>
    //-----------------------------------------------------------------------------
    /// Template Metaprogram which calculates at compile time the next
    /// power-of-two value which is greater or equal to specified integer value.
    /// @author zvh2hi
    //=============================================================================

    template <int32_t Value>
    class    TNextPow2
    {
        // check if value is not negative
        VFC_STATIC_ASSERT ( Value >= 0 );

        private:
            enum { TEMP0 = Value-1};
            enum { TEMP1 = TEMP0 | (TEMP0>>1) };
            enum { TEMP2 = TEMP1 | (TEMP1>>2) };
            enum { TEMP3 = TEMP2 | (TEMP2>>4) };
            enum { TEMP4 = TEMP3 | (TEMP3>>8) };
            enum { TEMP5 = TEMP4 | (TEMP4>>16) };
            enum { TEMP6 = TEMP5+1};
        public:
            enum { value = TEMP6 + (TEMP6==0) };
    };

    //=============================================================================
    //    TIsPow2<>
    //-----------------------------------------------------------------------------
    /// Template Metaprogram for testing specified value to be a power-of-two.
    /// @author zvh2hi
    //=============================================================================

    template <int32_t Value>
    struct TIsPow2
    {
        // check if value is not negative
        VFC_STATIC_ASSERT ( Value >= 0 );

        enum { value = ((!(Value & (Value - 1))) && Value) };
    };

    //=============================================================================
    //    TAlignUp<>
    //-----------------------------------------------------------------------------
    /// Template Metaprogram for up-aligning given integer value to a multiple of
    /// specified power-of-two value.
    /// @author zvh2hi
    //=============================================================================

    template <int32_t SizeValue, int32_t AlignValue>
    struct TAlignUp
    {
        // check if SizeValue is not negative
        VFC_STATIC_ASSERT ( SizeValue >= 0 );
        // check if alignment is pow2
        VFC_STATIC_ASSERT(TIsPow2<AlignValue>::value);
        enum    {     value = SizeValue + (AlignValue-1) & ~(AlignValue-1)};
    };

    //=============================================================================
    //    TAlignUp<>
    //-----------------------------------------------------------------------------
    /// Template Metaprogram for down-aligning given integer value to a multiple of
    /// specified power-of-two value.
    /// @author zvh2hi
    //=============================================================================

    template <size_t SizeValue, size_t AlignValue>
    struct TAlignDown
    {
        // check if SizeValue is not negative
        VFC_STATIC_ASSERT ( SizeValue >= 0 );
        // check if alignment is pow2
        VFC_STATIC_ASSERT(TIsPow2<AlignValue>::value);

        enum    {     value = SizeValue & ~(AlignValue-1)};
    };


    //=============================================================================
    //    vfc::TLog2Floor<>
    //-----------------------------------------------------------------------------
    /// Template Metaprogram for calculating the floor of log2(X) (or ld)
    /// @par Description:
    /// Calculates the floor of the logarithm dualis (log2 or ld) of specified
    /// template value at compile time. @n
    /// The result is stored in TLog2Floor<>::value
    /// @param ArgValue Value whose logarithm is to be found
    /// @author zvh2hi
    /// @relates TLog2Near
    //=============================================================================

    template <int32_t ArgValue>
    struct TLog2Floor
    {
        // check if ArgValue is not negative
        VFC_STATIC_ASSERT ( ArgValue >= 0 );

        enum
        {   /// calculation result
            value = TLog2Floor< (ArgValue >> 1) >::value + 1
        };
    };

    /// @cond VFC_DOXY_SPECIALIZATIONS

    /// specialization for ArgValue == 1 (recursion termination)
    template <>
    struct TLog2Floor<1>
    {
        enum { value = 0 };
    };

    /// specialization for undefined case ArgValue == 0 (generates compiler error)
    template <>
    struct TLog2Floor<0>
    {
    };

    /// @endcond

    //=============================================================================
    //    vfc::TLog2Near<>
    //-----------------------------------------------------------------------------
    /// Template Metaprogram for calculating the nearest integer of log2(X)
    /// @par Description:
    /// Calculates the nearest integer of the logarithm dualis (log2 or ld)
    /// of specified template value at compile time. @n
    /// The result is stored in TLog2Near<>::value
    /// @param ArgValue Value whose logarithm is to be found
    /// @par Algorithm:
    /// @code
    /// // pseudocode:
    /// y = log2(x)
    /// // going to y.1 fixpoint by transforming eq. (y.1 = 2*y)
    /// 2y  = 2 log2(x)
    /// y.1 = 2 log2(x)
    /// y.1 = log2(x^2)
    /// // go back and round
    /// nint(y) = (y.1 + 1 ) / 2
    /// @endcode
    /// @sa TLog2Floor
    /// @author zvh2hi
    //=============================================================================

    template <int32_t ArgValue>
    struct TLog2Near
    {
        // check if ArgValue is not negative
        VFC_STATIC_ASSERT ( ArgValue >= 0 );

        enum
        {   /// calculation result
            value = (TLog2Floor< (ArgValue * ArgValue) >::value + 1) >> 1
        };
    };

    //=============================================================================
    // vfc::TPower<>
    //-----------------------------------------------------------------------------
    /// Template Metaprogram for calculating the power(BaseValue,ExponentValue)
    /// @par Description:
    /// power returns "BaseValue" to the power of "ExponentValue", BaseValue and
    /// ExponentValue should be greater than Zero. Asserts if the value is less than zero.
    /// returns the template value at compile time.
    /// The result is stored in TPower<>::value
    /// @param BaseVal Basevalue of the power function.
    /// @param ExponentVal Exponentvalue of the power fucntion.
    /// @author
    //=============================================================================

    template <vfc::int32_t BaseValue, vfc::int32_t ExponentValue>
    struct TPower
    {
        VFC_STATIC_ASSERT (BaseValue > 0);
        VFC_STATIC_ASSERT (ExponentValue > 0);

        static const vfc::int32_t VALUE = BaseValue * ( TPower<BaseValue, ExponentValue-1>::VALUE ) ;

        // Asserts if value is greater than ( (2^31) -1 )
        VFC_STATIC_ASSERT (VALUE <= 2147483647LL );
    };

    template <vfc::int32_t BaseValue>
    struct TPower<BaseValue, 0>
    {
        VFC_STATIC_ASSERT (BaseValue > 0);
        static const vfc::int32_t VALUE = 1 ;
    };

    //=============================================================================
    //  DOYGEN DEFGROUP vfc_group_core_generic_metaprogram END
    //-----------------------------------------------------------------------------
    /// @}
    //=============================================================================

}    // namespace vfc closed


#endif //VFC_METAPROG_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_metaprog.hpp  $
//  Revision 1.24 2014/08/18 15:57:15MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - cutnpaste error in TIsSameTypeCV<> documentation (mantis0002454)
//  Revision 1.23 2014/03/27 13:05:52MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Move TPower implementation from siunits to metaprog (mantis0003445)
//  Revision 1.22 2014/03/27 11:02:07MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Add TEnableIf implementation to vfc_metaprog (mantis0004457)
//  Revision 1.21 2012/12/18 08:27:31MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.20 2012/12/17 13:22:12MEZ Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  - add signed to unsigned type promotion (mantis 0004196)
//  Revision 1.19 2009/02/11 14:01:59MEZ Gaurav Jain (RBEI/ESB3) (gaj2kor) 
//  -Removal of QAC++ warnings. (mantis2445)
//  Revision 1.18 2009/02/06 13:31:29IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Removal of QAC++ warnings.
//  (Mantis : 0002561)
//  Revision 1.17 2007/07/23 13:02:09IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - doxygen grouping (mantis1744)
//  - added documentation
//  Revision 1.16 2007/07/17 13:07:38CEST Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  - added TIsDerivedFrom<> according to proposal (mantis1688)
//  Revision 1.15 2006/12/19 11:44:03CET Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  removed doxygen warning (mantis 1344)
//  Revision 1.14 2006/11/24 09:31:15CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added TRemoveCV<> metaprog (mantis1322)
//  - changed TIsSameType<> metaprog (mantis1321)
//  - added TIsSameTypeCV<> metaprog (mantis1321)
//  - replaced size_t by int32_t and added static asserts (mantis1324)
//  - removed unecessary include (mantis1323)
//  - added docu
//  - renamed identifiers
//  Revision 1.13 2006/11/16 14:41:21CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.12 2006/10/13 12:41:30CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added metaprograms for calculating of algorithm dualis (mantis1204)
//  - changed header/footer templates
//  Revision 1.11 2006/05/12 13:57:53CEST Alaa El-Din Omar (AE-DA/ESA3-Hi) (alo2hi)
//  - corrected if in template TAbs
//  Revision 1.10 2006/05/09 09:50:25CEST Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  added TAbs
//  Revision 1.9 2006/03/02 11:30:18CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added TEvalIf<> and TType2Type<> metaprogramming structs
//  Revision 1.8 2005/11/16 15:56:55CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -added correct next pow2 calculation for zero
//  Revision 1.7 2005/11/16 15:37:32CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -changed TNextPow2<> implementation
//  Revision 1.6 2005/11/16 13:45:41CET Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  -changed TIsPow2<> metaprog
//  Revision 1.4 2005/10/06 16:53:54CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/vfc.pj
//  Revision 1.3 2005/04/20 14:18:46CEST zvh2hi
//  added TIf<> and TIsSameType<> structs
//  Revision 1.2 2005/04/18 17:14:09CEST zvh2hi
//  moved TInt2Boolean<> from vfc_type_traits.hpp to vfc_metaprog.hpp
//  Revision 1.1 2005/04/18 11:58:30CEST zvh2hi
//  Initial revision
//  Member added to project /import/mks/data/projects/cv/vfc/include/vfc/vfc.pj
//=============================================================================
