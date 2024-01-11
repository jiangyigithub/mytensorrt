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
//       Compiler(s): VS7.1, VS8.0, CW9.3, CWembedded8.5
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
///     $Source: include/vfc/core/vfc_fixedpoint.hpp $
///     $Revision: 1.81 $
///     $Author: Muehlmann Karsten (CC/PJ-FA1) (MUK2LR) $
///     $Date: 2011/05/18 17:35:32MESZ $
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

#ifndef VFC_FIXEDPOINT_HPP_INCLUDED
#define VFC_FIXEDPOINT_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_type_traits.hpp"
#include "vfc/core/vfc_math.hpp"
#include "vfc/core/vfc_metaprog.hpp"
#include "vfc/core/vfc_assert.hpp"
#include "vfc/core/vfc_static_assert.hpp"
#include "vfc/core/vfc_assert.hpp"
#include "vfc/core/vfc_type_traits.hpp"
//#include "vfc/core/vfc_int.hpp"
#include <limits>


namespace vfc
{   // namespace vfc opened

    // type promotion trait for getting next bigger int type
    template <class T> struct TFixedPointTypePromotionTrait                 { typedef T type; };
    template <>        struct TFixedPointTypePromotionTrait< vfc::int8_t >  { typedef vfc::int16_t type; };
    template <>        struct TFixedPointTypePromotionTrait< vfc::int16_t > { typedef vfc::int32_t type; };

    #ifndef    VFC_NO_INT64
      // typedef    vfc::TInt<128, vfc::int64_t, vfc::uint64_t> int128_t;
        template <>        struct TFixedPointTypePromotionTrait< vfc::int32_t > { typedef vfc::int64_t type; };
        //template <>        struct TFixedPointTypePromotionTrait< vfc::int64_t > { typedef vfc::int128_t type; };
        template <>        struct TFixedPointTypePromotionTrait< vfc::int64_t > { typedef vfc::int64_t type; };
    #else
        template <>        struct TFixedPointTypePromotionTrait< vfc::int32_t > { typedef vfc::int32_t type; };
    #endif

    // no default operation type

    template <class T> struct TLUTOperationType { typedef T type; };
    template <>        struct TLUTOperationType<vfc::int32_t> { typedef vfc::int64_t type; };

    #ifndef    VFC_NO_INT64
        template <>        struct TLUTOperationType<vfc::int64_t> { typedef vfc::int64_t type; };
    #endif

    namespace intern
    {
        /// helper class
        template <bool IsLeftShiftValue, vfc::int32_t ShiftValue>
        struct TDirectionalShift
        {
            VFC_STATIC_ASSERT(ShiftValue >= 0);

            template <class ValueType>
            static inline ValueType shift (const ValueType& f_value);
        };

        /// helper class - partial specialization
        template <vfc::int32_t ShiftValue>
        struct TDirectionalShift<false, ShiftValue>
        {
            VFC_STATIC_ASSERT(ShiftValue >= 0);

            template <class ValueType>
            static inline ValueType shift (const ValueType& f_value);
        };


        /// helper class for converting fixpoint values with different number of fractional bits
        template <vfc::int32_t SrcFracBits, vfc::int32_t DestFracBits>
        struct TConvertFracBits
        {
            VFC_STATIC_ASSERT(SrcFracBits  >= 0);
            VFC_STATIC_ASSERT(DestFracBits >= 0);

            enum  { diff = DestFracBits-SrcFracBits };
            typedef TDirectionalShift< (diff >= 0), vfc::TAbs<diff>::value >    shift_type;

            template <class ValueType>
            static inline ValueType shift (const ValueType& f_value);

        };
    }


    //============================================================================
    // TFixedPoint<>
    //----------------------------------------------------------------------------
    // default: FracBitsValue   = 14;
    //          ValueType       = vfc::int32_t;
    //      ->  maxIntValue = 131069, resolution = 6.10352E-05
    //============================================================================
    template
    <   vfc::int32_t    FracBitsValue   = 14,
        class           ValueType       = vfc::int32_t
    >
    class TFixedPoint
    {
    // typedefs and static asserts
    public:
        typedef ValueType value_type;

        typedef typename TFixedPointTypePromotionTrait<value_type>::type promotion_type;

        struct CNoShift {};

        //---------------------------------------------------
        // private constants
        //---------------------------------------------------
        // get number of valid bits without highest bit for signed types
        enum { VALIDBITS = (sizeof(value_type) * CHAR_BIT) - 1 };
        enum { SHIFT     = FracBitsValue };
        // number of integer bits
        enum { INTBITS   = VALIDBITS - SHIFT };
#ifndef VFC_NO_FLOAT
    static const vfc::float32_t int2floatMultiplier_f32;
    static const vfc::float32_t float2intMultiplier_f32;
	static const vfc::float64_t int2floatMultiplier_f64;
    static const vfc::float64_t float2intMultiplier_f64;
#endif

        static const value_type FAC;
        static const value_type MAXINTVAL;
        static const value_type ROUND;

        static const TFixedPoint zero;
        static const TFixedPoint one;

        static const TFixedPoint epsilon;
        static const TFixedPoint max;

        /// for reference see: Fixed-Point Arithmetic: An Introduction (http://www.digitalsignallabs.com/fp.pdf)
        /// e.g. fixedpoint int32 has 32 bit:
        /// 1                          for the sign
        /// N = 32 - 1 - FracBitsValue for integer bits
        /// F = FracBitsValue          for fractional bits
        /// addition                needs (N+1)   integer and (F)   fractional bits
        /// signed multiplication   needs (N+N+1) integer and (F+F) fractional bits
        /// signed division         needs (N+F+1) integer and (N+F) fractional bits

        typedef typename TLUTOperationType< value_type >::type lut_operation_type;

    private:
        //---------------------------------------------------
        // template argument requirements
        //---------------------------------------------------
        VFC_STATIC_ASSERT(vfc::TIsIntegral<value_type>::value);             // no floating point support
        VFC_STATIC_ASSERT(!(vfc::TIsUnsignedArithmetic<value_type>::value));// no unsigned types

        VFC_STATIC_ASSERT(0 < SHIFT);                                // assuring at least one bit fractional precision
        VFC_STATIC_ASSERT((sizeof(value_type) * CHAR_BIT) > SHIFT);  // assuring to have more bits than fractional bits

        typedef typename vfc::TIf< ((SHIFT + SHIFT) >= VALIDBITS),
                                   promotion_type,
                                   value_type
                                 >::type mult_else_type;
        typedef typename vfc::TIf< ((INTBITS + INTBITS + 1) >= VALIDBITS),
                                   promotion_type,
                                   mult_else_type
                                 >::type mult_op_type;

        typedef typename vfc::TIf< ((INTBITS + SHIFT) >= VALIDBITS),
                                   promotion_type,
                                   value_type
                                 >::type div_else_type;
        typedef typename vfc::TIf< ((INTBITS + SHIFT + 1) >= VALIDBITS),
                                   promotion_type,
                                   div_else_type
                                 >::type div_op_type;

    // member data
    private:
        // data members
        value_type  m_fpvalue;

    // methods
    public:
        // standard c'tor
        inline TFixedPoint (void);

        // specialization for copy c'tor
        inline TFixedPoint (const TFixedPoint& f_value);

        // c'tor for fixed point representation value
        // (do not use this c'tor for implicite scaling, if you want to use substitute class for floating point)
        inline TFixedPoint (const value_type&   f_value_fp, CNoShift );

        // copy constructor without shifting, if different type, (implicite scaling!)
        template <vfc::int32_t OtherFracBitsValue, class OtherTypeValue>
        explicit inline TFixedPoint (const TFixedPoint<OtherFracBitsValue, OtherTypeValue>& f_value, CNoShift);

        // constructor for vfc_types (either integral or floating)
        template <typename InputType>
        explicit inline TFixedPoint(const InputType& f_value);

        // conversion c'tor
        template <vfc::int32_t OtherFracBitsValue, class OtherTypeValue>
        explicit inline TFixedPoint (const TFixedPoint<OtherFracBitsValue, OtherTypeValue>& f_value);

        // there is no check if the set value is correct !!!
        inline void setFixedpoint(const value_type& f_value);

        inline bool isAdditionSafe(const TFixedPoint& f_rhs) const;

        inline bool isMultiplicationSafe(const TFixedPoint& f_rhs) const;

        template <vfc::int32_t otherFracBitsValue, class otherValueType>
        inline bool isDivisionSafe(const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs) const;

        inline bool isLeftShiftSafe(const vfc::int32_t f_shift_i32) const;

#ifndef    VFC_NO_FLOAT
        inline vfc::float32_t  to_float32    (void) const;

        inline vfc::float64_t  to_float64    (void) const;
#endif

        /// casts fixpoint to integer
        inline vfc::int32_t    to_int      (void) const;

        /// casts fixpoint to integer (round down)
        inline vfc::int32_t    to_floor    (void) const;

        /// casts fixpoint to integer (round up)
        inline vfc::int32_t    to_ceil     (void) const;

        /// rounds fixpoint to nearest integer
        inline vfc::int32_t    to_nint     (void) const;

        /// returns raw fixpoint value, do not use, if you also want to work with substitute class
        inline value_type      fixedpoint    (void) const;

        /// assignment op
        inline TFixedPoint& operator=  (const TFixedPoint& f_rhs);

        inline TFixedPoint& operator+= (const TFixedPoint& f_rhs);

        inline TFixedPoint& operator-= (const TFixedPoint& f_rhs);

        inline TFixedPoint& operator/= (const TFixedPoint& f_rhs);

        inline TFixedPoint& operator*= (const TFixedPoint& f_rhs);


        //---------------------------------------------------
        // unary arithmetic operators +, -, *, /
        //---------------------------------------------------

        inline TFixedPoint     operator-   (void) const;

        // right shift for scaling
        inline const TFixedPoint    operator>>  (const vfc::int32_t f_shift_i32) const;

        // left shift for scaling
        inline const TFixedPoint    operator<<  (const vfc::int32_t f_shift_i32) const;

        /// pre-increment
        inline TFixedPoint&    operator++  (void);

        /// post-increment
        // (Msg Disable 2427 : Direct use of fundamental type.)
        // PRQA S 2427 ++
        inline TFixedPoint     operator++  (int);
        // PRQA S 2427 --
        // (Msg Enable 2427 : Direct use of fundamental type.)

        /// pre-decrement
        inline TFixedPoint&    operator--  (void);

        /// post-decrement
        // (Msg Disable 2427 : Direct use of fundamental type.)
        // PRQA S 2427 ++
        inline TFixedPoint     operator--  (int);
        // PRQA S 2427 --
        // (Msg Enable 2427 : Direct use of fundamental type.)

        template<vfc::int32_t otherFracBitsValue, class otherValueType>
        inline const TFixedPoint operator* (const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs) const;

        template<vfc::int32_t otherFracBitsValue, class otherValueType>
        inline const TFixedPoint operator/ (const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs) const;

        template<vfc::int32_t otherFracBitsValue, class otherValueType>
        inline const TFixedPoint operator+ (const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs) const;

        template<vfc::int32_t otherFracBitsValue, class otherValueType>
        inline const TFixedPoint operator- (const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs) const;
        //---------------------------------------------------
        // comparison operators ==, !=, <, >, <= and >=
        //---------------------------------------------------
        inline bool    operator == (const TFixedPoint& f_rhs) const;

        inline bool    operator != (const TFixedPoint& f_rhs) const;

        inline bool    operator < (const TFixedPoint& f_rhs) const;

        inline bool    operator <= (const TFixedPoint& f_rhs) const;

        inline bool    operator > (const TFixedPoint& f_rhs) const;

        inline bool    operator >= (const TFixedPoint& f_rhs) const;

    private:
        //! init integral types
        template <typename InputType>
        inline void init(const InputType& f_value, vfc::true_t);

#ifndef    VFC_NO_FLOAT
        //! init for float64 ieee754 double precision
        inline void init(const vfc::float64_t& f_value_f64, vfc::false_t);

        //! init for float64 ieee754 double precision
        inline void init(const vfc::float32_t& f_value_f32, vfc::false_t);
#else
        template <typename InputType>
		inline void init(const InputType& f_value, vfc::false_t);
#endif

        //! init for fixedpoint type with another fracbits count
        template <vfc::int32_t OtherFracBitsValue, class OtherTypeValue>
        inline void init(const TFixedPoint<OtherFracBitsValue, OtherTypeValue>& f_value);

        //---------------------------------------------------
        // binary arithmetic operators +, -, *, /
        //---------------------------------------------------
        static inline TFixedPoint add (const TFixedPoint& f_op1, const TFixedPoint& f_op2);

        static inline TFixedPoint subtract (const TFixedPoint& f_op1, const TFixedPoint& f_op2);

        //---------------------------------------------------
        // binary arithmetic operators +, -, *, /  (different types of fractional bits)
        //---------------------------------------------------

        inline TFixedPoint switch_multiply(const TFixedPoint& f_rhs, vfc::true_t) const;

        template<vfc::int32_t otherFracBitsValue, class otherValueType>
        inline TFixedPoint switch_multiply(const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs,
                                           vfc::false_t) const;

        inline TFixedPoint switch_divide(const TFixedPoint& f_rhs, vfc::true_t) const;

        template<vfc::int32_t otherFracBitsValue, class otherValueType>
        inline TFixedPoint switch_divide(const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs,
                                         vfc::false_t) const;

        inline TFixedPoint switch_add(const TFixedPoint& f_rhs, vfc::true_t) const;

        template<vfc::int32_t otherFracBitsValue, class otherValueType>
        inline TFixedPoint switch_add(const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs,
                                      vfc::false_t) const;

        inline TFixedPoint switch_subtract(const TFixedPoint& f_rhs, vfc::true_t) const;

        template<vfc::int32_t otherFracBitsValue, class otherValueType>
        inline TFixedPoint switch_subtract(const TFixedPoint<otherFracBitsValue, otherValueType>& f_rhs,
                                           vfc::false_t) const;
    }; // class TFixedPoint<...>

    template<vfc::int32_t FracBitsValue, class ValueType>
    const typename vfc::TFixedPoint<FracBitsValue, ValueType>::value_type
        vfc::TFixedPoint<FracBitsValue, ValueType>::FAC =
            static_cast<typename vfc::TFixedPoint<FracBitsValue, ValueType>::value_type>(1) <<
                static_cast<vfc::int32_t>(vfc::TFixedPoint<FracBitsValue, ValueType>::SHIFT);

    template<vfc::int32_t FracBitsValue, class ValueType>
    const typename vfc::TFixedPoint<FracBitsValue, ValueType>::value_type
        vfc::TFixedPoint<FracBitsValue, ValueType>::MAXINTVAL  =
            ((static_cast<typename vfc::TFixedPoint<FracBitsValue, ValueType>::value_type>(1) <<
                static_cast<vfc::int32_t>(vfc::TFixedPoint<FracBitsValue, ValueType>::INTBITS)) -
                    static_cast<typename vfc::TFixedPoint<FracBitsValue, ValueType>::value_type>(1));

    template<vfc::int32_t FracBitsValue, class ValueType>
    const typename vfc::TFixedPoint<FracBitsValue, ValueType>::value_type
        vfc::TFixedPoint<FracBitsValue, ValueType>::ROUND =
        (static_cast<typename vfc::TFixedPoint<FracBitsValue, ValueType>::value_type>(1) <<
                (static_cast<vfc::int32_t>(vfc::TFixedPoint<FracBitsValue, ValueType>::SHIFT - 1)));

    //TIsFundamental specilization to ensure TFixedPoint as a fundamental type based on the ValueType
    template<vfc::int32_t FracBitsValue, class ValueType>
    struct TIsFundamental<vfc::TFixedPoint<FracBitsValue, ValueType> >
    {
        enum { value = TIsFundamental<ValueType>::value };
    };

    //! Adds f_a and f_b and places sum in f_result.
    //! \return true if result did overflow, false if not.
    template<vfc::int32_t FracBitsValue, class ValueType>
    inline
    bool fixedPointSumOverflowed(
        const vfc::TFixedPoint<FracBitsValue, ValueType>& f_a, //!< 1st input
        const vfc::TFixedPoint<FracBitsValue, ValueType>& f_b, //!< 2nd input
        vfc::TFixedPoint<FracBitsValue, ValueType>& f_sum);    //!< sum of inputs (might be overflowed)

}   // namespace vfc closed

#include "vfc/core/vfc_fixedpoint.inl"

#endif //VFC_FIXEDPOINT_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: include/vfc/core/vfc_fixedpoint.hpp  $
//  Revision 1.81 2011/05/18 17:35:32MESZ Muehlmann Karsten (CC/PJ-FA1) (MUK2LR) 
//  - faster float <-> FP (mantis3653)
//  Revision 1.80 2009/10/01 12:21:08CEST Gaurav Jain (RBEI/EAS3) (gaj2kor) 
//  - Clean separation of fpu-code and fpu-free code using VFC_NO_FLOAT.
//  Revision 1.79 2009/06/16 01:11:04GMT+05:30 Muehlmann Karsten (CC-DA/ESV2) (MUK2LR)
//  - addition overflow handling (mantis2872)
//  Revision 1.78 2009/03/25 15:59:48CET Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  - Linebreak at 120 in all fixedpoint files. (mantis 0002692)
//  Revision 1.77 2009/01/31 13:00:54IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Removal of QAC++ warnings.
//  (Mantis : 0002497)
//  Revision 1.76 2009/01/30 11:02:36IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Removed one of promotion_type.
//  (Mantis : 0002484)
//  Revision 1.75 2008/11/11 21:06:56IST Voelz Henning (CC-DA/ESV4) (VOH2HI)
//  setFixedpoint method added (manits:2414)
//  Revision 1.74 2008/09/30 16:53:37CEST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  -Addition of fundamental check, on ValueType passed to class.
//  (Mantis:2297)
//  Revision 1.73 2008/09/11 14:15:26IST Voelz Henning (CC-DA/ESV1) (VOH2HI)
//  remerge and close branch, because no longer  int64 with long_int supported
//  Revision 1.72.1.2 2008/08/12 10:29:47CEST Voelz Henning (CC-DA/ESV1) (VOH2HI)
//  one and zero as constant added
//  Revision 1.72.1.1 2008/03/28 16:58:56CET Voelz Henning (AE-DA/ESV1) (VOH2HI)
//  long int free variant
//  Revision 1.72 2008/03/25 08:52:14CET Voelz Henning (AE-DA/ESV1) (VOH2HI)
//  re-added long-int support
//  Revision 1.71 2008/03/25 08:48:51CET Voelz Henning (AE-DA/ESV1) (VOH2HI)
//  long_int128 removed for checkpoint
//  Revision 1.69 2008/03/12 11:02:52CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (SF71LR)
//  promotion_type and lut_operation_type are now public in fixedpoint. lut algorithm are now using the lut_operation_type which is 64bit for 32bit and 64bit fixedpoint. reworked atan so it can handle fixedpoint types with less fraction bits.
//  Revision 1.68 2008/02/28 09:21:18CET Vinaykumar Setty (RBEI/EAE6) (vmr1kor)
//  qac++,codecoverage testing done
//  Revision 1.67 2008/02/20 11:10:18IST Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  TFixedpointOperationPrecision,TFixedpointOperationSize classes are removed.
//  Removal od OpearationPolicy from TFixedPoint class
//  made Fixedpoint as Fundamental type.
//  Removed the warnings.
//  Revision 1.66 2008/02/13 14:27:06IST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Added init function for copy constructor with other fracbits. Now MW CW compiles.
//  Revision 1.65 2008/02/06 11:20:44CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  Added promotion_type typedef to fixedpoint. The promotion_type is defined by the TFixedPointTypePromotionTrait template class.
//  The promotion_type is used to do calculation with a higher fracbits count than the original fixedpoint. Changed the lut_math to use the promotion_type.
//  Revision 1.64 2008/01/30 18:55:29CET EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESV1) (sf71lr)
//  FixedPoint with 64bit valuetype should work now. When converting a 32bit fixedpoint into a 64bit fixedpoint MW CW 9.3 fails to compile.
//  Hack: always use 64bit as value_type if shifting.
//  Revision 1.63 2008/01/30 12:00:02CET Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  gcc compilation error fixed
//  Revision 1.62 2008/01/28 20:36:03IST Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  Added Promotion type
//  Revision 1.61 2007/10/24 12:39:49IST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  - obsolete methods removed
//  - cosmetics
//  Revision 1.60 2007/10/22 11:11:09CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  gcc errors fixed
//  Revision 1.59 2007/10/22 08:33:28CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  methods inlined
//  Revision 1.58 2007/10/17 16:45:57CEST vmr1kor
//  numeric_cast function added
//  Revision 1.57 2007/10/17 15:20:30IST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  TFixedPoint operations now using vfc_taylorpoly.hpp. Code from vfc::atan2 should be moved there too.
//  Revision 1.55 2007/10/16 13:49:27CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  Moved function definitions into inl.
//  Revision 1.54 2007/10/15 17:46:39CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  bugfix isDivisionSafe()
//  Revision 1.53 2007/10/15 17:30:37CEST Renner Christian (AE-DA/ESV1) (rec1lr)
//  renamed isDivisionSafe()
//  Revision 1.52 2007/10/15 16:57:34CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  implementations moved to inl file
//  Revision 1.51 2007/10/15 10:21:15CEST vmr1kor
//  to_float32() , to_float64() Definition changed
//  Revision 1.50 2007/10/12 17:14:16IST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  bugfix isMultiplicationSafe() access to fixedpoint()
//  Revision 1.49 2007/10/10 10:17:32CEST Renner Christian (AE-DA/ESV1) (rec1lr)
//  added isLeftShiftSafe()
//  Revision 1.48 2007/10/09 09:39:35CEST Renner Christian (AE-DA/ESV1) (rec1lr)
//  added isAdditionSafe and isMultiplicationSafe()
//  Revision 1.47 2007/10/08 14:08:38CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  const correctness isDevisionSafe()
//  Revision 1.46 2007/10/04 16:38:37CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  compiler warnings removed in case of NDEBUG
//  Revision 1.45 2007/10/04 10:33:09CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  gcc compiler warning removed
//  Revision 1.44 2007/10/04 08:45:55CEST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  - c++ conformance changes
//  Revision 1.43 2007/10/02 14:14:53CEST Renner Christian (AE-DA/ESV1) (rec1lr)
//  bugfix
//  Revision 1.42 2007/10/02 10:48:04CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  is_null removed use vfc::is_zero instead.
//  Revision 1.41 2007/10/02 08:48:44CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  Fixed devide operator (VFC_REQUIRE).
//  Revision 1.40 2007/10/01 10:50:18CEST Voelz Henning (AE-DA/ESV1) (voh2hi)
//  bugfix devision of two different types, shift first
//  Revision 1.39 2007/09/27 17:52:16CEST Renner Christian (AE-DA/ESA3) (rec1lr)
//  bugfix divide and isDevisionSafe()
//  Revision 1.37 2007/09/27 15:52:20CEST Renner Christian (AE-DA/ESA3) (rec1lr)
//  changed divide
//  Revision 1.36 2007/09/27 11:09:42CEST Renner Christian (AE-DA/ESA3) (rec1lr)
//  added isDevisionSafe()
//  Revision 1.35 2007/09/25 13:53:38CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  vfc:: to std::numeric_limits
//  Revision 1.34 2007/09/25 13:41:33CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  std:: -> vfc::
//  Revision 1.33 2007/09/25 13:26:35CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  Reworked test +,-,*,/.
//  Revision 1.32 2007/09/25 11:31:02CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  bugfixy assertion
//  Revision 1.31 2007/09/24 18:21:14CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  bugfix assert operator shift also for negativ values
//  Revision 1.30 2007/09/24 18:15:09CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  check overflow in left shift operation
//  Revision 1.29 2007/09/24 17:07:31CEST Renner Christian (AE-DA/ESA3) (rec1lr)
//  bugfix asserts for negative values
//  Revision 1.28 2007/09/24 15:51:27CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  asserts added in operation policy
//  Revision 1.27 2007/09/24 14:50:34CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  variable max added
//  Revision 1.26 2007/09/24 10:17:56CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  operations moved to _ops file
//  Revision 1.25 2007/09/21 09:35:54CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  cosmetics
//  Revision 1.24 2007/09/20 16:27:45CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  bug fix: wrong asserts deletet
//  Revision 1.23 2007/09/20 14:22:06CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  bugfix multiplication with different frac bits
//  Revision 1.22 2007/09/20 09:11:02CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  epsilon is not a method but a static constant
//  Revision 1.21 2007/09/19 17:01:52CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  epsilon() added
//  Revision 1.20 2007/09/19 10:53:10CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  bug fix asserts
//  Revision 1.19 2007/09/17 17:36:06CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  small bugfixes, sqrt for int64
//  Revision 1.18 2007/09/17 10:15:40CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  typedefs moved to single finle
//  Revision 1.17 2007/09/07 14:04:59CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  added is_null() method
//  Revision 1.16 2007/09/07 13:51:10CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  Fixed operator+=, operator-= and we hopefully have a working constructor for vfc_types now. Furthermore added a volatile test to the CppUnit tests.
//  Revision 1.15 2007/09/04 08:17:01CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  asserts corrected; make conversion c'tor explicit
//  Revision 1.14 2007/09/03 13:43:25CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  to_fp_x<>() and to_fp_op<>() replaced by conversion c'tor due to problems with gcc compiler
//  Revision 1.13 2007/08/31 17:56:28CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  make operator>> and << const
//  Revision 1.12 2007/08/31 09:58:07CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  typedefs for standard types added
//  Revision 1.11 2007/08/31 09:13:50CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  shift operator added
//  Revision 1.10 2007/08/30 15:02:26CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  to_fp_x<> (), to_fp_op<>(), min(), max() added; friend for comparison operators added
//  Revision 1.9 2007/08/29 13:49:52CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  atan(), asin(), acos() added; no friend operators*/-+; operator*/+- added for different fracbits fixedpoints
//  Revision 1.7 2007/08/27 13:59:24CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  Fixed constructor when using volatile floating. Removed (due static_cast) warning about unsigned long long from CW 9.4.
//  Revision 1.6 2007/08/27 13:22:41CEST EXTERNAL Schick Friedrich (Werkstudent; AE-DA/ESA3) (sf71lr)
//  Corrected VFC_REQUIRE for add/subtract and added true-cases for add/subtract cppunit test.
//  Revision 1.5 2007/08/27 11:17:43CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  c'tors for volatile float added
//  Revision 1.4 2007/08/27 08:15:21CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  c'tors and initialize changed
//  Revision 1.3 2007/08/24 16:30:54CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  inline function
//  conversion from float to fixed point with bit operations
//  Revision 1.2 2007/08/22 17:38:51CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  VFC_STATIC_ASSERT added to template c'tor
//  Revision 1.1 2007/08/22 17:07:11CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_fixedpoint/vfc_fixedpoint.pj
//  Revision 1.9 2007/08/22 17:00:09CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  second walkthrough findings: template c'tor; code moved from hpp to inl; using vfc::abs(); rename TypePromtionTrait
//  Revision 1.8 2007/08/16 14:27:30CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  warnings removed if instanciated with int64_t
//  Revision 1.7 2007/08/16 09:25:23CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  first walkthrough findings:
//  no unsigned integer support; int64_t support; typetrait for promotion_type; mult_op_type and div_op_type; rename to_float(); asserts; div. smaller changes
//  Revision 1.6 2007/08/14 14:24:40CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  replace VFC_ENSURE by VFC_REQUIRE
//  tan() added
//  Revision 1.5 2007/08/13 17:12:05CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  bugfix: cw9.3 compilation std::max(a,b); a and b have to be from same type
//  Revision 1.4 2007/08/13 15:21:50CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  TFixedPointOperation policy removed
//  bugfixes detected by cpp_unit_test
//  Revision 1.3 2007/08/10 09:39:41CEST Voelz Henning (AE-DA/ESA3) (voh2hi)
//  - TFixedPointOperation Policy added
//  - "error management"
//  - uintXX support
//  Revision 1.2 2007/08/03 17:26:03CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - removed friend declaration
//  Revision 1.1 2006/12/07 09:01:21CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/zvh2hi/zvh2hi.pj
//  Revision 1.2 2006/12/01 10:37:38CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - fixed bug in overloaded operators
//=============================================================================
