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
//  Department: CR/AEM
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_algorithm2d_fixed.inl $
///     $Revision: 1.9 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2012/12/18 08:27:28MEZ $
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

#include "vfc/core/vfc_static_assert.hpp"

namespace vfc
{   // namespace vfc opened

    //-------------------------------------------------------------------------
    // conditional doxygen documentation
    //! @cond VFC_DOXY_INTERN
    //-------------------------------------------------------------------------

    namespace intern
    {   // namespace intern opened

        //=============================================================================
        //    vfc::intern::TUnroll2DRoundForward<>
        //-----------------------------------------------------------------------------
        // algorithms for two-dimensional arguments with round bracket operator ( arg(n1,n2) )
        // e.g. vfc::TMatrix3<int>   mat;
        //=============================================================================

        template <  vfc::int32_t LoopCounter1Value, vfc::int32_t LoopCounter2Value,
                    vfc::int32_t Offset1Value=0, vfc::int32_t Offset2Value=0>
        struct TUnroll2DRoundForward
        {
            // used for stopping recursion at the end of row
            enum { COUNTER1_IN_FUNC2 = (LoopCounter2Value==1) ? 0 : LoopCounter1Value};

            // used for recursion stop in pathological cases
            // e.g. TUnroll2DRoundForward<1,0,x,y>()
            enum { COUNTER1_IN_FUNC1 = (LoopCounter2Value==0) ? 0 : LoopCounter1Value};

            template <class IOArgType, class FuncType>
            static inline
            void    for_each(IOArgType& f_ioArg, FuncType& f_func)
            {
                TUnroll2DRoundForward<  COUNTER1_IN_FUNC1,
                                        LoopCounter2Value,
                                        Offset1Value,
                                        Offset2Value>::for_each_idx2( f_ioArg, f_func);

                TUnroll2DRoundForward<  LoopCounter1Value-1,
                                        LoopCounter2Value,
                                        Offset1Value+1,
                                        Offset2Value>::for_each( f_ioArg, f_func);
            }

            template <class IOArgType, class FuncType>
            static inline
            void    for_each_idx2(IOArgType& f_ioArg, FuncType& f_func)
            {
                f_func( f_ioArg(Offset1Value,Offset2Value));

                TUnroll2DRoundForward<  COUNTER1_IN_FUNC2,
                                        LoopCounter2Value-1,
                                        Offset1Value,
                                        Offset2Value+1>::for_each_idx2( f_ioArg, f_func);
            }

            template <class IOArg1Type, class IOArg2Type, class FuncType>
            static inline
            void    for_each(IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2, FuncType& f_func)
            {
                TUnroll2DRoundForward<  COUNTER1_IN_FUNC1,
                                        LoopCounter2Value,
                                        Offset1Value,
                                        Offset2Value>::for_each_idx2( f_ioArg1, f_ioArg2, f_func);

                TUnroll2DRoundForward<  LoopCounter1Value-1,
                                        LoopCounter2Value,
                                        Offset1Value+1,
                                        Offset2Value>::for_each( f_ioArg1, f_ioArg2, f_func);
            }

            template <class IOArg1Type, class IOArg2Type, class FuncType>
            static inline
            void    for_each_idx2(IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2, FuncType& f_func)
            {
                f_func( f_ioArg1(Offset1Value,Offset2Value), f_ioArg2(Offset1Value,Offset2Value));

                TUnroll2DRoundForward<  COUNTER1_IN_FUNC2,
                                        LoopCounter2Value-1,
                                        Offset1Value,
                                        Offset2Value+1>::for_each_idx2( f_ioArg1, f_ioArg2, f_func);
            }

            template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType>
            static inline
            void    for_each(   IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2, IOArg3Type& f_ioArg3,
                                FuncType& f_func)
            {
                TUnroll2DRoundForward<  COUNTER1_IN_FUNC1,
                                        LoopCounter2Value,
                                        Offset1Value,
                                        Offset2Value>::for_each_idx2(   f_ioArg1, f_ioArg2, f_ioArg3,
                                                                        f_func);

                TUnroll2DRoundForward<  LoopCounter1Value-1,
                                        LoopCounter2Value,
                                        Offset1Value+1,
                                        Offset2Value>::for_each(    f_ioArg1, f_ioArg2, f_ioArg3,
                                                                    f_func);
            }

            template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType>
            static inline
            void    for_each_idx2 ( IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2, IOArg3Type& f_ioArg3,
                                    FuncType& f_func)
            {
                f_func(     f_ioArg1(Offset1Value,Offset2Value),
                            f_ioArg2(Offset1Value,Offset2Value),
                            f_ioArg3(Offset1Value,Offset2Value) );

                TUnroll2DRoundForward<  COUNTER1_IN_FUNC2,
                                        LoopCounter2Value-1,
                                        Offset1Value,
                                        Offset2Value+1>::for_each_idx2( f_ioArg1, f_ioArg2, f_ioArg3,
                                                                        f_func);
            }

            template <class OutArgType, class FuncType>
            static inline
            void    generate(OutArgType& f_outArg, FuncType& f_func)
            {
                TUnroll2DRoundForward<  COUNTER1_IN_FUNC1,
                                        LoopCounter2Value,
                                        Offset1Value,
                                        Offset2Value>::generate_idx2( f_outArg, f_func);

                TUnroll2DRoundForward<  LoopCounter1Value-1,
                                        LoopCounter2Value,
                                        Offset1Value+1,
                                        Offset2Value>::generate( f_outArg, f_func);
            }

            template <class OutArgType, class FuncType>
            static inline
            void    generate_idx2(OutArgType& f_outArg, FuncType& f_func)
            {
                f_outArg(Offset1Value,Offset2Value) = f_func();

                TUnroll2DRoundForward<  COUNTER1_IN_FUNC2,
                                        LoopCounter2Value-1,
                                        Offset1Value,
                                        Offset2Value+1>::generate_idx2( f_outArg, f_func);
            }

            template <class InArgType, class OutArgType, class FuncType>
            static inline
            void    transform(  const InArgType&  f_inArg, OutArgType& f_destArg,
                                FuncType&  f_func)
            {
                TUnroll2DRoundForward<  COUNTER1_IN_FUNC1,
                                        LoopCounter2Value,
                                        Offset1Value,
                                        Offset2Value>::transform_idx2( f_inArg, f_destArg, f_func);

                TUnroll2DRoundForward<  LoopCounter1Value-1,
                                        LoopCounter2Value,
                                        Offset1Value+1,
                                        Offset2Value>::transform( f_inArg, f_destArg, f_func);
            }

            template <class InArgType, class OutArgType, class FuncType>
            static inline
            void    transform_idx2( const InArgType&  f_inArg, OutArgType& f_destArg,
                                    FuncType&  f_func)
            {
                f_destArg(Offset1Value,Offset2Value) = f_func(f_inArg(Offset1Value,Offset2Value));

                TUnroll2DRoundForward<  COUNTER1_IN_FUNC2,
                                        LoopCounter2Value-1,
                                        Offset1Value,
                                        Offset2Value+1>::transform_idx2(    f_inArg, f_destArg,
                                                                            f_func);
            }

            template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType>
            static inline
            void    transform(  const InArg1Type&  f_inArg1, const InArg2Type&  f_inArg2,
                                OutArgType&  f_destArg,
                                FuncType&   f_func)
            {
                TUnroll2DRoundForward<  COUNTER1_IN_FUNC1,
                                        LoopCounter2Value,
                                        Offset1Value,
                                        Offset2Value>::transform_idx2(  f_inArg1, f_inArg2, f_destArg,
                                                                        f_func);

                TUnroll2DRoundForward<  LoopCounter1Value-1,
                                        LoopCounter2Value,
                                        Offset1Value+1,
                                        Offset2Value>::transform(   f_inArg1, f_inArg2, f_destArg,
                                                                    f_func);

            }

            template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType>
            static inline
            void    transform_idx2( const InArg1Type&  f_inArg1, const InArg2Type&  f_inArg2,
                                    OutArgType&  f_destArg,
                                    FuncType&   f_func)
            {
                f_destArg(Offset1Value,Offset2Value) = f_func(  f_inArg1(Offset1Value,Offset2Value),
                                                                f_inArg2(Offset1Value,Offset2Value));

                TUnroll2DRoundForward<  COUNTER1_IN_FUNC2,
                                        LoopCounter2Value-1,
                                        Offset1Value,
                                        Offset2Value+1>::transform_idx2(    f_inArg1, f_inArg2, f_destArg,
                                                                            f_func);
            }

        };

        //=============================================================================
        //    vfc::intern::TUnroll2DRoundForward<0>
        //-----------------------------------------------------------------------------
        // stopping recursion
        //=============================================================================

        template <vfc::int32_t LoopCounter2Value, vfc::int32_t Offset1Value, vfc::int32_t Offset2Value>
        struct TUnroll2DRoundForward<0,LoopCounter2Value,Offset1Value,Offset2Value>
        {
            template <class IOArgType, class FuncType>
            static inline
            void    for_each(IOArgType&, FuncType& )
            {
            }

            template <class IOArgType, class FuncType>
            static inline
            void    for_each_idx2(IOArgType&, FuncType& )
            {
            }

            template <class IOArg1Type, class IOArg2Type, class FuncType>
            static inline
            void    for_each(IOArg1Type&, IOArg2Type&, FuncType& )
            {
            }

            template <class IOArg1Type, class IOArg2Type, class FuncType>
            static inline
            void    for_each_idx2(IOArg1Type&, IOArg2Type&, FuncType& )
            {
            }

            template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType>
            static inline
            void    for_each(IOArg1Type&, IOArg2Type&, IOArg3Type&, FuncType& )
            {
            }

            template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType>
            static inline
            void    for_each_idx2(IOArg1Type&, IOArg2Type&, IOArg3Type&, FuncType& )
            {
            }

            template <class OutArgType, class FuncType>
            static inline
            void    generate(OutArgType&, FuncType& )
            {
            }

            template <class OutArgType, class FuncType>
            static inline
            void    generate_idx2(OutArgType&, FuncType& )
            {
            }

            template <class InArgType, class OutArgType, class FuncType>
            static inline
            void    transform(  const InArgType& , OutArgType&, FuncType&  )
            {
            }

            template <class InArgType, class OutArgType, class FuncType>
            static inline
            void    transform_idx2(  const InArgType& , OutArgType&, FuncType&  )
            {
            }

            template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType>
            static inline
            void    transform(  const InArg1Type& , const InArg2Type& , OutArgType& , FuncType&   )
            {
            }

            template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType>
            static inline
            void    transform_idx2(  const InArg1Type& , const InArg2Type& , OutArgType& , FuncType&   )
            {
            }
        };

        //=============================================================================
        //    vfc::intern::TUnroll2DSquareForward<>
        //-----------------------------------------------------------------------------
        // algorithms for two-dimensional arguments with square bracket operator ( arg[n1][n2] )
        // e.g. int** mat1;
        //      int   mat2[3][3]
        //      std::vector< std::vector<int> > mat3;
        //=============================================================================

        template <  vfc::int32_t LoopCounter1Value, vfc::int32_t LoopCounter2Value,
                    vfc::int32_t Offset1Value=0, vfc::int32_t Offset2Value=0>
        struct TUnroll2DSquareForward
        {
            // used for stopping recursion at the end of row
            enum { COUNTER1_IN_FUNC2 = (LoopCounter2Value==1) ? 0 : LoopCounter1Value};

            // used for recursion stop in pathological cases
            // e.g. TUnroll2DSquareForward<1,0,x,y>()
            enum { COUNTER1_IN_FUNC1 = (LoopCounter2Value==0) ? 0 : LoopCounter1Value};

            template <class IOArgType, class FuncType>
            static inline
            void    for_each(IOArgType& f_ioArg, FuncType& f_func)
            {
                TUnroll2DSquareForward< COUNTER1_IN_FUNC1,
                                        LoopCounter2Value,
                                        Offset1Value,
                                        Offset2Value>::for_each_idx2( f_ioArg, f_func);

                TUnroll2DSquareForward< LoopCounter1Value-1,
                                        LoopCounter2Value,
                                        Offset1Value+1,
                                        Offset2Value>::for_each( f_ioArg, f_func);
            }

            template <class IOArgType, class FuncType>
            static inline
            void    for_each_idx2(IOArgType& f_ioArg, FuncType& f_func)
            {
                f_func( f_ioArg[Offset1Value][Offset2Value]);

                TUnroll2DSquareForward< COUNTER1_IN_FUNC2,
                                        LoopCounter2Value-1,
                                        Offset1Value,
                                        Offset2Value+1>::for_each_idx2( f_ioArg, f_func);
            }

            template <class IOArg1Type, class IOArg2Type, class FuncType>
            static inline
            void    for_each(IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2, FuncType& f_func)
            {
                TUnroll2DSquareForward< COUNTER1_IN_FUNC1,
                                        LoopCounter2Value,
                                        Offset1Value,
                                        Offset2Value>::for_each_idx2( f_ioArg1, f_ioArg2, f_func);

                TUnroll2DSquareForward< LoopCounter1Value-1,
                                        LoopCounter2Value,
                                        Offset1Value+1,
                                        Offset2Value>::for_each( f_ioArg1, f_ioArg2, f_func);
            }

            template <class IOArg1Type, class IOArg2Type, class FuncType>
            static inline
            void    for_each_idx2(IOArg1Type& f_ioArg1, IOArg2Type f_ioArg2, FuncType& f_func)
            {
                f_func( f_ioArg1[Offset1Value][Offset2Value],
                        f_ioArg2[Offset1Value][Offset2Value]);

                TUnroll2DSquareForward< COUNTER1_IN_FUNC2,
                                        LoopCounter2Value-1,
                                        Offset1Value,
                                        Offset2Value+1>::for_each_idx2( f_ioArg1, f_ioArg2, f_func);
            }

            template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType>
            static inline
            void    for_each(   IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2, IOArg3Type& f_ioArg3,
                                FuncType& f_func)
            {
                TUnroll2DSquareForward< COUNTER1_IN_FUNC1,
                                        LoopCounter2Value,
                                        Offset1Value,
                                        Offset2Value>::for_each_idx2(   f_ioArg1, f_ioArg2,
                                                                        f_ioArg3, f_func);

                TUnroll2DSquareForward< LoopCounter1Value-1,
                                        LoopCounter2Value,
                                        Offset1Value+1,
                                        Offset2Value>::for_each( f_ioArg1, f_ioArg2, f_ioArg3, f_func);
            }

            template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType>
            static inline
            void    for_each_idx2(  IOArg1Type& f_ioArg1, IOArg2Type& f_ioArg2, IOArg3Type& f_ioArg3,
                                    FuncType& f_func)
            {
                f_func( f_ioArg1[Offset1Value][Offset2Value],
                        f_ioArg2[Offset1Value][Offset2Value],
                        f_ioArg3[Offset1Value][Offset2Value] );

                TUnroll2DSquareForward< COUNTER1_IN_FUNC2,
                                        LoopCounter2Value-1,
                                        Offset1Value,
                                        Offset2Value+1>::for_each_idx2( f_ioArg1, f_ioArg2, f_ioArg3,
                                                                        f_func);
            }

            template <class OutArgType, class FuncType>
            static inline
            void    generate(OutArgType& f_outArg, FuncType& f_func)
            {
                TUnroll2DSquareForward< COUNTER1_IN_FUNC1,
                                        LoopCounter2Value,
                                        Offset1Value,
                                        Offset2Value>::generate_idx2( f_outArg, f_func);

                TUnroll2DSquareForward< LoopCounter1Value-1,
                                        LoopCounter2Value,
                                        Offset1Value+1,
                                        Offset2Value>::generate( f_outArg, f_func);
            }

            template <class OutArgType, class FuncType>
            static inline
            void    generate_idx2(OutArgType& f_outArg, FuncType& f_func)
            {
                f_outArg[Offset1Value][Offset2Value] = f_func();

                TUnroll2DSquareForward< COUNTER1_IN_FUNC2,
                                        LoopCounter2Value-1,
                                        Offset1Value,
                                        Offset2Value+1>::generate_idx2( f_outArg, f_func);
            }

            template <class InArgType, class OutArgType, class FuncType>
            static inline
            void    transform(  const InArgType&  f_inArg, OutArgType& f_destArg, FuncType&  f_func)
            {
                TUnroll2DSquareForward< COUNTER1_IN_FUNC1,
                                        LoopCounter2Value,
                                        Offset1Value,
                                        Offset2Value>::transform_idx2( f_inArg, f_destArg, f_func);

                TUnroll2DSquareForward< LoopCounter1Value-1,
                                        LoopCounter2Value,
                                        Offset1Value+1,
                                        Offset2Value>::transform( f_inArg, f_destArg, f_func);
            }

            template <class InArgType, class OutArgType, class FuncType>
            static inline
            void    transform_idx2(  const InArgType&  f_inArg, OutArgType& f_destArg, FuncType&  f_func)
            {
                f_destArg[Offset1Value][Offset2Value] = f_func(f_inArg[Offset1Value][Offset2Value]);

                TUnroll2DSquareForward< COUNTER1_IN_FUNC2,
                                        LoopCounter2Value-1,
                                        Offset1Value,
                                        Offset2Value+1>::transform_idx2( f_inArg, f_destArg, f_func);
            }

            template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType>
            static inline
            void    transform(  const InArg1Type&  f_inArg1, const InArg2Type&  f_inArg2,
                                OutArgType&  f_destArg, FuncType&   f_func)
            {
                TUnroll2DSquareForward< COUNTER1_IN_FUNC1,
                                        LoopCounter2Value,
                                        Offset1Value,
                                        Offset2Value>::transform_idx2(  f_inArg1, f_inArg2, f_destArg,
                                                                        f_func);

                TUnroll2DSquareForward< LoopCounter1Value-1,
                                        LoopCounter2Value,
                                        Offset1Value+1,
                                        Offset2Value>::transform(   f_inArg1, f_inArg2, f_destArg,
                                                                    f_func);

            }

            template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType>
            static inline
            void    transform_idx2(  const InArg1Type&  f_inArg1, const InArg2Type&  f_inArg2,
                                    OutArgType&  f_destArg, FuncType&   f_func)
            {
                f_destArg[Offset1Value][Offset2Value] = f_func( f_inArg1[Offset1Value][Offset2Value],
                                                                f_inArg2[Offset1Value][Offset2Value]);

                TUnroll2DSquareForward< COUNTER1_IN_FUNC2,
                                        LoopCounter2Value-1,
                                        Offset1Value,
                                        Offset2Value+1>::transform_idx2(f_inArg1, f_inArg2, f_destArg,
                                                                        f_func);
            }

        };

        //=============================================================================
        //    vfc::intern::TUnroll2DSquareForward<0>
        //-----------------------------------------------------------------------------
        // stopping recursion
        //=============================================================================

        template <vfc::int32_t LoopCounter2Value, vfc::int32_t Offset1Value, vfc::int32_t Offset2Value>
        struct TUnroll2DSquareForward<0,LoopCounter2Value,Offset1Value,Offset2Value>
        {
            template <class IOArgType, class FuncType>
            static inline
            void    for_each(IOArgType&, FuncType&)
            {
            }

            template <class IOArgType, class FuncType>
            static inline
            void    for_each_idx2(IOArgType&, FuncType&)
            {
            }

            template <class IOArg1Type, class IOArg2Type, class FuncType>
            static inline
            void    for_each(IOArg1Type&, IOArg2Type&, FuncType&)
            {
            }

            template <class IOArg1Type, class IOArg2Type, class FuncType>
            static inline
            void    for_each_idx2(IOArg1Type&, IOArg2Type, FuncType&)
            {
            }

            template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType>
            static inline
            void    for_each(IOArg1Type&, IOArg2Type&, IOArg3Type&, FuncType&)
            {
            }

            template <class IOArg1Type, class IOArg2Type, class IOArg3Type, class FuncType>
            static inline
            void    for_each_idx2(IOArg1Type&, IOArg2Type&, IOArg3Type&, FuncType&)
            {
            }

             template <class OutArgType, class FuncType>
            static inline
            void    generate(OutArgType&, FuncType&)
            {
            }

            template <class OutArgType, class FuncType>
            static inline
            void    generate_idx2(OutArgType&, FuncType&)
            {
            }

            template <class InArgType, class OutArgType, class FuncType>
            static inline
            void    transform(  const InArgType& , OutArgType&, FuncType& )
            {
            }

            template <class InArgType, class OutArgType, class FuncType>
            static inline
            void    transform_idx2(  const InArgType& , OutArgType&, FuncType& )
            {
            }

            template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType>
            static inline
            void    transform(  const InArg1Type& , const InArg2Type& , OutArgType& , FuncType&  )
            {
            }

            template <class InArg1Type, class InArg2Type, class OutArgType, class FuncType>
            static inline
            void    transform_idx2(  const InArg1Type& , const InArg2Type& , OutArgType& , FuncType&  )
            {
            }
        };
    }   // namespace intern closed

    //-------------------------------------------------------------------------
    //! @endcond
    // of VFC_DOXY_INTERN
    //-------------------------------------------------------------------------

}   // namespace vfc closed

//=============================================================================
// modified STL for 2D random-access iterators (op()(n1,n2) round brackets)
//=============================================================================

//=============================================================================
//    vfc::for_each_2d_round_fixed_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The algorithm for_each is very flexible, allowing the modification of each
/// element within a range in different, user-specified ways.
/// Templatized functions may be reused in a modified form by passing different
/// parameters. User-defined functions may accumulate information within an
/// internal state that the algorithm may return after processing all of the
/// elements in the range. @n
/// The elements in the range @b must be accessible with a 2D round bracket
/// operator()(int,int).
/// @par implicit functor interface
/// @code void FuncType::operator()(ValueType&) @endcode
/// @return the function object.
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d_fixed.hpp
//=============================================================================

template <  vfc::int32_t Count1Value, vfc::int32_t Count2Value,
            class InArgType,
            class FuncType>
inline
FuncType    vfc::for_each_2d_round_fixed_n (InArgType& f_inArg,
                                            FuncType f_func)
{
    VFC_STATIC_ASSERT((Count1Value>=0) && (Count2Value >= 0));
    vfc::intern::TUnroll2DRoundForward<Count1Value,Count2Value>::for_each(  f_inArg,
                                                                            f_func);
    return f_func;
}

//=============================================================================
//    vfc::for_each_2d_round_fixed_n()
//-----------------------------------------------------------------------------
/// The elements in the range @b must be accessible with a 2D round bracket
/// operator()(int,int).
/// @sa for_each_2d_round_fixed_n
/// @par implicit functor interface
/// @code void FuncType::operator()(Value1Type&, Value2Type&) @endcode
/// @return the function object.
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d_fixed.hpp
//=============================================================================

template <  vfc::int32_t Count1Value, vfc::int32_t Count2Value,
            class IOArg1Type, class IOArg2Type,
            class FuncType>
inline
FuncType    vfc::for_each_2d_round_fixed_n (IOArg1Type& f_ioArg1,
                                            IOArg2Type& f_ioArg2,
                                            FuncType f_func)
{
    VFC_STATIC_ASSERT(Count1Value>=0 && Count2Value >= 0);
    vfc::intern::TUnroll2DRoundForward<Count1Value,Count2Value>::for_each(  f_ioArg1,
                                                                            f_ioArg2,
                                                                            f_func);
    return f_func;
}

//=============================================================================
//    vfc::for_each_2d_round_fixed_n()
//-----------------------------------------------------------------------------
/// The elements in the range @b must be accessible with a 2D round bracket
/// operator()(int,int).
/// @sa for_each_2d_round_fixed_n
/// @par implicit functor interface
/// @code
/// void FuncType::operator()(Value1Type&, Value2Type&, Value3Type&)
/// @endcode
/// @return the function object.
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d_fixed.hpp
//=============================================================================

template <  vfc::int32_t Count1Value, vfc::int32_t Count2Value,
            class IOArg1Type, class IOArg2Type, class IOArg3Type,
            class FuncType>
inline
FuncType    vfc::for_each_2d_round_fixed_n (    IOArg1Type& f_ioArg1,
                                                IOArg2Type& f_ioArg2,
                                                IOArg3Type& f_ioArg3,
                                                FuncType f_func)
{
    VFC_STATIC_ASSERT(Count1Value>=0 && Count2Value >= 0);
    vfc::intern::TUnroll2DRoundForward<Count1Value,Count2Value>::for_each(  f_ioArg1,
                                                                            f_ioArg2,
                                                                            f_ioArg3,
                                                                            f_func);
    return f_func;
}

//=============================================================================
//    vfc::generate_2d_round_fixed_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The function object is invoked for each element in the range and does not
/// need to return the same value each time it is called. It may, for example,
/// read from a file or refer to and modify a local state.The generator's result
/// type must be convertible to the value type of the forward iterators for the
/// range. @n
/// The elements in the range @b must be accessible with a 2D round bracket
/// operator()(int,int).
/// @par implicit functor interface
/// @code ValueType FuncType::operator()() @endcode
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d_fixed.hpp
//=============================================================================

template <  vfc::int32_t Count1Value, vfc::int32_t Count2Value,
            class FwArgType,
            class FuncType>
inline
void        vfc::generate_2d_round_fixed_n (FwArgType& f_fwArg,
                                            FuncType f_func)
{
    VFC_STATIC_ASSERT(Count1Value>=0 && Count2Value >= 0);
    vfc::intern::TUnroll2DRoundForward<Count1Value,Count2Value>::generate(  f_fwArg,
                                                                            f_func);
}

//=============================================================================
//    vfc::transform_2d_round_fixed_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The elements in the range @b must be accessible with a 2D round bracket
/// operator()(int,int).\n
/// The destination range must be large enough to contain the transformed
/// source range. If f_destArg is set equal to f_inArg, then the source and
/// destination ranges will be the same and the sequence will be modified in
/// place.
/// @par implicit functor interface
/// @code DestValueType FuncType::operator()(const InValueType&) @endcode
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d_fixed.hpp
//=============================================================================

template <  vfc::int32_t Count1Value, vfc::int32_t Count2Value,
            class InArgType, class OutArgType,
            class FuncType>
inline
void        vfc::transform_2d_round_fixed_n (const InArgType& f_inArg,
                                             OutArgType& f_destArg,
                                             FuncType f_func)
{
    VFC_STATIC_ASSERT(Count1Value>=0 && Count2Value >= 0);
    vfc::intern::TUnroll2DRoundForward<Count1Value,Count2Value>::transform( f_inArg,
                                                                            f_destArg,
                                                                            f_func);
}

//=============================================================================
//    vfc::transform_2d_round_fixed_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The elements in the range @b must be accessible with a 2D round bracket
/// operator()(int,int).\n
/// The destination range must be large enough to contain the transformed
/// source range. If f_destArg is set equal to f_inArg, then the source and
/// destination ranges will be the same and the sequence will be modified in
/// place.
/// @par implicit functor interface
/// @code
/// DestValueType FuncType::operator()(const InValue1Type&, const InValue2Type&)
/// @endcode
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d_fixed.hpp
//=============================================================================

template <  vfc::int32_t Count1Value, vfc::int32_t Count2Value,
            class InArg1Type, class InArg2Type, class OutArgType,
            class FuncType>
inline
void        vfc::transform_2d_round_fixed_n (const InArg1Type& f_inArg1,
                                             const InArg2Type& f_inArg2,
                                             OutArgType& f_destArg,
                                             FuncType f_func)
{
    VFC_STATIC_ASSERT(Count1Value>=0 && Count2Value >= 0);
    vfc::intern::TUnroll2DRoundForward<Count1Value,Count2Value>::transform( f_inArg1,
                                                                            f_inArg2,
                                                                            f_destArg,
                                                                            f_func);
}

//=============================================================================
// modified STL for 2D random-access iterators arg[n1][n2] (square brackets)
//=============================================================================

//=============================================================================
//    vfc::for_each_2d_square_fixed_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The algorithm for_each is very flexible, allowing the modification of each
/// element within a range in different, user-specified ways.
/// Templatized functions may be reused in a modified form by passing different
/// parameters.User-defined functions may accumulate information within an
/// internal state that the algorithm may return after processing all of the
/// elements in the range.@n
/// The elements in the range @b must be accessible with a 2D square
/// bracket arg[n1][n2].
/// @par implicit functor interface
/// @code void FuncType::operator()(ValueType&) @endcode
/// @return the function object.
/// @par Example usage:
/// @code
/// std::vector< std::vector<int32_t> > iMat;
/// //...
/// int32_t sum = for_each_2d_square_fixed_n<2,2>(iMat,TSum<int32_t>()).result();
/// // equals:
/// // sum = iMat[0][0] + iMat[0][1] + iMat[1][0] + iMat[1][1];
/// @endcode
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d_fixed.hpp
//=============================================================================

template <  vfc::int32_t Count1Value, vfc::int32_t Count2Value,
            class InArgType, class FuncType>
inline
FuncType    vfc::for_each_2d_square_fixed_n (   InArgType& f_inArg,
                                                FuncType f_func)
{
    VFC_STATIC_ASSERT(Count1Value>=0 && Count2Value >= 0);
    vfc::intern::TUnroll2DSquareForward<Count1Value,Count2Value>::for_each( f_inArg,
                                                                            f_func);
    return f_func;
}

//=============================================================================
//    vfc::for_each_2d_square_fixed_n()
//-----------------------------------------------------------------------------
/// The elements in the range @b must be accessible with a 2D square bracket
/// arg[n1][n2].
/// @sa for_each_2d_square_fixed_n
/// @par implicit functor interface
/// @code void FuncType::operator()(Value1Type&, Value2Type&) @endcode
/// @return the function object.
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d_fixed.hpp
//=============================================================================

template <  vfc::int32_t Count1Value, vfc::int32_t Count2Value,
            class IOArg1Type, class IOArg2Type,
            class FuncType>
inline
FuncType    vfc::for_each_2d_square_fixed_n (IOArg1Type& f_ioArg1,
                                             IOArg2Type& f_ioArg2,
                                             FuncType f_func)
{
    VFC_STATIC_ASSERT(Count1Value>=0 && Count2Value >= 0);
    vfc::intern::TUnroll2DSquareForward<Count1Value,Count2Value>::for_each( f_ioArg1,
                                                                            f_ioArg2,
                                                                            f_func);
    return f_func;
}

//=============================================================================
//    vfc::for_each_2d_square_fixed_n()
//-----------------------------------------------------------------------------
/// The elements in the range @b must be accessible with a 2D square bracket
/// arg[n1][n2].
/// @sa for_each_2d_square_fixed_n
/// @par implicit functor interface
/// @code
/// void FuncType::operator()(Value1Type&, Value2Type&, Value3Type&)
/// @endcode
/// @return the function object.
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d_fixed.hpp
//=============================================================================

template <  vfc::int32_t Count1Value, vfc::int32_t Count2Value,
            class IOArg1Type, class IOArg2Type, class IOArg3Type,
            class FuncType>
inline
FuncType    vfc::for_each_2d_square_fixed_n (IOArg1Type& f_ioArg1,
                                             IOArg2Type& f_ioArg2,
                                             IOArg3Type& f_ioArg3,
                                             FuncType f_func)
{
    VFC_STATIC_ASSERT(Count1Value>=0 && Count2Value >= 0);
    vfc::intern::TUnroll2DSquareForward<Count1Value,Count2Value>::for_each( f_ioArg1,
                                                                            f_ioArg2,
                                                                            f_ioArg3,
                                                                            f_func);
    return f_func;
}

//=============================================================================
//    vfc::generate_2d_square_fixed_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The function object is invoked for each element in the range and does not
/// need to return the same value each time it is called. It may, for example,
/// read from a file or refer to and modify a local state. The generator's
/// result type must be convertible to the value type of the forward iterators
/// for the range. @n
/// The elements in the range @b must be accessible with a 2D square
/// bracket arg[n1][n2].
/// @par implicit functor interface
/// @code ValueType FuncType::operator()() @endcode
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d_fixed.hpp
//=============================================================================

template <  vfc::int32_t Count1Value, vfc::int32_t Count2Value,
            class FwArgType,
            class FuncType>
inline
void        vfc::generate_2d_square_fixed_n (   FwArgType& f_fwArg,
                                                FuncType f_func)
{
    VFC_STATIC_ASSERT(Count1Value>=0 && Count2Value >= 0);
    vfc::intern::TUnroll2DSquareForward<Count1Value,Count2Value>::generate( f_fwArg,
                                                                            f_func);
}

//=============================================================================
//    vfc::transform_2d_square_fixed_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The elements in the range @b must be accessible with a 2D square bracket
/// arg[n1][n2]. @n
/// The destination range must be large enough to contain the transformed
/// source range. If f_destArg is set equal to f_inArg, then the source and
/// destination ranges will be the same and the sequence will be modified in
/// place.
/// @par implicit functor interface
/// @code DestValueType FuncType::operator()(const InValueType&) @endcode
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d_fixed.hpp
//=============================================================================

template <  vfc::int32_t Count1Value, vfc::int32_t Count2Value,
            class InArgType, class OutArgType,
            class FuncType>
inline
void        vfc::transform_2d_square_fixed_n (const InArgType& f_inArg,
                                              OutArgType& f_destArg,
                                              FuncType f_func)
{
    VFC_STATIC_ASSERT(Count1Value>=0 && Count2Value >= 0);
    vfc::intern::TUnroll2DSquareForward<Count1Value,Count2Value>::transform(    f_inArg,
                                                                                f_destArg,
                                                                                f_func);
}

//=============================================================================
//    vfc::transform_2d_square_fixed_n()
//-----------------------------------------------------------------------------
/// @par Description:
/// The elements in the range @b must be accessible with a 2D square bracket
/// arg[n1][n2].
/// The destination range must be large enough to contain the transformed
/// source range. If f_destArg is set equal to f_inArg, then the source and
/// destination ranges will be the same and the sequence will be modified in
/// place.
/// @par implicit functor interface
/// @code
/// DestValueType FuncType::operator()(const InValue1Type&, const InValue2Type&)
/// @endcode
/// @author zvh2hi
/// @par Requirements:
/// - vfc_algorithm2d_fixed.hpp
//=============================================================================

template <  vfc::int32_t Count1Value, vfc::int32_t Count2Value,
            class InArg1Type, class InArg2Type, class OutArgType,
            class FuncType>
inline
void        vfc::transform_2d_square_fixed_n (const InArg1Type& f_inArg1,
                                              const InArg2Type& f_inArg2,
                                              OutArgType& f_destArg,
                                              FuncType f_func)
{
    VFC_STATIC_ASSERT(Count1Value>=0 && Count2Value >= 0);
    vfc::intern::TUnroll2DSquareForward<Count1Value,Count2Value>::transform(    f_inArg1,
                                                                                f_inArg2,
                                                                                f_destArg,
                                                                                f_func);
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_algorithm2d_fixed.inl  $
//  Revision 1.9 2012/12/18 08:27:28MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.8 2009/02/11 14:01:57MEZ Gaurav Jain (RBEI/ESB3) (gaj2kor) 
//  -Removal of QAC++ warnings. (mantis2445)
//  Revision 1.7 2007/08/02 19:21:44IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added conditional doxygen documentation generation of vfc::intern (mantis1758)
//  Revision 1.6 2007/01/09 12:25:09CET dkn2kor
//  - Used signed type for count values, changed size_t to int32_t (mantis1376)
//  - added static assertion for count values less than 0 (mantis1375)
//  Revision 1.5 2006/11/24 14:10:12IST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - fixed docu bug (mantis1316)
//  Revision 1.4 2006/11/16 14:41:07CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.3 2006/11/06 11:19:20CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - added missing vfc scope (mantis1253)
//  - cosmetics
//  Revision 1.2 2006/10/23 14:01:50CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - fixed missing call-by-reference in TUnroll2DRoundForward::for_each_idx2() (mantis1217)
//=============================================================================
