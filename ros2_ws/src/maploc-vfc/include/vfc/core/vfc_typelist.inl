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
//        Name: sf71lr 
//  Department: 
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_typelist.inl $
///     $Revision: 1.4 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2012/12/18 08:27:32MEZ $
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

#include "vfc/core/vfc_types.hpp"   // used for int32_t
#include "vfc/core/vfc_static_assert.hpp" // used for VFC_STATIC_ASSERT

namespace vfc
{
    // exclude all specializations from doxygen 
    /// @cond VFC_DOXY_SPECIALIZATIONS
    
    //=========================================================================
    // TTypeListSize specializations
    //=========================================================================

    // terminator reached
    template <> 
    struct TTypeListSize<CTypeListNullType> 
    {
        enum { value = 0 };
    };

    // recursion
    template <typename HeadType, typename TailType> 
    struct TTypeListSize< TTypeList<HeadType, TailType> > 
    {
        enum { value = 1 + TTypeListSize<TailType>::value };
    };   
        
    //=========================================================================
    // TTypeListIndexOf specializations
    //=========================================================================

    // terminator reached (element as not be found)
    template <class SearchType> 
    struct TTypeListIndexOf<vfc::CTypeListNullType, SearchType> 
    {
        enum { value = -1 };
    };

    // type was found
    template <class SearchType, typename TailType> 
    struct TTypeListIndexOf< TTypeList<SearchType, TailType>, SearchType> 
    {
        enum { value = 0 };
    };    


    // If the the type was found (value == 0) then count up
    template <typename HeadType, typename TailType, typename SearchType> 
    struct TTypeListIndexOf<TTypeList<HeadType, TailType>, SearchType> 
    {
        private:
            enum { temp = TTypeListIndexOf<TailType, SearchType>::value };
        public:
            enum { value = ((temp == -1) ? (-1) : (1 + temp)) };
    };

    //=========================================================================
    // TTypeListTypeAt specializations
    //=========================================================================

    // recursion with argument validation 
    template <typename HeadType, typename TailType, int32_t IndexValue> 
    struct TTypeListTypeAt< TTypeList<HeadType, TailType>, IndexValue> 
    {
        typedef typename TTypeListTypeAt<TailType, IndexValue - 1>::type type;

        // check if index is positive
        VFC_STATIC_ASSERT ( 0 < IndexValue );
    };

    // results in an error if NullType is reached before Index goes Zero
    template <typename HeadType, int32_t IndexValue> 
    struct TTypeListTypeAt< TTypeList<HeadType, CTypeListNullType>, IndexValue>;
    // intentionally no definition here!!

    // terminate if index goes zero
    template <typename HeadType> 
    struct TTypeListTypeAt< TTypeList<HeadType, CTypeListNullType>, 0> 
    {
        typedef HeadType type;
    };

    // terminate if index goes zero
    template <typename HeadType, typename TailType> 
    struct TTypeListTypeAt< TTypeList<HeadType, TailType>, 0> 
    {
        typedef HeadType type;
    };

    //=========================================================================
    // TTypeListAppend specializations
    //=========================================================================

    // add terminator to terminator
    template <> 
    struct  TTypeListAppend<CTypeListNullType, CTypeListNullType>
    {
        typedef CTypeListNullType    type;
    };

    // append type specialization for typelist terminator, appending single type
    template <class AppendType> 
    struct  TTypeListAppend<CTypeListNullType, AppendType>
    {
        typedef TTypeList<AppendType,CTypeListNullType>  type;
    };

    // Append type specialization for typelist terminator, appending typelist
    template <class HeadType, class TailType>   
    struct TTypeListAppend < CTypeListNullType, TTypeList<HeadType,TailType> >
    {
        typedef TTypeList<HeadType,TailType> type;
    };

    // Append type for appending a type or another typelist to an existing typelist, 
    // resulting typelist is Append<TList,T>::result

    template <class HeadType, class TailType, class AppendType>   
    struct  TTypeListAppend< TTypeList<HeadType,TailType>, AppendType >
    {
        typedef TTypeList<HeadType, typename TTypeListAppend<TailType,AppendType>::type >   type;
    };

    /// @endcond 
    //  of VFC_DOXY_SPECIALIZATIONS
    
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_typelist.inl  $
//  Revision 1.4 2012/12/18 08:27:32MEZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Replace tabs by 4 spaces (mantis 4199)
//  Revision 1.3 2007/08/02 16:16:58MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - added paranthesis (mantis1696)
//  Revision 1.2 2007/07/23 09:22:19CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - added comment (mantis1744)
//  - moved #include to *.inl
//  Revision 1.1 2006/12/15 11:10:34CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/core.pj
//=============================================================================
