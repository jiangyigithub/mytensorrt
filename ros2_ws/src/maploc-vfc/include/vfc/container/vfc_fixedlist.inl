//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2008 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorised copy, use or 
//  distribution is an offensive act against international law and may be 
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/container
//          Synopsis: Fixed Linked List class.
//  Target system(s): 
//       Compiler(s): c++ std conformal
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes: 
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: vmr1kor
//  Department: 
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_fixedlist.inl $
///     $Revision: 1.2 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/10/06 12:36:49MESZ $
///     $Locker:  $
///     $Name:  $
///     $State: in_work $
///
/// @par Review Information:
/// - Reviewed version: 
/// - Type (use 'X' to mark):
///     - [ ] Formal Review
///     - [x] Walkthrough
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

// maxSizePolicy Definition
template<vfc::int32_t SizeValue>
inline vfc::int32_t vfc::intern::TFixedListMaxsizePolicy<SizeValue>::maxSizePolicy()
{
    return SizeValue;
}

// TFixedList

template<class ValueType,vfc::int32_t SizeValue>
vfc::TFixedList<ValueType,SizeValue>::TFixedList(void) 
            :   TList<ValueType,vfc::TFixedMemPoolAllocator<ValueType, SizeValue>,
                vfc::intern::TFixedListMaxsizePolicy<SizeValue> >()
{
    //intentionally left blank
}


template<class ValueType,vfc::int32_t SizeValue>
vfc::TFixedList<ValueType,SizeValue>::TFixedList(const nodealloc_type& f_alloc_r) 
            :   TList<ValueType,vfc::TFixedMemPoolAllocator<ValueType, SizeValue>,
                        vfc::intern::TFixedListMaxsizePolicy<SizeValue> >(f_alloc_r)
{
    //intentionally left blank
}


template<class ValueType,vfc::int32_t SizeValue>
vfc::TFixedList<ValueType,SizeValue>::TFixedList(
            size_type f_count)
            :   TList<ValueType,vfc::TFixedMemPoolAllocator<ValueType, SizeValue>,
                        vfc::intern::TFixedListMaxsizePolicy<SizeValue> >(f_count)
{
    //intentionally left blank
}


template<class ValueType,vfc::int32_t SizeValue>
vfc::TFixedList<ValueType,SizeValue>::TFixedList(
            size_type f_count, const value_type& f_value_r) 
            :   TList<ValueType,vfc::TFixedMemPoolAllocator<ValueType, SizeValue>,
                vfc::intern::TFixedListMaxsizePolicy<SizeValue> >(f_count,f_value_r) 
{
    //intentionally left blank
}


template<class ValueType,vfc::int32_t SizeValue>
vfc::TFixedList<ValueType,SizeValue>::TFixedList(
            size_type f_count, const value_type& f_value_r, const nodealloc_type& f_alloc_r)
            :   TList<ValueType,vfc::TFixedMemPoolAllocator<ValueType, SizeValue>,
                vfc::intern::TFixedListMaxsizePolicy<SizeValue> >(f_count,f_value_r,f_alloc_r)
{
    //intentionally left blank
}

template<class ValueType,vfc::int32_t SizeValue>
vfc::TFixedList<ValueType,SizeValue>::TFixedList(
    const fixed_list_type& f_rhs_r) 
    : TList<ValueType,vfc::TFixedMemPoolAllocator<ValueType, SizeValue>,vfc::intern::TFixedListMaxsizePolicy<SizeValue> >(f_rhs_r)
{
    //intentionally left blank
}


template<class ValueType,vfc::int32_t SizeValue>
template<class InIteratorType>
vfc::TFixedList<ValueType,SizeValue>::TFixedList(InIteratorType f_first, InIteratorType f_last) 
        : TList<ValueType,vfc::TFixedMemPoolAllocator<ValueType, SizeValue>,
        vfc::intern::TFixedListMaxsizePolicy<SizeValue> >(f_first,f_last)
{
    //intentionally left blank
}

template<class ValueType,vfc::int32_t SizeValue>
template<class InIteratorType>
vfc::TFixedList<ValueType,SizeValue>::TFixedList(InIteratorType f_first, InIteratorType f_last,
                                                const nodealloc_type& f_alloc_r) 
        : TList<ValueType,vfc::TFixedMemPoolAllocator<ValueType, SizeValue>,
        vfc::intern::TFixedListMaxsizePolicy<SizeValue> >(f_first,f_last,f_alloc_r)
{
    //intentionally left blank
}

template<class ValueType,vfc::int32_t SizeValue>
inline
    const vfc::TFixedList<ValueType,SizeValue>&
    vfc::TFixedList<ValueType,SizeValue>::operator= (const TFixedList& f_rhs_r)
{
    if(this != &f_rhs_r)
    {
        base_type::operator =(f_rhs_r);
    }
    return *this ;
}


//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fixedlist.inl  $
//  Revision 1.2 2014/10/06 12:36:49MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_list: using operator= in derived class returns wrong type (the base class type) (mantis 0004734)
//  Revision 1.1 2010/08/11 17:44:02MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/container/container.pj
//  Revision 1.3 2008/09/30 12:20:09MESZ Vinaykumar Setty (RBEI/ESB2) (vmr1kor) 
//  - project name , synopsis and compiler names are changed( mantis :- 1793)
//  Revision 1.2 2008/09/01 16:02:36IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor) 
//  - CList review comments are incorporated( Mantis id :- 1793)
//  - Linker error got fixed ( Mantis id :- 2309)
//  Revision 1.1 2008/02/05 16:01:44IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_list/vfc_list.pj
//=============================================================================
