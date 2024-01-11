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
///     $Source: vfc_fixedlist.hpp $
///     $Revision: 1.2 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2014/10/02 16:28:56MESZ $
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

#ifndef VFC_FIXEDLIST_HPP_INCLUDED
#define VFC_FIXEDLIST_HPP_INCLUDED

#include "vfc/container/vfc_list.hpp"                       //! used for TList class
#include "vfc/memory/vfc_fixedmempool_allocator.hpp"        //! used for TFixedMemPoolAllocator Allocator class

namespace vfc
{   // namespace vfc opened

    namespace intern
    {   // namespace intern opened
        
        template<vfc::int32_t SizeValue>
        struct TFixedListMaxsizePolicy
        {
            static vfc::int32_t maxSizePolicy();
        }; 

    }   // namespace intern closed

    //=============================================================================
    //  TFixedList<>
    //-----------------------------------------------------------------------------
    //! Fixed Linked List class.
    //! The list class is a template class of sequence containers that maintain 
    //! their elements in a linear arrangement and allow efficient insertions and 
    //! deletions at any location within the sequence. The sequence is stored as a 
    //! bidirectional linked list of elements, each containing a member of some type Type.
    //! $Source: vfc_fixedlist.hpp $
    //! @param ValueType        DataType to be stored.
    //! @param SizeValue        size of the fixedList.
    //! @author                 vmr1kor.
    //! @ingroup                vfc_group_containers.
    //=============================================================================
    template<class ValueType,vfc::int32_t SizeValue>
    class TFixedList : public vfc::TList<ValueType,
                                        vfc::TFixedMemPoolAllocator<ValueType, SizeValue>,
                                        vfc::intern::TFixedListMaxsizePolicy<SizeValue> >
    {
    public:

        typedef TFixedList<ValueType,SizeValue>                             fixed_list_type ;
        typedef vfc::TList<ValueType,vfc::TFixedMemPoolAllocator<ValueType, SizeValue>,
                  vfc::intern::TFixedListMaxsizePolicy<SizeValue> >         base_type ;

        /// A type that represents the data type stored in a list.
        typedef typename base_type::value_type                              value_type;
        /// A type that counts the number of elements in a list.
        typedef typename base_type::size_type                               size_type;
        /// node allocate type
        typedef typename base_type::nodealloc_type                          nodealloc_type;
        /// A type that provides a pointer to an element in a list.
        typedef typename base_type::pointer                                 pointer;
        /// A type that provides a pointer to a const element in a list.
        typedef typename base_type::const_pointer                           const_pointer;
        /// A type that provides a reference to a element stored in a list for reading 
        /// and performing operations.
        typedef typename base_type::reference                               reference;
        /// A type that provides a reference to a const element stored in a list for 
        /// reading and performing const operations.
        typedef typename base_type::const_reference                         const_reference;
        /// A type that provides a bidirectional iterator that can read a const element in a list.
        typedef typename base_type::const_iterator                          const_iterator;
        /// A type that provides a bidirectional iterator that can read or modify 
        /// any element in a list.
        typedef typename base_type::iterator                                iterator;
        /// A type that provides a bidirectional iterator that can read any const element 
        /// in a list.
        typedef typename base_type::const_reverse_iterator                  const_reverse_iterator;
        /// A type that provides a bidirectional iterator that can read or modify an element in 
        /// a reversed list.
        typedef typename base_type::reverse_iterator                        reverse_iterator;

        //---------------------------------------------------------------------
        //! Default constructor.
        //! $Source: vfc_fixedlist.hpp $
        //! @author vmr1kor
        //---------------------------------------------------------------------
        TFixedList(void);

        //---------------------------------------------------------------------
        //! Creates the list with a specific allocator..
        //! $Source: vfc_fixedlist.hpp $
        //! @author vmr1kor
        //---------------------------------------------------------------------
        explicit TFixedList(const nodealloc_type& f_alloc_r);

        //---------------------------------------------------------------------
        //! Creates a FixedList with default elements.
        //! Constructor fills the list with the specified number 
        //! copies of default-constructed element.
        //! $Source: vfc_fixedlist.hpp $
        //! @param  f_count     The number of elements to initially create.
        //! @author vmr1kor
        //---------------------------------------------------------------------
        explicit TFixedList(size_type f_count);

        //-----------------------------------------------------------------------------
        //! Creates a FixedList with specified element value.
        //! Constructor fills the list with the specified number 
        //! copies of element with the given value.
        //! $Source: vfc_fixedlist.hpp $
        //! @param  f_count     The number of elements to initially create.
        //! @param  f_value_r   value to be filled.
        //! @author vmr1kor
        //=============================================================================
        TFixedList( size_type f_count, const value_type& f_value_r);

        //-----------------------------------------------------------------------------
        //! Creates a TFixedList with specified element value.
        //! Constructor fills the list with the specified number 
        //! copies of element with the given value.
        //! $Source: vfc_fixedlist.hpp $
        //! @param  f_count     The number of elements to initially create.
        //! @param  f_value_r   value to be filled.
        //! @param  f_alloc_r   node type allocator.
        //! @author vmr1kor
        //=============================================================================
        TFixedList( size_type f_count, const value_type& f_value_r, const nodealloc_type& f_alloc_r);

        //-----------------------------------------------------------------------------
        //! Copy constructor.
        //! $Source: vfc_fixedlist.hpp $
        //! @param  f_rhs_r  A list with identical element type, size value and allocator type.
        //! @author vmr1kor
        //=============================================================================
        TFixedList(const fixed_list_type& f_rhs_r);
        

        //-----------------------------------------------------------------------------
        //! Builds a list from the given range.
        //! Create a list consisting of copies of the elements in the other list.
        //! $Source: vfc_fixedlist.hpp $
        //! @param  f_first  Position of the first element in the range of elements to be copied.
        //! @param  f_last   Position of the first element beyond the range of elements to be copied.
        //! @author vmr1kor
        //=============================================================================
        template<class InIteratorType>
        TFixedList(InIteratorType f_first, InIteratorType f_last);

        //-----------------------------------------------------------------------------
        //! Builds a list from the given range.
        //! Create a list consisting of copies of the elements in the other list.
        //! $Source: vfc_fixedlist.hpp $
        //! @param  f_first  Position of the first element in the range of elements to be copied.
        //! @param  f_last   Position of the first element beyond the range of elements to be copied.
        //! @param  f_alloc_r   node type allocator.
        //! @author vmr1kor
        //=============================================================================
        template<class InIteratorType>
        TFixedList(InIteratorType f_first, InIteratorType f_last, const nodealloc_type& f_alloc_r);

        //-----------------------------------------------------------------------------
        //! assignment operator.
        //! $Source: vfc_fixedlist.hpp $
        //! @param  f_rhs_r  A list with identical element type, size value and allocator type.
        //! @author vmr1kor
        //=============================================================================
        const TFixedList& operator=(const TFixedList& f_rhs_r);

    };

}   // namespace vfc closed


#include "vfc/container/vfc_fixedlist.inl"

#endif //VFC_FIXEDLIST_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fixedlist.hpp  $
//  Revision 1.2 2014/10/02 16:28:56MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - vfc_list: using operator= in derived class returns wrong type (the base class type) (mantis 0004734)
//  Revision 1.1 2010/08/11 17:44:02MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/container/container.pj
//  Revision 1.3 2008/09/30 12:20:08MESZ Vinaykumar Setty (RBEI/ESB2) (vmr1kor) 
//  - project name , synopsis and compiler names are changed( mantis :- 1793)
//  Revision 1.2 2008/09/01 16:02:36IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor) 
//  - CList review comments are incorporated( Mantis id :- 1793)
//  - Linker error got fixed ( Mantis id :- 2309)
//  Revision 1.1 2008/02/05 16:01:41IST Vinaykumar Setty (RBEI/EAE6) (vmr1kor) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_list/vfc_list.pj
//=============================================================================
