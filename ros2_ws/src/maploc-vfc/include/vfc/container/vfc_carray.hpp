//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2008 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorized copy or use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  P R O J E C T   I N F O R M A T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc
//  Target system(s):
//       Compiler(s): c++ std conformal
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: dkn2kor
//  Department:
//=============================================================================
//  F I L E   C O N T E N T S   A N D   R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @brief This is a brief description.
/// @par Synopsis:
///     This is the detailed description.
///
/// @par Revision History:
///     $Source: vfc_carray.hpp $
///     $Revision: 1.2 $
///     $Author: Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) $
///     $Date: 2012/01/18 06:57:06MEZ $
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

#ifndef VFC_CARRAY_HPP_INCLUDED
#define VFC_CARRAY_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"                //for vfc::int32_t
#include "vfc/core/vfc_static_assert.hpp"        //for VFC_STATIC_ASSERT
#include <iterator>                              //reverse_iterator

namespace vfc
{
    //=============================================================================
    //  TCArray<>
    //-----------------------------------------------------------------------------
    //! TCArray is a simple wrapper for a c-array for cases where the TFixedVector
    //! is too big.
    //! @par Description:
    //! A template class representing a TCArray.
    //! @param ValueType        DataType to be stored
    //! @param CapacityValue    Capacity of the Array
    //! @author                 dkn2kor
    //! @ingroup                vfc_group_containers
    //=============================================================================
    template<class ValueType, vfc::int32_t CapacityValue>
    class TCArray
    {
    public:

        VFC_STATIC_ASSERT(CapacityValue>0);

        enum
        {
            ARRAY_SIZE = CapacityValue  //! The static array size defined at compile time.
        };

        //! typedefs

        //! A type that represent the data type stored in a TCArray
        typedef ValueType                               value_type;

        //! A type that provides a iterator for TCArray which is used for traversing
        //! the elements.
        typedef ValueType*                              iterator;

        //! A type that provides a const iterator for TCArray which is used for traversing
        //! the elements.
        typedef const ValueType*                        const_iterator;

        //! A type that provides a iterator for TCArray which is used for traversing
        //! the elements.
        typedef stlalias::reverse_iterator<iterator>    reverse_iterator;

        //! A type that provides a const iterator for TCArray which is used for traversing
        //! the elements.
        typedef stlalias::reverse_iterator<const_iterator> const_reverse_iterator;

        //! A type that provides a reference to a element stored in a TCArray for reading
        //! and performing operations.
        typedef value_type&                             reference;

        //! A type that provides a reference to a const element stored in a TCArray for
        //! reading and performing const operations.
        typedef const value_type&                       const_reference;

        //! A type that provides reference to volatile element.
        typedef volatile value_type&                    volatile_reference;

        //! A type that provides const reference to volatile element.
        typedef const volatile value_type&              const_volatile_reference ;

        //! A type that counts the number of elements in a TCArray.
        typedef vfc::int32_t                            size_type;


        //---------------------------------------------------------------------
        //! Returns pointer to the first stored element.
        //! $Source: vfc_carray.hpp $
        //! @return  returns pointer
        //! @author  dhn1kor
        //---------------------------------------------------------------------
        iterator begin();

        //---------------------------------------------------------------------
        //! Returns const pointer to the first stored element.
        //! $Source: vfc_carray.hpp $
        //! @return  returns const pointer
        //! @author  dhn1kor
        //---------------------------------------------------------------------
        const_iterator begin() const;


        //---------------------------------------------------------------------
        //! Returns pointer after the last stored element.
        //! $Source: vfc_carray.hpp $
        //! @return  returns  pointer
        //! @author  dhn1kor
        //---------------------------------------------------------------------
        iterator end();

        //---------------------------------------------------------------------
        //! Returns const pointer after the last stored element.
        //! $Source: vfc_carray.hpp $
        //! @return  returns const pointer
        //! @author  dhn1kor
        //---------------------------------------------------------------------
        const_iterator end() const;


        //---------------------------------------------------------------------
        //! Returns pointer after the last stored element.
        //! $Source: vfc_carray.hpp $
        //! @return returns pointer
        //! @author dhn1kor
        //---------------------------------------------------------------------
        reverse_iterator rbegin();

        //---------------------------------------------------------------------
        //! Returns const pointer after the last stored element.
        //! $Source: vfc_carray.hpp $
        //! @return returns const pointer
        //! @author dhn1kor
        //---------------------------------------------------------------------
        const_reverse_iterator rbegin() const;

        //---------------------------------------------------------------------
        //! Returns pointer to the first stored element.
        //! $Source: vfc_carray.hpp $
        //! @return returns pointer
        //! @author dhn1kor
        //---------------------------------------------------------------------
        reverse_iterator rend();

        //---------------------------------------------------------------------
        //! Returns const pointer to the first stored element.
        //! $Source: vfc_carray.hpp $
        //! @return returns const pointer
        //! @author dhn1kor
        //---------------------------------------------------------------------
        const_reverse_iterator rend() const;

        //---------------------------------------------------------------------
        //! Returns a reference to the element at position f_pos in TCArray.
        //! $Source: vfc_carray.hpp $
        //! @return returns reference
        //! @author dkn2kor
        //---------------------------------------------------------------------
        reference operator[](size_type f_pos);

        //---------------------------------------------------------------------
        //! Returns volatile reference to the element at position f_pos in TCArray.
        //! $Source: vfc_carray.hpp $
        //! @return returns volatile reference
        //! @author vmr1kor
        //---------------------------------------------------------------------
        volatile_reference operator[](size_type f_pos) volatile;

        //---------------------------------------------------------------------
        //! Returns a const reference to the element at position f_pos in TCArray.
        //! $Source: vfc_carray.hpp $
        //! @return returns const reference
        //! @author dkn2kor
        //---------------------------------------------------------------------
        const_reference operator[](size_type f_pos) const;

        //---------------------------------------------------------------------
        //! Returns const volatile reference to the element at position f_pos in TCArray.
        //! $Source: vfc_carray.hpp $
        //! @return returns const volatile reference
        //! @author vmr1kor
        //---------------------------------------------------------------------
        const_volatile_reference operator[](size_type f_pos) const volatile;

        //---------------------------------------------------------------------
        //! Returns the number of elements the array can hold
        //! $Source: vfc_carray.hpp $
        //! @return returns size of array in elements, not bytes
        //! @author dkn2kor
        //---------------------------------------------------------------------
        size_type capacity() const;

        //---------------------------------------------------------------------
        //! Returns the number of elements the array can hold
        //! $Source: vfc_carray.hpp $
        //! @return returns size of array in elements, not bytes
        //! @author grs1cob
        //---------------------------------------------------------------------
        size_type size() const;

        //---------------------------------------------------------------------
        //! Returns the number of elements the array can hold
        //! $Source: vfc_carray.hpp $
        //! @return returns size of array in elements, not bytes
        //! @author grs1cob
        //---------------------------------------------------------------------
        size_type max_size() const;

        //---------------------------------------------------------------------
        //! Checks whether the container is empty(Capacity value == 0)
        //! $Source: vfc_carray.hpp $
        //! @return returns false(becasue Capacity value cannot be 0)
        //! @author grs1cob
        //---------------------------------------------------------------------
        bool empty();

    public:

        //This Datastructure is made Public as no constuctors are used
        //for member initsialization.The reason being the possibilty to
        //initialize at compile time shall be kept like with plain C
        //arrays because only then the code is ROMable.
        ValueType m_value[ARRAY_SIZE]; //!<    internal Datastructure.
    };
}

#include "vfc/container/vfc_carray.inl"

#endif //TEMPLATE_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_carray.hpp  $
//  Revision 1.2 2012/01/18 06:57:06MEZ Vanitha Nagarajan (RBEI/ESD1) (NVA1COB) 
//  - Added additional functionalities size(),maxsize(),empty() (mantis 3184)
//  Revision 1.1 2010/08/11 21:11:29IST Jaeger Thomas (CC/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/container/container.pj
//  Revision 1.7 2009/07/29 14:39:45MESZ Vinaykumar Setty (RBEI/ESB2) (vmr1kor) 
//  - Volatile version of the access operators are added ( volatile reference and const volatile reference)  : mantis (2842 )
//  Revision 1.6 2009/05/28 13:12:32IST Muehlmann Karsten (CC-DA/ESV2) (MUK2LR) 
//  - std:: to stlalias:: (mantis2720)
//  Revision 1.5 2009/01/16 05:50:41CET Vinaykumar Setty (RBEI/EAC1) (vmr1kor) 
//  - missing header file inclusion ( #include <iterator> ) added ( mantis id :- 2508 )
//  Revision 1.4 2008/12/08 21:49:59IST Muehlmann Karsten (CC-DA/ESV1) (MUK2LR) 
//  - add capacity() (mantis2466)
//  Revision 1.3 2008/08/11 11:13:29CEST Dhananjay N (RBEI/EAE6) (dhn1kor) 
//  -ValueType added as class template parameter , Iterator interface is added and  TCArray is made  Romable (Mantis:- 2214)
//  Revision 1.2 2008/07/08 18:03:06IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  - more assertions added
//  Revision 1.1 2008/07/08 16:16:46IST Dilip Krishna (RBEI/EAE6) (dkn2kor)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_carray/vfc_carray.pj
//=============================================================================
