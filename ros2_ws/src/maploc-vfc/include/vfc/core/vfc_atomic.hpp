//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2015 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorised copy, use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc/core
//          Synopsis:
//  Target system(s):
//       Compiler(s): ISO compliant
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: Thomas Jaeger
//  Department: CC-DA-ENV1
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_atomic.hpp $
///     $Revision: 1.2 $
///     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
///     $Date: 2016/03/30 15:53:18MESZ $
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

#ifndef VFC_ATOMIC_HPP_INCLUDED
#define VFC_ATOMIC_HPP_INCLUDED

#include "vfc/core/vfc_config.hpp"
#include "vfc/core/vfc_type_traits.hpp"
#include "vfc/core/vfc_static_assert.hpp"
#include "vfc/core/vfc_assert.hpp"

namespace vfc
{
    //-----------------------------------------------------------
    /// vfc::EMemoryOrder specifies the semantics how load and
    /// store operations should be allowed to be reordered around
    /// an atomic operation.
    /// It follows the semantics of the C11/C++11 memory model.
    /// $Source: vfc_atomic.hpp $
    /// @author jat2hi
    /// @ingroup vfc_group_core_misc
    //-----------------------------------------------------------
    enum EMemoryOrder
    {
        MEMORY_ORDER_RELAXED,
        MEMORY_ORDER_CONSUME,
        MEMORY_ORDER_ACQUIRE,
        MEMORY_ORDER_RELEASE,
        MEMORY_ORDER_ACQ_REL,
        MEMORY_ORDER_SEQ_CST
    };
    
    //-----------------------------------------------------------
    /// Implement a memory barrier.
    /// Use this function to insert a memory barrier to prevent
    /// synchronisation flaws in the sw execution related to weak
    /// memory ordering on ARM hardware.
    /// $Source: vfc_atomic.hpp $
    /// @author jat2hi
    /// @ingroup vfc_group_core_misc
    //-----------------------------------------------------------
    inline
    void atomic_thread_fence(EMemoryOrder f_memorder = MEMORY_ORDER_SEQ_CST);
    
    //=========================================================================
    // TAtomic<>
    //-------------------------------------------------------------------------
    /// @struct vfc::TAtomic<>
    /// Simple atomic container with C++11 like interface, but limited to PODs.
    /// 
    /// Example usage: 
    ///
    /// \code
    ///
    /// // instantiate an atomic pointer:
    /// vfc::TAtomic<value_type*> m_atomic_p; 
    ///
    /// // atomic store of a new pointer value into the m_atomic_p
    /// m_atomic_p.store(l_new_p);
    ///
    /// // atomic load of the m_atomic_p for usage
    /// l_local_p = m_atomic_p.load();
    ///
    /// \endcode
    //=========================================================================
    template<typename ValueType>
    class TAtomic
    {
        VFC_STATIC_ASSERT(vfc::TIsPOD<ValueType>::value);
        VFC_STATIC_ASSERT(sizeof(ValueType) <= sizeof(ValueType*));
        
    public:
    
        //---------------------------------------------------------------------
        /// TAtomic value default constructor.
        /// Does not initialize by design.
        //---------------------------------------------------------------------
        TAtomic() {}

        //---------------------------------------------------------------------
        /// TAtomic destructor.
        /// Does not destruct by design.
        //---------------------------------------------------------------------
        ~TAtomic() {}
   
        //---------------------------------------------------------------------
        /// TAtomic value constructor
        //---------------------------------------------------------------------
        explicit
        TAtomic(ValueType f_value);
    
        //---------------------------------------------------------------------
        /// atomic load of the stored value
        //---------------------------------------------------------------------
        ValueType load(EMemoryOrder f_order = MEMORY_ORDER_SEQ_CST) const;

        //---------------------------------------------------------------------
        /// atomic store of a new value to store
        //---------------------------------------------------------------------
        void store(ValueType f_value, EMemoryOrder f_order = MEMORY_ORDER_SEQ_CST);
        
    private: 
        TAtomic(const TAtomic&);
        const TAtomic& operator=(const TAtomic&);
    
        ValueType m_value;
    };
        
}   // namespace vfc closed

#include "vfc/core/vfc_atomic.inl"

#endif // VFC_ATOMIC_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_atomic.hpp  $
//  Revision 1.2 2016/03/30 15:53:18MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - 0005165: vfc::TAtomic should not be copyable and the Ctor should be explicit (mantis0005165)
//  Revision 1.1 2016/01/28 14:51:58MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/PC/video_mpc2/svc2pr/syssw/isw/pc_mpc2_vfc/include/vfc/core/core.pj
//=============================================================================
