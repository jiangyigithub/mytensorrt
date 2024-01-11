//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2007 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorised copy, use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc
//          Synopsis:
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
///     $Source: vfc_fifo.inl $
///     $Revision: 1.7 $
///     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
///     $Date: 2016/01/28 14:52:13MEZ $
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
#include "vfc/core/vfc_assert.hpp"  // used for VFC_ASSERT()
#include "vfc/core/vfc_algorithm.hpp"  // used for contiguous_copy_n()

template<class ValueType, vfc::int32_t CapacityValue>
inline
vfc::TFifo<ValueType, CapacityValue>::TFifo() :
    m_data(),
    m_start_p(m_data.begin()),
    m_write_p(m_data.begin()),
    m_read_p(m_data.begin()),
    m_end_p(m_data.end())
{
    VFC_ENSURE(m_start_p!=0);
}


template<class ValueType, vfc::int32_t CapacityValue>
inline
vfc::TFifo<ValueType, CapacityValue>::~TFifo()
{
}

template<class ValueType, vfc::int32_t CapacityValue>
inline
bool vfc::TFifo<ValueType, CapacityValue>::push(const value_type& f_param_r)
{
    value_type* const l_write_p = m_write_p.load(vfc::MEMORY_ORDER_RELAXED);
    value_type* l_next_write_p = l_write_p + 1;
    if (l_next_write_p == m_end_p) // wraparound
    {
        l_next_write_p = m_start_p;
    }

    // buffer overflow detection
    if (l_next_write_p == m_read_p.load(vfc::MEMORY_ORDER_ACQUIRE))
    {
        return false;
    }
    else
    {
        // assing the user data
        *l_write_p = f_param_r; 

        //Assign m_write_p to next writable location
        m_write_p.store(l_next_write_p, vfc::MEMORY_ORDER_RELEASE);

        return true;
    }
}

template<class ValueType, vfc::int32_t CapacityValue>
inline
bool vfc::TFifo<ValueType, CapacityValue>::pop(value_type& f_param_r)
{
    const value_type* const l_read_p = m_read_p.load(vfc::MEMORY_ORDER_RELAXED);
    const value_type* const l_write_p = m_write_p.load(vfc::MEMORY_ORDER_ACQUIRE);
    
    // buffer empty detection
    if (l_read_p == l_write_p)
    {
        return false;
    }
    else
    {
        const value_type* l_next_read_p = l_read_p + 1;
        if (l_next_read_p == m_end_p) // wraparound
        {
            l_next_read_p = m_start_p;
        }

        // assign the user data
        f_param_r = *l_read_p;

        //Assign m_read_p to next readable location
        m_read_p.store(l_next_read_p, vfc::MEMORY_ORDER_RELEASE);

        return true;
    }
}

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fifo.inl  $
//  Revision 1.7 2016/01/28 14:52:13MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - new module "synchronisation": c++11-like vfc_atomic (mantis0002164)
//  Revision 1.5 2014/09/18 14:10:36MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - vfc_fifo.inl: missing closing " character at the end of the dmb #error statements (mantis0004710)
//  Revision 1.4 2014/05/20 11:17:16MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - TFiFo read and write pointers suffer from volatile constfusion (mantis0004480)
//  Revision 1.3 2014/05/09 10:27:00MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Extend the Fifo with memory barriers around the write and read pointers (mantis0004213)
//  Revision 1.2 2014/04/04 10:10:08MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - Extend the Fifo with memory barriers around the write and read pointers (mantis0004213)
//  Revision 1.1 2010/08/11 17:34:58MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/container/container.pj
//  Revision 1.21 2009/08/11 13:09:30MESZ Dhananjay N (RBEI/ESB2) (dhn1kor) 
//  - the operator= and copy constructor of TFifo class should be declared private.(mantis : 2837)
//  Revision 1.20 2009/01/16 14:47:19IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Replacement of array with vfc::TCArray
//  (Mantis : 0002513)
//  Revision 1.19 2008/11/03 10:28:14IST Gaurav Jain (RBEI/EAE5) (gaj2kor)
//  -Resolution of QAC++ issue (Msg : 4051).
//  (Mantis : 2343 )
//  Revision 1.18 2008/08/08 20:26:33IST Jaeger Thomas (CC-DA/ESV1) (JAT2HI)
//  - cosmetics
//  Revision 1.17 2008/02/06 11:28:31CET Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  QAC++ Warnings are Eliminated
//  Revision 1.16 2008/01/30 12:17:53IST Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  set m_end_p in the Intitialization list
//  In copy ctor , set m_end_p and m_start_p in the Intitialization list
//  In Assignment Operator , m_start_p and m_end_p must not be re-set on assignment .
//  Mantis Id :- 1769
//  Revision 1.15 2008/01/11 16:56:39IST Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  Copy constructor and Assignment operator added
//  Mantis Id :- 1769
//  Revision 1.14 2007/12/11 14:40:28IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  - some docu updates
//  Revision 1.13 2007/10/08 07:51:11CEST vmr1kor
//  removed the Image for push/pop function
//  Revision 1.12 2007/10/05 18:32:33IST vmr1kor
//  Documentation changed done (Mantis Id :- 0001769)
//  Revision 1.11 2007/09/03 11:51:42IST vmr1kor
//  - TFifo::pop() made as non-const
//  - Documentation changes
//  mantis Id : 1769
//  Revision 1.10 2007/08/21 14:17:19IST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - removed unnecessary iostream include
//  Revision 1.9 2007/08/17 13:22:39CEST vmr1kor
//  removed m_DEBUG_size and m_DEBUG_sync member variables
//  mantis
//  Revision 1.6 2007/08/06 16:25:03IST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - removed commented out VFC_REQIRE()s
//  Revision 1.5 2007/06/19 14:48:35CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - huge rework of the fifo implementation:
//  - removed the "(m_read_p == 0) indicates buffer empty" construct as one pointer only must be changed within one thread to ensure synchronisation
//  - removed the asserts
//  - made the pointers and member data volatile to tell the compiler that they might be modified by another task
//  - incremented the internal buffer size by 1, as we need one spare element for correct buffer empty/buffer full detection
//  Revision 1.4 2007/06/18 16:36:42CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - added a lot of ecu multithread testing/debug code (to be removed again)
//  - switched off "buffer-full-detection" to provoke sync errors on ecu
//  Revision 1.3 2007/06/18 11:42:54CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - commented out not-working read-on-empty detection
//  Revision 1.2 2007/06/15 14:45:03CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - discarded the fixedmempool dependency, use c-array instead
//  - modified the buffer full/empty detection
//  Revision 1.1 2007/06/15 11:49:41CEST vmr1kor
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_fifo/vfc_fifo.pj
//=============================================================================

