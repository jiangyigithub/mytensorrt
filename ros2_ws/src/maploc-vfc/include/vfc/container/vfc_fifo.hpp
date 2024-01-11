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
///     $Source: vfc_fifo.hpp $
///     $Revision: 1.3 $
///     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
///     $Date: 2016/01/28 14:51:58MEZ $
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

#ifndef VFC_FIFO_HPP_INCLUDED
#define VFC_FIFO_HPP_INCLUDED


// vfc/core includes
#include "vfc/core/vfc_types.hpp"
#include "vfc/core/vfc_static_assert.hpp"
#include "vfc/core/vfc_atomic.hpp"
#include "vfc/container/vfc_carray.hpp"         //TCArray


namespace vfc
{   //  namespace vfc opened

    //=============================================================================
    //  TFifo<>
    //-----------------------------------------------------------------------------
    //! Thread safe producer and consumer queue without using locks.
    //! TFIFO class is used for synchronized data exchange with exactly one reader and
    //! one writer thread. class Internally allocate memory for CapacityValue + 1 , that
    //! extra memory location at the end is used for end of Memory Location.
    //! both reader and writer thread can work simultaneously without any data lose.
    //! if Queue is full then happens to call push() returns false else true
    //! if Queue is empty then happens to call pop() returns false else true
    //! $Source: vfc_fifo.hpp $
    //! @see http://msmvps.com/blogs/vandooren/archive/2007/01/05/creating-a-thread-safe-producer-consumer-queue-in-c-without-using-locks.aspx
    //! @par Description:
    //! A template class representing a TFifo.
    //! Implementation of FIFO ( First In First Out ) Queue.
    //! @param ValueType        DataType to be stored
    //! @param CapacityValue    Capacity of the Fifo
    //! @ingroup                vfc_containers
    //! @author                 vmr1kor
    //=============================================================================

    template<class ValueType,
            vfc::int32_t CapacityValue >
    class TFifo
    {
        VFC_STATIC_ASSERT(0 <= CapacityValue);

    public:

        enum
        {
            FIFO_SIZE = CapacityValue   //!< The static TFifo size
                                        //!< defined at compile time
        };

        //! typedefs

        //! A type that represent the TFifo Type
        typedef TFifo<ValueType,CapacityValue>                                  fifo_type ;

        //! A type that represent the data type stored in a TFifo
        typedef ValueType                                                       value_type;
        typedef value_type*                                                     pointer;


        //---------------------------------------------------------------------
        //! Default constructor, takes no arguments.
        //! $Source: vfc_fifo.hpp $
        //! @author  vmr1kor
        //---------------------------------------------------------------------
        TFifo();


        //---------------------------------------------------------------------
        //! Default Destructor.
        //! $Source: vfc_fifo.hpp $
        //! @author  vmr1kor
        //---------------------------------------------------------------------
        ~TFifo();

        //=====================================================================================
        //! Push the element into FIFO.
        //! @par Description:
        //! push() Function returns false when FIFO is Full and Data won't be pushed into FIFO
        //! else push data into FIFO and returns true.
        //! write_ptr is pointing to next writable memory location unless queue is Full.
        //! $Source: vfc_fifo.hpp $
        //! @code
        //! vfc::int32_t value ;
        //! bool status = push(value);
        //! @endcode
        //! @return return true if push was sucessfull else false.
        //! @code
        //! (if FIFO is FULL , calling push() No change)
        //!     Start|-------------|
        //!                       |-------------|
        //!                                      |-------------|
        //!                                                    |-------------|
        //!                                                      (NO Change)
        //!
        //! (if FIFO is not FULL , calling push() add new data)
        //!     Start|-------------|
        //!                        |-------------|  ( Initial FIFO )
        //!  (push() Called two times)
        //!
        //!                                      |-------------|
        //!                                      (New Data)
        //!                                                     |-------------|
        //!                                                      (New Data)
        //! @endcode
        //! @author  vmr1kor
        //====================================================================================
        bool push(const value_type& f_param_r);


        //=============================================================================
        //! Pop the element from FIFO.
        //! @par Description:
        //! pop() Function returns false when queue is empty and read_ptr is not updated , else
        //! data is poped out and returns true.
        //! read_ptr is pointing to next readable memory location unless queue is empty.
        //! $Source: vfc_fifo.hpp $
        //! @code
        //! vfc::int32_t f_Value ;
        //! bool status = pop(f_Value);
        //! @endcode
        //! @return return true if pop was sucessfull else false.
        //! @code
        //! (FIFO With Data)
        //!     Start|-------------|
        //!                        |-------------|
        //!                                      |-------------|
        //!                                                    |-------------|  ( 4 elements )
        //! ( Calling pop() on the FIFO )
        //!
        //!                        |-------------|
        //!                                      |-------------|
        //!                                                    |-------------|  ( 3 elements )
        //! @endcode
        //! @author  vmr1kor
        //=============================================================================
        bool pop(value_type& f_param_r);

    private:

        //! Internal size needs to be one element bigger than
        //! demanded, to come around the pointer equality == buffer empty
        //! problem.
        //! Due to this, we waste one element of memory space.
        enum
        {
            INTERNAL_FIFO_SIZE = CapacityValue + 1      //!< Internal FIFO size for internal purpose ,
                                                        //!< one greater than CapacityValue
        };


        //! The start/end pointers are "const pointers" as they are not modified after the ctor.
        //! the write/read pointers are "atomic pointers" so that they are not reordered (read or written too late)
        TCArray<value_type, INTERNAL_FIFO_SIZE>    m_data;         //!<    internal datastructure
        value_type* const                          m_start_p;      //!<    fixed starting pointer of internal datastructure
        vfc::TAtomic<value_type*>                  m_write_p;      //!<    write pointer, depends on fifo fill level
        vfc::TAtomic<const value_type*>            m_read_p;       //!<    read pointer, depends on fifo fill level
        value_type* const                          m_end_p;        //!<    fixed end pointer of internal datastructure


        //---------------------------------------------------------------------
        //! Copy constructor.
        //! $Source: vfc_fifo.hpp $
        //! @author  vmr1kor
        //---------------------------------------------------------------------
        TFifo(const TFifo& f_fifoobj);

        //---------------------------------------------------------------------
        //! Assignment Operator.
        //! $Source: vfc_fifo.hpp $
        //! @author  vmr1kor
        //---------------------------------------------------------------------
        const TFifo& operator =(const TFifo& f_fifoobj);

    };


} // namespace vfc


#include "vfc/container/vfc_fifo.inl"

#endif //VFC_FIFO_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_fifo.hpp  $
//  Revision 1.3 2016/01/28 14:51:58MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - new module "synchronisation": c++11-like vfc_atomic (mantis0002164)
//  Revision 1.2 2014/05/20 11:16:31MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - TFiFo read and write pointers suffer from volatile constfusion (mantis0004480)
//  Revision 1.1 2010/08/11 17:34:58MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs2/appl_sw/versatile/pc_ivs2_vfc/devl/include/vfc/container/container.pj
//  Revision 1.17 2009/08/11 13:09:13MESZ Dhananjay N (RBEI/ESB2) (dhn1kor) 
//  - the operator= and copy constructor of TFifo class should be declared private.(mantis : 2837)
//  Revision 1.16 2009/01/16 14:47:18IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  -Replacement of array with vfc::TCArray
//  (Mantis : 0002513)
//  Revision 1.15 2008/08/08 20:26:33IST Jaeger Thomas (CC-DA/ESV1) (JAT2HI)
//  - cosmetics
//  Revision 1.14 2008/02/06 11:28:32CET Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  QAC++ Warnings are Eliminated
//  Revision 1.13 2008/01/11 16:56:38IST Vinaykumar Setty (RBIN/EAE6) (vmr1kor)
//  Copy constructor and Assignment operator added
//  Mantis Id :- 1769
//  Revision 1.12 2007/12/11 14:40:25IST Jaeger Thomas (AE-DA/ESV1) (jat2hi)
//  - some docu updates
//  Revision 1.11 2007/10/08 07:50:49CEST vmr1kor
//  Added the Image for push/pop function
//  Revision 1.10 2007/10/05 18:32:32IST vmr1kor
//  Documentation changed done (Mantis Id :- 0001769)
//  Revision 1.9 2007/09/03 11:51:26IST vmr1kor
//  - TFifo::pop() made as non-const
//  - removed mutable for m_read_p
//  - Documentation changes
//  mantis Id : 1769
//  Revision 1.8 2007/08/17 16:52:23IST vmr1kor
//  removed m_DEBUG_size and m_DEBUG_sync member variables
//  mantis 1674
//  Revision 1.5 2007/06/19 18:18:35IST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - huge rework of the fifo implementation:
//  - removed the "(m_read_p == 0) indicates buffer empty" construct as one pointer only must be changed within one thread to ensure synchronisation
//  - removed the asserts
//  - made the pointers and member data volatile to tell the compiler that they might be modified by another task
//  - incremented the internal buffer size by 1, as we need one spare element for correct buffer empty/buffer full detection
//  Revision 1.4 2007/06/18 16:36:42CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - added a lot of ecu multithread testing/debug code (to be removed again)
//  - switched off "buffer-full-detection" to provoke sync errors on ecu
//  Revision 1.3 2007/06/18 11:42:31CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - added missing includes
//  Revision 1.2 2007/06/15 14:45:02CEST Jaeger Thomas (AE-DA/ESA3) (jat2hi)
//  - discarded the fixedmempool dependency, use c-array instead
//  - modified the buffer full/empty detection
//  Revision 1.1 2007/06/15 11:49:40CEST vmr1kor
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/proposals/vfc_fifo/vfc_fifo.pj
//=============================================================================
