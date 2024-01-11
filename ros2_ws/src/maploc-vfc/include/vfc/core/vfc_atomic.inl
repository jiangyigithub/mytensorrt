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
///     $Source: vfc_atomic.inl $
///     $Revision: 1.8 $
///     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
///     $Date: 2016/11/28 09:12:16MEZ $
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

#ifdef VFC_COMPILER_AC6
#include <arm_acle.h>
#endif

#if defined (VFC_COMPILER_VISUALC)
#include <intrin.h>
#endif


#include "vfc/core/vfc_metaprog.hpp"

namespace vfc
{
    namespace intern
    {
    
        //=============================================================================
        // Fences
        //=============================================================================
    
        // generic atomic fence for sequential consistency semantic
        inline
        void generic_atomic_fence_seq_cst()
        {
#if defined (VFC_PLATFORM_IVS_ARM)
#   if defined (VFC_COMPILER_ARMRVCT)
            __schedule_barrier(); // schedule barrier to prevent reordering of the dmb instruction
            __dmb(0xf);  // full system any-any ordering
            __schedule_barrier();
#   elif defined (VFC_COMPILER_AC6)
            // no schedule barrier as with AC6 the dmb also acts
            // as a schedule barrier according to the ARM support
            __dmb(0xf); // full system any-any ordering
#   elif defined (VFC_COMPILER_GCC)
            __asm volatile ("dmb" ::: "memory");
#   else
#       error "Implement dmb memory barrier for your compiler!"
#   endif // compiler
#elif defined (VFC_PLATFORM_IVS_ARM64)
#   if defined (VFC_COMPILER_AC6)
            // no schedule barrier as with AC6 the dmb also acts
            // as a schedule barrier according to the ARM support
            __dmb(0xf); // full system any-any ordering
#   elif defined (VFC_COMPILER_GCC)    
            __asm volatile ("dmb sy" ::: "memory");                
#   else
#       error "Implement dmb memory barrier for your compiler!"
#   endif // compiler
#elif defined (VFC_PLATFORM_WIN32)
#   if defined (VFC_COMPILER_VISUALC)
            _mm_mfence();
#   elif defined (VFC_COMPILER_GCC)            
            __asm  volatile ("mfence" ::: "memory");
#   else
#       error "Implement memory barrier for your compiler!"
#   endif // Compiler
#elif defined (VFC_PLATFORM_LINUX)
#   if defined(VFC_COMPILER_GCC)
#       if defined (VFC_PROCESSOR_IX86_64)
            __asm volatile ("mfence" ::: "memory");
#       elif defined (VFC_PROCESSOR_ARMV6)
            __asm volatile("mcr p15, 0, %[zero], c7, c10, 5" :: [zero] "r"(0) : "memory");
#       elif defined (VFC_PROCESSOR_ARM)
            __asm volatile ("dmb" ::: "memory");
#       elif defined (VFC_PROCESSOR_ARM64)
            __asm volatile ("dmb sy" ::: "memory");            
#       else
#           error "Implement memory barrier for your processor."            
#       endif
#   else
#       error "Implement memory barrier for your compiler."
#   endif
#elif defined (VFC_PLATFORM_EPPC)
#   if defined (VFC_COMPILER_DIAB)
        __asm volatile ("msync");
#   elif defined(VFC_COMPILER_GHS)
        asm ("msync");
#   else
#       error "Implement memory barrier for your compiler."
#   endif
#elif defined (VFC_PLATFORM_IVS_TRICORE)
#   if defined (VFC_COMPILER_HIGHTEC)
        __asm volatile ("dsync");
#   elif defined (VFC_COMPILER_GHS)
        __asm volatile ("dsync");
#   else
#       error "Implement memory barrier for your compiler."
#   endif
#else // catch-all
#   error "Implement memory barrier for your platform."
#endif // Target                        
        }

        // generic atomic fence for acquire/release semantics
        inline
        void generic_atomic_fence_acq_rel()
        {
#if defined (VFC_PLATFORM_IVS_ARM) || defined (VFC_PLATFORM_IVS_ARM64)
            generic_atomic_fence_seq_cst(); // just forward to seq_cst
#elif defined (VFC_PLATFORM_EPPC)
#   if defined (VFC_COMPILER_DIAB)
        __asm volatile ("msync");
#   elif defined(VFC_COMPILER_GHS)
        asm ("msync");
#   else
#       error "Implement memory barrier for your compiler."
#   endif
#elif defined (VFC_PLATFORM_WIN32)
// No barrier necessary on x86 target for acq/rel semantics.
#elif defined (VFC_PLATFORM_LINUX)
#   if defined(VFC_COMPILER_GCC)
#       if defined (VFC_PROCESSOR_IX86_64)
// No barrier necessary on x86 target for acq/rel semantics.
#       elif defined (VFC_PROCESSOR_ARMV6)
            generic_atomic_fence_seq_cst(); // just forward to seq_cst
#       elif defined (VFC_PROCESSOR_ARM)
            generic_atomic_fence_seq_cst(); // just forward to seq_cst
#       elif defined (VFC_PROCESSOR_ARM64)
            generic_atomic_fence_seq_cst(); // just forward to seq_cst            
#       else
#           error "Implement memory barrier for your platform."            
#       endif
#   else
#       error "Implement memory barrier for your compiler."
#   endif
#elif defined (VFC_PLATFORM_IVS_TRICORE)
#   if defined (VFC_COMPILER_HIGHTEC)
        generic_atomic_fence_seq_cst(); // just forward to seq_cst
#   elif defined (VFC_COMPILER_GHS)
        generic_atomic_fence_seq_cst(); // just forward to seq_cst
#   else
#       error "Implement memory barrier for your compiler."
#   endif
#else
#   error "Implement memory barrier for your platform."
#endif // Target  
        }
        
        // ==============================================================
        
        inline
        void atomic_fence_consume()
        {
             generic_atomic_fence_acq_rel();
        }

        inline
        void atomic_fence_acquire()
        {
            generic_atomic_fence_acq_rel();
        }

        inline
        void atomic_fence_release()
        {
            generic_atomic_fence_acq_rel();
        }

        inline
        void atomic_fence_acq_rel()
        {
            generic_atomic_fence_acq_rel();
        }

        inline
        void atomic_fence_seq_cst()
        {
            generic_atomic_fence_seq_cst();
        }        
        
        //=============================================================================
        // Stores and Loads
        //=============================================================================

#if (defined (VFC_PLATFORM_IVS_ARM64) && defined(VFC_COMPILER_AC6)) || (defined (VFC_PLATFORM_LINUX) && defined (VFC_COMPILER_GCC) && defined (VFC_PROCESSOR_ARM64)) || (defined (VFC_PLATFORM_IVS_ARM64) && defined (VFC_COMPILER_GCC) && defined (VFC_PROCESSOR_ARM64))

        //=============================================================================
        // Store

        template<vfc::int32_t DatasizeValue>
        struct TAtomicStlrAccess
        {
            template<class ValueType>
            static void armv8_atomic_stlr_sized(volatile ValueType* f_atom_p, ValueType f_value);
        };

        template<>
        struct TAtomicStlrAccess<8>
        {
            template<class ValueType>
            static void armv8_atomic_stlr_sized(volatile ValueType* f_atom_p, ValueType f_value)
            {
                __asm volatile ("stlr %x1, %0" : "=Q" (*f_atom_p) : "r" (f_value) : "memory");
            }
        };
        template<>
        struct TAtomicStlrAccess<4>
        {
            template<class ValueType>
            static void armv8_atomic_stlr_sized(volatile ValueType* f_atom_p, ValueType f_value)
            {
                __asm volatile ("stlr %w1, %0" : "=Q" (*f_atom_p) : "r" (f_value) : "memory");
            }
        };
        template<>
        struct TAtomicStlrAccess<2>
        {
            template<class ValueType>
            static void armv8_atomic_stlr_sized(volatile ValueType* f_atom_p, ValueType f_value)
            {
                __asm volatile ("stlrh %w1, %0" : "=Q" (*f_atom_p) : "r" (f_value) : "memory");
            }
        };
        template<>
        struct TAtomicStlrAccess<1>
        {
            template<class ValueType>
            static void armv8_atomic_stlr_sized(volatile ValueType* f_atom_p, ValueType f_value)
            {
                __asm volatile ("stlrb %w1, %0" : "=Q" (*f_atom_p) : "r" (f_value) : "memory");
            }
        };


        template<typename ValueType> inline
        void armv8_atomic_stlr(volatile ValueType* f_atom_p, ValueType f_value)
        {
            TAtomicStlrAccess<sizeof(ValueType)>::armv8_atomic_stlr_sized(f_atom_p, f_value);
        }

        //=============================================================================
        // Load

        template<vfc::int32_t DatasizeValue>
        struct TAtomicLdarAccess
        {
            template<class ValueType>
            static ValueType armv8_atomic_ldar_sized(volatile ValueType* f_atom_p);
        };

        template<>
        struct TAtomicLdarAccess<8>
        {
            template<class ValueType>
            static ValueType armv8_atomic_ldar_sized(volatile ValueType* f_atom_p)
            {
                typename vfc::TRemoveCV<ValueType>::type ret = 0;
                __asm volatile ("ldar %x0, %1" : "=r" (ret) : "Q" (*f_atom_p) : "memory");
                return ret;
            }
        };
        template<>
        struct TAtomicLdarAccess<4>
        {
            template<class ValueType>
            static ValueType armv8_atomic_ldar_sized(volatile ValueType* f_atom_p)
            {
                typename vfc::TRemoveCV<ValueType>::type ret = 0;
                __asm volatile ("ldar %w0, %1" : "=r" (ret) : "Q" (*f_atom_p) : "memory");
                return ret;
            }
        };
        template<>
        struct TAtomicLdarAccess<2>
        {
            template<class ValueType>
            static ValueType armv8_atomic_ldar_sized(volatile ValueType* f_atom_p)
            {
                typename vfc::TRemoveCV<ValueType>::type ret = 0;
                __asm volatile ("ldarh %w0, %1" : "=r" (ret) : "Q" (*f_atom_p) : "memory");
                return ret;
            }
        };
        template<>
        struct TAtomicLdarAccess<1>
        {
            template<class ValueType>
            static ValueType armv8_atomic_ldar_sized(volatile ValueType* f_atom_p)
            {
                typename vfc::TRemoveCV<ValueType>::type ret = 0;
                __asm volatile ("ldarb %w0, %1" : "=r" (ret) : "Q" (*f_atom_p) : "memory");
                return ret;
            }
        };

        template<typename ValueType> inline
        ValueType armv8_atomic_ldar(volatile ValueType* f_atom_p)
        {
            typename vfc::TRemoveCV<ValueType>::type ret = 0;
            ret = TAtomicLdarAccess<sizeof(ValueType)>::armv8_atomic_ldar_sized(f_atom_p);
            return ret;        
        }

#endif    

    
#if defined (VFC_PLATFORM_EPPC) && defined(VFC_COMPILER_DIAB)

asm  vfc::int32_t ppc_load_atomic_word(volatile void* f_param) 
{ 
% reg f_param; lab _ld_loop 

_ld_loop:
    lwz	r3, 0(f_param) 
    cmpw	r3, f_param
    bne	_ld_loop 
    isync
} 
asm  vfc::int16_t ppc_load_atomic_halfword(volatile void* f_param)
{
% reg f_param; lab _ld_loop 

_ld_loop:
    lhz	r3, 0(f_param) 
    cmpw	r3, f_param
    bne	_ld_loop 
    isync
}
asm  vfc::int8_t ppc_load_atomic_byte(volatile void* f_param)
{
% reg f_param; lab _ld_loop 

_ld_loop:
    lbz	r3, 0(f_param) 
    cmpw	r3, f_param
    bne	_ld_loop 
    isync
}

#endif // #if defined (VFC_PLATFORM_EPPC) && defined(VFC_COMPILER_DIAB)

#if defined (VFC_PLATFORM_EPPC) && defined(VFC_COMPILER_GHS)

asm  vfc::int32_t ppc_load_atomic_word(volatile void* f_param) 
{ 
% reg f_param; lab _ld_loop 

_ld_loop:
    e_lwz	r3, 0(f_param) 
    cmpw	r3, f_param
    se_bne	_ld_loop 
    se_isync
} 
asm  vfc::int16_t ppc_load_atomic_halfword(volatile void* f_param) 
{ 
% reg f_param; lab _ld_loop 

_ld_loop:
    e_lhz	r3, 0(f_param) 
    cmpw	r3, f_param
    se_bne	_ld_loop 
    se_isync
} 
asm  vfc::int8_t ppc_load_atomic_byte(volatile void* f_param) 
{ 
% reg f_param; lab _ld_loop 

_ld_loop:
    e_lbz	r3, 0(f_param) 
    cmpw	r3, f_param
    se_bne	_ld_loop 
    se_isync
} 

#endif // if defined (VFC_PLATFORM_EPPC) && defined(VFC_COMPILER_GHS)


#if defined (VFC_PLATFORM_EPPC) && (defined(VFC_COMPILER_DIAB) || defined(VFC_COMPILER_GHS))

        template<vfc::int32_t DatasizeValue>
        struct TAtomicLoadAccess
        {
            template<class ValueType>
            static ValueType atomic_load_sized(volatile ValueType* f_atom_p);
        };

        template<>
        struct TAtomicLoadAccess<4>
        {
            template<class ValueType>
            static ValueType atomic_load_sized(volatile ValueType* f_atom_p)
            {
                typename vfc::TRemoveCV<ValueType>::type ret = 0;
                ret = (typename vfc::TRemoveCV<ValueType>::type)(ppc_load_atomic_word(const_cast<vfc::TRemoveCV<ValueType>::type*>(f_atom_p)));
                return ret;
            }
        };
        template<>
        struct TAtomicLoadAccess<2>
        {
            template<class ValueType>
            static ValueType atomic_load_sized(volatile ValueType* f_atom_p)
            {
                typename vfc::TRemoveCV<ValueType>::type ret = 0;
                ret = (typename vfc::TRemoveCV<ValueType>::type)(ppc_load_atomic_halfword(const_cast<vfc::TRemoveCV<ValueType>::type*>(f_atom_p)));
                return ret;
            }
        };
        template<>
        struct TAtomicLoadAccess<1>
        {
            template<class ValueType>
            static ValueType atomic_load_sized(volatile ValueType* f_atom_p)
            {
                typename vfc::TRemoveCV<ValueType>::type ret = 0;
                ret = (typename vfc::TRemoveCV<ValueType>::type)(ppc_load_atomic_byte(const_cast<vfc::TRemoveCV<ValueType>::type*>(f_atom_p)));
                return ret;
            }
        };

        template<typename ValueType> inline
        ValueType ppc_load_atomic_generic(volatile ValueType* f_atom_p)
        {
            typename vfc::TRemoveCV<ValueType>::type ret = 0;
            ret = TAtomicLoadAccess<sizeof(ValueType)>::atomic_load_sized(f_atom_p);
            return ret;
        }
    
#endif // #if defined (VFC_PLATFORM_EPPC) && (defined(VFC_COMPILER_DIAB) || defined(VFC_COMPILER_GHS)) 
 
        template<typename ValueType> inline
        void atomic_store_release(volatile ValueType* f_atom_p, ValueType f_value)
        {
#if defined (VFC_PLATFORM_IVS_ARM)
#   if defined(VFC_COMPILER_ARMRVCT)
            atomic_fence_acq_rel();
            *f_atom_p = f_value;
#   elif defined(VFC_COMPILER_AC6)
            atomic_fence_acq_rel();
            *f_atom_p = f_value; 
#   elif defined(VFC_COMPILER_GCC)
            atomic_fence_acq_rel();
            *f_atom_p = f_value;
#   else
#   error "Implement store-release semantic for your compiler."
#   endif     
#elif defined (VFC_PLATFORM_IVS_ARM64)
#   if defined(VFC_COMPILER_AC6)    
            armv8_atomic_stlr(f_atom_p, f_value);
#   elif defined (VFC_COMPILER_GCC)
            armv8_atomic_stlr(f_atom_p, f_value);
#   else            
#   error "Implement store-release semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_WIN32)                   
#   if defined(VFC_COMPILER_VISUALC)
            // compiler barrier for msvc
            _ReadWriteBarrier();
            *f_atom_p = f_value;
#   elif defined(VFC_COMPILER_GCC)
            // compiler barrier for gcc
            asm volatile("" ::: "memory");
            *f_atom_p = f_value;
#   else            
#       error "Implement store-release semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_LINUX)
#   if defined(VFC_COMPILER_GCC)
#       if defined (VFC_PROCESSOR_IX86_64)
            // compiler barrier for gcc
            asm volatile("" ::: "memory");
            *f_atom_p = f_value;
#       elif defined (VFC_PROCESSOR_ARM)
            atomic_fence_acq_rel();
            *f_atom_p = f_value;
#       elif defined (VFC_PROCESSOR_ARM64)
            armv8_atomic_stlr(f_atom_p, f_value);           
#       else
#           error "Implement load-seq_cst semantic for your processor."            
#       endif
#   else
#       error "Implement load-seq_cst semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_EPPC)
#   if defined (VFC_COMPILER_DIAB)
            atomic_fence_acq_rel();
            *f_atom_p = f_value;
#   elif defined(VFC_COMPILER_GHS)
            atomic_fence_acq_rel();
            *f_atom_p = f_value;           
#   else
#       error "Implement store-release semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_IVS_TRICORE)
#   if defined (VFC_COMPILER_HIGHTEC)
            atomic_fence_acq_rel();
            *f_atom_p = f_value;
            atomic_fence_acq_rel();
            __asm volatile ("isync");
#   elif defined (VFC_COMPILER_GHS)
            atomic_fence_acq_rel();
            *f_atom_p = f_value;
            atomic_fence_acq_rel();
            __asm volatile ("isync");
#   else
#       error "Implement store-release semantic for your compiler."
#   endif
#else // catch-all
#   error "Implement store-release semantic for your platform."
#endif // Target
        }
        
        template<typename ValueType> inline
        void atomic_store_seq_cst(volatile ValueType* f_atom_p, ValueType f_value)
        {
#if defined (VFC_PLATFORM_IVS_ARM)
#   if defined(VFC_COMPILER_ARMRVCT)
            atomic_fence_seq_cst();
            *f_atom_p = f_value;
            atomic_fence_seq_cst(); 
#   elif defined(VFC_COMPILER_AC6)
            atomic_fence_seq_cst();
            *f_atom_p = f_value;       
            atomic_fence_seq_cst(); 
#   elif defined(VFC_COMPILER_GCC)
            atomic_fence_seq_cst();        
            *f_atom_p = f_value;
            atomic_fence_seq_cst();
#   else
#   error "Implement store-sequential consistency semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_IVS_ARM64)
#   if defined(VFC_COMPILER_AC6)      
            armv8_atomic_stlr(f_atom_p, f_value);
#   elif defined(VFC_COMPILER_GCC)            
            armv8_atomic_stlr(f_atom_p, f_value);
#   else
#   error "Implement store-sequential consistency semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_WIN32)                   
#   if defined(VFC_COMPILER_VISUALC)
            *f_atom_p = f_value;
            atomic_fence_seq_cst();   
#   elif defined(VFC_COMPILER_GCC)
            *f_atom_p = f_value;
            atomic_fence_seq_cst();                      
#   else
#   error "Implement store-sequential consistency semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_LINUX)
#   if defined(VFC_COMPILER_GCC)
#       if defined (VFC_PROCESSOR_IX86_64)
            *f_atom_p = f_value;
            atomic_fence_seq_cst();       
#       elif defined (VFC_PROCESSOR_ARM)                           
            atomic_fence_seq_cst();        
            *f_atom_p = f_value;
            atomic_fence_seq_cst();
#       elif defined (VFC_PROCESSOR_ARM64)                           
            armv8_atomic_stlr(f_atom_p, f_value);            
#       else
#           error "Implement load-seq_cst semantic for your processor."            
#       endif
#   else
#       error "Implement load-seq_cst semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_EPPC)
#   if defined (VFC_COMPILER_DIAB)
            atomic_fence_seq_cst();
            *f_atom_p = f_value;
#   elif defined (VFC_COMPILER_GHS)
            atomic_fence_seq_cst();
            *f_atom_p = f_value;
#   else
#       error "Implement store-sequential consistency semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_IVS_TRICORE)
#   if defined (VFC_COMPILER_HIGHTEC)
            atomic_fence_seq_cst();
            *f_atom_p = f_value;
            atomic_fence_seq_cst();
            __asm volatile ("isync");
#   elif defined (VFC_COMPILER_GHS)
            atomic_fence_seq_cst();
            *f_atom_p = f_value;
            atomic_fence_seq_cst();
            __asm volatile ("isync");
#   else
#       error "Implement store-sequential consistency semantic for your compiler."
#   endif
#else // catch-all
#   error "Implement store-sequential consistency semantic for your platform."
#endif // Target            
        }
        
        
        template<typename ValueType> inline
        ValueType atomic_load_acquire(volatile ValueType* f_atom_p)
        {
#if defined (VFC_PLATFORM_IVS_ARM)            
#   if defined(VFC_COMPILER_ARMRVCT)
            typename vfc::TRemoveCV<ValueType>::type ret = 0;
            ret = *f_atom_p; 
            atomic_fence_acq_rel(); 
            return ret;  
#   elif defined(VFC_COMPILER_AC6)
            typename vfc::TRemoveCV<ValueType>::type ret = 0;
            ret = *f_atom_p; 
            atomic_fence_acq_rel(); 
            return ret;  
#   elif defined(VFC_COMPILER_GCC)
            typename vfc::TRemoveCV<ValueType>::type ret = 0;
            ret = *f_atom_p; 
            atomic_fence_acq_rel(); 
            return ret;              
#   else
#       error "Implement load-aqcuire semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_IVS_ARM64)
#   if defined(VFC_COMPILER_AC6)   
            return armv8_atomic_ldar(f_atom_p);
#   elif defined(VFC_COMPILER_GCC)
            return armv8_atomic_ldar(f_atom_p);
#   else
#       error "Implement load-aqcuire semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_WIN32)
#   if defined(VFC_COMPILER_VISUALC)
            ValueType ret = *f_atom_p;
            // compiler barrier for msvc
            _ReadWriteBarrier();
            return ret;
#   elif defined(VFC_COMPILER_GCC)
            ValueType ret = *f_atom_p;
            // compiler barrier for gcc
            asm volatile("" ::: "memory");
            return ret;
#   else
#       error "Implement load-aqcuire semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_LINUX)
#   if defined(VFC_COMPILER_GCC)
#       if defined (VFC_PROCESSOR_IX86_64)
            ValueType ret = *f_atom_p;
            // compiler barrier for gcc
            asm volatile("" ::: "memory");
            return ret;
#       elif defined (VFC_PROCESSOR_ARM)            
            typename vfc::TRemoveCV<ValueType>::type ret = 0;
            ret = *f_atom_p; 
            atomic_fence_acq_rel(); 
            return ret;
#       elif defined (VFC_PROCESSOR_ARM64)            
            return armv8_atomic_ldar(f_atom_p);                           
#       else
#           error "Implement load-seq_cst semantic for your processor."            
#       endif
#   else
#       error "Implement load-seq_cst semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_EPPC)
#   if defined (VFC_COMPILER_DIAB)
        return ppc_load_atomic_generic(f_atom_p);
#   elif defined (VFC_COMPILER_GHS)
        return ppc_load_atomic_generic(f_atom_p);
#   else
#       error "Implement load-aqcuire semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_IVS_TRICORE)
#   if defined (VFC_COMPILER_HIGHTEC)
            ValueType ret = *f_atom_p;
            return ret;
#   elif defined (VFC_COMPILER_GHS)
            ValueType ret = *f_atom_p;
            return ret;
#   else
#       error "Implement load-aqcuire semantic for your compiler."
#   endif
#else
#   error "Implement load-aqcuire semantic for your platform."            
#endif            
        }        
        
        template<typename ValueType> inline
        ValueType atomic_load_seq_cst(volatile ValueType* f_atom_p)
        {
#if defined (VFC_PLATFORM_IVS_ARM)            
#   if defined(VFC_COMPILER_ARMRVCT)
            typename vfc::TRemoveCV<ValueType>::type ret = 0;
            atomic_fence_seq_cst();
            ret = *f_atom_p; 
            atomic_fence_seq_cst();  
            return ret;
#   elif defined(VFC_COMPILER_AC6)
            typename vfc::TRemoveCV<ValueType>::type ret = 0;
            atomic_fence_seq_cst();
            ret = *f_atom_p; 
            atomic_fence_seq_cst();  
            return ret;
#   elif  defined(VFC_COMPILER_GCC)
            typename vfc::TRemoveCV<ValueType>::type ret = 0;
            atomic_fence_seq_cst();
            ret = *f_atom_p; 
            atomic_fence_seq_cst();  
            return ret;                    
#   else
#       error "Implement load-seq_cst semantic for your compiler."
#   endif   
#elif defined (VFC_PLATFORM_IVS_ARM64)
#   if defined(VFC_COMPILER_AC6)   
            return armv8_atomic_ldar(f_atom_p);
#   elif defined(VFC_COMPILER_GCC)
            return armv8_atomic_ldar(f_atom_p);
#   else
#       error "Implement load-seq_cst semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_EPPC)
#   if defined (VFC_COMPILER_DIAB)
        atomic_fence_seq_cst();
        return ppc_load_atomic_generic(f_atom_p);
#   elif defined (VFC_COMPILER_GHS)
        atomic_fence_seq_cst();
        return ppc_load_atomic_generic(f_atom_p);
#   else
#       error "Implement load-seq_cst semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_WIN32)
#   if defined(VFC_COMPILER_VISUALC)
            ValueType ret = *f_atom_p;
            // compiler barrier for msvc
            _ReadWriteBarrier();
            return ret;
#   elif defined(VFC_COMPILER_GCC)
            ValueType ret = *f_atom_p;
            // compiler barrier for gcc
            asm volatile("" ::: "memory");
            return ret;
#   else
#       error "Implement load-seq_cst semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_LINUX)
#   if defined(VFC_COMPILER_GCC)
#       if defined (VFC_PROCESSOR_IX86_64)
            ValueType ret = *f_atom_p;
            // compiler barrier for gcc
            asm volatile("" ::: "memory");
            return ret;
#       elif defined (VFC_PROCESSOR_ARM)            
            typename vfc::TRemoveCV<ValueType>::type ret = 0;
            atomic_fence_seq_cst();
            ret = *f_atom_p; 
            atomic_fence_seq_cst();  
            return ret;
#       elif defined (VFC_PROCESSOR_ARM64)            
            return armv8_atomic_ldar(f_atom_p);                                
#       else
#           error "Implement load-seq_cst semantic for your processor."            
#       endif
#   else
#       error "Implement load-seq_cst semantic for your compiler."
#   endif
#elif defined (VFC_PLATFORM_IVS_TRICORE)
#   if defined (VFC_COMPILER_HIGHTEC)
            ValueType ret = *f_atom_p;
            return ret;
#   elif defined (VFC_COMPILER_GHS)
            ValueType ret = *f_atom_p;
            return ret;
#   else
#       error "Implement load-seq_cst semantic for your compiler."
#   endif
#else
#   error "Implement load-seq_cst semantic for your platform."            
#endif           
        }         
        
        template<typename ValueType> inline
        void atomic_store(volatile ValueType* f_atom_p, ValueType f_value, vfc::EMemoryOrder f_memorder)
        {
            switch (f_memorder)
            {
                case MEMORY_ORDER_RELAXED:
                {
                    *f_atom_p = f_value;
                    break;
                }
                case MEMORY_ORDER_RELEASE:
                {
                    atomic_store_release(f_atom_p, f_value);
                    break;
                }
                case MEMORY_ORDER_SEQ_CST:
                {
                    atomic_store_seq_cst(f_atom_p, f_value);
                    break;
                }
                default:
                {
                    VFC_ASSERT(false); // invalid memory order for store
                }
            }
        }
        
        template<typename ValueType> inline
        ValueType atomic_load(volatile const ValueType* const f_atom_p, vfc::EMemoryOrder f_memorder)
        {
            ValueType ret = 0;
            
            switch (f_memorder)
            {
                case MEMORY_ORDER_RELAXED:
                {
                    ret = *f_atom_p;
                    break;
                }
                case MEMORY_ORDER_CONSUME:  // not optimized yet, use acquire
                case MEMORY_ORDER_ACQUIRE:
                {
                    ret = atomic_load_acquire(f_atom_p);
                    break;
                }
                case MEMORY_ORDER_SEQ_CST:
                {
                    ret = atomic_load_seq_cst(f_atom_p);
                    break;
                }
                default:
                {
                    VFC_ASSERT(false); // invalid memory order for store
                    break;
                }
            }
            
            return ret;
        }
    } // namespace intern closed
} // namespace vfc closed
    

inline
void vfc::atomic_thread_fence(EMemoryOrder f_memorder)
{
        switch (f_memorder)
            {
                case MEMORY_ORDER_RELAXED:
                {
                    // no barrier on no architecture
                    break;
                }
                case MEMORY_ORDER_CONSUME:
                {
                    intern::atomic_fence_consume();
                    break;
                }
                case MEMORY_ORDER_ACQUIRE:
                {
                    intern::atomic_fence_acquire();
                    break;
                }
                case MEMORY_ORDER_RELEASE:
                {
                    intern::atomic_fence_release();
                    break;
                }
                case MEMORY_ORDER_ACQ_REL:
                {
                    intern::atomic_fence_acq_rel();
                    break;
                }                
                case MEMORY_ORDER_SEQ_CST:
                {
                    intern::atomic_fence_seq_cst();
                    break;
                }
                default:
                {
                    VFC_ASSERT(false); // invalid memory order for fence
                    break;
                }
            }     
}  

template<typename ValueType> inline
vfc::TAtomic<ValueType>::TAtomic(ValueType f_value) :
    m_value(f_value)
    {}

template<typename ValueType> inline
ValueType vfc::TAtomic<ValueType>::load(EMemoryOrder f_order) const
{
    return vfc::intern::atomic_load(&m_value, f_order);
}         

template<typename ValueType> inline
void vfc::TAtomic<ValueType>::store(ValueType f_value, EMemoryOrder f_order)
{
    vfc::intern::atomic_store(&m_value, f_value, f_order);
}   

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_atomic.inl  $
//  Revision 1.8 2016/11/28 09:12:16MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Support ARMv8 target with GCC compiler (mantis0005365)
//  - vfc_atomic: unaligned issues with data types smaller than system word width (mantis0005375)
//  Revision 1.7 2016/07/06 08:10:14MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Support GHS compiler for Tricore target (mantis0005105)
//  Revision 1.6 2016/06/30 16:30:35MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Support Hightec/GCC compiler for Tricore target (mantis0005108)
//  Revision 1.5 2016/06/30 15:58:58MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Support GHS 201419 with PPC target (mantis0005266)
//  Revision 1.4 2016/04/07 11:16:40MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - Update Diab compiler settings to reflect latest compiler version and atomic usage (mantis0005169)
//  Revision 1.3 2016/04/01 18:05:06MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - vfc_atomic: support linux/gcc with ARMv7 and x86 targets (mantis0005162)
//  - Support the ARMv6 target with Linux/GCC (Ras Pi) (mantis0005172)
//  Revision 1.2 2016/03/30 15:53:18MESZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  - 0005165: vfc::TAtomic should not be copyable and the Ctor should be explicit (mantis0005165)
//  - Support ARMv7 target with AC6 (mantis0005109)
//  - vfc_atomic: support linux/gcc with ARMv7 and x86 targets (mantis0005162)
//  Revision 1.1 2016/01/28 14:51:58MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/PC/video_mpc2/svc2pr/syssw/isw/pc_mpc2_vfc/include/vfc/core/core.pj
//=============================================================================
