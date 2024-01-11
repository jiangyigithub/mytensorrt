//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2006 by Robert Bosch GmbH. All rights reserved.
//
//  This file is property of Robert Bosch GmbH. Any unauthorised copy, use or
//  distribution is an offensive act against international law and may be
//  prosecuted under federal law. Its content is company confidential.
//=============================================================================
//  D E S C R I P T I O N
//-----------------------------------------------------------------------------
//       Projectname: vfc
//          Synopsis:
//  Target system(s): cross platform
//       Compiler(s): c++ std
//=============================================================================
//  N O T E S
//-----------------------------------------------------------------------------
//  Notes:
//=============================================================================
//  I N I T I A L   A U T H O R   I D E N T I T Y
//-----------------------------------------------------------------------------
//        Name: Thomas Jaeger
//  Department: CC/EPV2
//=============================================================================
//  R E V I S I O N   I N F O R M A T I O N
//-----------------------------------------------------------------------------
/// @file
/// @par Revision History:
///     $Source: vfc_endianess.hpp $
///     $Revision: 1.12 $
///     $Author: Jaeger Thomas (CC-DA/EPV2) (JAT2HI) $
///     $Date: 2011/05/31 15:59:20MESZ $
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

#ifndef VFC_ENDIANESS_HPP_INCLUDED
#define VFC_ENDIANESS_HPP_INCLUDED

#include "vfc/core/vfc_config.hpp"            // needed for VFC_BIG_ENDIAN
#include "vfc/core/vfc_types.hpp"            // needed for uint16_t, uint32_t, uint64_t
#include "vfc/core/vfc_type_traits.hpp"        // needed for TIsFundamental<>
#include "vfc/core/vfc_static_assert.hpp"    // needed for VFC_STATIC_ASSERT
#include "vfc/core/vfc_metaprog.hpp"            // needed for TInt2Type<>

namespace vfc
{    // namespace vfc opened

    //=========================================================================
    //  DOYGEN DEFGROUP vfc_group_core_algorithms_endianess BEGIN
    //-------------------------------------------------------------------------
    /// @defgroup vfc_group_core_algorithms_endianess Endianess
    /// @ingroup vfc_group_core_algorithms
    /// @brief endianess conversion functions.
    /// @{
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    // conditional doxygen documentation
    //! @cond VFC_DOXY_INTERN
    //-------------------------------------------------------------------------

    namespace intern
    {    // namespace intern opened

        //! token for bin endian
        struct    BigEndian        {};

        //! token for little endian
        struct    LittleEndian    {};

    #ifdef VFC_BIG_ENDIAN
        typedef    BigEndian        SystemByteOrder;
    #else
        typedef    LittleEndian    SystemByteOrder;
    #endif

        //! default swapBytes() function, declared without definition, does not compile when instantiated
        template <class T, uint32_t SizeValue>
        T    swapBytes    (T, TInt2Type<SizeValue>);

        //! overloaded swapBytes() function for sizeof(T) == 1 types
        template <class T>    inline
        T    swapBytes    (T    f_value, TInt2Type<1>)
        {
            return f_value;
        }

        //! overloaded swapBytes() function for sizeof(T) == 2 types
        template <class T>    inline
        T    swapBytes    (T    f_value, TInt2Type<2>)
        {
            uint16_t    val16 = *(reinterpret_cast<uint16_t*>(&f_value));
            val16 =        ((val16&0xFF00U) >> 8)
                    |    ((val16&0x00FFU) << 8);
            return *(reinterpret_cast<T*>(&val16));
        }

        //! overloaded swapBytes() function for sizeof(T) == 4 types
        template <class T>    inline
        T    swapBytes    (T f_value, TInt2Type<4>)
        {
            uint32_t    val32 = *(reinterpret_cast<uint32_t*>(&f_value));
            val32 =        ((val32&0xFF000000UL)>>24)
                    |    ((val32&0x00FF0000UL)>>8)
                    |    ((val32&0x0000FF00UL)<<8)
                    |    ((val32&0x000000FFUL)<<24) ;
            return *(reinterpret_cast<T*>(&val32));
        }

#ifndef    VFC_NO_INT64
        // PRQA S 476 ++
        //! overloaded swapBytes() function for sizeof(T) == 8 types
        template <class T>    inline
        T    swapBytes    (T f_value, TInt2Type<8>)
        {
            uint64_t    val64 = *(reinterpret_cast<uint64_t*>(&f_value));
            val64 =        ((val64&0xFF00000000000000ULL)>>56)
                    |    ((val64&0x00FF000000000000ULL)>>40)
                    |    ((val64&0x0000FF0000000000ULL)>>24)
                    |    ((val64&0x000000FF00000000ULL)>>8)
                    |    ((val64&0x00000000FF000000ULL)<<8)
                    |    ((val64&0x0000000000FF0000ULL)<<24)
                    |    ((val64&0x000000000000FF00ULL)<<40)
                    |    ((val64&0x00000000000000FFULL)<<56);
            return *(reinterpret_cast<T*>(&val64));
        }
        // PRQA S 476 --
#else
        //! overloaded swapBytes() function for sizeof(T) == 8 types
        template <class T>    inline
        T   swapBytes    (T f_value, TInt2Type<8>)
        {
            vfc::uint8_t* l_value_p = reinterpret_cast<vfc::uint8_t*>(&f_value);
            uint8_t l_temp_ui8 = 0;

            l_temp_ui8 = l_value_p[0];
            l_value_p[0] = l_value_p[7];
            l_value_p[7] = l_temp_ui8;

            l_temp_ui8 = l_value_p[1];
            l_value_p[1] = l_value_p[6];
            l_value_p[6] = l_temp_ui8;

            l_temp_ui8 = l_value_p[2];
            l_value_p[2] = l_value_p[5];
            l_value_p[5] = l_temp_ui8;

            l_temp_ui8 = l_value_p[3];
            l_value_p[3] = l_value_p[4];
            l_value_p[4] = l_temp_ui8;

            return *(reinterpret_cast<T*>(l_value_p));
        }

#endif

        //! overloaded swapBytes() function, for byte swapping from little to little endian order - ergo: does nothing
        template <class T>    inline
        T    swapBytes    (T f_value, LittleEndian, LittleEndian)    {    return f_value;}

        //! overloaded swapBytes() function, for byte swapping from little to big endian order
        template <class T>    inline
        T    swapBytes    (T f_value, LittleEndian, BigEndian)        {    return swapBytes(f_value,TInt2Type<sizeof(T)>());}

        //! overloaded swapBytes() function, for byte swapping from big to little endian order
        template <class T>    inline
        T    swapBytes    (T f_value, BigEndian, LittleEndian)        {    return swapBytes(f_value,TInt2Type<sizeof(T)>());}

        //! overloaded swapBytes() function, for byte swapping from big to big endian order - ergo: does nothing
        template <class T>    inline
        T    swapBytes    (T f_value, BigEndian, BigEndian)            {    return f_value;}

    }    // namespace intern closed

    //-------------------------------------------------------------------------
    //! @endcond
    // of VFC_DOXY_INTERN
    //-------------------------------------------------------------------------

    //-------------------------------------------------------------------------
    //! swaps bytes for fundamental types from one endianess to the other
    //-------------------------------------------------------------------------

    template <class T>    inline
    T    swapBytes    (T f_value)    
    {  
        VFC_STATIC_ASSERT(TIsFundamental<T>::value);  
        return intern::swapBytes(f_value,TInt2Type<sizeof(T)>());
    }

    //-------------------------------------------------------------------------
    //! converts given f_value of fundamental type from system byte order to
    //! little endian order.
    //-------------------------------------------------------------------------
    template <class T>    inline
    T    convertSystemToLittleEndian    (T f_value)    
    {   
        VFC_STATIC_ASSERT(TIsFundamental<T>::value);
        return intern::swapBytes(f_value, intern::SystemByteOrder(), intern::LittleEndian());
    }

    //-------------------------------------------------------------------------
    //! converts given f_value of fundamental type from system byte order to
    //! big endian order.
    //-------------------------------------------------------------------------
    template <class T>    inline
    T    convertSystemToBigEndian       (T f_value)    
    {   
        VFC_STATIC_ASSERT(TIsFundamental<T>::value);
        return intern::swapBytes(f_value, intern::SystemByteOrder(), intern::BigEndian());
    }

    //-------------------------------------------------------------------------
    //! converts given f_value of fundamental type from little endian order to
    //! system byte order.
    //-------------------------------------------------------------------------
    template <class T>    inline
    T    convertLittleEndianToSystem    (T f_value)    
    {   
        VFC_STATIC_ASSERT(TIsFundamental<T>::value);
        return intern::swapBytes(f_value, intern::LittleEndian(), intern::SystemByteOrder());
    }

    //-------------------------------------------------------------------------
    //! converts given f_value of fundamental type from big endian order to
    //! system byte order.
    //-------------------------------------------------------------------------
    template <class T>    inline
    T    convertBigEndianToSystem       (T f_value)    
    {   
        VFC_STATIC_ASSERT(TIsFundamental<T>::value);
        return intern::swapBytes(f_value, intern::BigEndian(), intern::SystemByteOrder());
    }

    //=========================================================================
    //  DOYGEN DEFGROUP vfc_group_core_algorithms_endianess END
    //-------------------------------------------------------------------------
    /// @}
    //-------------------------------------------------------------------------

}    // namespace vfc closed


#endif //VFC_ENDIANESS_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_endianess.hpp  $
//  Revision 1.12 2011/05/31 15:59:20MESZ Jaeger Thomas (CC-DA/EPV2) (JAT2HI) 
//  - add static asserts to the public interface functions (mantis2197)
//  Revision 1.11 2009/01/30 15:32:15MEZ Gaurav Jain (RBEI/ESB2) (gaj2kor) 
//  -Resolution of QAC++ warnings.
//  (Mantis : 0002496)
//  Revision 1.10 2008/08/29 18:28:29IST Gaurav Jain (RBEI/EAC1) (gaj2kor)
//  Addition of Byteswapping functionality for 8Byte data.
//  (Mantis :2174)
//  Revision 1.9 2007/08/02 19:19:39IST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - added conditional doxygen documentation generation of vfc::intern (mantis1758)
//  Revision 1.8 2007/07/23 09:29:41CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi)
//  - doxygen grouping (mantis1744)
//  - moved declarations to intern namespace
//  Revision 1.7 2006/11/16 14:41:15CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  - replaced tabs with 4 spaces (mantis1294)
//  Revision 1.6 2006/09/20 16:51:19CEST Muehlmann Karsten (AE-DA/ESA3) (muk2lr)
//  - change method of not compiling from static assert to declaration without definition (mantis1125)
//  Revision 1.5 2005/10/28 10:27:58CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/core/core.pj
//  Revision 1.4 2005/10/06 16:54:25CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI)
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/vfc.pj
//  Revision 1.3 2005/04/20 14:18:10CEST zvh2hi
//  added static assertions for non-fundamental types
//  Revision 1.2 2005/04/18 15:14:07CEST zvh2hi
//  cosmetics
//=============================================================================
