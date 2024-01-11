//=============================================================================
//  C O P Y R I G H T
//-----------------------------------------------------------------------------
//  Copyright (c) 2007 by Robert Bosch GmbH. All rights reserved.
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
///     $Source: vfc_config.hpp $
///     $Revision: 1.15 $
///     $Author: Zitzewitz Henning von (CR/AEM5) (zvh2hi) $
///     $Date: 2007/07/23 09:20:17MESZ $
///     $Locker:  $
///     $Name: 0032 RC1 Hello KW07  $
///     $State: In_Development $
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

#ifndef VFC_CONFIG_HPP_INCLUDED
#define VFC_CONFIG_HPP_INCLUDED

//=============================================================================
//  DOXYGEN ADDTOGROUP vfc_group_core_config
//-----------------------------------------------------------------------------
/// @addtogroup vfc_group_core_config Configuration
/// @brief Configure vfc for your platform.
/// vfc comes already configured for most common compilers and platforms; 
/// you should be able to use vfc <b>"as is"</b>. 
/// Using vfc "as is" without trying to reconfigure is the recommended method 
/// for using vfc. 
/// vfc library users can request support for additional compilers or platforms 
/// by visiting our Mantis project (@b http://aemweb.hi.de.bosch.com/mantis/main_page.php )
/// and submitting a support request. 
///
/// vfc library implementations access configuration macros via 
///  <tt>@#include "vfc/core/vfc_config.hpp"</tt>.
///
/// While vfc library users are not required to include that file directly, 
/// or use those configuration macros, such use is acceptable.  
/// The configuration macros are documented as to their purpose, usage, and 
/// limitations which makes them usable by both vfc library and user code.
///
/// vfc informational or helper macros are designed for use by vfc users as well 
/// as for our own internal use.  
/// Note however, that the feature test (@c _HAS_) and defect test (@c _NO_ )
/// macros were designed for internal use by vfc modules, not user code, so 
/// they can change at any time (though no gratuitous changes are made to them). 
/// 
/// By setting various macros on the compiler command line the vfc configuration 
/// setup can be optimised in a variety of ways.
/// vfc's configuration is structured so that the user-configuration is included 
/// first (defaulting to <tt>"vfc/core/config/vfc_user_cfg.hpp"</tt> if @ref VFC_USER_CONFIG 
/// is not defined).@n 
/// This sets up any user-defined policies, and gives the user-configuration a 
/// chance to influence what happens next.@n
/// Next the compiler and platform configuration files are included. 
/// These are included via macros ( @ref VFC_COMPILER_CONFIG etc, see @ref vfc_group_core_config_user ), 
/// and if the corresponding macro is undefined then a separate header that detects 
/// which compiler/platform is in use is included in order to set these. 
/// The config can be told to ignore these headers altogether if the corresponding 
/// @c VFC_NO_XXX macro is set (for example @ref VFC_NO_COMPILER_CONFIG to disable including
/// any compiler configuration file - see user settable macros).@n
/// Finally the vfc configuration header, includes <tt>"vfc/core/config/vfc_config_suffix.hpp"</tt>; 
/// this header contains any boiler plate configuration code - for example where 
/// one vfc macro being set implies that another must be set also.
///
/// @par Supported Platforms
/// With this release following platforms are supported:
/// -#  Win32
/// -#  AE-DA IVS (aka EPPC)
/// -#  Linux
/// .
/// @par Supported Compilers
/// Following compilers are supported:
/// -#  MS Visual C++ 8.0 (MS Visual Studio 2005)
/// -#  MS Visual C++ 7.1 (MS ViSual Studio 2003)
/// -#  Metrowerks CodeWarrior 9.3 Win32
/// -#  Metrowerks CodeWarrior 8.1 EPPC
/// -#  Metrowerks CodeWarrior 8.5 EPPC
/// -#  WindRiver C/C++ (aka Diab) EPPC
/// -#  GCC 3.44 Win32 (Cygwin)
/// -#  GCC 4.10 Linux 
/// .
/// @par Supported Processors
/// -#  Intel x86 32bit
/// -#  Intel x86 64bit (Linux)
/// -#  PowerPC 
/// .
//=============================================================================


//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_core_config_user
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_core_config_user User Configuration
/// @ingroup vfc_group_core_config
/// @brief configuration-options that represent user-choice.
/// There are some configuration-options that represent user choices, rather 
/// than compiler defects or platform specific options. 
/// 
/// @par VFC_USER_CONFIG @anchor VFC_USER_CONFIG
/// When defined, it should point to the name of the user configuration file 
/// to include prior to any vfc configuration files. 
/// When not defined, defaults to <tt>"vfc/core/config/vfc_user_cfg.hpp"</tt>.
/// 
/// @par VFC_COMPILER_CONFIG @anchor VFC_COMPILER_CONFIG
/// When defined, it should point to the name of the compiler configuration file 
/// to use. Defining this cuts out the compiler selection logic, and eliminates 
/// the dependency on the header containing that logic. 
/// For example if you are using gcc, then you could define @ref VFC_COMPILER_CONFIG to 
/// <tt>"vfc/core/config/compiler/vfc_gcc.hpp"</tt>.
///
/// @par VFC_PLATFORM_CONFIG @anchor VFC_PLATFORM_CONFIG
/// When defined, it should point to the name of the platform configuration file 
/// to use. Defining this cuts out the platform selection logic, and eliminates 
/// the dependency on the header containing that logic. 
/// For example if you are compiling on linux, then you could define 
/// @ref VFC_PLATFORM_CONFIG to <tt>"vfc/core/config/platform/vfc_linux.hpp"</tt>.
///
/// @par VFC_NO_CONFIG @anchor VFC_NO_CONFIG
/// Equivalent to defining all of @ref VFC_NO_COMPILER_CONFIG and @ref VFC_NO_PLATFORM_CONFIG.
///
/// @par VFC_NO_COMPILER_CONFIG @anchor VFC_NO_COMPILER_CONFIG
/// When defined, no compiler configuration file is selected or included, 
/// define when the compiler is fully conformant with the standard, or where the 
/// user header (see @ref VFC_USER_CONFIG), has had any options necessary added to it.
/// 
/// @par VFC_NO_PLATFORM_CONFIG @anchor VFC_NO_PLATFORM_CONFIG
/// When defined, no platform configuration file is selected or included, 
/// define when the platform is fully conformant with the standard 
/// (and has no useful extra features), or where the user header (see @ref VFC_USER_CONFIG), 
/// has had any options necessary added to it.
///
/// @par VFC_STL_ALIAS
/// When defined, it should expand to the namespace of the used stl 
/// implementation (e.g. _STL for STLport).
/// Instead of accessing STL members explicitly with ::std, vfc uses a namespace 
/// alias (stlalias). This allows the user to use an optional 3rd party implementation 
/// parallel to an existing compiler vendor implementation..
/// When not defined, stlalias maps to ::std.
/// 
/// <hr>
///
/// @par VFC_NDEBUG @anchor VFC_NDEBUG
/// When defined, all asserts, pre- and post-condition checks are disabled.
/// See also:
/// -   #VFC_ASSERT
/// -   #VFC_REQUIRE
/// -   #VFC_ENSURE
///
/// @par VFC_NO_PRECHECKS @anchor VFC_NO_PRECHECKS
/// When defined, all vfc pre-condition checks are disabled.
/// See also: #VFC_REQUIRE
///
/// @par VFC_NO_POSTCHECKS @anchor VFC_NO_POSTCHECKS
/// When defined, all vfc pre-condition checks are disabled.
/// See also: #VFC_ENSURE
//=============================================================================


//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_core_config_platform
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_core_config_platform Platform Configuration
/// @ingroup vfc_group_core_config
/// @brief Platform information, options and defects.
///
/// @par VFC_PLATFORM_STRING
/// Defined as a string describing the name of the platform. 
/// Mainly for debugging the configuration.
///
/// @par VFC_PLATFORM_EPPC
/// Defined if the platform is an EPPC (aka AE-DA IVS).
///
/// @par VFC_PLATFORM_LINUX
/// Defined if the platform is Linux.
///
/// @par VFC_PLATFORM_WIN32
/// Defined if the platform is MS Windows.
///
/// @par VFC_WINDOWS
/// @b Deprecated! Defined if the Windows platform API is available.
///
/// @par VFC_HAS_STDINT_H
/// There are no 1998 C++ Standard headers @< stdint.h @> or @< cstdint @>, although 
/// the 1999 C Standard does include @< stdint.h @>. If @< stdint.h @> is present, 
/// <tt>"vfc/core/vfc_types.hpp"</tt> can make good use of it, so a flag is 
/// supplied (signalling presence; 
/// thus the default is not present, conforming to the current C++ standard).
///
/// @par VFC_NO_INT64
/// Defined if there are no 64-bit integral types: int64_t, uint64_t etc.
/// (Defined in <tt>"vfc/core/vfc_types.hpp"</tt> ). See also: VFC_NO_INT64
///
/// @par VFC_MT
/// Defined if multi-threading is enabled.
///
/// @par VFC_EPPC_DETECTED
/// For internal use only.
/// 
/// @par VFC_CLR_DETECTED
/// Defined if the platform is CLR (Common Language Runtime).<br>
/// For more information about CLR, C++/CLI and managed code, cf.
/// <a href="http://en.wikipedia.org/wiki/Common_Language_Runtime">Wikipedia on Common Language Runtime</a> and 
/// <a href="http://en.wikipedia.org/wiki/C%2B%2B/CLI">Wikipedia on C++/CLI</a>.<br>
/// CLR is enabled via a compiler switch, and as the compiler then generates intermediate
/// language bytecode (CIL), native code expressions (assembler via <tt>__asm</tt> keyword)
/// should not be used.
//=============================================================================


//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_core_config_compiler
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_core_config_compiler Compiler Configuration
/// @ingroup vfc_group_core_config
/// @brief Compiler information, options and defects.
///
/// @par VFC_COMPILER_STRING
/// Defined as a string describing the name and version number of the compiler 
/// in use. Mainly for debugging the configuration.
///
/// @par VFC_COMPILER_VISUALC
/// Defined if the compiler is a MS Visual C++ compiler.
///
/// @par VFC_COMPILER_GCC
/// Defined if the compiler is a GCC compiler.
///
/// @par VFC_COMPILER_MWERKS
/// Defined if the compiler is a Metrowerks CodeWarrior compiler.
///
/// @par VFC_COMPILER_DIAB
/// Defined if the compiler is a WindRiver C++ (Diab) compiler.
///
/// @par VFC_NO_RTTI
/// The compiler does not support run-time type information.
///
/// @par VFC_NO_EXCEPTIONS
/// The compiler does not support exception handling (this setting is typically 
/// required by many C++ compilers for embedded platforms). 
///
/// @par VFC_HAS_MS_INT64
/// The compiler supports the __int64 data type.
///
/// @par VFC_HAS_LONG_LONG
/// The compiler supports the long long data type.
///
/// @par VFC_DISABLE_WIN32
/// @b Deprecated! When defined, disables the use of Win32 specific API's, even 
/// when these are available. 
/// This option may be set automatically by the config system when it detects 
/// that the compiler is in "strict mode".
///
/// @par VFC_MSVC
/// @b Deprecated! Defined if the compiler is really Microsoft Visual C++, as opposed to one 
/// of the many other compilers that also define _MSC_VER.
//=============================================================================


//=============================================================================
//  DOXYGEN DEFGROUP vfc_group_core_config_processor
//-----------------------------------------------------------------------------
/// @defgroup vfc_group_core_config_processor Processor Configuration
/// @ingroup vfc_group_core_config
/// @brief Processor information, options and defects.
///
/// @par VFC_PROCESSOR_IX86
/// Defined if the processor architecture is Intel x86 32bit
///
/// @par VFC_PROCESSOR_IX86_64
/// Defined if the processor architecture is Intel x86 64bit
///
/// @par VFC_PROCESSOR_PPC
/// Defined if the processor architecture is PowerPC
///
/// @par VFC_BIG_ENDIAN
/// Defined, if the byte order is big-endian. 
/// Otherwise the byte order is little-endian (default).
//=============================================================================

////////////////////////////
// USER configuration
////////////////////////////

// if we don't have a user config, then use the default location:
#if !defined(VFC_USER_CONFIG) && !defined(VFC_NO_USER_CONFIG)
#  define VFC_USER_CONFIG "vfc/core/config/vfc_user_cfg.hpp"
#endif
// include it first:
#ifdef VFC_USER_CONFIG
#  include VFC_USER_CONFIG
#endif

////////////////////////////
// COMPILER configuration
////////////////////////////

// if we don't have a compiler config set, try and find one:
#if !defined(VFC_COMPILER_CONFIG) && !defined(VFC_NO_COMPILER_CONFIG) && !defined(VFC_NO_CONFIG)
#  include "vfc/core/config/vfc_select_compiler_config.hpp"
#endif
// if we have a compiler config, include it now:
#ifdef VFC_COMPILER_CONFIG
#  include VFC_COMPILER_CONFIG
#endif

////////////////////////////
// PLATFORM configuration
////////////////////////////

// if we don't have a platform config set, try and find one:
#if !defined(VFC_PLATFORM_CONFIG) && !defined(VFC_NO_PLATFORM_CONFIG) && !defined(VFC_NO_CONFIG)
#  include "vfc/core/config/vfc_select_platform_config.hpp"
#endif
// if we have a platform config, include it now:
#ifdef VFC_PLATFORM_CONFIG
#  include VFC_PLATFORM_CONFIG
#endif

////////////////////////////
// COMMON configuration
////////////////////////////

// include remaining configuration code
#include "vfc/core/config/vfc_config_suffix.hpp"

#endif //VFC_CONFIG_HPP_INCLUDED

//=============================================================================
//  R E V I S I O N   H I S T O R Y
//-----------------------------------------------------------------------------
//  $Log: vfc_config.hpp  $
//  Revision 1.15 2007/07/23 09:20:17MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - fixed references (mantis1744)
//  Revision 1.14 2007/07/18 16:30:10CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - doxygen documentation grouping
//  Revision 1.13 2007/06/26 14:28:15CEST Hissmann Michael (CR/AEM5) (ihm2si) 
//  - added CLR support (new macro VFC_CLR_DETECTED) (mantis 0001721)
//  - fixed warning warning C4793 in vfc_math.inl (mantis 0001721)
//  Revision 1.12 2007/04/16 09:05:44CEST Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
//  - fixed doxygen errors (mantis1576)
//  Revision 1.11 2007/04/02 15:18:10CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - fixed doku bug (mantis1484)
//  Revision 1.10 2007/03/29 16:17:03CEST Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added configuration docu (mantis1484)
//  Revision 1.9 2007/03/14 14:40:19CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - more docu (mantis1484)
//  Revision 1.8 2007/03/13 18:22:33CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - reorganized documentation (mantis1484)
//  Revision 1.7 2007/03/13 17:53:11CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added more docs for configuration submodule (mantis1484)
//  Revision 1.6 2007/03/07 17:18:23CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
//  - added doxygen group for configuration submodule (mantis1484)
//  Revision 1.4 2005/10/06 16:54:44CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
//  Initial revision
//  Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/vfc.pj
//  Revision 1.3 2005/04/01 15:54:43CEST zvh2hi 
//  added configuration code
//  Revision 1.2 2005/04/01 12:00:15CEST zvh2hi 
//  updated dummy_test and dependencies
//=============================================================================

