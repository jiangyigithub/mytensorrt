/********************************************************************
* C O P Y R I G H T
*--------------------------------------------------------------------
* Copyright (c) 2005 by Robert Bosch GmbH. All rights reserved.
*
* This file is property of Robert Bosch GmbH. Any unauthorised copy,
* use or distribution is an offensive act against international law
* and may me prosecuted under federal law. Its content is company
* confidential.
*--------------------------------------------------------------------
* D E S C R I P T I O N
*--------------------------------------------------------------------
*   Projectname: vfc
*      Synopsis: 
* Target system: 
*      Compiler: VS7.1
*--------------------------------------------------------------------
* N O T E S
*--------------------------------------------------------------------
* Notes: 
*--------------------------------------------------------------------
* I N I T I A L   A U T H O R   I D E N T I T Y
*--------------------------------------------------------------------
*       Name: 
* Department: 
*--------------------------------------------------------------------
* R E V I S I O N   I N F O R M A T I O N
*------------------------------------------------------------------*/
/*!   \file
*     \par Revision History
*     $Source: vfc_aligned_storage.hpp $
*     $Revision: 1.7 $
*     $Author: Jaeger Thomas (CC-DA/ENV1) (JAT2HI) $
*     $Date: 2014/01/28 11:30:46MEZ $
*     $Locker:  $
*     $Name:  $
*     $State: in_work $
*/
/*******************************************************************/

#ifndef VFC_ALIGNED_STORAGE_HPP_INCLUDED
#define VFC_ALIGNED_STORAGE_HPP_INCLUDED

#include "vfc/core/vfc_types.hpp"

namespace vfc
{    // namespace vfc opened

    //=========================================================================
    //  TAlignedStorage<>
    //-------------------------------------------------------------------------
    //! a POD type with size SizeValue and an alignment that is determind by
    //! max_aligned_t.
    //! TAlignedStorage is an essential component for custom made memory 
    //! allocators.
    //! @param SizeValue specifies the size of the instantiated type.
    //! @sa max_aligned_t
    //! @ingroup vfc_group_core_types
    //=========================================================================

    template <size_t SizeValue>
    class    TAlignedStorage
    {        
    public:

        TAlignedStorage() {}

        enum    { SIZE = SizeValue};

        //! returns the size of the aligned storage in bytes
        size_t          size    (void)    const     {    return SIZE;}

        //! returns a mutable pointer to the first byte of the aligned memory chunk
        void*           begin   (void)              {    return static_cast<void*>(m_data);}
        //! returns a const pointer to the first byte of the aligned  memory chunk
        const void*     begin   (void)    const     {    return static_cast<const void*>(m_data);}

        //! returns a mutable pointer that points just beyond the last byte of the memory chunk.
        void*           end     (void)              {    return static_cast<void*>(m_data+SIZE);}
        //! returns a const pointer that points just beyond the last byte of the memory chunk.
        const void*     end     (void)    const     {    return static_cast<const void*>(m_data+SIZE);}
    
    private:
        union
        {
            max_aligned_t   m_aligner;
            char_t          m_data[SizeValue];
        };
    };
}    // namespace vfc closed


#endif //VFC_ALIGNED_STORAGE_HPP_INCLUDED

/*******************************************************************
|-------------------------------------------------------------------|
| R E V I S I O N   H I S T O R Y
|-------------------------------------------------------------------|
$Log: vfc_aligned_storage.hpp  $
Revision 1.7 2014/01/28 11:30:46MEZ Jaeger Thomas (CC-DA/ENV1) (JAT2HI) 
- Performance issue with TAlignedStorage and zero initialisation (mantis0004315)
Revision 1.6 2007/07/23 09:28:00MESZ Zitzewitz Henning von (CR/AEM5) (zvh2hi) 
- doxygen grouping (mantis1744)
- added documentation
Revision 1.5 2006/11/16 14:41:16CET Zitzewitz Henning von (CR/AEM5) (ZVH2HI) 
- replaced tabs with 4 spaces (mantis1294)
Revision 1.3 2005/10/06 16:55:22CEST Zitzewitz Henning von (CR/AEM5) * (ZVH2HI) 
Initial revision
Member added to project d:/MKS_Data/Projects/platform-components/ivs/algo/pc_ivs_vfc/include/vfc/vfc.pj
Revision 1.2 2005/04/20 10:21:46CEST zvh2hi 
removed unnecessary include
Revision 1.1 2005/04/18 12:01:23CEST zvh2hi 
Initial revision
Member added to project /import/mks/data/projects/cv/vfc/include/vfc/vfc.pj
********************************************************************/
