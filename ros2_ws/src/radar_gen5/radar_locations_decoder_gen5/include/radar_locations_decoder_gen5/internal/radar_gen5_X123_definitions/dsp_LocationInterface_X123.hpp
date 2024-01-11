#ifndef DSP_LOCATIONINTERFACE_INCLUDED
#define DSP_LOCATIONINTERFACE_INCLUDED

//#include <daddy_ifbase.hpp>
#include <vfc/core/vfc_types.hpp>
#include <vfc/container/vfc_fifo.hpp>

#include "dsp_location_list_X123.h"
#include "dsp_sensingstate_data_packet_X123.h"

#include <vfc/core/vfc_types.hpp>

namespace X123
{

namespace Dsp
{
   struct LocationInterface // : public daddy::CInterfaceBase
   {
      LocationInterface() // : daddy::CInterfaceBase()
      {
      }

      // from daddy::CInterfaceBase
      // (added here to avoid the daddy dependency)
      vfc::uint16_t m_referenceCounter;
      vfc::uint16_t m_sequenceNumber;

      struct DSP_LOCATION_LIST_ST m_LocationList;
      struct DSP_SENSINGSTATE_DATA_PACKET_ST m_SensState;
   };
}
}

#endif
