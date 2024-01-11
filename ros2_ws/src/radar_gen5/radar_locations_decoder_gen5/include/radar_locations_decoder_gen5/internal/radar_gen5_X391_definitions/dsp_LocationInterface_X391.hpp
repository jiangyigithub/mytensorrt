#ifndef DSP_LOCATIONINTERFACE_INCLUDED
#define DSP_LOCATIONINTERFACE_INCLUDED

#include <vfc/core/vfc_types.hpp>
#include <vfc/container/vfc_fifo.hpp>

#include "daddy_ifbase_X391.hpp"
#include "dsp_location_list_X391.h"
#include "dsp_sensingstate_data_packet_X391.h"

namespace X391
{
namespace Dsp
{
   struct LocationInterface : public daddy::CInterfaceBase
   {
      LocationInterface() : daddy::CInterfaceBase()
      {
      }

      struct DSP_LOCATION_LIST_ST m_LocationList;
      struct DSP_SENSINGSTATE_DATA_PACKET_ST m_SensState;
   };
}
}
#endif
