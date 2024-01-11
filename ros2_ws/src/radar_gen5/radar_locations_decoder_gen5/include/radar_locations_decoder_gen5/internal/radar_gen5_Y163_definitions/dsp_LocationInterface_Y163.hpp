#ifndef DSP_LOCATIONINTERFACE_INCLUDED
#define DSP_LOCATIONINTERFACE_INCLUDED

#include <vfc/core/vfc_types.hpp>
#include <vfc/container/vfc_fifo.hpp>

#include "daddy_ifbase_Y163.hpp"
#include "dsp_location_list_Y163.h"
#include "dsp_sensingstate_data_packet_Y163.h"

namespace Y163
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
