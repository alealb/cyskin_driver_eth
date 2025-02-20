#include <algorithm>

#include "cynet/cynet_module.h"

CyModule::CyModule(const CyModule& m) {
  sensors = m.sensors;
  sui = m.sui;
  grouping = m.grouping;
  already_cut_sensors = m.already_cut_sensors;
}

CyModule& CyModule::operator=(const CyModule& m) {
  sensors = m.sensors;
  sui = m.sui;
  grouping = m.grouping;
  already_cut_sensors = m.already_cut_sensors;

  return *this;
}
