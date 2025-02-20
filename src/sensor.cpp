#include <algorithm>
#include <iostream>

#include "cynet/cynet_sensor.h"

void CySensor::attach_processor(CySensorMeasurementProcessor *p) {
  mutex.lock();
  p->s = this;
  proc.push_back(p);
  mutex.unlock();
}

void CySensor::detach_processor(CySensorMeasurementProcessor *p) {
  mutex.lock();
  auto it = std::remove(proc.begin(), proc.end(), p);
  if (it != proc.end()) {
    if (*it) delete *it;
    proc.erase(it, proc.end());
  }
  mutex.unlock();
}

void CySensor::clear_processors() {
  mutex.lock();
  while (proc.size()) {
    proc.erase(std::remove(proc.begin(), proc.end(), *proc.begin()),
               proc.end());
  }
  mutex.unlock();
}

void CySensor::add_measurement(uint16_t in) {
  measurement = in;
  mutex.lock();
  for (auto &i : proc) {
    i->add_measurement(in);
  }
  mutex.unlock();
}
