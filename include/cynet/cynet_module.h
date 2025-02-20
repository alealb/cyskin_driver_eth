#ifndef CYNET_MODULE_H
#define CYNET_MODULE_H


#include <vector>
#include <iostream>
#include "cynet_sensor.h"
#include <mutex>

class EcatHandler;

class CyModule{


    std::vector<CySensor> sensors;
    uint32_t sui;
    uint8_t grouping;
    uint16_t already_cut_sensors;
    //std::ofstream *out;	/* if logging this module, this stream would be open */

public:

    CyModule(): sensors(0),sui(0),grouping(0),already_cut_sensors(0){}
    CyModule(const CyModule &);

    CyModule& operator=(const CyModule& m);

    uint32_t get_sui() const { return sui; }
    uint8_t get_grouping() const { return grouping; }
    uint16_t get_cut_sensors() const { return already_cut_sensors; }
    std::vector<CySensor> & get_sensors() { return sensors; }

    friend class EcatHandler;
};

#endif

