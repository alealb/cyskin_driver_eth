#ifndef CYNET_IHB_DEV_H
#define CYNET_IHB_DEV_H

#include "cynet_node_info.h"
#include "cynet_module.h"

class EcatHandler;

class CyIhbDev{

    //std::vector<uint8_t> subnets; //one entry for each subnet. each element is the number of nodes of the subnet
	//std::vector<CyNodeInfo> *nodes; //array of vectors(one for each subnet) containing nodes information
    std::vector<CyModule> modules;

    public:

    CyIhbDev(){}
    CyIhbDev(const CyIhbDev &co){std::copy(co.modules.begin(), co.modules.end(),std::back_inserter(modules));}


    std::vector<CyModule>& get_modules(){return modules;};

    friend class EcatHandler;

};

#endif