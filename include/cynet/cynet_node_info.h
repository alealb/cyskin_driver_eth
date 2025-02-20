#ifndef CYNET_NODE_INFO_H
#define CYNET_NODE_INFO_H

#include <stddef.h>

struct CyNodeInfo
{
	uint32_t node_id;
	uint8_t  type;


	CyNodeInfo(): node_id(0),type(0) {}
	CyNodeInfo(uint32_t n, uint8_t t): node_id(n), type(t){}

};


#endif // CYNET_NODE_INFO_H
