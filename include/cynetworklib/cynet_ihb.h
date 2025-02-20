#ifndef CYNET_IHB_H
#define CYNET_IHB_H

#include <stdint.h>
#include <atomic>
#include <thread>
#include "cyskin_driver/msg_struct.h"
#include "cynet_ihb_info.h"


/* class CyIhb */
class CyIhb
{
	public:
		CyIhb(CyIhbInfo i): info(i){};
		~CyIhb(){}

		uint32_t get_ihb_id() const { return info.ihb_id; }
		const CyIhbInfo& get_ihb_info() const { return info; }
		CyIhbDev& get_ihb_devs() { return *info.ihb_devs; }

	private:
		CyIhbInfo info;
};

#endif
