// network è una classe più ad alto livello rispetto a ecatHandler e gestisce le operazioni sull'intera rete di uC presenti.

#ifndef CYNET_NETWORK_H
#define CYNET_NETWORK_H

#include <vector>
#include <atomic>
#include "cynet_comm.h"
#include "cynet_ihb.h"
#include "cynet_comm_utils.h"

/* class CyNetwork */
class CyNetwork
{
	public:


		void shutdown();
        
		void switch_phase(CyCommPhase p);
		std::vector<CyIhb*> &get_network_structure(bool refresh=true, CyUSec timeout = (CY_1_SEC*3));
		std::vector<uint32_t> get_open_buses() { return ecat_comm->get_open_buses(); }

		void switch_skin_phase(CyCommPhase p);
		void start_skin_acquisition(uint16_t autosync = 50); // 50ms acquisition loop
		void stop_skin_acquisition();
		
	private:
		std::vector<CyIhb*> ihbs;

		void clear_network_structure(void);
		void discover_ihbs(std::vector<CyIhbInfo> *ret, CyUSec timeout = CY_100_MSEC);

		void setup_interrupt_handler();
		void discover_skin(std::vector<CyIhbInfo> *ret, CyUSec timeout = CY_100_MSEC);
		void decode_and_process_skin_data( CyIhbInfo ihb, uint8_t *data, uint32_t data_len);

};

#endif
