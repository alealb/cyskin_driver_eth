#include "cyskin_driver/ecatHandler.hpp"

#include <new>

#include "ihb-pc-shared/ihbcmd.h"

void timespec_diff(struct timespec *start, struct timespec *stop,
                   struct timespec *result) {
  if ((stop->tv_nsec - start->tv_nsec) < 0) {
    result->tv_sec = stop->tv_sec - start->tv_sec - 1;
    result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
  } else {
    result->tv_sec = stop->tv_sec - start->tv_sec;
    result->tv_nsec = stop->tv_nsec - start->tv_nsec;
  }

  return;
}

EcatHandler::EcatHandler(char *ifname, int macroc_time,
                         int check_thread_sleep) {
  set_RT_thread_macrocycle(macroc_time);
  set_ecatcheck_sleep(check_thread_sleep);
  init_ecatthread();
  init_ecatcheck();
  ecat_init(ifname);
  remove_baseline = 0;
}

EcatHandler::~EcatHandler() {
  printf("Request safe operational state for all slaves\n");
  ec_slave[0].state = EC_STATE_SAFE_OP;
  /* request SAFE_OP state for all slaves */
  ec_writestate(0);

  printf("End test, close socket\n");
  /* stop SOEM, close socket */
  ec_close();

  delete[] tof_filtered_buffer->tof_uids;
  delete[] tof_filtered_buffer->tof_range_status;
  delete[] tof_filtered_buffer->tof_range_value;
  delete[] tof_filtered_buffer->tof_range_sigma;

  delete tof_filtered_buffer;
  delete cyskin_filtered_buffer;
  delete buffer;
}

void EcatHandler::makeThisThreadWait() {
  std::unique_lock<std::mutex> lk(mutex_data);
  cv_data.wait(lk, [&] { return updated; });
  updated = false;
  lk.unlock();
  cv_data.notify_one();
}

void EcatHandler::swapBuffer() {
  // bounded_buffer = (bounded_buffer+1)%2;
  fflush(stdout);
  {
    std::lock_guard<std::mutex> lk(mutex_data);
    updated = true;
  }
  cv_data.notify_one();
}

void EcatHandler::ecat_init(char *ifname) {
  printf("Starting SM test\n");
  /* initialise SOEM, bind socket to ifname */
  if (ec_init(ifname)) {
    printf("ec_init on %s succeeded.\n", ifname);
    /* find and auto-config slaves */
    if (ec_config_init(FALSE) > 0)  // INIT-> PRE-OP state
    {
      myfile.open("cycle_time_analysis_1_slave_minimum_cycle_time_NEW.csv",
                  ios::out | ios::app);
      N_SLAVES = ec_slavecount;
      uint32 slave_id[N_SLAVES];
      uint32 slave_sensor_type[N_SLAVES];

      printf("%d slaves found and configured.\n", ec_slavecount);
      ec_config_map(&IOmap);

      /*Read slave properties via SDO before OP state(e.g. the type of slave and
       * its payload )*/
      for (int i = 1; i <= ec_slavecount; i++) {
        printf("Slave %d (%s) State: %d\n", i, ec_slave[i].name,
               ec_slave[i].state);
        uint32 data;  // data will be stored here
        int rdl = sizeof(data);
        ec_SDOread(i, 0x6010, 0x01, FALSE, &rdl, &data, EC_TIMEOUTRXM);
        slave_id[i - 1] = data;
        ec_SDOread(i, 0x6010, 0x02, FALSE, &rdl, &data, EC_TIMEOUTRXM);
        slave_sensor_type[i - 1] = data;
        printf("SLAVE ID: %d, SLAVE SENSOR_TYPE: %d\n", slave_id[i - 1],
               slave_sensor_type[i - 1]);
        if (slave_sensor_type[i - 1] == TYPE_TOF_SENSOR)
          N_TOF_SLAVES += 1;
        else if (slave_sensor_type[i - 1] == TYPE_CYSKIN_SENSOR)
          N_CYSKIN_SLAVES += 1;
      }

      /* length of a segment i.e. the data that is contained in a single
       * etherCAT frame */
      seg_len = SEG_LEN;
      /* initalise with zeros the buffer that'll contain sensor data of each
       * slave  (i.e. 3 frames of seg_len size each for each slave) */
      // buffer= calloc(N_SLAVES* N_SEGMENTS*seg_len,sizeof(uint8));

      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

      /* read individual slave state and store in ec_slave[] */
      ec_readstate();

      /* number of input and output bytes for each slave (0 is for master) */
      oloop = (int *)malloc((N_SLAVES + 1) * sizeof(int));
      iloop = (int *)malloc((N_SLAVES + 1) * sizeof(int));
      // oloop= new int(N_SLAVES*sizeof(int));
      // iloop= new int(N_SLAVES*sizeof(int));

      printf("%d\n", *oloop);
      printf("%d\n", *iloop);

      for (int slave = 0; slave <= N_SLAVES; slave++) {
        oloop[slave] = ec_slave[slave].Obytes;
        if ((oloop[slave] == 0) && (ec_slave[slave].Obits > 0))
          oloop[slave] = 1;
        iloop[slave] = ec_slave[slave].Ibytes;
        if ((iloop[slave] == 0) && (ec_slave[slave].Ibits > 0))
          iloop[slave] = 1;
      }

      printf("M: output bytes: %d, input bytes: %d\n", oloop[0], iloop[0]);
      printf("S1: output bytes: %d, input bytes: %d\n", oloop[1],
             iloop[1]);  // print of first slave I/O bytes, as an example
      printf("S2: output bytes: %d, input bytes: %d\n", oloop[2], iloop[2]);
      // printf("S3: output bytes: %d, input bytes: %d\n",oloop[3],iloop[3]);
      // printf("S4: output bytes: %d, input bytes: %d\n",oloop[4],iloop[4]);
      // printf("S5: output bytes: %d, input bytes: %d\n",oloop[5],iloop[5]);
      // printf("S6: output bytes: %d, input bytes: %d\n",oloop[6],iloop[6]);
      printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments,
             ec_group[0].IOsegment[0], ec_group[0].IOsegment[1],
             ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

      expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
      printf("Calculated workcounter %d\n", expectedWKC);
      printf("Request operational state for all slaves\n");
      ec_slave[0].state = EC_STATE_OPERATIONAL;

      /*Wait for a few seconds*/
      osal_usleep(300);

      /* Example of how to read EtherCAT registers of a certain slave.*/
      for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
        int ret = 0, l;
        uint16_t sync_mode;
        uint32_t cycle_time;
        uint32_t minimum_cycle_time;
        uint32_t sync0_cycle_time;
        uint32_t get_cycle_time = 0x01;
        l = sizeof(sync_mode);
        ret += ec_SDOwrite(cnt, 0x1c32, 0x08, FALSE, l, &get_cycle_time,
                           EC_TIMEOUTRXM);

        ret +=
            ec_SDOread(cnt, 0x1c32, 0x01, FALSE, &l, &sync_mode, EC_TIMEOUTRXM);
        l = sizeof(cycle_time);
        ret += ec_SDOread(cnt, 0x1c32, 0x02, FALSE, &l, &cycle_time,
                          EC_TIMEOUTRXM);
        l = sizeof(minimum_cycle_time);
        ret += ec_SDOread(cnt, 0x1c32, 0x05, FALSE, &l, &minimum_cycle_time,
                          EC_TIMEOUTRXM);
        l = sizeof(sync0_cycle_time);
        ret += ec_SDOread(cnt, 0x1c32, 0x0a, FALSE, &l, &sync0_cycle_time,
                          EC_TIMEOUTRXM);
        printf(
            "PDO syncmode %02x, cycle time %d ns (min %d), sync0 cycle time %d "
            "ns, ret = %d\n",
            sync_mode, cycle_time, minimum_cycle_time, sync0_cycle_time, ret);
        printf(
            "Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d "
            "delay:%d, hasDC: %d, isDCactive: %d\n",
            cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
            ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc,
            ec_slave[cnt].DCactive);
        printf("         Out:%p,%4d In:%p,%4d\n", ec_slave[cnt].outputs,
               ec_slave[cnt].Obytes, ec_slave[cnt].inputs,
               ec_slave[cnt].Ibytes);
      }

      buffer = new uint8_t[N_SLAVES * N_SEGMENTS * seg_len];

      /* request OP state for all slaves */
      ec_writestate(0);
      /* activate cyclic process data */
      set_is_ecatthread_running(1);

      /* wait for all slaves to reach OP state */
      ec_statecheck(0, EC_STATE_OPERATIONAL, 5 * EC_TIMEOUTSTATE);

      if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        printf("Operational state reached for all slaves.\n");
        inOP = TRUE;
      } else {
        printf("Not all slaves reached operational state.\n");
        ec_readstate();
        for (int i = 1; i <= ec_slavecount; i++) {
          if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
            printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n", i,
                   ec_slave[i].state, ec_slave[i].ALstatuscode,
                   ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
          }
        }
      }

    } else {
      printf("No slaves found!\n");
    }

  } else {
    printf("No socket connection on %s\nExcecute as root\n", ifname);
  }

  // free soem areas
  // free(oloop);
  // free(iloop);
}

void EcatHandler::add_timespec(struct timespec *ts, int64 addtime) {
  int64 sec, nsec;

  nsec = addtime % NSEC_PER_SEC;
  sec = (addtime - nsec) / NSEC_PER_SEC;
  ts->tv_sec += sec;
  ts->tv_nsec += nsec;
  if (ts->tv_nsec >= NSEC_PER_SEC) {
    nsec = ts->tv_nsec % NSEC_PER_SEC;
    ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
    ts->tv_nsec = nsec;
  }
}

// void EcatHandler::print_data_vector(){

//       for (int slave = 0; slave < N_SLAVES; slave++) {
//          printf("Vector %d:\n", slave + 1);
//          for (const auto* ptr : data_vector[slave]) {
//             printf("id_tof: %d\n", ptr->id_tof);
//             printf("status_get_measurement: %d\n",
//             ptr->status_get_measurement);

//             printf("range_status: ");
//             for (int i = 0; i < VL53L5CX_RESOLUTION_64; ++i) {
//                   printf("%d ", ptr->range_status[i]);
//             }
//             printf("\n");

//             printf("range_val: ");
//             for (int i = 0; i < VL53L5CX_RESOLUTION_64; ++i) {
//                   printf("%d ", ptr->range_val[i]);
//             }
//             printf("\n");

//             printf("sigma: ");
//             for (int i = 0; i < VL53L5CX_RESOLUTION_64; ++i) {
//                   printf("%d ", ptr->sigma[i]);
//             }
//             printf("\n");
//          }
//       }

// }

void EcatHandler::print_TOF_mask() {
  printf("---- TOF mask ----\n");
  for (uint8_t slave = 0; slave < N_SLAVES; slave++) {
    if (sensor_types[slave] == TYPE_TOF_SENSOR) {
      printf("SLAVE %d \n", slave + 1);
      for (uint8_t tof = 0; tof < TOF_TOTAL_NUMBER; tof++) {
        printf(" %d ", TOF_mask[slave][tof]);
      }
      printf("\n");
    }
  }
}

void EcatHandler::print_slave_buffer(uint8_t slave, uint8_t printOutput,
                                     uint8_t printInput) {
  if (slave <= N_SLAVES) {
    printf("\n --------- IOmap BUFFER of slave %d----------\n", slave);
    if (printOutput) {
      printf("Outputs:\n");
      for (uint32_t j = 0; j < ec_slave[slave].Obytes; j++) {
        printf(" %2.2x", *(ec_slave[slave].outputs + j));
      }
      printf("\n");
    }
    if (printInput) {
      printf("Inputs:\n");
      for (uint32_t j = 0; j < ec_slave[slave].Ibytes; j++) {
        printf(" %2.2x", *(ec_slave[slave].inputs + j));
      }
    }

    printf("\n ------------------- \n");
  } else {
    printf("Slave %d does not exist!\n", slave);
  }
  fflush(stdout);
}

void EcatHandler::print_raw_buffer() {
  printf("\n ------------ Local BUFFER -----------\n");
  for (uint8 slave = 1; slave <= N_SLAVES; slave++) {
    printf("---> SLAVE %d \n", slave);

    for (int i = 0; i < N_SEGMENTS * seg_len;
         i++)  // each Slave is reserved a portion of the raw buffer of length
               // N_SEGMENTS*seg_len
    {
      printf(" %2.2x", *(buffer + i + (slave - 1) * N_SEGMENTS * seg_len));
    }
    printf("\n ------------------- \n");
    fflush(stdout);
  }
}

void EcatHandler::print_raw_buffer_single_slave(uint8_t slave) {
  printf("\n ------------ Local BUFFER -----------\n");

  if (slave < N_SLAVES) {
    printf("---> SLAVE %d \n", slave);

    for (int i = 0; i < N_SEGMENTS * seg_len; i++) {
      printf(" %2.2x", *(buffer + i + (slave - 1) * N_SEGMENTS * seg_len));
    }
  } else {
    printf("Slave %d does not exist!\n", slave);
  }
  printf("\n ------------------- \n");
}

void EcatHandler::print_generic_tof_buffer(generic_buffer_tof *b) {
  printf("-----GENERIC BUFFER : tof uids  ------\n");
  for (int i = 0; i < b->size; i++) {
    printf(" %d, ", *(b->tof_uids + i));
  }
  printf("\n ----------------\n");
  fflush(stdout);

  printf("-----GENERIC BUFFER : range status  ------\n");
  for (int i = 0; i < b->size; i++) {
    printf(" %d, ", *(b->tof_range_status + i));
  }
  printf("\n ----------------\n");
  fflush(stdout);

  printf("-----GENERIC BUFFER : range values  ------\n");
  for (int i = 0; i < b->size; i++) {
    printf(" %d, ", *(b->tof_range_value + i));
  }
  printf("\n ----------------\n");
  fflush(stdout);

  printf("-----GENERIC BUFFER : range sigma  ------\n");
  for (int i = 0; i < b->size; i++) {
    printf(" %d, ", *(b->tof_range_sigma + i));
  }
  printf("\n ----------------\n");
  fflush(stdout);
}

void EcatHandler::print_generic_cyskin_buffer(generic_buffer_cyskin *b) {
  printf("-----GENERIC BUFFER : cyskin uids  ------\n");
  for (int i = 0; i < b->size; i++) {
    printf(" %d, ", *(b->cyskin_uids + i));
  }
  printf("\n ----------------\n");
  fflush(stdout);

  printf("-----GENERIC BUFFER : cyskin responces ------\n");
  for (int i = 0; i < b->size; i++) {
    printf(" %d, ", *(b->cyskin_responces + i));
  }
  printf("\n ----------------\n");
  fflush(stdout);
}

void EcatHandler::xmc43_output_cast() {
  out_xmc43 = (out_xmc43_t **)malloc(N_SLAVES * sizeof(*out_xmc43));
  /*cast of the output process data for each slave (to send a command to a
   * specific slave) */
  for (int slave = 0; slave < N_SLAVES; slave++) {
    out_xmc43[slave] = (out_xmc43_t *)ec_slave[slave + 1].outputs;
  }
}
void EcatHandler::xmc43_input_cast() {
  in_xmc43 = (in_xmc43_t **)malloc(N_SLAVES * sizeof(*in_xmc43));
  /*cast of the input process data for each slave (INIT state) */
  for (int slave = 0; slave < N_SLAVES; slave++) {
    in_xmc43[slave] = (in_xmc43_t *)(ec_slave[slave + 1].inputs);
  }
}

void EcatHandler::xmc43_set_MSG_ID(uint8_t val) {
  /* Message we want to receive, val=1 is for sensor reading*/
  for (int slave = 0; slave < N_SLAVES; slave++) {
    out_xmc43[slave]->MSG_ID = val;
  }
}
void EcatHandler::xmc43_set_SEGMENT(uint8_t val) {
  for (int slave = 0; slave < N_SLAVES; slave++) {
    out_xmc43[slave]->SEGMENT = val;
  }
}

void EcatHandler::xmc43_set_PARAMETERS(uint8_t val1, uint8_t val2,
                                       uint8_t val3) {
  for (int slave = 0; slave < N_SLAVES; slave++) {
    out_xmc43[slave]->RES_TOF = val1;
    out_xmc43[slave]->FREQ_TOF = val2;
    out_xmc43[slave]->SHARP_TOF = val3;
    out_xmc43[slave]->CDC_CYSKIN = 0;
    out_xmc43[slave]->PERIOD_CYSKIN = 0;
    out_xmc43[slave]->SPI_CYSKIN = 0;
  }
}

void EcatHandler::copy_segments_to_raw_buffer(uint16_t offset) {
  for (uint8_t slave = 0; slave < N_SLAVES; slave++) {
    uint16 idx = offset + slave * N_SEGMENTS * seg_len;
    memcpy(&buffer[idx], ec_slave[slave + 1].inputs,
           seg_len);  // WARNING: "+1" added
  }
  // std::cout << "raw buffer values: \n\n";
  // for(int i = 0; i < N_SEGMENTS*seg_len; i++)
  // { std::cout << (int)buffer[i] << "  ";}
}

void EcatHandler::copy_raw_buffer_to_tof_filtered_buffer() {
  // int cnt=sizeof(VL53LX_DATA_64_MSG);
  int status_index = 0;
  int range_index = 0;
  int sigma_index = 0;
  int tof_counter = 0;
  int endpoint_cnt = 0;
  int buffer_index = 0;
  uint8 slave_tof_cnt = 0;

  for (uint8_t slave = 0; slave < N_SLAVES; slave++) {
    if (sensor_types[slave] == TYPE_TOF_SENSOR)  // TOF
    {
      // std::cout << "filtered buffer size: \n\n";
      // std::cout << (int) tof_filtered_buffer->size << "  ";
      // for (uint8_t tof=0;tof<TOF_TOTAL_NUMBER;tof++)
      for (uint8_t tof = 0; tof < TOFs_in_slave[slave_tof_cnt]; tof++) {
        if (TOF_mask[slave_tof_cnt][tof] ==
            1)  // check whether or not data from a certain ToF sensor is valid
        {
          status_index = slave * N_SEGMENTS * seg_len + 24 +
                         tof * NUMBER_TOF_ENDPOINTS;  // da parametrizzare
          range_index = slave * N_SEGMENTS * seg_len + 24 +
                        TOF_TOTAL_NUMBER * NUMBER_TOF_ENDPOINTS +
                        tof * 2 * NUMBER_TOF_ENDPOINTS;
          sigma_index = slave * N_SEGMENTS * seg_len + 24 +
                        TOF_TOTAL_NUMBER * NUMBER_TOF_ENDPOINTS * 3 +
                        tof * 2 * NUMBER_TOF_ENDPOINTS;

          //  std::cout << "ToFs in slave: " << (int)
          //  TOFs_in_slave[slave_tof_cnt] << "\n\n"; std::cout << "status
          //  index: " << (int) status_index << "\n\n"; std::cout << "range
          //  index: " << (int) range_index << "\n\n"; std::cout << "sigma
          //  index: " << (int) sigma_index << "\n\n";

          for (endpoint_cnt = 0; endpoint_cnt < NUMBER_TOF_ENDPOINTS;
               endpoint_cnt++) {
            // buffer_index = slave * NUMBER_TOF_ENDPOINTS * tof_counter +
            // NUMBER_TOF_ENDPOINTS * tof + endpoint_cnt;

            tof_filtered_buffer->tof_uids[buffer_index] =
                COMPUTE_UID_TOF(slave_ids[slave], tof, endpoint_cnt);
            // tof_filtered_buffer->tof_uids[buffer_index] = endpoint_cnt;
            tof_filtered_buffer->tof_range_status[buffer_index] =
                buffer[status_index + endpoint_cnt];
            tof_filtered_buffer->tof_range_value[buffer_index] =
                (buffer[range_index + 2 * endpoint_cnt] |
                 buffer[range_index + 2 * endpoint_cnt + 1] << 8);
            tof_filtered_buffer->tof_range_sigma[buffer_index] =
                (buffer[sigma_index + 2 * endpoint_cnt] |
                 buffer[sigma_index + 2 * endpoint_cnt + 1] << 8);
            buffer_index++;

            // std::cout << (int) tof_filtered_buffer->tof_uids[buffer_index] <<
            // "  "; std::cout << (int)
            // tof_filtered_buffer->tof_range_status[buffer_index] << "  ";
            // std::cout << (int)
            // tof_filtered_buffer->tof_range_value[buffer_index] << "  ";
            //  std::cout << (int)
            //  tof_filtered_buffer->tof_range_sigma[buffer_index] << "  ";
          }
          // std::cout << "\n\n";
          // tof_counter++;
        }
      }
      slave_tof_cnt++;
    }
    // std::cout << "\n\n";
  }
}

void EcatHandler::copy_raw_buffer_to_cydata() {
  uint8_t pos = 0;
  for (uint8 slave = 0; slave < N_SLAVES; slave++) {
    if (sensor_types[slave] == TYPE_CYSKIN_SENSOR)  // CYSKIN
    {
      uint16_t msg_size = buffer[slave * SEG_LEN * N_SEGMENTS + 2] << 8 |
                          buffer[slave * SEG_LEN * N_SEGMENTS + 3];
      uint8_t module = buffer[slave * SEG_LEN * N_SEGMENTS + 4];
      uint8_t sensor = buffer[slave * SEG_LEN * N_SEGMENTS + 5];
      // CyIhbDev ihb_dev = ihbs.at(slave)->get_ihb_devs();
      for (unsigned int i = 6; i < msg_size; i += 2) {
        uint16_t response = buffer[slave * SEG_LEN * N_SEGMENTS + i] << 8 |
                            buffer[slave * SEG_LEN * N_SEGMENTS + i + 1];

        if (module >= ihbs.at(pos)->get_ihb_devs().modules.size() ||
            sensor >=
                ihbs.at(pos)->get_ihb_devs().modules[module].sensors.size()) {
          // std::cerr << "Too much sensor data from ihb " <<
          // ihbs.at(pos)->get_ihb_id() << std::endl;
          break;
        }

        ihbs.at(pos)
            ->get_ihb_devs()
            .modules[module]
            .sensors[sensor]
            .add_measurement(response);
        // printf("\nsensor val:   %d\n",
        // ihbs.at(slave)->get_ihb_devs().modules[module].sensors[sensor].get_measurement());

        ++sensor;

        /* if module data is complete and the module is to be logged, do it
         * right now */
        // Skin data is sent out sequentially. If the number of received sensor
        // measurements is equal to the sensor size, all the sensors have been
        // received for this cycle.

        // As in the previous comment, if the last module has been received i
        // can publish the whole ihb
        while (
            module < ihbs.at(pos)->get_ihb_devs().modules.size() &&
            sensor >=
                ihbs.at(pos)->get_ihb_devs().modules[module].sensors.size()) {
          ++module;
          sensor = 0;
        }
      }
      pos++;
    }
    // std::cout << "Qui ci sono 1!\n";
  }
}

void EcatHandler::copy_cydata_to_filtered_buffer() {
  uint16_t sensor_id;
  uint16_t module_uid;
  uint32_t response;
  uint32_t sensor_uid;
  int k = 0;

  for (auto ihb : ihbs) {
    for (auto &module : ihb->get_ihb_devs().get_modules()) {
      for (uint32_t s = 0; s < module.get_sensors().size(); s++) {
        module_uid = module.get_sui();
        sensor_id = module.sensors[s].get_name();
        sensor_uid = COMPUTE_UID_CYSKIN(module_uid, sensor_id);
        response = module.sensors[s].get_measurement();
        // cout << "response :     " << response << endl;

        if (steps_baseline > 0) {
          baseline[k] = response;
        } else {
          cyskin_filtered_buffer->cyskin_uids[k] = sensor_uid;
          // cyskin_filtered_buffer->cyskin_responces[k] = (int)(response - baseline[k]) > 0 ? (response - baseline[k]) : 0;
          cyskin_filtered_buffer->cyskin_responces[k] = response;
          // cout << "filtered buffer id:    " <<
          // cyskin_filtered_buffer->buffer[bounded_buffer][k] << endl; cout <<
          // "filtered buffer val:    "
          // <<cyskin_filtered_buffer->buffer[bounded_buffer][k+n_sensors]  <<
          // endl;
        }
        k++;
      }
    }
  }
  if (steps_baseline > 0) steps_baseline--;
  // std::cout << "Qui ci sono 2!\n";
}

void EcatHandler::set_is_ecatthread_running(int count) { dorun = count; }
int EcatHandler::get_is_ecatthread_running() { return dorun; }

OSAL_THREAD_FUNC_RT *EcatHandler::ecatthread(void *ptr) {
  set_is_ecatthread_running(0);

  // Wait until the ecatthread is allowed to run
  while (!get_is_ecatthread_running());

  xmc43_output_cast();
  // print_slave_buffer(0, 1, 1); //just as an example
  // print_slave_buffer(1, 1, 1);
  // print_slave_buffer(2, 1, 1);

  /********* CONFIG STATE *********/
  xmc43_set_PARAMETERS(TOF_HIGH_RESOLUTION, TOF_MAX_FREQUENCY_H_RES,
                       TOF_SHARPENER_DEFAULT);

  /********** INIT STATE **********/
  uint8_t n_ready_slaves = 0;

  slave_ids = (uint16 *)malloc(N_SLAVES * sizeof(uint16_t));
  sensor_types = (uint16 *)malloc(N_SLAVES * sizeof(uint16_t));

  set_ECAT_STATE(INIT_STATE);

  xmc43_set_MSG_ID(INIT_STATE);
  xmc43_input_cast();
  ec_send_processdata();

  /* Read input PDO until all slaves have sent the correct message (i.e., the
   * header must contain the appropriate value) */
  while (n_ready_slaves != N_SLAVES) {
    n_ready_slaves = 0;
    ec_receive_processdata(EC_TIMEOUTRET);

    for (uint8 slave = 0; slave < N_SLAVES; slave++) {
      if (in_xmc43[slave]->arr[0] == ID_VL53LX_INIT_MSG &&
          in_xmc43[slave]->arr[1] == 0xAA) {
        n_ready_slaves++;
        if (in_xmc43[slave]->sensor_type == TYPE_TOF_SENSOR) {
          std::cout << "SLAVE ID: " << (int)in_xmc43[slave]->slave_id
                    << ",  SENSOR TYPE: TOF\n";
          slave_ids[slave] = in_xmc43[slave]->slave_id;
          sensor_types[slave] = in_xmc43[slave]->sensor_type;
        } else if (in_xmc43[slave]->sensor_type == TYPE_CYSKIN_SENSOR) {
          std::cout << "SLAVE ID: " << (int)in_xmc43[slave]->slave_id
                    << ",  SENSOR TYPE: CYSKIN\n";
          slave_ids[slave] = in_xmc43[slave]->slave_id;
          sensor_types[slave] = in_xmc43[slave]->sensor_type;
        }
      }
    }

    printf("READY SLAVES: %d\n", n_ready_slaves);
    // print_slave_buffer(1, 1, 1);
    // print_slave_buffer(2, 1, 1);

    ec_send_processdata();

    osal_usleep(20000);
  }

  TOF_mask = (uint8_t **)malloc(N_TOF_SLAVES * sizeof(uint8_t *));

  if (TOF_mask == NULL) {
    printf("Error: Memory allocation failed\n");
    while (1);
  }
  /* Statically allocated array of structs containing initialization message of
   * each slave */
  VL53LX_FULL_INIT_MSG init_tof_data[N_TOF_SLAVES];
  CYSKIN_INIT_MESSAGE init_cyskin_data[N_CYSKIN_SLAVES];
  uint8 slave_tof_cnt = 0;
  uint8 slave_cyskin_cnt = 0;

  for (uint8 slave = 0; slave < N_SLAVES; slave++) {
    /********************* TOF SENSORS SLAVES ***********************/
    if (in_xmc43[slave]->sensor_type == TYPE_TOF_SENSOR) {
      std::cout << "\nqui ci sono 2\n";
      memcpy(&init_tof_data[slave_tof_cnt], in_xmc43[slave],
             sizeof(VL53LX_FULL_INIT_MSG));

      TOF_mask[slave_tof_cnt] =
          (uint8_t *)malloc(TOF_TOTAL_NUMBER * sizeof(uint8_t));
      if (TOF_mask[slave_tof_cnt] == NULL) {
        printf("Error: Memory allocation failed\n");
        while (1);
      }

      for (uint8_t tof = 0; tof < TOF_TOTAL_NUMBER; tof++) {
        if (init_tof_data[slave_tof_cnt].init_tof[tof].tof_status ==
            TOF_IS_ACTIVE) {
          TOF_mask[slave_tof_cnt][tof] = 1;
          actual_TOF_number++;
          std::cout << "TOF cnt" << actual_TOF_number << std::endl;
        } else {
          TOF_mask[slave_tof_cnt][tof] = 0;
        }
      }
      slave_tof_cnt++;
    }

    /********************* CYSKIN SLAVES ***********************/
    if (in_xmc43[slave]->sensor_type == TYPE_CYSKIN_SENSOR) {
      CyIhbInfo rihbs;
      memcpy(&init_cyskin_data[slave_cyskin_cnt], in_xmc43[slave],
             (sizeof(MSG_HEADER) + sizeof(IhbInfo)));
      uint8_t total_modules_count =
          init_cyskin_data[slave_cyskin_cnt].ihb_info.tot_modules_cnt;

      init_cyskin_data[slave_cyskin_cnt].mod_info =
          (ModuleInfo *)malloc(sizeof(ModuleInfo) * total_modules_count);
      memcpy(init_cyskin_data[slave_cyskin_cnt].mod_info,
             &in_xmc43[slave]->arr[9],
             (sizeof(ModuleInfo) * total_modules_count));
      printf("\n----- INIT DATA SLAVE %d -----\n", slave + 1);
      printf("message header: %d\n",
             (uint16_t)init_cyskin_data[slave_cyskin_cnt].header.msg_id);
      printf("ihb_id: %d\n",
             (uint32_t)init_cyskin_data[slave_cyskin_cnt].ihb_info.ihb_id);
      printf(
          "number of modules: %d\n",
          (uint8_t)init_cyskin_data[slave_cyskin_cnt].ihb_info.tot_modules_cnt);

      rihbs = CyIhbInfo((init_cyskin_data[slave_cyskin_cnt].ihb_info.ihb_id));

      for (uint8_t i = 0; i < total_modules_count; i++) {
        CyModule m;
        m.sensors.resize(
            init_cyskin_data[slave_cyskin_cnt].mod_info[i].module_sensor_num);
        m.sui = init_cyskin_data[slave_cyskin_cnt].mod_info[i].module_sui_id;
        m.grouping =
            init_cyskin_data[slave_cyskin_cnt].mod_info[i].module_sensor_group;
        m.already_cut_sensors = ~(init_cyskin_data[slave_cyskin_cnt]
                                      .mod_info[i]
                                      .module_sensor_uncut) &
                                0xFFF;

        int sensor_name = 0;

        for (size_t j = 0; j < m.sensors.size(); ++j) {
          /* <= is intentional, to catch the error case if data are inconsistent
           */
          while (sensor_name <= init_cyskin_data[slave_cyskin_cnt]
                                    .mod_info[i]
                                    .module_sensor_num &&
                 (init_cyskin_data[slave_cyskin_cnt]
                      .mod_info[i]
                      .module_sensor_uncut &
                  1 << sensor_name) == 0)
            ++sensor_name;
          m.sensors[j].sensor_name = sensor_name++;
          // printf("\nsensor name (SUID):   %d\n", sensor_name);
        }

        rihbs.ihb_devs->modules.push_back(m);
      }
      ihbs.push_back(new CyIhb(rihbs));
      slave_cyskin_cnt++;
      std::cout << "\nqui ci sono 1\n";
    }
  }

  TOFs_in_slave = (uint8 *)malloc(slave_tof_cnt * sizeof(uint8_t));
  slave_tof_cnt = 0;

  for (uint8 slave = 0; slave < N_SLAVES; slave++) {
    /********************* TOF SENSORS SLAVES ***********************/
    if (in_xmc43[slave]->sensor_type == TYPE_TOF_SENSOR) {
      uint8 tof_cnt_in_slave = 0;
      for (uint8_t tof = 0; tof < TOF_TOTAL_NUMBER; tof++) {
        if (TOF_mask[slave_tof_cnt][tof] == 1)
          TOFs_in_slave[slave_tof_cnt] += 1;
      }
      slave_tof_cnt++;
    }
  }

  // print_TOF_mask();
  printf("Actual TOF number: %d\n", actual_TOF_number);

  for (auto ihb : ihbs) {
    for (auto &module : ihb->get_ihb_devs().get_modules()) {
      std::cout << "module id:  " << (int)module.get_sui() << "\n\n";
      uint16_t cnt = 0;
      for (uint32_t s = 0; s < module.get_sensors().size(); s++) {
        taxel_number++;
        cnt++;
        // std::cout << "taxel number:  " << taxel_number << "\n\n";
      }
      std::cout << "taxel number:  " << (int)cnt << "\n\n";
    }
  }

  std::cout << "taxel number:  " << taxel_number << "\n\n";
  fflush(stdout);

  /* Allocate filtered_buffer with a VL53LX_DATA_64_MSG struct for each active
   * sensor + a slave ID to add to each TOF struct */
  int size_of_tof_filtered_buffer = actual_TOF_number * NUMBER_TOF_ENDPOINTS;
  tof_filtered_buffer = new generic_buffer_tof{
      /* tof_uids*/ new uint32_t[size_of_tof_filtered_buffer],
      /* tof_range_status */ new uint32_t[size_of_tof_filtered_buffer],
      /* tof_range_value */ new uint32_t[size_of_tof_filtered_buffer],
      /* tof_range_sigma */ new uint32_t[size_of_tof_filtered_buffer],
      size_of_tof_filtered_buffer,
      actual_TOF_number};

  int buffer_index = 0;
  int endpoint_cnt = 0;
  slave_tof_cnt = 0;

  for (uint8_t slave = 0; slave < N_SLAVES; slave++) {
    if (sensor_types[slave] == TYPE_TOF_SENSOR)  // TOF
    {
      // std::cout << (int) tof_filtered_buffer->size << "  ";
      for (uint8_t tof = 0; tof < TOF_TOTAL_NUMBER; tof++) {
        if (TOF_mask[slave_tof_cnt][tof] ==
            1)  // check whether or not data from a certain ToF sensor is valid
        {
          for (endpoint_cnt = 0; endpoint_cnt < NUMBER_TOF_ENDPOINTS;
               endpoint_cnt++) {
            tof_filtered_buffer->tof_uids[buffer_index] =
                COMPUTE_UID_TOF(slave_ids[slave], tof, endpoint_cnt);
            // tof_filtered_buffer->tof_uids[buffer_index] = endpoint_cnt;
            buffer_index++;
          }
        }
      }
      slave_tof_cnt++;
    }
  }

  int size_of_cyskin_filtered_buffer = taxel_number * NUMBER_CYSKIN_ENDPOINTS;
  cyskin_filtered_buffer = new generic_buffer_cyskin{
      /*cyskin_uids*/ new uint32_t[size_of_cyskin_filtered_buffer],
      /*cyskin_responces*/ new uint32_t[size_of_cyskin_filtered_buffer],
      size_of_cyskin_filtered_buffer, taxel_number};

  uint16_t sensor_id;
  uint16_t module_uid;
  uint32_t sensor_uid;
  int k = 0;

  for (auto ihb : ihbs) {
    for (auto &module : ihb->get_ihb_devs().get_modules()) {
      for (uint32_t s = 0; s < module.get_sensors().size(); s++) {
        module_uid = module.get_sui();
        sensor_id = module.sensors[s].get_name();
        sensor_uid = COMPUTE_UID_CYSKIN(module_uid, sensor_id);

        cyskin_filtered_buffer->cyskin_uids[k] = sensor_uid;
        k++;
      }
    }
  }

  baseline = std::vector<uint32_t>(taxel_number, 0);
  if (remove_baseline) steps_baseline = 50;

  printf("FSM: from %d to %d", INIT_STATE, DATA_ACQUISITION_STATE);

  /********** DATA ACQUISITION STATE **********/
  set_ECAT_STATE(DATA_ACQUISITION_STATE);
  xmc43_set_MSG_ID(DATA_ACQUISITION_STATE);

  curSegment = 0;
  xmc43_set_SEGMENT(curSegment);
  // print_slave_buffer(1, TRUE, TRUE);
  // print_slave_buffer(2, TRUE, TRUE);

  uint32_t cycle_time_SM2;
  uint32_t minimum_cycle_time_SM2_1;
  // uint32_t minimum_cycle_time_SM2_2;
  // uint32_t minimum_cycle_time_SM2_3;
  // uint32_t minimum_cycle_time_SM2_4;
  // uint32_t minimum_cycle_time_SM2_5;
  // uint32_t minimum_cycle_time_SM2_6;

  uint32_t cycle_time_SM3;
  uint32_t minimum_cycle_time_SM3_1;
  // uint32_t minimum_cycle_time_SM3_2;
  // uint32_t minimum_cycle_time_SM3_3;
  // uint32_t minimum_cycle_time_SM3_4;
  // uint32_t minimum_cycle_time_SM3_5;
  // uint32_t minimum_cycle_time_SM3_6;

  int ret = 0, l;
  l = sizeof(cycle_time_SM2);
  uint16 cnt = 3;

  // ret += ec_SDOread(1, 0x1c32, 0x05, FALSE, &l, &minimum_cycle_time_SM2_1,
  // EC_TIMEOUTRXM); ret += ec_SDOread(1, 0x1c33, 0x05, FALSE, &l,
  // &minimum_cycle_time_SM3_1, EC_TIMEOUTRXM); ret += ec_SDOread(2, 0x1c32,
  // 0x05, FALSE, &l, &minimum_cycle_time_SM2_2, EC_TIMEOUTRXM); ret +=
  // ec_SDOread(2, 0x1c33, 0x05, FALSE, &l, &minimum_cycle_time_SM3_2,
  // EC_TIMEOUTRXM); ret += ec_SDOread(3, 0x1c32, 0x05, FALSE, &l,
  // &minimum_cycle_time_SM2_3, EC_TIMEOUTRXM); ret += ec_SDOread(3, 0x1c33,
  // 0x05, FALSE, &l, &minimum_cycle_time_SM3_3, EC_TIMEOUTRXM); ret +=
  // ec_SDOread(4, 0x1c32, 0x05, FALSE, &l, &minimum_cycle_time_SM2_4,
  // EC_TIMEOUTRXM); ret += ec_SDOread(4, 0x1c33, 0x05, FALSE, &l,
  // &minimum_cycle_time_SM3_4, EC_TIMEOUTRXM); ret += ec_SDOread(5, 0x1c32,
  // 0x05, FALSE, &l, &minimum_cycle_time_SM2_5, EC_TIMEOUTRXM); ret +=
  // ec_SDOread(5, 0x1c33, 0x05, FALSE, &l, &minimum_cycle_time_SM3_5,
  // EC_TIMEOUTRXM); ret += ec_SDOread(6, 0x1c32, 0x05, FALSE, &l,
  // &minimum_cycle_time_SM2_6, EC_TIMEOUTRXM); ret += ec_SDOread(6, 0x1c33,
  // 0x05, FALSE, &l, &minimum_cycle_time_SM3_6, EC_TIMEOUTRXM);

  struct timespec ts, tleft, tprev, tnext, tdiff, t_full_prev, t_full_next,
      t_total;
  int ht;
  int64 macrocycletime = macroc_time * 1000; /* macro cycletime in ns */
  ;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
  ts.tv_nsec = ht * 1000000;
  if (ts.tv_nsec >= NSEC_PER_SEC) {
    ts.tv_sec++;
    ts.tv_nsec -= NSEC_PER_SEC;
  }

  timespec_get(&tprev, TIME_UTC);
  ec_send_processdata();

  while (1) {
    /* calculate next cycle start */
    add_timespec(&ts, macrocycletime);

    // print_raw_buffer();
    copy_raw_buffer_to_tof_filtered_buffer();

    copy_raw_buffer_to_cydata();
    copy_cydata_to_filtered_buffer();

    // printf("\n\n set_data_ready TRUE from EcatHandler,is_data_ready=
    // %d\n\n",is_data_ready());
    /* wait for cycle start */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
    timespec_get(&t_full_prev, TIME_UTC);

    if (get_is_ecatthread_running() > 0) {
      do {
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        set_is_ecatthread_running(get_is_ecatthread_running() + 1);

        timespec_get(&tnext, TIME_UTC);
        timespec_diff(&tprev, &tnext, &tdiff);

        // print_slave_buffer(1, TRUE, TRUE);
        // print_slave_buffer(2, TRUE, TRUE);
        // print_slave_buffer(3, TRUE, TRUE);
        // print_slave_buffer(4, TRUE, TRUE);
        // print_slave_buffer(5, TRUE, TRUE);
        // print_slave_buffer(6, TRUE, TRUE);

        // ret += ec_SDOread(cnt, 0x1c32, 0x02, FALSE, &l, &cycle_time_SM2,
        // EC_TIMEOUTRXM);

        // ret += ec_SDOread(cnt, 0x1c32, 0x06, FALSE, &l,
        // &calc_and_copy_time_SM2, EC_TIMEOUTRXM);
        // ret += ec_SDOread(cnt, 0x1c33, 0x02, FALSE, &l, &cycle_time_SM3,
        // EC_TIMEOUTRXM);
        // ret += ec_SDOread(cnt, 0x1c33, 0x06, FALSE, &l,
        // &calc_and_copy_time_SM3, EC_TIMEOUTRXM);

        // ret += ec_SDOread(1, 0x1c32, 0x05, FALSE, &l,
        // &minimum_cycle_time_SM2_1, EC_TIMEOUTRXM); ret += ec_SDOread(1,
        // 0x1c33, 0x05, FALSE, &l, &minimum_cycle_time_SM3_1, EC_TIMEOUTRXM);
        // ret += ec_SDOread(2, 0x1c32, 0x05, FALSE, &l,
        // &minimum_cycle_time_SM2_2, EC_TIMEOUTRXM); ret += ec_SDOread(2,
        // 0x1c33, 0x05, FALSE, &l, &minimum_cycle_time_SM3_2, EC_TIMEOUTRXM);
        // ret += ec_SDOread(3, 0x1c32, 0x05, FALSE, &l,
        // &minimum_cycle_time_SM2_3, EC_TIMEOUTRXM); ret += ec_SDOread(3,
        // 0x1c33, 0x05, FALSE, &l, &minimum_cycle_time_SM3_3, EC_TIMEOUTRXM);
        // ret += ec_SDOread(4, 0x1c32, 0x05, FALSE, &l,
        // &minimum_cycle_time_SM2_4, EC_TIMEOUTRXM); ret += ec_SDOread(4,
        // 0x1c33, 0x05, FALSE, &l, &minimum_cycle_time_SM3_4, EC_TIMEOUTRXM);
        // ret += ec_SDOread(5, 0x1c32, 0x05, FALSE, &l,
        // &minimum_cycle_time_SM2_5, EC_TIMEOUTRXM); ret += ec_SDOread(5,
        // 0x1c33, 0x05, FALSE, &l, &minimum_cycle_time_SM3_5, EC_TIMEOUTRXM);
        // ret += ec_SDOread(6, 0x1c32, 0x05, FALSE, &l,
        // &minimum_cycle_time_SM2_6, EC_TIMEOUTRXM); ret += ec_SDOread(6,
        // 0x1c33, 0x05, FALSE, &l, &minimum_cycle_time_SM3_6, EC_TIMEOUTRXM);

        // cout << "cycle_time SM2: " << (int) cycle_time_SM2 << ";\n";
        // cout << "minimum cycle_time SM2: " << (int) minimum_cycle_time_SM2 <<
        // ";\n";
        //  cout << "calc_and_copy_time SM2: " << (int) calc_and_copy_time_SM2
        //  << ";\n";
        // cout << "cycle_time SM3: " << (int) cycle_time_SM3 << ";\n";
        // cout << "minimum cycle_time SM3: " << (int) minimum_cycle_time_SM3 <<
        // ";\n";
        //  cout << "calc_and_copy_time  SM3: " << (int) calc_and_copy_time_SM3
        //  << ";\n"; fflush(stdout);

        // myfile << (int)curSegment << ",";
        // myfile << (int)(curSegment % N_SEGMENTS) << ",";
        // myfile << (int)minimum_cycle_time_SM2_1 << ",";
        // myfile << (int)minimum_cycle_time_SM3_1 << "\n";
        // myfile << (int)minimum_cycle_time_SM2_2 << ",";
        // myfile << (int)minimum_cycle_time_SM3_2 << "\n";
        // myfile << (int)minimum_cycle_time_SM2_3 << ",";
        // myfile << (int)minimum_cycle_time_SM3_3 << "\n";
        // myfile << (int)minimum_cycle_time_SM2_4 << ",";
        // myfile << (int)minimum_cycle_time_SM3_4 << "\n";
        // myfile << (int)minimum_cycle_time_SM2_5 << ",";
        // myfile << (int)minimum_cycle_time_SM3_5 << "\n";
        // myfile << (int)minimum_cycle_time_SM2_6 << ",";
        // myfile << (int)minimum_cycle_time_SM3_6 << "\n";
        // myfile.flush();

        curSegment += 1;

        /* bound the value of out_xmc43_x->SEGMENT from 0 to (N_SEGMENTS-1) */
        xmc43_set_SEGMENT(curSegment % N_SEGMENTS);
        // printf("RUNNING... segment: %d\n", curSegment % N_SEGMENTS);

        copy_segments_to_raw_buffer(prev_offset);

        prev_offset = curr_offset;
        /* Given the current slave ID and segment, maps them from 0 to
         * N_SLAVES*N_SEGMENTS*seg_len, so that we can copy input data to the
         * buffer with the correct offset */
        tmp_offset = (out_xmc43[0]->SEGMENT);  // 0,1,2...
        curr_offset = tmp_offset * seg_len;  // if seg_len=1280 -> 0,1280,2560.

        osal_usleep(
            1000);  // MAY BE USEFUL FOR LETTING SLAVES UPDATE THEIR INPUTS

        timespec_get(&tprev, TIME_UTC);
        ec_send_processdata();

      } while (curSegment % N_SEGMENTS);

      timespec_get(&t_full_next, TIME_UTC);
      timespec_diff(&t_full_prev, &t_full_next, &t_total);
      // myfile << (int)t_total.tv_nsec << "\n";

      // cout << "Raw timespec.time_t: " << tdiff.tv_sec << "\n" << "Raw
      // timespec.tv_nsec: " << tdiff.tv_nsec << "\n";
    }
  }

  return NULL;
}

void EcatHandler::init_ecatcheck() {
  int ret;
  pthread_attr_t attr;
  pthread_t *threadp;

  threadp = &check_thread;
  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, stack64k * 4);
  ret = pthread_create(threadp, &attr, (THREADFUNCPTR)&EcatHandler::ecatcheck,
                       this);
  // Clean up resources
  pthread_attr_destroy(&attr);
  if (ret != 0) {
    cerr << "Failed to create ecatcheck thread. Error code: " << ret << endl;
    return;
  }
  // Thread created successfully
}

void EcatHandler::init_ecatthread() {
  int ret;
  pthread_attr_t attr;
  struct sched_param schparam;
  pthread_t *threadp;

  threadp = &ecat_thread;
  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, stack64k * 2);
  ret = pthread_create(threadp, &attr, (THREADFUNCPTR)&EcatHandler::ecatthread,
                       this);
  pthread_attr_destroy(&attr);
  if (ret != 0) {
    cerr << "Failed to create ecatthread thread. Error code: " << ret << endl;
    return;
  }
  memset(&schparam, 0, sizeof(schparam));
  schparam.sched_priority = 40;
  pthread_setschedparam(*threadp, SCHED_FIFO, &schparam);
  if (ret != 0) {
    // Error occurred while setting thread scheduling parameters
    cerr << "Failed to set scheduling parameters for ecatthread. Error code: "
         << ret << endl;
    return;
  }
}

void EcatHandler::set_RT_thread_macrocycle(int time) {
  this->macroc_time = time;
}
void EcatHandler::set_ecatcheck_sleep(int time) { this->thread_sleep = time; }

void EcatHandler::set_ECAT_STATE(uint8_t state) { ECAT_STATE = state; }
uint8_t EcatHandler::get_ECAT_STATE() { return ECAT_STATE; }

OSAL_THREAD_FUNC *EcatHandler::ecatcheck(void *ptr) {
  int slave;
  int sleep = thread_sleep * 1000; /* sleep in ns */

  while (1) {
    // Check if inOP and there are slaves that are not responding
    if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
      if (needlf) {
        needlf = FALSE;
        printf("\n");
      }
      // One or more slaves are not responding, read the state of all slaves
      ec_group[currentgroup].docheckstate = FALSE;
      ec_readstate();

      // Iterate through all slaves
      for (slave = 1; slave <= ec_slavecount; slave++) {
        // Check if the slave belongs to the current group and is not in
        // operational state
        if ((ec_slave[slave].group == currentgroup) &&
            (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
          ec_group[currentgroup].docheckstate = TRUE;

          // Check the state of the slave and perform appropriate actions
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            printf("ERROR: slave %d is in SAFE_OP + ERROR, attempting ack.\n",
                   slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
          } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
            printf("WARNING: slave %d is in SAFE_OP, change to OPERATIONAL.\n",
                   slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
          } else if (ec_slave[slave].state > EC_STATE_NONE) {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              printf("MESSAGE: slave %d reconfigured\n", slave);
            }
          } else if (!ec_slave[slave].islost) {
            // Re-check the state of the slave
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (ec_slave[slave].state == EC_STATE_NONE) {
              ec_slave[slave].islost = TRUE;
              printf("ERROR: slave %d lost\n", slave);
            }
          }
        }

        // Handle the case when a slave is lost or recovered
        if (ec_slave[slave].islost) {
          if (ec_slave[slave].state == EC_STATE_NONE) {
            if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
              ec_slave[slave].islost = FALSE;
              printf("MESSAGE: slave %d recovered\n", slave);
            }
          } else {
            ec_slave[slave].islost = FALSE;
            printf("MESSAGE: slave %d found\n", slave);
          }
        }
      }

      // Check if all slaves have resumed operational state
      if (!ec_group[currentgroup].docheckstate)
        printf("OK: all slaves resumed OPERATIONAL.\n");
    }

    // Sleep for the specified duration
    osal_usleep(sleep);
  }

  return NULL;
}

generic_buffer_tof *EcatHandler::getTofGenericBuffer() {
  return tof_filtered_buffer;
}

generic_buffer_cyskin *EcatHandler::getCyskinGenericBuffer() {
  return cyskin_filtered_buffer;
}