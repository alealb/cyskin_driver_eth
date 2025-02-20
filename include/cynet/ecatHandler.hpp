/**
 * @file ecatHandler.hpp
 *
 * @brief Header file for the EcatHandler class.
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include <inttypes.h>
#include "ethercat.h"
#include <functional>   // std::bind
#include "msg_struct.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable
#include <atomic>
#include "cynet_ihb.h"
#include <fstream>


#ifndef ECATHANDLER_H
#define ECATHANDLER_H

using namespace std;

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500
#define stack64k (64 * 1024)

// TOF-SPECIFIC MACROS
#define NUMBER_TOF_ENDPOINTS 64
#define NUMBER_TOF_CHANNELS 3

#define NUMBER_CYSKIN_ENDPOINTS 1
#define NUMBER_CYSKIN_CHANNELS  1

#define N_SEGMENTS 3 /**< Number of segments needed for each slave's payload.*/
#define SEG_LEN 1280//1280 /**< Length of the segment. TODO: Find a way to retrieve the dimension of the first PDO only. */

#define TYPE_TOF_SENSOR         0
#define TYPE_CYSKIN_SENSOR      1

/**
 * @brief Flag to determine whether to use a vector for buffer storage.
 * 
 * If set to TRUE, the content of 'raw' buffer is copied into 'data_vector' instead of 'filtered_buffer'.
 * WARNING: This feature is not supported by Updater and SharedMemoryPublisher classes.
 */
#define USE_VECTOR 0

/* EtherCAT State Machine */
#define IDLE_STATE              0 /**< Idle state until all slaves go to OP-state. */
#define INIT_STATE              1 /**< Requests initialization data from slaves. */
#define DATA_ACQUISITION_STATE  2 /**< Requests sensor measurements from slaves. */

#define TOF_HIGH_RESOLUTION         64
#define TOF_LOW_RESOLUTION          16
#define TOF_MAX_FREQUENCY_H_RES     15
#define TOF_MAX_FREQUENCY_L_RES     30
#define TOF_SHARPENER_DEFAULT       20


#define COMPUTE_UID_TOF(a,b,c)({a << 10 | b << 6 | c;})
#define COMPUTE_UID_CYSKIN(a,b)({a << 4 | b;})

/**
 * @brief Structure of the output commands (Master->Slaves).
 */
typedef struct PACKED
{
    uint8_t MSG_ID;
    uint8_t SEGMENT;  /**< Starts from 0 to 2 (3 frames for each slave) */
    uint8_t RES_TOF;
    uint8_t FREQ_TOF; 
    uint8_t SHARP_TOF;
    uint8_t CDC_CYSKIN;
    uint8_t PERIOD_CYSKIN; 
    uint8_t SPI_CYSKIN; 
} out_xmc43_t;

typedef struct PACKED
{
    uint8_t arr[SEG_LEN];
    uint16_t slave_id;
    uint16_t sensor_type;
} in_xmc43_t;

typedef struct generic_buffer_tof
{
    uint32_t* tof_uids;
    uint32_t* tof_range_status;
    uint32_t* tof_range_value;
    uint32_t* tof_range_sigma;
    int size;
    uint16_t active_sensors;
} generic_buffer_tof;

typedef struct generic_buffer_cyskin
{
    uint32_t* cyskin_uids;
    uint32_t* cyskin_responces;
    int size;
    uint16_t active_sensors;
} generic_buffer_cyskin;

typedef struct slave_info
{
    uint16_t slave_id;
    uint16_t sensor_type;
} slave_info;

typedef void *(*THREADFUNCPTR)(void *);

/**
 * @class EcatHandler
 * @brief Class for handling EtherCAT communication.
 */
class EcatHandler
{
private:
    uint8_t ECAT_STATE = IDLE_STATE; /**< EtherCAT Finite State Machine state */

    uint8_t *buffer; /**< 'raw' local buffer, slaves' input PDOs are copied into it */
    generic_buffer_tof *tof_filtered_buffer; /**< 'filtered' buffer, copies data only valid sensor data from raw buffer */
    generic_buffer_cyskin *cyskin_filtered_buffer; /**< 'filtered' buffer, copies data only valid sensor data from raw buffer */
    std::vector<CyIhb*> ihbs;

    uint8_t actual_TOF_number = 0; /**< Number of detected valid TOFs */
    uint16_t taxel_number = 0;

    bool remove_baseline;
    uint32_t steps_baseline;
    std::vector<uint32_t> baseline;

    uint8_t **TOF_mask; /**< Array of arrays (2D array) for each slave with active TOF values ('0': not valid, '1': valid) */
    uint16_t *slave_ids;
    uint16_t *sensor_types;
    uint8_t *TOFs_in_slave;

    //vector<vector<VL53LX_DATA_64_MSG *>> data_vector; /**< Dynamic array of vectors, used ONLY if USE_VECTOR == TRUE */

    uint8_t N_SLAVES;
    uint8_t N_TOF_SLAVES = 0;
    uint8_t N_CYSKIN_SLAVES = 0;
    uint16_t seg_len;

    uint16_t prev_offset = 0;   /**< Offset and slave id needed to copy input data to the correct location of the buffer */
    uint16_t tmp_offset = 0;
    uint16_t curr_offset = 0;
    uint16_t curSegment;

    out_xmc43_t **out_xmc43;
    VL53LX_FULL_INIT_MSG **in_xmc43_init; /**< input PDOs are casted to this structure when ECAT_STATE = INIT_STATE */
    VL53LX_FULL_DATA_64_MSG **in_xmc43_data; /**< input PDOs are casted to this structure when ECAT_STATE = DATA_ACQUISITION_STATE */
    in_xmc43_t **in_xmc43;
    int *oloop; /**< number of output bytes for each slave (index '0' is for master) */
    int *iloop; /**< number of input bytes for each slave (index '0' is for master) */

    pthread_t ecat_thread, check_thread;
    struct sched_param schedp;
    //char IOmap[4096]; /**<IOmap that refers to PDO mapping */
    char IOmap[8000]; /**<IOmap that refers to PDO mapping */
    int expectedWKC;
    boolean needlf;
    volatile int wkc;
    struct timeval tv, t1, t2;
    uint8 currentgroup = 0;

    int dorun = 0;
    boolean inOP;

    //std::atomic<int> bounded_buffer;
    int bounded_buffer = 0;
    std::mutex mutex_data;
    std::condition_variable cv_data;
    // std::atomic_bool updated = { false };
    // //std::atomic<bool> updated = false;
    bool updated = false;

    std::ofstream myfile;

        
    int macroc_time;
    int thread_sleep;

    void add_timespec(struct timespec *_ts, int64 _addtime);
    void print_slave_buffer(uint8_t slave, uint8_t printOutput, uint8_t printInput);
    void print_raw_buffer();
    void print_raw_buffer_single_slave(uint8_t slave);
    void print_TOF_mask();
    void print_data_vector();
    void print_generic_tof_buffer(generic_buffer_tof *b);
    void print_generic_cyskin_buffer(generic_buffer_cyskin *b);
    void xmc43_output_cast();
    void xmc43_input_cast();

    /**
     * @brief Set MSG_ID, i.e., output command related to the kind of data the master wants to receive.
     * @param val MSG_ID value
     */
    void xmc43_set_MSG_ID(uint8_t val);

    /**
     * @brief Set SEGMENT, i.e., output command related to the i-th segment master wants to receive. (Only used when MSG_ID == DATA_ACQUISITION_STATE)
     * @param val SEGMENT value
     */
    void xmc43_set_SEGMENT(uint8_t val);

    /**
     * @brief Set PARAMETERS, i.e., output command used to configure the sensor acquisition paraments 
     * @param val1 value of param1
     * @param val2 value of param2
     * @param val3 value of param3
     */
    void xmc43_set_PARAMETERS(uint8_t val1, uint8_t val2, uint8_t val3);

    /**
     * @brief Copy a certain segment of sensor data from each slave's input space to 'raw' local buffer.
     *  
     * Please note that the slave and offset we are interested in are the ones of the previous cycle, 
     * since we get a response after a cycle.
     * @param offset Offset value
     */
    void copy_segments_to_raw_buffer(uint16_t offset);

    // TODO Overloading on following functions

    /**
     * @brief Copy content of 'raw' local buffer to filtered_buffer. (USE_VECTOR == FALSE)
     */
    void copy_raw_buffer_to_tof_filtered_buffer();

    // /**
    //  * @brief Copy content of 'raw' local buffer to data_vector. (USE_VECTOR == TRUE)
    //  */
    // void copy_raw_buffer_to_vector();

    void copy_raw_buffer_to_cydata();
    void copy_cydata_to_filtered_buffer();

public:
    /**
     * @brief Constructor for the EcatHandler class.
     *
     * @param ifname The name of the Ethernet interface.
     * @param macroc_time The macro cycle time.
     * @param check_thread_sleep The sleep time for the check thread.
     */
    EcatHandler(char *ifname, int macroc_time, int check_thread_sleep);

    /**
     * @brief Destructor.
     * 
     * Request safe operational state for all slaves.
     * Close the socket.
     */
    ~EcatHandler();

    /**
     * @brief EtherCAT Master initialization.
     * 
     * Initialize SOEM, bind socket to ifname.
     * Find and auto-config slaves.
     * Configure IOmap.
     * Allocate 'raw' local buffer.
     * Activate cyclic process data.
     * Request and wait for all slaves to reach OP state.
     */
    void ecat_init(char *ifname);

    /**
     * @brief Set the macrocycle period.
     * @param time Macrocycle period
     */
    void set_RT_thread_macrocycle(int time);

    /**
     * @brief Set the sleep time of ecatcheck thread.
     * @param time Sleep time of ecatcheck thread
     */
    void set_ecatcheck_sleep(int time);

    /**
     * @brief Initialize ecatcheck thread.
     */
    void init_ecatcheck();

    /**
     * @brief Initialize ecatthread thread.
     */
    void init_ecatthread();

    /**
     * @brief Thread function for EtherCAT communication and data acquisition.
     *
     * This function is responsible for handling EtherCAT communication with the slaves and performing data acquisition.
     *
     * @param ptr Pointer to the thread data (not used in this function).
     * @return NULL
     */
    OSAL_THREAD_FUNC_RT *ecatthread(void *ptr);

    /**
     * @brief Thread function for EtherCAT slave state checking.
     *
     * This function is responsible for continuously checking the state of EtherCAT slaves and performing
     * appropriate actions based on their current state.
     *
     * @param ptr Pointer to the thread data (not used in this function).
     * @return NULL
     */
    OSAL_THREAD_FUNC *ecatcheck(void *ptr);

    /**
     * @brief Set ECAT_STATE.
     * @param state EtherCAT state
     */
    void set_ECAT_STATE(uint8_t state);

    /**
     * @brief Get ECAT_STATE.
     * @return ECAT_STATE value
     */
    uint8_t get_ECAT_STATE();

    /**
     * @brief Get the generic_buffer_tof struct.
     * @return Pointer to the generic_buffer_tof struct
     */
    generic_buffer_tof *getTofGenericBuffer();

    /**
     * @brief Get the generic_buffer_cyskin struct.
     * @return Pointer to the generic_buffer_cyskin struct
     */
    generic_buffer_cyskin *getCyskinGenericBuffer();

    /**
     * @brief Set the is_ecatthread_running value.
     * @param count is_ecatthread_running value. If count > 0 ecatthread is running.
     */
    void set_is_ecatthread_running(int count);

    /**
     * @brief Get the is_ecatthread_running value.
     * @return is_ecatthread_running value. If > 0 ecatthread is running.
     */
    int get_is_ecatthread_running();


    /**
     * @brief Swaps the buffer just after the EtherCAT process data has been copied to one of the two buffers.
     * @return NULL
     */
    void swapBuffer() ;

    /**
     * @brief Wait until the EtherCAT process data has been copied to the local buffer. After that, it is possible to publish it to the shared memory.
     * @return NULL
     */
    void makeThisThreadWait();

    
};

#endif

