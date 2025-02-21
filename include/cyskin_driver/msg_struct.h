/**
 * @file msg_struct.h
 *
 * @brief Definition of all structures for TOF data acquisition.
 */
#ifndef INCLUDE_CYSKIN_DRIVER_MSG_STRUCT_H_
#define INCLUDE_CYSKIN_DRIVER_MSG_STRUCT_H_

#include "cynetworklib/cynet_ihb.h"

/* IDs of OUTCOMING MESSAGES */
#define ID_VL53LX_INIT_MSG 1
#define ID_VL53LX_DATA_MSG 2

/* IDs of OUTCOMING MESSAGES */
#define ID_VL53LX_CONFIG_MSG \
  111  // config sensor resolution, frequency and sharpener

// LEN of MESSAGES
#define LEN_VL53LX_INIT_MSG 6
#define LEN_VL53LX_FULL_INIT_MSG 64

#define LEN_VL53LX_CONFIG_MSG 6

#define LEN_VL53LX_DATA_64_MSG 322
#define LEN_VL53LX_DATA_16_MSG 82

#define LEN_VL53LX_FULL_DATA_64_MSG 3246
#define LEN_VL53LX_FULL_DATA_16_MSG 652

#define VL53L5CX_RESOLUTION_64 64
#define VL53L5CX_RESOLUTION_16 16

#define TOF_IS_ACTIVE 0
#define TOF_TOTAL_NUMBER 10

#pragma pack(push)
#pragma pack(1)

// dichiarazione della struttura dell'header comune a tutti i messaggi
typedef struct {
  uint8_t msg_id;
  uint8_t sensor_type;
  uint8_t msg_len_H;
  uint8_t msg_len_L;
} MSG_HEADER;

typedef struct {
  uint32_t ihb_id;
  uint8_t tot_modules_cnt;
} IhbInfo;

typedef struct {
  uint8_t module_global_id;
  uint32_t module_sui_id;
  uint8_t module_sensor_num;
  uint8_t module_sensor_group;
  uint16_t module_sensor_uncut;
  uint8_t module_cdc_control;
  uint8_t module_cdc_speed;
} ModuleInfo;

typedef struct {
  MSG_HEADER header;
  IhbInfo ihb_info;
  ModuleInfo* mod_info;
} CYSKIN_INIT_MESSAGE;

typedef struct {
  MSG_HEADER header;
  uint8_t module_id;
  uint8_t sensor_id;
  uint16_t* sensor_data;
} CYSKIN_DATA_MESSAGE;

typedef struct {
  uint8_t tof_id;
  uint8_t tof_status;
  uint8_t status_resolution;
  uint8_t status_ranging_frequency;
  uint8_t status_ranging_mode;
  uint8_t status_sharpener_percent;
} VL53LX_INIT_MSG;

typedef struct {
  MSG_HEADER header;
  VL53LX_INIT_MSG init_tof[TOF_TOTAL_NUMBER];
} VL53LX_FULL_INIT_MSG;  // 63

// typedef struct
// {
//     uint8_t id_tof;
//     uint8_t status_get_measurement;
//     uint8_t range_status[VL53L5CX_RESOLUTION_64];
//     uint16_t range_val[VL53L5CX_RESOLUTION_64];
//     uint16_t sigma[VL53L5CX_RESOLUTION_64];

// } VL53LX_DATA_64_MSG;      //len 322

// typedef struct
// {
//     MSG_HEADER header;
//     VL53LX_DATA_64_MSG tof[TOF_TOTAL_NUMBER];
//     uint8_t is_ready[TOF_TOTAL_NUMBER];
//     uint8_t status_ready[TOF_TOTAL_NUMBER];
//     uint16_t ms_count;

// } VL53LX_FULL_DATA_64_MSG; //len 3245

typedef struct {
  MSG_HEADER header;
  uint8_t id_tof[TOF_TOTAL_NUMBER];
  uint8_t status_get_measurement[TOF_TOTAL_NUMBER];
  uint8_t range_status[VL53L5CX_RESOLUTION_64 * TOF_TOTAL_NUMBER];
  uint8_t range_val[VL53L5CX_RESOLUTION_64 * TOF_TOTAL_NUMBER];
  uint8_t sigma[VL53L5CX_RESOLUTION_64 * TOF_TOTAL_NUMBER];
  uint8_t is_ready[TOF_TOTAL_NUMBER];
  uint8_t status_ready[TOF_TOTAL_NUMBER];
  uint16_t ms_count;
} VL53LX_FULL_DATA_64_MSG;  // len 3245

typedef struct {
  uint8_t id_tof;
  uint8_t status_get_measurement;
  uint8_t range_status[VL53L5CX_RESOLUTION_16];
  uint16_t range_val[VL53L5CX_RESOLUTION_16];
  uint16_t sigma[VL53L5CX_RESOLUTION_16];
} VL53LX_DATA_16_MSG;  // len 82

typedef struct {
  MSG_HEADER header;
  VL53LX_DATA_16_MSG tof[TOF_TOTAL_NUMBER];  // 820
  uint8_t is_ready[TOF_TOTAL_NUMBER];
  uint8_t status_ready[TOF_TOTAL_NUMBER];
  uint16_t ms_count;
} VL53LX_FULL_DATA_16_MSG;  // len 845

typedef struct {
  MSG_HEADER header;
  uint8_t resolution;
  uint8_t frequency;
  uint8_t sharpener;
} VL53LX_CONFIG_MSG;  // len 6

typedef struct {
  MSG_HEADER header;
  uint8_t stop;
} VL53LX_STOP_MSG;  // len 4

#pragma pack(pop)

#endif  // INCLUDE_CYSKIN_DRIVER_MSG_STRUCT_H_
