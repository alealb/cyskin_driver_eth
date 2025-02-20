/**
 * @file
 * @author Francesco Giovinazzo
 * @copyright 2024 Mechatronics and Automatic Control Laboratory (MACLAB)
 * University of Genoa
 */

#include <memory>

#include "skin/communication/driver_interface.hpp"

#ifndef INCLUDE_CYSKIN_DRIVER_CYSKIN_DRIVER_HPP_
#define INCLUDE_CYSKIN_DRIVER_CYSKIN_DRIVER_HPP_

// ecat comm
#include "cyskin_driver/ecatHandler.hpp"  

// #define DEFAULT_MACROCYCLE 66777         // CySKIN frequency in us
#define DEFAULT_MACROCYCLE 50000  // CySKIN frequency in us
#define DEFAULT_CHECK_THREAD_SLEEP \
  10000  // Default sleep of ecatcheck() thread, which handles the EtherCAT
         // State Machine (OP, PRE-OP, SAFE-OP etc.)

class DriverCyskin : public skin::DriverInterface {
 public:
  explicit DriverCyskin(const char* ifname_);
  ~DriverCyskin();

  bool Attach() final;
  void Update() final;
  bool Detach() final;

 private:
  std::shared_ptr<EcatHandler> ecat_;
  generic_buffer_cyskin* genbuf_cyskin_;
  skin::SensorsResponses cyskin_baseline_;
  const char* network_name;
};

DriverCyskin::DriverCyskin(const char* ifname_) {
  network_name = ifname_;
}

DriverCyskin::~DriverCyskin() { Detach(); }

bool DriverCyskin::Attach() {
  // Ecat config params
  int macroc_time = DEFAULT_MACROCYCLE;
  int check_thread_sleep = DEFAULT_CHECK_THREAD_SLEEP;

  // Ecat handler
  try {
    ecat_ = std::make_unique<EcatHandler>(const_cast<char*>(network_name),
                                          macroc_time, check_thread_sleep);
    // EtherCAT Master waiting for all patches to be ready
    while (ecat_->get_ECAT_STATE() < DATA_ACQUISITION_STATE) continue;
    std::cout << "\nEtherCAT network ready\n";
  } catch (const std::exception& e) {
    std::cerr << "Exception occurred: " << e.what() << std::endl;
    exit(EXIT_FAILURE);
  }

  // get ecat buffer info

  genbuf_cyskin_ = ecat_->getCyskinGenericBuffer();
  // get the number of sensors
  size_t num_of_sensors = genbuf_cyskin_->size;
  size_t num_of_channels = 2;

  AddDeviceInfo(skin::DeviceInfo{skin::DeviceType::kPressure, num_of_sensors,
                                 num_of_channels});

  // // OPTIONAL //////////////////////////////////////////////////////////////

  // // Define the tree structure. By default, the tree is constructed
  // // with 1 patch, and 1 module for each device added with AddDeviceInfo.

  // // Only Patches and Modules need to be define. Multiple channels are
  // // automatically added. In this example there is a single device with 20
  // // sensors. I define the tree as composed by 2 patches, each one with 10
  // // modules.

  // // Start by defining the structure of the channel
  // skin::TreeChannelNode channel;
  // // Add 2 patches to the channel
  // channel.AddNode(skin::TreePatchNode());

  // // Add 10 modules to each patch. A Module represents a set of sensors
  // stored
  // // in memory contiguously. To add a module you need to specify the start
  // index
  // // of the sensors and the number of sensors.
  // // The first module starts at index 0 and contains 10 sensors.
  // channel.GetPatches()[0].AddNode(skin::TreeModule(0, 640));

  // // Connect the channel to the device. The second argument is the index of
  // the
  // // device (0 by default). If you have multiple devices, you may want to
  // // specify it. IMPORTANT: ConnectChannel will delete the default channel
  // // structure and MUST be called after AddDeviceInfo.
  // ConnectChannel(std::move(channel));

  // // END OF OPTIONAL PART ////////////////////////////////////////////////

  skin::sensorIds_t uids(GetNumberOfSensors());
  for (int i = 0; i < devices_info[0].number_of_sensors; i++) {
    uids[i] = genbuf_cyskin_->cyskin_uids[i];
    std::cout << static_cast<int>(uids[i]) << ",  ";
  }
  std::cout << "\n\n  ";

  SetSensorsUIds(devices_info[0], uids);

  // Set the baseline
  genbuf_cyskin_ = ecat_->getCyskinGenericBuffer();
  cyskin_baseline_.resize(GetNumberOfSensors());
  for (int i = 0; i < devices_info[0].number_of_sensors; i++) {
    cyskin_baseline_[i] = genbuf_cyskin_->cyskin_responces[i];
  }

  // Remember to set is_attached_ to true when the connection is established
  is_attached_ = true;
  return true;
}

void DriverCyskin::Update() {
  genbuf_cyskin_ = ecat_->getCyskinGenericBuffer();

  // Get sensors responses
  skin::SensorsResponses sensors_readings(GetNumberOfMeasurements());
  for (int i = 0; i < devices_info[0].number_of_sensors; i++) {
    sensors_readings[i] = genbuf_cyskin_->cyskin_responces[i];
  }
  // Add the baseline
  // sensors_readings.insert(sensors_readings.end(), cyskin_baseline_.begin(),cyskin_baseline_.end());

  SetSensorsResponses(devices_info[0], std::move(sensors_readings));
}

bool DriverCyskin::Detach() {
  // This is automatically called by the destructor.

  // (i) Check if the driver is attached first
  if (!is_attached_) return false;

  // (ii) Then close the connection

  // (iii) Set is_attached_ to false
  is_attached_ = false;
  return true;
}

#endif  // INCLUDE_CYSKIN_DRIVER_CYSKIN_DRIVER_HPP_
