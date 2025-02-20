
#include <algorithm>
#include <chrono>
#include <csignal>
#include <random>
#include <thread>
#include <vector>

#include "cyskin_driver/cyskin_driver.hpp"
#include "skin/communication/writer/shared_memory_writer.hpp"

std::unique_ptr<skin::SharedMemoryWriter> writer;
std::unique_ptr<skin::DriverInterface> driver;

void signalHandler(int signum) {
  // Stop the Writer
  writer->Stop();
}

int main(int argc, char* argv[]) {
  if (argc <= 1) {
    std::cout << "Usage: main ifname macro_cycletime check_thread_sleep\n"
                 "    - ifname = enp2s0 for example\n";
    return 1;
  }

  const char* ifname = argv[1];

  driver = std::make_unique<DriverCyskin>(ifname);

  // Register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  // Create the Driver for CySkin and bind it to the Writer
  writer = std::make_unique<skin::SharedMemoryWriter>(0.1, *driver);

  // Define a thread to print the content of the shared memory every second
  std::thread print_thread([&] {
    while (!writer->IsRunning()) continue;
    while (writer->IsRunning()) {
      std::cout << *writer << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  });

  // Start the Writer
  writer->Write();

  // Join the printing thread
  print_thread.join();

  std::cout << "Writer stopped.\n";

  return 1;
}
