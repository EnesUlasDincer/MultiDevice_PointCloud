// new_file.hpp
#ifndef NEW_FILE_HPP
#define NEW_FILE_HPP

#include <algorithm>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../include/cJSON.h"
#include "window.hpp"
#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#else
#include <strings.h>
#endif

#define MAX_DEVICE_COUNT 9
#define CURRENT_DIRERCTORY "/home/enes/Desktop/ORBBEC/Orbbec_Codes/MultiDevice/"
#define CONFIG_FILE CURRENT_DIRERCTORY "/MultiDeviceSyncConfig.json"

typedef struct DeviceConfigInfo_t {
  std::string deviceSN;
  OBMultiDeviceSyncConfig syncConfig;
} DeviceConfigInfo;

// First, let's properly define the PipelineHolder structure
typedef struct PipelineHolder_t {
    std::shared_ptr<ob::Pipeline> pipeline;
    OBSensorType sensorType;
    int deviceIndex;
    std::string deviceSN;
    float depthValueScale;
} PipelineHolder;

std::ostream &operator<<(std::ostream &os, const PipelineHolder &holder);
std::ostream &operator<<(std::ostream &os,
                         std::shared_ptr<PipelineHolder> holder);

extern std::mutex frameMutex;
extern std::map<uint8_t, std::shared_ptr<ob::Frame>> colorFrames;
extern std::map<uint8_t, std::shared_ptr<ob::Frame>> depthFrames;

extern std::vector<std::shared_ptr<ob::Device>> streamDevList;
extern std::vector<std::shared_ptr<ob::Device>> configDevList;
extern std::vector<std::shared_ptr<DeviceConfigInfo>> deviceConfigList;

extern std::vector<ob::PointCloudFilter> pointCloudFilters;

extern std::vector<std::shared_ptr<PipelineHolder>> pipelineHolderList;

extern std::condition_variable waitRebootCompleteCondition;
extern std::mutex rebootingDevInfoListMutex;
extern std::vector<std::shared_ptr<ob::DeviceInfo>> rebootingDevInfoList;

OBFrameType mapFrameType(OBSensorType sensorType);
OBMultiDeviceSyncMode textToOBSyncMode(const char *text);
std::string readFileContent(const char *filePath);
bool loadConfigFile();
int configMultiDeviceSync();
int testMultiDeviceSync();
bool checkDevicesWithDeviceConfigs(
    const std::vector<std::shared_ptr<ob::Device>> &deviceList);
int strcmp_nocase(const char *str0, const char *str1);

// std::shared_ptr<PipelineHolder> createPipelineHolder(
//     std::shared_ptr<ob::Device> device, OBSensorType sensorType,
//     int deviceIndex);

void startStream(std::shared_ptr<PipelineHolder> holder, ob::PointCloudFilter &pointCloudFilter) ;
void stopStream(std::shared_ptr<PipelineHolder> pipelineHolder);

void handleColorStream(int devIndex, std::shared_ptr<ob::Frame> frame);
void handleDepthStream(int devIndex, std::shared_ptr<ob::Frame> frame);

extern ob::Context context;

void wait_any_key();

#endif // NEW_FILE_HPP