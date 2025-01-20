
#include "../include/multiDevice_funcs.hpp"
#include <iostream>

std::mutex frameMutex;
std::map<uint8_t, std::shared_ptr<ob::Frame>> colorFrames;
std::map<uint8_t, std::shared_ptr<ob::Frame>> depthFrames;

std::vector<std::shared_ptr<ob::Device>> streamDevList;
std::vector<std::shared_ptr<ob::Device>> configDevList;
std::vector<std::shared_ptr<DeviceConfigInfo>> deviceConfigList;
std::vector<ob::PointCloudFilter> pointCloudFilters;

std::vector<std::shared_ptr<PipelineHolder>> pipelineHolderList;

std::condition_variable waitRebootCompleteCondition;
std::mutex rebootingDevInfoListMutex;
std::vector<std::shared_ptr<ob::DeviceInfo>> rebootingDevInfoList;

ob::Context context;


void wait_any_key() { system("pause"); }

int configMultiDeviceSync() try {
  if (!loadConfigFile()) {
    std::cout << "load config failed" << std::endl;
    return -1;
  }

  if (deviceConfigList.empty()) {
    std::cout << "DeviceConfigList is empty. please check config file: "
              << CONFIG_FILE << std::endl;
    return -1;
  }

  // Query the list of connected devices
  auto devList = context.queryDeviceList();

  // Get the number of connected devices
  int devCount = devList->deviceCount();
  for (int i = 0; i < devCount; i++) {
    configDevList.push_back(devList->getDevice(i));
  }

  if (configDevList.empty()) {
    std::cerr << "Device list is empty. please check device connection state"
              << std::endl;
    return -1;
  }

  // write configuration to device
  for (auto config : deviceConfigList) {
    auto findItr = std::find_if(
        configDevList.begin(), configDevList.end(),
        [config](std::shared_ptr<ob::Device> device) {
          auto serialNumber = device->getDeviceInfo()->serialNumber();
          return strcmp_nocase(serialNumber, config->deviceSN.c_str()) == 0;
        });
    if (findItr != configDevList.end()) {
      auto device = (*findItr);

      auto curConfig = device->getMultiDeviceSyncConfig();

      // Update the configuration items of the configuration file, and keep the
      // original configuration for other items
      curConfig.syncMode = config->syncConfig.syncMode;
      curConfig.depthDelayUs = config->syncConfig.depthDelayUs;
      curConfig.colorDelayUs = config->syncConfig.colorDelayUs;
      curConfig.trigger2ImageDelayUs = config->syncConfig.trigger2ImageDelayUs;
      curConfig.triggerOutEnable = config->syncConfig.triggerOutEnable;
      curConfig.triggerOutDelayUs = config->syncConfig.triggerOutDelayUs;
      curConfig.framesPerTrigger = config->syncConfig.framesPerTrigger;

      device->setMultiDeviceSyncConfig(curConfig);
    }
  }

  configDevList.clear();

  // Logout callback to avoid affecting the next test multi-machine
  // synchronization
  context.setDeviceChangedCallback(nullptr);

  return 0;
} catch (ob::Error &e) {
  std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs()
            << "\nmessage:" << e.getMessage()
            << "\ntype:" << e.getExceptionType() << std::endl;
  wait_any_key();
  exit(EXIT_FAILURE);
}

int testMultiDeviceSync() try {
 
  streamDevList.clear();
  // Query the list of connected devices
  auto devList = context.queryDeviceList();

  // Get the number of connected devices
  int devCount = devList->deviceCount();
  for (int i = 0; i < devCount; i++) {
    streamDevList.push_back(devList->getDevice(i));
  }

  if (streamDevList.empty()) {
    std::cerr << "Device list is empty. Please check device connection state."
              << std::endl;
    return -1;
  }

  // Configure all devices in standalone mode and prepare pipelines
  int deviceIndex = 0;
  for (auto &dev : streamDevList) {
    auto config = dev->getMultiDeviceSyncConfig();
    config.syncMode = OB_MULTI_DEVICE_SYNC_MODE_STANDALONE;
    dev->setMultiDeviceSyncConfig(config);

    std::cout << "Configured device " << deviceIndex
              << " to standalone mode." << std::endl;

    // Create pipelines for depth and color streams
    auto depthHolder = createPipelineHolder(dev, OB_SENSOR_DEPTH, deviceIndex);
    pipelineHolderList.push_back(depthHolder);
    startStream(depthHolder);

    auto colorHolder = createPipelineHolder(dev, OB_SENSOR_COLOR, deviceIndex);
    pipelineHolderList.push_back(colorHolder);
    startStream(colorHolder);

    
    // Initialize PointCloudFilter and set camera parameters
    ob::PointCloudFilter pointCloudFilter;
    auto cameraParam = dev->getCalibrationCameraParamList()->getCameraParam(0);
    pointCloudFilter.setCameraParam(cameraParam);
    pointCloudFilters.push_back(pointCloudFilter);

    deviceIndex++;
  }

  // Create a window for rendering and set the resolution of the window
  Window app("MultiDeviceSyncViewer", 1600, 900, RENDER_GRID);
  app.setShowInfo(true);

  while (app) {
    // Get the key value of the key event
    auto key = app.waitKey();
    if (key == 'S' || key == 's') {
      std::cout << "syncDevicesTime..." << std::endl;
      context.enableDeviceClockSync(3600000);  // Manual update synchronization
    } else if (key == 'T' || key == 't') {
      // software trigger
      for (auto &dev : streamDevList) {
        auto multiDeviceSyncConfig = dev->getMultiDeviceSyncConfig();
        if (multiDeviceSyncConfig.syncMode ==
            OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING) {
          dev->triggerCapture();
        }
      }
    }
    
    


    std::vector<std::shared_ptr<ob::Frame>> framesVec;
    {
      std::lock_guard<std::mutex> lock(frameMutex);
      for (int i = 0; i < MAX_DEVICE_COUNT; i++) {

        if (depthFrames[i] != nullptr) {
          framesVec.emplace_back(depthFrames[i]);
        }
        if (colorFrames[i] != nullptr) {
          framesVec.emplace_back(colorFrames[i]);
        }
      }
    }
    // Render a set of frame in the window, where the depth and color frames of
    // all devices will be rendered.
    app.addToRender(framesVec);
  }

  // close data stream
  for (auto itr = pipelineHolderList.begin(); itr != pipelineHolderList.end();
       itr++) {
    stopStream(*itr);
  }
  pipelineHolderList.clear();

  std::lock_guard<std::mutex> lock(frameMutex);
  depthFrames.clear();
  colorFrames.clear();

  // Release resource
  streamDevList.clear();
  configDevList.clear();
  deviceConfigList.clear();
  return 0;
} catch (ob::Error &e) {
  std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs()
            << "\nmessage:" << e.getMessage()
            << "\ntype:" << e.getExceptionType() << std::endl;
  wait_any_key();
  exit(EXIT_FAILURE);
}

std::shared_ptr<PipelineHolder> createPipelineHolder(
    std::shared_ptr<ob::Device> device, OBSensorType sensorType,
    int deviceIndex) {
  PipelineHolder *pHolder = new PipelineHolder();
  pHolder->pipeline = std::shared_ptr<ob::Pipeline>(new ob::Pipeline(device));
  pHolder->sensorType = sensorType;
  pHolder->deviceIndex = deviceIndex;
  pHolder->deviceSN = std::string(device->getDeviceInfo()->serialNumber());

  return std::shared_ptr<PipelineHolder>(pHolder);
}

void startStream(std::shared_ptr<PipelineHolder> holder) {
  std::cout << "startStream. " << holder << std::endl;
  try {
    auto pipeline = holder->pipeline;
    // Configure which streams to enable or disable for the Pipeline by creating
    // a Config
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // get Stream Profile.
    auto profileList = pipeline->getStreamProfileList(holder->sensorType);
    auto streamProfile = profileList->getProfile(OB_PROFILE_DEFAULT)
                             ->as<ob::VideoStreamProfile>();
    config->enableStream(streamProfile);
    auto frameType = mapFrameType(holder->sensorType);
    auto deviceIndex = holder->deviceIndex;
    pipeline->start(config, [frameType, deviceIndex](
                                std::shared_ptr<ob::FrameSet> frameSet) {
      auto frame = frameSet->getFrame(frameType);
      if (frame) {
        if (frameType == OB_FRAME_COLOR) {
          handleColorStream(deviceIndex, frame);
        } else if (frameType == OB_FRAME_DEPTH) {
          handleDepthStream(deviceIndex, frame);
        }
      }
    });
  } catch (ob::Error &e) {
    std::cerr << "startStream failed. " << "function:" << e.getName()
              << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage()
              << "\ntype:" << e.getExceptionType() << std::endl;
  }
}

void stopStream(std::shared_ptr<PipelineHolder> holder) {
  try {
    std::cout << "stopStream " << holder << std::endl;
    holder->pipeline->stop();
  } catch (ob::Error &e) {
    std::cerr << "stopStream failed. " << "function:" << e.getName()
              << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage()
              << "\ntype:" << e.getExceptionType() << std::endl;
  }
}

void handleColorStream(int devIndex, std::shared_ptr<ob::Frame> frame) {
  std::lock_guard<std::mutex> lock(frameMutex);
  std::cout << "Device#" << devIndex << ", color frame index=" << frame->index()
            << ", timestamp=" << frame->timeStamp()
            << ", system timestamp=" << frame->systemTimeStamp() << std::endl;

  colorFrames[devIndex] = frame;
}

void handleDepthStream(int devIndex, std::shared_ptr<ob::Frame> frame) {
  std::lock_guard<std::mutex> lock(frameMutex);
  std::cout << "Device#" << devIndex << ", depth frame index=" << frame->index()
            << ", timestamp=" << frame->timeStamp()
            << ", system timestamp=" << frame->systemTimeStamp() << std::endl;

  depthFrames[devIndex] = frame;
}

std::string readFileContent(const char *filePath) {
  std::ostringstream oss;

  long length = 0;
  long readSum = 0;
  int readSize = 0;
  char buf[512];
  bool isOpened = false;
  bool success = false;
  std::ifstream file;
  file.exceptions(std::fstream::badbit | std::fstream::failbit);
  try {
    file.open(filePath, std::fstream::in);
    isOpened = true;
    file.seekg(0, std::fstream::end);
    length = file.tellg();
    file.seekg(0, std::fstream::beg);

    while (!file.eof() && readSum < length) {
      readSize = (std::min)((long)512, length - readSum);
      file.read(buf, readSize);
      if (file.gcount() > 0) {
        oss << std::string(buf, file.gcount());
        readSum += file.gcount();
      }
    }
    success = true;
  } catch (std::fstream::failure e) {
    if ((file.rdstate() & std::fstream::failbit) != 0 &&
        (file.rdstate() & std::fstream::eofbit) != 0) {
      if (readSize > 0 && file.gcount() > 0) {
        oss << std::string(buf, file.gcount());
        readSum += file.gcount();
      }
      success = true;
    } else {
      std::string errorMsg = (nullptr != e.what() ? std::string(e.what()) : "");
      std::cerr << "open or reading file: " << std::string(filePath)
                << ", errorMsg: " << errorMsg << std::endl;
    }
  }

  if (isOpened) {
    try {
      file.close();
    } catch (std::fstream::failure e) {
      std::string errorMsg = (nullptr != e.what() ? std::string(e.what()) : "");
      std::cerr << "close file: " << std::string(filePath)
                << ", errorMsg: " << errorMsg << std::endl;
    }
  }

  return success ? oss.str() : "";
}

bool loadConfigFile() {
  auto content = readFileContent(CONFIG_FILE);
  if (content.empty()) {
    std::cerr << "load config file failed." << std::endl;
    return false;
  }

  int deviceCount = 0;

  cJSON *rootElem = cJSON_Parse(content.c_str());
  if (rootElem == nullptr) {
    const char *errMsg = cJSON_GetErrorPtr();
    std::cout << std::string(errMsg) << std::endl;
    cJSON_Delete(rootElem);
    return true;
  }

  std::shared_ptr<DeviceConfigInfo> devConfigInfo = nullptr;
  cJSON *deviceElem = nullptr;
  cJSON *devicesElem = cJSON_GetObjectItem(rootElem, "devices");
  cJSON_ArrayForEach(deviceElem, devicesElem) {
    devConfigInfo = std::make_shared<DeviceConfigInfo>();
    memset(&devConfigInfo->syncConfig, 0, sizeof(devConfigInfo->syncConfig));
    devConfigInfo->syncConfig.syncMode = OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN;

    cJSON *snElem = cJSON_GetObjectItem(deviceElem, "sn");
    if (cJSON_IsString(snElem) && snElem->valuestring != nullptr) {
      devConfigInfo->deviceSN = std::string(snElem->valuestring);
    }

    cJSON *deviceConfigElem = cJSON_GetObjectItem(deviceElem, "syncConfig");
    if (cJSON_IsObject(deviceConfigElem)) {
      cJSON *numberElem = nullptr;
      cJSON *strElem = nullptr;
      cJSON *bElem = nullptr;
      strElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "syncMode");
      if (cJSON_IsString(strElem) && strElem->valuestring != nullptr) {
        devConfigInfo->syncConfig.syncMode =
            textToOBSyncMode(strElem->valuestring);
        std::cout << "config[" << (deviceCount++)
                  << "]: SN=" << std::string(devConfigInfo->deviceSN)
                  << ", mode=" << strElem->valuestring << std::endl;
      }

      numberElem =
          cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "depthDelayUs");
      if (cJSON_IsNumber(numberElem)) {
        devConfigInfo->syncConfig.depthDelayUs = numberElem->valueint;
      }

      numberElem =
          cJSON_GetObjectItemCaseSensitive(deviceConfigElem, "colorDelayUs");
      if (cJSON_IsNumber(numberElem)) {
        devConfigInfo->syncConfig.colorDelayUs = numberElem->valueint;
      }

      numberElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem,
                                                    "trigger2ImageDelayUs");
      if (cJSON_IsNumber(numberElem)) {
        devConfigInfo->syncConfig.trigger2ImageDelayUs = numberElem->valueint;
      }

      numberElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem,
                                                    "triggerOutDelayUs");
      if (cJSON_IsNumber(numberElem)) {
        devConfigInfo->syncConfig.triggerOutDelayUs = numberElem->valueint;
      }

      bElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem,
                                               "triggerOutEnable");
      if (cJSON_IsBool(bElem)) {
        devConfigInfo->syncConfig.triggerOutEnable = (bool)bElem->valueint;
      }

      bElem = cJSON_GetObjectItemCaseSensitive(deviceConfigElem,
                                               "framesPerTrigger");
      if (cJSON_IsNumber(bElem)) {
        devConfigInfo->syncConfig.framesPerTrigger = bElem->valueint;
      }
    }

    if (OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN !=
        devConfigInfo->syncConfig.syncMode) {
      deviceConfigList.push_back(devConfigInfo);
    } else {
      std::cerr << "invalid sync mode of deviceSN: " << devConfigInfo->deviceSN
                << std::endl;
    }

    devConfigInfo = nullptr;
  }

  cJSON_Delete(rootElem);
  return true;
}

OBMultiDeviceSyncMode textToOBSyncMode(const char *text) {
  if (strcmp(text, "OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN") == 0) {
    return OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN;
  } else if (strcmp(text, "OB_MULTI_DEVICE_SYNC_MODE_STANDALONE") == 0) {
    return OB_MULTI_DEVICE_SYNC_MODE_STANDALONE;
  } else if (strcmp(text, "OB_MULTI_DEVICE_SYNC_MODE_PRIMARY") == 0) {
    return OB_MULTI_DEVICE_SYNC_MODE_PRIMARY;
  } else if (strcmp(text, "OB_MULTI_DEVICE_SYNC_MODE_SECONDARY") == 0) {
    return OB_MULTI_DEVICE_SYNC_MODE_SECONDARY;
  } else if (strcmp(text, "OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED") == 0) {
    return OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED;
  } else if (strcmp(text, "OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING") ==
             0) {
    return OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING;
  } else if (strcmp(text, "OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING") ==
             0) {
    return OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING;
  } else {
    return OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN;
  }
}

int strcmp_nocase(const char *str0, const char *str1) {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  return _strcmpi(str0, str1);
#else
  return strcasecmp(str0, str1);
#endif
}

OBFrameType mapFrameType(OBSensorType sensorType) {
  switch (sensorType) {
    case OB_SENSOR_COLOR:
      return OB_FRAME_COLOR;
    case OB_SENSOR_IR:
      return OB_FRAME_IR;
    case OB_SENSOR_IR_LEFT:
      return OB_FRAME_IR_LEFT;
    case OB_SENSOR_IR_RIGHT:
      return OB_FRAME_IR_RIGHT;
    case OB_SENSOR_DEPTH:
      return OB_FRAME_DEPTH;
    default:
      return OBFrameType::OB_FRAME_UNKNOWN;
  }
}

std::ostream &operator<<(std::ostream &os, const PipelineHolder &holder) {
  os << "deviceSN: " << holder.deviceSN << ", sensorType: ";
  if (holder.sensorType == OB_SENSOR_COLOR) {
    os << "OB_SENSOR_COLOR";
  } else if (holder.sensorType == OB_SENSOR_DEPTH) {
    os << "OB_SENSOR_DEPTH";
  } else {
    os << (int)holder.sensorType;
  }

  os << ", deviceIndex: " << holder.deviceIndex;

  return os;
}

std::ostream &operator<<(std::ostream &os,
                         std::shared_ptr<PipelineHolder> holder) {
  return os << *holder;
}
