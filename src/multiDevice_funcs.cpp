
#include "../include/multiDevice_funcs.hpp"
#include <iostream>

std::mutex frameMutex; // Ensures thread-safe access to shared frame data.
std::map<uint8_t, std::shared_ptr<ob::Frame>> colorFrames; // Stores color frames from each device.
std::map<uint8_t, std::shared_ptr<ob::Frame>> depthFrames; // Stores depth frames from each device.

std::vector<std::shared_ptr<ob::Device>> streamDevList;
std::vector<std::shared_ptr<ob::Device>> configDevList;
std::vector<std::shared_ptr<DeviceConfigInfo>> deviceConfigList;


std::condition_variable waitRebootCompleteCondition;
std::mutex rebootingDevInfoListMutex;
std::vector<std::shared_ptr<ob::DeviceInfo>> rebootingDevInfoList;

ob::Context context;

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
namespace InputUtils {

int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    // Save terminal settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

char getch() {
    struct termios oldt, newt;
    char ch;

    // Save terminal settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    ch = getchar();

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}
}

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



// Convert a std::shared_ptr<ob::Frame> to a std::vector<OBColorPoint>
std::vector<OBColorPoint> frameToVector(const std::shared_ptr<ob::Frame> &frame) {
    std::vector<OBColorPoint> points;
    if (!frame) {
        std::cerr << "Invalid frame provided!" << std::endl;
        return points;
    }

    int pointsSize = frame->dataSize() / sizeof(OBColorPoint);
    std::cout << "Total Points in Point Cloud: " << pointsSize << std::endl;

    if (pointsSize <= 0) {
        std::cerr << "No points found in the frame!" << std::endl;
        return points;
    }

    OBColorPoint *rawPoints = static_cast<OBColorPoint *>(frame->data());
    points.assign(rawPoints, rawPoints + pointsSize);

    return points;
}

// Save a vector of colored points to a PLY file
void saveRGBPointsToPly(const std::vector<OBColorPoint> &points, std::string fileName) {
    std::cout << "Saving " << points.size() << " points to " << fileName << std::endl;

    if (points.empty()) {
        std::cerr << "Error: No valid points to save!" << std::endl;
        return;
    }

    std::cout << "First 5 Points:\n";
    for (int i = 0; i < std::min(5, (int)points.size()); i++) {
        std::cout << "  (" << points[i].x << ", " << points[i].y << ", " << points[i].z 
                  << ") Color: (" << (int)points[i].r << ", " << (int)points[i].g << ", " << (int)points[i].b << ")\n";
    }

    std::cout << "open file: " << fileName << std::endl;

    FILE *fp = fopen(fileName.c_str(), "wb+");
    if (!fp) {
        std::cerr << "Failed to open file: " << fileName << std::endl;
        return;
    }

    // Write PLY header
    fprintf(fp, "ply\nformat ascii 1.0\nelement vertex %lu\n", points.size());
    fprintf(fp, "property float x\nproperty float y\nproperty float z\n");
    fprintf(fp, "property uchar red\nproperty uchar green\nproperty uchar blue\n");
    fprintf(fp, "end_header\n");

    std::cout << "Writing points to file..." << std::endl;
    
    if(points.empty()) {
        std::cerr << "No points to write!" << std::endl;
        fclose(fp);
        return;
    }

    // Just for debugging:
      std::cout << "points.size() = " << points.size() << std::endl;

    for(const auto &point : points) {
        fprintf(fp, "%.3f %.3f %.3f %d %d %d\n",
                point.x, point.y, point.z,
                (int)point.r, (int)point.g, (int)point.b);
    }

    std::cout << "Closing file..." << std::endl;

    fclose(fp);
    std::cout << "File saved successfully: " << fileName << std::endl;
}

// Utility function to find the highest resolution that meets your desired specs
std::shared_ptr<ob::StreamProfile> findHighResDepthProfile(
    std::shared_ptr<ob::StreamProfileList> depthProfiles, 
    int desiredFPS = 15
) {
    std::shared_ptr<ob::StreamProfile> bestProfile = nullptr;
    int bestResolution = 0; // Keep track of area = width * height

    for (int i = 0; i < depthProfiles->count(); i++) {
        auto sp = depthProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
        if (!sp) continue;

        if (sp->fps() == desiredFPS || desiredFPS == OB_FPS_ANY) {
            int resolution = sp->width() * sp->height();
            if (resolution > bestResolution) {
                bestResolution = resolution;
                bestProfile = sp;
            }
        }
    }
    return bestProfile;
}

std::shared_ptr<ob::StreamProfileList> configureDepthStream(
    ob::Pipeline &pipeline, 
    std::shared_ptr<ob::VideoStreamProfile> colorProfile, 
    OBAlignMode &alignMode,
    std::shared_ptr<ob::Config> config
) {
    std::shared_ptr<ob::StreamProfileList> depthProfileList;

    try {
        // Try hardware-based D2C alignment if a color profile is available
        if (colorProfile) {
            depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
            if (depthProfileList && depthProfileList->count() > 0) {
                alignMode = ALIGN_D2C_HW_MODE;
                std::cout << "Hardware-based D2C alignment enabled." << std::endl;
            } else {
                // Try software-based D2C alignment if hardware-based alignment fails
                depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
                if (depthProfileList && depthProfileList->count() > 0) {
                    alignMode = ALIGN_D2C_SW_MODE;
                    std::cout << "Software-based D2C alignment enabled." << std::endl;
                } else {
                    std::cerr << "No compatible D2C alignment profiles found. Disabling alignment." << std::endl;
                }
            }
        }

        // If no D2C alignment is needed or supported, fallback to default depth profiles
        if (!depthProfileList || depthProfileList->count() == 0) {
            depthProfileList = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
            alignMode = ALIGN_DISABLE;
            std::cout << "No D2C alignment profiles found. Disabling alignment." << std::endl;
        }

        // Configure depth stream if profiles are available
        if (depthProfileList && depthProfileList->count() > 0) {
            std::shared_ptr<ob::StreamProfile> depthProfile;
            try {
                // Attempt to match the frame rate with the color profile
                if (colorProfile) {
                    std::cout << "Matching depth frame rate with color profile..." << std::endl;
                    //depthProfile = depthProfileList->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, colorProfile->fps());
                    // Configure the depth stream explicitly

                    int choice = 0
                    ;                    

                    OBFormat f = OB_FORMAT_Y16;
                    if(choice == 0){
                      depthProfile = depthProfileList->getVideoStreamProfile(640, 576, f, 30);
                    }else if (choice == 1)
                    {
                      depthProfile = depthProfileList->getVideoStreamProfile(512, 512, f, 30);
                    }else if (choice == 2)
                    {
                      depthProfile = depthProfileList->getVideoStreamProfile(320, 288, f, 30);
                    }else if (choice == 3)
                    {
                      depthProfile = depthProfileList->getVideoStreamProfile(1024, 1024, f, 30);
                    }
                    
                }
            } catch (...) {
                // Fallback to default profile if no matching frame rate is found
                depthProfile = nullptr;
            }

            if (!depthProfile) {
                depthProfile = depthProfileList->getProfile(OB_PROFILE_DEFAULT);
                std::cout << "Using default depth profile." << std::endl;
            }
            

            if (!depthProfile) {
                std::cerr << "Unable to find the requested depth stream profile: 640x576, 30 fps, Y16 format!" << std::endl;
                exit(EXIT_FAILURE);
            }
            std::cout << "config->enableStream" << std::endl;
            config->enableStream(depthProfile);
        } else {
            std::cerr << "No depth profiles available for the current device!" << std::endl;
        }

    } catch (const ob::Error &e) {
        std::cerr << "Error configuring depth stream: " << e.getMessage() << std::endl;
        alignMode = ALIGN_DISABLE;
        return nullptr; // Explicitly return nullptr on failure
    }

    alignMode = ALIGN_D2C_SW_MODE;
    config->setAlignMode(alignMode);

    if (depthProfileList && depthProfileList->count() > 0) {
        std::cout << "Depth stream successfully configured." << std::endl;
    } else {
        std::cerr << "Failed to configure depth stream." << std::endl;
    }

    return depthProfileList;
}

// Function to configure and enable the color stream
std::shared_ptr<ob::VideoStreamProfile> configureColorStream(ob::Pipeline &pipeline, std::shared_ptr<ob::Config> config) {
  
    std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
    try {
        // Get all stream profiles of the color camera
        auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
        if (colorProfiles) {
            // Select the default profile
            // auto profile = colorProfiles->getProfile(OB_PROFILE_DEFAULT);
            // colorProfile = profile->as<ob::VideoStreamProfile>();

            OBFormat f = OB_FORMAT_RGB888;


            colorProfile = colorProfiles->getVideoStreamProfile(1920, 1080, f, 30); // nv12 bgra

            // 3840, 2160
            // 2560, 1440
            // 1920, 1080
            // 1280,  960
            // 1280,  720

            config->enableStream(colorProfile);
            std::cout << "Color stream enabled successfully." << std::endl;
        } else {
            std::cerr << "No color profiles available for the current device." << std::endl;
        }
    } catch (const ob::Error &e) {
        config->setAlignMode(ALIGN_DISABLE); // Disable alignment if color stream is unavailable
        std::cerr << "Failed to enable color stream: " << e.getMessage() << std::endl;
    }
    return colorProfile;
}


int testMultiDeviceSync() try {
    streamDevList.clear();
    auto devList = context.queryDeviceList();
    int devCount = devList->deviceCount();


    std::map<uint8_t, std::shared_ptr<ob::Frame>> pointCloudFrames;
    std::vector<ob::PointCloudFilter> pointCloudFilters;

    std::vector<std::shared_ptr<PipelineHolder>> pipelineHolderList(devCount);

    
    for (int i = 0; i < devCount; i++) {
        streamDevList.push_back(devList->getDevice(i));
    }

    if (streamDevList.empty()) {
        std::cerr << "Device list is empty. Please check device connection state." << std::endl;
        return -1;
    }

    // Configure devices and prepare pipelines
    int deviceIndex = 0;
    for (auto &dev : streamDevList) {
        if (deviceIndex >= MAX_DEVICE_COUNT) break;

        // Set standalone mode
        auto config = dev->getMultiDeviceSyncConfig();
        config.syncMode = OB_MULTI_DEVICE_SYNC_MODE_STANDALONE;
        dev->setMultiDeviceSyncConfig(config);
        std::cout << "Configured device " << deviceIndex << " to standalone mode." << std::endl;

        // Configure depth range
        try {
            // Set minimum depth to 100mm (10cm)
            dev->setIntProperty(OB_PROP_MIN_DEPTH_INT, 100);
            std::cout << "Set minimum depth to: " << dev->getIntProperty(OB_PROP_MIN_DEPTH_INT) << "mm" << std::endl;

            // Set maximum depth to 10000mm (10m)
            dev->setIntProperty(OB_PROP_MAX_DEPTH_INT, 10000);
            std::cout << "Set maximum depth to: " << dev->getIntProperty(OB_PROP_MAX_DEPTH_INT) << "mm" << std::endl;

            // Optional: Print current depth range
            int minDepth = dev->getIntProperty(OB_PROP_MIN_DEPTH_INT);
            int maxDepth = dev->getIntProperty(OB_PROP_MAX_DEPTH_INT);
            std::cout << "Device " << deviceIndex << " depth range: " 
                     << minDepth << "mm to " << maxDepth << "mm" << std::endl;
        }
        catch (ob::Error &e) {
            std::cerr << "Failed to set depth range for device " << deviceIndex 
                     << ": " << e.getMessage() << std::endl;
        }
        
        auto pipelineHolder = std::make_shared<PipelineHolder>();
        pipelineHolder->pipeline = std::make_shared<ob::Pipeline>(dev);
        pipelineHolder->deviceIndex = deviceIndex;
        pipelineHolder->deviceSN = std::string(dev->getDeviceInfo()->serialNumber());

        // Configure pipeline
        std::shared_ptr<ob::Config> pipelineConfig = std::make_shared<ob::Config>();

            // Turn on D2C alignment, which needs to be turned on when generating RGBD point clouds
        std::shared_ptr<ob::VideoStreamProfile> colorProfile = configureColorStream(*pipelineHolder->pipeline, pipelineConfig);

        // Get all stream profiles of the depth camera, including stream resolution, frame rate, and frame format
        std::shared_ptr<ob::StreamProfileList> depthProfileList;
        OBAlignMode                            alignMode = ALIGN_DISABLE; // ALIGN_D2C_SW_MODE, ALIGN_DISABLE
        
        
        // Configure depth stream with or without color alignment
        depthProfileList = configureDepthStream(*pipelineHolder->pipeline, colorProfile, alignMode, pipelineConfig);

        // Initialize point cloud filter
        ob::PointCloudFilter pointCloudFilter;
        //dev->getIntProperty

        auto cameraParam = dev->getCalibrationCameraParamList()->getCameraParam(0);

        // print cameraParam
        std::cout << "Device: " << deviceIndex << std::endl;
        std::cout << "CameraParam: " << std::endl;
        std::cout << "Depth Intrinsic: " << cameraParam.depthIntrinsic.width << "x" << cameraParam.depthIntrinsic.height << std::endl;
        std::cout << "RGB Intrinsic: " << cameraParam.rgbIntrinsic.width << "x" << cameraParam.rgbIntrinsic.height << std::endl;
        std::cout << "Depth Distortion: " << cameraParam.depthDistortion.k1 << " " << cameraParam.depthDistortion.k2 << " " << cameraParam.depthDistortion.k3 << " " << cameraParam.depthDistortion.k4 << " " << cameraParam.depthDistortion.k5 << std::endl;
        std::cout << "RGB Distortion: " << cameraParam.rgbDistortion.k1 << " " << cameraParam.rgbDistortion.k2 << " " << cameraParam.rgbDistortion.k3 << " " << cameraParam.rgbDistortion.k4 << " " << cameraParam.rgbDistortion.k5 << std::endl;
        std::cout << "Transform: " << cameraParam.transform.rot[0] << " " << cameraParam.transform.rot[1] << " " << cameraParam.transform.rot[2] << " " << cameraParam.transform.rot[3] << " " << cameraParam.transform.rot[4] << " " << cameraParam.transform.rot[5] << " " << cameraParam.transform.rot[6] << " " << cameraParam.transform.rot[7] << " " << cameraParam.transform.rot[8] << " " << cameraParam.transform.trans[0] << " " << cameraParam.transform.trans[1] << " " << cameraParam.transform.trans[2] << std::endl;
        
        /*
                Device: 0
        CameraParam: 
        Depth Intrinsic: 1024x1024
        RGB Intrinsic: 640x360
        Depth Distortion: 13.9249 6.82882 0.235893 14.2438 11.5366
        RGB Distortion: 0.0764899 -0.105664 0.0430727 0 0
        Transform: 0.994121 0.00221697 0.00181275 -0.00240018 0.99412 0.108254 -0.00156209 -0.108258 0.994122 -32.6268 -0.878779 2.29152

        Device: 1
        CameraParam: 
        Depth Intrinsic: 1024x1024
        RGB Intrinsic: 640x360
        Depth Distortion: 26.1786 13.5443 0.496677 26.4655 22.4464
        RGB Distortion: 0.0737156 -0.10446 0.043283 0 0
        Transform: 0.993581 0.00790242 -0.000680822 -0.00777466 0.993549 0.113136 0.00157048 -0.113127 0.993579 -32.6164 -0.868447 2.46261
        */

        pointCloudFilter.setCameraParam(cameraParam);
        pointCloudFilter.setCreatePointFormat(OB_FORMAT_RGB_POINT);
        pointCloudFilter.setCoordinateSystem(OB_RIGHT_HAND_COORDINATE_SYSTEM);
        pointCloudFilters.push_back(pointCloudFilter);

        // Start pipeline with frame callback
        pipelineHolder->pipeline->start(pipelineConfig, [deviceIndex, &pointCloudFrames, &pointCloudFilters](std::shared_ptr<ob::FrameSet> frameSet) {
            if (!frameSet) return;

            auto colorFrame = frameSet->colorFrame();
            auto depthFrame = frameSet->depthFrame();


            if (colorFrame && depthFrame) {
                std::lock_guard<std::mutex> lock(frameMutex);

                // Check frame validity
                if (colorFrame->dataSize() == 0 || depthFrame->dataSize() == 0) {
                    std::cerr << "Empty frame detected!" << std::endl;
                    return;
                }
                
                colorFrames[deviceIndex] = colorFrame;
                depthFrames[deviceIndex] = depthFrame;

                pointCloudFilters[deviceIndex].setPositionDataScaled(depthFrame->getValueScale());
                
                auto pointCloud = pointCloudFilters[deviceIndex].process(frameSet);
                if (pointCloud && pointCloud->dataSize() > 0) {
                    size_t numPoints = pointCloud->dataSize() / sizeof(OBColorPoint);
                    if (numPoints > 0) {
                        pointCloudFrames[deviceIndex] = pointCloud;
                        std::cout << "Generated point cloud with " << numPoints << " points" << std::endl;
                    }
                }

            }
        });

        pipelineHolderList.push_back(pipelineHolder);
        deviceIndex++;
    }


    while (true) {

      if(InputUtils::kbhit()) {

        int key = InputUtils::getch();
        
        if (key == 'S' || key == 's') {
            std::cout << "Synchronizing devices..." << std::endl;
            context.enableDeviceClockSync(3600000);
        }
        else if (key == 'T' || key == 't') {
            for (auto &dev : streamDevList) {
                auto syncConfig = dev->getMultiDeviceSyncConfig();
                if (syncConfig.syncMode == OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING) {
                    dev->triggerCapture();
                }
            }
        }

        if (key == 'P' || key == 'p') {  // Save point cloud when 'P' is pressed
        for (const auto& pair : pointCloudFrames) {
            if (pair.second) {
                std::vector<OBColorPoint> pointCloudFrame_points = frameToVector(pair.second);

                // Generate a unique filename using a timestamp
                std::stringstream filename;
                
                filename << "GeneratedRGBPoints_" << static_cast<int>(pair.first)  << ".ply";

                // Save point cloud
                saveRGBPointsToPly(pointCloudFrame_points, filename.str());

                std::cout << "Saved point cloud as " << filename.str() << std::endl;
            }
        }
      }

    }
      }

    // Cleanup
    for (auto &holder : pipelineHolderList) {
        holder->pipeline->stop();
    }
    
    pipelineHolderList.clear();
    streamDevList.clear();
    
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        depthFrames.clear();
        colorFrames.clear();
        pointCloudFrames.clear();
    }

    return 0;
} catch (ob::Error &e) {
    std::cerr << "Error: " << e.getMessage() << std::endl;
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

void startStream(std::shared_ptr<PipelineHolder> holder, ob::PointCloudFilter &pointCloudFilter) {
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
    // config->setAlignMode(ALIGN_D2C_SW_MODE);
    auto frameType = mapFrameType(holder->sensorType);
    auto deviceIndex = holder->deviceIndex;
    pipeline->start(config, [frameType, deviceIndex, &holder, &pointCloudFilter](
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
