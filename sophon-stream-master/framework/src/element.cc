#include "element.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace {
bool isEnvEnabled(const char* name) {
  const char* raw = std::getenv(name);
  if (raw == nullptr) return false;
  std::string value(raw);
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value == "1" || value == "true" || value == "yes" || value == "on";
}

bool dropOnPipeFullEnabled() {
  static const bool enabled = isEnvEnabled("SOPHON_STREAM_DROP_ON_FULL");
  return enabled;
}

using ExtraOutputConnectorMap =
    std::map<int, std::vector<std::weak_ptr<sophon_stream::framework::Connector>>>;
std::mutex gExtraOutputConnectorsMtx;
std::map<const sophon_stream::framework::Element*, ExtraOutputConnectorMap>
    gExtraOutputConnectors;
}  // namespace

namespace sophon_stream {
namespace framework {

void Element::connect(Element& srcElement, int srcElementPort,
                      Element& dstElement, int dstElementPort) {
  auto& inputConnector = dstElement.mInputConnectorMap[dstElementPort];
  if (!inputConnector) {
    inputConnector =
        std::make_shared<framework::Connector>(dstElement.getThreadNumber());
    IVS_DEBUG(
        "InputConnector initialized, mId = {0}, inputPort = {1}, dataPipeNum = "
        "{2}",
        dstElement.getId(), dstElementPort, dstElement.getThreadNumber());
  }
  dstElement.addInputPort(dstElementPort);
  srcElement.addOutputPort(srcElementPort);
  auto& outputConnector = srcElement.mOutputConnectorMap[srcElementPort];
  if (outputConnector.expired()) {
    outputConnector = inputConnector;
    return;
  }
  if (outputConnector.lock() == inputConnector) {
    return;
  }

  std::lock_guard<std::mutex> lock(gExtraOutputConnectorsMtx);
  auto& extraConnectors = gExtraOutputConnectors[&srcElement][srcElementPort];
  for (const auto& extraConnector : extraConnectors) {
    if (extraConnector.lock() == inputConnector) {
      return;
    }
  }
  extraConnectors.push_back(inputConnector);
}

Element::Element()
    : mId(-1),
      mDeviceId(-1),
      mThreadNumber(1),
      mThreadStatus(ThreadStatus::STOP) {}

Element::~Element() {
  std::lock_guard<std::mutex> lock(gExtraOutputConnectorsMtx);
  gExtraOutputConnectors.erase(this);
}

common::ErrorCode Element::init(const std::string& json) {
  IVS_INFO("Init start, json: {0}", json);

  common::ErrorCode errorCode = common::ErrorCode::SUCCESS;

  do {
    auto configure = nlohmann::json::parse(json, nullptr, false);
    if (!configure.is_object()) {
      IVS_ERROR("Parse json fail or json is not object, json: {0}", json);
      errorCode = common::ErrorCode::PARSE_CONFIGURE_FAIL;
      break;
    }

    auto idIt = configure.find(JSON_ID_FIELD);
    if (configure.end() == idIt || !idIt->is_number_integer()) {
      IVS_ERROR(
          "Can not find {0} with integer type in element json configure, json: "
          "{1}",
          JSON_ID_FIELD, json);
      errorCode = common::ErrorCode::PARSE_CONFIGURE_FAIL;
      break;
    }

    mId = idIt->get<int>();

    auto sideIt = configure.find(JSON_SIDE_FIELD);
    if (configure.end() != sideIt && sideIt->is_string()) {
      mSide = sideIt->get<std::string>();
    }

    auto sinkIt = configure.find(JSON_IS_SINK_FILED);
    if (configure.end() != sinkIt && sinkIt->is_boolean()) {
      mSinkElementFlag = sinkIt->get<bool>();
    }

    auto deviceIdIt = configure.find(JSON_DEVICE_ID_FIELD);
    if (configure.end() != deviceIdIt && deviceIdIt->is_number_integer()) {
      mDeviceId = deviceIdIt->get<int>();
    }

    auto threadNumberIt = configure.find(JSON_THREAD_NUMBER_FIELD);
    if (configure.end() != threadNumberIt &&
        threadNumberIt->is_number_integer()) {
      mThreadNumber = threadNumberIt->get<int>();
    }

    std::vector<int> inner_elements_id;
    bool is_group = false;
    auto innerIdsIt = configure.find(JSON_INNER_ELEMENTS_ID);
    if (configure.end() != innerIdsIt) {
      inner_elements_id = innerIdsIt->get<std::vector<int>>();
    }

    std::string internalConfigure;
    auto internalConfigureIt = configure.find(JSON_CONFIGURE_FIELD);
    if (configure.end() != internalConfigureIt) {
      if (getGroup())
        (*internalConfigureIt)[JSON_INNER_ELEMENTS_ID] = inner_elements_id;
      internalConfigure = internalConfigureIt->dump();
    }

    errorCode = initInternal(internalConfigure);
    if (common::ErrorCode::SUCCESS != errorCode) {
      IVS_ERROR("Init internal fail, json: {0}", internalConfigure);
      break;
    }

  } while (false);

  IVS_INFO("Init finish, json: {0}", json);
  return errorCode;
}

common::ErrorCode Element::start() {
  IVS_INFO("Start element thread start, element id: {0:d}", mId);

  if (ThreadStatus::STOP != mThreadStatus) {
    IVS_ERROR("Can not start, current thread status is not stop");
    return common::ErrorCode::THREAD_STATUS_ERROR;
  }

  mThreadStatus = ThreadStatus::RUN;

  mThreads.reserve(mThreadNumber);
  for (int i = 0; i < mThreadNumber; ++i) {
    mThreads.push_back(
        std::make_shared<std::thread>(std::bind(&Element::run, this, i)));
  }

  IVS_INFO("Start element thread finish, element id: {0:d}", mId);
  return common::ErrorCode::SUCCESS;
}

common::ErrorCode Element::stop() {
  IVS_INFO("Stop element thread start, element id: {0:d}", mId);

  if (ThreadStatus::STOP == mThreadStatus) {
    IVS_ERROR("Can not stop, current thread status is stop");
    return common::ErrorCode::THREAD_STATUS_ERROR;
  }

  mThreadStatus = ThreadStatus::STOP;

  for (auto thread : mThreads) {
    thread->join();
  }
  mThreads.clear();

  IVS_INFO("Stop element thread finish, element id: {0:d}", mId);
  return common::ErrorCode::SUCCESS;
}

common::ErrorCode Element::pause() {
  IVS_INFO("Pause element thread start, element id: {0:d}", mId);

  if (ThreadStatus::RUN != mThreadStatus) {
    IVS_ERROR("Can not pause, current thread status is not run");
    return common::ErrorCode::THREAD_STATUS_ERROR;
  }

  mThreadStatus = ThreadStatus::PAUSE;

  IVS_INFO("Pause element thread finish, element id: {0:d}", mId);
  return common::ErrorCode::SUCCESS;
}

common::ErrorCode Element::resume() {
  IVS_INFO("Resume element thread start, element id: {0:d}", mId);

  if (ThreadStatus::PAUSE != mThreadStatus) {
    IVS_ERROR("Can not resume, current thread status is not pause");
    return common::ErrorCode::THREAD_STATUS_ERROR;
  }

  mThreadStatus = ThreadStatus::RUN;

  IVS_INFO("Resume element thread finish, element id: {0:d}", mId);
  return common::ErrorCode::SUCCESS;
}

void Element::run(int dataPipeId) {
  onStart();
  prctl(PR_SET_NAME, std::to_string(mId).c_str());
  while (ThreadStatus::RUN == mThreadStatus) {
    doWork(dataPipeId);
    std::this_thread::yield();
  }
  onStop();
}

common::ErrorCode Element::pushInputData(int inputPort, int dataPipeId,
                                         std::shared_ptr<void> data) {
  IVS_DEBUG("push data, element id: {0:d}, input port: {1:d}, data: {2:p}", mId,
            inputPort, data.get());

  auto& inputConnector = mInputConnectorMap[inputPort];
  if (!inputConnector) {
    inputConnector = std::make_shared<framework::Connector>(mThreadNumber);
    IVS_DEBUG(
        "InputConnector initialized, mId = {0}, inputPort = {1}, dataPipeNum = "
        "{2}",
        mId, inputPort, mThreadNumber);
  }
  while (mInputConnectorMap[inputPort]->pushData(dataPipeId, data) !=
         common::ErrorCode::SUCCESS) {
    listenThreadPtr->report_status(common::ErrorCode::DECODE_CHANNEL_PIPE_FULL);
    IVS_DEBUG("Input DataPipe is full, now sleeping...");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return common::ErrorCode::SUCCESS;
}

std::shared_ptr<void> Element::popInputData(int inputPort, int dataPipeId) {
  if (mInputConnectorMap[inputPort] == nullptr)
    mInputConnectorMap[inputPort] =
        std::make_shared<framework::Connector>(mThreadNumber);
  return mInputConnectorMap[inputPort]->popData(dataPipeId);
}

void Element::setSinkHandler(int outputPort, SinkHandler dataHandler) {
  IVS_INFO("Set data handler, element id: {0:d}, output port: {1:d}", mId,
           outputPort);
  if (mSinkElementFlag) mSinkHandlerMap[outputPort] = dataHandler;
}

common::ErrorCode Element::pushOutputData(int outputPort, int dataPipeId,
                                          std::shared_ptr<void> data) {
  IVS_DEBUG("send data, element id: {0:d}, output port: {1:d}, data:{2:p}", mId,
            outputPort, data.get());
  if (mSinkElementFlag) {
    auto handlerIt = mSinkHandlerMap.find(outputPort);
    if (mSinkHandlerMap.end() != handlerIt) {
      auto dataHandler = handlerIt->second;
      if (dataHandler) {
        dataHandler(data);
        return common::ErrorCode::SUCCESS;
      }
    }
  }
  std::vector<std::weak_ptr<framework::Connector>> connectors;
  auto pushToConnector =
      [&](const std::weak_ptr<framework::Connector>& connectorWeak,
          common::ErrorCode& finalCode, bool& hasLiveConnector) {
        auto outputConnector = connectorWeak.lock();
        if (!outputConnector) {
          return;
        }
        hasLiveConnector = true;
        int connectorCapacity = std::max(1, outputConnector->getCapacity());
        int connectorDataPipeId = dataPipeId % connectorCapacity;
        while (outputConnector->pushData(connectorDataPipeId, data) !=
               common::ErrorCode::SUCCESS &&
               mThreadStatus != ThreadStatus::STOP) {
          listenThreadPtr->report_status(common::ErrorCode::DATA_PIPE_FULL);
          IVS_DEBUG(
              "DataPipe is full, now sleeping. ElementID is {0}, outputPort is {1}, "
              "dataPipeId is {2}",
              mId, outputPort, connectorDataPipeId);
          if (dropOnPipeFullEnabled()) {
            IVS_DEBUG(
                "DataPipe full, dropping frame in low-latency mode. ElementID is {0}, "
                "outputPort is {1}, dataPipeId is {2}",
                mId, outputPort, connectorDataPipeId);
            finalCode = common::ErrorCode::DATA_PIPE_FULL;
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      };

  auto outputConnectorIt = mOutputConnectorMap.find(outputPort);
  if (outputConnectorIt != mOutputConnectorMap.end()) {
    connectors.push_back(outputConnectorIt->second);
  }
  {
    std::lock_guard<std::mutex> lock(gExtraOutputConnectorsMtx);
    auto extraElementIt = gExtraOutputConnectors.find(this);
    if (extraElementIt != gExtraOutputConnectors.end()) {
      auto extraPortIt = extraElementIt->second.find(outputPort);
      if (extraPortIt != extraElementIt->second.end()) {
        connectors.insert(connectors.end(), extraPortIt->second.begin(),
                          extraPortIt->second.end());
      }
    }
  }
  common::ErrorCode finalCode = common::ErrorCode::SUCCESS;
  bool hasLiveConnector = false;
  for (const auto& connector : connectors) {
    pushToConnector(connector, finalCode, hasLiveConnector);
  }
  if (!hasLiveConnector) {
    IVS_ERROR(
        "Output connector not found, element id: {0}, output port: {1}, "
        "dataPipeId: {2}",
        mId, outputPort, dataPipeId);
    return common::ErrorCode::NO_SUCH_WORKER_PORT;
  }
  return finalCode;
}

int Element::getOutputConnectorCapacity(int outputPort) {
  int capacity = 0;
  auto outputConnectorIt = mOutputConnectorMap.find(outputPort);
  if (outputConnectorIt != mOutputConnectorMap.end()) {
    auto outputConnector = outputConnectorIt->second.lock();
    if (outputConnector) {
      capacity = std::max(capacity, outputConnector->getCapacity());
    }
  }
  {
    std::lock_guard<std::mutex> lock(gExtraOutputConnectorsMtx);
    auto extraElementIt = gExtraOutputConnectors.find(this);
    if (extraElementIt != gExtraOutputConnectors.end()) {
      auto extraPortIt = extraElementIt->second.find(outputPort);
      if (extraPortIt != extraElementIt->second.end()) {
        for (const auto& outputConnectorWeak : extraPortIt->second) {
          auto outputConnector = outputConnectorWeak.lock();
          if (!outputConnector) {
            continue;
          }
          capacity = std::max(capacity, outputConnector->getCapacity());
        }
      }
    }
  }
  if (capacity <= 0) {
    IVS_WARN("Output connector capacity fallback, element id: {0}, output port: {1}",
             mId, outputPort);
    return std::max(1, mThreadNumber);
  }
  return capacity;
}

int Element::getInputConnectorCapacity(int inputPort) {
  auto inputConnector = mInputConnectorMap[inputPort];
  if (!inputConnector) {
    IVS_WARN("Input connector capacity fallback, element id: {0}, input port: {1}",
             mId, inputPort);
    return std::max(1, mThreadNumber);
  }
  return inputConnector->getCapacity();
}

void Element::addInputPort(int port) {
  if (std::find(mInputPorts.begin(), mInputPorts.end(), port) ==
      mInputPorts.end()) {
    mInputPorts.push_back(port);
  }
}
void Element::addOutputPort(int port) {
  if (std::find(mOutputPorts.begin(), mOutputPorts.end(), port) ==
      mOutputPorts.end()) {
    mOutputPorts.push_back(port);
  }
}

std::vector<int> Element::getInputPorts() { return mInputPorts; }
std::vector<int> Element::getOutputPorts() { return mOutputPorts; };

}  // namespace framework
}  // namespace sophon_stream
