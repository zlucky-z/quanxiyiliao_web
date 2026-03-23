#include "httplib.hpp"
#include "json.hpp"
#include <iostream>
#include <fstream>
#include <mutex>
#include <cstring>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <sstream>
#include <string>
#include <cstdio>
#include <getopt.h>

#include <iomanip>
#include <condition_variable>
#include <ctime>

#include <vector>
#include <iterator>
#include <algorithm>
#include <map>
#include <set>
#include <unordered_map>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <dirent.h> // 添加的这个头文件，用于目录操作相关函数及结构体的声明
#include <cerrno>
#include <signal.h>
#include <sys/stat.h> // 添加用于读取文件状态的头文件
#include <unistd.h>
using json = nlohmann::json;
using namespace httplib;

//----------------------------------------------------------------------------------------------------------------------------
std::string executeCommand(const char *command);
bool isProcessRunning(int pid);
std::vector<int> listChildProcesses(int parentPid);
void collectProcessTree(int rootPid, std::vector<int>& orderedPids, std::set<int>& visited);
void stopManagedProcess(int rootPid, const std::string& taskLabel = "");
void deleteOldestFiles(const std::string &mountPath);
void stopRecording();
std::string buildRuntimeConfigDir(const std::string& baseName);
std::string buildTaskLogPath(const std::string& taskName, int taskId);
std::string normalizeAlarmImageUrl(const std::string& rawImageUrl);
bool buildLocalAlarmImagePath(const std::string& rawImageUrl, std::string& localImageUrl, std::string& imagePath);
bool captureRtspSnapshot(const std::string& rtspUrl, const std::string& outputPath);
bool captureRtspSnapshotWithError(const std::string& rtspUrl,
                                  const std::string& outputPath,
                                  std::string& errorMessage);
std::string normalizeAlarmVideoUrl(const std::string& rawVideoUrl);
bool buildLocalAlarmVideoPath(const std::string& rawVideoUrl,
                              std::string& localVideoUrl,
                              std::string& videoPath);
bool isAlarmVideoAvailable(const std::string& rawVideoUrl);
bool captureRtspVideoClip(const std::string& rtspUrl,
                          const std::string& outputPath,
                          int durationSec);
bool captureRtspVideoClipWithError(const std::string& rtspUrl,
                                   const std::string& outputPath,
                                   int durationSec,
                                   std::string& errorMessage);
int readAlarmVideoDurationSecondsFromParams();
bool isAlarmVideoClipEnabled();
bool isRegisteredPersonTrackingAlgorithm(const std::string& algorithm);
bool isRegionIntrusionAlgorithm(const std::string& algorithm);
json buildRegisteredPersonTrackingAreaPoints(int frameWidth = 1920, int frameHeight = 1080);
json buildTaskRegionFilterAreas(const json& regionConfigs,
                                int targetFrameWidth,
                                int targetFrameHeight,
                                bool fallbackToLegacyPolygon);
json buildTaskRegionOverlayRegions(const json& regionConfigs,
                                   int targetFrameWidth,
                                   int targetFrameHeight,
                                   bool fallbackToLegacyPolygon);
bool loadJsonFile(const std::string& path, json& value, std::string& errorMessage);
int parseJsonInt(const json& value, int fallbackValue = 0);
bool probeStreamFrameSize(const std::string& streamUrl, int& width, int& height);
bool updatePolygonInFilterConfigFile(const std::string& configPath,
                                     const json& filterAreas,
                                     int channelId);
bool readAlarmRecordById(int alarmId, json& alarmOut);
bool updateAlarmRecordById(int alarmId, const json& patch, json* updatedAlarmOut);
void processAlarmVideoAndReportAsync(int alarmId, int taskId, const std::string& alarmType);
struct RoiRect
{
    int left;
    int top;
    int width;
    int height;
};
bool computeRoiRectFromAreaPoints(const json& areas,
                                  int frameWidth,
                                  int frameHeight,
                                  RoiRect& roi);
void ensureYolov8RegionFilterGraph(json& graph,
                                   const std::string& runtimeFilterConfigPath);
bool shouldCountRegisteredPersonDetectionInCurrentPolygon(const json& detectedObject);
std::unordered_map<std::string, std::string> buildFaceLabelToNameMap(const json& labelMap);
long long currentTimeMillis();
long long normalizeFrameTimestampToMillis(long long rawTimestamp, long long fallbackTimestampMs);
extern bool isFfmpegRunning;
extern bool isFfmpegRunning2;
// 结构体用于存储TF卡信息
struct TFCardInfo
{
    std::string mountPath;
    std::string totalMemory;
    std::string usedMemory;
    std::string freeMemory;
    std::string usePercent;
};
// 在全局作用域声明用于记录开始录制时间的变量
std::chrono::time_point<std::chrono::system_clock> startRecordingTime;
// 在全局作用域定义线程结束标志位，初始化为false
std::atomic<bool> stopTimerThread(false);
// 在全局作用域定义VIDEO_RETENTION_MINUTES变量
const int VIDEO_RETENTION_MINUTES = 10;
// 全局变量用于存储开机自启的默认视频流地址和保存地址（初始化为从JSON文件读取的值）
std::string defaultRtspStreamUrl;
std::string defaultSaveLocation;
std::string defaultRtspStreamUrl2;
std::string defaultSaveLocation2;
// 全局变量用于存储开机自启的默认分段时间（单位：秒）
int defaultSegmentTime = 600;
// 用于标记是否是开机自启的情况，初始化为false
bool isAutoStart = false;
const std::string FIRMWARE_TARGET_PATH = "/data/lintech/celectronicfence/server";
const std::string RECORDING_STATUS_FILE = "/data/lintech/celectronicfence/recording_status.json";
const std::string RECORDING_PID_FILE_1 = "/tmp/recording1.pid";
const std::string RECORDING_PID_FILE_2 = "/tmp/recording2.pid";
const std::string RECORDING_LOG_FILE_1 = "/tmp/recording_ffmpeg1.log";
const std::string RECORDING_LOG_FILE_2 = "/tmp/recording_ffmpeg2.log";
std::mutex recordingStateMutex;
std::string recordingLastError;
std::string activeRecordingRtspUrl1;
std::string activeRecordingRtspUrl2;
std::string activeRecordingSaveLocation1;
std::string activeRecordingSaveLocation2;
std::mutex firmwareUpdateMutex;
std::mutex alarmFileMutex;
//----------------------------------------------------------------------------------------------------------------------------
// 获取TF卡信息的函数（修改为了直接提取df -h命令输出中的已用和剩余内存信息）
TFCardInfo getTFCardInfo()
{
    TFCardInfo tfCardInfo;
    tfCardInfo.mountPath = "/mnt/tfcard"; // 设置TF卡挂载路径

    // 基于设置好的挂载路径构建df -h命令，去获取对应磁盘信息
    std::string dfCommand = "df -h " + tfCardInfo.mountPath;
    std::string dfOutput = executeCommand(dfCommand.c_str());

    // 解析df -h命令输出内容，获取已用和剩余内存信息
    std::istringstream iss(dfOutput);
    std::string line;
    std::getline(iss, line);
    std::getline(iss, line);
    std::istringstream lineStream(line);
    std::string filesystem, size, used, free, usePercent, mountPoint;
    lineStream >> filesystem >> size >> used >> free >> usePercent >> mountPoint;
    tfCardInfo.totalMemory = size;
    tfCardInfo.usedMemory = used;
    tfCardInfo.freeMemory = free;
    tfCardInfo.usePercent = usePercent;

    return tfCardInfo;
}
//----------------------------------------------------------------------------------------------------------------------------
bool waitForMountPoint(const std::string &mountPath, int maxWaitSeconds = 45)
{
    int waitSeconds = 0;
    struct stat sb;
    while (stat(mountPath.c_str(), &sb) != 0)
    {
        if (waitSeconds >= maxWaitSeconds)
        {
            std::cerr << "等待TF卡挂载超时,挂载点 " << mountPath << " 不可用。" << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        waitSeconds++;
    }
    return true;
}
//----------------------------------------------------------------------------------------------------------------------------
// 执行系统命令并获取输出结果的函数（优化缓冲区大小版本）
std::string executeCommand(const char *command)
{
    char buffer[1024]; // 增大缓冲区大小
    std::string result = "";
    FILE *pipe = popen(command, "r");
    if (!pipe)
    {
        std::cerr << "执行命令失败: " << command << ", 错误码: " << errno << std::endl;
        return result;
    }
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
    {
        result += buffer;
    }
    pclose(pipe);
    return result;
}

std::string detectSophonStreamRoot()
{
    const std::vector<std::string> candidates = {
        "/data/sophon-stream-master",
        "/data/sophon-stream",
        "/data/mediacal/sophon-stream-master",
        "/data/mediacal/sophon-stream"
    };

    struct stat st;
    for (size_t i = 0; i < candidates.size(); ++i)
    {
        if (stat(candidates[i].c_str(), &st) == 0 && S_ISDIR(st.st_mode))
        {
            return candidates[i];
        }
    }

    std::cerr << "[路径告警] 未检测到 sophon-stream 目录，默认回退到 /data/sophon-stream-master" << std::endl;
    return "/data/sophon-stream-master";
}

const std::string SOPHON_STREAM_ROOT = detectSophonStreamRoot();
const std::string SOPHON_STREAM_SAMPLES_BUILD_DIR = SOPHON_STREAM_ROOT + "/samples/build";
const std::string SOPHON_STREAM_BUILD_LIB_DIR = SOPHON_STREAM_ROOT + "/build/lib";
const std::string SOPHON_STREAM_YOLO_DIR = SOPHON_STREAM_ROOT + "/samples/yolov8";
const std::string SOPHON_STREAM_YOLO_CONFIG_DIR = SOPHON_STREAM_YOLO_DIR + "/config";
const std::string SOPHON_STREAM_YOLO_DATA_DIR = SOPHON_STREAM_YOLO_DIR + "/data";

// 人脸库相关路径
const std::string FACE_PROJECT_ROOT = SOPHON_STREAM_ROOT + "/samples/retinaface_distributor_resnet_faiss_converger";
const std::string FACE_SAMPLES_BUILD_DIR = SOPHON_STREAM_SAMPLES_BUILD_DIR;
const std::string FACE_BUILD_LIB_DIR = SOPHON_STREAM_BUILD_LIB_DIR;
const std::string FACE_TRAIN_DIR = FACE_PROJECT_ROOT + "/data/images/face_data_train";
const std::string FACE_DB_DIR = FACE_PROJECT_ROOT + "/data/face_data";
const std::string FACE_DB_DATA_FILE = FACE_DB_DIR + "/faiss_db_data.txt";
const std::string FACE_INDEX_LABEL_FILE = FACE_DB_DIR + "/faiss_index_label.name";
const std::string FACE_LABEL_MAP_FILE = FACE_DB_DIR + "/face_label_map.json";
const std::string FACE_SCRIPT_DIR = FACE_PROJECT_ROOT + "/scripts";
const std::string FACE_WRITE_SCRIPT = FACE_SCRIPT_DIR + "/resnet_opencv_faiss_write.py";
const std::string FACE_BATCH_IMPORT_SCRIPT = FACE_SCRIPT_DIR + "/batch_face_zip_import.py";
const std::string FACE_REBUILD_LOG_FILE = "/tmp/face_db_rebuild.log";
const std::string FACE_RUNTIME_CONFIG_DIR = buildRuntimeConfigDir("cef_face_runtime");
const std::string REGISTERED_PERSON_TRACKING_PROJECT_ROOT =
    SOPHON_STREAM_ROOT + "/samples/registered_person_tracking";
const std::string REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR =
    buildRuntimeConfigDir("cef_registered_person_tracking_runtime");
const std::string REGISTERED_PERSON_TRACKING_HTTP_REPORT_PATH =
    "/api/registered-person-tracking/report";
const std::string YOLO_RUNTIME_CONFIG_DIR = buildRuntimeConfigDir("cef_yolov8_runtime");
const std::string TASK_LOG_DIR = buildRuntimeConfigDir("cef_task_logs");
const std::string TASK_REGION_FRAME_DIR = buildRuntimeConfigDir("cef_task_region_frames");
const std::string TASKS_JSON_PATH = "/data/lintech/celectronicfence/tasks.json";
const std::string CHANNELS_JSON_PATH = "/data/lintech/celectronicfence/channels.json";
const int TASK_STREAM_CHANNEL_BASE = 1000;

std::string buildRuntimeConfigDir(const std::string& baseName)
{
    uid_t uid = ::getuid();
    return "/tmp/" + baseName + "_uid" + std::to_string(static_cast<unsigned long long>(uid));
}

std::string buildTaskLogPath(const std::string& taskName, int taskId)
{
    return TASK_LOG_DIR + "/" + taskName + "_task_" + std::to_string(taskId) + ".log";
}

std::string trimString(const std::string& input)
{
    size_t start = 0;
    while (start < input.size() && std::isspace(static_cast<unsigned char>(input[start])))
    {
        start++;
    }
    size_t end = input.size();
    while (end > start && std::isspace(static_cast<unsigned char>(input[end - 1])))
    {
        end--;
    }
    return input.substr(start, end - start);
}

bool pathExists(const std::string& path)
{
    struct stat st;
    return stat(path.c_str(), &st) == 0;
}

bool isDirectoryPath(const std::string& path)
{
    struct stat st;
    if (stat(path.c_str(), &st) != 0)
    {
        return false;
    }
    return S_ISDIR(st.st_mode);
}

int clampInt(int value, int minValue, int maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }
    if (value > maxValue)
    {
        return maxValue;
    }
    return value;
}

double clampDouble(double value, double minValue, double maxValue)
{
    if (value < minValue)
    {
        return minValue;
    }
    if (value > maxValue)
    {
        return maxValue;
    }
    return value;
}

std::string shellQuote(const std::string& input)
{
    std::string quoted = "'";
    for (size_t i = 0; i < input.size(); ++i)
    {
        if (input[i] == '\'')
        {
            quoted += "'\"'\"'";
        }
        else
        {
            quoted += input[i];
        }
    }
    quoted += "'";
    return quoted;
}

bool isProcessRunning(int pid)
{
    if (pid <= 0)
    {
        return false;
    }
    if (::kill(pid, 0) == 0)
    {
        return true;
    }
    return errno == EPERM;
}

std::vector<int> listChildProcesses(int parentPid)
{
    std::vector<int> childPids;
    if (parentPid <= 0)
    {
        return childPids;
    }

    std::string command = "pgrep -P " + std::to_string(parentPid) + " 2>/dev/null";
    std::string output = executeCommand(command.c_str());
    std::istringstream iss(output);
    std::string line;
    while (std::getline(iss, line))
    {
        line = trimString(line);
        if (line.empty())
        {
            continue;
        }

        try
        {
            int childPid = std::stoi(line);
            if (childPid > 0)
            {
                childPids.push_back(childPid);
            }
        }
        catch (...)
        {
        }
    }
    return childPids;
}

void collectProcessTree(int rootPid, std::vector<int>& orderedPids, std::set<int>& visited)
{
    if (rootPid <= 0 || !visited.insert(rootPid).second)
    {
        return;
    }

    std::vector<int> childPids = listChildProcesses(rootPid);
    for (size_t i = 0; i < childPids.size(); ++i)
    {
        collectProcessTree(childPids[i], orderedPids, visited);
    }

    orderedPids.push_back(rootPid);
}

void stopManagedProcess(int rootPid, const std::string& taskLabel)
{
    if (rootPid <= 0)
    {
        return;
    }

    std::vector<int> processTree;
    std::set<int> visited;
    collectProcessTree(rootPid, processTree, visited);
    if (processTree.empty())
    {
        processTree.push_back(rootPid);
    }

    if (!taskLabel.empty())
    {
        std::cout << "[停止任务] " << taskLabel << ", 进程树根PID: " << rootPid << std::endl;
    }

    for (size_t i = 0; i < processTree.size(); ++i)
    {
        int pid = processTree[i];
        if (!isProcessRunning(pid))
        {
            continue;
        }
        if (::kill(pid, SIGTERM) != 0 && errno != ESRCH)
        {
            std::cerr << "[停止任务] SIGTERM 失败, PID: " << pid << ", errno: " << errno << std::endl;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    for (size_t i = 0; i < processTree.size(); ++i)
    {
        int pid = processTree[i];
        if (!isProcessRunning(pid))
        {
            continue;
        }
        if (::kill(pid, SIGKILL) != 0 && errno != ESRCH)
        {
            std::cerr << "[停止任务] SIGKILL 失败, PID: " << pid << ", errno: " << errno << std::endl;
        }
    }
}

std::string readProcessCmdline(int pid)
{
    if (pid <= 0)
    {
        return "";
    }

    std::ifstream in(("/proc/" + std::to_string(pid) + "/cmdline").c_str(), std::ios::in | std::ios::binary);
    if (!in.is_open())
    {
        return "";
    }

    std::string raw((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    for (size_t i = 0; i < raw.size(); ++i)
    {
        if (raw[i] == '\0')
        {
            raw[i] = ' ';
        }
    }
    return trimString(raw);
}

bool readProcessStartTimeTicks(int pid, long long& startTicks)
{
    startTicks = 0;
    if (pid <= 0)
    {
        return false;
    }

    std::ifstream in(("/proc/" + std::to_string(pid) + "/stat").c_str());
    if (!in.is_open())
    {
        return false;
    }

    std::string line;
    std::getline(in, line);
    if (line.empty())
    {
        return false;
    }

    size_t rightParenPos = line.rfind(')');
    if (rightParenPos == std::string::npos || rightParenPos + 2 >= line.size())
    {
        return false;
    }

    std::string rest = line.substr(rightParenPos + 2);
    std::istringstream iss(rest);
    std::vector<std::string> fields;
    std::string token;
    while (iss >> token)
    {
        fields.push_back(token);
    }

    // /proc/<pid>/stat 第22列是starttime；在去掉pid/comm后，位于第20个token(0-based index 19)
    if (fields.size() <= 19)
    {
        return false;
    }

    try
    {
        startTicks = std::stoll(fields[19]);
        return startTicks > 0;
    }
    catch (...)
    {
        return false;
    }
}

bool doesPidMatchTaskProcess(const json& task, int pid, std::string& reason, bool checkPidStartTime = true)
{
    if (pid <= 0)
    {
        reason = "PID无效";
        return false;
    }

    if (!isProcessRunning(pid))
    {
        reason = "进程不存在";
        return false;
    }

    if (checkPidStartTime && task.contains("pidStartTime") && task["pidStartTime"].is_number_integer())
    {
        long long expectedStartTime = task["pidStartTime"].get<long long>();
        if (expectedStartTime > 0)
        {
            long long actualStartTime = 0;
            if (readProcessStartTimeTicks(pid, actualStartTime) && actualStartTime != expectedStartTime)
            {
                reason = "PID已复用(starttime不匹配)";
                return false;
            }
        }
    }

    std::string cmdline = readProcessCmdline(pid);
    if (cmdline.empty())
    {
        reason = "无法读取进程命令行";
        return false;
    }

    std::string algorithm = task.value("algorithm", "");
    bool matched = false;
    if (algorithm == "face_recognition")
    {
        matched = cmdline.find("retinaface_distributor_resnet_faiss_converger") != std::string::npos;
    }
    else if (isRegisteredPersonTrackingAlgorithm(algorithm))
    {
        matched = (cmdline.find("registered_person_tracking") != std::string::npos) ||
                  (cmdline.find("--demo_config_path") != std::string::npos &&
                   (cmdline.find("./main") != std::string::npos ||
                    cmdline.find("/sophon-stream/samples/build/main") != std::string::npos ||
                    cmdline.find("/sophon-stream-master/samples/build/main") != std::string::npos));
    }
    else if (algorithm == "打架检测")
    {
        matched = (cmdline.find("detect_fight.py") != std::string::npos) ||
                  (cmdline.find("/samples/fight/start.sh") != std::string::npos);
    }
    else if (algorithm == "跌倒检测")
    {
        matched = (cmdline.find("detect_fall.py") != std::string::npos) ||
                  (cmdline.find("/samples/fall/start.sh") != std::string::npos);
    }
    else if (algorithm == "明烟明火")
    {
        matched = (cmdline.find("detect_fire_smoke.py") != std::string::npos) ||
                  (cmdline.find("/samples/fire-smoke/start.sh") != std::string::npos);
    }
    else
    {
        matched = (cmdline.find("yolov8_demo") != std::string::npos) ||
                  (cmdline.find("--demo_config_path") != std::string::npos &&
                   (cmdline.find("./main") != std::string::npos ||
                    cmdline.find("/sophon-stream/samples/build/main") != std::string::npos ||
                    cmdline.find("/sophon-stream-master/samples/build/main") != std::string::npos));
    }

    if (!matched)
    {
        reason = "进程命令行与任务算法不匹配: " + cmdline;
        return false;
    }

    std::string configPath = task.value("configPath", "");
    if (!configPath.empty() && cmdline.find(configPath) == std::string::npos)
    {
        reason = "进程命令行未命中任务配置: " + configPath;
        return false;
    }

    std::string outputRtsp = task.value("outputRtsp", "");
    bool outputRtspShouldMatch =
        (algorithm == "打架检测" || algorithm == "跌倒检测" || algorithm == "明烟明火");
    if (outputRtspShouldMatch && !outputRtsp.empty() && cmdline.find(outputRtsp) == std::string::npos)
    {
        reason = "进程命令行未命中任务输出流: " + outputRtsp;
        return false;
    }

    reason.clear();
    return true;
}

void stopTaskProcessIfMatched(const json& task, int pid, const std::string& taskLabel)
{
    std::string reason;
    if (!doesPidMatchTaskProcess(task, pid, reason))
    {
        std::cout << "[停止任务] 跳过终止PID " << pid << "，原因: " << reason << std::endl;
        return;
    }
    stopManagedProcess(pid, taskLabel);
}

std::vector<int> listPidsByPattern(const std::string& pattern)
{
    std::vector<int> pids;
    if (pattern.empty())
    {
        return pids;
    }

    std::string command = "pgrep -f " + shellQuote(pattern) + " 2>/dev/null";
    std::string output = executeCommand(command.c_str());
    std::istringstream iss(output);
    std::string line;
    while (std::getline(iss, line))
    {
        line = trimString(line);
        if (line.empty())
        {
            continue;
        }
        try
        {
            int pid = std::stoi(line);
            if (pid > 0)
            {
                pids.push_back(pid);
            }
        }
        catch (...)
        {
        }
    }
    return pids;
}

void stopFaceTaskProcessesByTaskId(int taskId, int skipPid, const std::string& taskLabel)
{
    if (taskId <= 0)
    {
        return;
    }

    std::string pattern =
        FACE_RUNTIME_CONFIG_DIR + "/retinaface_distributor_resnet_faiss_converger_runtime_task_" +
        std::to_string(taskId) + ".json";
    std::vector<int> candidatePids = listPidsByPattern(pattern);
    if (candidatePids.empty())
    {
        return;
    }

    std::sort(candidatePids.begin(), candidatePids.end());
    candidatePids.erase(std::unique(candidatePids.begin(), candidatePids.end()), candidatePids.end());

    for (size_t i = 0; i < candidatePids.size(); ++i)
    {
        int pid = candidatePids[i];
        if (pid <= 0 || pid == skipPid || !isProcessRunning(pid))
        {
            continue;
        }
        stopManagedProcess(pid, taskLabel);
    }
}

int discoverRunningTaskPid(const json& task)
{
    std::string algorithm = task.value("algorithm", "");
    std::string pattern;

    std::string configPath = task.value("configPath", "");
    if (!configPath.empty())
    {
        pattern = configPath;
    }
    else
    {
        std::string outputRtsp = task.value("outputRtsp", "");
        if (!outputRtsp.empty())
        {
            pattern = outputRtsp;
        }
    }

    if (pattern.empty() && algorithm == "face_recognition")
    {
        pattern = "retinaface_distributor_resnet_faiss_converger/config/retinaface_distributor_resnet_faiss_converger.json";
    }
    else if (pattern.empty() && isRegisteredPersonTrackingAlgorithm(algorithm))
    {
        pattern = "registered_person_tracking_runtime_task_" + std::to_string(task.value("id", 0)) + ".json";
    }
    else if (pattern.empty() && algorithm == "打架检测")
    {
        pattern = "detect_fight.py";
    }
    else if (pattern.empty() && algorithm == "跌倒检测")
    {
        pattern = "detect_fall.py";
    }
    else if (pattern.empty() && algorithm == "明烟明火")
    {
        pattern = "detect_fire_smoke.py";
    }
    else if (pattern.empty())
    {
        pattern = "yolov8_demo";
    }

    std::vector<int> candidates = listPidsByPattern(pattern);
    if (candidates.empty())
    {
        return 0;
    }

    int channelId = task.value("channelId", 0);
    std::string channelTag = channelId > 0 ? ("--channel_id " + std::to_string(channelId)) : "";

    // 优先匹配通道参数（打架/跌倒/烟火）
    for (size_t i = 0; i < candidates.size(); ++i)
    {
        int pid = candidates[i];
        if (pid <= 0 || !isProcessRunning(pid))
        {
            continue;
        }
        std::string cmdline = readProcessCmdline(pid);
        if ((algorithm == "打架检测" || algorithm == "跌倒检测" || algorithm == "明烟明火") &&
            !channelTag.empty() &&
            cmdline.find(channelTag) == std::string::npos)
        {
            continue;
        }

        std::string reason;
        if (doesPidMatchTaskProcess(task, pid, reason, false))
        {
            return pid;
        }
    }

    // 回退：返回第一个算法匹配的存活进程
    for (size_t i = 0; i < candidates.size(); ++i)
    {
        int pid = candidates[i];
        if (pid <= 0 || !isProcessRunning(pid))
        {
            continue;
        }
        std::string reason;
        if (doesPidMatchTaskProcess(task, pid, reason, false))
        {
            return pid;
        }
    }

    return 0;
}

bool ensureDirectory(const std::string& path)
{
    if (isDirectoryPath(path))
    {
        return access(path.c_str(), W_OK | X_OK) == 0;
    }
    std::string mkdirCmd = "mkdir -p " + shellQuote(path);
    if (std::system(mkdirCmd.c_str()) != 0)
    {
        return false;
    }
    return isDirectoryPath(path) && (access(path.c_str(), W_OK | X_OK) == 0);
}

bool writeJsonFile(const std::string& path, const json& value)
{
    std::string tmpPath = path + ".tmp." + std::to_string(::getpid());
    std::ofstream out(tmpPath.c_str());
    if (!out.is_open())
    {
        return false;
    }
    out << value.dump(4);
    out.close();
    if (!out.good())
    {
        std::remove(tmpPath.c_str());
        return false;
    }

    if (std::rename(tmpPath.c_str(), path.c_str()) != 0)
    {
        std::remove(tmpPath.c_str());
        return false;
    }
    return true;
}

std::string resolveConfigPath(const std::string& baseDir, const std::string& rawPath)
{
    std::string trimmed = trimString(rawPath);
    if (trimmed.empty())
    {
        return "";
    }
    if (!trimmed.empty() && trimmed[0] == '/')
    {
        return trimmed;
    }

    std::string combined = baseDir;
    if (!combined.empty() && combined.back() != '/')
    {
        combined += "/";
    }
    combined += trimmed;
    return combined;
}

std::string fallbackBm1688ModelPath(const std::string& modelPath)
{
    const std::string marker = "/BM1688/";
    size_t pos = modelPath.find(marker);
    if (pos == std::string::npos)
    {
        return modelPath;
    }
    return modelPath.substr(0, pos + 1) + modelPath.substr(pos + marker.size());
}

std::string resolveExistingModelPath(const std::string& baseDir, const std::string& rawPath)
{
    std::string resolvedPath = resolveConfigPath(baseDir, rawPath);
    if (resolvedPath.empty())
    {
        return resolvedPath;
    }
    if (pathExists(resolvedPath))
    {
        return resolvedPath;
    }

    std::string fallbackPath = fallbackBm1688ModelPath(resolvedPath);
    if (fallbackPath != resolvedPath && pathExists(fallbackPath))
    {
        return fallbackPath;
    }
    return resolvedPath;
}

std::string buildTaskRtspOutputUrl(int taskId)
{
    return "rtsp://localhost:8554/task_" + std::to_string(taskId);
}

int buildTaskStreamChannelId(int taskId)
{
    return TASK_STREAM_CHANNEL_BASE + taskId;
}

bool prepareFaceRuntimeConfigs(int taskId,
                               int streamChannelId,
                               const std::string& channelUrl,
                               int sampleInterval,
                               const std::string& sampleStrategyRaw,
                               int targetEncodeFps,
                               int recognitionFrameInterval,
                               std::string& runtimeFaceConfigPath,
                               std::string& errorMessage)
{
    // 默认更偏向实时预览流畅度，同时仍控制人脸支路负载。
    sampleInterval = clampInt(sampleInterval, 1, 12);
    std::string sampleStrategy = trimString(sampleStrategyRaw);
    if (sampleStrategy != "KEEP" && sampleStrategy != "DROP")
    {
        sampleStrategy = "DROP";
    }
    targetEncodeFps = clampInt(targetEncodeFps, 1, 25);
    recognitionFrameInterval = clampInt(recognitionFrameInterval, 1, 12);
    const int kFaceOsdElementId = 5010;
    const int kFaceEncodeElementId = 5012;

    const std::string sourceFaceConfigPath =
        FACE_PROJECT_ROOT + "/config/retinaface_distributor_resnet_faiss_converger.json";
    const std::string sourceFaceConfigDir = FACE_PROJECT_ROOT + "/config";

    if (!ensureDirectory(FACE_RUNTIME_CONFIG_DIR))
    {
        errorMessage = "Failed to create face runtime config directory";
        return false;
    }

    std::ifstream faceConfigIn(sourceFaceConfigPath);
    if (!faceConfigIn.is_open())
    {
        errorMessage = "Face config not found";
        return false;
    }

    json faceConfigJson;
    try
    {
        faceConfigIn >> faceConfigJson;
    }
    catch (const std::exception&)
    {
        errorMessage = "Face config is invalid";
        return false;
    }
    faceConfigIn.close();

    std::string engineConfigPath = resolveConfigPath(
        FACE_SAMPLES_BUILD_DIR,
        faceConfigJson.value("engine_config_path", "")
    );
    if (engineConfigPath.empty())
    {
        errorMessage = "Face engine config path is missing";
        return false;
    }

    std::ifstream engineConfigIn(engineConfigPath);
    if (!engineConfigIn.is_open())
    {
        errorMessage = "Face engine config not found";
        return false;
    }

    json engineConfigJson;
    try
    {
        engineConfigIn >> engineConfigJson;
    }
    catch (const std::exception&)
    {
        errorMessage = "Face engine config is invalid";
        return false;
    }
    engineConfigIn.close();

    if (!engineConfigJson.is_array() || engineConfigJson.empty() || !engineConfigJson[0].is_object())
    {
        errorMessage = "Face engine config format is invalid";
        return false;
    }

    json& graph = engineConfigJson[0];
    if (!graph.contains("elements") || !graph["elements"].is_array() ||
        !graph.contains("connections") || !graph["connections"].is_array())
    {
        errorMessage = "Face engine graph is missing elements or connections";
        return false;
    }

    std::string distributorConfigPath;
    std::string encodeConfigPath;
    int httpPushElementId = -1;

    for (auto& element : graph["elements"])
    {
        int elementId = element.value("element_id", -1);
        std::string elementConfigPath = resolveConfigPath(
            FACE_SAMPLES_BUILD_DIR,
            element.value("element_config", "")
        );
        if (!elementConfigPath.empty())
        {
            element["element_config"] = elementConfigPath;
        }

        if (elementId == 5004)
        {
            distributorConfigPath = elementConfigPath;
        }
        else if (elementId == kFaceEncodeElementId)
        {
            encodeConfigPath = elementConfigPath;
        }
        else if (elementId == 5011)
        {
            httpPushElementId = elementId;
        }
    }

    if (distributorConfigPath.empty())
    {
        errorMessage = "Face distributor config path is missing";
        return false;
    }

    if (encodeConfigPath.empty())
    {
        encodeConfigPath = sourceFaceConfigDir + "/encode.json";
        if (!pathExists(encodeConfigPath))
        {
            errorMessage = "Face encode config not found";
            return false;
        }

        graph["elements"].push_back({
            {"element_id", kFaceEncodeElementId},
            {"element_config", encodeConfigPath},
            {"ports", {
                {"output", json::array({
                    {
                        {"port_id", 0},
                        {"is_sink", true},
                        {"is_src", false}
                    }
                })}
            }}
        });
    }

    std::ifstream distributorConfigIn(distributorConfigPath);
    if (!distributorConfigIn.is_open())
    {
        errorMessage = "Face distributor config not found";
        return false;
    }

    json distributorConfigJson;
    try
    {
        distributorConfigIn >> distributorConfigJson;
    }
    catch (const std::exception&)
    {
        errorMessage = "Face distributor config is invalid";
        return false;
    }
    distributorConfigIn.close();

    if (!distributorConfigJson.contains("configure") || !distributorConfigJson["configure"].is_object())
    {
        distributorConfigJson["configure"] = json::object();
    }

    json& distributorConfigure = distributorConfigJson["configure"];
    distributorConfigure["class_names_file"] = resolveConfigPath(
        FACE_SAMPLES_BUILD_DIR,
        distributorConfigure.value("class_names_file", "")
    );
    if (distributorConfigure.contains("rules") && distributorConfigure["rules"].is_array())
    {
        for (auto& rule : distributorConfigure["rules"])
        {
            if (rule.is_object())
            {
                rule["frame_interval"] = recognitionFrameInterval;
            }
        }
    }

    std::ifstream encodeConfigIn(encodeConfigPath);
    if (!encodeConfigIn.is_open())
    {
        errorMessage = "Face encode config not found";
        return false;
    }

    json encodeConfigJson;
    try
    {
        encodeConfigIn >> encodeConfigJson;
    }
    catch (const std::exception&)
    {
        errorMessage = "Face encode config is invalid";
        return false;
    }
    encodeConfigIn.close();

    if (!encodeConfigJson.contains("configure") || !encodeConfigJson["configure"].is_object())
    {
        encodeConfigJson["configure"] = json::object();
    }
    encodeConfigJson["configure"]["encode_type"] = "RTSP";
    encodeConfigJson["configure"]["ip"] = "127.0.0.1";
    encodeConfigJson["configure"]["fps"] = targetEncodeFps;
    encodeConfigJson["thread_number"] = 4;

    const std::string taskSuffix = "_task_" + std::to_string(taskId);
    const std::string runtimeDistributorConfigPath =
        FACE_RUNTIME_CONFIG_DIR + "/distributor_frame_class_runtime" + taskSuffix + ".json";
    const std::string runtimeEncodeConfigPath =
        FACE_RUNTIME_CONFIG_DIR + "/encode_runtime" + taskSuffix + ".json";
    const std::string runtimeEngineConfigPath =
        FACE_RUNTIME_CONFIG_DIR + "/engine_group_runtime" + taskSuffix + ".json";
    runtimeFaceConfigPath =
        FACE_RUNTIME_CONFIG_DIR + "/retinaface_distributor_resnet_faiss_converger_runtime" + taskSuffix + ".json";

    if (!writeJsonFile(runtimeDistributorConfigPath, distributorConfigJson) ||
        !writeJsonFile(runtimeEncodeConfigPath, encodeConfigJson))
    {
        errorMessage = "Failed to write face runtime element config";
        return false;
    }

    for (auto& element : graph["elements"])
    {
        int elementId = element.value("element_id", -1);
        if (elementId == 5004)
        {
            element["element_config"] = runtimeDistributorConfigPath;
        }
        else if (elementId == 5012)
        {
            element["element_config"] = runtimeEncodeConfigPath;
        }
    }

    if (httpPushElementId > 0)
    {
        auto& elements = graph["elements"];
        elements.erase(
            std::remove_if(
                elements.begin(),
                elements.end(),
                [httpPushElementId](const json& element)
                {
                    return element.value("element_id", -1) == httpPushElementId;
                }
            ),
            elements.end()
        );

        json filteredConnections = json::array();
        for (const auto& connection : graph["connections"])
        {
            int srcId = connection.value("src_element_id", -1);
            int dstId = connection.value("dst_element_id", -1);
            if (srcId == httpPushElementId || dstId == httpPushElementId)
            {
                continue;
            }
            filteredConnections.push_back(connection);
        }
        graph["connections"] = filteredConnections;
    }

    bool hasEncodeConnection = false;
    for (const auto& connection : graph["connections"])
    {
        if (connection.value("src_element_id", -1) == kFaceOsdElementId &&
            connection.value("dst_element_id", -1) == kFaceEncodeElementId)
        {
            hasEncodeConnection = true;
            break;
        }
    }
    if (!hasEncodeConnection)
    {
        graph["connections"].push_back({
            {"src_element_id", kFaceOsdElementId},
            {"src_port", 0},
            {"dst_element_id", kFaceEncodeElementId},
            {"dst_port", 0}
        });
    }

    faceConfigJson["channels"] = json::array();
    faceConfigJson["channels"].push_back({
        {"channel_id", streamChannelId},
        {"url", channelUrl},
        {"source_type", "RTSP"},
        {"loop_num", 1},
        {"sample_interval", sampleInterval},
        {"sample_strategy", sampleStrategy},
        {"fps", targetEncodeFps}
    });
    faceConfigJson["engine_config_path"] = runtimeEngineConfigPath;

    if (!writeJsonFile(runtimeEngineConfigPath, engineConfigJson) ||
        !writeJsonFile(runtimeFaceConfigPath, faceConfigJson))
    {
        errorMessage = "Failed to write face runtime graph config";
        return false;
    }

    errorMessage.clear();
    return true;
}

bool prepareYolov8RuntimeDemoConfig(int taskId,
                                    const std::string& algorithm,
                                    int streamChannelId,
                                    const std::string& channelUrl,
                                    const json& regionConfigs,
                                    std::string& runtimeDemoConfigPath,
                                    std::string& runtimeThresholdConfigPath,
                                    std::string& runtimeFilterConfigPath,
                                    std::string& runtimeOsdConfigPath,
                                    std::string& errorMessage)
{
    const std::string sourceDemoConfigPath = SOPHON_STREAM_YOLO_CONFIG_DIR + "/yolov8_demo.json";
    const std::string sourceThresholdConfigPath =
        SOPHON_STREAM_YOLO_CONFIG_DIR + "/yolov8_classthresh_roi_example.json";
    const std::string sourceFilterConfigPath =
        SOPHON_STREAM_ROOT + "/samples/tripwire/config/filter.json";
    const std::string sourceSamplesBuildDir = SOPHON_STREAM_SAMPLES_BUILD_DIR;
    const bool enableTaskRegionOverlay = isRegionIntrusionAlgorithm(algorithm);
    const bool enableTaskRegionFilter =
        enableTaskRegionOverlay && regionConfigs.is_array() && !regionConfigs.empty();

    runtimeFilterConfigPath.clear();
    runtimeOsdConfigPath.clear();

    if (!ensureDirectory(YOLO_RUNTIME_CONFIG_DIR))
    {
        errorMessage = "Failed to create yolov8 runtime config directory";
        return false;
    }

    std::ifstream demoConfigIn(sourceDemoConfigPath.c_str());
    if (!demoConfigIn.is_open())
    {
        errorMessage = "Yolov8 demo config not found";
        return false;
    }

    json demoConfigJson;
    try
    {
        demoConfigIn >> demoConfigJson;
    }
    catch (const std::exception&)
    {
        errorMessage = "Yolov8 demo config is invalid";
        return false;
    }
    demoConfigIn.close();

    demoConfigJson["channels"] = json::array();
    demoConfigJson["channels"].push_back({
        {"channel_id", streamChannelId},
        {"fps", 3},
        {"loop_num", 1},
        {"sample_interval", 2},
        {"source_type", "RTSP"},
        {"url", channelUrl}
    });

    std::ifstream thresholdConfigIn(sourceThresholdConfigPath.c_str());
    if (!thresholdConfigIn.is_open())
    {
        errorMessage = "Yolov8 threshold config not found";
        return false;
    }

    json thresholdConfigJson;
    try
    {
        thresholdConfigIn >> thresholdConfigJson;
    }
    catch (const std::exception&)
    {
        errorMessage = "Yolov8 threshold config is invalid";
        return false;
    }
    thresholdConfigIn.close();

    std::string runtimeEngineConfigPath;
    std::string engineConfigPath = resolveConfigPath(
        sourceSamplesBuildDir,
        demoConfigJson.value("engine_config_path", "")
    );
    if (engineConfigPath.empty())
    {
        errorMessage = "Yolov8 engine config path is missing";
        return false;
    }

    std::ifstream engineConfigIn(engineConfigPath.c_str());
    if (!engineConfigIn.is_open())
    {
        errorMessage = "Yolov8 engine config not found";
        return false;
    }

    json engineConfigJson;
    try
    {
        engineConfigIn >> engineConfigJson;
    }
    catch (const std::exception&)
    {
        errorMessage = "Yolov8 engine config is invalid";
        return false;
    }
    engineConfigIn.close();

    if (!engineConfigJson.is_array())
    {
        errorMessage = "Yolov8 engine config format is invalid";
        return false;
    }

    runtimeThresholdConfigPath =
        YOLO_RUNTIME_CONFIG_DIR + "/yolov8_classthresh_task_" + std::to_string(taskId) + ".json";
    runtimeEngineConfigPath =
        YOLO_RUNTIME_CONFIG_DIR + "/engine_group_task_" + std::to_string(taskId) + ".json";

    bool thresholdBound = false;
    std::string osdConfigPath;
    for (auto& graph : engineConfigJson)
    {
        if (!graph.is_object() || !graph.contains("elements") || !graph["elements"].is_array())
        {
            continue;
        }
        for (auto& element : graph["elements"])
        {
            if (!element.is_object())
            {
                continue;
            }
            if (element.value("element_id", -1) == 5001)
            {
                element["element_config"] = runtimeThresholdConfigPath;
                thresholdBound = true;
            }
            else if (enableTaskRegionOverlay && element.value("element_id", -1) == 5005)
            {
                std::string elementConfigPath = resolveConfigPath(
                    sourceSamplesBuildDir,
                    element.value("element_config", "")
                );
                if (!elementConfigPath.empty())
                {
                    osdConfigPath = elementConfigPath;
                }
            }
        }
    }

    if (!thresholdBound)
    {
        errorMessage = "Yolov8 engine config is missing threshold element";
        return false;
    }

    json osdConfigJson;
    json filterConfigJson;
    int runtimeFrameWidth = 1920;
    int runtimeFrameHeight = 1080;
    if (!probeStreamFrameSize(channelUrl, runtimeFrameWidth, runtimeFrameHeight))
    {
        if (regionConfigs.is_array() && !regionConfigs.empty() && regionConfigs[0].is_object())
        {
            runtimeFrameWidth = clampInt(
                regionConfigs[0].contains("frameWidth")
                    ? parseJsonInt(regionConfigs[0]["frameWidth"], runtimeFrameWidth)
                    : runtimeFrameWidth,
                1,
                16384);
            runtimeFrameHeight = clampInt(
                regionConfigs[0].contains("frameHeight")
                    ? parseJsonInt(regionConfigs[0]["frameHeight"], runtimeFrameHeight)
                    : runtimeFrameHeight,
                1,
                16384);
        }
    }

    if (!thresholdConfigJson.contains("configure") || !thresholdConfigJson["configure"].is_object())
    {
        thresholdConfigJson["configure"] = json::object();
    }

    if (enableTaskRegionOverlay)
    {
        if (osdConfigPath.empty())
        {
            errorMessage = "Yolov8 osd config path is missing";
            return false;
        }

        if (!loadJsonFile(osdConfigPath, osdConfigJson, errorMessage))
        {
            errorMessage = "Yolov8 osd config not found or invalid";
            return false;
        }

        json overlayRegions = buildTaskRegionOverlayRegions(
            regionConfigs, runtimeFrameWidth, runtimeFrameHeight, false);
        if (!osdConfigJson.contains("configure") || !osdConfigJson["configure"].is_object())
        {
            osdConfigJson["configure"] = json::object();
        }
        osdConfigJson["configure"]["overlay_regions"] = overlayRegions;
        // 区域入侵不再使用旧的矩形 ROI 裁剪，只保留任务级多边形区域。
        thresholdConfigJson["configure"].erase("roi");
    }

    if (enableTaskRegionFilter)
    {
        if (!loadJsonFile(sourceFilterConfigPath, filterConfigJson, errorMessage))
        {
            errorMessage = "Yolov8 runtime filter config template not found or invalid";
            return false;
        }
        if (!filterConfigJson.contains("configure") || !filterConfigJson["configure"].is_object())
        {
            filterConfigJson["configure"] = json::object();
        }
        // 保持视频链路持续出帧，避免未命中区域时 RTSP 预览卡断重连。
        filterConfigJson["configure"]["forward_empty_frame"] = true;
    }

    runtimeDemoConfigPath =
        YOLO_RUNTIME_CONFIG_DIR + "/yolov8_demo_task_" + std::to_string(taskId) + ".json";
    runtimeOsdConfigPath =
        YOLO_RUNTIME_CONFIG_DIR + "/osd_task_" + std::to_string(taskId) + ".json";
    runtimeFilterConfigPath =
        YOLO_RUNTIME_CONFIG_DIR + "/filter_task_" + std::to_string(taskId) + ".json";
    demoConfigJson["engine_config_path"] = runtimeEngineConfigPath;

    if (enableTaskRegionFilter && !engineConfigJson.empty() && engineConfigJson[0].is_object())
    {
        ensureYolov8RegionFilterGraph(engineConfigJson[0], runtimeFilterConfigPath);
    }

    for (auto& graph : engineConfigJson)
    {
        if (!graph.is_object() || !graph.contains("elements") || !graph["elements"].is_array())
        {
            continue;
        }
        for (auto& element : graph["elements"])
        {
            if (!element.is_object())
            {
                continue;
            }

            int elementId = element.value("element_id", -1);
            if (elementId == 5001)
            {
                element["element_config"] = runtimeThresholdConfigPath;
            }
            else if (enableTaskRegionOverlay && elementId == 5005)
            {
                element["element_config"] = runtimeOsdConfigPath;
            }
            else if (enableTaskRegionFilter && elementId == 5006)
            {
                element["element_config"] = runtimeFilterConfigPath;
            }
        }
    }

    bool writeOk =
        writeJsonFile(runtimeThresholdConfigPath, thresholdConfigJson) &&
        writeJsonFile(runtimeEngineConfigPath, engineConfigJson) &&
        writeJsonFile(runtimeDemoConfigPath, demoConfigJson);

    if (writeOk && enableTaskRegionOverlay)
    {
        writeOk = writeOk && writeJsonFile(runtimeOsdConfigPath, osdConfigJson);
    }

    if (writeOk && enableTaskRegionFilter)
    {
        json filterAreas = buildTaskRegionFilterAreas(
            regionConfigs, runtimeFrameWidth, runtimeFrameHeight, false);
        writeOk =
            writeOk &&
            writeJsonFile(runtimeFilterConfigPath, filterConfigJson) &&
            updatePolygonInFilterConfigFile(runtimeFilterConfigPath, filterAreas, streamChannelId);
    }

    if (!writeOk)
    {
        errorMessage = "Failed to write yolov8 runtime config";
        return false;
    }

    if (!enableTaskRegionOverlay)
    {
        runtimeOsdConfigPath.clear();
    }
    if (!enableTaskRegionFilter)
    {
        runtimeFilterConfigPath.clear();
    }

    errorMessage.clear();
    return true;
}

std::string toLowerString(const std::string& value)
{
    std::string out = value;
    for (size_t i = 0; i < out.size(); ++i)
    {
        out[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(out[i])));
    }
    return out;
}

std::string fileExtension(const std::string& filename)
{
    size_t dotPos = filename.find_last_of('.');
    if (dotPos == std::string::npos || dotPos == filename.size() - 1)
    {
        return "";
    }
    return toLowerString(filename.substr(dotPos));
}

bool isSupportedImageFile(const std::string& filename)
{
    static const std::set<std::string> kExts = {
        ".jpg", ".jpeg", ".png", ".bmp", ".webp"
    };
    return kExts.find(fileExtension(filename)) != kExts.end();
}

std::string sanitizeUploadFilename(const std::string& rawFilename, int indexSeed)
{
    std::string filename = rawFilename;
    size_t slashPos = filename.find_last_of("/\\");
    if (slashPos != std::string::npos)
    {
        filename = filename.substr(slashPos + 1);
    }

    std::string sanitized;
    for (size_t i = 0; i < filename.size(); ++i)
    {
        unsigned char c = static_cast<unsigned char>(filename[i]);
        if (std::isalnum(c) || c == '.' || c == '_' || c == '-')
        {
            sanitized.push_back(static_cast<char>(c));
        }
        else
        {
            sanitized.push_back('_');
        }
    }

    if (sanitized.empty() || sanitized == "." || sanitized == "..")
    {
        sanitized = "face_" + std::to_string(indexSeed) + ".jpg";
    }
    else if (!isSupportedImageFile(sanitized))
    {
        sanitized += ".jpg";
    }
    return sanitized;
}

bool isValidFaceName(const std::string& name)
{
    std::string trimmed = trimString(name);
    if (trimmed.empty() || trimmed.size() > 64)
    {
        return false;
    }
    if (trimmed.find("..") != std::string::npos)
    {
        return false;
    }
    if (trimmed.find('/') != std::string::npos || trimmed.find('\\') != std::string::npos)
    {
        return false;
    }
    if (trimmed.find_first_of("\"'<>`;&|$") != std::string::npos)
    {
        return false;
    }
    for (size_t i = 0; i < trimmed.size(); ++i)
    {
        if (static_cast<unsigned char>(trimmed[i]) < 32)
        {
            return false;
        }
    }
    return true;
}

bool isAsciiFaceLabelChar(char c)
{
    unsigned char uc = static_cast<unsigned char>(c);
    return std::isalnum(uc) || c == '_' || c == '-';
}

bool isValidAsciiFaceLabel(const std::string& label)
{
    if (label.empty() || label.size() > 64)
    {
        return false;
    }
    for (size_t i = 0; i < label.size(); ++i)
    {
        if (!isAsciiFaceLabelChar(label[i]))
        {
            return false;
        }
    }
    return true;
}

bool isAllAscii(const std::string& value)
{
    for (size_t i = 0; i < value.size(); ++i)
    {
        if (static_cast<unsigned char>(value[i]) >= 128)
        {
            return false;
        }
    }
    return true;
}

uint32_t fnv1a32(const std::string& value)
{
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < value.size(); ++i)
    {
        hash ^= static_cast<unsigned char>(value[i]);
        hash *= 16777619u;
    }
    return hash;
}

std::string makeFallbackFaceLabel(const std::string& faceName, int salt = 0)
{
    std::string seed = faceName + "#" + std::to_string(salt);
    uint32_t hash = fnv1a32(seed);
    std::ostringstream oss;
    oss << "face_" << std::hex << std::nouppercase << std::setw(8) << std::setfill('0') << hash;
    return oss.str();
}

std::vector<std::string> listFaceDirectoryNames()
{
    std::vector<std::string> names;
    DIR* dir = opendir(FACE_TRAIN_DIR.c_str());
    if (!dir)
    {
        return names;
    }

    struct dirent* entry = nullptr;
    while ((entry = readdir(dir)) != nullptr)
    {
        std::string name = entry->d_name;
        if (name == "." || name == ".." || (!name.empty() && name[0] == '.'))
        {
            continue;
        }
        std::string fullPath = FACE_TRAIN_DIR + "/" + name;
        if (isDirectoryPath(fullPath))
        {
            names.push_back(name);
        }
    }
    closedir(dir);
    std::sort(names.begin(), names.end());
    return names;
}

json loadFaceLabelMap()
{
    std::ifstream in(FACE_LABEL_MAP_FILE.c_str());
    if (!in.is_open())
    {
        return json::object();
    }
    json data;
    try
    {
        in >> data;
    }
    catch (...)
    {
        return json::object();
    }
    if (!data.is_object())
    {
        return json::object();
    }
    return data;
}

bool saveFaceLabelMap(const json& labelMap)
{
    std::ofstream out(FACE_LABEL_MAP_FILE.c_str(), std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        return false;
    }
    out << labelMap.dump(2);
    return out.good();
}

std::set<std::string> collectUsedFaceLabels(const json& labelMap, const std::string& exceptFaceName = "")
{
    std::set<std::string> used;
    if (!labelMap.is_object())
    {
        return used;
    }
    for (json::const_iterator it = labelMap.begin(); it != labelMap.end(); ++it)
    {
        if (!exceptFaceName.empty() && it.key() == exceptFaceName)
        {
            continue;
        }
        if (it.value().is_string())
        {
            std::string label = trimString(it.value().get<std::string>());
            if (isValidAsciiFaceLabel(label))
            {
                used.insert(label);
            }
        }
    }
    return used;
}

std::string makeUniqueFaceLabel(const std::string& baseLabel, const std::set<std::string>& used)
{
    if (!used.count(baseLabel))
    {
        return baseLabel;
    }
    for (int i = 2; i < 10000; ++i)
    {
        std::string suffix = "_" + std::to_string(i);
        std::string candidate = baseLabel;
        if (candidate.size() + suffix.size() > 64)
        {
            candidate = candidate.substr(0, 64 - suffix.size());
        }
        candidate += suffix;
        if (!used.count(candidate))
        {
            return candidate;
        }
    }
    return makeFallbackFaceLabel(baseLabel, static_cast<int>(used.size()));
}

std::string ensureFaceLabelForName(const std::string& faceName, const std::string& preferredLabel, json& labelMap)
{
    if (!labelMap.is_object())
    {
        labelMap = json::object();
    }

    std::set<std::string> used = collectUsedFaceLabels(labelMap, faceName);
    std::string existing;
    if (labelMap.contains(faceName) && labelMap[faceName].is_string())
    {
        existing = trimString(labelMap[faceName].get<std::string>());
    }

    std::string candidate = trimString(preferredLabel);
    if (candidate.empty() || !isValidAsciiFaceLabel(candidate))
    {
        candidate = existing;
    }
    if (candidate.empty() || !isValidAsciiFaceLabel(candidate))
    {
        if (isAllAscii(faceName) && isValidAsciiFaceLabel(faceName))
        {
            candidate = faceName;
        }
        else
        {
            candidate = makeFallbackFaceLabel(faceName, 0);
        }
    }
    if (!isValidAsciiFaceLabel(candidate))
    {
        candidate = makeFallbackFaceLabel(faceName, 1);
    }

    if (used.count(candidate))
    {
        candidate = makeUniqueFaceLabel(candidate, used);
    }

    labelMap[faceName] = candidate;
    return candidate;
}

void syncFaceLabelMapWithCurrentFaces(const std::vector<std::string>& faceNames, json& labelMap)
{
    if (!labelMap.is_object())
    {
        labelMap = json::object();
    }

    std::set<std::string> nameSet(faceNames.begin(), faceNames.end());
    std::vector<std::string> staleKeys;
    for (json::iterator it = labelMap.begin(); it != labelMap.end(); ++it)
    {
        if (!nameSet.count(it.key()))
        {
            staleKeys.push_back(it.key());
        }
    }
    for (size_t i = 0; i < staleKeys.size(); ++i)
    {
        labelMap.erase(staleKeys[i]);
    }

    for (size_t i = 0; i < faceNames.size(); ++i)
    {
        ensureFaceLabelForName(faceNames[i], "", labelMap);
    }
}

std::string getFaceLabelForName(const json& labelMap, const std::string& faceName)
{
    if (labelMap.is_object() && labelMap.contains(faceName) && labelMap[faceName].is_string())
    {
        std::string label = trimString(labelMap[faceName].get<std::string>());
        if (isValidAsciiFaceLabel(label))
        {
            return label;
        }
    }
    if (isAllAscii(faceName) && isValidAsciiFaceLabel(faceName))
    {
        return faceName;
    }
    return makeFallbackFaceLabel(faceName, 0);
}

bool remapFaceIndexLabelFile(const std::string& indexFilePath, json& labelMap, std::string& errorMessage)
{
    std::ifstream in(indexFilePath.c_str());
    if (!in.is_open())
    {
        errorMessage = "无法读取临时标签文件";
        return false;
    }

    std::vector<std::string> mappedLines;
    std::string line;
    while (std::getline(in, line))
    {
        std::string faceName = trimString(line);
        if (faceName.empty())
        {
            continue;
        }
        std::string label = ensureFaceLabelForName(faceName, "", labelMap);
        mappedLines.push_back(label);
    }
    in.close();

    std::string mappedFilePath = indexFilePath + ".mapped";
    std::ofstream out(mappedFilePath.c_str(), std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        errorMessage = "无法写入映射后的标签文件";
        return false;
    }
    for (size_t i = 0; i < mappedLines.size(); ++i)
    {
        out << mappedLines[i] << "\n";
    }
    out.close();
    if (!out.good())
    {
        errorMessage = "写入映射标签失败";
        return false;
    }

    if (std::rename(mappedFilePath.c_str(), indexFilePath.c_str()) != 0)
    {
        std::remove(mappedFilePath.c_str());
        errorMessage = "替换标签文件失败";
        return false;
    }
    return true;
}

bool truncateFile(const std::string& path)
{
    std::ofstream out(path.c_str(), std::ios::binary | std::ios::trunc);
    return out.good();
}

std::string detectFaceBmodelPath()
{
    std::vector<std::string> candidates;
    candidates.push_back(FACE_PROJECT_ROOT + "/data/models/BM1688/resnet_arcface_fp32_1b.bmodel");
    candidates.push_back(FACE_PROJECT_ROOT + "/data/models/BM1684X/resnet_arcface_fp32_1b.bmodel");

    for (size_t i = 0; i < candidates.size(); ++i)
    {
        if (pathExists(candidates[i]))
        {
            return candidates[i];
        }
    }
    return candidates[0];
}

std::string buildFacePythonPath()
{
    std::vector<std::string> candidates;
    candidates.push_back("/home/linaro/.local/lib/python3.10/site-packages");
    candidates.push_back("/opt/sophon/sophon-opencv_1.9.0/opencv-python");
    candidates.push_back("/opt/sophon/sophon-opencv-latest/opencv-python");

    std::vector<std::string> validPaths;
    for (size_t i = 0; i < candidates.size(); ++i)
    {
        if (isDirectoryPath(candidates[i]))
        {
            validPaths.push_back(candidates[i]);
        }
    }

    std::ostringstream oss;
    for (size_t i = 0; i < validPaths.size(); ++i)
    {
        if (i > 0)
        {
            oss << ":";
        }
        oss << validPaths[i];
    }
    oss << ":$PYTHONPATH";
    return oss.str();
}

std::string buildFaceLdLibraryPath()
{
    std::vector<std::string> candidates;
    candidates.push_back("/opt/sophon/libsophon-current/lib");
    candidates.push_back(SOPHON_STREAM_BUILD_LIB_DIR);
    candidates.push_back("/opt/sophon/sophon-opencv_1.9.0/lib");

    std::vector<std::string> validPaths;
    for (size_t i = 0; i < candidates.size(); ++i)
    {
        if (isDirectoryPath(candidates[i]))
        {
            validPaths.push_back(candidates[i]);
        }
    }

    std::ostringstream oss;
    for (size_t i = 0; i < validPaths.size(); ++i)
    {
        if (i > 0)
        {
            oss << ":";
        }
        oss << validPaths[i];
    }
    oss << ":$LD_LIBRARY_PATH";
    return oss.str();
}

int rebuildFaceFeatureDatabase(std::string& logOutput)
{
    if (!ensureDirectory(FACE_TRAIN_DIR) || !ensureDirectory(FACE_DB_DIR))
    {
        logOutput = "无法创建人脸库目录";
        return -1;
    }

    std::string tmpDbData = FACE_DB_DATA_FILE + ".tmp";
    std::string tmpIndexLabel = FACE_INDEX_LABEL_FILE + ".tmp";
    if (!truncateFile(tmpDbData) || !truncateFile(tmpIndexLabel))
    {
        logOutput = "无法写入人脸库数据文件";
        return -2;
    }

    json labelMap = loadFaceLabelMap();
    std::vector<std::string> currentFaces = listFaceDirectoryNames();
    syncFaceLabelMapWithCurrentFaces(currentFaces, labelMap);
    saveFaceLabelMap(labelMap);

    std::string bmodelPath = detectFaceBmodelPath();
    std::string pythonPath = buildFacePythonPath();
    std::string ldLibraryPath = buildFaceLdLibraryPath();
    std::string cmd = "cd " + shellQuote(FACE_SCRIPT_DIR) +
                      " && PYTHONPATH=" + pythonPath +
                      " LD_LIBRARY_PATH=" + ldLibraryPath + " " +
                      "python3 " + shellQuote(FACE_WRITE_SCRIPT) +
                      " --input " + shellQuote(FACE_TRAIN_DIR) +
                      " --bmodel " + shellQuote(bmodelPath) +
                      " --db_data " + shellQuote(tmpDbData) +
                      " --index_label " + shellQuote(tmpIndexLabel) +
                      " --dev_id 0 > " + shellQuote(FACE_REBUILD_LOG_FILE) + " 2>&1";

    int ret = std::system(cmd.c_str());
    if (ret == 0)
    {
        std::string mapErr;
        if (!remapFaceIndexLabelFile(tmpIndexLabel, labelMap, mapErr))
        {
            ret = -4;
            logOutput = mapErr;
            std::remove(tmpDbData.c_str());
            std::remove(tmpIndexLabel.c_str());
        }
        else if (!saveFaceLabelMap(labelMap))
        {
            ret = -5;
            logOutput = "保存标签映射文件失败";
            std::remove(tmpDbData.c_str());
            std::remove(tmpIndexLabel.c_str());
        }
    }

    if (ret == 0)
    {
        if (std::rename(tmpDbData.c_str(), FACE_DB_DATA_FILE.c_str()) != 0 ||
            std::rename(tmpIndexLabel.c_str(), FACE_INDEX_LABEL_FILE.c_str()) != 0)
        {
            ret = -3;
            std::remove(tmpDbData.c_str());
            std::remove(tmpIndexLabel.c_str());
        }
    }
    else
    {
        std::remove(tmpDbData.c_str());
        std::remove(tmpIndexLabel.c_str());
    }

    std::string fileLog;
    std::ifstream logFile(FACE_REBUILD_LOG_FILE.c_str(), std::ios::in | std::ios::binary);
    if (logFile.good())
    {
        std::ostringstream oss;
        oss << logFile.rdbuf();
        fileLog = oss.str();
    }

    if (!fileLog.empty())
    {
        if (logOutput.empty())
        {
            logOutput = fileLog;
        }
        else
        {
            logOutput += "\n";
            logOutput += fileLog;
        }
    }
    else if (logOutput.empty())
    {
        logOutput = "";
    }

    return ret;
}

std::vector<std::string> listImageFiles(const std::string& folder)
{
    std::vector<std::string> files;
    DIR* dir = opendir(folder.c_str());
    if (!dir)
    {
        return files;
    }

    struct dirent* entry = nullptr;
    while ((entry = readdir(dir)) != nullptr)
    {
        std::string name = entry->d_name;
        if (name == "." || name == "..")
        {
            continue;
        }

        std::string fullPath = folder + "/" + name;
        struct stat st;
        if (stat(fullPath.c_str(), &st) != 0 || !S_ISREG(st.st_mode))
        {
            continue;
        }

        if (isSupportedImageFile(name))
        {
            files.push_back(name);
        }
    }
    closedir(dir);
    std::sort(files.begin(), files.end());
    return files;
}

std::map<std::string, int> loadFaceFeatureCountMap()
{
    std::map<std::string, int> counter;
    std::ifstream in(FACE_INDEX_LABEL_FILE.c_str());
    if (!in.is_open())
    {
        return counter;
    }

    std::string line;
    while (std::getline(in, line))
    {
        line = trimString(line);
        if (!line.empty())
        {
            counter[line] += 1;
        }
    }
    return counter;
}

std::string guessImageMimeType(const std::string& filename)
{
    std::string ext = fileExtension(filename);
    if (ext == ".png")
    {
        return "image/png";
    }
    if (ext == ".bmp")
    {
        return "image/bmp";
    }
    if (ext == ".webp")
    {
        return "image/webp";
    }
    return "image/jpeg";
}

bool readBinaryContent(const std::string& filePath, std::string& content)
{
    std::ifstream in(filePath.c_str(), std::ios::in | std::ios::binary);
    if (!in.good())
    {
        return false;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    content = ss.str();
    return true;
}
//----------------------------------------------------------------------------------------------------------------------------
// 从default.json文件读取默认地址信息
void readDefaultAddressesFromJson(json &defaultAddresses)
{
    std::ifstream jsonFile("default.json");
    if (jsonFile.is_open())
    {
        jsonFile >> defaultAddresses;
        jsonFile.close();
    }
    else
    {
        std::cerr << "Error opening default.json file for reading." << std::endl;
    }
    // 初始化全局的默认地址变量（之前已经有这些全局变量声明）
    defaultRtspStreamUrl = defaultAddresses["current"].value("defaultRtspStreamUrl", "");
    defaultSaveLocation = defaultAddresses["current"].value("defaultSaveLocation", "");
    defaultRtspStreamUrl2 = defaultAddresses["current"].value("defaultRtspStreamUrl2", "");
    defaultSaveLocation2 = defaultAddresses["current"].value("defaultSaveLocation2", "");
}
//----------------------------------------------------------------------------------------------------------------------------
// 更新default.json文件中的默认地址信息
void updateDefaultAddressesInJson(const json &newAddresses)
{
    // 验证传入的JSON数据结构，只允许包含"current"和"initial"节点
    if (!newAddresses.contains("current") ||!newAddresses.contains("initial"))
    {
        std::cerr << "Invalid JSON structure passed to updateDefaultAddressesInJson. Must contain 'current' and 'initial' nodes." << std::endl;
        return;
    }

    std::ofstream jsonFile("default.json");
    if (jsonFile.is_open())
    {
        jsonFile << newAddresses.dump(4);
        jsonFile.close();
    }
    else
    {
        std::cerr << "Error opening default.json file for writing." << std::endl;
    }
}
//----------------------------------------------------------------------------------------------------------------------------
json readJsonObjectFile(const std::string& path)
{
    json data = json::object();
    std::ifstream in(path.c_str());
    if (!in.is_open())
    {
        return data;
    }

    try
    {
        in >> data;
    }
    catch (...)
    {
        data = json::object();
    }

    if (!data.is_object())
    {
        data = json::object();
    }
    return data;
}

void ensureDefaultAddressSections(json& defaultAddresses)
{
    if (!defaultAddresses.is_object())
    {
        defaultAddresses = json::object();
    }
    if (!defaultAddresses.contains("current") || !defaultAddresses["current"].is_object())
    {
        defaultAddresses["current"] = json::object();
    }
    if (!defaultAddresses.contains("initial") || !defaultAddresses["initial"].is_object())
    {
        defaultAddresses["initial"] = json::object();
    }
}

bool parseJsonBoolLike(const json& value, bool fallback = true)
{
    if (value.is_boolean())
    {
        return value.get<bool>();
    }
    if (value.is_number_integer())
    {
        return value.get<long long>() != 0;
    }
    if (value.is_string())
    {
        std::string normalized = toLowerString(trimString(value.get<std::string>()));
        if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on")
        {
            return true;
        }
        if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off")
        {
            return false;
        }
    }
    return fallback;
}

bool readDualStreamEnabledFromDefaultAddresses(const json& defaultAddresses, bool fallback = true)
{
    if (!defaultAddresses.is_object() ||
        !defaultAddresses.contains("current") ||
        !defaultAddresses["current"].is_object())
    {
        return fallback;
    }

    const json& current = defaultAddresses["current"];
    if (current.contains("dualStreamEnabled"))
    {
        return parseJsonBoolLike(current["dualStreamEnabled"], fallback);
    }
    if (current.contains("dual_stream_enabled"))
    {
        return parseJsonBoolLike(current["dual_stream_enabled"], fallback);
    }
    return fallback;
}

bool readPidFromFile(const std::string& pidFile, int& pid)
{
    pid = 0;
    std::ifstream in(pidFile.c_str());
    if (!in.is_open())
    {
        return false;
    }

    in >> pid;
    return pid > 0;
}

bool writePidFile(const std::string& pidFile, int pid)
{
    if (pid <= 0)
    {
        return false;
    }

    std::ofstream out(pidFile.c_str(), std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        return false;
    }

    out << pid;
    out.close();
    return out.good();
}

void removeFileIfExists(const std::string& path)
{
    if (!path.empty() && pathExists(path))
    {
        std::remove(path.c_str());
    }
}

double parseHumanSizeToGigabytes(const std::string& rawValue)
{
    std::string value = trimString(rawValue);
    if (value.empty())
    {
        return 0.0;
    }

    size_t numericEnd = 0;
    while (numericEnd < value.size() &&
           (std::isdigit(static_cast<unsigned char>(value[numericEnd])) || value[numericEnd] == '.'))
    {
        numericEnd++;
    }

    if (numericEnd == 0)
    {
        return 0.0;
    }

    double number = 0.0;
    try
    {
        number = std::stod(value.substr(0, numericEnd));
    }
    catch (...)
    {
        return 0.0;
    }

    char unit = 'G';
    for (size_t i = numericEnd; i < value.size(); ++i)
    {
        unsigned char c = static_cast<unsigned char>(value[i]);
        if (std::isalpha(c))
        {
            unit = static_cast<char>(std::toupper(c));
            break;
        }
    }

    switch (unit)
    {
    case 'T':
        return number * 1024.0;
    case 'G':
        return number;
    case 'M':
        return number / 1024.0;
    case 'K':
        return number / (1024.0 * 1024.0);
    case 'B':
        return number / (1024.0 * 1024.0 * 1024.0);
    default:
        return number;
    }
}

std::string formatByteSize(long long bytes)
{
    const char* units[] = {"B", "KB", "MB", "GB", "TB"};
    double size = static_cast<double>(bytes);
    int unitIndex = 0;
    while (size >= 1024.0 && unitIndex < 4)
    {
        size /= 1024.0;
        unitIndex++;
    }

    std::ostringstream oss;
    if (unitIndex == 0)
    {
        oss << bytes << " " << units[unitIndex];
    }
    else
    {
        oss << std::fixed << std::setprecision(size >= 10.0 ? 0 : 1) << size << " " << units[unitIndex];
    }
    return oss.str();
}

std::string formatLocalDateTime(time_t timestamp)
{
    char timeBuffer[32];
    std::tm localTm;
    std::memset(&localTm, 0, sizeof(localTm));
    localtime_r(&timestamp, &localTm);
    std::strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", &localTm);
    return timeBuffer;
}

bool isSupportedRecordingVideoFile(const std::string& filename)
{
    static const std::set<std::string> kExts = {
        ".mp4", ".avi", ".mkv", ".mov", ".m4v"
    };
    return kExts.find(fileExtension(filename)) != kExts.end();
}

bool isSupportedRecordingMediaFile(const std::string& filename)
{
    return isSupportedRecordingVideoFile(filename) || isSupportedImageFile(filename);
}

std::string guessRecordingMimeType(const std::string& filename)
{
    std::string ext = fileExtension(filename);
    if (ext == ".jpg" || ext == ".jpeg")
    {
        return "image/jpeg";
    }
    if (ext == ".png")
    {
        return "image/png";
    }
    if (ext == ".bmp")
    {
        return "image/bmp";
    }
    if (ext == ".webp")
    {
        return "image/webp";
    }
    if (ext == ".avi")
    {
        return "video/x-msvideo";
    }
    if (ext == ".mkv")
    {
        return "video/x-matroska";
    }
    if (ext == ".mov")
    {
        return "video/quicktime";
    }
    if (ext == ".m4v")
    {
        return "video/x-m4v";
    }
    return "video/mp4";
}

std::string tailTextFile(const std::string& filePath, size_t maxLines)
{
    if (maxLines == 0)
    {
        return "";
    }

    std::ifstream in(filePath.c_str());
    if (!in.is_open())
    {
        return "";
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line))
    {
        lines.push_back(line);
        if (lines.size() > maxLines)
        {
            lines.erase(lines.begin());
        }
    }

    std::ostringstream oss;
    for (size_t i = 0; i < lines.size(); ++i)
    {
        if (i > 0)
        {
            oss << "\n";
        }
        oss << lines[i];
    }
    return oss.str();
}

void loadCurrentRecordingDirectories(std::string& savePath1, std::string& savePath2)
{
    json defaultAddresses;
    readDefaultAddressesFromJson(defaultAddresses);
    ensureDefaultAddressSections(defaultAddresses);

    std::string activePath1;
    std::string activePath2;
    {
        std::lock_guard<std::mutex> lock(recordingStateMutex);
        activePath1 = activeRecordingSaveLocation1;
        activePath2 = activeRecordingSaveLocation2;
    }

    savePath1 = trimString(activePath1.empty()
                               ? defaultAddresses["current"].value("defaultSaveLocation", "")
                               : activePath1);
    savePath2 = trimString(activePath2.empty()
                               ? defaultAddresses["current"].value("defaultSaveLocation2", "")
                               : activePath2);
}

bool hasPathPrefix(const std::string& path, const std::string& basePath)
{
    std::string normalizedBase = trimString(basePath);
    if (normalizedBase.empty())
    {
        return false;
    }

    if (!normalizedBase.empty() && normalizedBase[normalizedBase.size() - 1] == '/')
    {
        normalizedBase.erase(normalizedBase.size() - 1);
    }

    if (path == normalizedBase)
    {
        return true;
    }

    return path.size() > normalizedBase.size() &&
           path.compare(0, normalizedBase.size(), normalizedBase) == 0 &&
           path[normalizedBase.size()] == '/';
}

bool resolveRecordingRelativePath(const std::string& relativePath, std::string& fullPath, std::string& channelName)
{
    std::string trimmed = trimString(relativePath);
    fullPath.clear();
    channelName.clear();

    if (trimmed.empty() || trimmed[0] == '/' || trimmed.find('\\') != std::string::npos ||
        trimmed.find("..") != std::string::npos || trimmed.find("//") != std::string::npos)
    {
        return false;
    }

    std::string savePath1;
    std::string savePath2;
    loadCurrentRecordingDirectories(savePath1, savePath2);

    std::string suffix;
    if (trimmed.compare(0, 8, "videos1/") == 0)
    {
        channelName = "videos1";
        suffix = trimmed.substr(8);
        fullPath = savePath1;
    }
    else if (trimmed.compare(0, 8, "videos2/") == 0)
    {
        channelName = "videos2";
        suffix = trimmed.substr(8);
        fullPath = savePath2;
    }
    else
    {
        return false;
    }

    suffix = trimString(suffix);
    if (fullPath.empty() || suffix.empty() || suffix[0] == '/')
    {
        return false;
    }

    fullPath += "/";
    fullPath += suffix;
    return hasPathPrefix(fullPath, channelName == "videos1" ? savePath1 : savePath2);
}

std::string deriveRecordingRelativePath(const std::string& filePath)
{
    std::string savePath1;
    std::string savePath2;
    loadCurrentRecordingDirectories(savePath1, savePath2);

    if (hasPathPrefix(filePath, savePath1))
    {
        std::string prefix = savePath1;
        if (!prefix.empty() && prefix[prefix.size() - 1] != '/')
        {
            prefix += "/";
        }
        return "videos1/" + filePath.substr(prefix.size());
    }

    if (hasPathPrefix(filePath, savePath2))
    {
        std::string prefix = savePath2;
        if (!prefix.empty() && prefix[prefix.size() - 1] != '/')
        {
            prefix += "/";
        }
        return "videos2/" + filePath.substr(prefix.size());
    }

    return "";
}

int sanitizeRecordingSegmentTime(int requestedValue)
{
    return clampInt(requestedValue <= 0 ? defaultSegmentTime : requestedValue, 60, 24 * 3600);
}

json buildRecordingTfcardJson()
{
    TFCardInfo tfCardInfo = getTFCardInfo();
    double usedGb = parseHumanSizeToGigabytes(tfCardInfo.usedMemory);
    double freeGb = parseHumanSizeToGigabytes(tfCardInfo.freeMemory);
    double totalGb = usedGb + freeGb;
    int usagePercent = totalGb > 0.0 ? static_cast<int>((usedGb / totalGb) * 100.0 + 0.5) : 0;

    json responseJson = {
        {"mountPath", tfCardInfo.mountPath},
        {"totalSpace", tfCardInfo.totalMemory.empty() ? formatByteSize(static_cast<long long>(totalGb * 1024.0 * 1024.0 * 1024.0)) : tfCardInfo.totalMemory},
        {"usedSpace", tfCardInfo.usedMemory.empty() ? "0G" : tfCardInfo.usedMemory},
        {"freeSpace", tfCardInfo.freeMemory.empty() ? "0G" : tfCardInfo.freeMemory},
        {"usagePercent", std::to_string(usagePercent) + "%"}
    };
    return responseJson;
}

json buildRecordingConfigJson()
{
    json defaultAddresses;
    readDefaultAddressesFromJson(defaultAddresses);
    ensureDefaultAddressSections(defaultAddresses);
    bool dualStreamEnabled = readDualStreamEnabledFromDefaultAddresses(defaultAddresses, true);

    json configJson = {
        {"rtsp_url_1", defaultAddresses["current"].value("defaultRtspStreamUrl", "")},
        {"rtsp_url_2", defaultAddresses["current"].value("defaultRtspStreamUrl2", "")},
        {"save_path_1", defaultAddresses["current"].value("defaultSaveLocation", "")},
        {"save_path_2", defaultAddresses["current"].value("defaultSaveLocation2", "")},
        {"segment_time", sanitizeRecordingSegmentTime(defaultSegmentTime)},
        {"dual_stream_enabled", dualStreamEnabled},
        {"dualStreamEnabled", dualStreamEnabled}
    };
    return configJson;
}

bool waitForProcessStartup(int pid, int maxWaitMs)
{
    if (pid <= 0)
    {
        return false;
    }

    int elapsedMs = 0;
    while (elapsedMs <= maxWaitMs)
    {
        if (isProcessRunning(pid))
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        elapsedMs += 100;
    }
    return false;
}

bool launchRecordingChannel(const std::string& streamUrl,
                            const std::string& saveLocation,
                            int segmentTime,
                            const std::string& logPath,
                            int& pidOut,
                            std::string& errorMessage)
{
    pidOut = 0;
    errorMessage.clear();

    const std::string kFfmpegPrimary = "/opt/sophon/sophon-ffmpeg-latest/bin/ffmpeg";
    std::string ffmpegBinary = pathExists(kFfmpegPrimary) ? kFfmpegPrimary : "ffmpeg";
    std::string outputPattern = saveLocation + "/%Y-%m-%d_%H-%M-%S.mp4";

    std::string innerCommand =
        "exec " + shellQuote(ffmpegBinary) +
        " -nostdin -loglevel error -rtsp_transport tcp -i " + shellQuote(streamUrl) +
        " -c:v copy -c:a aac -strict experimental -f segment -segment_time " +
        std::to_string(segmentTime) +
        " -reset_timestamps 1 -strftime 1 -segment_format mp4 " +
        shellQuote(outputPattern) +
        " > " + shellQuote(logPath) + " 2>&1";

    std::string launchCommand = "sh -c " + shellQuote(innerCommand) + " >/dev/null 2>&1 & echo $!";
    std::string output = trimString(executeCommand(launchCommand.c_str()));
    if (output.empty())
    {
        errorMessage = "启动 ffmpeg 失败，未获取到进程 PID";
        return false;
    }

    try
    {
        pidOut = std::stoi(output);
    }
    catch (...)
    {
        errorMessage = "解析 ffmpeg PID 失败: " + output;
        return false;
    }

    if (!waitForProcessStartup(pidOut, 1500))
    {
        errorMessage = "ffmpeg 进程未成功启动";
        return false;
    }

    return true;
}

bool isRecordingProcessActive(const std::string& pidFile, int* pidOut = nullptr)
{
    int pid = 0;
    if (!readPidFromFile(pidFile, pid))
    {
        return false;
    }

    if (!isProcessRunning(pid))
    {
        removeFileIfExists(pidFile);
        return false;
    }

    if (pidOut != nullptr)
    {
        *pidOut = pid;
    }
    return true;
}

void writeRecordingStatusSnapshot(bool recording1,
                                  bool recording2,
                                  long long startTimeSeconds,
                                  const std::string& lastError)
{
    json statusJson = readJsonObjectFile(RECORDING_STATUS_FILE);
    statusJson["recording_status"] = recording1 || recording2;
    statusJson["recording1"] = recording1;
    statusJson["recording2"] = recording2;
    statusJson["start_time"] = (recording1 || recording2) ? startTimeSeconds : 0;
    statusJson["last_error"] = lastError;
    writeJsonFile(RECORDING_STATUS_FILE, statusJson);
}

void refreshRecordingRuntimeState()
{
    bool recording1 = isRecordingProcessActive(RECORDING_PID_FILE_1);
    bool recording2 = isRecordingProcessActive(RECORDING_PID_FILE_2);
    json statusJson = readJsonObjectFile(RECORDING_STATUS_FILE);
    long long startTimeSeconds = statusJson.value("start_time", static_cast<long long>(0));
    if ((recording1 || recording2) && startTimeSeconds <= 0)
    {
        startTimeSeconds = static_cast<long long>(std::time(nullptr));
    }
    if (!recording1 && !recording2)
    {
        startTimeSeconds = 0;
        std::lock_guard<std::mutex> lock(recordingStateMutex);
        activeRecordingRtspUrl1.clear();
        activeRecordingRtspUrl2.clear();
        activeRecordingSaveLocation1.clear();
        activeRecordingSaveLocation2.clear();
    }

    isFfmpegRunning = recording1;
    isFfmpegRunning2 = recording2;
    if (startTimeSeconds > 0)
    {
        startRecordingTime = std::chrono::system_clock::from_time_t(static_cast<time_t>(startTimeSeconds));
    }
    else
    {
        startRecordingTime = std::chrono::system_clock::time_point();
    }
    recordingLastError = statusJson.value("last_error", "");
    writeRecordingStatusSnapshot(recording1, recording2, startTimeSeconds, recordingLastError);
}

std::vector<json> collectRecordingFiles()
{
    std::vector<json> files;
    std::string savePath1;
    std::string savePath2;
    loadCurrentRecordingDirectories(savePath1, savePath2);

    const std::pair<std::string, std::string> directories[] = {
        std::make_pair("videos1", savePath1),
        std::make_pair("videos2", savePath2)
    };

    time_t nowTs = std::time(nullptr);
    for (size_t dirIndex = 0; dirIndex < 2; ++dirIndex)
    {
        const std::string& channel = directories[dirIndex].first;
        const std::string& basePath = directories[dirIndex].second;
        if (basePath.empty() || !isDirectoryPath(basePath))
        {
            continue;
        }

        DIR* dir = opendir(basePath.c_str());
        if (dir == nullptr)
        {
            continue;
        }

        struct dirent* entry = nullptr;
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string fileName = entry->d_name;
            if (fileName.empty() || fileName == "." || fileName == ".." || !isSupportedRecordingMediaFile(fileName))
            {
                continue;
            }

            std::string fullPath = basePath + "/" + fileName;
            struct stat fileStat;
            if (stat(fullPath.c_str(), &fileStat) != 0 || !S_ISREG(fileStat.st_mode))
            {
                continue;
            }

            json item = {
                {"name", fileName},
                {"fullPath", fullPath},
                {"relativePath", channel + "/" + fileName},
                {"size", static_cast<long long>(fileStat.st_size)},
                {"sizeStr", formatByteSize(static_cast<long long>(fileStat.st_size))},
                {"modifyTime", static_cast<long long>(fileStat.st_mtime)},
                {"timeStr", formatLocalDateTime(fileStat.st_mtime)},
                {"channel", channel},
                {"isRecording", isSupportedRecordingVideoFile(fileName) &&
                                    (static_cast<long long>(nowTs - fileStat.st_mtime) < 5)}
            };
            files.push_back(item);
        }
        closedir(dir);
    }

    std::sort(files.begin(), files.end(),
              [](const json& left, const json& right)
              {
                  return left.value("modifyTime", static_cast<long long>(0)) >
                         right.value("modifyTime", static_cast<long long>(0));
              });
    return files;
}

json buildRecordingDiagnosisJson()
{
    refreshRecordingRuntimeState();

    int pid1 = 0;
    int pid2 = 0;
    bool recording1 = isRecordingProcessActive(RECORDING_PID_FILE_1, &pid1);
    bool recording2 = isRecordingProcessActive(RECORDING_PID_FILE_2, &pid2);

    std::string savePath1;
    std::string savePath2;
    loadCurrentRecordingDirectories(savePath1, savePath2);

    auto buildPathDiagnosis = [](const std::string& path)
    {
        json diag = {
            {"path", path},
            {"exists", isDirectoryPath(path)},
            {"writable", (!path.empty() && access(path.c_str(), W_OK | X_OK) == 0)},
            {"video_count", 0},
            {"files", json::array()}
        };

        if (!isDirectoryPath(path))
        {
            return diag;
        }

        DIR* dir = opendir(path.c_str());
        if (dir == nullptr)
        {
            return diag;
        }

        std::vector<std::string> fileNames;
        struct dirent* entry = nullptr;
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string fileName = entry->d_name;
            if (fileName.empty() || fileName == "." || fileName == ".." || !isSupportedRecordingVideoFile(fileName))
            {
                continue;
            }
            fileNames.push_back(fileName);
        }
        closedir(dir);

        std::sort(fileNames.begin(), fileNames.end());
        diag["video_count"] = static_cast<int>(fileNames.size());
        json sampleFiles = json::array();
        for (size_t i = 0; i < fileNames.size() && i < 5; ++i)
        {
            sampleFiles.push_back(fileNames[i]);
        }
        diag["files"] = sampleFiles;
        return diag;
    };

    json diagnosis = {
        {"path1", buildPathDiagnosis(savePath1)},
        {"path2", buildPathDiagnosis(savePath2)},
        {"recording_status", {
            {"recording1", recording1},
            {"recording2", recording2},
            {"pid1_exists", pathExists(RECORDING_PID_FILE_1)},
            {"pid2_exists", pathExists(RECORDING_PID_FILE_2)},
            {"pid1", pid1},
            {"pid2", pid2}
        }},
        {"ffmpeg_logs", {
            {"log1_exists", pathExists(RECORDING_LOG_FILE_1)},
            {"log2_exists", pathExists(RECORDING_LOG_FILE_2)},
            {"log1_tail", tailTextFile(RECORDING_LOG_FILE_1, 20)},
            {"log2_tail", tailTextFile(RECORDING_LOG_FILE_2, 20)}
        }}
    };
    return diagnosis;
}

struct AutoCleanupConfig
{
    bool enabled;
    double retentionDays;
};

AutoCleanupConfig loadAutoCleanupConfig()
{
    const bool defaultEnabled = true;
    const double defaultRetentionDays = 10.0;

    AutoCleanupConfig config = {defaultEnabled, defaultRetentionDays};
    json paramsJson = readJsonObjectFile("/data/lintech/celectronicfence/params.json");

    if (!paramsJson.is_object())
    {
        return config;
    }

    if (paramsJson.contains("AutoCleanupEnabled"))
    {
        config.enabled = parseJsonBoolLike(paramsJson["AutoCleanupEnabled"], defaultEnabled);
    }
    else if (paramsJson.contains("auto_cleanup_enabled"))
    {
        config.enabled = parseJsonBoolLike(paramsJson["auto_cleanup_enabled"], defaultEnabled);
    }

    const json* retentionNode = nullptr;
    if (paramsJson.contains("VideoRetentionDays"))
    {
        retentionNode = &paramsJson["VideoRetentionDays"];
    }
    else if (paramsJson.contains("video_retention_days"))
    {
        retentionNode = &paramsJson["video_retention_days"];
    }

    if (retentionNode != nullptr)
    {
        if (retentionNode->is_number())
        {
            config.retentionDays = retentionNode->get<double>();
        }
        else if (retentionNode->is_string())
        {
            std::string retentionText = trimString(retentionNode->get<std::string>());
            if (!retentionText.empty())
            {
                try
                {
                    config.retentionDays = std::stod(retentionText);
                }
                catch (...)
                {
                }
            }
        }
    }

    if (config.retentionDays <= 0.0 || config.retentionDays != config.retentionDays)
    {
        config.retentionDays = defaultRetentionDays;
    }

    config.retentionDays = clampDouble(config.retentionDays, 1.0, 365.0);
    return config;
}

//----------------------------------------------------------------------------------------------------------------------------
// 定时器线程函数，定期调用deleteOldestFiles函数
void timerThreadFunction()
{
    while (!stopTimerThread.load())
    {
        // 我们的TF卡挂载路径是硬编码的 "/mnt/tfcard"，可调整传入参数方式
        deleteOldestFiles("/mnt/tfcard");
        std::this_thread::sleep_for(std::chrono::seconds(180));
    }
    std::cout << "定时器线程已结束。" << std::endl;
}
//----------------------------------------------------------------------------------------------------------------------------
// 删除挂载路径下最早生成的文件（修改后的版本，基于时间判断来删除超过设定天数的文件）
void deleteOldestFiles(const std::string &mountPath)
{
    AutoCleanupConfig cleanupConfig = loadAutoCleanupConfig();
    if (!cleanupConfig.enabled)
    {
        std::cout << "[自动清理] 已禁用，跳过本轮清理。" << std::endl;
        return;
    }

    const double videoRetentionDays = cleanupConfig.retentionDays;
    std::vector<std::string> fileList;
    // 构建查找指定挂载路径下所有视频文件的命令
    // std::string listCommand = "sudo find " + mountPath + "/videos -type f";
    std::string listCommand = "sudo find " + mountPath + "/videos1 " + mountPath + "/videos2 -type f";
    // 执行命令获取文件列表
    std::string fileOutput = executeCommand(listCommand.c_str());
    char *token = strtok(const_cast<char *>(fileOutput.c_str()), "\n");
    while (token != nullptr)
    {
        fileList.push_back(token);
        token = strtok(nullptr, "\n");
    }

    // 比较函数，基于文件的最后修改时间判断是否超过保留天数
    auto compareFilesByTime = [](const std::string &file1, const std::string &file2)
    {
        struct stat fileStat1, fileStat2;
        if (stat(file1.c_str(), &fileStat1) < 0 || stat(file2.c_str(), &fileStat2) < 0)
        {
            std::cerr << "获取文件时间属性失败，可能影响排序" << std::endl;
            return file1 < file2;
        }
        // 将fileStat1.st_mtime和fileStat2.st_mtime转换为std::chrono::system_clock::time_point类型来计算时间差
        auto timePoint1 = std::chrono::system_clock::from_time_t(fileStat1.st_mtime);
        auto timePoint2 = std::chrono::system_clock::from_time_t(fileStat2.st_mtime);
        // 获取当前时间并转换为std::chrono::system_clock::time_point类型
        auto currentTime = std::chrono::system_clock::now();
        // 计算文件1距离当前时间的分钟数差，使用std::chrono库来更精确处理时间
        auto diff1_minutes = std::chrono::duration_cast<std::chrono::minutes>(currentTime - timePoint1).count();
        // 将分钟数差换算为天数差，并添加详细调试输出，这里改为double类型计算
        double diff1 = diff1_minutes / (24.0 * 60.0);
        std::cout << "文件1的时间戳(st_mtime)转换后的时间点（精确到纳秒）: " << std::chrono::duration_cast<std::chrono::nanoseconds>(timePoint1.time_since_epoch()).count() << " 纳秒" << std::endl;
        std::cout << "当前时间（精确到纳秒）: " << std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime.time_since_epoch()).count() << std::endl;
        std::cout << "文件1距离当前时间换算后的天数差: " << diff1 << ", 分钟数差: " << diff1_minutes << std::endl;
        // 计算文件2距离当前时间的分钟数差
        auto diff2_minutes = std::chrono::duration_cast<std::chrono::minutes>(currentTime - timePoint2).count();
        double diff2 = diff2_minutes / (24.0 * 60.0);
        std::cout << "文件2的时间戳(st_mtime)转换后的时间点（精确到纳秒）: " << std::chrono::duration_cast<std::chrono::nanoseconds>(timePoint2.time_since_epoch()).count() << " 纳秒" << std::endl;
        std::cout << "当前时间（精确到纳秒）: " << std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime.time_since_epoch()).count() << std::endl;
        std::cout << "文件2距离当前时间换算后的天数差: " << diff2 << ", 分钟数差: " << diff2_minutes << std::endl;
        return diff1 > diff2;
    };

    // 根据比较函数对文件列表进行排序，使得超过保留天数的文件排在前面
    std::sort(fileList.begin(), fileList.end(), compareFilesByTime);

    // 新增的逻辑：获取当前时间，用于和文件的时间差做比较
    // 获取当前时间并转换为std::chrono::system_clock::time_point类型
    auto currentTime = std::chrono::system_clock::now();
    while (!fileList.empty())
    {
        // 取出排在最前面的文件（当前认为是最旧的文件）
        std::string fileName = fileList.front();
        struct stat fileStat;
        if (stat(fileName.c_str(), &fileStat) < 0)
        {
            std::cerr << "获取文件时间属性失败，无法准确判断是否删除，文件名: " << fileName << std::endl;
            fileList.erase(fileList.begin());
            continue;
        }
        // 将文件的最后修改时间转换为可计算时间差的类型
        auto fileTimePoint = std::chrono::system_clock::from_time_t(fileStat.st_mtime);
        // 计算该文件距离当前时间的分钟数差，使用std::chrono库来更精确处理时间
        auto diff_minutes = std::chrono::duration_cast<std::chrono::minutes>(currentTime - fileTimePoint).count();
        // 将分钟数差换算为天数差，并添加详细调试输出，这里改为double类型计算
        double diff = diff_minutes / (24.0 * 60.0);
        std::cout << "正在判断文件 " << fileName << " 是否删除,文件时间戳(st_mtime)转换后的时间点（精确到纳秒）: " << std::chrono::duration_cast<std::chrono::nanoseconds>(fileTimePoint.time_since_epoch()).count() << " 纳秒" << std::endl;
        std::cout << "当前时间（精确到纳秒）: " << std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime.time_since_epoch()).count() << std::endl;
        std::cout << "文件 " << fileName << " 距离当前时间换算后的天数差: " << diff << ", 分钟数差: " << diff_minutes << std::endl;
        // 新增逻辑：只有当文件距离当前时间的天数差大于等于设定的阈值时，才执行删除操作，这里将diff显式转换为double类型进行比较
        if (diff >= videoRetentionDays)
        {
            std::cout << "文件 " << fileName << " 满足删除条件，准备删除" << std::endl;
            std::stringstream fileNamesToDelete;
            const int batchSize = 1; // 每次批量删除的文件数量，可调整
            for (int i = 0; i < batchSize && !fileList.empty(); ++i)
            {
                fileNamesToDelete << fileList.front() << " ";
                fileList.erase(fileList.begin());
            }

            std::string deleteCommand = "sudo rm " + fileNamesToDelete.str();
            int deleteResult = std::system(deleteCommand.c_str());
            if (deleteResult == 0)
            {
                std::cout << "已删除文件: " << fileName << std::endl;
                // 文件删除成功后，添加适当延迟，确保文件系统完成更新
                std::this_thread::sleep_for(std::chrono::seconds(3));
            }
            else
            {
                std::cerr << "删除文件 " << fileName << " 失败，错误码: " << deleteResult << ", 文件路径示例: " << fileNamesToDelete.str() << std::endl;
                // 将错误文件信息记录到日志文件
                std::ofstream errorLog("error.log", std::ios::app);
                if (errorLog.is_open())
                {
                    errorLog << "[" << std::time(nullptr) << "] 删除文件 " << fileName << " 失败: " << fileNamesToDelete.str() << ", 错误码: " << deleteResult << std::endl;
                    errorLog.close();
                }
                // 尝试最多3次重试删除文件
                int retryCount = 0;
                while (retryCount < 3)
                {
                    std::system(deleteCommand.c_str());
                    if (std::system(deleteCommand.c_str()) == 0)
                    {
                        break;
                    }
                    std::cerr << "重试删除文件 " << fileName << "，第 " << (retryCount + 1) << " 次失败" << std::endl;
                    retryCount++;
                }
                if (retryCount == 3)
                {
                    std::cerr << "多次重试后仍无法删除文件 " << fileName << "，放弃操作" << std::endl;
                }
            }
        }
        else
        {
            std::cout << "文件 " << fileName << " 未满足删除条件，跳过删除" << std::endl;
            // 如果文件未达到删除阈值，直接跳出循环，不再继续尝试删除其他文件
            break;
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------
// 用于标记ffmpeg是否正在运行，用于主线程内简单判断录制状态
bool stopVideoStreamProcessing = false;
bool isFfmpegRunning = false;
// 全局变量用于标记第二路ffmpeg是否正在运行
bool isFfmpegRunning2 = false;
namespace JsonFile1
{
    // 定义网络配置结构体
    struct NetworkConfig
    {
        std::string ipAddress;
        std::string subnetMask;
        std::string gateway;
        std::vector<std::string> dnsServers;
    };
    //----------------------------------------------------------------------------------------------------------------------------
    // 全局变量存储网络配置
    NetworkConfig currentConfig;
    // 解析 JSON 数据并更新网络配置

    //----------------------------------------------------------------------------------------------------------------------------
    void updateNetworkConfig(const json &j)
    {
        currentConfig.ipAddress = j.value("ipAddress", "");
        currentConfig.subnetMask = j.value("subnetMask", "");
        currentConfig.gateway = j.value("gateway", "");
        currentConfig.dnsServers = j.value("dnsServers", std::vector<std::string>());
    }
    //----------------------------------------------------------------------------------------------------------------------------
    // 应用网络配置到开发板
    void applyNetworkConfigToBoard()
    {
        std::ofstream interfacesFile("/etc/network/interfaces");
        if (interfacesFile.is_open())
        {
            interfacesFile << "auto eth0\n";
            interfacesFile << "iface eth0 inet static\n";
            interfacesFile << "address " << currentConfig.ipAddress << "\n";
            interfacesFile << "netmask " << currentConfig.subnetMask << "\n";
            interfacesFile << "gateway " << currentConfig.gateway << "\n";
            interfacesFile << "dns-nameservers " << currentConfig.dnsServers[0];
            for (size_t i = 1; i < currentConfig.dnsServers.size(); ++i)
            {
                interfacesFile << " " << currentConfig.dnsServers[i];
            }
            interfacesFile.close();

            // 通过执行命令来重启网络服务使配置生效
            std::system("sudo /etc/init.d/networking restart");
            // 等待网络服务启动完成（这里可以根据实际情况调整等待时间或进行网络连接测试）
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
        else
        {
            std::cerr << "Error opening /etc/network/interfaces for writing." << std::endl;
        }
    }
    //----------------------------------------------------------------------------------------------------------------------------
    // 处理网络配置保存的 POST 请求
    void handleSaveNetworkConfig(const Request &req, Response &res)
    {
        if (req.method == "POST" && req.path == "/save_network_config")
        {
            try
            {
                json reqJson = json::parse(req.body);
                updateNetworkConfig(reqJson);
                applyNetworkConfigToBoard();
                // 添加短暂延迟
                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                res.status = 200;
                res.set_content("Configuration saved successfully and applied to board.", "text/plain");
            }
            catch (const std::exception &e)
            {
                res.status = 500;
                res.set_content("Error saving configuration: " + std::string(e.what()), "text/plain");
            }
        }
        else
        {
            res.status = 404;
        }
    }
}

// ----------------------------------------------------------------------------

// 旧版全局 polygon/前端红框链路已废弃，只保留任务级区域配置。
std::mutex taskRegionFrameCaptureMutex;
std::atomic<unsigned long long> taskRegionFrameRequestSequence(0);

bool isRegisteredPersonTrackingAlgorithm(const std::string& algorithm)
{
    return algorithm == "registered_person_tracking";
}

bool isRegionIntrusionAlgorithm(const std::string& algorithm)
{
    return algorithm == "region_intrusion";
}

bool supportsTaskRegionConfigAlgorithm(const std::string& algorithm)
{
    return isRegisteredPersonTrackingAlgorithm(algorithm) ||
           isRegionIntrusionAlgorithm(algorithm);
}

bool isRtspLikeUrl(const std::string& url)
{
    return url.rfind("rtsp://", 0) == 0 || url.rfind("rtsps://", 0) == 0;
}

std::string detectStreamSourceType(const std::string& url)
{
    return isRtspLikeUrl(url) ? "RTSP" : "VIDEO";
}

json buildRegisteredPersonTrackingAreaPoints(int frameWidth, int frameHeight)
{
    return json::array({
        {{"left", 0}, {"top", 0}},
        {{"left", std::max(0, frameWidth - 1)}, {"top", 0}},
        {{"left", std::max(0, frameWidth - 1)}, {"top", std::max(0, frameHeight - 1)}},
        {{"left", 0}, {"top", std::max(0, frameHeight - 1)}}
    });
}

bool updatePolygonInFilterConfigFile(const std::string& configPath,
                                     const json& filterAreas,
                                     int channelId)
{
    std::ifstream in(configPath.c_str());
    if (!in.is_open())
    {
        return false;
    }

    json conf;
    try
    {
        in >> conf;
    }
    catch (...)
    {
        return false;
    }
    in.close();

    if (!conf.contains("configure") || !conf["configure"].is_object())
    {
        conf["configure"] = json::object();
    }

    json filterRule = {
        {"alert_first_frame", 0},
        {"alert_frame_skip_nums", 1},
        {"areas", filterAreas.is_array() && !filterAreas.empty()
            ? filterAreas
            : json::array({buildRegisteredPersonTrackingAreaPoints()})},
        {"classes", json::array({0})},
        {"polygon_mode", "any_overlap"},
        {"times", json::array({
            {
                {"time_start", "00 00 00"},
                {"time_end", "23 59 59"}
            }
        })},
        {"type", 1}
    };

    conf["configure"]["rules"] = json::array({
        {
            {"channel_id", channelId},
            {"filters", json::array({filterRule})}
        }
    });

    std::ofstream out(configPath.c_str(), std::ios::out | std::ios::trunc);
    if (!out.is_open())
    {
        return false;
    }
    out << conf.dump(4);
    return out.good();
}

bool loadJsonFile(const std::string& path, json& value, std::string& errorMessage)
{
    std::ifstream in(path.c_str());
    if (!in.is_open())
    {
        errorMessage = "Cannot open file: " + path;
        return false;
    }

    try
    {
        in >> value;
    }
    catch (const std::exception& e)
    {
        errorMessage = e.what();
        return false;
    }

    errorMessage.clear();
    return true;
}

bool loadTasksJson(json& tasksJson, std::string& errorMessage)
{
    if (!pathExists(TASKS_JSON_PATH))
    {
        tasksJson = json::array();
        errorMessage.clear();
        return true;
    }

    if (!loadJsonFile(TASKS_JSON_PATH, tasksJson, errorMessage))
    {
        return false;
    }

    if (!tasksJson.is_array())
    {
        tasksJson = json::array();
    }

    errorMessage.clear();
    return true;
}

bool saveTasksJson(const json& tasksJson, std::string& errorMessage)
{
    if (!writeJsonFile(TASKS_JSON_PATH, tasksJson))
    {
        errorMessage = "Cannot write tasks.json";
        return false;
    }

    errorMessage.clear();
    return true;
}

bool loadChannelsJson(json& channelsJson, std::string& errorMessage)
{
    if (!pathExists(CHANNELS_JSON_PATH))
    {
        channelsJson["channels"] = json::array();
        errorMessage.clear();
        return true;
    }

    if (!loadJsonFile(CHANNELS_JSON_PATH, channelsJson, errorMessage))
    {
        return false;
    }

    if (!channelsJson.contains("channels") || !channelsJson["channels"].is_array())
    {
        channelsJson["channels"] = json::array();
    }

    errorMessage.clear();
    return true;
}

json* findTaskById(json& tasksJson, int taskId)
{
    if (!tasksJson.is_array())
    {
        return NULL;
    }

    for (auto& task : tasksJson)
    {
        if (task.is_object() && task.value("id", 0) == taskId)
        {
            return &task;
        }
    }

    return NULL;
}

const json* findTaskById(const json& tasksJson, int taskId)
{
    if (!tasksJson.is_array())
    {
        return NULL;
    }

    for (auto it = tasksJson.begin(); it != tasksJson.end(); ++it)
    {
        if (it->is_object() && it->value("id", 0) == taskId)
        {
            return &(*it);
        }
    }

    return NULL;
}

const json* findChannelById(const json& channelsJson, int channelId)
{
    if (!channelsJson.is_object() ||
        !channelsJson.contains("channels") ||
        !channelsJson["channels"].is_array())
    {
        return NULL;
    }

    for (auto it = channelsJson["channels"].begin(); it != channelsJson["channels"].end(); ++it)
    {
        if (it->is_object() && it->value("id", 0) == channelId)
        {
            return &(*it);
        }
    }

    return NULL;
}

int parseJsonInt(const json& value, int fallbackValue)
{
    if (value.is_number_integer())
    {
        return value.get<int>();
    }
    if (value.is_number_unsigned())
    {
        return static_cast<int>(value.get<unsigned int>());
    }
    if (value.is_number_float())
    {
        return static_cast<int>(std::lround(value.get<double>()));
    }
    if (value.is_string())
    {
        try
        {
            return std::stoi(trimString(value.get<std::string>()));
        }
        catch (...)
        {
        }
    }
    return fallbackValue;
}

std::string generateTaskRegionId(int taskId, size_t index)
{
    return "task_" + std::to_string(taskId) +
           "_region_" + std::to_string(currentTimeMillis()) +
           "_" + std::to_string(static_cast<unsigned long long>(index));
}

bool isSupportedTaskRegionType(const std::string& regionType)
{
    return regionType == "interest" || regionType == "exclusive";
}

bool sanitizeTaskRegionConfigs(const json& rawRegions,
                               const json& task,
                               json& sanitizedRegions,
                               std::string& errorMessage)
{
    if (!rawRegions.is_array())
    {
        errorMessage = "regions must be an array";
        return false;
    }

    sanitizedRegions = json::array();
    long long nowMs = currentTimeMillis();
    int taskId = task.value("id", 0);
    std::string ownerTaskNumber = trimString(task.value("taskNumber", ""));
    std::string ownerAlgorithm = trimString(task.value("algorithm", ""));
    for (size_t i = 0; i < rawRegions.size(); ++i)
    {
        const json& region = rawRegions[i];
        if (!region.is_object())
        {
            errorMessage = "Each region must be an object";
            return false;
        }

        std::string label = trimString(region.value("label", ""));
        if (label.empty())
        {
            errorMessage = "Region label is required";
            return false;
        }

        std::string regionType = trimString(region.value("regionType", ""));
        if (!isSupportedTaskRegionType(regionType))
        {
            errorMessage = "Region type must be interest or exclusive";
            return false;
        }

        std::string shape = trimString(region.value("shape", "polygon"));
        if (shape != "polygon")
        {
            errorMessage = "Only polygon region is supported";
            return false;
        }

        int frameWidth = clampInt(
            region.contains("frameWidth") ? parseJsonInt(region["frameWidth"]) : 0,
            0,
            16384);
        int frameHeight = clampInt(
            region.contains("frameHeight") ? parseJsonInt(region["frameHeight"]) : 0,
            0,
            16384);
        if (frameWidth <= 0 || frameHeight <= 0)
        {
            errorMessage = "frameWidth and frameHeight must be positive";
            return false;
        }

        if (!region.contains("points") || !region["points"].is_array() || region["points"].size() < 3)
        {
            errorMessage = "Polygon region must have at least three points";
            return false;
        }

        json sanitizedPoints = json::array();
        for (size_t pointIndex = 0; pointIndex < region["points"].size(); ++pointIndex)
        {
            const json& pointJson = region["points"][pointIndex];
            if (!pointJson.is_object() ||
                !pointJson.contains("x") ||
                !pointJson.contains("y"))
            {
                errorMessage = "Region point is invalid";
                return false;
            }

            int x = clampInt(parseJsonInt(pointJson["x"]), 0, std::max(0, frameWidth - 1));
            int y = clampInt(parseJsonInt(pointJson["y"]), 0, std::max(0, frameHeight - 1));
            sanitizedPoints.push_back({
                {"x", x},
                {"y", y}
            });
        }

        std::string id = trimString(region.value("id", ""));
        if (id.empty())
        {
            id = generateTaskRegionId(taskId, i);
        }

        long long createdAtMs = region.value("createdAtMs", nowMs);
        if (createdAtMs <= 0)
        {
            createdAtMs = nowMs;
        }

        sanitizedRegions.push_back({
            {"id", id},
            {"label", label},
            {"regionType", regionType},
            {"shape", "polygon"},
            {"ownerTaskId", taskId},
            {"ownerTaskNumber", ownerTaskNumber},
            {"ownerAlgorithm", ownerAlgorithm},
            {"frameWidth", frameWidth},
            {"frameHeight", frameHeight},
            {"points", sanitizedPoints},
            {"createdAtMs", createdAtMs},
            {"updatedAtMs", nowMs}
        });
    }

    errorMessage.clear();
    return true;
}

json buildTaskRegionTaskSummary(const json& task)
{
    return {
        {"id", task.value("id", 0)},
        {"taskNumber", task.value("taskNumber", "")},
        {"algorithm", task.value("algorithm", "")},
        {"status", task.value("status", "stopped")},
        {"videoSourceId", task.value("videoSourceId", 0)},
        {"editable", supportsTaskRegionConfigAlgorithm(task.value("algorithm", ""))}
    };
}

bool readBinaryFile(const std::string& path, std::string& content)
{
    std::ifstream in(path.c_str(), std::ios::in | std::ios::binary);
    if (!in.is_open())
    {
        return false;
    }

    std::ostringstream oss;
    oss << in.rdbuf();
    content = oss.str();
    return in.good() || in.eof();
}

bool probeStreamFrameSize(const std::string& streamUrl, int& width, int& height)
{
    width = 0;
    height = 0;

    std::string url = trimString(streamUrl);
    if (url.empty())
    {
        return false;
    }

    const std::string kFfprobePrimary = "/opt/sophon/sophon-ffmpeg-latest/bin/ffprobe";
    std::string ffprobeBinary = pathExists(kFfprobePrimary) ? kFfprobePrimary : "ffprobe";
    std::string timeoutPrefix;
    if (pathExists("/usr/bin/timeout"))
    {
        timeoutPrefix = "/usr/bin/timeout 5 ";
    }
    else if (pathExists("/bin/timeout"))
    {
        timeoutPrefix = "/bin/timeout 5 ";
    }

    std::vector<std::string> candidateUrls;
    candidateUrls.push_back(url);
    std::string noTrailingSlashUrl = url;
    while (noTrailingSlashUrl.size() > 8 &&
           noTrailingSlashUrl[noTrailingSlashUrl.size() - 1] == '/')
    {
        noTrailingSlashUrl.erase(noTrailingSlashUrl.size() - 1);
    }
    if (noTrailingSlashUrl != url)
    {
        candidateUrls.push_back(noTrailingSlashUrl);
    }

    for (size_t i = 0; i < candidateUrls.size(); ++i)
    {
        std::string command =
            timeoutPrefix + shellQuote(ffprobeBinary) +
            " -v error -select_streams v:0 -show_entries stream=width,height " +
            " -of csv=p=0:s=x " + shellQuote(candidateUrls[i]) +
            " 2>/dev/null";
        std::string output = trimString(executeCommand(command.c_str()));
        if (output.empty())
        {
            continue;
        }

        size_t xPos = output.find('x');
        if (xPos == std::string::npos)
        {
            continue;
        }

        try
        {
            int parsedWidth = std::stoi(output.substr(0, xPos));
            int parsedHeight = std::stoi(output.substr(xPos + 1));
            if (parsedWidth > 0 && parsedHeight > 0)
            {
                width = parsedWidth;
                height = parsedHeight;
                return true;
            }
        }
        catch (...)
        {
        }
    }

    return false;
}

json scaleTaskRegionPointsToFilterArea(const json& region,
                                       int targetFrameWidth,
                                       int targetFrameHeight)
{
    int sourceFrameWidth = clampInt(
        region.contains("frameWidth") ? parseJsonInt(region["frameWidth"], targetFrameWidth) : targetFrameWidth,
        1,
        16384);
    int sourceFrameHeight = clampInt(
        region.contains("frameHeight") ? parseJsonInt(region["frameHeight"], targetFrameHeight) : targetFrameHeight,
        1,
        16384);

    double scaleX = static_cast<double>(targetFrameWidth) / static_cast<double>(sourceFrameWidth);
    double scaleY = static_cast<double>(targetFrameHeight) / static_cast<double>(sourceFrameHeight);

    json points = json::array();
    if (!region.contains("points") || !region["points"].is_array())
    {
        return points;
    }

    for (size_t i = 0; i < region["points"].size(); ++i)
    {
        const json& pointJson = region["points"][i];
        int sourceX = pointJson.contains("x") ? parseJsonInt(pointJson["x"], 0) : 0;
        int sourceY = pointJson.contains("y") ? parseJsonInt(pointJson["y"], 0) : 0;
        int scaledX = clampInt(
            static_cast<int>(std::lround(static_cast<double>(sourceX) * scaleX)),
            0,
            std::max(0, targetFrameWidth - 1));
        int scaledY = clampInt(
            static_cast<int>(std::lround(static_cast<double>(sourceY) * scaleY)),
            0,
            std::max(0, targetFrameHeight - 1));
        points.push_back({
            {"left", scaledX},
            {"top", scaledY}
        });
    }

    return points;
}

json buildTaskRegionFilterAreas(const json& regionConfigs,
                                int targetFrameWidth,
                                int targetFrameHeight,
                                bool fallbackToLegacyPolygon)
{
    json filterAreas = json::array();
    if (regionConfigs.is_array())
    {
        for (size_t i = 0; i < regionConfigs.size(); ++i)
        {
            const json& region = regionConfigs[i];
            if (!region.is_object() ||
                region.value("shape", "") != "polygon" ||
                !region.contains("points") ||
                !region["points"].is_array() ||
                region["points"].size() < 3)
            {
                continue;
            }

            json areaPoints = scaleTaskRegionPointsToFilterArea(
                region, targetFrameWidth, targetFrameHeight);
            if (areaPoints.size() >= 3)
            {
                filterAreas.push_back(areaPoints);
            }
        }
    }

    if (filterAreas.empty() && fallbackToLegacyPolygon)
    {
        filterAreas.push_back(buildRegisteredPersonTrackingAreaPoints(
            targetFrameWidth, targetFrameHeight));
    }

    return filterAreas;
}

json buildRegisteredPersonTrackingFilterAreas(const json& regionConfigs,
                                              int targetFrameWidth,
                                              int targetFrameHeight)
{
    return buildTaskRegionFilterAreas(
        regionConfigs, targetFrameWidth, targetFrameHeight, true);
}

json buildTaskRegionOverlayRegions(const json& regionConfigs,
                                   int targetFrameWidth,
                                   int targetFrameHeight,
                                   bool fallbackToLegacyPolygon)
{
    json overlayRegions = json::array();
    if (regionConfigs.is_array())
    {
        for (size_t i = 0; i < regionConfigs.size(); ++i)
        {
            const json& region = regionConfigs[i];
            if (!region.is_object() ||
                region.value("shape", "") != "polygon" ||
                !region.contains("points") ||
                !region["points"].is_array() ||
                region["points"].size() < 3)
            {
                continue;
            }

            json overlayPoints = scaleTaskRegionPointsToFilterArea(
                region, targetFrameWidth, targetFrameHeight);
            if (overlayPoints.size() < 3)
            {
                continue;
            }

            overlayRegions.push_back({
                {"label", trimString(region.value("label", ""))},
                {"points", overlayPoints}
            });
        }
    }

    if (overlayRegions.empty() && fallbackToLegacyPolygon)
    {
        overlayRegions.push_back({
            {"label", ""},
            {"points", buildRegisteredPersonTrackingAreaPoints(
                targetFrameWidth, targetFrameHeight)}
        });
    }

    return overlayRegions;
}

json buildRegisteredPersonTrackingOverlayRegions(const json& regionConfigs,
                                                 int targetFrameWidth,
                                                 int targetFrameHeight)
{
    return buildTaskRegionOverlayRegions(
        regionConfigs, targetFrameWidth, targetFrameHeight, true);
}

bool computeRoiRectFromAreaPoints(const json& areas,
                                  int frameWidth,
                                  int frameHeight,
                                  RoiRect& roi)
{
    if (!areas.is_array() || frameWidth <= 0 || frameHeight <= 0)
    {
        return false;
    }

    bool hasPoint = false;
    int minX = 0;
    int maxX = 0;
    int minY = 0;
    int maxY = 0;

    for (size_t areaIndex = 0; areaIndex < areas.size(); ++areaIndex)
    {
        const json& area = areas[areaIndex];
        if (!area.is_array())
        {
            continue;
        }

        for (size_t pointIndex = 0; pointIndex < area.size(); ++pointIndex)
        {
            const json& pointJson = area[pointIndex];
            if (!pointJson.is_object())
            {
                continue;
            }

            int x = clampInt(
                pointJson.contains("left") ? parseJsonInt(pointJson["left"]) : 0,
                0,
                std::max(0, frameWidth - 1));
            int y = clampInt(
                pointJson.contains("top") ? parseJsonInt(pointJson["top"]) : 0,
                0,
                std::max(0, frameHeight - 1));
            if (!hasPoint)
            {
                minX = maxX = x;
                minY = maxY = y;
                hasPoint = true;
                continue;
            }
            minX = std::min(minX, x);
            maxX = std::max(maxX, x);
            minY = std::min(minY, y);
            maxY = std::max(maxY, y);
        }
    }

    if (!hasPoint)
    {
        return false;
    }

    if (maxX <= minX)
    {
        minX = clampInt(minX, 0, std::max(0, frameWidth - 1));
        maxX = std::min(frameWidth, minX + 1);
    }
    else
    {
        maxX = std::min(frameWidth, maxX + 1);
    }
    if (maxY <= minY)
    {
        minY = clampInt(minY, 0, std::max(0, frameHeight - 1));
        maxY = std::min(frameHeight, minY + 1);
    }
    else
    {
        maxY = std::min(frameHeight, maxY + 1);
    }

    roi.left = clampInt(minX, 0, std::max(0, frameWidth - 1));
    roi.top = clampInt(minY, 0, std::max(0, frameHeight - 1));
    roi.width = std::max(1, maxX - roi.left);
    roi.height = std::max(1, maxY - roi.top);
    return true;
}

void ensureYolov8RegionFilterGraph(json& graph,
                                   const std::string& runtimeFilterConfigPath)
{
    if (!graph.is_object())
    {
        return;
    }

    if (!graph.contains("elements") || !graph["elements"].is_array())
    {
        graph["elements"] = json::array();
    }
    if (!graph.contains("connections") || !graph["connections"].is_array())
    {
        graph["connections"] = json::array();
    }

    bool hasFilterElement = false;
    for (auto& element : graph["elements"])
    {
        if (element.is_object() && element.value("element_id", -1) == 5006)
        {
            element["element_config"] = runtimeFilterConfigPath;
            hasFilterElement = true;
            break;
        }
    }

    if (!hasFilterElement)
    {
        graph["elements"].push_back({
            {"element_id", 5006},
            {"element_config", runtimeFilterConfigPath}
        });
    }

    json rewrittenConnections = json::array();
    bool hasBytetrackToFilter = false;
    bool hasFilterToOsd = false;
    std::set<std::string> connectionKeys;

    auto appendConnection = [&](const json& connection) {
        if (!connection.is_object())
        {
            return;
        }

        std::string key =
            std::to_string(connection.value("src_element_id", -1)) + ":" +
            std::to_string(connection.value("src_port", -1)) + "->" +
            std::to_string(connection.value("dst_element_id", -1)) + ":" +
            std::to_string(connection.value("dst_port", -1));
        if (!connectionKeys.insert(key).second)
        {
            return;
        }

        if (connection.value("src_element_id", -1) == 5004 &&
            connection.value("dst_element_id", -1) == 5006)
        {
            hasBytetrackToFilter = true;
        }
        if (connection.value("src_element_id", -1) == 5006 &&
            connection.value("dst_element_id", -1) == 5005)
        {
            hasFilterToOsd = true;
        }

        rewrittenConnections.push_back(connection);
    };

    for (auto& connection : graph["connections"])
    {
        if (!connection.is_object())
        {
            continue;
        }

        if (connection.value("src_element_id", -1) == 5004 &&
            connection.value("dst_element_id", -1) == 5005)
        {
            json rewritten = connection;
            rewritten["src_element_id"] = 5006;
            rewritten["src_port"] = 0;
            appendConnection(rewritten);
            continue;
        }

        appendConnection(connection);
    }

    if (!hasBytetrackToFilter)
    {
        appendConnection({
            {"src_element_id", 5004},
            {"src_port", 0},
            {"dst_element_id", 5006},
            {"dst_port", 0}
        });
    }
    if (!hasFilterToOsd)
    {
        appendConnection({
            {"src_element_id", 5006},
            {"src_port", 0},
            {"dst_element_id", 5005},
            {"dst_port", 0}
        });
    }

    graph["connections"] = rewrittenConnections;
}

void ensureRegisteredPersonTrackingFilterGraph(json& graph,
                                               const std::string& runtimeFilterConfigPath)
{
    if (!graph.is_object())
    {
        return;
    }

    if (!graph.contains("elements") || !graph["elements"].is_array())
    {
        graph["elements"] = json::array();
    }
    if (!graph.contains("connections") || !graph["connections"].is_array())
    {
        graph["connections"] = json::array();
    }

    bool hasFilterElement = false;
    for (auto& element : graph["elements"])
    {
        if (element.is_object() && element.value("element_id", -1) == 5005)
        {
            element["element_config"] = runtimeFilterConfigPath;
            hasFilterElement = true;
            break;
        }
    }

    if (!hasFilterElement)
    {
        graph["elements"].push_back({
            {"element_id", 5005},
            {"element_config", runtimeFilterConfigPath}
        });
    }

    json rewrittenConnections = json::array();
    bool hasBytetrackToFilter = false;
    bool hasFilterToMatcher = false;
    bool hasFilterToStatusAttach = false;
    std::set<std::string> connectionKeys;

    auto appendConnection = [&](const json& connection) {
        if (!connection.is_object())
        {
            return;
        }

        std::string key =
            std::to_string(connection.value("src_element_id", -1)) + ":" +
            std::to_string(connection.value("src_port", -1)) + "->" +
            std::to_string(connection.value("dst_element_id", -1)) + ":" +
            std::to_string(connection.value("dst_port", -1));
        if (!connectionKeys.insert(key).second)
        {
            return;
        }

        if (connection.value("src_element_id", -1) == 5002 &&
            connection.value("dst_element_id", -1) == 5005)
        {
            hasBytetrackToFilter = true;
        }
        if (connection.value("src_element_id", -1) == 5005 &&
            connection.value("dst_element_id", -1) == 5023)
        {
            hasFilterToMatcher = true;
        }
        if (connection.value("src_element_id", -1) == 5005 &&
            connection.value("dst_element_id", -1) == 5015)
        {
            hasFilterToStatusAttach = true;
        }

        rewrittenConnections.push_back(connection);
    };

    for (auto& connection : graph["connections"])
    {
        if (!connection.is_object())
        {
            continue;
        }

        int srcElementId = connection.value("src_element_id", -1);
        int dstElementId = connection.value("dst_element_id", -1);
        if (srcElementId == 5002 && (dstElementId == 5023 || dstElementId == 5015))
        {
            json rewritten = connection;
            rewritten["src_element_id"] = 5005;
            rewritten["src_port"] = 0;
            appendConnection(rewritten);
            continue;
        }

        appendConnection(connection);
    }

    if (!hasBytetrackToFilter)
    {
        appendConnection({
            {"src_element_id", 5002},
            {"src_port", 0},
            {"dst_element_id", 5005},
            {"dst_port", 0}
        });
    }
    if (!hasFilterToMatcher)
    {
        appendConnection({
            {"src_element_id", 5005},
            {"src_port", 0},
            {"dst_element_id", 5023},
            {"dst_port", 0}
        });
    }
    if (!hasFilterToStatusAttach)
    {
        appendConnection({
            {"src_element_id", 5005},
            {"src_port", 0},
            {"dst_element_id", 5015},
            {"dst_port", 0}
        });
    }

    graph["connections"] = rewrittenConnections;
}

bool shouldCountRegisteredPersonDetectionInCurrentPolygon(const json& detectedObject)
{
    (void)detectedObject;
    return true;
}

bool extractRegisteredPersonTrackingBox(const json& detectedObject,
                                        int& left,
                                        int& top,
                                        int& width,
                                        int& height)
{
    left = 0;
    top = 0;
    width = 0;
    height = 0;
    if (!detectedObject.is_object() ||
        !detectedObject.contains("mBox") ||
        !detectedObject["mBox"].is_object())
    {
        return false;
    }

    const json& box = detectedObject["mBox"];
    left = box.value("mX", 0);
    top = box.value("mY", 0);
    width = box.value("mWidth", 0);
    height = box.value("mHeight", 0);
    return width > 0 && height > 0;
}

void stopRegisteredPersonTrackingProcessesByTaskId(int taskId, int skipPid, const std::string& taskLabel)
{
    if (taskId <= 0)
    {
        return;
    }

    std::string pattern =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR +
        "/registered_person_tracking_runtime_task_" + std::to_string(taskId) + ".json";
    std::vector<int> candidatePids = listPidsByPattern(pattern);
    if (candidatePids.empty())
    {
        return;
    }

    std::sort(candidatePids.begin(), candidatePids.end());
    candidatePids.erase(std::unique(candidatePids.begin(), candidatePids.end()), candidatePids.end());

    for (size_t i = 0; i < candidatePids.size(); ++i)
    {
        int pid = candidatePids[i];
        if (pid <= 0 || pid == skipPid || !isProcessRunning(pid))
        {
            continue;
        }
        stopManagedProcess(pid, taskLabel);
    }
}

bool prepareRegisteredPersonTrackingRuntimeConfigs(int taskId,
                                                   int streamChannelId,
                                                   const std::string& channelUrl,
                                                   int sampleInterval,
                                                   const std::string& sampleStrategyRaw,
                                                   int targetEncodeFps,
                                                   int recognitionFrameInterval,
                                                   bool lowLatencyMode,
                                                   double trackingDetectionThreshold,
                                                   double faceSimilarityThreshold,
                                                   const json& regionConfigs,
                                                   std::string& runtimeDemoConfigPath,
                                                   std::string& runtimeFilterConfigPath,
                                                   std::string& errorMessage)
{
    sampleInterval = clampInt(sampleInterval, 1, 12);
    std::string sampleStrategy = trimString(sampleStrategyRaw);
    if (sampleStrategy != "KEEP" && sampleStrategy != "DROP")
    {
        sampleStrategy = "DROP";
    }
    targetEncodeFps = clampInt(targetEncodeFps, 1, 60);
    recognitionFrameInterval = clampInt(recognitionFrameInterval, 1, 12);
    const int kRegisteredTrackHttpPushElementId = 5011;
    const int kRegisteredTrackOsdElementId = 5012;

    const std::string sourceDemoConfigPath =
        REGISTERED_PERSON_TRACKING_PROJECT_ROOT + "/config/registered_person_tracking.json";
    const std::string sourceFilterConfigPath =
        REGISTERED_PERSON_TRACKING_PROJECT_ROOT + "/config/filter.json";
    const std::string sourceHttpPushConfigPath =
        REGISTERED_PERSON_TRACKING_PROJECT_ROOT + "/config/http_push.json";

    if (!ensureDirectory(REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR))
    {
        errorMessage = "Failed to create registered person tracking runtime config directory";
        return false;
    }

    std::ifstream demoConfigIn(sourceDemoConfigPath.c_str());
    if (!demoConfigIn.is_open())
    {
        errorMessage = "Registered person tracking demo config not found";
        return false;
    }

    json demoConfigJson;
    try
    {
        demoConfigIn >> demoConfigJson;
    }
    catch (const std::exception&)
    {
        errorMessage = "Registered person tracking demo config is invalid";
        return false;
    }
    demoConfigIn.close();

    std::string engineConfigPath = resolveConfigPath(
        SOPHON_STREAM_SAMPLES_BUILD_DIR,
        demoConfigJson.value("engine_config_path", "")
    );
    if (engineConfigPath.empty())
    {
        errorMessage = "Registered person tracking engine config path is missing";
        return false;
    }

    std::ifstream engineConfigIn(engineConfigPath.c_str());
    if (!engineConfigIn.is_open())
    {
        errorMessage = "Registered person tracking engine config not found";
        return false;
    }

    json engineConfigJson;
    try
    {
        engineConfigIn >> engineConfigJson;
    }
    catch (const std::exception&)
    {
        errorMessage = "Registered person tracking engine config is invalid";
        return false;
    }
    engineConfigIn.close();

    if (!engineConfigJson.is_array() || engineConfigJson.empty() || !engineConfigJson[0].is_object())
    {
        errorMessage = "Registered person tracking engine config format is invalid";
        return false;
    }

    json& graph = engineConfigJson[0];
    if (!graph.contains("elements") || !graph["elements"].is_array())
    {
        errorMessage = "Registered person tracking graph is missing elements";
        return false;
    }

    json filterConfigJson;
    if (!loadJsonFile(sourceFilterConfigPath, filterConfigJson, errorMessage))
    {
        errorMessage = "Registered person tracking filter config not found or invalid";
        return false;
    }

    std::string distributorConfigPath;
    std::string encodeConfigPath;
    std::string trackOsdConfigPath;
    std::string trackStatusAttachConfigPath;
    std::string faceTrackMatcherConfigPath;
    std::string yoloxConfigPath;
    std::string bytetrackConfigPath;
    std::string faissConfigPath;
    std::string debugDistributorConfigPath;
    std::string httpPushConfigPath;
    bool hasHttpPushElement = false;
    for (auto& element : graph["elements"])
    {
        std::string elementConfigPath = resolveConfigPath(
            SOPHON_STREAM_SAMPLES_BUILD_DIR,
            element.value("element_config", "")
        );
        if (!elementConfigPath.empty())
        {
            element["element_config"] = elementConfigPath;
        }

        int elementId = element.value("element_id", -1);
        if (elementId == 5001)
        {
            yoloxConfigPath = elementConfigPath;
        }
        else if (elementId == 5002)
        {
            bytetrackConfigPath = elementConfigPath;
        }
        else if (elementId == 5017)
        {
            distributorConfigPath = elementConfigPath;
        }
        else if (elementId == 5019)
        {
            faissConfigPath = elementConfigPath;
        }
        else if (elementId == 5012)
        {
            trackOsdConfigPath = elementConfigPath;
        }
        else if (elementId == 5015)
        {
            trackStatusAttachConfigPath = elementConfigPath;
        }
        else if (elementId == 5023)
        {
            faceTrackMatcherConfigPath = elementConfigPath;
        }
        else if (elementId == 5022)
        {
            debugDistributorConfigPath = elementConfigPath;
        }
        else if (elementId == 5014)
        {
            encodeConfigPath = elementConfigPath;
        }
        else if (elementId == kRegisteredTrackHttpPushElementId)
        {
            httpPushConfigPath = elementConfigPath;
            hasHttpPushElement = true;
        }
    }

    if (yoloxConfigPath.empty() || bytetrackConfigPath.empty() ||
        distributorConfigPath.empty() ||
        faissConfigPath.empty() || trackOsdConfigPath.empty() ||
        trackStatusAttachConfigPath.empty() ||
        faceTrackMatcherConfigPath.empty() || debugDistributorConfigPath.empty() ||
        encodeConfigPath.empty())
    {
        errorMessage = "Registered person tracking graph is missing runtime element configs";
        return false;
    }
    if (httpPushConfigPath.empty())
    {
        httpPushConfigPath = sourceHttpPushConfigPath;
    }
    if (!pathExists(httpPushConfigPath))
    {
        errorMessage = "Registered person tracking http push config is missing";
        return false;
    }

    std::ifstream yoloxConfigIn(yoloxConfigPath.c_str());
    std::ifstream bytetrackConfigIn(bytetrackConfigPath.c_str());
    std::ifstream distributorConfigIn(distributorConfigPath.c_str());
    std::ifstream faissConfigIn(faissConfigPath.c_str());
    std::ifstream trackOsdConfigIn(trackOsdConfigPath.c_str());
    std::ifstream trackStatusAttachConfigIn(trackStatusAttachConfigPath.c_str());
    std::ifstream faceTrackMatcherConfigIn(faceTrackMatcherConfigPath.c_str());
    std::ifstream debugDistributorConfigIn(debugDistributorConfigPath.c_str());
    std::ifstream encodeConfigIn(encodeConfigPath.c_str());
    std::ifstream httpPushConfigIn(httpPushConfigPath.c_str());
    if (!yoloxConfigIn.is_open() || !bytetrackConfigIn.is_open() ||
        !distributorConfigIn.is_open() ||
        !faissConfigIn.is_open() || !trackOsdConfigIn.is_open() ||
        !trackStatusAttachConfigIn.is_open() ||
        !faceTrackMatcherConfigIn.is_open() || !debugDistributorConfigIn.is_open() ||
        !encodeConfigIn.is_open() || !httpPushConfigIn.is_open())
    {
        errorMessage = "Registered person tracking element config is missing";
        return false;
    }

    json yoloxConfigJson;
    json bytetrackConfigJson;
    json distributorConfigJson;
    json faissConfigJson;
    json trackOsdConfigJson;
    json trackStatusAttachConfigJson;
    json faceTrackMatcherConfigJson;
    json debugDistributorConfigJson;
    json encodeConfigJson;
    json httpPushConfigJson;
    try
    {
        yoloxConfigIn >> yoloxConfigJson;
        bytetrackConfigIn >> bytetrackConfigJson;
        distributorConfigIn >> distributorConfigJson;
        faissConfigIn >> faissConfigJson;
        trackOsdConfigIn >> trackOsdConfigJson;
        trackStatusAttachConfigIn >> trackStatusAttachConfigJson;
        faceTrackMatcherConfigIn >> faceTrackMatcherConfigJson;
        debugDistributorConfigIn >> debugDistributorConfigJson;
        encodeConfigIn >> encodeConfigJson;
        httpPushConfigIn >> httpPushConfigJson;
    }
    catch (const std::exception&)
    {
        errorMessage = "Registered person tracking element config is invalid";
        return false;
    }

    trackingDetectionThreshold = clampDouble(
        trackingDetectionThreshold,
        0.05,
        0.95
    );
    faceSimilarityThreshold = clampDouble(faceSimilarityThreshold, 0.05, 0.95);
    const int registeredPersonTrackingStatusLagMinMs = 1200;
    const int registeredPersonTrackingStatusLagMaxMs = 4000;
    const int registeredPersonTrackingStatusLagPerRecognitionFrameMs = 600;
    const int statusLagMs = clampInt(
        recognitionFrameInterval * registeredPersonTrackingStatusLagPerRecognitionFrameMs,
        registeredPersonTrackingStatusLagMinMs,
        registeredPersonTrackingStatusLagMaxMs
    );
    double trackThreshold = clampDouble(trackingDetectionThreshold - 0.05, 0.05, 0.95);
    double trackHighThreshold = clampDouble(trackingDetectionThreshold + 0.10, 0.05, 0.95);

    if (!yoloxConfigJson.contains("configure") || !yoloxConfigJson["configure"].is_object())
    {
        yoloxConfigJson["configure"] = json::object();
    }
    yoloxConfigJson["configure"]["model_path"] = resolveExistingModelPath(
        SOPHON_STREAM_SAMPLES_BUILD_DIR,
        yoloxConfigJson["configure"].value("model_path", "")
    );
    yoloxConfigJson["configure"]["threshold_conf"] = trackingDetectionThreshold;
    yoloxConfigJson["configure"]["threshold_nms"] = 0.45;
    if (!pathExists(yoloxConfigJson["configure"].value("model_path", "")))
    {
        errorMessage = "Registered person tracking yolox model not found";
        return false;
    }

    if (!bytetrackConfigJson.contains("configure") || !bytetrackConfigJson["configure"].is_object())
    {
        bytetrackConfigJson["configure"] = json::object();
    }
    bytetrackConfigJson["configure"]["track_thresh"] = trackThreshold;
    bytetrackConfigJson["configure"]["high_thresh"] = trackHighThreshold;
    bytetrackConfigJson["configure"]["match_thresh"] = 0.65;
    bytetrackConfigJson["configure"]["min_box_area"] = 1;
    bytetrackConfigJson["configure"]["track_buffer"] = lowLatencyMode ? 20 : 30;

    if (!distributorConfigJson.contains("configure") || !distributorConfigJson["configure"].is_object())
    {
        distributorConfigJson["configure"] = json::object();
    }
    distributorConfigJson["configure"]["class_names_file"] = resolveConfigPath(
        SOPHON_STREAM_SAMPLES_BUILD_DIR,
        distributorConfigJson["configure"].value("class_names_file", "")
    );
    if (distributorConfigJson["configure"].contains("rules") &&
        distributorConfigJson["configure"]["rules"].is_array())
    {
        for (auto& rule : distributorConfigJson["configure"]["rules"])
        {
            if (rule.is_object())
            {
                rule["frame_interval"] = recognitionFrameInterval;
                if (rule.contains("time_interval"))
                {
                    rule.erase("time_interval");
                }
            }
        }
    }

    if (!debugDistributorConfigJson.contains("configure") || !debugDistributorConfigJson["configure"].is_object())
    {
        debugDistributorConfigJson["configure"] = json::object();
    }
    debugDistributorConfigJson["configure"]["class_names_file"] = resolveConfigPath(
        SOPHON_STREAM_SAMPLES_BUILD_DIR,
        debugDistributorConfigJson["configure"].value("class_names_file", "")
    );
    if (debugDistributorConfigJson["configure"].contains("rules") &&
        debugDistributorConfigJson["configure"]["rules"].is_array())
    {
        for (auto& rule : debugDistributorConfigJson["configure"]["rules"])
        {
            if (!rule.is_object())
            {
                continue;
            }
            rule["frame_interval"] = recognitionFrameInterval;
            if (rule.contains("time_interval"))
            {
                rule.erase("time_interval");
            }
        }
    }

    if (!faissConfigJson.contains("configure") || !faissConfigJson["configure"].is_object())
    {
        faissConfigJson["configure"] = json::object();
    }
    faissConfigJson["configure"]["similarity_threshold"] = faceSimilarityThreshold;

    if (!trackStatusAttachConfigJson.contains("configure") ||
        !trackStatusAttachConfigJson["configure"].is_object())
    {
        trackStatusAttachConfigJson["configure"] = json::object();
    }
    trackStatusAttachConfigJson["configure"]["main_port"] = 0;
    trackStatusAttachConfigJson["configure"]["status_port"] = 1;
    trackStatusAttachConfigJson["configure"]["track_timeout_ms"] = clampInt(
        trackStatusAttachConfigJson["configure"].value("track_timeout_ms", 3000),
        500,
        10000
    );
    trackStatusAttachConfigJson["configure"]["max_status_lag_ms"] = statusLagMs;

    if (!faceTrackMatcherConfigJson.contains("configure") ||
        !faceTrackMatcherConfigJson["configure"].is_object())
    {
        faceTrackMatcherConfigJson["configure"] = json::object();
    }
    faceTrackMatcherConfigJson["configure"]["main_port"] = 0;
    faceTrackMatcherConfigJson["configure"]["face_port"] = 1;
    faceTrackMatcherConfigJson["configure"]["face_cache_max_age_ms"] = statusLagMs;
    faceTrackMatcherConfigJson["configure"]["max_status_lag_ms"] = statusLagMs;
    faceTrackMatcherConfigJson["configure"]["min_face_person_overlap_ratio"] = 0.2;

    if (!encodeConfigJson.contains("configure") || !encodeConfigJson["configure"].is_object())
    {
        encodeConfigJson["configure"] = json::object();
    }
    encodeConfigJson["configure"]["encode_type"] = "RTSP";
    encodeConfigJson["configure"]["ip"] = "127.0.0.1";
    encodeConfigJson["configure"]["fps"] = targetEncodeFps;
    encodeConfigJson["thread_number"] = 4;

    if (!httpPushConfigJson.contains("configure") || !httpPushConfigJson["configure"].is_object())
    {
        httpPushConfigJson["configure"] = json::object();
    }
    // 登记人员追踪统计只依赖帧元数据和识别结果，不需要把整帧 JPEG/base64 一起上报。
    httpPushConfigJson["configure"]["include_frame_data"] = false;
    httpPushConfigJson["configure"]["connection_timeout_ms"] = 1000;
    httpPushConfigJson["configure"]["read_timeout_ms"] = 1000;
    httpPushConfigJson["configure"]["write_timeout_ms"] = 1000;
    httpPushConfigJson["configure"]["latest_only"] = true;
    httpPushConfigJson["configure"]["min_post_interval_ms"] = 200;

    int runtimeFrameWidth = 1920;
    int runtimeFrameHeight = 1080;
    if (!probeStreamFrameSize(channelUrl, runtimeFrameWidth, runtimeFrameHeight))
    {
        if (regionConfigs.is_array() && !regionConfigs.empty() && regionConfigs[0].is_object())
        {
            runtimeFrameWidth = clampInt(
                regionConfigs[0].contains("frameWidth")
                    ? parseJsonInt(regionConfigs[0]["frameWidth"], runtimeFrameWidth)
                    : runtimeFrameWidth,
                1,
                16384);
            runtimeFrameHeight = clampInt(
                regionConfigs[0].contains("frameHeight")
                    ? parseJsonInt(regionConfigs[0]["frameHeight"], runtimeFrameHeight)
                    : runtimeFrameHeight,
                1,
                16384);
        }

        std::cout << "[登记人员追踪] 未探测到输入流尺寸，区域缩放回退到 "
                  << runtimeFrameWidth << "x" << runtimeFrameHeight << std::endl;
    }

    json filterAreas = buildRegisteredPersonTrackingFilterAreas(
        regionConfigs, runtimeFrameWidth, runtimeFrameHeight);
    if (!trackOsdConfigJson.contains("configure") ||
        !trackOsdConfigJson["configure"].is_object())
    {
        trackOsdConfigJson["configure"] = json::object();
    }
    trackOsdConfigJson["configure"]["draw_utils"] = "OPENCV";
    trackOsdConfigJson["configure"]["put_text"] = true;
    trackOsdConfigJson["configure"]["overlay_regions"] =
        buildRegisteredPersonTrackingOverlayRegions(
            regionConfigs, runtimeFrameWidth, runtimeFrameHeight);

    const std::string taskSuffix = "_task_" + std::to_string(taskId);
    const std::string runtimeYoloxConfigPath =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR + "/yolox_group_runtime" + taskSuffix + ".json";
    const std::string runtimeBytetrackConfigPath =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR + "/bytetrack_runtime" + taskSuffix + ".json";
    runtimeFilterConfigPath =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR + "/filter_runtime" + taskSuffix + ".json";
    const std::string runtimeDistributorConfigPath =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR + "/face_distributor_runtime" + taskSuffix + ".json";
    const std::string runtimeFaissConfigPath =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR + "/faiss_runtime" + taskSuffix + ".json";
    const std::string runtimeTrackOsdConfigPath =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR + "/track_osd_runtime" + taskSuffix + ".json";
    const std::string runtimeTrackStatusAttachConfigPath =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR + "/track_status_attach_runtime" + taskSuffix + ".json";
    const std::string runtimeFaceTrackMatcherConfigPath =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR + "/face_track_matcher_runtime" + taskSuffix + ".json";
    const std::string runtimeDebugDistributorConfigPath =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR + "/distributor_frame_runtime" + taskSuffix + ".json";
    const std::string runtimeEncodeConfigPath =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR + "/encode_runtime" + taskSuffix + ".json";
    const std::string runtimeHttpPushConfigPath =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR + "/http_push_runtime" + taskSuffix + ".json";
    const std::string runtimeEngineConfigPath =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR + "/engine_group_runtime" + taskSuffix + ".json";
    const std::string runtimeManifestPath =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR + "/registered_person_tracking_runtime_manifest" + taskSuffix + ".json";
    runtimeDemoConfigPath =
        REGISTERED_PERSON_TRACKING_RUNTIME_CONFIG_DIR +
        "/registered_person_tracking_runtime" + taskSuffix + ".json";

    if (!writeJsonFile(runtimeYoloxConfigPath, yoloxConfigJson) ||
        !writeJsonFile(runtimeBytetrackConfigPath, bytetrackConfigJson) ||
        !writeJsonFile(runtimeFilterConfigPath, filterConfigJson) ||
        !writeJsonFile(runtimeDistributorConfigPath, distributorConfigJson) ||
        !writeJsonFile(runtimeFaissConfigPath, faissConfigJson) ||
        !writeJsonFile(runtimeTrackOsdConfigPath, trackOsdConfigJson) ||
        !writeJsonFile(runtimeTrackStatusAttachConfigPath, trackStatusAttachConfigJson) ||
        !writeJsonFile(runtimeFaceTrackMatcherConfigPath, faceTrackMatcherConfigJson) ||
        !writeJsonFile(runtimeDebugDistributorConfigPath, debugDistributorConfigJson) ||
        !writeJsonFile(runtimeEncodeConfigPath, encodeConfigJson) ||
        !writeJsonFile(runtimeHttpPushConfigPath, httpPushConfigJson) ||
        !updatePolygonInFilterConfigFile(runtimeFilterConfigPath, filterAreas, streamChannelId))
    {
        errorMessage = "Failed to write registered person tracking runtime element config";
        return false;
    }

    for (auto& element : graph["elements"])
    {
        int elementId = element.value("element_id", -1);
        if (elementId == 5001)
        {
            element["element_config"] = runtimeYoloxConfigPath;
        }
        else if (elementId == 5002)
        {
            element["element_config"] = runtimeBytetrackConfigPath;
        }
        else if (elementId == 5005)
        {
            element["element_config"] = runtimeFilterConfigPath;
        }
        else if (elementId == 5017)
        {
            element["element_config"] = runtimeDistributorConfigPath;
        }
        else if (elementId == 5019)
        {
            element["element_config"] = runtimeFaissConfigPath;
        }
        else if (elementId == 5012)
        {
            element["element_config"] = runtimeTrackOsdConfigPath;
        }
        else if (elementId == 5015)
        {
            element["element_config"] = runtimeTrackStatusAttachConfigPath;
        }
        else if (elementId == 5023)
        {
            element["element_config"] = runtimeFaceTrackMatcherConfigPath;
        }
        else if (elementId == 5022)
        {
            element["element_config"] = runtimeDebugDistributorConfigPath;
        }
        else if (elementId == 5014)
        {
            element["element_config"] = runtimeEncodeConfigPath;
        }
        else if (elementId == kRegisteredTrackHttpPushElementId)
        {
            element["element_config"] = runtimeHttpPushConfigPath;
        }
    }

    if (!hasHttpPushElement)
    {
        graph["elements"].push_back({
            {"element_id", kRegisteredTrackHttpPushElementId},
            {"element_config", runtimeHttpPushConfigPath},
            {"ports", {
                {"output", json::array({
                    {
                        {"port_id", 0},
                        {"is_sink", true},
                        {"is_src", false}
                    }
                })}
            }}
        });
    }

    ensureRegisteredPersonTrackingFilterGraph(graph, runtimeFilterConfigPath);

    if (!graph.contains("connections") || !graph["connections"].is_array())
    {
        graph["connections"] = json::array();
    }
    bool hasTrackOsdToHttpPush = false;
    for (const auto& connection : graph["connections"])
    {
        if (!connection.is_object())
        {
            continue;
        }
        if (connection.value("src_element_id", -1) == kRegisteredTrackOsdElementId &&
            connection.value("src_port", -1) == 0 &&
            connection.value("dst_element_id", -1) == kRegisteredTrackHttpPushElementId &&
            connection.value("dst_port", -1) == 0)
        {
            hasTrackOsdToHttpPush = true;
            break;
        }
    }
    if (!hasTrackOsdToHttpPush)
    {
        graph["connections"].push_back({
            {"src_element_id", kRegisteredTrackOsdElementId},
            {"src_port", 0},
            {"dst_element_id", kRegisteredTrackHttpPushElementId},
            {"dst_port", 0}
        });
    }

    demoConfigJson["channels"] = json::array();
    demoConfigJson["channels"].push_back({
        {"channel_id", streamChannelId},
        {"url", channelUrl},
        {"source_type", detectStreamSourceType(channelUrl)},
        {"loop_num", 1},
        {"sample_interval", sampleInterval},
        {"sample_strategy", sampleStrategy},
        {"fps", targetEncodeFps}
    });
    demoConfigJson["engine_config_path"] = runtimeEngineConfigPath;

    json runtimeManifestJson = {
        {"taskId", taskId},
        {"streamChannelId", streamChannelId},
        {"sourceDemoConfigPath", sourceDemoConfigPath},
        {"sourceEngineConfigPath", engineConfigPath},
        {"sourceFilterConfigPath", sourceFilterConfigPath},
        {"generatedAtMs", currentTimeMillis()},
        {"overrides", {
            {"sampleInterval", sampleInterval},
            {"sampleStrategy", sampleStrategy},
            {"targetEncodeFps", targetEncodeFps},
            {"recognitionFrameInterval", recognitionFrameInterval},
            {"lowLatencyMode", lowLatencyMode},
            {"trackingDetectionThreshold", trackingDetectionThreshold},
            {"faceSimilarityThreshold", faceSimilarityThreshold},
            {"regionCount", filterAreas.size()},
            {"runtimeFrameWidth", runtimeFrameWidth},
            {"runtimeFrameHeight", runtimeFrameHeight}
        }},
        {"runtimeFiles", {
            {"demo", runtimeDemoConfigPath},
            {"engine", runtimeEngineConfigPath},
            {"yolox", runtimeYoloxConfigPath},
            {"bytetrack", runtimeBytetrackConfigPath},
            {"filter", runtimeFilterConfigPath},
            {"faceTrackMatcher", runtimeFaceTrackMatcherConfigPath},
            {"faceDistributor", runtimeDistributorConfigPath},
            {"trackOsd", runtimeTrackOsdConfigPath},
            {"trackStatusAttach", runtimeTrackStatusAttachConfigPath},
            {"debugDistributor", runtimeDebugDistributorConfigPath},
            {"faiss", runtimeFaissConfigPath},
            {"httpPush", runtimeHttpPushConfigPath},
            {"encode", runtimeEncodeConfigPath}
        }}
    };

    if (!writeJsonFile(runtimeEngineConfigPath, engineConfigJson) ||
        !writeJsonFile(runtimeManifestPath, runtimeManifestJson) ||
        !writeJsonFile(runtimeDemoConfigPath, demoConfigJson))
    {
        errorMessage = "Failed to write registered person tracking runtime config";
        return false;
    }

    errorMessage.clear();
    return true;
}

long long currentTimeMillis()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

long long normalizeFrameTimestampToMillis(long long rawTimestamp, long long fallbackTimestampMs)
{
    if (rawTimestamp <= 0)
    {
        return fallbackTimestampMs > 0 ? fallbackTimestampMs : currentTimeMillis();
    }

    // Sophon payload timestamps may arrive in seconds, milliseconds, microseconds, or nanoseconds.
    if (rawTimestamp >= 100000000000000000LL)
    {
        return rawTimestamp / 1000000LL;
    }
    if (rawTimestamp >= 100000000000000LL)
    {
        return rawTimestamp / 1000LL;
    }
    if (rawTimestamp >= 100000000000LL)
    {
        return rawTimestamp;
    }
    if (rawTimestamp >= 1000000000LL)
    {
        return rawTimestamp * 1000LL;
    }

    return fallbackTimestampMs > 0 ? fallbackTimestampMs : currentTimeMillis();
}

std::string normalizeRecognizedLabel(const std::string& rawLabel)
{
    return trimString(rawLabel);
}

std::string extractTrackStatusLabel(const json& trackedObject)
{
    if (!trackedObject.is_object())
    {
        return "";
    }

    std::string statusText = trimString(trackedObject.value("mName", ""));
    if (statusText.empty())
    {
        return "";
    }

    const std::string prefix = "status:";
    if (statusText.size() >= prefix.size() &&
        statusText.compare(0, prefix.size(), prefix) == 0)
    {
        statusText = trimString(statusText.substr(prefix.size()));
    }

    if (statusText == "unknown" || statusText == "no_face" ||
        statusText == "low_score" || statusText == "pending")
    {
        return "";
    }
    return statusText;
}

bool isKnownRecognizedLabel(const std::string& label)
{
    if (label.empty())
    {
        return false;
    }
    std::string lower = label;
    for (size_t i = 0; i < lower.size(); ++i)
    {
        lower[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(lower[i])));
    }
    return lower != "unknown";
}

double extractRecognizedScore(const json& recognizedObject)
{
    if (recognizedObject.contains("mScores") &&
        recognizedObject["mScores"].is_array() &&
        !recognizedObject["mScores"].empty() &&
        recognizedObject["mScores"][0].is_number())
    {
        return recognizedObject["mScores"][0].get<double>();
    }
    return 0.0;
}

void collectRecognizedCandidates(const json& objectJson,
                                 std::vector<std::pair<std::string, double> >& candidates,
                                 bool knownOnly)
{
    if (!objectJson.is_object())
    {
        return;
    }

    if (objectJson.contains("mRecognizedObjectMetadatas") &&
        objectJson["mRecognizedObjectMetadatas"].is_array())
    {
        for (const auto& recognizedObject : objectJson["mRecognizedObjectMetadatas"])
        {
            if (!recognizedObject.is_object())
            {
                continue;
            }
            std::string label = normalizeRecognizedLabel(recognizedObject.value("mLabelName", ""));
            if (label.empty())
            {
                continue;
            }
            if (knownOnly && !isKnownRecognizedLabel(label))
            {
                continue;
            }
            candidates.push_back(std::make_pair(label, extractRecognizedScore(recognizedObject)));
        }
    }

    if (objectJson.contains("mSubObjectMetadatas") && objectJson["mSubObjectMetadatas"].is_array())
    {
        for (const auto& subObject : objectJson["mSubObjectMetadatas"])
        {
            collectRecognizedCandidates(subObject, candidates, knownOnly);
        }
    }
}

bool extractBestRecognizedLabel(const json& personSubObject, std::string& label, double& score)
{
    std::vector<std::pair<std::string, double> > candidates;
    collectRecognizedCandidates(personSubObject, candidates, true);
    if (candidates.empty())
    {
        label.clear();
        score = 0.0;
        return false;
    }

    size_t bestIndex = 0;
    for (size_t i = 1; i < candidates.size(); ++i)
    {
        if (candidates[i].second > candidates[bestIndex].second)
        {
            bestIndex = i;
        }
    }

    label = candidates[bestIndex].first;
    score = candidates[bestIndex].second;
    return true;
}

bool extractBestObservedLabel(const json& personSubObject, std::string& label, double& score)
{
    std::vector<std::pair<std::string, double> > candidates;
    collectRecognizedCandidates(personSubObject, candidates, false);
    if (candidates.empty())
    {
        label.clear();
        score = 0.0;
        return false;
    }

    size_t bestIndex = 0;
    for (size_t i = 1; i < candidates.size(); ++i)
    {
        if (candidates[i].second > candidates[bestIndex].second)
        {
            bestIndex = i;
        }
    }

    label = candidates[bestIndex].first;
    score = candidates[bestIndex].second;
    return true;
}

struct RegisteredTrackState
{
    long long trackId = -1;
    long long recordId = 0;
    bool active = false;
    bool counted = false;
    bool alarmTriggered = false;
    bool hasBox = false;
    long long firstSeenMs = 0;
    long long enterTimeMs = 0;
    long long lastSeenMs = 0;
    long long confirmTimeMs = 0;
    long long exitTimeMs = 0;
    long long alarmTriggeredMs = 0;
    std::string pendingLabel;
    int pendingStreak = 0;
    long long pendingSinceMs = 0;
    std::string confirmedLabel;
    double bestScore = 0.0;
    std::string lastObservedLabel;
    double lastObservedScore = 0.0;
    int boxLeft = 0;
    int boxTop = 0;
    int boxWidth = 0;
    int boxHeight = 0;
};

struct RegisteredPersonAlarmEvent
{
    int taskId = 0;
    long long trackId = -1;
    std::string label;
    double score = 0.0;
};

struct RegisteredPersonVisitRecord
{
    long long recordId = 0;
    long long trackId = -1;
    std::string label;
    long long firstSeenMs = 0;
    long long enterTimeMs = 0;
    long long confirmTimeMs = 0;
    long long lastSeenMs = 0;
    long long exitTimeMs = 0;
    long long durationMs = 0;
};

struct RegisteredPersonAggregate
{
    int entries = 0;
    int exits = 0;
    long long totalDwellMs = 0;
    long long firstSeenMs = 0;
    long long lastSeenMs = 0;
    long long lastEnterMs = 0;
    long long lastExitMs = 0;
    std::set<long long> activeTrackIds;
};

struct RegisteredPersonTrackingTaskState
{
    int taskId = 0;
    int streamChannelId = 0;
    int frameWidth = 0;
    int frameHeight = 0;
    long long lastFrameId = 0;
    long long lastFrameTimestampMs = 0;
    std::unordered_map<long long, RegisteredTrackState> tracks;
    std::unordered_map<std::string, RegisteredPersonAggregate> persons;
    std::vector<RegisteredPersonVisitRecord> records;
    long long nextRecordId = 1;
};

std::mutex registeredPersonTrackingStateMutex;
std::unordered_map<int, RegisteredPersonTrackingTaskState> registeredPersonTrackingStates;
std::unordered_map<int, int> registeredPersonTrackingChannelToTaskId;
const std::string REGISTERED_PERSON_TRACKING_STATS_FILE =
    "/data/lintech/celectronicfence/registered_person_tracking_stats.json";
const int REGISTERED_PERSON_TRACKING_CONFIRM_FRAMES = 2;
const long long REGISTERED_PERSON_TRACKING_CONFIRM_STABLE_MS = 400;
const long long REGISTERED_PERSON_TRACKING_TRACK_TIMEOUT_MS = 3000;
const long long REGISTERED_PERSON_TRACKING_TRACK_EVICT_MS = 30000;
const size_t REGISTERED_PERSON_TRACKING_MAX_RECORDS = 1000;
const long long REGISTERED_PERSON_TRACKING_RECORD_MERGE_GAP_MS = 5000;
const size_t REGISTERED_PERSON_TRACKING_RECENT_RECORD_SCAN_LIMIT = 48;
const double REGISTERED_PERSON_TRACKING_DEFAULT_FACE_SIMILARITY_THRESHOLD = 0.45;
bool registeredPersonTrackingPersistenceLoaded = false;
bool registeredPersonTrackingPersistenceDirty = false;
std::mutex registeredPersonAlarmCooldownMutex;
std::unordered_map<std::string, long long> registeredPersonAlarmCooldownUntilMs;

void finalizeRegisteredTrackStateUnlocked(RegisteredPersonTrackingTaskState& taskState,
                                          RegisteredTrackState& trackState);
void emitRegisteredPersonTrackingAlarm(const RegisteredPersonAlarmEvent& event);

void markRegisteredPersonTrackingPersistenceDirtyUnlocked()
{
    registeredPersonTrackingPersistenceDirty = true;
}

json serializeRegisteredPersonTrackingTaskStateForPersistenceUnlocked(
    const RegisteredPersonTrackingTaskState& taskState)
{
    json taskJson;
    taskJson["taskId"] = taskState.taskId;
    taskJson["nextRecordId"] = taskState.nextRecordId;

    json personsJson = json::array();
    for (auto it = taskState.persons.begin(); it != taskState.persons.end(); ++it)
    {
        const RegisteredPersonAggregate& aggregate = it->second;
        personsJson.push_back({
            {"label", it->first},
            {"entries", aggregate.entries},
            {"exits", aggregate.exits},
            {"totalDwellMs", aggregate.totalDwellMs},
            {"firstSeenMs", aggregate.firstSeenMs},
            {"lastSeenMs", aggregate.lastSeenMs},
            {"lastEnterMs", aggregate.lastEnterMs},
            {"lastExitMs", aggregate.lastExitMs}
        });
    }

    json recordsJson = json::array();
    for (size_t i = 0; i < taskState.records.size(); ++i)
    {
        const RegisteredPersonVisitRecord& record = taskState.records[i];
        recordsJson.push_back({
            {"recordId", record.recordId},
            {"trackId", record.trackId},
            {"label", record.label},
            {"firstSeenMs", record.firstSeenMs},
            {"enterTimeMs", record.enterTimeMs},
            {"confirmTimeMs", record.confirmTimeMs},
            {"lastSeenMs", record.lastSeenMs},
            {"exitTimeMs", record.exitTimeMs},
            {"durationMs", record.durationMs}
        });
    }

    taskJson["persons"] = personsJson;
    taskJson["records"] = recordsJson;
    return taskJson;
}

void ensureRegisteredPersonTrackingPersistenceLoadedUnlocked()
{
    if (registeredPersonTrackingPersistenceLoaded)
    {
        return;
    }

    registeredPersonTrackingPersistenceLoaded = true;
    registeredPersonTrackingStates.clear();

    std::ifstream statsFileIn(REGISTERED_PERSON_TRACKING_STATS_FILE.c_str());
    if (!statsFileIn.is_open())
    {
        return;
    }

    json persistedJson;
    try
    {
        statsFileIn >> persistedJson;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[登记人员追踪] 读取持久化统计失败: " << e.what() << std::endl;
        return;
    }

    json tasksArray = json::array();
    if (persistedJson.is_object() &&
        persistedJson.contains("tasks") &&
        persistedJson["tasks"].is_array())
    {
        tasksArray = persistedJson["tasks"];
    }
    else if (persistedJson.is_array())
    {
        tasksArray = persistedJson;
    }
    else
    {
        return;
    }

    for (size_t i = 0; i < tasksArray.size(); ++i)
    {
        const json& taskJson = tasksArray[i];
        if (!taskJson.is_object())
        {
            continue;
        }

        int taskId = taskJson.value("taskId", 0);
        if (taskId <= 0)
        {
            continue;
        }

        RegisteredPersonTrackingTaskState taskState;
        taskState.taskId = taskId;
        taskState.nextRecordId = std::max(1LL, taskJson.value("nextRecordId", 1LL));

        long long maxRecordId = 0;
        if (taskJson.contains("records") && taskJson["records"].is_array())
        {
            for (const auto& recordJson : taskJson["records"])
            {
                if (!recordJson.is_object())
                {
                    continue;
                }

                RegisteredPersonVisitRecord record;
                record.recordId = recordJson.value("recordId", 0LL);
                record.trackId = recordJson.value("trackId", -1LL);
                record.label = recordJson.value("label", "");
                record.firstSeenMs = recordJson.value("firstSeenMs", 0LL);
                record.enterTimeMs = recordJson.value("enterTimeMs", 0LL);
                record.confirmTimeMs = recordJson.value("confirmTimeMs", 0LL);
                record.lastSeenMs = recordJson.value("lastSeenMs", 0LL);
                record.exitTimeMs = recordJson.value("exitTimeMs", 0LL);
                record.durationMs = std::max(0LL, recordJson.value("durationMs", 0LL));
                taskState.records.push_back(record);
                if (record.recordId > maxRecordId)
                {
                    maxRecordId = record.recordId;
                }
            }
        }

        if (taskJson.contains("persons") && taskJson["persons"].is_array())
        {
            for (const auto& personJson : taskJson["persons"])
            {
                if (!personJson.is_object())
                {
                    continue;
                }

                std::string label = personJson.value("label", "");
                if (label.empty())
                {
                    continue;
                }

                RegisteredPersonAggregate aggregate;
                aggregate.entries = std::max(0, personJson.value("entries", 0));
                aggregate.exits = std::max(0, personJson.value("exits", 0));
                aggregate.totalDwellMs = std::max(0LL, personJson.value("totalDwellMs", 0LL));
                aggregate.firstSeenMs = personJson.value("firstSeenMs", 0LL);
                aggregate.lastSeenMs = personJson.value("lastSeenMs", 0LL);
                aggregate.lastEnterMs = personJson.value("lastEnterMs", 0LL);
                aggregate.lastExitMs = personJson.value("lastExitMs", 0LL);
                taskState.persons[label] = aggregate;
            }
        }

        taskState.nextRecordId = std::max(taskState.nextRecordId, maxRecordId + 1);
        registeredPersonTrackingStates[taskId] = taskState;
    }

    registeredPersonTrackingPersistenceDirty = false;
}

void flushRegisteredPersonTrackingPersistenceUnlocked()
{
    ensureRegisteredPersonTrackingPersistenceLoadedUnlocked();
    if (!registeredPersonTrackingPersistenceDirty)
    {
        return;
    }

    json tasksArray = json::array();
    for (auto it = registeredPersonTrackingStates.begin(); it != registeredPersonTrackingStates.end(); ++it)
    {
        tasksArray.push_back(serializeRegisteredPersonTrackingTaskStateForPersistenceUnlocked(it->second));
    }

    json rootJson;
    rootJson["schemaVersion"] = 1;
    rootJson["updatedAtMs"] = currentTimeMillis();
    rootJson["tasks"] = tasksArray;

    const std::string tempStatsFile = REGISTERED_PERSON_TRACKING_STATS_FILE + ".tmp";
    std::ofstream statsFileOut(tempStatsFile.c_str(), std::ios::trunc);
    if (!statsFileOut.is_open())
    {
        std::cerr << "[登记人员追踪] 写入持久化统计失败: "
                  << tempStatsFile << std::endl;
        return;
    }

    statsFileOut << rootJson.dump(2);
    statsFileOut.close();
    if (!statsFileOut.good())
    {
        std::remove(tempStatsFile.c_str());
        std::cerr << "[登记人员追踪] 写入持久化统计临时文件失败: "
                  << tempStatsFile << std::endl;
        return;
    }

    if (std::rename(tempStatsFile.c_str(), REGISTERED_PERSON_TRACKING_STATS_FILE.c_str()) != 0)
    {
        std::cerr << "[登记人员追踪] 替换持久化统计文件失败: "
                  << std::strerror(errno) << std::endl;
        std::remove(tempStatsFile.c_str());
        return;
    }
    registeredPersonTrackingPersistenceDirty = false;
}

void trimRegisteredPersonTrackingRecordsUnlocked(RegisteredPersonTrackingTaskState& taskState)
{
    if (taskState.records.size() <= REGISTERED_PERSON_TRACKING_MAX_RECORDS)
    {
        return;
    }
    taskState.records.erase(taskState.records.begin(),
                            taskState.records.begin() +
                                (taskState.records.size() -
                                 REGISTERED_PERSON_TRACKING_MAX_RECORDS));
    markRegisteredPersonTrackingPersistenceDirtyUnlocked();
}

void resetRegisteredTrackPendingLabelUnlocked(RegisteredTrackState& trackState)
{
    trackState.pendingLabel.clear();
    trackState.pendingStreak = 0;
    trackState.pendingSinceMs = 0;
}

void observeRegisteredTrackPendingLabelUnlocked(RegisteredTrackState& trackState,
                                                const std::string& label,
                                                long long frameTimestampMs)
{
    if (label.empty())
    {
        resetRegisteredTrackPendingLabelUnlocked(trackState);
        return;
    }

    if (trackState.pendingLabel == label)
    {
        trackState.pendingStreak += 1;
        if (trackState.pendingSinceMs <= 0)
        {
            trackState.pendingSinceMs = frameTimestampMs;
        }
        return;
    }

    trackState.pendingLabel = label;
    trackState.pendingStreak = 1;
    trackState.pendingSinceMs = frameTimestampMs;
}

bool hasRegisteredTrackPendingLabelStabilizedUnlocked(const RegisteredTrackState& trackState,
                                                      long long frameTimestampMs)
{
    if (trackState.pendingLabel.empty() ||
        trackState.pendingStreak < REGISTERED_PERSON_TRACKING_CONFIRM_FRAMES)
    {
        return false;
    }

    if (REGISTERED_PERSON_TRACKING_CONFIRM_STABLE_MS <= 0)
    {
        return true;
    }

    if (trackState.pendingSinceMs <= 0 || frameTimestampMs < trackState.pendingSinceMs)
    {
        return false;
    }

    return frameTimestampMs - trackState.pendingSinceMs >=
           REGISTERED_PERSON_TRACKING_CONFIRM_STABLE_MS;
}

void recomputeRegisteredPersonAggregateUnlocked(RegisteredPersonTrackingTaskState& taskState,
                                                const std::string& label)
{
    if (label.empty())
    {
        return;
    }

    RegisteredPersonAggregate aggregate;
    bool hasData = false;

    for (size_t i = 0; i < taskState.records.size(); ++i)
    {
        const RegisteredPersonVisitRecord& record = taskState.records[i];
        if (record.label != label)
        {
            continue;
        }

        hasData = true;
        aggregate.entries += 1;
        aggregate.exits += 1;
        aggregate.totalDwellMs += std::max(0LL, record.durationMs);

        long long enterMs = record.enterTimeMs > 0 ? record.enterTimeMs : record.firstSeenMs;
        if (enterMs > 0 &&
            (aggregate.firstSeenMs <= 0 || enterMs < aggregate.firstSeenMs))
        {
            aggregate.firstSeenMs = enterMs;
        }
        aggregate.lastSeenMs = std::max(aggregate.lastSeenMs, record.lastSeenMs);
        aggregate.lastEnterMs = std::max(aggregate.lastEnterMs, record.enterTimeMs);
        aggregate.lastExitMs = std::max(aggregate.lastExitMs, record.exitTimeMs);
    }

    for (auto it = taskState.tracks.begin(); it != taskState.tracks.end(); ++it)
    {
        const RegisteredTrackState& trackState = it->second;
        if (!trackState.active || !trackState.counted || trackState.confirmedLabel != label)
        {
            continue;
        }

        hasData = true;
        aggregate.entries += 1;
        long long enterMs =
            trackState.enterTimeMs > 0 ? trackState.enterTimeMs : trackState.firstSeenMs;
        if (enterMs > 0 &&
            (aggregate.firstSeenMs <= 0 || enterMs < aggregate.firstSeenMs))
        {
            aggregate.firstSeenMs = enterMs;
        }
        aggregate.lastSeenMs = std::max(aggregate.lastSeenMs, trackState.lastSeenMs);
        aggregate.lastEnterMs = std::max(aggregate.lastEnterMs, enterMs);
        aggregate.activeTrackIds.insert(trackState.trackId);
    }

    if (!hasData && aggregate.activeTrackIds.empty())
    {
        taskState.persons.erase(label);
        return;
    }

    taskState.persons[label] = aggregate;
}

void applyRegisteredTrackConfirmedLabelUnlocked(RegisteredPersonTrackingTaskState& taskState,
                                                RegisteredTrackState& trackState,
                                                const std::string& label,
                                                double score,
                                                long long frameTimestampMs)
{
    if (label.empty())
    {
        return;
    }

    const std::string previousLabel = trackState.confirmedLabel;
    const bool changed = !previousLabel.empty() && previousLabel != label;

    trackState.confirmedLabel = label;
    trackState.confirmTimeMs = frameTimestampMs;
    trackState.bestScore = changed ? (score > 0.0 ? score : 0.0)
                                   : std::max(trackState.bestScore, score);
    resetRegisteredTrackPendingLabelUnlocked(trackState);

    if (!changed || !trackState.counted)
    {
        return;
    }

    recomputeRegisteredPersonAggregateUnlocked(taskState, previousLabel);
    recomputeRegisteredPersonAggregateUnlocked(taskState, trackState.confirmedLabel);
    markRegisteredPersonTrackingPersistenceDirtyUnlocked();
}

bool mergeRecentRegisteredVisitRecordUnlocked(RegisteredPersonTrackingTaskState& taskState,
                                              RegisteredPersonAggregate& aggregate,
                                              const RegisteredPersonVisitRecord& record)
{
    size_t scanned = 0;
    for (auto it = taskState.records.rbegin();
         it != taskState.records.rend() &&
         scanned < REGISTERED_PERSON_TRACKING_RECENT_RECORD_SCAN_LIMIT;
         ++it, ++scanned)
    {
        if (it->trackId != record.trackId || it->label != record.label)
        {
            continue;
        }

        long long existingEnterMs = it->enterTimeMs > 0 ? it->enterTimeMs : it->firstSeenMs;
        long long existingExitMs = it->exitTimeMs > 0 ? it->exitTimeMs : it->lastSeenMs;
        long long recordEnterMs = record.enterTimeMs > 0 ? record.enterTimeMs : record.firstSeenMs;
        long long recordExitMs = record.exitTimeMs > 0 ? record.exitTimeMs : record.lastSeenMs;

        if (existingExitMs > 0 && recordEnterMs > existingExitMs + REGISTERED_PERSON_TRACKING_RECORD_MERGE_GAP_MS)
        {
            continue;
        }
        if (recordExitMs > 0 && existingEnterMs > recordExitMs + REGISTERED_PERSON_TRACKING_RECORD_MERGE_GAP_MS)
        {
            continue;
        }

        long long previousDurationMs = it->durationMs;
        if (record.firstSeenMs > 0 && (it->firstSeenMs <= 0 || record.firstSeenMs < it->firstSeenMs))
        {
            it->firstSeenMs = record.firstSeenMs;
        }
        if (record.enterTimeMs > 0 && (it->enterTimeMs <= 0 || record.enterTimeMs < it->enterTimeMs))
        {
            it->enterTimeMs = record.enterTimeMs;
        }
        if (record.confirmTimeMs > 0 && (it->confirmTimeMs <= 0 || record.confirmTimeMs < it->confirmTimeMs))
        {
            it->confirmTimeMs = record.confirmTimeMs;
        }
        if (record.lastSeenMs > it->lastSeenMs)
        {
            it->lastSeenMs = record.lastSeenMs;
        }
        if (record.exitTimeMs > it->exitTimeMs)
        {
            it->exitTimeMs = record.exitTimeMs;
        }
        if (it->enterTimeMs > 0 && it->exitTimeMs >= it->enterTimeMs)
        {
            it->durationMs = it->exitTimeMs - it->enterTimeMs;
        }

        if (it->durationMs > previousDurationMs)
        {
            aggregate.totalDwellMs += (it->durationMs - previousDurationMs);
        }
        if (it->lastSeenMs > aggregate.lastSeenMs)
        {
            aggregate.lastSeenMs = it->lastSeenMs;
        }
        if (it->exitTimeMs > aggregate.lastExitMs)
        {
            aggregate.lastExitMs = it->exitTimeMs;
        }
        return true;
    }

    return false;
}

void rollbackRecentRegisteredVisitRecordUnlocked(RegisteredPersonTrackingTaskState& taskState,
                                                 RegisteredPersonAggregate& aggregate,
                                                 const RegisteredTrackState& trackState)
{
    if (trackState.recordId <= 0 || trackState.trackId < 0 || trackState.confirmedLabel.empty())
    {
        return;
    }

    for (auto it = taskState.records.end(); it != taskState.records.begin();)
    {
        --it;
        if (it->recordId != trackState.recordId ||
            it->trackId != trackState.trackId ||
            it->label != trackState.confirmedLabel)
        {
            continue;
        }

        const long long removedExitMs = it->exitTimeMs;
        const long long removedDurationMs = std::max(0LL, it->durationMs);
        taskState.records.erase(it);
        if (removedDurationMs > 0)
        {
            aggregate.totalDwellMs = std::max(0LL, aggregate.totalDwellMs - removedDurationMs);
        }
        if (aggregate.exits > 0)
        {
            aggregate.exits -= 1;
        }
        if (aggregate.lastExitMs == removedExitMs)
        {
            aggregate.lastExitMs = 0;
            for (size_t recordIndex = 0; recordIndex < taskState.records.size(); ++recordIndex)
            {
                const RegisteredPersonVisitRecord& existingRecord = taskState.records[recordIndex];
                if (existingRecord.label == trackState.confirmedLabel &&
                    existingRecord.exitTimeMs > aggregate.lastExitMs)
                {
                    aggregate.lastExitMs = existingRecord.exitTimeMs;
                }
            }
        }
        markRegisteredPersonTrackingPersistenceDirtyUnlocked();
        return;
    }
}

void bindRegisteredPersonTrackingStreamTask(int streamChannelId, int taskId)
{
    if (streamChannelId <= 0 || taskId <= 0)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(registeredPersonTrackingStateMutex);
    registeredPersonTrackingChannelToTaskId[streamChannelId] = taskId;
}

void unbindRegisteredPersonTrackingStreamTaskByTaskId(int taskId)
{
    std::lock_guard<std::mutex> lock(registeredPersonTrackingStateMutex);
    for (auto it = registeredPersonTrackingChannelToTaskId.begin();
         it != registeredPersonTrackingChannelToTaskId.end();)
    {
        if (it->second == taskId)
        {
            it = registeredPersonTrackingChannelToTaskId.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

int resolveRegisteredPersonTrackingTaskIdByStreamChannelId(int streamChannelId)
{
    if (streamChannelId <= 0)
    {
        return 0;
    }

    {
        std::lock_guard<std::mutex> lock(registeredPersonTrackingStateMutex);
        std::unordered_map<int, int>::const_iterator it =
            registeredPersonTrackingChannelToTaskId.find(streamChannelId);
        if (it != registeredPersonTrackingChannelToTaskId.end())
        {
            return it->second;
        }
    }

    std::ifstream tasksFileIn("/data/lintech/celectronicfence/tasks.json");
    if (!tasksFileIn.is_open())
    {
        return 0;
    }

    json tasksJson;
    try
    {
        tasksFileIn >> tasksJson;
    }
    catch (...)
    {
        return 0;
    }

    if (!tasksJson.is_array())
    {
        return 0;
    }

    for (size_t i = 0; i < tasksJson.size(); ++i)
    {
        const json& task = tasksJson[i];
        if (!isRegisteredPersonTrackingAlgorithm(task.value("algorithm", "")))
        {
            continue;
        }
        if (task.value("streamChannelId", 0) != streamChannelId)
        {
            continue;
        }

        int taskId = task.value("id", 0);
        if (taskId > 0)
        {
            bindRegisteredPersonTrackingStreamTask(streamChannelId, taskId);
            return taskId;
        }
    }

    return 0;
}

void clearRegisteredPersonTrackingTaskState(int taskId, bool preserveHistory = true)
{
    if (taskId <= 0)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(registeredPersonTrackingStateMutex);
    ensureRegisteredPersonTrackingPersistenceLoadedUnlocked();
    std::unordered_map<int, RegisteredPersonTrackingTaskState>::iterator stateIt =
        registeredPersonTrackingStates.find(taskId);
    if (stateIt != registeredPersonTrackingStates.end())
    {
        if (preserveHistory)
        {
            for (auto trackIt = stateIt->second.tracks.begin();
                 trackIt != stateIt->second.tracks.end();
                 ++trackIt)
            {
                finalizeRegisteredTrackStateUnlocked(stateIt->second, trackIt->second);
            }
            stateIt->second.streamChannelId = 0;
            stateIt->second.frameWidth = 0;
            stateIt->second.frameHeight = 0;
            stateIt->second.lastFrameId = 0;
            stateIt->second.lastFrameTimestampMs = 0;
            stateIt->second.tracks.clear();
        }
        else
        {
            registeredPersonTrackingStates.erase(stateIt);
            markRegisteredPersonTrackingPersistenceDirtyUnlocked();
        }
    }
    for (auto it = registeredPersonTrackingChannelToTaskId.begin();
         it != registeredPersonTrackingChannelToTaskId.end();)
    {
        if (it->second == taskId)
        {
            it = registeredPersonTrackingChannelToTaskId.erase(it);
        }
        else
        {
            ++it;
        }
    }
    flushRegisteredPersonTrackingPersistenceUnlocked();
}

void resetRegisteredPersonTrackingTaskStatsUnlocked(RegisteredPersonTrackingTaskState& taskState)
{
    taskState.records.clear();
    taskState.persons.clear();
    taskState.tracks.clear();
    taskState.nextRecordId = 1;
    markRegisteredPersonTrackingPersistenceDirtyUnlocked();
}

bool resetRegisteredPersonTrackingTaskStats(int taskId)
{
    if (taskId <= 0)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(registeredPersonTrackingStateMutex);
    ensureRegisteredPersonTrackingPersistenceLoadedUnlocked();
    std::unordered_map<int, RegisteredPersonTrackingTaskState>::iterator stateIt =
        registeredPersonTrackingStates.find(taskId);
    if (stateIt == registeredPersonTrackingStates.end())
    {
        return false;
    }

    resetRegisteredPersonTrackingTaskStatsUnlocked(stateIt->second);
    flushRegisteredPersonTrackingPersistenceUnlocked();
    return true;
}

void finalizeRegisteredTrackStateUnlocked(RegisteredPersonTrackingTaskState& taskState,
                                          RegisteredTrackState& trackState)
{
    if (!trackState.active)
    {
        return;
    }
    trackState.active = false;
    if (trackState.lastSeenMs > 0)
    {
        trackState.exitTimeMs = trackState.lastSeenMs;
    }

    if (!trackState.counted || trackState.confirmedLabel.empty())
    {
        return;
    }

    RegisteredPersonAggregate& aggregate = taskState.persons[trackState.confirmedLabel];
    if (aggregate.firstSeenMs <= 0)
    {
        if (trackState.enterTimeMs > 0)
        {
            aggregate.firstSeenMs = trackState.enterTimeMs;
        }
        else
        {
            aggregate.firstSeenMs =
                trackState.firstSeenMs > 0 ? trackState.firstSeenMs : trackState.lastSeenMs;
        }
    }
    aggregate.lastSeenMs = std::max(aggregate.lastSeenMs, trackState.lastSeenMs);
    if (trackState.enterTimeMs > 0)
    {
        aggregate.lastEnterMs = std::max(aggregate.lastEnterMs, trackState.enterTimeMs);
    }
    if (trackState.exitTimeMs > 0)
    {
        aggregate.lastExitMs = std::max(aggregate.lastExitMs, trackState.exitTimeMs);
    }
    aggregate.activeTrackIds.erase(trackState.trackId);

    RegisteredPersonVisitRecord record;
    record.recordId = trackState.recordId > 0 ? trackState.recordId
                                              : taskState.nextRecordId++;
    record.trackId = trackState.trackId;
    record.label = trackState.confirmedLabel;
    record.firstSeenMs = trackState.firstSeenMs;
    record.enterTimeMs = trackState.enterTimeMs;
    record.confirmTimeMs = trackState.confirmTimeMs;
    record.lastSeenMs = trackState.lastSeenMs;
    record.exitTimeMs = trackState.exitTimeMs;
    if (trackState.enterTimeMs > 0 && trackState.exitTimeMs >= trackState.enterTimeMs)
    {
        record.durationMs = trackState.exitTimeMs - trackState.enterTimeMs;
    }

    if (mergeRecentRegisteredVisitRecordUnlocked(taskState, aggregate, record))
    {
        markRegisteredPersonTrackingPersistenceDirtyUnlocked();
        return;
    }

    if (record.durationMs > 0)
    {
        aggregate.totalDwellMs += record.durationMs;
    }
    aggregate.exits += 1;
    taskState.records.push_back(record);
    trimRegisteredPersonTrackingRecordsUnlocked(taskState);
    markRegisteredPersonTrackingPersistenceDirtyUnlocked();
}

void pruneRegisteredPersonTrackingTaskStateUnlocked(RegisteredPersonTrackingTaskState& taskState,
                                                    long long nowMs)
{
    for (auto it = taskState.tracks.begin(); it != taskState.tracks.end();)
    {
        RegisteredTrackState& trackState = it->second;
        if (trackState.active &&
            trackState.lastSeenMs > 0 &&
            nowMs - trackState.lastSeenMs > REGISTERED_PERSON_TRACKING_TRACK_TIMEOUT_MS)
        {
            finalizeRegisteredTrackStateUnlocked(taskState, trackState);
        }

        if (trackState.lastSeenMs > 0 &&
            nowMs - trackState.lastSeenMs > REGISTERED_PERSON_TRACKING_TRACK_EVICT_MS)
        {
            it = taskState.tracks.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void updateRegisteredPersonTrackingTaskStateUnlocked(RegisteredPersonTrackingTaskState& taskState,
                                                     long long trackId,
                                                     const std::string& label,
                                                     double score,
                                                     const std::string& observedLabel,
                                                     double observedScore,
                                                     long long frameTimestampMs,
                                                     bool hasBox,
                                                     int boxLeft,
                                                     int boxTop,
                                                     int boxWidth,
                                                     int boxHeight,
                                                     std::vector<RegisteredPersonAlarmEvent>* pendingAlarmEvents)
{
    RegisteredTrackState& trackState = taskState.tracks[trackId];
    if (trackState.trackId < 0)
    {
        trackState.trackId = trackId;
    }
    if (!trackState.active)
    {
        const bool resumePreviousVisit =
            trackState.counted &&
            !trackState.confirmedLabel.empty() &&
            trackState.lastSeenMs > 0 &&
            frameTimestampMs >= trackState.lastSeenMs &&
            frameTimestampMs - trackState.lastSeenMs <= REGISTERED_PERSON_TRACKING_RECORD_MERGE_GAP_MS;
        if (resumePreviousVisit)
        {
            RegisteredPersonAggregate& aggregate = taskState.persons[trackState.confirmedLabel];
            rollbackRecentRegisteredVisitRecordUnlocked(taskState, aggregate, trackState);
            trackState.active = true;
            trackState.hasBox = false;
            trackState.exitTimeMs = 0;
        }
        else
        {
            trackState.recordId = 0;
            trackState.active = true;
            trackState.counted = false;
            trackState.alarmTriggered = false;
            trackState.hasBox = false;
            trackState.firstSeenMs = frameTimestampMs;
            trackState.enterTimeMs = 0;
            trackState.lastSeenMs = 0;
            trackState.confirmTimeMs = 0;
            trackState.exitTimeMs = 0;
            trackState.alarmTriggeredMs = 0;
            trackState.pendingStreak = 0;
            trackState.pendingSinceMs = 0;
            trackState.pendingLabel.clear();
            trackState.confirmedLabel.clear();
            trackState.bestScore = 0.0;
            trackState.lastObservedLabel.clear();
            trackState.lastObservedScore = 0.0;
            trackState.boxLeft = 0;
            trackState.boxTop = 0;
            trackState.boxWidth = 0;
            trackState.boxHeight = 0;
        }
    }
    else if (trackState.firstSeenMs <= 0)
    {
        trackState.firstSeenMs = frameTimestampMs;
    }
    trackState.lastSeenMs = frameTimestampMs;
    trackState.hasBox = hasBox;
    if (hasBox)
    {
        trackState.boxLeft = boxLeft;
        trackState.boxTop = boxTop;
        trackState.boxWidth = boxWidth;
        trackState.boxHeight = boxHeight;
    }
    else
    {
        trackState.boxLeft = 0;
        trackState.boxTop = 0;
        trackState.boxWidth = 0;
        trackState.boxHeight = 0;
    }
    trackState.lastObservedLabel = observedLabel;
    trackState.lastObservedScore = observedScore;

    if (isKnownRecognizedLabel(label))
    {
        if (trackState.confirmedLabel.empty())
        {
            observeRegisteredTrackPendingLabelUnlocked(trackState, label, frameTimestampMs);
            if (hasRegisteredTrackPendingLabelStabilizedUnlocked(trackState, frameTimestampMs))
            {
                applyRegisteredTrackConfirmedLabelUnlocked(
                    taskState, trackState, label, score, frameTimestampMs);
            }
        }
        else if (trackState.confirmedLabel == label)
        {
            trackState.bestScore = std::max(trackState.bestScore, score);
            resetRegisteredTrackPendingLabelUnlocked(trackState);
        }
        else
        {
            observeRegisteredTrackPendingLabelUnlocked(trackState, label, frameTimestampMs);
            if (hasRegisteredTrackPendingLabelStabilizedUnlocked(trackState, frameTimestampMs))
            {
                applyRegisteredTrackConfirmedLabelUnlocked(
                    taskState, trackState, label, score, frameTimestampMs);
            }
        }
    }
    else
    {
        resetRegisteredTrackPendingLabelUnlocked(trackState);
    }

    if (!trackState.confirmedLabel.empty())
    {
        RegisteredPersonAggregate& aggregate = taskState.persons[trackState.confirmedLabel];
        if (!trackState.counted)
        {
            if (trackState.recordId <= 0)
            {
                trackState.recordId = taskState.nextRecordId++;
            }
            trackState.enterTimeMs =
                trackState.firstSeenMs > 0 ? trackState.firstSeenMs : frameTimestampMs;
            if (aggregate.firstSeenMs <= 0 || trackState.enterTimeMs < aggregate.firstSeenMs)
            {
                aggregate.firstSeenMs = trackState.enterTimeMs;
            }
            aggregate.entries += 1;
            aggregate.lastEnterMs = std::max(aggregate.lastEnterMs, trackState.enterTimeMs);
            trackState.counted = true;
            markRegisteredPersonTrackingPersistenceDirtyUnlocked();
            if (!trackState.alarmTriggered && pendingAlarmEvents != nullptr)
            {
                RegisteredPersonAlarmEvent event;
                event.taskId = taskState.taskId;
                event.trackId = trackId;
                event.label = trackState.confirmedLabel;
                event.score = trackState.bestScore > 0.0 ? trackState.bestScore : score;
                pendingAlarmEvents->push_back(event);
                trackState.alarmTriggered = true;
                trackState.alarmTriggeredMs = frameTimestampMs;
            }
        }
        else if (trackState.enterTimeMs > 0)
        {
            aggregate.lastEnterMs = std::max(aggregate.lastEnterMs, trackState.enterTimeMs);
        }

        aggregate.lastSeenMs = frameTimestampMs;
        aggregate.activeTrackIds.insert(trackId);
    }
}

void processRegisteredPersonTrackingPayload(int taskId,
                                            int streamChannelId,
                                            const json& payload)
{
    long long nowMs = currentTimeMillis();
    long long frameTimestampMs = nowMs;
    long long frameId = 0;
    int frameWidth = 0;
    int frameHeight = 0;
    if (payload.contains("mFrame") && payload["mFrame"].is_object())
    {
        frameTimestampMs = normalizeFrameTimestampToMillis(
            payload["mFrame"].value("mTimestamp", nowMs),
            nowMs
        );
        frameId = payload["mFrame"].value("mFrameId", 0LL);
        frameWidth = payload["mFrame"].value("mWidth", 0);
        frameHeight = payload["mFrame"].value("mHeight", 0);
    }

    std::vector<RegisteredPersonAlarmEvent> pendingAlarmEvents;
    {
        std::lock_guard<std::mutex> lock(registeredPersonTrackingStateMutex);
        ensureRegisteredPersonTrackingPersistenceLoadedUnlocked();
        RegisteredPersonTrackingTaskState& taskState = registeredPersonTrackingStates[taskId];
        taskState.taskId = taskId;
        taskState.streamChannelId = streamChannelId;
        if (frameWidth > 0)
        {
            taskState.frameWidth = frameWidth;
        }
        if (frameHeight > 0)
        {
            taskState.frameHeight = frameHeight;
        }
        taskState.lastFrameId = frameId;
        taskState.lastFrameTimestampMs = frameTimestampMs;
        pruneRegisteredPersonTrackingTaskStateUnlocked(taskState, frameTimestampMs);
        const json trackedObjects = payload.value("mTrackedObjectMetadatas", json::array());
        const json detectedObjects = payload.value("mDetectedObjectMetadatas", json::array());
        const json subObjects = payload.value("mSubObjectMetadatas", json::array());
        std::unordered_map<long long, const json*> personSubObjectsByParentTrackId;
        for (size_t i = 0; i < subObjects.size(); ++i)
        {
            if (!subObjects[i].is_object())
            {
                continue;
            }

            const json& subObject = subObjects[i];
            long long parentTrackId = subObject.value("mParentTrackId", -1LL);
            if (parentTrackId >= 0 && !personSubObjectsByParentTrackId.count(parentTrackId))
            {
                personSubObjectsByParentTrackId[parentTrackId] = &subObject;
            }
        }

        for (size_t i = 0; i < trackedObjects.size(); ++i)
        {
            if (!trackedObjects[i].is_object())
            {
                continue;
            }
            long long trackId = trackedObjects[i].value("mTrackId", -1LL);
            if (trackId < 0)
            {
                continue;
            }
            std::string label;
            double score = 0.0;
            std::string observedLabel;
            double observedScore = 0.0;
            int boxLeft = 0;
            int boxTop = 0;
            int boxWidth = 0;
            int boxHeight = 0;
            bool hasBox = false;
            const json* personSubObject = nullptr;
            std::unordered_map<long long, const json*>::const_iterator parentIt =
                personSubObjectsByParentTrackId.find(trackId);
            if (parentIt != personSubObjectsByParentTrackId.end())
            {
                personSubObject = parentIt->second;
            }

            if (personSubObject != nullptr)
            {
                extractBestRecognizedLabel(*personSubObject, label, score);
                extractBestObservedLabel(*personSubObject, observedLabel, observedScore);
            }
            if (label.empty())
            {
                std::string statusLabel = extractTrackStatusLabel(trackedObjects[i]);
                if (isKnownRecognizedLabel(statusLabel))
                {
                    label = statusLabel;
                }
            }
            if (observedLabel.empty())
            {
                std::string statusLabel = extractTrackStatusLabel(trackedObjects[i]);
                if (!statusLabel.empty())
                {
                    observedLabel = statusLabel;
                }
            }
            if (i < detectedObjects.size())
            {
                hasBox = extractRegisteredPersonTrackingBox(
                    detectedObjects[i], boxLeft, boxTop, boxWidth, boxHeight);
                if (hasBox &&
                    !shouldCountRegisteredPersonDetectionInCurrentPolygon(detectedObjects[i]))
                {
                    std::unordered_map<long long, RegisteredTrackState>::iterator existingTrackIt =
                        taskState.tracks.find(trackId);
                    if (existingTrackIt != taskState.tracks.end())
                    {
                        finalizeRegisteredTrackStateUnlocked(taskState, existingTrackIt->second);
                    }
                    continue;
                }
            }
            if (!hasBox)
            {
                continue;
            }
            updateRegisteredPersonTrackingTaskStateUnlocked(taskState,
                                                           trackId,
                                                           label,
                                                           score,
                                                           observedLabel,
                                                           observedScore,
                                                           frameTimestampMs,
                                                           hasBox,
                                                           boxLeft,
                                                           boxTop,
                                                           boxWidth,
                                                           boxHeight,
                                                           &pendingAlarmEvents);
        }
        pruneRegisteredPersonTrackingTaskStateUnlocked(taskState, nowMs);
        flushRegisteredPersonTrackingPersistenceUnlocked();
    }

    for (size_t i = 0; i < pendingAlarmEvents.size(); ++i)
    {
        emitRegisteredPersonTrackingAlarm(pendingAlarmEvents[i]);
    }
}

json buildRegisteredPersonTrackingTaskStatsJsonUnlocked(RegisteredPersonTrackingTaskState& taskState)
{
    ensureRegisteredPersonTrackingPersistenceLoadedUnlocked();
    pruneRegisteredPersonTrackingTaskStateUnlocked(taskState, currentTimeMillis());
    std::unordered_map<std::string, std::string> labelToName =
        buildFaceLabelToNameMap(loadFaceLabelMap());

    json taskJson;
    taskJson["taskId"] = taskState.taskId;
    taskJson["streamChannelId"] = taskState.streamChannelId;
    taskJson["frameWidth"] = taskState.frameWidth;
    taskJson["frameHeight"] = taskState.frameHeight;
    taskJson["lastFrameId"] = taskState.lastFrameId;
    taskJson["lastFrameTimestampMs"] = taskState.lastFrameTimestampMs;

    json tracksJson = json::array();
    int currentRegisteredTrackCount = 0;
    for (auto it = taskState.tracks.begin(); it != taskState.tracks.end(); ++it)
    {
        const RegisteredTrackState& trackState = it->second;
        if (trackState.active && trackState.counted && !trackState.confirmedLabel.empty())
        {
            currentRegisteredTrackCount += 1;
        }
        std::string debugLabel = trackState.confirmedLabel;
        std::string recognitionState = "confirmed";
        if (debugLabel.empty() && !trackState.pendingLabel.empty())
        {
            recognitionState = "pending";
        }
        else if (debugLabel.empty() && !trackState.lastObservedLabel.empty())
        {
            debugLabel = trackState.lastObservedLabel;
            recognitionState = isKnownRecognizedLabel(trackState.lastObservedLabel) ? "observed" : "unknown";
        }
        if (debugLabel.empty() && recognitionState != "pending")
        {
            if (!trackState.active)
            {
                continue;
            }
            recognitionState = "unknown";
        }
        std::string displayName = "未匹配";
        std::string labelText = "未匹配";
        if (recognitionState == "pending")
        {
            displayName = "待确认";
            labelText = "待确认";
        }
        else if (recognitionState == "unknown")
        {
            displayName = "未匹配";
            labelText = "未匹配";
        }
        else
        {
            std::string resolvedName = debugLabel;
            std::unordered_map<std::string, std::string>::const_iterator labelIt =
                labelToName.find(debugLabel);
            if (labelIt != labelToName.end() && !labelIt->second.empty())
            {
                resolvedName = labelIt->second;
            }
            displayName = resolvedName;
            labelText = recognitionState == "observed"
                            ? "观察到：" + resolvedName
                            : "匹配成功：" + resolvedName;
        }
        tracksJson.push_back({
            {"trackId", trackState.trackId},
            {"recordId", trackState.recordId},
            {"active", trackState.active},
            {"confirmedLabel", trackState.confirmedLabel},
            {"displayName", displayName},
            {"labelText", labelText},
            {"recognitionState", recognitionState},
            {"pendingLabel", trackState.pendingLabel},
            {"observedLabel", trackState.lastObservedLabel},
            {"observedScore", trackState.lastObservedScore},
            {"firstSeenMs", trackState.firstSeenMs},
            {"enterTimeMs", trackState.enterTimeMs},
            {"lastSeenMs", trackState.lastSeenMs},
            {"confirmTimeMs", trackState.confirmTimeMs},
            {"exitTimeMs", trackState.exitTimeMs},
            {"bestScore", trackState.bestScore},
            {"counted", trackState.counted},
            {"hasBox", trackState.hasBox},
            {"box", {
                {"left", trackState.boxLeft},
                {"top", trackState.boxTop},
                {"width", trackState.boxWidth},
                {"height", trackState.boxHeight}
            }}
        });
    }

    json recordsJson = json::array();
    for (size_t i = 0; i < taskState.records.size(); ++i)
    {
        const RegisteredPersonVisitRecord& record = taskState.records[i];
        recordsJson.push_back({
            {"recordId", record.recordId},
            {"trackId", record.trackId},
            {"label", record.label},
            {"firstSeenMs", record.firstSeenMs},
            {"enterTimeMs", record.enterTimeMs},
            {"confirmTimeMs", record.confirmTimeMs},
            {"lastSeenMs", record.lastSeenMs},
            {"exitTimeMs", record.exitTimeMs},
            {"durationMs", record.durationMs}
        });
    }

    json personsJson = json::array();
    int currentRegisteredPersonCount = 0;
    for (auto it = taskState.persons.begin(); it != taskState.persons.end(); ++it)
    {
        const RegisteredPersonAggregate& aggregate = it->second;
        bool inside = !aggregate.activeTrackIds.empty();
        if (inside)
        {
            currentRegisteredPersonCount += 1;
        }
        std::string displayName = it->first;
        std::unordered_map<std::string, std::string>::const_iterator labelIt =
            labelToName.find(it->first);
        if (labelIt != labelToName.end() && !labelIt->second.empty())
        {
            displayName = labelIt->second;
        }
        personsJson.push_back({
            {"label", it->first},
            {"displayName", displayName},
            {"entries", aggregate.entries},
            {"exits", aggregate.exits},
            {"reentries", std::max(0, aggregate.entries - 1)},
            {"currentInside", inside},
            {"activeTrackCount", static_cast<int>(aggregate.activeTrackIds.size())},
            {"firstSeenMs", aggregate.firstSeenMs},
            {"lastSeenMs", aggregate.lastSeenMs},
            {"lastEnterMs", aggregate.lastEnterMs},
            {"lastExitMs", aggregate.lastExitMs},
            {"totalDwellMs", aggregate.totalDwellMs}
        });
    }

    taskJson["currentRegisteredTrackCount"] = currentRegisteredTrackCount;
    taskJson["currentRegisteredPersonCount"] = currentRegisteredPersonCount;
    taskJson["records"] = recordsJson;
    taskJson["persons"] = personsJson;
    taskJson["tracks"] = tracksJson;
    return taskJson;
}

void route_polygon(Server &svr)
{
    auto respondDeprecated = [](Response &res)
    {
        json responseJson = {
            {"status", "deprecated"},
            {"message", "Legacy stream polygon overlay has been removed. Please use task region configuration instead."}
        };
        res.status = 410;
        res.set_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
        res.set_content(responseJson.dump(), "application/json");
    };

    svr.Get("/stream/polygon/:id", [respondDeprecated](const Request &, Response &res)
    {
        respondDeprecated(res);
    });

    svr.Post("/stream/polygon/:id", [respondDeprecated](const Request &, Response &res)
    {
        respondDeprecated(res);
    });
}

// ----------------------------------------------------------------------------

// 人员检测模块
namespace HumanDetection {
    // 人员检测状态结构体
    struct HumanDetectionStatus {
        bool channel20_detected = false;
        bool channel10_detected = false;
        std::string timestamp;
    };

    // 读取GPIO文件状态
    bool readGPIOStatus(const std::string& gpioPath) {
        std::ifstream gpioFile(gpioPath);
        if (gpioFile.is_open()) {
            std::string value;
            std::getline(gpioFile, value);
            gpioFile.close();
            return (value == "1");
        }
        return false;
    }

    // 获取当前时间戳
    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }

    // 处理human_detected API请求
    void handleHumanDetectedAPI(const Request& req, Response& res) {
        try {
            // 读取GPIO状态
            bool channel20_detected = readGPIOStatus("/sys/class/gpio/gpio429/value");
            bool channel10_detected = readGPIOStatus("/sys/class/gpio/gpio430/value");
            
            // 构建响应JSON
            json responseJson;
            responseJson["status"] = "success";
            responseJson["timestamp"] = getCurrentTimestamp();
            responseJson["channels"] = json::array();
            
            // 添加通道20的状态
            json channel20;
            channel20["channel_id"] = 20;
            channel20["human_detected"] = !channel20_detected;  // 反转逻辑：GPIO为0表示检测到人
            channel20["gpio_pin"] = 429;
            responseJson["channels"].push_back(channel20);
            
            // 添加通道10的状态
            json channel10;
            channel10["channel_id"] = 10;
            channel10["human_detected"] = !channel10_detected;  // 反转逻辑：GPIO为0表示检测到人
            channel10["gpio_pin"] = 430;
            responseJson["channels"].push_back(channel10);

            // 设置CORS头
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
            res.set_header("Access-Control-Allow-Headers", "Content-Type");
            
            // 返回JSON响应
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            errorJson["timestamp"] = getCurrentTimestamp();
            
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    }
}

// 烟火检测模块
namespace FireDetection {
    // 读取GPIO文件状态
    bool readGPIOStatus(const std::string& gpioPath) {
        std::ifstream gpioFile(gpioPath);
        if (gpioFile.is_open()) {
            std::string value;
            std::getline(gpioFile, value);
            gpioFile.close();
            return (value == "1");
        }
        return false;
    }

    // 获取当前时间戳
    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }

    // 处理fire_detected API请求
    void handleFireDetectedAPI(const Request& req, Response& res) {
        try {
            // 读取GPIO 344状态（烟火检测）
            bool gpio_status = readGPIOStatus("/sys/class/gpio/gpio344/value");
            
            // 构建响应JSON
            json responseJson;
            responseJson["status"] = "success";
            responseJson["timestamp"] = getCurrentTimestamp();
            responseJson["fire_detected"] = !gpio_status;  // 反转逻辑：GPIO为0表示检测到火焰/烟雾
            responseJson["gpio_pin"] = 344;
            responseJson["gpio_value"] = gpio_status ? 1 : 0;

            // 设置CORS头
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
            res.set_header("Access-Control-Allow-Headers", "Content-Type");
            
            // 返回JSON响应
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            errorJson["timestamp"] = getCurrentTimestamp();
            
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    }
}

std::string time_local()
{
    auto p = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(p);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%d/%b/%Y:%H:%M:%S %z");
    return ss.str();
}
//----------------------------------------------------------------------------------------------------------------------------
std::string log(const Request &req, const Response &res)
{
    return "[" + time_local() + "] " + req.remote_addr + " " +
           std::to_string(res.status) + " " + req.method + " " + req.path;
}
//----------------------------------------------------------------------------------------------------------------------------
namespace JsonFile2
{
    const std::string kYoloConfigDir = SOPHON_STREAM_YOLO_CONFIG_DIR;
    const std::string kYoloClassNamesDefault = SOPHON_STREAM_YOLO_DATA_DIR + "/coco.names";

    std::string resolveConfigPath(const std::string& path)
    {
        if (path.empty())
        {
            return kYoloClassNamesDefault;
        }
        if (!path.empty() && path[0] == '/')
        {
            return path;
        }
        return kYoloConfigDir + "/" + path;
    }

    std::set<std::string> loadClassNameSet(const std::string& classFilePath)
    {
        std::set<std::string> names;
        std::ifstream in(classFilePath.c_str());
        if (!in.is_open())
        {
            return names;
        }

        std::string line;
        while (std::getline(in, line))
        {
            line = trimString(line);
            if (!line.empty())
            {
                names.insert(line);
            }
        }
        return names;
    }

    std::set<std::string> loadClassNameSetFromThresholdConfig(const json& data)
    {
        std::string classNamesPath = kYoloClassNamesDefault;
        if (data.contains("configure") &&
            data["configure"].is_object() &&
            data["configure"].contains("class_names_file") &&
            data["configure"]["class_names_file"].is_string())
        {
            classNamesPath = resolveConfigPath(data["configure"]["class_names_file"].get<std::string>());
        }

        std::set<std::string> names = loadClassNameSet(classNamesPath);
        if (names.empty() && classNamesPath != kYoloClassNamesDefault)
        {
            names = loadClassNameSet(kYoloClassNamesDefault);
        }
        return names;
    }

    bool sanitizeThresholdConf(json& data, const std::set<std::string>& classNames, int* removedCount = nullptr)
    {
        if (removedCount)
        {
            *removedCount = 0;
        }
        if (!data.contains("configure") || !data["configure"].is_object() || classNames.empty())
        {
            return false;
        }

        json& configure = data["configure"];
        json oldConf = json::object();
        if (configure.contains("threshold_conf") && configure["threshold_conf"].is_object())
        {
            oldConf = configure["threshold_conf"];
        }

        json cleanedConf = json::object();
        for (std::set<std::string>::const_iterator it = classNames.begin(); it != classNames.end(); ++it)
        {
            double value = (*it == "person") ? 0.3 : 1.0;
            if (oldConf.contains(*it) && oldConf[*it].is_number())
            {
                value = oldConf[*it].get<double>();
            }
            cleanedConf[*it] = value;
        }

        if (removedCount)
        {
            for (json::iterator it = oldConf.begin(); it != oldConf.end(); ++it)
            {
                if (classNames.find(it.key()) == classNames.end())
                {
                    (*removedCount)++;
                }
            }
        }

        bool changed = (oldConf != cleanedConf);
        configure["threshold_conf"] = cleanedConf;
        return changed;
    }

    // 加载 JSON 文件并更新全局变量
    void loadJsonData(json &data)
    {
        std::ifstream jsonFile((SOPHON_STREAM_YOLO_CONFIG_DIR + "/yolov8_classthresh_roi_example.json").c_str());
        if (jsonFile.is_open())
        {
            jsonFile >> data;
            jsonFile.close();
        }
        else
        {
            std::cerr << "Error opening JSON file for reading." << std::endl;
        }
    }
    //----------------------------------------------------------------------------------------------------------------------------
    // 更新 JSON 文件
    void updateJsonFile(const json &data)
    {
        std::ofstream jsonFile((SOPHON_STREAM_YOLO_CONFIG_DIR + "/yolov8_classthresh_roi_example.json").c_str());
        if (jsonFile.is_open())
        {
            jsonFile << data.dump(4);
            jsonFile.close();
            std::cout << "[配置更新] 已写入阈值配置文件，不再自动执行run.sh" << std::endl;
        }
        else
        {
            std::cerr << "Error opening JSON file for writing." << std::endl;
        }
    }
    //----------------------------------------------------------------------------------------------------------------------------
    // 加载 JSON 文件并更新全局变量
    void loadJsonDataForDemo(json &data)
    {
        std::ifstream jsonFile((SOPHON_STREAM_YOLO_CONFIG_DIR + "/yolov8_demo.json").c_str());
        if (jsonFile.is_open())
        {
            jsonFile >> data;
            jsonFile.close();
        }
        else
        {
            std::cerr << "Error opening JSON file for reading." << std::endl;
        }
    }
    //----------------------------------------------------------------------------------------------------------------------------
    // 更新 JSON 文件
    void updateJsonFileForDemo(const json &data)
    {
        std::ofstream jsonFile((SOPHON_STREAM_YOLO_CONFIG_DIR + "/yolov8_demo.json").c_str());
        if (jsonFile.is_open())
        {
            jsonFile << data.dump(4);
            jsonFile.close();
        }
        else
        {
            std::cerr << "Error opening JSON file for writing." << std::endl;
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------
// 启动ffmpeg命令进行录制，接收视频流地址、保存位置以及分段时间参数（以秒为单位）
void startRecording(const std::string &rtspStreamUrl = "", const std::string &saveLocation = "",
                    const std::string &rtspStreamUrl2 = "", const std::string &saveLocation2 = "",
                    int segmentTimeOverride = -1,
                    bool dualStreamEnabled = true)
{
    refreshRecordingRuntimeState();
    std::lock_guard<std::mutex> lock(recordingStateMutex);
    if (isFfmpegRunning || isFfmpegRunning2)
    {
        throw std::runtime_error("Recording is already running.");
    }

    json defaultAddresses;
    readDefaultAddressesFromJson(defaultAddresses);
    ensureDefaultAddressSections(defaultAddresses);
    std::string actualRtspStreamUrl =
        trimString(rtspStreamUrl.empty() ? defaultAddresses["current"].value("defaultRtspStreamUrl", "") : rtspStreamUrl);
    std::string actualSaveLocation =
        trimString(saveLocation.empty() ? defaultAddresses["current"].value("defaultSaveLocation", "") : saveLocation);
    std::string actualRtspStreamUrl2 =
        trimString(rtspStreamUrl2.empty() ? defaultAddresses["current"].value("defaultRtspStreamUrl2", "") : rtspStreamUrl2);
    std::string actualSaveLocation2 =
        trimString(saveLocation2.empty() ? defaultAddresses["current"].value("defaultSaveLocation2", "") : saveLocation2);
    int segmentTime = sanitizeRecordingSegmentTime(segmentTimeOverride);

    bool useDualStream = dualStreamEnabled;
    if (!useDualStream &&
        (actualRtspStreamUrl.empty() || actualSaveLocation.empty()) &&
        !actualRtspStreamUrl2.empty() && !actualSaveLocation2.empty())
    {
        actualRtspStreamUrl = actualRtspStreamUrl2;
        actualSaveLocation = actualSaveLocation2;
        actualRtspStreamUrl2.clear();
        actualSaveLocation2.clear();
    }

    if (actualRtspStreamUrl.empty() || actualSaveLocation.empty())
    {
        throw std::runtime_error("Recording configuration is incomplete.");
    }

    if (useDualStream && (actualRtspStreamUrl2.empty() || actualSaveLocation2.empty()))
    {
        throw std::runtime_error("Dual-stream recording requires both channel configurations.");
    }

    if (!ensureDirectory(actualSaveLocation))
    {
        throw std::runtime_error("Cannot access recording directory (missing or not writable by current user): " + actualSaveLocation);
    }
    if (useDualStream && !ensureDirectory(actualSaveLocation2))
    {
        throw std::runtime_error("Cannot access recording directory (missing or not writable by current user): " + actualSaveLocation2);
    }

    removeFileIfExists(RECORDING_PID_FILE_1);
    removeFileIfExists(RECORDING_PID_FILE_2);

    int pid1 = 0;
    int pid2 = 0;
    std::string errorMessage;

    if (!launchRecordingChannel(actualRtspStreamUrl, actualSaveLocation, segmentTime, RECORDING_LOG_FILE_1, pid1, errorMessage))
    {
        recordingLastError = errorMessage;
        writeRecordingStatusSnapshot(false, false, 0, recordingLastError);
        throw std::runtime_error(errorMessage);
    }
    if (!writePidFile(RECORDING_PID_FILE_1, pid1))
    {
        stopManagedProcess(pid1, "录制通道1");
        recordingLastError = "Cannot write PID file for recording channel 1.";
        writeRecordingStatusSnapshot(false, false, 0, recordingLastError);
        throw std::runtime_error(recordingLastError);
    }

    std::cout << "[录制] 通道1已启动, PID=" << pid1 << ", RTSP=" << actualRtspStreamUrl << std::endl;
    bool channel2Started = false;
    if (useDualStream)
    {
        if (!launchRecordingChannel(actualRtspStreamUrl2, actualSaveLocation2, segmentTime, RECORDING_LOG_FILE_2, pid2, errorMessage))
        {
            stopManagedProcess(pid1, "录制通道1");
            removeFileIfExists(RECORDING_PID_FILE_1);
            recordingLastError = errorMessage;
            writeRecordingStatusSnapshot(false, false, 0, recordingLastError);
            throw std::runtime_error(errorMessage);
        }
        if (!writePidFile(RECORDING_PID_FILE_2, pid2))
        {
            stopManagedProcess(pid1, "录制通道1");
            stopManagedProcess(pid2, "录制通道2");
            removeFileIfExists(RECORDING_PID_FILE_1);
            removeFileIfExists(RECORDING_PID_FILE_2);
            recordingLastError = "Cannot write PID file for recording channel 2.";
            writeRecordingStatusSnapshot(false, false, 0, recordingLastError);
            throw std::runtime_error(recordingLastError);
        }

        channel2Started = true;
        std::cout << "[录制] 通道2已启动, PID=" << pid2 << ", RTSP=" << actualRtspStreamUrl2 << std::endl;
    }
    else
    {
        removeFileIfExists(RECORDING_PID_FILE_2);
    }

    isFfmpegRunning = true;
    isFfmpegRunning2 = channel2Started;
    activeRecordingRtspUrl1 = actualRtspStreamUrl;
    activeRecordingRtspUrl2 = channel2Started ? actualRtspStreamUrl2 : "";
    activeRecordingSaveLocation1 = actualSaveLocation;
    activeRecordingSaveLocation2 = channel2Started ? actualSaveLocation2 : "";
    startRecordingTime = std::chrono::system_clock::now();
    recordingLastError.clear();
    long long startTimeSeconds = static_cast<long long>(std::time(nullptr));
    writeRecordingStatusSnapshot(true, channel2Started, startTimeSeconds, recordingLastError);
}
//----------------------------------------------------------------------------------------------------------------------------
// 停止ffmpeg命令的录制
void stopRecording()
{
    std::lock_guard<std::mutex> lock(recordingStateMutex);

    int pid1 = 0;
    int pid2 = 0;
    if (isRecordingProcessActive(RECORDING_PID_FILE_1, &pid1))
    {
        stopManagedProcess(pid1, "录制通道1");
    }
    if (isRecordingProcessActive(RECORDING_PID_FILE_2, &pid2))
    {
        stopManagedProcess(pid2, "录制通道2");
    }

    removeFileIfExists(RECORDING_PID_FILE_1);
    removeFileIfExists(RECORDING_PID_FILE_2);

    isFfmpegRunning = false;
    isFfmpegRunning2 = false;
    activeRecordingRtspUrl1.clear();
    activeRecordingRtspUrl2.clear();
    activeRecordingSaveLocation1.clear();
    activeRecordingSaveLocation2.clear();
    recordingLastError.clear();
    startRecordingTime = std::chrono::system_clock::time_point();
    writeRecordingStatusSnapshot(false, false, 0, "");
}
//----------------------------------------------------------------------------------------------------------------------------
// 告警远程上报功能

// 解析URL，提取host、port和path
struct ParsedUrl {
    std::string protocol; // http 或 https
    std::string host;
    int port;
    std::string path;
    bool valid;
};

// 读取远程上报地址配置
std::string getRemoteAlarmUrl()
{
    try {
        std::ifstream file("/data/lintech/celectronicfence/params.json");
        if (file.is_open()) {
            json paramsJson;
            file >> paramsJson;
            file.close();
            
            // 读取远程上报地址参数（支持多种可能的key名称）
            std::string url = "";
            
            // 优先读取 RemoteInfo（与前端参数标识一致）
            if (paramsJson.contains("RemoteInfo") && paramsJson["RemoteInfo"].is_string()) {
                url = paramsJson["RemoteInfo"].get<std::string>();
            }
            // 兼容旧版本的 remoteAlarmUrl
            else if (paramsJson.contains("remoteAlarmUrl") && paramsJson["remoteAlarmUrl"].is_string()) {
                url = paramsJson["remoteAlarmUrl"].get<std::string>();
            }
            
            // 如果URL不为空，返回它
            if (!url.empty()) {
                std::cout << "[远程上报] 读取到配置地址: " << url << std::endl;
                return url;
            } else {
                std::cout << "[远程上报] 配置地址为空" << std::endl;
            }
        } else {
            std::cerr << "[远程上报] 无法打开params.json文件" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[远程上报] 读取配置失败: " << e.what() << std::endl;
    }
    return ""; // 返回空字符串表示未配置
}

int readAlarmTriggerCooldownSecondsFromParams(int fallbackValue = 10)
{
    int cooldownSeconds = fallbackValue;
    try
    {
        std::ifstream file("/data/lintech/celectronicfence/params.json");
        if (file.is_open())
        {
            json paramsJson;
            file >> paramsJson;
            file.close();
            if (paramsJson.contains("AlarmCooldownTime"))
            {
                cooldownSeconds = parseJsonInt(paramsJson["AlarmCooldownTime"], cooldownSeconds);
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "[告警参数] 读取 AlarmCooldownTime 失败: " << e.what() << std::endl;
    }
    return clampInt(cooldownSeconds, 0, 300);
}

int readAlarmVideoDurationSecondsFromParams()
{
    int durationSec = 10;
    try
    {
        std::ifstream file("/data/lintech/celectronicfence/params.json");
        if (file.is_open())
        {
            json paramsJson;
            file >> paramsJson;
            file.close();
            if (paramsJson.contains("AlarmVideoDurationSec"))
            {
                durationSec = parseJsonInt(paramsJson["AlarmVideoDurationSec"], durationSec);
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "[告警参数] 读取 AlarmVideoDurationSec 失败: " << e.what() << std::endl;
    }
    return clampInt(durationSec, 3, 60);
}

bool isAlarmVideoClipEnabled()
{
    bool enabled = true;
    try
    {
        std::ifstream file("/data/lintech/celectronicfence/params.json");
        if (file.is_open())
        {
            json paramsJson;
            file >> paramsJson;
            file.close();
            if (paramsJson.contains("EnableAlarmVideoClip"))
            {
                const json& value = paramsJson["EnableAlarmVideoClip"];
                if (value.is_boolean())
                {
                    enabled = value.get<bool>();
                }
                else if (value.is_number_integer())
                {
                    enabled = value.get<int>() != 0;
                }
                else if (value.is_string())
                {
                    std::string text = trimString(value.get<std::string>());
                    for (size_t i = 0; i < text.size(); ++i)
                    {
                        text[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(text[i])));
                    }
                    enabled = !(text == "0" || text == "false" || text == "no" || text == "off");
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "[告警参数] 读取 EnableAlarmVideoClip 失败: " << e.what() << std::endl;
    }
    return enabled;
}

ParsedUrl parseUrl(const std::string& url)
{
    ParsedUrl result;
    result.valid = false;
    result.port = 80; // 默认端口
    
    if (url.empty()) {
        return result;
    }
    
    try {
        // 查找协议
        size_t protocolEnd = url.find("://");
        if (protocolEnd == std::string::npos) {
            // 如果没有协议，默认使用http
            result.protocol = "http";
            protocolEnd = 0;
        } else {
            result.protocol = url.substr(0, protocolEnd);
            protocolEnd += 3; // 跳过 "://"
        }
        
        // 设置默认端口
        if (result.protocol == "https") {
            result.port = 443;
        }
        
        // 查找路径开始位置
        size_t pathStart = url.find("/", protocolEnd);
        size_t hostEnd = (pathStart == std::string::npos) ? url.length() : pathStart;
        
        // 提取host和port
        std::string hostPort = url.substr(protocolEnd, hostEnd - protocolEnd);
        size_t colonPos = hostPort.find(":");
        if (colonPos != std::string::npos) {
            result.host = hostPort.substr(0, colonPos);
            result.port = std::stoi(hostPort.substr(colonPos + 1));
        } else {
            result.host = hostPort;
        }
        
        // 提取path
        if (pathStart != std::string::npos) {
            result.path = url.substr(pathStart);
        } else {
            result.path = "/";
        }
        
        result.valid = !result.host.empty();
    } catch (const std::exception& e) {
        std::cerr << "[远程上报] URL解析失败: " << e.what() << std::endl;
        result.valid = false;
    }
    
    return result;
}

// 将文件内容转换为base64编码
std::string fileToBase64(const std::string& filePath)
{
    try {
        std::ifstream file(filePath, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "[远程上报] 无法打开图片文件: " << filePath << std::endl;
            return "";
        }
        
        // 读取文件内容
        std::string fileContent((std::istreambuf_iterator<char>(file)),
                                std::istreambuf_iterator<char>());
        file.close();
        
        if (fileContent.empty()) {
            std::cerr << "[远程上报] 图片文件为空: " << filePath << std::endl;
            return "";
        }
        
        // Base64编码表
        const std::string base64_chars = 
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        
        std::string base64;
        int val = 0, valb = -6;
        for (unsigned char c : fileContent) {
            val = (val << 8) + c;
            valb += 8;
            while (valb >= 0) {
                base64.push_back(base64_chars[(val >> valb) & 0x3F]);
                valb -= 6;
            }
        }
        if (valb > -6) {
            base64.push_back(base64_chars[((val << 8) >> (valb + 8)) & 0x3F]);
        }
        while (base64.size() % 4) {
            base64.push_back('=');
        }
        
        std::cout << "[远程上报] 图片转换为base64成功，大小: " << base64.size() << " 字符" << std::endl;
        return base64;
    } catch (const std::exception& e) {
        std::cerr << "[远程上报] 图片转base64失败: " << e.what() << std::endl;
        return "";
    }
}

// 异步上报告警到远程地址
void reportAlarmToRemote(const json& alarmData)
{
    // 在独立线程中执行上报，避免阻塞主线程
    std::thread reportThread([alarmData]() {
        try {
            // 读取远程上报地址
            std::string remoteUrl = getRemoteAlarmUrl();
            if (remoteUrl.empty())
            {
                remoteUrl = trimString(alarmData.value("reportUrl", ""));
            }
            if (remoteUrl.empty()) {
                std::cout << "[远程上报] 未配置远程上报地址，跳过上报" << std::endl;
                return;
            }
            
            std::cout << "[远程上报] 开始上报告警到: " << remoteUrl << std::endl;
            
            // 解析URL
            ParsedUrl parsed = parseUrl(remoteUrl);
            if (!parsed.valid) {
                std::cerr << "[远程上报] URL格式无效: " << remoteUrl << std::endl;
                return;
            }
            
            std::cout << "[远程上报] 解析结果 - 协议: " << parsed.protocol 
                      << ", 主机: " << parsed.host 
                      << ", 端口: " << parsed.port 
                      << ", 路径: " << parsed.path << std::endl;
            
            // 准备要发送的数据（使用最新的告警数据）
            json dataToSend = alarmData;
            
            // 1. 读取最新的告警状态（包括更新后的reportStatus）
            int alarmId = alarmData.value("id", 0);
            if (alarmId > 0)
            {
                json latestAlarm;
                if (readAlarmRecordById(alarmId, latestAlarm) && latestAlarm.is_object())
                {
                    dataToSend = latestAlarm;
                }
            }
            
            // 2. 将reportStatus映射为status值（无论是否读取到最新状态，都要映射）
            std::string reportStatus = dataToSend.value("reportStatus", "");
            std::string statusForRemote = "pending"; // 默认值
            
            if (reportStatus == "上报成功") {
                statusForRemote = "reported";
            } else if (reportStatus == "上报失败") {
                statusForRemote = "failed";
            } else if (reportStatus == "上报中" || reportStatus == "未上报" || reportStatus.empty()) {
                statusForRemote = "pending";
            }
            
            // 保存原始status（告警处理状态）到processStatus字段（如果存在）
            std::string originalProcessStatus =
                trimString(dataToSend.value("processStatus", ""));
            if (originalProcessStatus.empty())
            {
                originalProcessStatus = trimString(alarmData.value("processStatus", ""));
            }
            if (originalProcessStatus.empty())
            {
                originalProcessStatus = trimString(alarmData.value("status", ""));
            }
            if (originalProcessStatus == "未处理" || originalProcessStatus == "已处理") {
                dataToSend["processStatus"] = originalProcessStatus;
            }
            
            // 设置映射后的status（用于远程服务器，表示上报状态）
            dataToSend["status"] = statusForRemote;
            
            std::cout << "[远程上报] 准备发送数据: reportStatus=" 
                      << reportStatus
                      << ", status=" << statusForRemote << std::endl;
            
            // 2. 读取图片文件并转换为base64
            std::string imageUrl = normalizeAlarmImageUrl(dataToSend.value("imageUrl", ""));
            if (!imageUrl.empty()) {
                std::string normalizedUrl;
                std::string imagePath;
                if (!buildLocalAlarmImagePath(imageUrl, normalizedUrl, imagePath))
                {
                    imagePath.clear();
                }
                else
                {
                    dataToSend["imageUrl"] = normalizedUrl;
                }

                if (!imagePath.empty())
                {
                    std::cout << "[远程上报] 读取图片文件: " << imagePath << std::endl;
                }
                std::string base64Image = imagePath.empty() ? "" : fileToBase64(imagePath);
                if (!base64Image.empty()) {
                    // 添加base64图片数据到JSON
                    dataToSend["imageBase64"] = base64Image;
                    // 添加图片MIME类型（根据文件扩展名判断）
                    if (imagePath.find(".jpg") != std::string::npos || 
                        imagePath.find(".jpeg") != std::string::npos) {
                        dataToSend["imageMimeType"] = "image/jpeg";
                    } else if (imagePath.find(".png") != std::string::npos) {
                        dataToSend["imageMimeType"] = "image/png";
                    } else {
                        dataToSend["imageMimeType"] = "image/jpeg"; // 默认
                    }
                    std::cout << "[远程上报] 图片已转换为base64并添加到数据中" << std::endl;
                } else {
                    std::cerr << "[远程上报] 图片转base64失败，将只发送URL" << std::endl;
                }
            }

            std::string videoUrl = normalizeAlarmVideoUrl(dataToSend.value("videoUrl", ""));
            if (!videoUrl.empty())
            {
                std::string normalizedVideoUrl;
                std::string videoPath;
                if (buildLocalAlarmVideoPath(videoUrl, normalizedVideoUrl, videoPath))
                {
                    dataToSend["videoUrl"] = normalizedVideoUrl;
                    if (dataToSend.value("videoMimeType", "").empty())
                    {
                        dataToSend["videoMimeType"] = guessRecordingMimeType(videoPath);
                    }
                }
            }
            
            // 创建HTTP/HTTPS客户端并发送POST请求
            std::string contentType = "application/json";
            bool requestSuccess = false;
            int statusCode = 0;
            std::string responseBody;
            
            if (parsed.protocol == "https") {
                // 使用HTTPS客户端
                #ifdef CPPHTTPLIB_OPENSSL_SUPPORT
                httplib::SSLClient cli(parsed.host.c_str(), parsed.port);
                cli.set_connection_timeout(10, 0);
                cli.set_read_timeout(10, 0);
                cli.enable_server_certificate_verification(false); // 禁用证书验证（可根据需要启用）
                auto res = cli.Post(parsed.path.c_str(), dataToSend.dump(), contentType.c_str());
                if (res) {
                    requestSuccess = true;
                    statusCode = res->status;
                    responseBody = res->body;
                }
                #else
                std::cerr << "[远程上报] 错误：当前编译版本不支持HTTPS，请使用HTTP或重新编译httplib with OpenSSL支持" << std::endl;
                return;
                #endif
            } else {
                // 使用HTTP客户端
                httplib::Client cli(parsed.host.c_str(), parsed.port);
                cli.set_connection_timeout(10, 0); // 10秒连接超时
                cli.set_read_timeout(10, 0);       // 10秒读取超时
                auto res = cli.Post(parsed.path.c_str(), dataToSend.dump(), contentType.c_str());
                if (res) {
                    requestSuccess = true;
                    statusCode = res->status;
                    responseBody = res->body;
                }
            }
            
            // 更新告警记录中的上报状态
            reportStatus = "上报失败";
            if (requestSuccess) {
                std::cout << "[远程上报] 上报成功，状态码: " << statusCode << std::endl;
                if (statusCode >= 200 && statusCode < 300) {
                    std::cout << "[远程上报] 告警已成功上报到远程服务器" << std::endl;
                    reportStatus = "上报成功";
                } else {
                    std::cerr << "[远程上报] 上报失败，状态码: " << statusCode 
                              << ", 响应: " << responseBody.substr(0, 200) << std::endl;
                    reportStatus = "上报失败";
                }
            } else {
                std::cerr << "[远程上报] HTTP请求失败：无响应或连接失败" << std::endl;
                reportStatus = "上报失败";
            }
            
            // 更新告警文件中的上报状态
            bool statusUpdated = false;
            try {
                int alarmId = alarmData.value("id", 0);
                json patch;
                patch["reportStatus"] = reportStatus;
                json updatedAlarm;
                statusUpdated = updateAlarmRecordById(alarmId, patch, &updatedAlarm);
                if (statusUpdated) {
                    std::cout << "[远程上报] 更新告警ID " << alarmId
                              << " 的上报状态为: " << reportStatus << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "[远程上报] 更新告警状态失败: " << e.what() << std::endl;
            }
            
            // 如果上报成功且状态更新成功，再次发送一次更新后的状态（只发送状态更新，不包含图片）
            if (requestSuccess && statusCode >= 200 && statusCode < 300 && statusUpdated) {
                try {
                    // 等待一小段时间，确保文件写入完成
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    
                    int alarmId = alarmData.value("id", 0);
                    json updatedAlarm;
                    if (readAlarmRecordById(alarmId, updatedAlarm)) {
                            // 准备状态更新数据（只包含关键字段，不包含图片base64）
                            json statusUpdate;
                            statusUpdate["id"] = updatedAlarm.value("id", 0);
                            statusUpdate["reportStatus"] = updatedAlarm.value("reportStatus", "");
                            
                            // 将reportStatus映射为status值
                            std::string reportStatus = updatedAlarm.value("reportStatus", "");
                            std::string statusForRemote = "pending";
                            if (reportStatus == "上报成功") {
                                statusForRemote = "reported";
                            } else if (reportStatus == "上报失败") {
                                statusForRemote = "failed";
                            } else if (reportStatus == "上报中" || reportStatus == "未上报") {
                                statusForRemote = "pending";
                            }
                            statusUpdate["status"] = statusForRemote;
                            
                            statusUpdate["timestamp"] = updatedAlarm.value("timestamp", "");
                            statusUpdate["alarmType"] = updatedAlarm.value("alarmType", "");
                            statusUpdate["videoSourceId"] = updatedAlarm.value("videoSourceId", 0);
                            statusUpdate["videoSourceName"] = updatedAlarm.value("videoSourceName", "");
                            statusUpdate["reportUrl"] = updatedAlarm.value("reportUrl", "");
                            statusUpdate["imageUrl"] = updatedAlarm.value("imageUrl", "");
                            statusUpdate["videoUrl"] = updatedAlarm.value("videoUrl", "");
                            statusUpdate["videoDurationSec"] = updatedAlarm.value("videoDurationSec", 0);
                            statusUpdate["videoMimeType"] = updatedAlarm.value("videoMimeType", "");
                            statusUpdate["videoStatus"] = updatedAlarm.value("videoStatus", "");
                            // 不包含imageBase64，只发送状态更新
                            
                            std::cout << "[远程上报] 发送状态更新: reportStatus=" 
                                      << statusUpdate.value("reportStatus", "")
                                      << ", status=" << statusForRemote << std::endl;
                            
                            // 发送状态更新
                            if (parsed.protocol == "https") {
                                #ifdef CPPHTTPLIB_OPENSSL_SUPPORT
                                httplib::SSLClient cli(parsed.host.c_str(), parsed.port);
                                cli.set_connection_timeout(10, 0);
                                cli.set_read_timeout(10, 0);
                                cli.enable_server_certificate_verification(false);
                                auto res = cli.Post(parsed.path.c_str(), statusUpdate.dump(), contentType.c_str());
                                if (res && res->status >= 200 && res->status < 300) {
                                    std::cout << "[远程上报] 状态更新发送成功" << std::endl;
                                }
                                #endif
                            } else {
                                httplib::Client cli(parsed.host.c_str(), parsed.port);
                                cli.set_connection_timeout(10, 0);
                                cli.set_read_timeout(10, 0);
                                auto res = cli.Post(parsed.path.c_str(), statusUpdate.dump(), contentType.c_str());
                                if (res && res->status >= 200 && res->status < 300) {
                                    std::cout << "[远程上报] 状态更新发送成功" << std::endl;
                                }
                            }
                    }
                } catch (const std::exception& e) {
                    std::cerr << "[远程上报] 发送状态更新失败: " << e.what() << std::endl;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[远程上报] 上报异常: " << e.what() << std::endl;
        }
    });
    
    // 分离线程，让它在后台运行
    reportThread.detach();
}

std::atomic<bool> stopRegionAlarmMonitor(false);
std::mutex regionAlarmStateMutex;
std::map<int, bool> regionTaskDetectedState;
std::map<int, std::chrono::time_point<std::chrono::system_clock>> regionTaskLastAlarmTime;

int readAlarmCooldownSecondsFromParams()
{
    // 对齐旧项目行为：区域入侵告警固定 5 秒冷却。
    return 5;
}

std::string resolveVideoSourceNameById(int videoSourceId)
{
    std::ifstream in("/data/lintech/celectronicfence/channels.json");
    if (!in.is_open())
    {
        return "VideoSource-" + std::to_string(videoSourceId);
    }

    json channelsJson;
    try
    {
        in >> channelsJson;
    }
    catch (...)
    {
        return "VideoSource-" + std::to_string(videoSourceId);
    }

    if (channelsJson.contains("channels") && channelsJson["channels"].is_array())
    {
        for (size_t i = 0; i < channelsJson["channels"].size(); ++i)
        {
            const json& channel = channelsJson["channels"][i];
            if (channel.value("id", 0) == videoSourceId)
            {
                std::string name = trimString(channel.value("name", ""));
                if (!name.empty())
                {
                    return name;
                }
                break;
            }
        }
    }

    return "VideoSource-" + std::to_string(videoSourceId);
}

std::string buildRegionAlarmImageFilename(const std::string& alarmKey, int taskId)
{
    auto now = std::chrono::system_clock::now();
    auto nowTime = std::chrono::system_clock::to_time_t(now);
    std::tm* localTm = std::localtime(&nowTime);
    char timeBuffer[32];
    std::strftime(timeBuffer, sizeof(timeBuffer), "%Y%m%d_%H%M%S", localTm);
    long long millis = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;

    std::ostringstream oss;
    oss << "alarm_" << alarmKey << "_" << timeBuffer
        << "_" << std::setw(3) << std::setfill('0') << millis
        << "_task" << taskId << ".jpg";
    return oss.str();
}

std::string buildAlarmVideoFilename(const std::string& alarmKey, int taskId)
{
    auto now = std::chrono::system_clock::now();
    auto nowTime = std::chrono::system_clock::to_time_t(now);
    std::tm* localTm = std::localtime(&nowTime);
    char timeBuffer[32];
    std::strftime(timeBuffer, sizeof(timeBuffer), "%Y%m%d_%H%M%S", localTm);
    long long millis = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;

    std::ostringstream oss;
    oss << "alarm_" << alarmKey << "_" << timeBuffer
        << "_" << std::setw(3) << std::setfill('0') << millis
        << "_task" << taskId << ".mp4";
    return oss.str();
}

std::string normalizeAlarmImageUrl(const std::string& rawImageUrl)
{
    std::string imageUrl = trimString(rawImageUrl);
    if (imageUrl.empty())
    {
        return "";
    }

    const std::string kAlarmPrefix = "/upload/alarm/";
    size_t prefixPos = imageUrl.find(kAlarmPrefix);
    if (prefixPos != std::string::npos)
    {
        return imageUrl.substr(prefixPos);
    }

    if (imageUrl.find('/') == std::string::npos &&
        imageUrl.find('\\') == std::string::npos &&
        isSupportedImageFile(imageUrl))
    {
        return kAlarmPrefix + imageUrl;
    }

    return imageUrl;
}

bool buildLocalAlarmImagePath(const std::string& rawImageUrl, std::string& localImageUrl, std::string& imagePath)
{
    localImageUrl = normalizeAlarmImageUrl(rawImageUrl);
    imagePath.clear();
    if (localImageUrl.empty())
    {
        return false;
    }

    const std::string kAlarmPrefix = "/upload/alarm/";
    if (localImageUrl.find(kAlarmPrefix) != 0)
    {
        return false;
    }
    if (localImageUrl.find("..") != std::string::npos)
    {
        return false;
    }

    imagePath = "/data/lintech/celectronicfence/static" + localImageUrl;
    return true;
}

bool isAlarmImageAvailable(const std::string& rawImageUrl)
{
    std::string localImageUrl;
    std::string imagePath;
    if (!buildLocalAlarmImagePath(rawImageUrl, localImageUrl, imagePath))
    {
        return false;
    }

    struct stat st;
    if (stat(imagePath.c_str(), &st) != 0)
    {
        return false;
    }
    return S_ISREG(st.st_mode) && st.st_size > 0;
}

std::string normalizeAlarmVideoUrl(const std::string& rawVideoUrl)
{
    std::string videoUrl = trimString(rawVideoUrl);
    if (videoUrl.empty())
    {
        return "";
    }

    const std::string kAlarmPrefix = "/upload/alarm/";
    size_t prefixPos = videoUrl.find(kAlarmPrefix);
    if (prefixPos != std::string::npos)
    {
        return videoUrl.substr(prefixPos);
    }

    if (videoUrl.find('/') == std::string::npos &&
        videoUrl.find('\\') == std::string::npos &&
        isSupportedRecordingVideoFile(videoUrl))
    {
        return kAlarmPrefix + videoUrl;
    }

    return videoUrl;
}

bool buildLocalAlarmVideoPath(const std::string& rawVideoUrl,
                              std::string& localVideoUrl,
                              std::string& videoPath)
{
    localVideoUrl = normalizeAlarmVideoUrl(rawVideoUrl);
    videoPath.clear();
    if (localVideoUrl.empty())
    {
        return false;
    }

    const std::string kAlarmPrefix = "/upload/alarm/";
    if (localVideoUrl.find(kAlarmPrefix) != 0)
    {
        return false;
    }
    if (localVideoUrl.find("..") != std::string::npos)
    {
        return false;
    }

    videoPath = "/data/lintech/celectronicfence/static" + localVideoUrl;
    return true;
}

bool isAlarmVideoAvailable(const std::string& rawVideoUrl)
{
    std::string localVideoUrl;
    std::string videoPath;
    if (!buildLocalAlarmVideoPath(rawVideoUrl, localVideoUrl, videoPath))
    {
        return false;
    }

    struct stat st;
    if (stat(videoPath.c_str(), &st) != 0)
    {
        return false;
    }
    return S_ISREG(st.st_mode) && st.st_size > 0;
}

std::string findTaskOutputRtspByTaskId(int taskId)
{
    if (taskId <= 0)
    {
        return "";
    }

    std::ifstream in("/data/lintech/celectronicfence/tasks.json");
    if (!in.is_open())
    {
        return "";
    }

    json tasksJson;
    try
    {
        in >> tasksJson;
    }
    catch (...)
    {
        return "";
    }

    if (!tasksJson.is_array())
    {
        return "";
    }

    for (size_t i = 0; i < tasksJson.size(); ++i)
    {
        const json& task = tasksJson[i];
        if (task.value("id", 0) != taskId)
        {
            continue;
        }
        return trimString(task.value("outputRtsp", ""));
    }
    return "";
}

std::unordered_map<std::string, std::string> buildFaceLabelToNameMap(const json& labelMap)
{
    std::unordered_map<std::string, std::string> labelToName;
    if (!labelMap.is_object())
    {
        return labelToName;
    }

    for (json::const_iterator it = labelMap.begin(); it != labelMap.end(); ++it)
    {
        if (!it.value().is_string())
        {
            continue;
        }
        std::string faceName = trimString(it.key());
        std::string faceLabel = trimString(it.value().get<std::string>());
        if (faceName.empty() || faceLabel.empty())
        {
            continue;
        }
        labelToName[faceLabel] = faceName;
    }
    return labelToName;
}

std::string buildAlarmImageKeyFromAlarmType(const std::string& alarmTypeRaw)
{
    std::string alarmType = trimString(alarmTypeRaw);
    if (alarmType.find("区域") != std::string::npos)
    {
        return "region";
    }
    if (alarmType.find("人员") != std::string::npos)
    {
        return "person";
    }
    if (alarmType.find("烟") != std::string::npos || alarmType.find("火") != std::string::npos)
    {
        return "fire";
    }
    return "alarm";
}

std::string findLatestAlarmImageUrl(const std::string& alarmKey, int taskId)
{
    const std::string alarmImageDir = "/data/lintech/celectronicfence/static/upload/alarm";
    DIR* dir = opendir(alarmImageDir.c_str());
    if (dir == nullptr)
    {
        return "";
    }

    std::string prefix = "alarm_" + alarmKey + "_";
    std::string taskSuffix;
    if (taskId > 0)
    {
        taskSuffix = "_task" + std::to_string(taskId) + ".jpg";
    }

    auto hasPrefix = [](const std::string& value, const std::string& prefixValue) -> bool
    {
        return value.size() >= prefixValue.size() &&
               value.compare(0, prefixValue.size(), prefixValue) == 0;
    };

    auto hasSuffix = [](const std::string& value, const std::string& suffixValue) -> bool
    {
        return value.size() >= suffixValue.size() &&
               value.compare(value.size() - suffixValue.size(), suffixValue.size(), suffixValue) == 0;
    };

    std::string bestName;
    time_t bestMtime = 0;
    auto selectBest = [&](bool requireTaskSuffix)
    {
        rewinddir(dir);
        struct dirent* entry = nullptr;
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string name = entry->d_name;
            if (name.empty() || name == "." || name == "..")
            {
                continue;
            }
            if (!hasPrefix(name, prefix))
            {
                continue;
            }
            if (!isSupportedImageFile(name))
            {
                continue;
            }
            if (requireTaskSuffix && !taskSuffix.empty() && !hasSuffix(name, taskSuffix))
            {
                continue;
            }

            std::string path = alarmImageDir + "/" + name;
            struct stat st;
            if (stat(path.c_str(), &st) != 0)
            {
                continue;
            }
            if (!S_ISREG(st.st_mode) || st.st_size <= 0)
            {
                continue;
            }

            if (bestName.empty() || st.st_mtime > bestMtime)
            {
                bestName = name;
                bestMtime = st.st_mtime;
            }
        }
    };

    if (!taskSuffix.empty())
    {
        selectBest(true);
    }
    if (bestName.empty())
    {
        selectBest(false);
    }

    closedir(dir);
    if (bestName.empty())
    {
        return "";
    }
    return "/upload/alarm/" + bestName;
}

std::string tryGenerateAlarmImageFromTask(int taskId, const std::string& alarmType)
{
    std::string outputRtsp = findTaskOutputRtspByTaskId(taskId);
    if (outputRtsp.empty())
    {
        return findLatestAlarmImageUrl(buildAlarmImageKeyFromAlarmType(alarmType), taskId);
    }

    const std::string alarmImageDir = "/data/lintech/celectronicfence/static/upload/alarm";
    if (!ensureDirectory(alarmImageDir))
    {
        return findLatestAlarmImageUrl(buildAlarmImageKeyFromAlarmType(alarmType), taskId);
    }

    std::string imageKey = buildAlarmImageKeyFromAlarmType(alarmType);
    std::string filename = buildRegionAlarmImageFilename(imageKey, taskId);
    std::string outputPath = alarmImageDir + "/" + filename;
    if (!captureRtspSnapshot(outputRtsp, outputPath))
    {
        return findLatestAlarmImageUrl(imageKey, taskId);
    }
    return "/upload/alarm/" + filename;
}

bool captureRtspSnapshot(const std::string& rtspUrl, const std::string& outputPath)
{
    std::string errorMessage;
    return captureRtspSnapshotWithError(rtspUrl, outputPath, errorMessage);
}

bool captureRtspSnapshotWithError(const std::string& rtspUrl,
                                  const std::string& outputPath,
                                  std::string& errorMessage)
{
    if (rtspUrl.empty() || outputPath.empty())
    {
        errorMessage = "Task video source URL is empty";
        return false;
    }

    const std::string kFfmpegPrimary = "/opt/sophon/sophon-ffmpeg-latest/bin/ffmpeg";
    std::string ffmpegBinary = pathExists(kFfmpegPrimary) ? kFfmpegPrimary : "ffmpeg";
    if (trimString(ffmpegBinary).empty())
    {
        errorMessage = "ffmpeg is not available";
        return false;
    }

    std::string timeoutPrefix;
    if (pathExists("/usr/bin/timeout"))
    {
        timeoutPrefix = "/usr/bin/timeout 12 ";
    }
    else if (pathExists("/bin/timeout"))
    {
        timeoutPrefix = "/bin/timeout 12 ";
    }

    std::vector<std::string> candidateUrls;
    std::string primaryUrl = trimString(rtspUrl);
    if (!primaryUrl.empty())
    {
        candidateUrls.push_back(primaryUrl);
    }

    std::string noTrailingSlashUrl = primaryUrl;
    while (noTrailingSlashUrl.size() > 8 && !noTrailingSlashUrl.empty() &&
           noTrailingSlashUrl[noTrailingSlashUrl.size() - 1] == '/')
    {
        noTrailingSlashUrl.erase(noTrailingSlashUrl.size() - 1);
    }
    if (!noTrailingSlashUrl.empty() && noTrailingSlashUrl != primaryUrl)
    {
        candidateUrls.push_back(noTrailingSlashUrl);
    }

    struct SnapshotAttempt
    {
        std::string label;
        std::string inputOptions;
    };

    std::string lastErrorMessage;
    for (size_t i = 0; i < candidateUrls.size(); ++i)
    {
        const std::string& candidateUrl = candidateUrls[i];
        std::vector<SnapshotAttempt> attempts;
        if (isRtspLikeUrl(candidateUrl))
        {
            attempts.push_back({"tcp", " -rtsp_transport tcp"});
            attempts.push_back({"udp", " -rtsp_transport udp"});
            attempts.push_back({"auto", ""});
        }
        else
        {
            attempts.push_back({"default", ""});
        }

        for (size_t attemptIndex = 0; attemptIndex < attempts.size(); ++attemptIndex)
        {
            std::remove(outputPath.c_str());

            std::string ffmpegLogPath =
                "/tmp/cef_snapshot_" + std::to_string(currentTimeMillis()) +
                "_" + std::to_string(static_cast<unsigned long long>(i)) +
                "_" + std::to_string(static_cast<unsigned long long>(attemptIndex)) + ".log";
            std::string command =
                timeoutPrefix + shellQuote(ffmpegBinary) +
                " -nostdin -loglevel error -y" +
                attempts[attemptIndex].inputOptions +
                " -analyzeduration 1000000 -probesize 1000000 -i " + shellQuote(candidateUrl) +
                " -frames:v 1 -an -sn -dn -q:v 2 " + shellQuote(outputPath) +
                " >" + shellQuote(ffmpegLogPath) + " 2>&1";

            int ret = std::system(command.c_str());
            struct stat st;
            if (ret == 0 &&
                stat(outputPath.c_str(), &st) == 0 &&
                st.st_size > 0)
            {
                std::remove(ffmpegLogPath.c_str());
                errorMessage.clear();
                return true;
            }

            std::string ffmpegLog;
            if (readBinaryFile(ffmpegLogPath, ffmpegLog))
            {
                ffmpegLog = trimString(ffmpegLog);
            }
            std::remove(ffmpegLogPath.c_str());

            lastErrorMessage = ffmpegLog.empty()
                ? ("Failed to capture frame via " + attempts[attemptIndex].label + " mode")
                : ("Failed to capture frame via " + attempts[attemptIndex].label + " mode: " + ffmpegLog);
        }
    }

    errorMessage = lastErrorMessage.empty()
        ? "Failed to capture frame from configured channel stream"
        : lastErrorMessage;
    return false;
}

bool captureRtspVideoClip(const std::string& rtspUrl,
                          const std::string& outputPath,
                          int durationSec)
{
    std::string errorMessage;
    return captureRtspVideoClipWithError(rtspUrl, outputPath, durationSec, errorMessage);
}

bool captureRtspVideoClipWithError(const std::string& rtspUrl,
                                   const std::string& outputPath,
                                   int durationSec,
                                   std::string& errorMessage)
{
    durationSec = clampInt(durationSec, 1, 300);
    if (rtspUrl.empty() || outputPath.empty())
    {
        errorMessage = "Task video source URL is empty";
        return false;
    }

    const std::string kFfmpegPrimary = "/opt/sophon/sophon-ffmpeg-latest/bin/ffmpeg";
    std::string ffmpegBinary = pathExists(kFfmpegPrimary) ? kFfmpegPrimary : "ffmpeg";
    if (trimString(ffmpegBinary).empty())
    {
        errorMessage = "ffmpeg is not available";
        return false;
    }

    std::string timeoutPrefix;
    const int timeoutSeconds = std::max(20, durationSec + 12);
    if (pathExists("/usr/bin/timeout"))
    {
        timeoutPrefix = "/usr/bin/timeout " + std::to_string(timeoutSeconds) + " ";
    }
    else if (pathExists("/bin/timeout"))
    {
        timeoutPrefix = "/bin/timeout " + std::to_string(timeoutSeconds) + " ";
    }

    std::vector<std::string> candidateUrls;
    std::string primaryUrl = trimString(rtspUrl);
    if (!primaryUrl.empty())
    {
        candidateUrls.push_back(primaryUrl);
    }

    std::string noTrailingSlashUrl = primaryUrl;
    while (noTrailingSlashUrl.size() > 8 && !noTrailingSlashUrl.empty() &&
           noTrailingSlashUrl[noTrailingSlashUrl.size() - 1] == '/')
    {
        noTrailingSlashUrl.erase(noTrailingSlashUrl.size() - 1);
    }
    if (!noTrailingSlashUrl.empty() && noTrailingSlashUrl != primaryUrl)
    {
        candidateUrls.push_back(noTrailingSlashUrl);
    }

    struct ClipAttempt
    {
        std::string label;
        std::string inputOptions;
    };

    std::string lastErrorMessage;
    for (size_t i = 0; i < candidateUrls.size(); ++i)
    {
        const std::string& candidateUrl = candidateUrls[i];
        std::vector<ClipAttempt> attempts;
        if (isRtspLikeUrl(candidateUrl))
        {
            attempts.push_back({"tcp", " -rtsp_transport tcp"});
            attempts.push_back({"udp", " -rtsp_transport udp"});
            attempts.push_back({"auto", ""});
        }
        else
        {
            attempts.push_back({"default", ""});
        }

        for (size_t attemptIndex = 0; attemptIndex < attempts.size(); ++attemptIndex)
        {
            std::remove(outputPath.c_str());

            std::string ffmpegLogPath =
                "/tmp/cef_alarm_clip_" + std::to_string(currentTimeMillis()) +
                "_" + std::to_string(static_cast<unsigned long long>(i)) +
                "_" + std::to_string(static_cast<unsigned long long>(attemptIndex)) + ".log";
            std::string command =
                timeoutPrefix + shellQuote(ffmpegBinary) +
                " -nostdin -loglevel error -y" +
                attempts[attemptIndex].inputOptions +
                " -analyzeduration 1000000 -probesize 1000000 -i " + shellQuote(candidateUrl) +
                " -map 0:v:0 -t " + std::to_string(durationSec) +
                " -an -sn -dn -c:v copy -movflags +faststart " + shellQuote(outputPath) +
                " >" + shellQuote(ffmpegLogPath) + " 2>&1";

            int ret = std::system(command.c_str());
            struct stat st;
            if (ret == 0 &&
                stat(outputPath.c_str(), &st) == 0 &&
                st.st_size > 0)
            {
                std::remove(ffmpegLogPath.c_str());
                errorMessage.clear();
                return true;
            }

            std::string ffmpegLog;
            if (readBinaryFile(ffmpegLogPath, ffmpegLog))
            {
                ffmpegLog = trimString(ffmpegLog);
            }
            std::remove(ffmpegLogPath.c_str());

            lastErrorMessage = ffmpegLog.empty()
                ? ("Failed to record alarm clip via " + attempts[attemptIndex].label + " mode")
                : ("Failed to record alarm clip via " + attempts[attemptIndex].label + " mode: " + ffmpegLog);
        }
    }

    errorMessage = lastErrorMessage.empty()
        ? "Failed to record clip from configured channel stream"
        : lastErrorMessage;
    return false;
}

bool postAlarmToLocalApi(const json& payload)
{
    httplib::Client cli("127.0.0.1", 8088);
    cli.set_connection_timeout(2, 0);
    cli.set_read_timeout(6, 0);
    cli.set_write_timeout(6, 0);

    auto res = cli.Post("/api/alarms", payload.dump(), "application/json");
    if (!res)
    {
        std::cerr << "[区域告警] 调用 /api/alarms 失败，无响应" << std::endl;
        return false;
    }
    if (res->status < 200 || res->status >= 300)
    {
        std::cerr << "[区域告警] 调用 /api/alarms 失败，status="
                  << res->status << ", body=" << res->body << std::endl;
        return false;
    }
    return true;
}

bool readAlarmRecordById(int alarmId, json& alarmOut)
{
    alarmOut = json::object();
    if (alarmId <= 0)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(alarmFileMutex);
    std::ifstream fileIn("/data/lintech/celectronicfence/alarms.json");
    if (!fileIn.is_open())
    {
        return false;
    }

    json alarmsJson = json::array();
    try
    {
        fileIn >> alarmsJson;
    }
    catch (...)
    {
        return false;
    }

    if (!alarmsJson.is_array())
    {
        return false;
    }

    for (size_t i = 0; i < alarmsJson.size(); ++i)
    {
        const json& alarm = alarmsJson[i];
        if (alarm.contains("id") && alarm["id"] == alarmId)
        {
            alarmOut = alarm;
            return true;
        }
    }
    return false;
}

bool updateAlarmRecordById(int alarmId, const json& patch, json* updatedAlarmOut)
{
    if (updatedAlarmOut != nullptr)
    {
        *updatedAlarmOut = json::object();
    }
    if (alarmId <= 0 || !patch.is_object())
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(alarmFileMutex);
    std::ifstream fileIn("/data/lintech/celectronicfence/alarms.json");
    json alarmsJson = json::array();
    if (fileIn.is_open())
    {
        try
        {
            fileIn >> alarmsJson;
        }
        catch (...)
        {
            alarmsJson = json::array();
        }
        fileIn.close();
    }

    if (!alarmsJson.is_array())
    {
        alarmsJson = json::array();
    }

    bool found = false;
    for (size_t i = 0; i < alarmsJson.size(); ++i)
    {
        json& alarm = alarmsJson[i];
        if (!alarm.contains("id") || alarm["id"] != alarmId)
        {
            continue;
        }

        for (json::const_iterator it = patch.begin(); it != patch.end(); ++it)
        {
            alarm[it.key()] = it.value();
        }
        if (updatedAlarmOut != nullptr)
        {
            *updatedAlarmOut = alarm;
        }
        found = true;
        break;
    }

    if (!found)
    {
        return false;
    }

    std::ofstream fileOut("/data/lintech/celectronicfence/alarms.json");
    if (!fileOut.is_open())
    {
        return false;
    }
    fileOut << alarmsJson.dump(4);
    fileOut.close();
    return true;
}

void processAlarmVideoAndReportAsync(int alarmId, int taskId, const std::string& alarmType)
{
    std::thread worker([alarmId, taskId, alarmType]() {
        json patch = json::object();
        json updatedAlarm;
        int durationSec = readAlarmVideoDurationSecondsFromParams();
        patch["videoDurationSec"] = durationSec;
        patch["videoMimeType"] = "video/mp4";

        std::string outputRtsp = findTaskOutputRtspByTaskId(taskId);
        if (outputRtsp.empty())
        {
            patch["videoStatus"] = "failed";
            patch["videoError"] = "Task output stream is empty";
        }
        else
        {
            const std::string alarmVideoDir = "/data/lintech/celectronicfence/static/upload/alarm";
            if (!ensureDirectory(alarmVideoDir))
            {
                patch["videoStatus"] = "failed";
                patch["videoError"] = "Alarm video directory is not writable";
            }
            else
            {
                std::string alarmKey = buildAlarmImageKeyFromAlarmType(alarmType);
                std::string filename = buildAlarmVideoFilename(alarmKey, taskId);
                std::string outputPath = alarmVideoDir + "/" + filename;
                std::string errorMessage;
                if (captureRtspVideoClipWithError(outputRtsp, outputPath, durationSec, errorMessage))
                {
                    patch["videoUrl"] = "/upload/alarm/" + filename;
                    patch["videoStatus"] = "ready";
                    patch["videoError"] = "";
                }
                else
                {
                    patch["videoStatus"] = "failed";
                    patch["videoError"] = errorMessage;
                }
            }
        }

        if (!updateAlarmRecordById(alarmId, patch, &updatedAlarm))
        {
            return;
        }

        std::string remoteUrl = getRemoteAlarmUrl();
        if (remoteUrl.empty())
        {
            remoteUrl = trimString(updatedAlarm.value("reportUrl", ""));
        }
        if (!remoteUrl.empty())
        {
            reportAlarmToRemote(updatedAlarm);
        }
    });
    worker.detach();
}

void emitRegisteredPersonTrackingAlarm(const RegisteredPersonAlarmEvent& event)
{
    if (event.taskId <= 0 || trimString(event.label).empty())
    {
        return;
    }

    int cooldownSeconds = readAlarmTriggerCooldownSecondsFromParams(10);
    long long nowMs = currentTimeMillis();
    if (cooldownSeconds > 0)
    {
        const std::string cooldownKey =
            std::to_string(event.taskId) + ":" + trimString(event.label);
        std::lock_guard<std::mutex> lock(registeredPersonAlarmCooldownMutex);
        std::unordered_map<std::string, long long>::const_iterator cooldownIt =
            registeredPersonAlarmCooldownUntilMs.find(cooldownKey);
        if (cooldownIt != registeredPersonAlarmCooldownUntilMs.end() &&
            cooldownIt->second > nowMs)
        {
            return;
        }
        registeredPersonAlarmCooldownUntilMs[cooldownKey] =
            nowMs + static_cast<long long>(cooldownSeconds) * 1000;
    }

    int videoSourceId = 0;
    std::ifstream tasksIn(TASKS_JSON_PATH);
    if (tasksIn.is_open())
    {
        json tasksJson;
        try
        {
            tasksIn >> tasksJson;
            if (tasksJson.is_array())
            {
                for (size_t i = 0; i < tasksJson.size(); ++i)
                {
                    const json& task = tasksJson[i];
                    if (task.value("id", 0) == event.taskId)
                    {
                        videoSourceId = task.value("videoSourceId", 0);
                        break;
                    }
                }
            }
        }
        catch (...)
        {
            videoSourceId = 0;
        }
    }

    std::string videoSourceName = resolveVideoSourceNameById(videoSourceId);
    std::unordered_map<std::string, std::string> labelToName =
        buildFaceLabelToNameMap(loadFaceLabelMap());
    std::string matchedName = event.label;
    std::unordered_map<std::string, std::string>::const_iterator nameIt =
        labelToName.find(event.label);
    if (nameIt != labelToName.end() && !trimString(nameIt->second).empty())
    {
        matchedName = trimString(nameIt->second);
    }

    std::ostringstream description;
    description << "区域内出现匹配人脸: " << matchedName
                << " (trackId=" << event.trackId;
    if (event.score > 0.0)
    {
        description << ", score=" << std::fixed << std::setprecision(2)
                    << event.score;
    }
    description << ")";

    json payload;
    payload["taskId"] = event.taskId;
    payload["videoSourceId"] = videoSourceId;
    payload["videoSourceName"] = videoSourceName;
    payload["alarmType"] = "登记人员告警";
    payload["status"] = "未处理";
    payload["reportUrl"] = "";
    payload["description"] = description.str();

    if (postAlarmToLocalApi(payload))
    {
        std::cout << "[登记人员告警] 已写入告警，taskId=" << event.taskId
                  << ", trackId=" << event.trackId
                  << ", label=" << event.label << std::endl;
    }
}

bool readGpioDetectedStateByChannel(int channelId, bool& detected)
{
    detected = false;
    std::string gpioPath;
    if (channelId == 20)
    {
        gpioPath = "/sys/class/gpio/gpio429/value";
    }
    else if (channelId == 10)
    {
        gpioPath = "/sys/class/gpio/gpio430/value";
    }
    else
    {
        return false;
    }

    std::ifstream in(gpioPath.c_str());
    if (!in.is_open())
    {
        return false;
    }

    std::string value;
    std::getline(in, value);
    value = trimString(value);
    if (value != "0" && value != "1")
    {
        return false;
    }

    // 与 /api/human_detected 语义一致：GPIO=0 表示检测到人员/入侵
    detected = (value == "0");
    return true;
}

void emitRegionAlarmForTask(const json& task)
{
    int taskId = task.value("id", 0);
    int videoSourceId = task.value("videoSourceId", 0);
    std::string algorithm = task.value("algorithm", "");
    std::string outputRtsp = task.value("outputRtsp", "");
    std::string alarmKey = (algorithm == "person_detection") ? "person" : "region";
    std::string alarmType = (algorithm == "person_detection") ? "人员检测" : "区域入侵";
    std::string description = (algorithm == "person_detection") ? "检测到人员" : "检测到区域入侵";

    std::string imageUrl;
    const std::string alarmImageDir = "/data/lintech/celectronicfence/static/upload/alarm";
    if (ensureDirectory(alarmImageDir))
    {
        std::string filename = buildRegionAlarmImageFilename(alarmKey, taskId);
        std::string outputPath = alarmImageDir + "/" + filename;
        if (captureRtspSnapshot(outputRtsp, outputPath))
        {
            imageUrl = "/upload/alarm/" + filename;
        }
        else
        {
            std::cerr << "[区域告警] 抓拍失败，任务ID: " << taskId
                      << ", rtsp: " << outputRtsp << std::endl;
            // 对齐旧项目行为：抓拍失败时不写入告警，避免出现“无图片”记录。
            return;
        }
    }
    else
    {
        std::cerr << "[区域告警] 告警图片目录不可写: " << alarmImageDir << std::endl;
        return;
    }

    json payload;
    payload["taskId"] = taskId;
    payload["videoSourceId"] = videoSourceId;
    payload["videoSourceName"] = resolveVideoSourceNameById(videoSourceId);
    payload["alarmType"] = alarmType;
    payload["imageUrl"] = imageUrl;
    payload["status"] = "未处理";
    payload["description"] = description;
    payload["reportUrl"] = "";

    if (postAlarmToLocalApi(payload))
    {
        std::cout << "[区域告警] 已写入告警，任务ID: " << taskId
                  << ", imageUrl: " << imageUrl << std::endl;
    }
}

void regionAlarmMonitorThreadFunction()
{
    while (!stopRegionAlarmMonitor.load())
    {
        std::ifstream tasksIn("/data/lintech/celectronicfence/tasks.json");
        if (!tasksIn.is_open())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(800));
            continue;
        }

        json tasksJson;
        try
        {
            tasksIn >> tasksJson;
        }
        catch (...)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(800));
            continue;
        }

        if (!tasksJson.is_array())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(800));
            continue;
        }

        const int cooldownSeconds = readAlarmCooldownSecondsFromParams();
        std::set<int> activeTaskIds;
        bool channel20Polled = false;
        bool channel10Polled = false;
        bool channel20Available = false;
        bool channel10Available = false;
        bool channel20Detected = false;
        bool channel10Detected = false;

        for (size_t i = 0; i < tasksJson.size(); ++i)
        {
            const json& task = tasksJson[i];
            int taskId = task.value("id", 0);
            if (taskId <= 0)
            {
                continue;
            }

            std::string status = task.value("status", "");
            std::string algorithm = task.value("algorithm", "");
            if (status != "running" ||
                (algorithm != "region_intrusion" && algorithm != "person_detection"))
            {
                continue;
            }

            int channelId = task.value("channelId", 0);
            bool detected = false;
            bool available = false;
            if (channelId == 20)
            {
                if (!channel20Polled)
                {
                    channel20Polled = true;
                    channel20Available = readGpioDetectedStateByChannel(20, channel20Detected);
                }
                available = channel20Available;
                detected = channel20Detected;
            }
            else if (channelId == 10)
            {
                if (!channel10Polled)
                {
                    channel10Polled = true;
                    channel10Available = readGpioDetectedStateByChannel(10, channel10Detected);
                }
                available = channel10Available;
                detected = channel10Detected;
            }
            else
            {
                continue;
            }

            activeTaskIds.insert(taskId);
            if (!available)
            {
                continue;
            }

            bool needEmit = false;
            auto now = std::chrono::system_clock::now();
            {
                std::lock_guard<std::mutex> lock(regionAlarmStateMutex);
                bool lastDetected = false;
                if (regionTaskDetectedState.find(taskId) != regionTaskDetectedState.end())
                {
                    lastDetected = regionTaskDetectedState[taskId];
                }

                if (detected && !lastDetected)
                {
                    bool cooldownOk = true;
                    if (regionTaskLastAlarmTime.find(taskId) != regionTaskLastAlarmTime.end())
                    {
                        long long elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                            now - regionTaskLastAlarmTime[taskId]).count();
                        cooldownOk = elapsed >= cooldownSeconds;
                    }
                    if (cooldownOk)
                    {
                        needEmit = true;
                        regionTaskLastAlarmTime[taskId] = now;
                    }
                }

                regionTaskDetectedState[taskId] = detected;
            }

            if (needEmit)
            {
                // 对齐旧项目：告警抓拍/上报异步执行，避免阻塞主检测链路。
                json taskSnapshot = task;
                std::thread([taskSnapshot]()
                {
                    emitRegionAlarmForTask(taskSnapshot);
                }).detach();
            }
        }

        {
            std::lock_guard<std::mutex> lock(regionAlarmStateMutex);
            for (std::map<int, bool>::iterator it = regionTaskDetectedState.begin();
                 it != regionTaskDetectedState.end();)
            {
                if (activeTaskIds.find(it->first) == activeTaskIds.end())
                {
                    it = regionTaskDetectedState.erase(it);
                }
                else
                {
                    ++it;
                }
            }

            for (std::map<int, std::chrono::time_point<std::chrono::system_clock>>::iterator it =
                     regionTaskLastAlarmTime.begin();
                 it != regionTaskLastAlarmTime.end();)
            {
                if (activeTaskIds.find(it->first) == activeTaskIds.end())
                {
                    it = regionTaskLastAlarmTime.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

//----------------------------------------------------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    const std::string tfMountPath = "/mnt/tfcard";
 if (!waitForMountPoint(tfMountPath))
{
    return 1; // 根据实际情况返回错误码，这里简单地返回1表示出错了
}

// 解析命令行参数，判断是否有 -a 参数（标识开机自启）
int opt;
while ((opt = getopt(argc, argv, "a"))!= -1)
{
    switch (opt)
    {
    case 'a':
        isAutoStart = true;
        break;
    default:
        // 在这里可以添加错误处理的逻辑，比如打印帮助信息等，当前先省略不写
        break;
    }
}

json defaultAddresses;
readDefaultAddressesFromJson(defaultAddresses);
ensureDefaultAddressSections(defaultAddresses);

// 确保defaultAddresses中存在"current"节点，如果不存在则创建一个空对象作为"current"节点
if (!defaultAddresses.contains("current"))
{
    defaultAddresses["current"] = json::object();
}

// 获取当前默认的视频流地址和保存地址，用于开机自启录制（从"current"节点下获取）
std::string defaultRtspStreamUrl = defaultAddresses["current"].value("defaultRtspStreamUrl", "");
std::string defaultSaveLocation = defaultAddresses["current"].value("defaultSaveLocation", "");
std::string defaultRtspStreamUrl2 = defaultAddresses["current"].value("defaultRtspStreamUrl2", "");
std::string defaultSaveLocation2 = defaultAddresses["current"].value("defaultSaveLocation2", "");
bool autoStartDualStreamEnabled = readDualStreamEnabledFromDefaultAddresses(defaultAddresses, true);

if (isAutoStart)
{
    // 如果是开机自启情况，尝试启动录制，传入从配置中获取的当前默认地址
    try
    {
        startRecording(defaultRtspStreamUrl,
                       defaultSaveLocation,
                       defaultRtspStreamUrl2,
                       defaultSaveLocation2,
                       defaultSegmentTime,
                       autoStartDualStreamEnabled);
    }
    catch (const std::exception &e)
    {
        std::cerr << "开机自动启动录制出错: " << e.what() << std::endl;
    }
}

    Server svr;

    auto ret = svr.set_mount_point("/", "./static");
    if (!ret)
    {
        printf("The./static directory doesn't exist...\n");
    }
    //----------------------------------------------------------------------------------------------------------------------------
    svr.set_logger([](const Request &req, const Response &res)
                   { std::cout << log(req, res) << std::endl; });

    //----------------------------------------------------------------------------------------------------------------------------
    //     //获取 TF 卡信息并显示在页面上
    svr.Get("/get_tf_info", [](const Request &req, Response &res)
            {
    std::cout << "接受到tf请求"<< std::endl;
    TFCardInfo tfCardInfo = getTFCardInfo();
    json responseJson;
    responseJson["mountPath"] = tfCardInfo.mountPath;
    responseJson["totalMemory"] = tfCardInfo.totalMemory;
    responseJson["usedMemory"] = tfCardInfo.usedMemory;
    responseJson["freeMemory"] = tfCardInfo.freeMemory;
    responseJson["usePercent"] = tfCardInfo.usePercent;
    res.set_content(responseJson.dump(), "application/json"); });
    //----------------------------------------------------------------------------------------------------------------------------
    // 添加网络配置保存的路由
    svr.Post("/save_network_config", JsonFile1::handleSaveNetworkConfig);
    //----------------------------------------------------------------------------------------------------------------------------
    // 添加对 JSON 文件的路由
    svr.Get("/get_json_file", [](const Request &req, Response &res)
            {
        json data;
        JsonFile2::loadJsonData(data);
        res.set_content(data.dump(), "application/json"); });
    svr.Post("/get_json_file", [](const Request &req, Response &res)
             {
        try {
            json reqJson = json::parse(req.body);
            std::cout << reqJson.dump(4) << std::endl;
            json data;
            JsonFile2::loadJsonData(data);
            std::cout << data.dump(4) << std::endl;

            std::set<std::string> classNames = JsonFile2::loadClassNameSetFromThresholdConfig(data);
            if (classNames.empty()) {
                res.status = 500;
                res.set_content("Error updating configuration: class_names_file is missing or invalid", "text/plain");
                return;
            }

            int removedInvalidCount = 0;
            bool cleaned = JsonFile2::sanitizeThresholdConf(data, classNames, &removedInvalidCount);
            if (cleaned || removedInvalidCount > 0) {
                std::cout << "[阈值配置] 已清理非法类别键数量: " << removedInvalidCount << std::endl;
            }

            if (!data.contains("configure") || !data["configure"].is_object()) {
                data["configure"] = json::object();
            }
            if (!data["configure"].contains("threshold_conf") || !data["configure"]["threshold_conf"].is_object()) {
                data["configure"]["threshold_conf"] = json::object();
            }

            // 只允许更新合法类别，避免写入额外键导致类别数量不匹配
            for (auto it = reqJson.begin(); it != reqJson.end(); ++it) {
                const std::string& key = it.key();
                const json& value = it.value();
                if (!value.is_number()) {
                    continue;
                }
                if (classNames.find(key) == classNames.end()) {
                    std::cout << "[阈值配置] 忽略非法类别键: " << key << std::endl;
                    continue;
                }
                data["configure"]["threshold_conf"][key] = value;
            }
            
            JsonFile2::updateJsonFile(data);
            std::cout << data.dump(4) << std::endl;
            res.status = 200;
            res.set_content("Configuration updated successfully.", "text/plain");
        } catch (const std::exception& e) {
            res.status = 500;
            res.set_content("Error updating configuration: " + std::string(e.what()), "text/plain");
        } });
    //----------------------------------------------------------------------------------------------------------------------------
    // 处理视频流地址更新的路由
    svr.Post("/update_video_stream", [](const Request &req, Response &res)
             {
        try {
            json reqJson = json::parse(req.body);
            std::string videoStreamUrl = reqJson.value("videoStreamUrl", "");
            std::string videoStreamUrl2 = reqJson.value("videoStreamUrl2", "");
            json data;
            JsonFile2::loadJsonDataForDemo(data);
        // 在channels数组中查找对应channel_id的通道并更新其url字段
        for (auto& channel : data["channels"]) {
            if (channel["channel_id"] == 20 &&!videoStreamUrl.empty()) {
                channel["url"] = videoStreamUrl;
            } else if (channel["channel_id"] == 10 &&!videoStreamUrl2.empty()) {
                channel["url"] = videoStreamUrl2;
            }
        }
            // data["channels"][0]["url"] = videoStreamUrl;
            JsonFile2::updateJsonFileForDemo(data);
            std::cout << "[配置更新] 已更新视频流配置，不再自动执行run.sh" << std::endl;

            res.status = 200;
            res.set_content("Configuration updated successfully.", "text/plain");
        } catch (const std::exception& e) {
            res.status = 500;
            res.set_content("Error updating configuration: " + std::string(e.what()), "text/plain");
        } });
    //----------------------------------------------------------------------------------------------------------------------------
    // 处理开始录制的路由
    svr.Post("/start_recording", [](const Request &req, Response &res)
             {
    try {
        json reqJson = req.body.empty() ? json::object() : json::parse(req.body);
        std::string rtspStreamUrl = trimString(reqJson.value("rtspStreamUrl", ""));
        std::string saveLocation = trimString(reqJson.value("saveLocation", ""));
        std::string rtspStreamUrl2 = trimString(reqJson.value("rtspStreamUrl2", ""));
        std::string saveLocation2 = trimString(reqJson.value("saveLocation2", ""));
        int segmentTime = sanitizeRecordingSegmentTime(reqJson.value("segmentTime", defaultSegmentTime));

        // 获取TF卡信息
        TFCardInfo tfCardInfo = getTFCardInfo();
        double freeMemoryInGB = parseHumanSizeToGigabytes(tfCardInfo.freeMemory);
        // 检查TF卡可用内存是否足够
        if (freeMemoryInGB < 20.0) {
            res.status = 400;
            res.set_content("TF card has insufficient free memory. Please replace with a larger capacity TF card.", "text/plain");
            return;
        }

        defaultSegmentTime = segmentTime;
        startRecording(rtspStreamUrl, saveLocation, rtspStreamUrl2, saveLocation2, segmentTime);
        res.status = 200;
        res.set_content("Recording started successfully.", "text/plain");
    } catch (const std::exception& e) {
        res.status = 500;
        res.set_content("Error starting recording: " + std::string(e.what()), "text/plain");
    } });
    //----------------------------------------------------------------------------------------------------------------------------
    // 添加处理结束录制的路由
    svr.Post("/stop_recording", [](const Request &req, Response &res)
             {
        stopRecording();
        res.status = 200;
        res.set_content("Recording stopped successfully.", "text/plain"); });
    //----------------------------------------------------------------------------------------------------------------------------
    // 启动定时器线程
    std::thread timerThread(timerThreadFunction);
    timerThread.detach();
    //----------------------------------------------------------------------------------------------------------------------------
    // 读取json文件中的录制状态返回给前端
    svr.Get("/get_recording_status", [](const Request &req, Response &res)
            {
        refreshRecordingRuntimeState();
        json statusJson = readJsonObjectFile(RECORDING_STATUS_FILE);
        bool recordingStatus = statusJson.value("recording_status", false);
        res.set_content(recordingStatus ? "true" : "false", "application/json"); });
    //----------------------------------------------------------------------------------------------------------------------------
    svr.Get("/api/recording/status", [](const Request &req, Response &res)
            {
        refreshRecordingRuntimeState();
        json statusJson = readJsonObjectFile(RECORDING_STATUS_FILE);
        json responseJson = {
            {"success", true},
            {"status", {
                {"recording1", statusJson.value("recording1", false)},
                {"recording2", statusJson.value("recording2", false)},
                {"start_time", statusJson.value("start_time", static_cast<long long>(0))},
                {"error", statusJson.value("last_error", "")},
                {"tfcard", buildRecordingTfcardJson()}
            }}
        };
        res.set_content(responseJson.dump(), "application/json"); });
    //----------------------------------------------------------------------------------------------------------------------------
    svr.Get("/api/recording/config", [](const Request &req, Response &res)
            {
        json responseJson = {
            {"success", true},
            {"config", buildRecordingConfigJson()}
        };
        res.set_content(responseJson.dump(), "application/json"); });
    //----------------------------------------------------------------------------------------------------------------------------
    svr.Post("/api/recording/config", [](const Request &req, Response &res)
             {
        try {
            json reqJson = req.body.empty() ? json::object() : json::parse(req.body);
            json defaultAddresses;
            readDefaultAddressesFromJson(defaultAddresses);
            ensureDefaultAddressSections(defaultAddresses);

            std::string rtspUrl1 = trimString(reqJson.contains("rtsp_url_1") && reqJson["rtsp_url_1"].is_string()
                                                  ? reqJson["rtsp_url_1"].get<std::string>()
                                                  : reqJson.value("rtspStreamUrl", defaultAddresses["current"].value("defaultRtspStreamUrl", "")));
            std::string rtspUrl2 = trimString(reqJson.contains("rtsp_url_2") && reqJson["rtsp_url_2"].is_string()
                                                  ? reqJson["rtsp_url_2"].get<std::string>()
                                                  : reqJson.value("rtspStreamUrl2", defaultAddresses["current"].value("defaultRtspStreamUrl2", "")));
            std::string savePath1 = trimString(reqJson.contains("save_path_1") && reqJson["save_path_1"].is_string()
                                                   ? reqJson["save_path_1"].get<std::string>()
                                                   : reqJson.value("saveLocation", defaultAddresses["current"].value("defaultSaveLocation", "")));
            std::string savePath2 = trimString(reqJson.contains("save_path_2") && reqJson["save_path_2"].is_string()
                                                   ? reqJson["save_path_2"].get<std::string>()
                                                   : reqJson.value("saveLocation2", defaultAddresses["current"].value("defaultSaveLocation2", "")));
            bool dualStreamEnabled = readDualStreamEnabledFromDefaultAddresses(defaultAddresses, true);
            if (reqJson.contains("dual_stream_enabled"))
            {
                dualStreamEnabled = parseJsonBoolLike(reqJson["dual_stream_enabled"], dualStreamEnabled);
            }
            else if (reqJson.contains("dualStreamEnabled"))
            {
                dualStreamEnabled = parseJsonBoolLike(reqJson["dualStreamEnabled"], dualStreamEnabled);
            }

            if (!rtspUrl1.empty()) {
                defaultAddresses["current"]["defaultRtspStreamUrl"] = rtspUrl1;
            }
            if (!rtspUrl2.empty()) {
                defaultAddresses["current"]["defaultRtspStreamUrl2"] = rtspUrl2;
            }
            if (!savePath1.empty()) {
                defaultAddresses["current"]["defaultSaveLocation"] = savePath1;
            }
            if (!savePath2.empty()) {
                defaultAddresses["current"]["defaultSaveLocation2"] = savePath2;
            }
            defaultAddresses["current"]["dualStreamEnabled"] = dualStreamEnabled;
            defaultAddresses["current"]["dual_stream_enabled"] = dualStreamEnabled;

            if (reqJson.contains("segment_time") && reqJson["segment_time"].is_number_integer()) {
                defaultSegmentTime = sanitizeRecordingSegmentTime(reqJson["segment_time"].get<int>());
            } else if (reqJson.contains("segmentTime") && reqJson["segmentTime"].is_number_integer()) {
                defaultSegmentTime = sanitizeRecordingSegmentTime(reqJson["segmentTime"].get<int>());
            }

            updateDefaultAddressesInJson(defaultAddresses);

            json responseJson = {
                {"success", true},
                {"message", "配置已更新"},
                {"config", buildRecordingConfigJson()}
            };
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson = {
                {"success", false},
                {"message", e.what()}
            };
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        } });
    //----------------------------------------------------------------------------------------------------------------------------
    svr.Post("/api/recording/start", [](const Request &req, Response &res)
             {
        try {
            json reqJson = req.body.empty() ? json::object() : json::parse(req.body);
            json defaultAddresses;
            readDefaultAddressesFromJson(defaultAddresses);
            ensureDefaultAddressSections(defaultAddresses);

            std::string rtspUrl1 = trimString(reqJson.contains("rtsp_url_1") && reqJson["rtsp_url_1"].is_string()
                                                  ? reqJson["rtsp_url_1"].get<std::string>()
                                                  : reqJson.value("rtspStreamUrl", ""));
            std::string rtspUrl2 = trimString(reqJson.contains("rtsp_url_2") && reqJson["rtsp_url_2"].is_string()
                                                  ? reqJson["rtsp_url_2"].get<std::string>()
                                                  : reqJson.value("rtspStreamUrl2", ""));
            std::string savePath1 = trimString(reqJson.contains("save_path_1") && reqJson["save_path_1"].is_string()
                                                   ? reqJson["save_path_1"].get<std::string>()
                                                   : reqJson.value("saveLocation", ""));
            std::string savePath2 = trimString(reqJson.contains("save_path_2") && reqJson["save_path_2"].is_string()
                                                   ? reqJson["save_path_2"].get<std::string>()
                                                   : reqJson.value("saveLocation2", ""));
            bool dualStreamEnabled = readDualStreamEnabledFromDefaultAddresses(defaultAddresses, true);
            if (reqJson.contains("dual_stream_enabled"))
            {
                dualStreamEnabled = parseJsonBoolLike(reqJson["dual_stream_enabled"], dualStreamEnabled);
            }
            else if (reqJson.contains("dualStreamEnabled"))
            {
                dualStreamEnabled = parseJsonBoolLike(reqJson["dualStreamEnabled"], dualStreamEnabled);
            }

            int segmentTime = defaultSegmentTime;
            if (reqJson.contains("segment_time") && reqJson["segment_time"].is_number_integer()) {
                segmentTime = sanitizeRecordingSegmentTime(reqJson["segment_time"].get<int>());
            } else if (reqJson.contains("segmentTime") && reqJson["segmentTime"].is_number_integer()) {
                segmentTime = sanitizeRecordingSegmentTime(reqJson["segmentTime"].get<int>());
            } else {
                segmentTime = sanitizeRecordingSegmentTime(defaultSegmentTime);
            }

            TFCardInfo tfCardInfo = getTFCardInfo();
            double freeMemoryInGB = parseHumanSizeToGigabytes(tfCardInfo.freeMemory);
            if (freeMemoryInGB < 20.0) {
                json errorJson = {
                    {"success", false},
                    {"message", "TF卡可用空间不足 20GB，无法开始录制。"}
                };
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            defaultSegmentTime = segmentTime;
            startRecording(rtspUrl1, savePath1, rtspUrl2, savePath2, segmentTime, dualStreamEnabled);

            json responseJson = {
                {"success", true},
                {"message", "录制已启动"}
            };
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson = {
                {"success", false},
                {"message", std::string("启动失败: ") + e.what()}
            };
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        } });
    //----------------------------------------------------------------------------------------------------------------------------
    svr.Post("/api/recording/stop", [](const Request &req, Response &res)
             {
        try {
            stopRecording();
            json responseJson = {
                {"success", true},
                {"message", "录制已停止"}
            };
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson = {
                {"success", false},
                {"message", std::string("停止失败: ") + e.what()}
            };
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        } });
    //----------------------------------------------------------------------------------------------------------------------------
    svr.Get("/api/recording/files", [](const Request &req, Response &res)
            {
        json filesJson = json::array();
        std::vector<json> files = collectRecordingFiles();
        for (size_t i = 0; i < files.size(); ++i) {
            filesJson.push_back(files[i]);
        }
        json responseJson = {
            {"success", true},
            {"files", filesJson}
        };
        res.set_content(responseJson.dump(), "application/json"); });
    //----------------------------------------------------------------------------------------------------------------------------
    svr.Post("/api/recording/files/delete", [](const Request &req, Response &res)
             {
        try {
            json reqJson = req.body.empty() ? json::object() : json::parse(req.body);
            std::string relativePath = trimString(reqJson.value("relativePath", ""));
            std::string filePath = trimString(reqJson.value("filePath", ""));

            if (relativePath.empty() && !filePath.empty()) {
                relativePath = deriveRecordingRelativePath(filePath);
            }

            std::string fullPath;
            std::string channelName;
            if (!resolveRecordingRelativePath(relativePath, fullPath, channelName)) {
                json errorJson = {
                    {"success", false},
                    {"message", "不允许删除此路径的文件"}
                };
                res.status = 403;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            struct stat fileStat;
            if (stat(fullPath.c_str(), &fileStat) != 0 || !S_ISREG(fileStat.st_mode)) {
                json errorJson = {
                    {"success", false},
                    {"message", "文件不存在"}
                };
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            if (static_cast<long long>(std::time(nullptr) - fileStat.st_mtime) < 5) {
                json errorJson = {
                    {"success", false},
                    {"message", "无法删除正在录制的文件"}
                };
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            if (std::remove(fullPath.c_str()) != 0 && pathExists(fullPath)) {
                std::string deleteCommand = "rm -f -- " + shellQuote(fullPath);
                int deleteResult = std::system(deleteCommand.c_str());
                if (deleteResult != 0 && pathExists(fullPath)) {
                    throw std::runtime_error("删除文件失败: " + fullPath);
                }
            }

            std::string basePath = fullPath;
            size_t dotPos = basePath.find_last_of('.');
            if (dotPos != std::string::npos) {
                basePath = basePath.substr(0, dotPos);
                const char* imageExts[] = {".jpg", ".jpeg", ".png"};
                for (size_t i = 0; i < 3; ++i) {
                    removeFileIfExists(basePath + imageExts[i]);
                }
            }

            json responseJson = {
                {"success", true},
                {"message", "文件已删除"}
            };
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson = {
                {"success", false},
                {"message", e.what()}
            };
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        } });
    //----------------------------------------------------------------------------------------------------------------------------
    svr.Get(R"(/api/recording/preview/(.+))", [](const Request &req, Response &res)
            {
        std::string relativePath = req.matches.size() > 1 ? std::string(req.matches[1]) : "";
        std::string fullPath;
        std::string channelName;
        if (!resolveRecordingRelativePath(relativePath, fullPath, channelName)) {
            res.status = 404;
            res.set_content("File not found", "text/plain");
            return;
        }

        struct stat fileStat;
        if (stat(fullPath.c_str(), &fileStat) != 0 || !S_ISREG(fileStat.st_mode)) {
            res.status = 404;
            res.set_content("File not found", "text/plain");
            return;
        }

        std::string fileName = fullPath.substr(fullPath.find_last_of('/') + 1);
        if (req.has_param("download")) {
            res.set_header("Content-Disposition", "attachment; filename=\"" + fileName + "\"");
        }
        auto mm = std::make_shared<detail::mmap>(fullPath.c_str());
        if (!mm->is_open())
        {
            res.status = 404;
            res.set_content("File not found", "text/plain");
            return;
        }

        res.set_header("Accept-Ranges", "bytes");
        res.set_content_provider(
            mm->size(),
            guessRecordingMimeType(fullPath),
            [mm](size_t offset, size_t length, DataSink &sink) -> bool
            {
                sink.write(mm->data() + offset, length);
                return true;
            });
        });
    //----------------------------------------------------------------------------------------------------------------------------
    svr.Get("/api/recording/diagnose", [](const Request &req, Response &res)
            {
        json responseJson = {
            {"success", true},
            {"diagnosis", buildRecordingDiagnosisJson()}
        };
        res.set_content(responseJson.dump(), "application/json"); });
    //----------------------------------------------------------------------------------------------------------------------------
    svr.Get("/get_all_default_addresses", [](const Request &req, Response &res)
            {
    json defaultAddresses;
    readDefaultAddressesFromJson(defaultAddresses);
    json responseJson = {
        {"defaultRtspStreamUrl", defaultAddresses["current"].value("defaultRtspStreamUrl", "")},
        {"defaultSaveLocation", defaultAddresses["current"].value("defaultSaveLocation", "")},
        {"defaultRtspStreamUrl2", defaultAddresses["current"].value("defaultRtspStreamUrl2", "")},
        {"defaultSaveLocation2", defaultAddresses["current"].value("defaultSaveLocation2", "")}
    };
    res.set_content(responseJson.dump(), "application/json"); });
    //----------------------------------------------------------------------------------------------------------------------------
svr.Post("/update_default_addresses", [](const Request &req, Response &res)
{
    try
    {
        json reqJson = json::parse(req.body);
        json defaultAddresses;
        readDefaultAddressesFromJson(defaultAddresses);

        // 确保defaultAddresses中存在"current"节点，如果不存在则创建一个空对象作为"current"节点
        if (!defaultAddresses.contains("current"))
        {
            defaultAddresses["current"] = json::object();
        }

        // 更新各个默认地址信息，只操作"current"节点下的字段
        defaultAddresses["current"]["defaultRtspStreamUrl"] = reqJson.value("rtspStreamUrl", "");
        defaultAddresses["current"]["defaultSaveLocation"] = reqJson.value("saveLocation", "");
        defaultAddresses["current"]["defaultRtspStreamUrl2"] = reqJson.value("rtspStreamUrl2", "");
        defaultAddresses["current"]["defaultSaveLocation2"] = reqJson.value("saveLocation2", "");

        updateDefaultAddressesInJson(defaultAddresses);

        res.status = 200;
        res.set_content("Default addresses updated successfully.", "text/plain");
    }
    catch (const std::exception& e)
    {
        res.status = 500;
        res.set_content("Error updating default addresses: " + std::string(e.what()), "text/plain");
    }
});
//----------------------------------------------------------------------------------------------------------------------------
// 处理设置默认地址（更新default.json文件中"initial"部分）的路由
svr.Post("/set_default_addresses", [](const Request &req, Response &res)
{
    try
    {
        json reqJson = json::parse(req.body);
        json defaultAddresses;
        readDefaultAddressesFromJson(defaultAddresses);

        // 确保defaultAddresses中存在"initial"节点，如果不存在则创建一个空对象作为"initial"节点
        if (!defaultAddresses.contains("initial"))
        {
            defaultAddresses["initial"] = json::object();
        }

        // 更新"initial"节点下的各个默认地址信息
        defaultAddresses["initial"]["defaultRtspStreamUrl"] = reqJson.value("rtspStreamUrl", "");
        defaultAddresses["initial"]["defaultSaveLocation"] = reqJson.value("saveLocation", "");
        defaultAddresses["initial"]["defaultRtspStreamUrl2"] = reqJson.value("rtspStreamUrl2", "");
        defaultAddresses["initial"]["defaultSaveLocation2"] = reqJson.value("saveLocation2", "");

        updateDefaultAddressesInJson(defaultAddresses);

        res.status = 200;
        res.set_content("默认地址已设置成功，并更新到 'initial' 部分。", "text/plain");
    }
    catch (const std::exception &e)
    {
        res.status = 500;
        res.set_content("默认地址设置出错: " + std::string(e.what()), "text/plain");
    }
});
//----------------------------------------------------------------------------------------------------------------------------
svr.Post("/init_default_addresses", [](const Request &req, Response &res)
{
    try
    {
        // 从default.json文件读取配置信息
        std::ifstream file("default.json");
        if (file.is_open())
        {
            nlohmann::json configData;
            file >> configData;

            // 获取初始的默认地址配置部分（从"initial"节点下获取）
            nlohmann::json initialAddresses = configData["initial"];

            // 确保"current"节点存在，如果不存在则创建一个空的对象作为"current"节点
            if (!configData.contains("current"))
            {
                configData["current"] = nlohmann::json::object();
            }

            // 将初始地址覆盖到"current"节点下对应的字段中
            configData["current"]["defaultRtspStreamUrl"] = initialAddresses["defaultRtspStreamUrl"];
            configData["current"]["defaultRtspStreamUrl2"] = initialAddresses["defaultRtspStreamUrl2"];
            configData["current"]["defaultSaveLocation"] = initialAddresses["defaultSaveLocation"];
            configData["current"]["defaultSaveLocation2"] = initialAddresses["defaultSaveLocation2"];

            updateDefaultAddressesInJson(configData);

            res.status = 200;
            res.set_content("Default addresses have been initialized.", "text/plain");
            file.close();
        }
        else
        {
            res.status = 500;
            res.set_content("Error opening default.json file.", "text/plain");
        }
    }
    catch (...)
    {
        res.status = 500;
        res.set_content("Error parsing default.json file during initialization.", "text/plain");
    }
});
    //----------------------------------------------------------------------------------------------------------------------------
    // 添加人员检测API路由
    svr.Get("/api/human_detected", HumanDetection::handleHumanDetectedAPI);
    
    // 添加烟火检测API路由
    svr.Get("/api/fire_detected", FireDetection::handleFireDetectedAPI);
    
    // 添加综合检测状态API（同时返回人员检测和烟火检测状态）
    svr.Get("/api/detection_status", [](const Request &req, Response &res)
    {
        try {
            // 读取GPIO状态
            bool channel20_detected = HumanDetection::readGPIOStatus("/sys/class/gpio/gpio429/value");
            bool channel10_detected = HumanDetection::readGPIOStatus("/sys/class/gpio/gpio430/value");
            bool fire_detected = FireDetection::readGPIOStatus("/sys/class/gpio/gpio344/value");
            
            // 构建响应JSON
            json responseJson;
            responseJson["status"] = "success";
            responseJson["timestamp"] = HumanDetection::getCurrentTimestamp();
            
            // 人员检测状态
            responseJson["human_detection"] = json::object();
            responseJson["human_detection"]["channel20"] = {
                {"detected", !channel20_detected},
                {"gpio_pin", 429},
                {"gpio_value", channel20_detected ? 1 : 0}
            };
            responseJson["human_detection"]["channel10"] = {
                {"detected", !channel10_detected},
                {"gpio_pin", 430},
                {"gpio_value", channel10_detected ? 1 : 0}
            };
            
            // 烟火检测状态
            responseJson["fire_detection"] = {
                {"detected", fire_detected},  // GPIO 344: 高电平=检测到火焰/烟雾
                {"gpio_pin", 344},
                {"gpio_value", fire_detected ? 1 : 0}
            };

            // 设置CORS头
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
            res.set_header("Access-Control-Allow-Headers", "Content-Type");
            
            // 返回JSON响应
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            errorJson["timestamp"] = HumanDetection::getCurrentTimestamp();
            
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    // 获取芯片温度API
    svr.Get("/api/chip_temp", [](const Request &req, Response &res)
    {
        try {
            // 读取芯片温度（单位为毫摄氏度）
            std::ifstream tempFile("/sys/class/thermal/thermal_zone0/temp");
            if (tempFile.is_open()) {
                std::string tempStr;
                std::getline(tempFile, tempStr);
                tempFile.close();
                
                // 转换为摄氏度
                double temp = std::stod(tempStr) / 1000.0;
                
                json responseJson;
                responseJson["status"] = "success";
                responseJson["temperature"] = temp;
                responseJson["unit"] = "celsius";
                
                res.set_header("Access-Control-Allow-Origin", "*");
                res.set_content(responseJson.dump(), "application/json");
            } else {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Unable to read temperature file";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
            }
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    // 获取系统资源使用率API
    svr.Get("/api/system_resources", [](const Request &req, Response &res)
    {
        try {
            // 获取CPU使用率
            std::string topOutput = executeCommand("top -bn1 | grep 'Cpu(s)' | awk '{print $2}' | cut -d'%' -f1");
            double cpuUsage = topOutput.empty() ? 0.0 : std::stod(topOutput);
            
            // 获取内存使用率
            std::string freeOutput = executeCommand("free | grep Mem | awk '{print ($3/$2) * 100.0}'");
            double memUsage = freeOutput.empty() ? 0.0 : std::stod(freeOutput);
            
            // NPU使用率（如果系统有的话，这里使用模拟数据）
            // 可以根据实际系统调整命令
            std::string npuOutput = executeCommand("cat /sys/kernel/debug/bm-sophon/npu_usage 2>/dev/null || echo '0'");
            double npuUsage = npuOutput.empty() ? 0.0 : std::stod(npuOutput);
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["cpu"] = cpuUsage;
            responseJson["memory"] = memUsage;
            responseJson["npu"] = npuUsage;
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    // 获取设备信息API
    svr.Get("/api/device_info", [](const Request &req, Response &res)
    {
        try {
            // 获取设备ID（MAC地址或其他唯一标识）
            std::string deviceId = executeCommand("cat /sys/class/net/eth0/address 2>/dev/null || echo 'Unknown'");
            deviceId.erase(std::remove(deviceId.begin(), deviceId.end(), '\n'), deviceId.end());
            
            // 获取系统版本
            std::string systemVersion = executeCommand("uname -r");
            systemVersion.erase(std::remove(systemVersion.begin(), systemVersion.end(), '\n'), systemVersion.end());
            
            // 获取存储信息
            TFCardInfo tfCardInfo = getTFCardInfo();
            std::string storageInfo = "存储空间(" + tfCardInfo.usedMemory + "/" + tfCardInfo.freeMemory + ")";
            
            // 获取软件版本（可以从配置文件读取）
            std::string softwareVersion = "1.0.65 Patch 2";
            
            // GPS信息（如果有的话）
            std::string gpsInfo = "纬度(-) 经度(-) UTC时间(-)";
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["deviceId"] = deviceId;
            responseJson["systemVersion"] = systemVersion;
            responseJson["storage"] = storageInfo;
            responseJson["softwareVersion"] = softwareVersion;
            responseJson["gps"] = gpsInfo;
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    // 获取通道列表API
    svr.Get("/api/channels", [](const Request &req, Response &res)
    {
        try {
            std::ifstream channelsFile("channels.json");
            json responseJson;
            
            if (channelsFile.is_open()) {
                json channelsData;
                channelsFile >> channelsData;
                channelsFile.close();
                
                responseJson["status"] = "success";
                responseJson["channels"] = channelsData["channels"];
            } else {
                // 如果文件不存在，返回空数组
                responseJson["status"] = "success";
                responseJson["channels"] = json::array();
            }
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    // 添加通道API
    svr.Post("/api/channels", [](const Request &req, Response &res)
    {
        try {
            json requestData = json::parse(req.body);
            
            // 读取现有通道
            json channelsData;
            std::ifstream channelsFile("channels.json");
            if (channelsFile.is_open()) {
                channelsFile >> channelsData;
                channelsFile.close();
            } else {
                channelsData["channels"] = json::array();
            }
            
            // 生成新ID
            int newId = 1;
            if (!channelsData["channels"].empty()) {
                for (const auto& channel : channelsData["channels"]) {
                    if (channel["id"] >= newId) {
                        newId = channel["id"].get<int>() + 1;
                    }
                }
            }
            
            // 创建新通道
            json newChannel;
            newChannel["id"] = newId;
            newChannel["name"] = requestData["name"];
            newChannel["url"] = requestData["url"];
            newChannel["channelAssignment"] = requestData["channelAssignment"];
            newChannel["description"] = requestData.value("description", "");
            newChannel["gb28181"] = requestData.value("gb28181", "");
            newChannel["status"] = requestData.value("status", "configured");
            
            // 添加到数组
            channelsData["channels"].push_back(newChannel);
            
            // 保存到文件
            std::ofstream outFile("channels.json");
            if (outFile.is_open()) {
                outFile << channelsData.dump(4);
                outFile.close();
                
                json responseJson;
                responseJson["status"] = "success";
                responseJson["message"] = "Channel added successfully";
                responseJson["channel"] = newChannel;
                
                res.set_header("Access-Control-Allow-Origin", "*");
                res.set_content(responseJson.dump(), "application/json");
            } else {
                throw std::runtime_error("Cannot write to channels.json");
            }
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    // 更新通道API
    svr.Put(R"(/api/channels/(\d+))", [](const Request &req, Response &res)
    {
        try {
            int channelId = std::stoi(req.matches[1]);
            json requestData = json::parse(req.body);
            
            // 读取现有通道
            std::ifstream channelsFile("channels.json");
            if (!channelsFile.is_open()) {
                throw std::runtime_error("Cannot read channels.json");
            }
            
            json channelsData;
            channelsFile >> channelsData;
            channelsFile.close();
            
            // 查找并更新通道
            bool found = false;
            for (auto& channel : channelsData["channels"]) {
                if (channel["id"] == channelId) {
                    channel["name"] = requestData["name"];
                    channel["url"] = requestData["url"];
                    channel["channelAssignment"] = requestData["channelAssignment"];
                    channel["description"] = requestData.value("description", "");
                    channel["gb28181"] = requestData.value("gb28181", "");
                    channel["status"] = requestData.value("status", channel["status"]);
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                throw std::runtime_error("Channel not found");
            }
            
            // 保存到文件
            std::ofstream outFile("channels.json");
            if (outFile.is_open()) {
                outFile << channelsData.dump(4);
                outFile.close();
                
                json responseJson;
                responseJson["status"] = "success";
                responseJson["message"] = "Channel updated successfully";
                
                res.set_header("Access-Control-Allow-Origin", "*");
                res.set_content(responseJson.dump(), "application/json");
            } else {
                throw std::runtime_error("Cannot write to channels.json");
            }
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    // 删除通道API
    svr.Delete(R"(/api/channels/(\d+))", [](const Request &req, Response &res)
    {
        try {
            int channelId = std::stoi(req.matches[1]);
            
            // 读取现有通道
            std::ifstream channelsFile("channels.json");
            if (!channelsFile.is_open()) {
                throw std::runtime_error("Cannot read channels.json");
            }
            
            json channelsData;
            channelsFile >> channelsData;
            channelsFile.close();
            
            // 删除通道
            auto& channels = channelsData["channels"];
            bool found = false;
            for (auto it = channels.begin(); it != channels.end(); ++it) {
                if ((*it)["id"] == channelId) {
                    channels.erase(it);
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                throw std::runtime_error("Channel not found");
            }
            
            // 保存到文件
            std::ofstream outFile("channels.json");
            if (outFile.is_open()) {
                outFile << channelsData.dump(4);
                outFile.close();
                
                json responseJson;
                responseJson["status"] = "success";
                responseJson["message"] = "Channel deleted successfully";
                
                res.set_header("Access-Control-Allow-Origin", "*");
                res.set_content(responseJson.dump(), "application/json");
            } else {
                throw std::runtime_error("Cannot write to channels.json");
            }
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    // 检测RTSP流地址是否可用API
    svr.Post("/api/channels/check", [](const Request &req, Response &res)
    {
        try {
            json requestData = json::parse(req.body);
            std::string url = requestData["url"];
            
            // 使用 sophon-ffprobe 检测流地址
            // 设置超时时间为10秒，避免长时间等待
            std::string ffprobePath = "/opt/sophon/sophon-ffmpeg-latest/bin/ffprobe";
            std::string checkCommand = "timeout 10 " + ffprobePath + " -v error -show_entries stream=codec_type -of default=noprint_wrappers=1 \"" + url + "\" 2>&1";
            std::string result = executeCommand(checkCommand.c_str());
            
            // 调试日志
            std::cout << "检测流地址: " << url << std::endl;
            std::cout << "ffprobe输出: " << result << std::endl;
            
            json responseJson;
            responseJson["status"] = "success";
            
            // 检查ffprobe输出 - 只要找到codec_type=video就认为流可用
            if (result.find("codec_type=video") != std::string::npos) {
                responseJson["streamStatus"] = "active";
                responseJson["message"] = "流地址正常";
                
                // 尝试获取分辨率信息
                std::string resCommand = "timeout 10 " + ffprobePath + " -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0 \"" + url + "\" 2>&1";
                std::string resResult = executeCommand(resCommand.c_str());
                std::string compactResolution;
                compactResolution.reserve(resResult.size());
                for (char ch : resResult) {
                    if (!std::isspace(static_cast<unsigned char>(ch))) {
                        compactResolution.push_back(ch);
                    }
                }

                std::string normalizedResolution;
                for (size_t i = 0; i < compactResolution.size();) {
                    if (!std::isdigit(static_cast<unsigned char>(compactResolution[i]))) {
                        ++i;
                        continue;
                    }

                    size_t start = i;
                    while (i < compactResolution.size() &&
                           std::isdigit(static_cast<unsigned char>(compactResolution[i]))) {
                        ++i;
                    }

                    if (i >= compactResolution.size() || compactResolution[i] != 'x') {
                        continue;
                    }

                    ++i;
                    size_t heightStart = i;
                    while (i < compactResolution.size() &&
                           std::isdigit(static_cast<unsigned char>(compactResolution[i]))) {
                        ++i;
                    }

                    if (heightStart < i) {
                        normalizedResolution = compactResolution.substr(start, i - start);
                        break;
                    }
                }
                
                responseJson["resolution"] = normalizedResolution;
            } else if (result.find("Connection refused") != std::string::npos) {
                responseJson["streamStatus"] = "error";
                responseJson["message"] = "连接被拒绝";
                responseJson["resolution"] = "";
            } else if (result.find("Connection timed out") != std::string::npos || 
                       result.find("timed out") != std::string::npos) {
                responseJson["streamStatus"] = "error";
                responseJson["message"] = "连接超时";
                responseJson["resolution"] = "";
            } else if (result.find("Invalid data") != std::string::npos) {
                responseJson["streamStatus"] = "invalid";
                responseJson["message"] = "流数据无效";
                responseJson["resolution"] = "";
            } else if (result.find("Protocol not found") != std::string::npos) {
                responseJson["streamStatus"] = "invalid";
                responseJson["message"] = "协议不支持";
                responseJson["resolution"] = "";
            } else if (result.find("Server returned 404") != std::string::npos || 
                       result.find("not found") != std::string::npos) {
                responseJson["streamStatus"] = "invalid";
                responseJson["message"] = "流地址不存在";
                responseJson["resolution"] = "";
            } else {
                // 如果没有明确的错误但也没检测到视频，返回详细信息用于调试
                responseJson["streamStatus"] = "inactive";
                responseJson["message"] = "流地址无法访问";
                responseJson["resolution"] = "";
                responseJson["debug"] = result.substr(0, 200); // 返回前200字符用于调试
            }
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["streamStatus"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    // 获取已分配的通道地址API
    svr.Get("/api/channels/assigned", [](const Request &req, Response &res)
    {
        try {
            std::ifstream channelsFile("channels.json");
            json responseJson;
            responseJson["status"] = "success";
            responseJson["channel1"] = "";
            responseJson["channel2"] = "";
            
            if (channelsFile.is_open()) {
                json channelsData;
                channelsFile >> channelsData;
                channelsFile.close();
                
                // 查找分配给通道1和通道2的地址
                for (const auto& channel : channelsData["channels"]) {
                    if (channel["channelAssignment"] == "1") {
                        responseJson["channel1"] = channel["url"];
                    } else if (channel["channelAssignment"] == "2") {
                        responseJson["channel2"] = channel["url"];
                    }
                }
            }
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    // 人脸管理API
    svr.Get("/api/faces", [](const Request &req, Response &res)
    {
        try {
            ensureDirectory(FACE_TRAIN_DIR);
            std::map<std::string, int> featureCountMap = loadFaceFeatureCountMap();
            std::vector<std::string> faceNames = listFaceDirectoryNames();
            json labelMap = loadFaceLabelMap();
            syncFaceLabelMapWithCurrentFaces(faceNames, labelMap);
            saveFaceLabelMap(labelMap);

            json faceArray = json::array();
            for (size_t i = 0; i < faceNames.size(); ++i)
            {
                const std::string& faceName = faceNames[i];
                std::string faceLabel = getFaceLabelForName(labelMap, faceName);
                std::vector<std::string> imageFiles = listImageFiles(FACE_TRAIN_DIR + "/" + faceName);
                json faceItem;
                faceItem["name"] = faceName;
                faceItem["label"] = faceLabel;
                faceItem["imageCount"] = static_cast<int>(imageFiles.size());
                faceItem["featureCount"] = featureCountMap.count(faceLabel) ? featureCountMap[faceLabel] : 0;
                faceArray.push_back(faceItem);
            }

            json responseJson;
            responseJson["status"] = "success";
            responseJson["faces"] = faceArray;
            responseJson["trainDir"] = FACE_TRAIN_DIR;
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });

    svr.Get("/api/faces/preview", [](const Request &req, Response &res)
    {
        try {
            if (!req.has_param("name"))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Missing face name";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::string faceName = trimString(req.get_param_value("name"));
            if (!isValidFaceName(faceName))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Invalid face name";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::string faceDir = FACE_TRAIN_DIR + "/" + faceName;
            if (!isDirectoryPath(faceDir))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Face not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::vector<std::string> imageFiles = listImageFiles(faceDir);
            if (imageFiles.empty())
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "No image found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::string firstImagePath = faceDir + "/" + imageFiles[0];
            std::string content;
            if (!readBinaryContent(firstImagePath, content))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Failed to read image";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            res.set_header("Cache-Control", "no-cache");
            res.set_content(content, guessImageMimeType(firstImagePath));
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });

    svr.Post("/api/faces/register", [](const Request &req, Response &res)
    {
        try {
            std::string faceName = "";
            if (req.has_param("name"))
            {
                faceName = req.get_param_value("name");
            }
            else if (req.has_file("name"))
            {
                faceName = req.get_file_value("name").content;
            }
            faceName = trimString(faceName);

            if (!isValidFaceName(faceName))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Invalid face name";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::string faceLabel = "";
            if (req.has_param("label"))
            {
                faceLabel = req.get_param_value("label");
            }
            else if (req.has_file("label"))
            {
                faceLabel = req.get_file_value("label").content;
            }
            faceLabel = trimString(faceLabel);
            if (!faceLabel.empty() && !isValidAsciiFaceLabel(faceLabel))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Invalid face label, only letters/numbers/_/- are allowed";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::vector<MultipartFormData> files;
            if (req.has_file("images"))
            {
                std::vector<MultipartFormData> values = req.get_file_values("images");
                files.insert(files.end(), values.begin(), values.end());
            }
            if (files.empty() && req.has_file("image"))
            {
                std::vector<MultipartFormData> values = req.get_file_values("image");
                files.insert(files.end(), values.begin(), values.end());
            }

            if (files.empty())
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Please upload at least one image";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            if (!ensureDirectory(FACE_TRAIN_DIR))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Failed to create training directory";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::string faceDir = FACE_TRAIN_DIR + "/" + faceName;
            if (!ensureDirectory(faceDir))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Failed to create face directory";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            int savedCount = 0;
            int seed = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count() % 1000000000);

            for (size_t i = 0; i < files.size(); ++i)
            {
                const MultipartFormData& fileData = files[i];
                if (fileData.content.empty())
                {
                    continue;
                }

                std::string fileName = sanitizeUploadFilename(fileData.filename, seed + static_cast<int>(i));
                std::string savePath = faceDir + "/" + fileName;

                int suffix = 1;
                while (pathExists(savePath))
                {
                    size_t dotPos = fileName.find_last_of('.');
                    std::string base = (dotPos == std::string::npos) ? fileName : fileName.substr(0, dotPos);
                    std::string ext = (dotPos == std::string::npos) ? "" : fileName.substr(dotPos);
                    savePath = faceDir + "/" + base + "_" + std::to_string(suffix++) + ext;
                }

                std::ofstream out(savePath.c_str(), std::ios::binary);
                if (!out.good())
                {
                    continue;
                }
                out.write(fileData.content.data(), static_cast<std::streamsize>(fileData.content.size()));
                out.close();
                savedCount++;
            }

            if (savedCount <= 0)
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "No valid image saved";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            json labelMap = loadFaceLabelMap();
            std::string assignedLabel = ensureFaceLabelForName(faceName, faceLabel, labelMap);
            saveFaceLabelMap(labelMap);

            std::string rebuildLog;
            int rebuildRet = rebuildFaceFeatureDatabase(rebuildLog);
            if (rebuildRet != 0)
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Face database rebuild failed";
                if (!rebuildLog.empty())
                {
                    errorJson["log"] = rebuildLog.substr(0, 1200);
                }
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Face registered successfully";
            responseJson["name"] = faceName;
            responseJson["label"] = assignedLabel;
            responseJson["savedCount"] = savedCount;
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });

    svr.Post("/api/faces/batch_register", [](const Request &req, Response &res)
    {
        std::string tempDir;
        auto cleanup = [&]() {
            if (!tempDir.empty())
            {
                std::string rmCmd = "rm -rf -- " + shellQuote(tempDir);
                std::system(rmCmd.c_str());
                tempDir.clear();
            }
        };
        auto sendError = [&](int status, const std::string& message, const std::string& log = std::string()) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = message;
            if (!log.empty())
            {
                errorJson["log"] = log.substr(0, 2000);
            }
            cleanup();
            res.status = status;
            res.set_content(errorJson.dump(), "application/json");
        };

        try {
            MultipartFormData archiveData;
            bool hasArchive = false;
            if (req.has_file("archive"))
            {
                archiveData = req.get_file_value("archive");
                hasArchive = true;
            }
            else if (req.has_file("zip"))
            {
                archiveData = req.get_file_value("zip");
                hasArchive = true;
            }

            if (!hasArchive || archiveData.content.empty())
            {
                sendError(400, "请上传 zip 压缩包");
                return;
            }

            if (!ensureDirectory(FACE_TRAIN_DIR))
            {
                sendError(500, "无法创建人脸训练目录");
                return;
            }

            if (!pathExists(FACE_BATCH_IMPORT_SCRIPT))
            {
                sendError(500, "批量导入脚本不存在");
                return;
            }

            std::string tempPattern = "/tmp/face_batch_import_XXXXXX";
            std::vector<char> tempBuffer(tempPattern.begin(), tempPattern.end());
            tempBuffer.push_back('\0');
            char* createdDir = mkdtemp(tempBuffer.data());
            if (!createdDir)
            {
                sendError(500, "无法创建临时导入目录");
                return;
            }
            tempDir = createdDir;

            std::string archivePath = tempDir + "/upload_faces.zip";
            std::ofstream archiveOut(archivePath.c_str(), std::ios::binary | std::ios::trunc);
            if (!archiveOut.good())
            {
                sendError(500, "无法写入上传的压缩包");
                return;
            }
            archiveOut.write(archiveData.content.data(), static_cast<std::streamsize>(archiveData.content.size()));
            archiveOut.close();
            if (!archiveOut.good())
            {
                sendError(500, "压缩包保存失败");
                return;
            }

            std::string resultPath = tempDir + "/batch_import_result.json";
            std::string logPath = tempDir + "/batch_import.log";
            std::string importCommand =
                "python3 " + shellQuote(FACE_BATCH_IMPORT_SCRIPT) +
                " --zip " + shellQuote(archivePath) +
                " --train-dir " + shellQuote(FACE_TRAIN_DIR) +
                " --result " + shellQuote(resultPath) +
                " > " + shellQuote(logPath) + " 2>&1";

            int importRet = std::system(importCommand.c_str());
            json importResult;
            std::string loadError;
            if (!loadJsonFile(resultPath, importResult, loadError) || !importResult.is_object())
            {
                std::string scriptLog;
                readBinaryContent(logPath, scriptLog);
                sendError(500, "批量导入结果解析失败", scriptLog.empty() ? loadError : scriptLog);
                return;
            }

            if (importRet != 0 || importResult.value("status", "") != "success")
            {
                std::string importMessage = trimString(importResult.value("message", ""));
                std::string scriptLog;
                readBinaryContent(logPath, scriptLog);
                sendError(400,
                          importMessage.empty() ? "批量导入失败" : importMessage,
                          scriptLog.empty() ? importMessage : scriptLog);
                return;
            }

            json importedFaces = importResult.value("faces", json::array());
            if (!importedFaces.is_array() || importedFaces.empty())
            {
                std::string scriptLog;
                readBinaryContent(logPath, scriptLog);
                sendError(400, "压缩包中未解析到有效的人脸文件夹", scriptLog);
                return;
            }

            json labelMap = loadFaceLabelMap();
            json responseFaces = json::array();
            for (size_t i = 0; i < importedFaces.size(); ++i)
            {
                if (!importedFaces[i].is_object())
                {
                    continue;
                }
                std::string faceName = trimString(importedFaces[i].value("name", ""));
                int savedCount = importedFaces[i].value("savedCount", 0);
                if (!isValidFaceName(faceName) || !isValidAsciiFaceLabel(faceName))
                {
                    continue;
                }

                std::string assignedLabel = ensureFaceLabelForName(faceName, faceName, labelMap);
                json faceItem;
                faceItem["name"] = faceName;
                faceItem["label"] = assignedLabel;
                faceItem["savedCount"] = savedCount;
                responseFaces.push_back(faceItem);
            }

            if (responseFaces.empty())
            {
                sendError(400, "压缩包中没有符合要求的英文文件夹");
                return;
            }

            if (!saveFaceLabelMap(labelMap))
            {
                sendError(500, "保存人脸标签映射失败");
                return;
            }

            std::string rebuildLog;
            int rebuildRet = rebuildFaceFeatureDatabase(rebuildLog);
            if (rebuildRet != 0)
            {
                sendError(500, "人脸库重建失败", rebuildLog);
                return;
            }

            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "批量导入完成";
            responseJson["archiveName"] = archiveData.filename.empty() ? "upload_faces.zip" : archiveData.filename;
            responseJson["totalFaces"] = static_cast<int>(responseFaces.size());
            responseJson["totalSaved"] = importResult.value("total_saved", 0);
            responseJson["faces"] = responseFaces;
            responseJson["warnings"] = importResult.value("warnings", json::array());
            cleanup();
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            sendError(500, e.what());
        }
    });

    svr.Post("/api/faces/delete", [](const Request &req, Response &res)
    {
        try {
            json requestData = json::parse(req.body);
            std::string faceName = trimString(requestData.value("name", ""));
            if (!isValidFaceName(faceName))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Invalid face name";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::string faceDir = FACE_TRAIN_DIR + "/" + faceName;
            if (!isDirectoryPath(faceDir))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Face not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::string rmCmd = "rm -rf -- " + shellQuote(faceDir);
            if (std::system(rmCmd.c_str()) != 0)
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Failed to delete face directory";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            json labelMap = loadFaceLabelMap();
            if (labelMap.is_object() && labelMap.contains(faceName))
            {
                labelMap.erase(faceName);
                saveFaceLabelMap(labelMap);
            }

            std::string rebuildLog;
            int rebuildRet = rebuildFaceFeatureDatabase(rebuildLog);
            if (rebuildRet != 0)
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Face database rebuild failed";
                if (!rebuildLog.empty())
                {
                    errorJson["log"] = rebuildLog.substr(0, 1200);
                }
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Face deleted successfully";
            responseJson["name"] = faceName;
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });

    svr.Post("/api/faces/batch_delete", [](const Request &req, Response &res)
    {
        try {
            json requestData = json::parse(req.body.empty() ? "{}" : req.body);
            if (!requestData.contains("names") || !requestData["names"].is_array())
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Missing names array";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::set<std::string> uniqueNames;
            json warnings = json::array();
            const json& namesJson = requestData["names"];
            for (size_t i = 0; i < namesJson.size(); ++i)
            {
                if (!namesJson[i].is_string())
                {
                    warnings.push_back("已忽略无效名称项");
                    continue;
                }
                std::string faceName = trimString(namesJson[i].get<std::string>());
                if (!isValidFaceName(faceName))
                {
                    warnings.push_back("已忽略非法名称: " + faceName);
                    continue;
                }
                uniqueNames.insert(faceName);
            }

            if (uniqueNames.empty())
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "No valid face names provided";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            json labelMap = loadFaceLabelMap();
            json deletedNames = json::array();
            int deletedCount = 0;

            for (std::set<std::string>::const_iterator it = uniqueNames.begin(); it != uniqueNames.end(); ++it)
            {
                const std::string& faceName = *it;
                std::string faceDir = FACE_TRAIN_DIR + "/" + faceName;
                if (!isDirectoryPath(faceDir))
                {
                    warnings.push_back("未找到: " + faceName);
                    continue;
                }

                std::string rmCmd = "rm -rf -- " + shellQuote(faceDir);
                int rmRet = std::system(rmCmd.c_str());
                if (rmRet != 0 && isDirectoryPath(faceDir))
                {
                    warnings.push_back("删除失败: " + faceName);
                    continue;
                }

                if (labelMap.is_object() && labelMap.contains(faceName))
                {
                    labelMap.erase(faceName);
                }
                deletedNames.push_back(faceName);
                deletedCount++;
            }

            if (deletedCount <= 0)
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "No face deleted";
                errorJson["warnings"] = warnings;
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            if (!saveFaceLabelMap(labelMap))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Failed to save face label map";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::string rebuildLog;
            int rebuildRet = rebuildFaceFeatureDatabase(rebuildLog);
            if (rebuildRet != 0)
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Face database rebuild failed";
                if (!rebuildLog.empty())
                {
                    errorJson["log"] = rebuildLog.substr(0, 1200);
                }
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "批量删除完成";
            responseJson["deletedCount"] = deletedCount;
            responseJson["deletedNames"] = deletedNames;
            responseJson["warnings"] = warnings;
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });

    svr.Post("/api/faces/rebuild", [](const Request &req, Response &res)
    {
        try {
            std::string rebuildLog;
            int rebuildRet = rebuildFaceFeatureDatabase(rebuildLog);
            if (rebuildRet != 0)
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Face database rebuild failed";
                if (!rebuildLog.empty())
                {
                    errorJson["log"] = rebuildLog.substr(0, 1200);
                }
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Face database rebuilt successfully";
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });

    svr.Post(REGISTERED_PERSON_TRACKING_HTTP_REPORT_PATH.c_str(), [](const Request &req, Response &res)
    {
        json payload = json::parse(req.body, nullptr, false);
        if (!payload.is_object())
        {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = "Invalid payload";
            res.status = 400;
            res.set_content(errorJson.dump(), "application/json");
            return;
        }

        int streamChannelId = 0;
        if (payload.contains("mFrame") && payload["mFrame"].is_object())
        {
            streamChannelId = payload["mFrame"].value("mChannelId", 0);
        }

        int taskId = resolveRegisteredPersonTrackingTaskIdByStreamChannelId(streamChannelId);
        if (taskId > 0)
        {
            processRegisteredPersonTrackingPayload(taskId, streamChannelId, payload);
        }

        json responseJson;
        responseJson["status"] = "success";
        responseJson["taskId"] = taskId;
        responseJson["streamChannelId"] = streamChannelId;
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_content(responseJson.dump(), "application/json");
    });

    svr.Get("/api/registered-person-tracking/stats", [](const Request &req, Response &res)
    {
        json responseJson;
        responseJson["status"] = "success";

        std::lock_guard<std::mutex> lock(registeredPersonTrackingStateMutex);
        ensureRegisteredPersonTrackingPersistenceLoadedUnlocked();
        if (req.has_param("taskId"))
        {
            int taskId = 0;
            try
            {
                taskId = std::stoi(req.get_param_value("taskId"));
            }
            catch (...)
            {
                taskId = 0;
            }

            if (taskId > 0)
            {
                auto it = registeredPersonTrackingStates.find(taskId);
                if (it != registeredPersonTrackingStates.end())
                {
                    responseJson["task"] = buildRegisteredPersonTrackingTaskStatsJsonUnlocked(it->second);
                }
                else
                {
                    responseJson["task"] = {
                        {"taskId", taskId},
                        {"frameWidth", 0},
                        {"frameHeight", 0},
                        {"lastFrameId", 0},
                        {"lastFrameTimestampMs", 0},
                        {"records", json::array()},
                        {"persons", json::array()},
                        {"tracks", json::array()},
                        {"currentRegisteredTrackCount", 0},
                        {"currentRegisteredPersonCount", 0}
                    };
                }
            }
            else
            {
                responseJson["task"] = {
                    {"taskId", 0},
                    {"frameWidth", 0},
                    {"frameHeight", 0},
                    {"lastFrameId", 0},
                    {"lastFrameTimestampMs", 0},
                    {"records", json::array()},
                    {"persons", json::array()},
                    {"tracks", json::array()},
                    {"currentRegisteredTrackCount", 0},
                    {"currentRegisteredPersonCount", 0}
                };
            }
        }
        else
        {
            json tasksArray = json::array();
            for (auto it = registeredPersonTrackingStates.begin();
                 it != registeredPersonTrackingStates.end();
                 ++it)
            {
                tasksArray.push_back(buildRegisteredPersonTrackingTaskStatsJsonUnlocked(it->second));
            }
            responseJson["tasks"] = tasksArray;
        }
        flushRegisteredPersonTrackingPersistenceUnlocked();

        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_content(responseJson.dump(), "application/json");
    });

    svr.Delete("/api/registered-person-tracking/stats", [](const Request &req, Response &res)
    {
        json responseJson;
        int taskId = 0;
        if (req.has_param("taskId"))
        {
            try
            {
                taskId = std::stoi(req.get_param_value("taskId"));
            }
            catch (...)
            {
                taskId = 0;
            }
        }

        if (taskId <= 0)
        {
            responseJson["status"] = "error";
            responseJson["message"] = "taskId 无效";
            responseJson["taskId"] = 0;
            res.status = 400;
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
            return;
        }

        bool cleared = resetRegisteredPersonTrackingTaskStats(taskId);
        responseJson["status"] = "success";
        responseJson["taskId"] = taskId;
        responseJson["cleared"] = cleared;
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_content(responseJson.dump(), "application/json");
    });

    //----------------------------------------------------------------------------------------------------------------------------
    // 任务管理API
    
    // 全局变量：记录最后一次停止任务的时间
    static std::chrono::time_point<std::chrono::system_clock> lastStopTime = std::chrono::system_clock::now() - std::chrono::seconds(60);
    static std::mutex stopTimeMutex;
    static std::mutex taskOperationMutex;
    
    // 获取所有任务
    svr.Get("/api/tasks", [](const Request &req, Response &res)
    {
        try {
            std::ifstream file("/data/lintech/celectronicfence/tasks.json");
            json tasksJson;
            
            if (file.is_open()) {
                file >> tasksJson;
                file.close();
            } else {
                tasksJson = json::array();
            }
            
            // 检查是否在停止任务后的5秒内（避免自动同步覆盖停止状态）
            bool skipAutoSync = false;
            {
                std::lock_guard<std::mutex> lock(stopTimeMutex);
                auto timeSinceStop = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::system_clock::now() - lastStopTime).count();
                if (timeSinceStop < 5) {
                    skipAutoSync = true;
                    std::cout << "[自动同步] 跳过自动同步（停止任务后 " << timeSinceStop << " 秒）" << std::endl;
                }
            }
            
            // 同步任务状态（如果不在禁用期内）
            bool taskUpdated = false;
            if (!skipAutoSync) {
                for (auto& task : tasksJson) {
                    int taskPid = task.value("pid", 0);
                    std::string taskStatus = task.value("status", "stopped");
                    bool isRunning = false;
                    bool pidRebound = false;

                    if (taskPid > 0) {
                        std::string reason;
                        isRunning = doesPidMatchTaskProcess(task, taskPid, reason);
                        if (!isRunning && taskStatus == "running") {
                            std::cout << "[自动同步] PID " << taskPid << " 不再属于该任务，原因: " << reason << std::endl;
                            int recoveredPid = discoverRunningTaskPid(task);
                            if (recoveredPid > 0 && recoveredPid != taskPid) {
                                task["pid"] = recoveredPid;
                                long long pidStartTime = 0;
                                if (readProcessStartTimeTicks(recoveredPid, pidStartTime) && pidStartTime > 0) {
                                    task["pidStartTime"] = pidStartTime;
                                } else {
                                    task.erase("pidStartTime");
                                }
                                isRunning = true;
                                pidRebound = true;
                                taskUpdated = true;
                                std::cout << "[自动同步] 任务PID已重绑定: " << taskPid << " -> " << recoveredPid << std::endl;
                            }
                        }
                    }

                    if (isRunning) {
                        if (taskStatus != "running" || pidRebound) {
                            task["status"] = "running";
                            taskUpdated = true;
                            std::cout << "[自动同步] 检测到运行中的任务 PID: " << task.value("pid", 0) << "，更新状态" << std::endl;
                        }
                    } else {
                        if (taskStatus == "running") {
                            if (isRegisteredPersonTrackingAlgorithm(task.value("algorithm", ""))) {
                                clearRegisteredPersonTrackingTaskState(task.value("id", 0));
                            }
                            task["status"] = "stopped";
                            task["pid"] = 0;
                            task.erase("pidStartTime");
                            taskUpdated = true;
                            std::cout << "[自动同步] 任务进程已停止，更新任务状态" << std::endl;
                        }
                    }
                }
            }
            
            // 如果状态有更新，保存到文件
            if (taskUpdated) {
                std::ofstream fileOut("/data/lintech/celectronicfence/tasks.json");
                if (fileOut.is_open()) {
                    fileOut << tasksJson.dump(4);
                    fileOut.close();
                }
            }
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["tasks"] = tasksJson;
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });

    svr.Get("/api/tasks/:id/regions", [](const Request &req, Response &res)
    {
        try
        {
            int taskId = std::stoi(req.path_params.at("id"));
            json tasksJson;
            std::string errorMessage;
            if (!loadTasksJson(tasksJson, errorMessage))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = errorMessage.empty() ? "Failed to load tasks.json" : errorMessage;
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            const json* task = findTaskById(tasksJson, taskId);
            if (task == NULL)
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Task not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            json responseJson;
            responseJson["status"] = "success";
            responseJson["task"] = buildTaskRegionTaskSummary(*task);
            responseJson["regions"] =
                task->contains("regionConfigs") && (*task)["regionConfigs"].is_array()
                    ? (*task)["regionConfigs"]
                    : json::array();

            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        }
        catch (const std::exception& e)
        {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });

    svr.Put("/api/tasks/:id/regions", [](const Request &req, Response &res)
    {
        try
        {
            int taskId = std::stoi(req.path_params.at("id"));
            json requestData = json::parse(req.body);

            json tasksJson;
            std::string errorMessage;
            if (!loadTasksJson(tasksJson, errorMessage))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = errorMessage.empty() ? "Failed to load tasks.json" : errorMessage;
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            json* task = findTaskById(tasksJson, taskId);
            if (task == NULL)
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Task not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            if (!supportsTaskRegionConfigAlgorithm(task->value("algorithm", "")))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Current task algorithm does not support region configuration";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            json sanitizedRegions;
            if (!sanitizeTaskRegionConfigs(
                    requestData.contains("regions") ? requestData["regions"] : json::array(),
                    *task,
                    sanitizedRegions,
                    errorMessage))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = errorMessage;
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            (*task)["regionConfigs"] = sanitizedRegions;
            if (!saveTasksJson(tasksJson, errorMessage))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = errorMessage.empty() ? "Failed to write tasks.json" : errorMessage;
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Task regions updated successfully";
            responseJson["task"] = buildTaskRegionTaskSummary(*task);
            responseJson["regions"] = sanitizedRegions;
            responseJson["restartRequired"] = task->value("status", "stopped") == "running";

            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        }
        catch (const std::exception& e)
        {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });

    svr.Get("/api/tasks/:id/regions/frame", [](const Request &req, Response &res)
    {
        try
        {
            int taskId = std::stoi(req.path_params.at("id"));

            json tasksJson;
            json channelsJson;
            std::string errorMessage;
            if (!loadTasksJson(tasksJson, errorMessage))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = errorMessage.empty() ? "Failed to load tasks.json" : errorMessage;
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }
            if (!loadChannelsJson(channelsJson, errorMessage))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = errorMessage.empty() ? "Failed to load channels.json" : errorMessage;
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            const json* task = findTaskById(tasksJson, taskId);
            if (task == NULL)
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Task not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            const json* channel = findChannelById(channelsJson, task->value("videoSourceId", 0));
            if (channel == NULL)
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Video source not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::string sourceUrl = trimString(channel->value("url", ""));
            if (sourceUrl.empty())
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Video source URL is empty";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            if (!ensureDirectory(TASK_REGION_FRAME_DIR))
            {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Snapshot directory is not writable";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            unsigned long long requestSequence =
                taskRegionFrameRequestSequence.fetch_add(1, std::memory_order_relaxed) + 1;
            std::string snapshotPath =
                TASK_REGION_FRAME_DIR + "/task_region_frame_" + std::to_string(taskId) +
                "_" + std::to_string(currentTimeMillis()) +
                "_" + std::to_string(requestSequence) + ".jpg";
            std::string captureErrorMessage;
            {
                std::lock_guard<std::mutex> captureLock(taskRegionFrameCaptureMutex);
                if (!captureRtspSnapshotWithError(sourceUrl, snapshotPath, captureErrorMessage))
                {
                    json errorJson;
                    errorJson["status"] = "error";
                    errorJson["message"] = captureErrorMessage.empty()
                        ? "Failed to capture frame from configured channel stream"
                        : captureErrorMessage;
                    res.status = 500;
                    res.set_content(errorJson.dump(), "application/json");
                    return;
                }
            }

            auto mm = std::make_shared<detail::mmap>(snapshotPath.c_str());
            if (!mm->is_open() || mm->size() == 0)
            {
                std::remove(snapshotPath.c_str());
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Failed to read captured frame";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
            res.set_content_provider(
                mm->size(),
                "image/jpeg",
                [mm](size_t offset, size_t length, DataSink &sink) -> bool
                {
                    sink.write(mm->data() + offset, length);
                    return true;
                },
                [mm, snapshotPath](bool)
                {
                    std::remove(snapshotPath.c_str());
                });
        }
        catch (const std::exception& e)
        {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 添加新任务
    svr.Post("/api/tasks", [](const Request &req, Response &res)
    {
        try {
            json requestData = json::parse(req.body);
            
            // 读取现有任务
            std::ifstream fileIn("/data/lintech/celectronicfence/tasks.json");
            json tasksJson;
            
            if (fileIn.is_open()) {
                fileIn >> tasksJson;
                fileIn.close();
            } else {
                tasksJson = json::array();
            }
            
            // 生成新ID
            int newId = 1;
            if (!tasksJson.empty()) {
                for (const auto& task : tasksJson) {
                    if (task.contains("id") && task["id"].is_number()) {
                        newId = std::max(newId, (int)task["id"] + 1);
                    }
                }
            }
            
            // 创建新任务
            json newTask;
            newTask["id"] = newId;
            newTask["taskNumber"] = requestData["taskNumber"];
            newTask["description"] = requestData.value("description", "");
            newTask["videoSourceId"] = requestData["videoSourceId"];
            newTask["algorithm"] = requestData["algorithm"];
            newTask["status"] = "stopped";
            newTask["pid"] = 0;
            
            // 保存远程上报地址
            if (requestData.contains("remoteUrl")) {
                newTask["remoteUrl"] = requestData["remoteUrl"];
            }
            if (requestData.contains("demoMode")) {
                newTask["demoMode"] = requestData["demoMode"];
            }
            
            // 保存阈值参数（区域入侵/人员检测）
            if (requestData.contains("threshold")) {
                newTask["threshold"] = requestData["threshold"];
            }
            if (requestData.contains("thresholdMode")) {
                newTask["thresholdMode"] = requestData["thresholdMode"];
            }
            
            // 保存烟火检测的双阈值
            if (requestData.contains("flameThreshold")) {
                newTask["flameThreshold"] = requestData["flameThreshold"];
            }
            if (requestData.contains("smokeThreshold")) {
                newTask["smokeThreshold"] = requestData["smokeThreshold"];
            }
            if (requestData.contains("fightThreshold")) {
                newTask["fightThreshold"] = requestData["fightThreshold"];
            }
            if (requestData.contains("fallThreshold")) {
                newTask["fallThreshold"] = requestData["fallThreshold"];
            }
            if (requestData.contains("fallPreset")) {
                newTask["fallPreset"] = requestData["fallPreset"];
            }
            if (requestData.contains("trackIouThresh")) {
                newTask["trackIouThresh"] = requestData["trackIouThresh"];
            }
            if (requestData.contains("maxTrackAge")) {
                newTask["maxTrackAge"] = requestData["maxTrackAge"];
            }
            if (requestData.contains("minAreaRatio")) {
                newTask["minAreaRatio"] = requestData["minAreaRatio"];
            }
            if (requestData.contains("uprightRatioMax")) {
                newTask["uprightRatioMax"] = requestData["uprightRatioMax"];
            }
            if (requestData.contains("fallRatioMin")) {
                newTask["fallRatioMin"] = requestData["fallRatioMin"];
            }
            if (requestData.contains("heightDropRatio")) {
                newTask["heightDropRatio"] = requestData["heightDropRatio"];
            }
            if (requestData.contains("staticFallRatioMin")) {
                newTask["staticFallRatioMin"] = requestData["staticFallRatioMin"];
            }
            if (requestData.contains("staticHeightRatioMax")) {
                newTask["staticHeightRatioMax"] = requestData["staticHeightRatioMax"];
            }
            if (requestData.contains("staticBottomRatio")) {
                newTask["staticBottomRatio"] = requestData["staticBottomRatio"];
            }
            if (requestData.contains("uprightFrames")) {
                newTask["uprightFrames"] = requestData["uprightFrames"];
            }
            if (requestData.contains("fallFrames")) {
                newTask["fallFrames"] = requestData["fallFrames"];
            }
            if (requestData.contains("staticFallFrames")) {
                newTask["staticFallFrames"] = requestData["staticFallFrames"];
            }
            if (requestData.contains("historyHoldFrames")) {
                newTask["historyHoldFrames"] = requestData["historyHoldFrames"];
            }
            if (requestData.contains("alarmHoldFrames")) {
                newTask["alarmHoldFrames"] = requestData["alarmHoldFrames"];
            }
            if (requestData.contains("minBottomRatio")) {
                newTask["minBottomRatio"] = requestData["minBottomRatio"];
            }
            if (requestData.contains("minCenterDropRatio")) {
                newTask["minCenterDropRatio"] = requestData["minCenterDropRatio"];
            }
            if (requestData.contains("faceSampleInterval")) {
                newTask["faceSampleInterval"] = requestData["faceSampleInterval"];
            }
            if (requestData.contains("faceSampleStrategy")) {
                newTask["faceSampleStrategy"] = requestData["faceSampleStrategy"];
            }
            if (requestData.contains("facePreviewFps")) {
                newTask["facePreviewFps"] = requestData["facePreviewFps"];
            }
            if (requestData.contains("faceRecognitionInterval")) {
                newTask["faceRecognitionInterval"] = requestData["faceRecognitionInterval"];
            }
            if (requestData.contains("faceLowLatencyMode")) {
                newTask["faceLowLatencyMode"] = requestData["faceLowLatencyMode"];
            }
            if (requestData.contains("trackingDetectionThreshold")) {
                newTask["trackingDetectionThreshold"] = requestData["trackingDetectionThreshold"];
            }
            if (requestData.contains("faceSimilarityThreshold")) {
                newTask["faceSimilarityThreshold"] = requestData["faceSimilarityThreshold"];
            }
            
            tasksJson.push_back(newTask);
            
            // 保存到文件
            std::ofstream fileOut("/data/lintech/celectronicfence/tasks.json");
            fileOut << tasksJson.dump(4);
            fileOut.close();
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Task added successfully";
            responseJson["task"] = newTask;
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 更新任务
    svr.Put("/api/tasks/:id", [](const Request &req, Response &res)
    {
        try {
            int taskId = std::stoi(req.path_params.at("id"));
            json requestData = json::parse(req.body);
            
            // 读取现有任务
            std::ifstream fileIn("/data/lintech/celectronicfence/tasks.json");
            json tasksJson;
            
            if (fileIn.is_open()) {
                fileIn >> tasksJson;
                fileIn.close();
            } else {
                tasksJson = json::array();
            }
            
            // 查找并更新任务
            bool found = false;
            for (auto& task : tasksJson) {
                if (task["id"] == taskId) {
                    std::string oldAlgorithm = task.value("algorithm", "");
                    task["taskNumber"] = requestData["taskNumber"];
                    task["description"] = requestData.value("description", "");
                    task["videoSourceId"] = requestData["videoSourceId"];
                    task["algorithm"] = requestData["algorithm"];
                    std::string newAlgorithm = task.value("algorithm", "");
                    if (isRegisteredPersonTrackingAlgorithm(oldAlgorithm) ||
                        isRegisteredPersonTrackingAlgorithm(newAlgorithm)) {
                        clearRegisteredPersonTrackingTaskState(
                            taskId,
                            isRegisteredPersonTrackingAlgorithm(newAlgorithm));
                    }
                    
                    // 更新远程上报地址
                    if (requestData.contains("remoteUrl")) {
                        task["remoteUrl"] = requestData["remoteUrl"];
                    } else {
                        task.erase("remoteUrl");  // 如果不存在则删除该字段
                    }
                    if (requestData.contains("demoMode")) {
                        task["demoMode"] = requestData["demoMode"];
                    } else {
                        task.erase("demoMode");
                    }
                    
                    // 更新阈值参数（区域入侵/人员检测）
                    if (requestData.contains("threshold")) {
                        task["threshold"] = requestData["threshold"];
                    } else {
                        task.erase("threshold");
                    }
                    if (requestData.contains("thresholdMode")) {
                        task["thresholdMode"] = requestData["thresholdMode"];
                    } else {
                        task.erase("thresholdMode");
                    }
                    
                    // 更新烟火检测的双阈值
                    if (requestData.contains("flameThreshold")) {
                        task["flameThreshold"] = requestData["flameThreshold"];
                    } else {
                        task.erase("flameThreshold");
                    }
                    if (requestData.contains("smokeThreshold")) {
                        task["smokeThreshold"] = requestData["smokeThreshold"];
                    } else {
                        task.erase("smokeThreshold");
                    }
                    if (requestData.contains("fightThreshold")) {
                        task["fightThreshold"] = requestData["fightThreshold"];
                    } else {
                        task.erase("fightThreshold");
                    }
                    if (requestData.contains("fallThreshold")) {
                        task["fallThreshold"] = requestData["fallThreshold"];
                    } else {
                        task.erase("fallThreshold");
                    }
                    if (requestData.contains("fallPreset")) {
                        task["fallPreset"] = requestData["fallPreset"];
                    } else {
                        task.erase("fallPreset");
                    }
                    if (requestData.contains("trackIouThresh")) {
                        task["trackIouThresh"] = requestData["trackIouThresh"];
                    } else {
                        task.erase("trackIouThresh");
                    }
                    if (requestData.contains("maxTrackAge")) {
                        task["maxTrackAge"] = requestData["maxTrackAge"];
                    } else {
                        task.erase("maxTrackAge");
                    }
                    if (requestData.contains("minAreaRatio")) {
                        task["minAreaRatio"] = requestData["minAreaRatio"];
                    } else {
                        task.erase("minAreaRatio");
                    }
                    if (requestData.contains("uprightRatioMax")) {
                        task["uprightRatioMax"] = requestData["uprightRatioMax"];
                    } else {
                        task.erase("uprightRatioMax");
                    }
                    if (requestData.contains("fallRatioMin")) {
                        task["fallRatioMin"] = requestData["fallRatioMin"];
                    } else {
                        task.erase("fallRatioMin");
                    }
                    if (requestData.contains("heightDropRatio")) {
                        task["heightDropRatio"] = requestData["heightDropRatio"];
                    } else {
                        task.erase("heightDropRatio");
                    }
                    if (requestData.contains("staticFallRatioMin")) {
                        task["staticFallRatioMin"] = requestData["staticFallRatioMin"];
                    } else {
                        task.erase("staticFallRatioMin");
                    }
                    if (requestData.contains("staticHeightRatioMax")) {
                        task["staticHeightRatioMax"] = requestData["staticHeightRatioMax"];
                    } else {
                        task.erase("staticHeightRatioMax");
                    }
                    if (requestData.contains("staticBottomRatio")) {
                        task["staticBottomRatio"] = requestData["staticBottomRatio"];
                    } else {
                        task.erase("staticBottomRatio");
                    }
                    if (requestData.contains("uprightFrames")) {
                        task["uprightFrames"] = requestData["uprightFrames"];
                    } else {
                        task.erase("uprightFrames");
                    }
                    if (requestData.contains("fallFrames")) {
                        task["fallFrames"] = requestData["fallFrames"];
                    } else {
                        task.erase("fallFrames");
                    }
                    if (requestData.contains("staticFallFrames")) {
                        task["staticFallFrames"] = requestData["staticFallFrames"];
                    } else {
                        task.erase("staticFallFrames");
                    }
                    if (requestData.contains("historyHoldFrames")) {
                        task["historyHoldFrames"] = requestData["historyHoldFrames"];
                    } else {
                        task.erase("historyHoldFrames");
                    }
                    if (requestData.contains("alarmHoldFrames")) {
                        task["alarmHoldFrames"] = requestData["alarmHoldFrames"];
                    } else {
                        task.erase("alarmHoldFrames");
                    }
                    if (requestData.contains("minBottomRatio")) {
                        task["minBottomRatio"] = requestData["minBottomRatio"];
                    } else {
                        task.erase("minBottomRatio");
                    }
                    if (requestData.contains("minCenterDropRatio")) {
                        task["minCenterDropRatio"] = requestData["minCenterDropRatio"];
                    } else {
                        task.erase("minCenterDropRatio");
                    }
                    if (requestData.contains("faceSampleInterval")) {
                        task["faceSampleInterval"] = requestData["faceSampleInterval"];
                    } else {
                        task.erase("faceSampleInterval");
                    }
                    if (requestData.contains("faceSampleStrategy")) {
                        task["faceSampleStrategy"] = requestData["faceSampleStrategy"];
                    } else {
                        task.erase("faceSampleStrategy");
                    }
                    if (requestData.contains("facePreviewFps")) {
                        task["facePreviewFps"] = requestData["facePreviewFps"];
                    } else {
                        task.erase("facePreviewFps");
                    }
                    if (requestData.contains("faceRecognitionInterval")) {
                        task["faceRecognitionInterval"] = requestData["faceRecognitionInterval"];
                    } else {
                        task.erase("faceRecognitionInterval");
                    }
                    if (requestData.contains("faceLowLatencyMode")) {
                        task["faceLowLatencyMode"] = requestData["faceLowLatencyMode"];
                    } else {
                        task.erase("faceLowLatencyMode");
                    }
                    if (requestData.contains("trackingDetectionThreshold")) {
                        task["trackingDetectionThreshold"] = requestData["trackingDetectionThreshold"];
                    } else {
                        task.erase("trackingDetectionThreshold");
                    }
                    if (requestData.contains("faceSimilarityThreshold")) {
                        task["faceSimilarityThreshold"] = requestData["faceSimilarityThreshold"];
                    } else {
                        task.erase("faceSimilarityThreshold");
                    }
                    
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Task not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }
            
            // 保存到文件
            std::ofstream fileOut("/data/lintech/celectronicfence/tasks.json");
            fileOut << tasksJson.dump(4);
            fileOut.close();
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Task updated successfully";
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 删除任务
    svr.Delete("/api/tasks/:id", [](const Request &req, Response &res)
    {
        try {
            int taskId = std::stoi(req.path_params.at("id"));
            
            // 读取现有任务
            std::ifstream fileIn("/data/lintech/celectronicfence/tasks.json");
            json tasksJson;
            
            if (fileIn.is_open()) {
                fileIn >> tasksJson;
                fileIn.close();
            } else {
                tasksJson = json::array();
            }
            
            // 删除任务
            json newTasksJson = json::array();
            bool found = false;
            for (const auto& task : tasksJson) {
                if (task["id"] == taskId) {
                    // 如果任务正在运行，先停止它
                    if (task.contains("pid") && task["pid"].is_number() && task["pid"] > 0) {
                        stopTaskProcessIfMatched(task, (int)task["pid"], "删除任务");
                    }
                    if (task.value("algorithm", "") == "face_recognition") {
                        stopFaceTaskProcessesByTaskId(
                            taskId, task.value("pid", 0), "删除任务清理残留人脸进程");
                    } else if (isRegisteredPersonTrackingAlgorithm(task.value("algorithm", ""))) {
                        stopRegisteredPersonTrackingProcessesByTaskId(
                            taskId, task.value("pid", 0), "删除任务清理残留登记人员追踪进程");
                        clearRegisteredPersonTrackingTaskState(taskId, false);
                    }
                    found = true;
                } else {
                    newTasksJson.push_back(task);
                }
            }
            
            if (!found) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Task not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }
            
            // 保存到文件
            std::ofstream fileOut("/data/lintech/celectronicfence/tasks.json");
            fileOut << newTasksJson.dump(4);
            fileOut.close();
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Task deleted successfully";
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 启动任务
    svr.Post("/api/tasks/:id/start", [](const Request &req, Response &res)
    {
        try {
            std::lock_guard<std::mutex> taskLock(taskOperationMutex);
            int mtxCheck = std::system("bash -c \"ss -lnt | grep -q ':8554' && ss -lnt | grep -q ':8889'\"");
            if (mtxCheck != 0) {
                std::cout << "[启动任务] 检测到mediamtx未运行，尝试自动启动..." << std::endl;
                std::system("bash -c \"cd /data/lintech && nohup /data/lintech/mediamtx /data/lintech/mediamtx.yml > /data/lintech/mediamtx.log 2>&1 &\"");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            if (!ensureDirectory(TASK_LOG_DIR)) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Task log directory is not writable";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            int taskId = std::stoi(req.path_params.at("id"));
            json requestData = json::object();
            if (!req.body.empty()) {
                json parsed = json::parse(req.body, nullptr, false);
                if (parsed.is_object()) {
                    requestData = parsed;
                }
            }
            bool skipHealthCheck = requestData.value("skipHealthCheck", false);

            std::ifstream tasksFileIn("/data/lintech/celectronicfence/tasks.json");
            json tasksJson;
            if (tasksFileIn.is_open()) {
                tasksFileIn >> tasksJson;
                tasksFileIn.close();
            } else {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Tasks file not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::ifstream channelsFileIn("/data/lintech/celectronicfence/channels.json");
            json channelsData;
            if (channelsFileIn.is_open()) {
                channelsFileIn >> channelsData;
                channelsFileIn.close();
            } else {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Channels file not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            bool found = false;
            for (auto& task : tasksJson) {
                if (task.value("id", 0) != taskId) {
                    continue;
                }

                found = true;
                if (requestData.contains("remoteUrl")) {
                    task["remoteUrl"] = requestData["remoteUrl"];
                }

                std::string algorithm = task.value("algorithm", "");
                int videoSourceId = task.value("videoSourceId", 0);
                std::string channelUrl;
                std::string channelName;
                int channelId = 20;
                bool channelFound = false;

                if (channelsData.contains("channels") && channelsData["channels"].is_array()) {
                    for (const auto& channel : channelsData["channels"]) {
                        if (channel.value("id", 0) == videoSourceId) {
                            channelUrl = channel.value("url", "");
                            channelName = channel.value("name", "");
                            channelId = channel.value("channelAssignment", "1") == "2" ? 10 : 20;
                            channelFound = true;
                            break;
                        }
                    }
                }

                if (!channelFound || channelUrl.empty()) {
                    json errorJson;
                    errorJson["status"] = "error";
                    errorJson["message"] = "Video source URL not found";
                    res.status = 400;
                    res.set_content(errorJson.dump(), "application/json");
                    return;
                }

                task["channelId"] = channelId;

                std::cout << "[启动任务] 任务ID: " << taskId
                          << ", 算法: " << algorithm
                          << ", 视频源ID: " << videoSourceId
                          << ", 通道: " << channelId
                          << ", 名称: " << channelName << std::endl;

                int currentPid = task.value("pid", 0);
                if (currentPid > 0) {
                    stopTaskProcessIfMatched(task, currentPid, "启动前停止当前任务");
                }
                if (algorithm == "face_recognition") {
                    stopFaceTaskProcessesByTaskId(taskId, currentPid, "启动前清理残留人脸进程");
                } else if (isRegisteredPersonTrackingAlgorithm(algorithm)) {
                    stopRegisteredPersonTrackingProcessesByTaskId(
                        taskId, currentPid, "启动前清理残留登记人员追踪进程");
                    clearRegisteredPersonTrackingTaskState(taskId);
                }
                task.erase("configPath");
                task.erase("engineConfigPath");
                task.erase("thresholdConfigPath");
                task.erase("filterConfigPath");
                task.erase("outputRtsp");
                task.erase("streamChannelId");
                task["status"] = "stopped";
                task["pid"] = 0;
                task.erase("pidStartTime");

	                int pid = 0;
	                std::string outputRtsp = buildTaskRtspOutputUrl(taskId);
	                std::string commandOutput;
	                std::string taskLogPath;
	                std::string alarmSourceName = channelName.empty()
	                    ? ("VideoSource-" + std::to_string(videoSourceId))
	                    : channelName;

                if (algorithm == "明烟明火") {
                    double flameThreshold = task.value("flameThreshold", 0.25);
                    double smokeThreshold = task.value("smokeThreshold", 0.5);
                    std::string fireScriptPath = "/data/lintech/samples/fire-smoke/start.sh";
                    taskLogPath = buildTaskLogPath("fire", taskId);
	                    std::string command = "cd /data/lintech/samples/fire-smoke && "
	                                          "export PYTHONPATH=$PYTHONPATH:/opt/sophon/sophon-opencv_1.9.0/opencv-python && "
	                                          "nohup bash " + fireScriptPath +
	                                          " -i " + shellQuote(channelUrl) +
	                                          " -o " + shellQuote(outputRtsp) +
	                                          " --fire_thresh " + std::to_string(flameThreshold) +
	                                          " --smoke_thresh " + std::to_string(smokeThreshold) +
	                                          " --channel_id " + std::to_string(channelId) +
	                                          " --task_id " + std::to_string(taskId) +
	                                          " --video_source_id " + std::to_string(videoSourceId) +
	                                          " --video_source_name " + shellQuote(alarmSourceName) +
	                                          " > " + shellQuote(taskLogPath) + " 2>&1 & echo $!";
                    commandOutput = executeCommand(command.c_str());
                    task["outputRtsp"] = outputRtsp;
                } else if (algorithm == "打架检测") {
                    double fightThreshold = task.value("fightThreshold", 0.45);
                    std::string fightScriptPath = "/data/lintech/samples/fight/start.sh";
                    taskLogPath = buildTaskLogPath("fight", taskId);
	                    std::string command = "cd /data/lintech/samples/fight && "
	                                          "export PYTHONPATH=$PYTHONPATH:/opt/sophon/sophon-opencv_1.9.0/opencv-python && "
	                                          "nohup bash " + fightScriptPath +
	                                          " -i " + shellQuote(channelUrl) +
	                                          " -o " + shellQuote(outputRtsp) +
	                                          " --fight_thresh " + std::to_string(fightThreshold) +
	                                          " --channel_id " + std::to_string(channelId) +
	                                          " --task_id " + std::to_string(taskId) +
	                                          " --video_source_id " + std::to_string(videoSourceId) +
	                                          " --video_source_name " + shellQuote(alarmSourceName) +
	                                          " > " + shellQuote(taskLogPath) + " 2>&1 & echo $!";
                    commandOutput = executeCommand(command.c_str());
                    task["outputRtsp"] = outputRtsp;
                } else if (algorithm == "跌倒检测") {
                    double fallThreshold = task.value("fallThreshold", 0.60);
                    double trackIouThresh = task.value("trackIouThresh", 0.35);
                    int maxTrackAge = task.value("maxTrackAge", 10);
                    double minAreaRatio = task.value("minAreaRatio", 0.012);
                    double uprightRatioMax = task.value("uprightRatioMax", 1.00);
                    double fallRatioMin = task.value("fallRatioMin", 1.00);
                    double heightDropRatio = task.value("heightDropRatio", 0.90);
                    double staticFallRatioMin = task.value("staticFallRatioMin", 1.20);
                    double staticHeightRatioMax = task.value("staticHeightRatioMax", 0.45);
                    double staticBottomRatio = task.value("staticBottomRatio", 0.50);
                    int uprightFrames = task.value("uprightFrames", 1);
                    int fallFrames = task.value("fallFrames", 1);
                    int staticFallFrames = task.value("staticFallFrames", 2);
                    int historyHoldFrames = task.value("historyHoldFrames", 18);
                    int alarmHoldFrames = task.value("alarmHoldFrames", 8);
                    double minBottomRatio = task.value("minBottomRatio", 0.35);
                    double minCenterDropRatio = task.value("minCenterDropRatio", 0.05);
                    std::string fallScriptPath = "/data/lintech/samples/fall/start.sh";
                    taskLogPath = buildTaskLogPath("fall", taskId);
	                    std::string command = "cd /data/lintech/samples/fall && "
	                                          "export PYTHONPATH=$PYTHONPATH:/opt/sophon/sophon-opencv_1.9.0/opencv-python && "
	                                          "nohup bash " + fallScriptPath +
	                                          " -i " + shellQuote(channelUrl) +
	                                          " -o " + shellQuote(outputRtsp) +
                                          " --fall_thresh " + std::to_string(fallThreshold) +
                                          " --track_iou_thresh " + std::to_string(trackIouThresh) +
                                          " --max_track_age " + std::to_string(maxTrackAge) +
                                          " --min_area_ratio " + std::to_string(minAreaRatio) +
                                          " --upright_ratio_max " + std::to_string(uprightRatioMax) +
                                          " --fall_ratio_min " + std::to_string(fallRatioMin) +
                                          " --height_drop_ratio " + std::to_string(heightDropRatio) +
                                          " --static_fall_ratio_min " + std::to_string(staticFallRatioMin) +
                                          " --static_height_ratio_max " + std::to_string(staticHeightRatioMax) +
                                          " --static_bottom_ratio " + std::to_string(staticBottomRatio) +
                                          " --upright_frames " + std::to_string(uprightFrames) +
                                          " --fall_frames " + std::to_string(fallFrames) +
	                                          " --static_fall_frames " + std::to_string(staticFallFrames) +
	                                          " --history_hold_frames " + std::to_string(historyHoldFrames) +
	                                          " --alarm_hold_frames " + std::to_string(alarmHoldFrames) +
	                                          " --min_bottom_ratio " + std::to_string(minBottomRatio) +
	                                          " --min_center_drop_ratio " + std::to_string(minCenterDropRatio) +
	                                          " --channel_id " + std::to_string(channelId) +
	                                          " --task_id " + std::to_string(taskId) +
	                                          " --video_source_id " + std::to_string(videoSourceId) +
	                                          " --video_source_name " + shellQuote(alarmSourceName) +
	                                          " > " + shellQuote(taskLogPath) + " 2>&1 & echo $!";
                    commandOutput = executeCommand(command.c_str());
                    task["outputRtsp"] = outputRtsp;
                } else if (algorithm == "face_recognition") {
                    if (channelUrl.rfind("rtsp://", 0) != 0 && channelUrl.rfind("rtsps://", 0) != 0) {
                        json errorJson;
                        errorJson["status"] = "error";
                        errorJson["message"] = "Face recognition only supports RTSP URL";
                        res.status = 400;
                        res.set_content(errorJson.dump(), "application/json");
                        return;
                    }

                    std::string runtimeFaceConfigPath;
                    std::string runtimeConfigError;
                    int streamChannelId = buildTaskStreamChannelId(taskId);
                    const int kDefaultFaceSampleInterval = 2;
                    const std::string kDefaultFaceSampleStrategy = "DROP";
                    const int kDefaultFacePreviewFps = 12;
                    const int kDefaultFaceRecognitionInterval = 1;
                    const bool kDefaultFaceLowLatencyMode = true;
                    const int kLowLatencyFaceSampleInterval = 3;
                    const int kLowLatencyFaceRecognitionInterval = 1;
                    const int kLowLatencyFacePreviewFps = 10;
                    int faceSampleInterval = clampInt(
                        task.value("faceSampleInterval", kDefaultFaceSampleInterval),
                        1,
                        12
                    );
                    std::string faceSampleStrategy = trimString(
                        task.value("faceSampleStrategy", kDefaultFaceSampleStrategy)
                    );
                    if (faceSampleStrategy != "KEEP" && faceSampleStrategy != "DROP")
                    {
                        faceSampleStrategy = kDefaultFaceSampleStrategy;
                    }
                    int facePreviewFps = clampInt(
                        task.value("facePreviewFps", kDefaultFacePreviewFps),
                        1,
                        25
                    );
                    int faceRecognitionInterval = clampInt(
                        task.value("faceRecognitionInterval", kDefaultFaceRecognitionInterval),
                        1,
                        12
                    );
                    bool faceLowLatencyMode = task.value("faceLowLatencyMode", kDefaultFaceLowLatencyMode);

                    if (requestData.contains("faceSampleInterval")) {
                        faceSampleInterval = clampInt(requestData.value("faceSampleInterval", faceSampleInterval), 1, 12);
                    }
                    if (requestData.contains("faceSampleStrategy")) {
                        std::string strategyFromRequest = trimString(requestData.value("faceSampleStrategy", faceSampleStrategy));
                        if (strategyFromRequest == "KEEP" || strategyFromRequest == "DROP") {
                            faceSampleStrategy = strategyFromRequest;
                        }
                    }
                    if (requestData.contains("facePreviewFps")) {
                        facePreviewFps = clampInt(requestData.value("facePreviewFps", facePreviewFps), 1, 25);
                    }
                    if (requestData.contains("faceRecognitionInterval")) {
                        faceRecognitionInterval = clampInt(
                            requestData.value("faceRecognitionInterval", faceRecognitionInterval), 1, 12);
                    }
                    if (requestData.contains("faceLowLatencyMode")) {
                        faceLowLatencyMode = requestData.value("faceLowLatencyMode", faceLowLatencyMode);
                    }

                    if (faceLowLatencyMode) {
                        faceSampleStrategy = "DROP";
                        faceSampleInterval = std::max(faceSampleInterval, kLowLatencyFaceSampleInterval);
                        faceRecognitionInterval = kLowLatencyFaceRecognitionInterval;
                        facePreviewFps = std::min(facePreviewFps, kLowLatencyFacePreviewFps);
                    }
                    std::cout << "[启动任务][人脸] 参数: sample_interval=" << faceSampleInterval
                              << ", sample_strategy=" << faceSampleStrategy
                              << ", preview_fps=" << facePreviewFps
                              << ", recognition_interval=" << faceRecognitionInterval
                              << ", low_latency_mode=" << (faceLowLatencyMode ? "true" : "false")
                              << std::endl;

                    if (!prepareFaceRuntimeConfigs(
                            taskId,
                            streamChannelId,
                            channelUrl,
                            faceSampleInterval,
                            faceSampleStrategy,
                            facePreviewFps,
                            faceRecognitionInterval,
                            runtimeFaceConfigPath,
                            runtimeConfigError)) {
                        json errorJson;
                        errorJson["status"] = "error";
                        errorJson["message"] = runtimeConfigError.empty() ?
                            "Failed to prepare face runtime config" :
                            runtimeConfigError;
                        res.status = 500;
                        res.set_content(errorJson.dump(), "application/json");
                        return;
                    }

                    task["streamChannelId"] = streamChannelId;
                    task["outputRtsp"] = "rtsp://localhost:8554/live/0_" + std::to_string(streamChannelId) + "/";
                    task["configPath"] = runtimeFaceConfigPath;
                    task["faceSampleInterval"] = faceSampleInterval;
                    task["faceSampleStrategy"] = faceSampleStrategy;
                    task["facePreviewFps"] = facePreviewFps;
                    task["faceRecognitionInterval"] = faceRecognitionInterval;
                    task["faceLowLatencyMode"] = faceLowLatencyMode;
                    taskLogPath = buildTaskLogPath("face", taskId);
                    std::string command =
                        "export PYTHONPATH=$PYTHONPATH:/opt/sophon/sophon-opencv_1.9.0/opencv-python && "
                        "export SOPHON_STREAM_LOG_LEVEL=warn && "
                        "export PATH=$PATH:/opt/bin:/bm_bin && "
                        "export CEF_CHANNEL_ID=" + std::to_string(channelId) + " && "
                        "export QTDIR=/usr/lib/aarch64-linux-gnu && "
                        "export QT_QPA_PLATFORM_PLUGIN_PATH=$QTDIR/qt5/plugins/ && "
                        "export QT_QPA_FONTDIR=$QTDIR/fonts && "
                        "export LD_LIBRARY_PATH=" + shellQuote(FACE_BUILD_LIB_DIR) + ":$LD_LIBRARY_PATH && "
                        "export NO_FRAMEBUFFER=1 && "
                        "cd " + shellQuote(FACE_SAMPLES_BUILD_DIR) + " && "
                        "nohup ./main --demo_config_path=" + shellQuote(runtimeFaceConfigPath) +
                        " > " + shellQuote(taskLogPath) + " 2>&1 & echo $!";
                    commandOutput = executeCommand(command.c_str());
                } else if (isRegisteredPersonTrackingAlgorithm(algorithm)) {
                    std::string runtimeDemoConfigPath;
                    std::string runtimeFilterConfigPath;
                    std::string runtimeConfigError;
                    int streamChannelId = buildTaskStreamChannelId(taskId);
                    json regionConfigs =
                        task.contains("regionConfigs") && task["regionConfigs"].is_array()
                            ? task["regionConfigs"]
                            : json::array();
                    const int kDefaultTrackFaceSampleInterval = 1;
                    const std::string kDefaultTrackFaceSampleStrategy = "DROP";
                    const int kDefaultTrackFacePreviewFps = 25;
                    const int kDefaultTrackFaceRecognitionInterval = 1;
                    const bool kDefaultTrackFaceLowLatencyMode = true;
                    const int kLowLatencyTrackFaceSampleInterval = 1;
                    const int kLowLatencyTrackFaceRecognitionInterval = 1;
                    const int kLowLatencyTrackFacePreviewFps = 25;
                    int trackFaceSampleInterval = clampInt(
                        task.value("faceSampleInterval", kDefaultTrackFaceSampleInterval), 1, 12);
                    std::string trackFaceSampleStrategy = trimString(
                        task.value("faceSampleStrategy", kDefaultTrackFaceSampleStrategy));
                    if (trackFaceSampleStrategy != "KEEP" && trackFaceSampleStrategy != "DROP")
                    {
                        trackFaceSampleStrategy = kDefaultTrackFaceSampleStrategy;
                    }
                    int trackFacePreviewFps = clampInt(
                        task.value("facePreviewFps", kDefaultTrackFacePreviewFps), 1, 60);
                    int trackFaceRecognitionInterval = clampInt(
                        task.value("faceRecognitionInterval", kDefaultTrackFaceRecognitionInterval), 1, 12);
                    bool trackFaceLowLatencyMode =
                        task.value("faceLowLatencyMode", kDefaultTrackFaceLowLatencyMode);

                    if (requestData.contains("faceSampleInterval")) {
                        trackFaceSampleInterval = clampInt(
                            requestData.value("faceSampleInterval", trackFaceSampleInterval), 1, 12);
                    }
                    if (requestData.contains("faceSampleStrategy")) {
                        std::string strategyFromRequest = trimString(
                            requestData.value("faceSampleStrategy", trackFaceSampleStrategy));
                        if (strategyFromRequest == "KEEP" || strategyFromRequest == "DROP") {
                            trackFaceSampleStrategy = strategyFromRequest;
                        }
                    }
                    if (requestData.contains("facePreviewFps")) {
                        trackFacePreviewFps = clampInt(
                            requestData.value("facePreviewFps", trackFacePreviewFps), 1, 60);
                    }
                    if (requestData.contains("faceRecognitionInterval")) {
                        trackFaceRecognitionInterval = clampInt(
                            requestData.value("faceRecognitionInterval", trackFaceRecognitionInterval), 1, 12);
                    }
                    if (requestData.contains("faceLowLatencyMode")) {
                        trackFaceLowLatencyMode =
                            requestData.value("faceLowLatencyMode", trackFaceLowLatencyMode);
                    }

                    if (trackFaceLowLatencyMode) {
                        trackFaceSampleStrategy = "DROP";
                        trackFaceSampleInterval = kLowLatencyTrackFaceSampleInterval;
                        trackFaceRecognitionInterval = kLowLatencyTrackFaceRecognitionInterval;
                        trackFacePreviewFps =
                            std::max(trackFacePreviewFps, kLowLatencyTrackFacePreviewFps);
                    }
                    double trackingDetectionThreshold = clampDouble(
                        task.value(
                            "trackingDetectionThreshold",
                            trackFaceLowLatencyMode ? 0.20 : 0.25
                        ),
                        0.05,
                        0.95
                    );
                    double faceSimilarityThreshold = clampDouble(
                        task.value(
                            "faceSimilarityThreshold",
                            REGISTERED_PERSON_TRACKING_DEFAULT_FACE_SIMILARITY_THRESHOLD
                        ),
                        0.05,
                        0.95
                    );
                    if (requestData.contains("trackingDetectionThreshold")) {
                        trackingDetectionThreshold = clampDouble(
                            requestData.value("trackingDetectionThreshold", trackingDetectionThreshold),
                            0.05,
                            0.95
                        );
                    }
                    if (requestData.contains("faceSimilarityThreshold")) {
                        faceSimilarityThreshold = clampDouble(
                            requestData.value("faceSimilarityThreshold", faceSimilarityThreshold),
                            0.05,
                            0.95
                        );
                    }

                    std::cout << "[启动任务][登记人员追踪] 参数: sample_interval=" << trackFaceSampleInterval
                              << ", sample_strategy=" << trackFaceSampleStrategy
                              << ", preview_fps=" << trackFacePreviewFps
                              << ", recognition_interval=" << trackFaceRecognitionInterval
                              << ", low_latency_mode=" << (trackFaceLowLatencyMode ? "true" : "false")
                              << ", tracking_detection_threshold=" << trackingDetectionThreshold
                              << ", face_similarity_threshold=" << faceSimilarityThreshold
                              << std::endl;

                    if (!prepareRegisteredPersonTrackingRuntimeConfigs(
                            taskId,
                            streamChannelId,
                            channelUrl,
                            trackFaceSampleInterval,
                            trackFaceSampleStrategy,
                            trackFacePreviewFps,
                            trackFaceRecognitionInterval,
                            trackFaceLowLatencyMode,
                            trackingDetectionThreshold,
                            faceSimilarityThreshold,
                            regionConfigs,
                            runtimeDemoConfigPath,
                            runtimeFilterConfigPath,
                            runtimeConfigError)) {
                        json errorJson;
                        errorJson["status"] = "error";
                        errorJson["message"] = runtimeConfigError.empty() ?
                            "Failed to prepare registered person tracking runtime config" :
                            runtimeConfigError;
                        res.status = 500;
                        res.set_content(errorJson.dump(), "application/json");
                        return;
                    }

                    task["streamChannelId"] = streamChannelId;
                    task["outputRtsp"] = "rtsp://localhost:8554/live/0_" + std::to_string(streamChannelId) + "/";
                    task["configPath"] = runtimeDemoConfigPath;
                    task["filterConfigPath"] = runtimeFilterConfigPath;
                    task["faceSampleInterval"] = trackFaceSampleInterval;
                    task["faceSampleStrategy"] = trackFaceSampleStrategy;
                    task["facePreviewFps"] = trackFacePreviewFps;
                    task["faceRecognitionInterval"] = trackFaceRecognitionInterval;
                    task["faceLowLatencyMode"] = trackFaceLowLatencyMode;
                    task["trackingDetectionThreshold"] = trackingDetectionThreshold;
                    task["faceSimilarityThreshold"] = faceSimilarityThreshold;
                    taskLogPath = buildTaskLogPath("registered_person_tracking", taskId);
                    std::string dropOnFullExport;
                    if (trackFaceLowLatencyMode) {
                        dropOnFullExport = "export SOPHON_STREAM_DROP_ON_FULL=1 && ";
                    }
                    std::string command =
                        std::string("export PYTHONPATH=$PYTHONPATH:/opt/sophon/sophon-opencv_1.9.0/opencv-python && ") +
                        "export SOPHON_STREAM_LOG_LEVEL=warn && " +
                        "export PATH=$PATH:/opt/bin:/bm_bin && " +
                        "export CEF_CHANNEL_ID=" + std::to_string(channelId) + " && " +
                        "export QTDIR=/usr/lib/aarch64-linux-gnu && " +
                        "export QT_QPA_PLATFORM_PLUGIN_PATH=$QTDIR/qt5/plugins/ && " +
                        "export QT_QPA_FONTDIR=$QTDIR/fonts && " +
                        "export LD_LIBRARY_PATH=" + shellQuote(SOPHON_STREAM_BUILD_LIB_DIR) + ":$LD_LIBRARY_PATH && " +
                        "export NO_FRAMEBUFFER=1 && " +
                        dropOnFullExport +
                        "cd " + shellQuote(SOPHON_STREAM_SAMPLES_BUILD_DIR) + " && " +
                        "nohup ./main --demo_config_path=" + shellQuote(runtimeDemoConfigPath) +
                        " > " + shellQuote(taskLogPath) + " 2>&1 & echo $!";
                    commandOutput = executeCommand(command.c_str());
                } else {
                    const std::string thresholdConfigPath =
                        SOPHON_STREAM_YOLO_CONFIG_DIR + "/yolov8_classthresh_roi_example.json";
                    std::ifstream thresholdConfigIn(thresholdConfigPath.c_str());
                    if (!thresholdConfigIn.is_open()) {
                        json errorJson;
                        errorJson["status"] = "error";
                        errorJson["message"] = "Yolov8 threshold config not found";
                        res.status = 500;
                        res.set_content(errorJson.dump(), "application/json");
                        return;
                    }

                    json thresholdConfigJson;
                    thresholdConfigIn >> thresholdConfigJson;
                    thresholdConfigIn.close();

                    std::set<std::string> classNames =
                        JsonFile2::loadClassNameSetFromThresholdConfig(thresholdConfigJson);
                    if (classNames.empty()) {
                        json errorJson;
                        errorJson["status"] = "error";
                        errorJson["message"] = "Invalid class_names_file for Yolov8";
                        res.status = 500;
                        res.set_content(errorJson.dump(), "application/json");
                        return;
                    }

                    int removedInvalidCount = 0;
                    bool cleaned =
                        JsonFile2::sanitizeThresholdConf(thresholdConfigJson, classNames, &removedInvalidCount);
                    if (cleaned || removedInvalidCount > 0) {
                        std::ofstream thresholdConfigOut(thresholdConfigPath.c_str());
                        thresholdConfigOut << thresholdConfigJson.dump(4);
                        thresholdConfigOut.close();
                        std::cout << "[启动任务] 已清理Yolov8阈值配置非法键数量: " << removedInvalidCount << std::endl;
                    }

                    int streamChannelId = buildTaskStreamChannelId(taskId);
                    std::string runtimeDemoConfigPath;
                    std::string runtimeThresholdConfigPath;
                    std::string runtimeFilterConfigPath;
                    std::string runtimeOsdConfigPath;
                    std::string runtimeConfigError;
                    json regionConfigs =
                        task.contains("regionConfigs") && task["regionConfigs"].is_array()
                            ? task["regionConfigs"]
                            : json::array();
                    if (!prepareYolov8RuntimeDemoConfig(
                            taskId,
                            algorithm,
                            streamChannelId,
                            channelUrl,
                            regionConfigs,
                            runtimeDemoConfigPath,
                            runtimeThresholdConfigPath,
                            runtimeFilterConfigPath,
                            runtimeOsdConfigPath,
                            runtimeConfigError)) {
                        json errorJson;
                        errorJson["status"] = "error";
                        errorJson["message"] = runtimeConfigError.empty() ?
                            "Failed to prepare yolov8 runtime config" :
                            runtimeConfigError;
                        res.status = 500;
                        res.set_content(errorJson.dump(), "application/json");
                        return;
                    }

                    task["streamChannelId"] = streamChannelId;
                    task["outputRtsp"] = "rtsp://localhost:8554/live/0_" + std::to_string(streamChannelId) + "/";
                    task["configPath"] = runtimeDemoConfigPath;
                    task["thresholdConfigPath"] = runtimeThresholdConfigPath;
                    if (!runtimeFilterConfigPath.empty()) {
                        task["filterConfigPath"] = runtimeFilterConfigPath;
                    } else {
                        task.erase("filterConfigPath");
                    }
                    if (!runtimeOsdConfigPath.empty()) {
                        task["osdConfigPath"] = runtimeOsdConfigPath;
                    } else {
                        task.erase("osdConfigPath");
                    }
                    taskLogPath = buildTaskLogPath("yolo", taskId);
                    std::string command =
                        "export PYTHONPATH=$PYTHONPATH:/opt/sophon/sophon-opencv_1.9.0/opencv-python && "
                        "export SOPHON_STREAM_LOG_LEVEL=warn && "
                        "export PATH=$PATH:/opt/bin:/bm_bin && "
                        "export CEF_CHANNEL_ID=" + std::to_string(channelId) + " && "
                        "export QTDIR=/usr/lib/aarch64-linux-gnu && "
                        "export QT_QPA_PLATFORM_PLUGIN_PATH=$QTDIR/qt5/plugins/ && "
                        "export QT_QPA_FONTDIR=$QTDIR/fonts && "
                        "export LD_LIBRARY_PATH=" + shellQuote(SOPHON_STREAM_BUILD_LIB_DIR) + ":$LD_LIBRARY_PATH && "
                        "export NO_FRAMEBUFFER=1 && "
                        "cd " + shellQuote(SOPHON_STREAM_SAMPLES_BUILD_DIR) + " && "
                        "nohup ./main --demo_config_path=" + shellQuote(runtimeDemoConfigPath) +
                        " > " + shellQuote(taskLogPath) + " 2>&1 & echo $!";
                    commandOutput = executeCommand(command.c_str());
                }

                std::string pidText = trimString(commandOutput);
                if (!pidText.empty()) {
                    try {
                        pid = std::stoi(pidText);
                    } catch (...) {
                        pid = 0;
                    }
                }

                if (pid <= 0) {
                    json errorJson;
                    errorJson["status"] = "error";
                    errorJson["message"] = "Task process did not start";
                    res.status = 500;
                    res.set_content(errorJson.dump(), "application/json");
                    return;
                }

                if (!skipHealthCheck) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(800));
                    if (!isProcessRunning(pid)) {
                        json errorJson;
                        errorJson["status"] = "error";
                        errorJson["message"] = "Task process exited immediately after start";
                        res.status = 500;
                        res.set_content(errorJson.dump(), "application/json");
                        return;
                    }
                } else {
                    std::cout << "[启动任务] 快速模式已启用，跳过启动健康检查等待" << std::endl;
                }

                task["status"] = "running";
                task["pid"] = pid;
                if (isRegisteredPersonTrackingAlgorithm(algorithm)) {
                    clearRegisteredPersonTrackingTaskState(taskId);
                    bindRegisteredPersonTrackingStreamTask(task.value("streamChannelId", 0), taskId);
                }
                long long pidStartTime = 0;
                if (readProcessStartTimeTicks(pid, pidStartTime) && pidStartTime > 0) {
                    task["pidStartTime"] = pidStartTime;
                } else {
                    task.erase("pidStartTime");
                }

                std::ofstream tasksFileOut("/data/lintech/celectronicfence/tasks.json");
                tasksFileOut << tasksJson.dump(4);
                tasksFileOut.close();

                std::cout << "[启动任务] 启动成功，PID: " << pid << std::endl;
                break;
            }

            if (!found) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Task not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Task started successfully";
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });

    // 停止任务
    svr.Post("/api/tasks/:id/stop", [](const Request &req, Response &res)
    {
        try {
            std::lock_guard<std::mutex> taskLock(taskOperationMutex);
            int taskId = std::stoi(req.path_params.at("id"));
            std::ifstream tasksFileIn("/data/lintech/celectronicfence/tasks.json");
            json tasksJson;
            if (tasksFileIn.is_open()) {
                tasksFileIn >> tasksJson;
                tasksFileIn.close();
            } else {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Tasks file not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            bool found = false;
            for (auto& task : tasksJson) {
                if (task.value("id", 0) != taskId) {
                    continue;
                }

                found = true;
                std::cout << "停止任务 " << taskId << ", 算法: " << task.value("algorithm", "") << std::endl;
                int pid = task.value("pid", 0);
                if (pid > 0) {
                    stopTaskProcessIfMatched(task, pid, "停止任务");
                }
                if (task.value("algorithm", "") == "face_recognition") {
                    stopFaceTaskProcessesByTaskId(taskId, pid, "停止任务清理残留人脸进程");
                } else if (isRegisteredPersonTrackingAlgorithm(task.value("algorithm", ""))) {
                    stopRegisteredPersonTrackingProcessesByTaskId(
                        taskId, pid, "停止任务清理残留登记人员追踪进程");
                    clearRegisteredPersonTrackingTaskState(taskId);
                }
                task["status"] = "stopped";
                task["pid"] = 0;
                task.erase("pidStartTime");

                std::ofstream tasksFileOut("/data/lintech/celectronicfence/tasks.json");
                tasksFileOut << tasksJson.dump(4);
                tasksFileOut.close();

                {
                    std::lock_guard<std::mutex> lock(stopTimeMutex);
                    lastStopTime = std::chrono::system_clock::now();
                }

                std::cout << "任务停止成功" << std::endl;
                break;
            }

            if (!found) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Task not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Task stopped successfully";
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    // 网络配置API
    
    // 获取网络配置
    svr.Get("/api/network/config", [](const Request &req, Response &res)
    {
        try {
            std::ifstream file("/etc/netplan/01-netcfg.yaml");
            std::string content;
            std::string line;
            
            if (file.is_open()) {
                while (std::getline(file, line)) {
                    content += line + "\n";
                }
                file.close();
            } else {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "无法读取网络配置文件";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }
            
            // 解析YAML配置（简单解析）
            json config;
            json eth0Config, eth1Config;
            
            std::istringstream iss(content);
            std::string currentInterface = "";
            
            while (std::getline(iss, line)) {
                // 检测接口名称
                if (line.find("eth0:") != std::string::npos) {
                    currentInterface = "eth0";
                } else if (line.find("eth1:") != std::string::npos) {
                    currentInterface = "eth1";
                }
                
                if (currentInterface.empty()) continue;
                
                json* currentConfig = (currentInterface == "eth0") ? &eth0Config : &eth1Config;
                
                // 解析dhcp4
                if (line.find("dhcp4:") != std::string::npos) {
                    if (line.find("yes") != std::string::npos) {
                        (*currentConfig)["dhcp4"] = "yes";
                    } else {
                        (*currentConfig)["dhcp4"] = "no";
                    }
                }
                
                // 解析addresses
                if (line.find("addresses:") != std::string::npos) {
                    (*currentConfig)["addresses"] = json::array();
                    // 读取下一行的地址
                    if (std::getline(iss, line)) {
                        size_t bracketStart = line.find("[");
                        size_t bracketEnd = line.find("]");
                        if (bracketStart != std::string::npos && bracketEnd != std::string::npos) {
                            std::string addressesStr = line.substr(bracketStart + 1, bracketEnd - bracketStart - 1);
                            // 移除引号和空格
                            addressesStr.erase(std::remove(addressesStr.begin(), addressesStr.end(), '"'), addressesStr.end());
                            addressesStr.erase(std::remove(addressesStr.begin(), addressesStr.end(), ' '), addressesStr.end());
                            if (!addressesStr.empty()) {
                                (*currentConfig)["addresses"].push_back(addressesStr);
                            }
                        }
                    }
                }
                
                // 解析gateway4
                if (line.find("gateway4:") != std::string::npos) {
                    size_t colonPos = line.find(":");
                    if (colonPos != std::string::npos) {
                        std::string gateway = line.substr(colonPos + 1);
                        gateway.erase(0, gateway.find_first_not_of(" \t"));
                        gateway.erase(gateway.find_last_not_of(" \t\r\n") + 1);
                        (*currentConfig)["gateway4"] = gateway;
                    }
                }
                
                // 解析nameservers
                if (line.find("nameservers:") != std::string::npos) {
                    (*currentConfig)["nameservers"] = json::object();
                    // 读取addresses行
                    if (std::getline(iss, line) && line.find("addresses:") != std::string::npos) {
                        if (std::getline(iss, line)) {
                            size_t bracketStart = line.find("[");
                            size_t bracketEnd = line.find("]");
                            if (bracketStart != std::string::npos && bracketEnd != std::string::npos) {
                                std::string dnsStr = line.substr(bracketStart + 1, bracketEnd - bracketStart - 1);
                                json dnsArray = json::array();
                                
                                // 分割DNS服务器
                                size_t pos = 0;
                                while ((pos = dnsStr.find(",")) != std::string::npos) {
                                    std::string dns = dnsStr.substr(0, pos);
                                    dns.erase(std::remove(dns.begin(), dns.end(), ' '), dns.end());
                                    if (!dns.empty()) {
                                        dnsArray.push_back(dns);
                                    }
                                    dnsStr.erase(0, pos + 1);
                                }
                                dnsStr.erase(std::remove(dnsStr.begin(), dnsStr.end(), ' '), dnsStr.end());
                                if (!dnsStr.empty()) {
                                    dnsArray.push_back(dnsStr);
                                }
                                
                                (*currentConfig)["nameservers"]["addresses"] = dnsArray;
                            }
                        }
                    }
                }
            }
            
            config["eth0"] = eth0Config;
            config["eth1"] = eth1Config;
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["config"] = config;
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 保存网络配置
    svr.Post("/api/network/config", [](const Request &req, Response &res)
    {
        try {
            json requestData = json::parse(req.body);
            json config = requestData["config"];
            
            // 生成netplan配置文件内容
            std::ostringstream yamlContent;
            yamlContent << "network:\n";
            yamlContent << "        version: 2\n";
            yamlContent << "        renderer: networkd\n";
            yamlContent << "        ethernets:\n";
            
            // eth0配置
            if (config.contains("eth0")) {
                json eth0 = config["eth0"];
                yamlContent << "                eth0:\n";
                
                if (eth0.contains("dhcp4")) {
                    yamlContent << "                        dhcp4: " << eth0["dhcp4"].get<std::string>() << "\n";
                }
                
                if (eth0.value("dhcp4", "no") == "no") {
                    if (eth0.contains("addresses") && eth0["addresses"].is_array() && !eth0["addresses"].empty()) {
                        yamlContent << "                        addresses: [" << eth0["addresses"][0].get<std::string>() << "]\n";
                    }
                    
                    yamlContent << "                        optional: yes\n";
                    
                    if (eth0.contains("gateway4")) {
                        yamlContent << "                        gateway4: " << eth0["gateway4"].get<std::string>() << "\n";
                    }
                    
                    if (eth0.contains("nameservers") && eth0["nameservers"].contains("addresses")) {
                        yamlContent << "                        nameservers:\n";
                        yamlContent << "                                addresses: [";
                        json dnsServers = eth0["nameservers"]["addresses"];
                        for (size_t i = 0; i < dnsServers.size(); ++i) {
                            if (i > 0) yamlContent << ", ";
                            yamlContent << dnsServers[i].get<std::string>();
                        }
                        yamlContent << "]\n";
                    }
                }
            }
            
            // eth1配置
            if (config.contains("eth1")) {
                json eth1 = config["eth1"];
                yamlContent << "                eth1:\n";
                
                if (eth1.contains("dhcp4")) {
                    yamlContent << "                        dhcp4: " << eth1["dhcp4"].get<std::string>() << "\n";
                }
                
                if (eth1.value("dhcp4", "no") == "no") {
                    if (eth1.contains("addresses") && eth1["addresses"].is_array() && !eth1["addresses"].empty()) {
                        yamlContent << "                        addresses: [" << eth1["addresses"][0].get<std::string>() << "]\n";
                    }
                    
                    yamlContent << "                        optional: yes\n";
                    
                    if (eth1.contains("gateway4")) {
                        yamlContent << "                        gateway4: " << eth1["gateway4"].get<std::string>() << "\n";
                    }
                    
                    if (eth1.contains("nameservers") && eth1["nameservers"].contains("addresses")) {
                        yamlContent << "                        nameservers:\n";
                        yamlContent << "                                addresses: [";
                        json dnsServers = eth1["nameservers"]["addresses"];
                        for (size_t i = 0; i < dnsServers.size(); ++i) {
                            if (i > 0) yamlContent << ", ";
                            yamlContent << dnsServers[i].get<std::string>();
                        }
                        yamlContent << "]\n";
                    }
                }
            }
            
            std::string yamlStr = yamlContent.str();
            std::cout << "生成的netplan配置:\n" << yamlStr << std::endl;
            
            // 先写入临时文件
            std::ofstream tempFile("/tmp/01-netcfg.yaml.tmp");
            if (!tempFile.is_open()) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "无法创建临时配置文件";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }
            tempFile << yamlStr;
            tempFile.close();
            
            // 使用sudo复制到目标位置并应用
            std::string copyCmd = "sudo cp /tmp/01-netcfg.yaml.tmp /etc/netplan/01-netcfg.yaml";
            std::string applyCmd = "sudo netplan apply";
            
            int copyResult = std::system(copyCmd.c_str());
            if (copyResult != 0) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "无法保存网络配置文件";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }
            
            // 应用配置
            std::system(applyCmd.c_str());
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "网络配置已保存并应用";
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 检测网络状态
    svr.Get("/api/network/status", [](const Request &req, Response &res)
    {
        try {
            auto readTrimmedCommand = [](const std::string& command) -> std::string {
                std::string output = executeCommand(command.c_str());
                output.erase(std::remove(output.begin(), output.end(), '\n'), output.end());
                output.erase(std::remove(output.begin(), output.end(), '\r'), output.end());
                return trimString(output);
            };

            auto buildInterfaceStatus = [&](const std::string& iface) -> json {
                json ifaceJson = json::object();

                std::string carrier = readTrimmedCommand("cat /sys/class/net/" + iface + "/carrier 2>/dev/null");
                ifaceJson["connected"] = (carrier == "1");

                std::string macAddress = readTrimmedCommand("cat /sys/class/net/" + iface + "/address 2>/dev/null");
                ifaceJson["macAddress"] = macAddress;

                std::string addressCidr = readTrimmedCommand(
                    "ip -4 -o addr show dev " + iface + " 2>/dev/null | awk '{print $4}' | head -n 1"
                );
                ifaceJson["addressCidr"] = addressCidr;

                std::string gateway = readTrimmedCommand(
                    "ip route show default dev " + iface + " 2>/dev/null | awk '{print $3}' | head -n 1"
                );
                ifaceJson["gateway4"] = gateway;
                ifaceJson["type"] = "ether";

                return ifaceJson;
            };

            json interfaces;
            interfaces["eth0"] = buildInterfaceStatus("eth0");
            interfaces["eth1"] = buildInterfaceStatus("eth1");
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["interfaces"] = interfaces;
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
//----------------------------------------------------------------------------------------------------------------------------
// 告警管理API

    // 获取所有告警
    svr.Get("/api/alarms", [](const Request &req, Response &res)
    {
        try {
            std::lock_guard<std::mutex> alarmLock(alarmFileMutex);
            std::ifstream file("/data/lintech/celectronicfence/alarms.json");
            json alarmsJson;
            
            if (file.is_open()) {
                file >> alarmsJson;
                file.close();
            } else {
                alarmsJson = json::array();
            }
            bool alarmFileUpdated = false;
            
            // 确保每个告警都有reportStatus字段（兼容旧数据）
            // 同时将reportStatus映射为前端期望的status值
            for (auto& alarm : alarmsJson) {
                std::string originalImageUrl = alarm.value("imageUrl", "");
                std::string normalizedImageUrl = normalizeAlarmImageUrl(originalImageUrl);
                if (normalizedImageUrl != originalImageUrl) {
                    alarm["imageUrl"] = normalizedImageUrl;
                    alarmFileUpdated = true;
                }

                std::string originalVideoUrl = alarm.value("videoUrl", "");
                std::string normalizedVideoUrl = normalizeAlarmVideoUrl(originalVideoUrl);
                if (normalizedVideoUrl != originalVideoUrl) {
                    alarm["videoUrl"] = normalizedVideoUrl;
                    alarmFileUpdated = true;
                }
                if (!normalizedVideoUrl.empty()) {
                    std::string localVideoUrl;
                    std::string localVideoPath;
                    if (buildLocalAlarmVideoPath(normalizedVideoUrl, localVideoUrl, localVideoPath) &&
                        !localVideoPath.empty() && !isAlarmVideoAvailable(normalizedVideoUrl)) {
                        alarm["videoStatus"] = "failed";
                        if (trimString(alarm.value("videoError", "")).empty()) {
                            alarm["videoError"] = "告警视频文件不存在或已失效";
                        }
                        alarmFileUpdated = true;
                    }
                }

                std::string currentImageUrl = alarm.value("imageUrl", "");
                std::string localImageUrl;
                std::string localImagePath;
                bool localImageRef = buildLocalAlarmImagePath(currentImageUrl, localImageUrl, localImagePath);
                if ((currentImageUrl.empty() || (localImageRef && !localImagePath.empty() &&
                                                 !isAlarmImageAvailable(currentImageUrl))) &&
                    alarm.value("taskId", 0) > 0) {
                    std::string generatedImageUrl = tryGenerateAlarmImageFromTask(
                        alarm.value("taskId", 0),
                        alarm.value("alarmType", "")
                    );
                    if (!generatedImageUrl.empty()) {
                        alarm["imageUrl"] = generatedImageUrl;
                        alarmFileUpdated = true;
                    }
                }

                if (!alarm.contains("reportStatus")) {
                    // 如果没有reportStatus字段，根据reportUrl判断
                    std::string reportUrl = alarm.value("reportUrl", "");
                    if (reportUrl.empty()) {
                        alarm["reportStatus"] = "未上报";
                    } else {
                        alarm["reportStatus"] = "未知状态"; // 旧数据，状态未知
                    }
                    alarmFileUpdated = true;
                }
                
                // 将reportStatus映射为前端期望的status值（用于显示上报状态）
                // 前端getStatusBadge函数期望: 'reported', 'failed', 'pending'
                std::string reportStatus = alarm.value("reportStatus", "");
                std::string statusForDisplay = "pending"; // 默认值
                
                if (reportStatus == "上报成功") {
                    statusForDisplay = "reported";
                } else if (reportStatus == "上报失败") {
                    statusForDisplay = "failed";
                } else if (reportStatus == "上报中") {
                    statusForDisplay = "pending";
                } else if (reportStatus == "未上报") {
                    statusForDisplay = "pending";
                } else {
                    // "未知状态"或其他值，保持原status或设为pending
                    statusForDisplay = "pending";
                }
                
                // 如果原status字段是告警处理状态（如"未处理"），保留它
                // 但为了兼容前端显示上报状态，我们需要一个临时字段
                // 由于前端使用alarm.status来显示上报状态，我们需要将上报状态映射到status
                // 但这样会覆盖告警处理状态，所以我们需要检查
                std::string originalStatus = trimString(alarm.value("status", ""));
                std::string processStatus = trimString(alarm.value("processStatus", ""));
                if (processStatus.empty() &&
                    (originalStatus == "未处理" || originalStatus == "已处理")) {
                    processStatus = originalStatus;
                }

                if (!processStatus.empty()) {
                    alarm["status"] = statusForDisplay;
                    alarm["processStatus"] = processStatus;
                } else {
                    // 如果status已经是上报状态格式，直接使用
                    alarm["status"] = statusForDisplay;
                }
            }

            if (alarmFileUpdated) {
                std::ofstream fileOut("/data/lintech/celectronicfence/alarms.json");
                if (fileOut.is_open()) {
                    fileOut << alarmsJson.dump(4);
                    fileOut.close();
                }
            }
            
            // 按时间戳倒序排序（新的在前，旧的在后）
            std::sort(alarmsJson.begin(), alarmsJson.end(), [](const json& a, const json& b) {
                std::string timestampA = a.value("timestamp", "");
                std::string timestampB = b.value("timestamp", "");
                
                // 如果时间戳格式为 "YYYY-MM-DD HH:MM:SS"，可以直接字符串比较
                // 因为这种格式的字符串排序就是时间顺序
                if (timestampA.empty() && timestampB.empty()) {
                    return false;
                }
                if (timestampA.empty()) {
                    return false; // 空时间戳排在后面
                }
                if (timestampB.empty()) {
                    return true; // 空时间戳排在后面
                }
                
                // 倒序：timestampA > timestampB 时返回true，使新的排在前面
                return timestampA > timestampB;
            });
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["alarms"] = alarmsJson;
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 添加新告警
    svr.Post("/api/alarms", [](const Request &req, Response &res)
    {
        try {
            json requestData = json::parse(req.body);
            
            json newAlarm;
            bool shouldAutoCaptureVideo = false;
            int autoVideoDurationSec = 0;
            {
                std::lock_guard<std::mutex> lock(alarmFileMutex);
                std::ifstream fileIn("/data/lintech/celectronicfence/alarms.json");
                json alarmsJson;
                
                if (fileIn.is_open()) {
                    fileIn >> alarmsJson;
                    fileIn.close();
                } else {
                    alarmsJson = json::array();
                }
                if (!alarmsJson.is_array()) {
                    alarmsJson = json::array();
                }
                
                int newId = 1;
                if (!alarmsJson.empty()) {
                    for (const auto& alarm : alarmsJson) {
                        if (alarm.contains("id") && alarm["id"].is_number()) {
                            newId = std::max(newId, (int)alarm["id"] + 1);
                        }
                    }
                }
                
                newAlarm["id"] = newId;
                int taskId = requestData.value("taskId", 0);
                std::string alarmType = requestData.value("alarmType", "");
                std::string imageUrl = normalizeAlarmImageUrl(requestData.value("imageUrl", ""));
                std::string normalizedLocalImageUrl;
                std::string normalizedLocalImagePath;
                if (buildLocalAlarmImagePath(imageUrl, normalizedLocalImageUrl, normalizedLocalImagePath))
                {
                    imageUrl = normalizedLocalImageUrl;
                    if (normalizedLocalImagePath.empty() || !isAlarmImageAvailable(imageUrl))
                    {
                        imageUrl.clear();
                    }
                }
                if (imageUrl.empty())
                {
                    imageUrl = tryGenerateAlarmImageFromTask(taskId, alarmType);
                }

                std::string videoUrl = normalizeAlarmVideoUrl(requestData.value("videoUrl", ""));
                std::string normalizedLocalVideoUrl;
                std::string normalizedLocalVideoPath;
                bool localVideoRef = buildLocalAlarmVideoPath(
                    videoUrl, normalizedLocalVideoUrl, normalizedLocalVideoPath);
                bool hasReadyVideo = false;
                if (localVideoRef)
                {
                    videoUrl = normalizedLocalVideoUrl;
                    hasReadyVideo = !normalizedLocalVideoPath.empty() &&
                                    isAlarmVideoAvailable(videoUrl);
                    if (!hasReadyVideo)
                    {
                        videoUrl.clear();
                    }
                }
                else
                {
                    hasReadyVideo = !videoUrl.empty();
                }

                autoVideoDurationSec = readAlarmVideoDurationSecondsFromParams();
                if (requestData.contains("videoDurationSec"))
                {
                    autoVideoDurationSec = clampInt(
                        parseJsonInt(requestData["videoDurationSec"], autoVideoDurationSec),
                        3,
                        60
                    );
                }

                std::string videoMimeType = trimString(requestData.value("videoMimeType", ""));
                if (videoMimeType.empty() && !videoUrl.empty())
                {
                    if (localVideoRef)
                    {
                        videoMimeType = guessRecordingMimeType(normalizedLocalVideoPath);
                    }
                    else
                    {
                        videoMimeType = guessRecordingMimeType(videoUrl);
                    }
                }
                if (videoMimeType.empty())
                {
                    videoMimeType = "video/mp4";
                }

                newAlarm["taskId"] = taskId;
                newAlarm["videoSourceId"] = requestData.value("videoSourceId", 0);
                newAlarm["videoSourceName"] = requestData.value("videoSourceName", "");
                newAlarm["alarmType"] = alarmType;
                newAlarm["imageUrl"] = imageUrl;
                newAlarm["videoUrl"] = videoUrl;
                newAlarm["videoDurationSec"] = autoVideoDurationSec;
                newAlarm["videoMimeType"] = videoMimeType;
                
                std::string remoteUrl = getRemoteAlarmUrl();
                std::string requestReportUrl = trimString(requestData.value("reportUrl", ""));
                if (!remoteUrl.empty()) {
                    newAlarm["reportUrl"] = remoteUrl;
                    newAlarm["reportStatus"] = "上报中";
                } else if (!requestReportUrl.empty()) {
                    newAlarm["reportUrl"] = requestReportUrl;
                    newAlarm["reportStatus"] = "上报中";
                } else {
                    newAlarm["reportUrl"] = "";
                    newAlarm["reportStatus"] = "未上报";
                }
                
                newAlarm["status"] = requestData.value("status", "pending");
                newAlarm["description"] = requestData.value("description", "");

                if (hasReadyVideo)
                {
                    newAlarm["videoStatus"] = trimString(requestData.value("videoStatus", "ready"));
                }
                else if (isAlarmVideoClipEnabled() && taskId > 0)
                {
                    std::string outputRtsp = findTaskOutputRtspByTaskId(taskId);
                    if (!outputRtsp.empty())
                    {
                        newAlarm["videoStatus"] = "pending";
                        shouldAutoCaptureVideo = true;
                    }
                    else
                    {
                        newAlarm["videoStatus"] = "failed";
                        newAlarm["videoError"] = "Task output stream is empty";
                    }
                }
                else
                {
                    newAlarm["videoStatus"] = trimString(requestData.value("videoStatus", ""));
                }
                
                auto now = std::chrono::system_clock::now();
                std::time_t now_c = std::chrono::system_clock::to_time_t(now);
                std::tm* now_tm = std::localtime(&now_c);
                char timeBuffer[32];
                std::strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", now_tm);
                newAlarm["timestamp"] = std::string(timeBuffer);
                
                alarmsJson.push_back(newAlarm);
                
                std::ofstream fileOut("/data/lintech/celectronicfence/alarms.json");
                fileOut << alarmsJson.dump(4);
                fileOut.close();
            }
            
            if (shouldAutoCaptureVideo)
            {
                processAlarmVideoAndReportAsync(
                    newAlarm.value("id", 0),
                    newAlarm.value("taskId", 0),
                    newAlarm.value("alarmType", "")
                );
            }
            else
            {
                reportAlarmToRemote(newAlarm);
            }
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Alarm added successfully";
            responseJson["alarm"] = newAlarm;
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 删除告警
    svr.Delete("/api/alarms/:id", [](const Request &req, Response &res)
    {
        try {
            int alarmId = std::stoi(req.path_params.at("id"));
            
            json newAlarmsJson = json::array();
            bool found = false;
            std::string imageUrlToDelete = "";
            std::string videoUrlToDelete = "";
            {
                std::lock_guard<std::mutex> lock(alarmFileMutex);
                std::ifstream fileIn("/data/lintech/celectronicfence/alarms.json");
                json alarmsJson;
                
                if (fileIn.is_open()) {
                    fileIn >> alarmsJson;
                    fileIn.close();
                } else {
                    alarmsJson = json::array();
                }
                
                for (const auto& alarm : alarmsJson) {
                    if (alarm["id"] == alarmId) {
                        found = true;
                        if (alarm.contains("imageUrl") && alarm["imageUrl"].is_string()) {
                            imageUrlToDelete = alarm["imageUrl"].get<std::string>();
                        }
                        if (alarm.contains("videoUrl") && alarm["videoUrl"].is_string()) {
                            videoUrlToDelete = alarm["videoUrl"].get<std::string>();
                        }
                    } else {
                        newAlarmsJson.push_back(alarm);
                    }
                }

                if (found)
                {
                    std::ofstream fileOut("/data/lintech/celectronicfence/alarms.json");
                    fileOut << newAlarmsJson.dump(4);
                    fileOut.close();
                }
            }
            
            if (!found) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Alarm not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }
            
            // 删除图片文件
            if (!imageUrlToDelete.empty()) {
                std::string localImageUrl;
                std::string imagePath;
                if (buildLocalAlarmImagePath(imageUrlToDelete, localImageUrl, imagePath)) {
                    if (std::remove(imagePath.c_str()) == 0) {
                        std::cout << "[删除告警] 已删除图片文件: " << localImageUrl << std::endl;
                    } else {
                        std::cerr << "[删除告警] 无法删除图片文件: " << localImageUrl << " (可能已不存在)" << std::endl;
                    }
                }
            }

            if (!videoUrlToDelete.empty()) {
                std::string localVideoUrl;
                std::string videoPath;
                if (buildLocalAlarmVideoPath(videoUrlToDelete, localVideoUrl, videoPath)) {
                    if (std::remove(videoPath.c_str()) == 0) {
                        std::cout << "[删除告警] 已删除视频文件: " << localVideoUrl << std::endl;
                    } else {
                        std::cerr << "[删除告警] 无法删除视频文件: " << localVideoUrl << " (可能已不存在)" << std::endl;
                    }
                }
            }
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Alarm deleted successfully";
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 批量删除告警
    svr.Post("/api/alarms/batch-delete", [](const Request &req, Response &res)
    {
        try {
            json requestData = json::parse(req.body);
            json idsToDelete = requestData["ids"];
            
            json newAlarmsJson = json::array();
            std::vector<std::string> imageUrlsToDelete;
            std::vector<std::string> videoUrlsToDelete;
            
            {
                std::lock_guard<std::mutex> lock(alarmFileMutex);
                std::ifstream fileIn("/data/lintech/celectronicfence/alarms.json");
                json alarmsJson;
                
                if (fileIn.is_open()) {
                    fileIn >> alarmsJson;
                    fileIn.close();
                } else {
                    alarmsJson = json::array();
                }

                for (const auto& alarm : alarmsJson) {
                    bool shouldDelete = false;
                    for (const auto& id : idsToDelete) {
                        if (alarm["id"] == id) {
                            shouldDelete = true;
                            if (alarm.contains("imageUrl") && alarm["imageUrl"].is_string()) {
                                imageUrlsToDelete.push_back(alarm["imageUrl"].get<std::string>());
                            }
                            if (alarm.contains("videoUrl") && alarm["videoUrl"].is_string()) {
                                videoUrlsToDelete.push_back(alarm["videoUrl"].get<std::string>());
                            }
                            break;
                        }
                    }
                    if (!shouldDelete) {
                        newAlarmsJson.push_back(alarm);
                    }
                }

                std::ofstream fileOut("/data/lintech/celectronicfence/alarms.json");
                fileOut << newAlarmsJson.dump(4);
                fileOut.close();
            }
            
            // 批量删除图片文件
            int deletedImageCount = 0;
            for (const auto& imageUrl : imageUrlsToDelete) {
                std::string localImageUrl;
                std::string imagePath;
                if (!buildLocalAlarmImagePath(imageUrl, localImageUrl, imagePath)) {
                    continue;
                }
                if (std::remove(imagePath.c_str()) == 0) {
                    deletedImageCount++;
                } else {
                    std::cerr << "[批量删除告警] 无法删除图片文件: " << localImageUrl << " (可能已不存在)" << std::endl;
                }
            }
            std::cout << "[批量删除告警] 已删除 " << deletedImageCount << " 个图片文件" << std::endl;

            int deletedVideoCount = 0;
            for (const auto& videoUrl : videoUrlsToDelete) {
                std::string localVideoUrl;
                std::string videoPath;
                if (!buildLocalAlarmVideoPath(videoUrl, localVideoUrl, videoPath)) {
                    continue;
                }
                if (std::remove(videoPath.c_str()) == 0) {
                    deletedVideoCount++;
                } else {
                    std::cerr << "[批量删除告警] 无法删除视频文件: " << localVideoUrl << " (可能已不存在)" << std::endl;
                }
            }
            std::cout << "[批量删除告警] 已删除 " << deletedVideoCount << " 个视频文件" << std::endl;
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Alarms deleted successfully";
            responseJson["deletedCount"] = idsToDelete.size();
            responseJson["deletedImageCount"] = deletedImageCount;
            responseJson["deletedVideoCount"] = deletedVideoCount;
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 手动触发告警上报到远程地址
    svr.Post("/api/alarms/:id/report", [](const Request &req, Response &res)
    {
        try {
            int alarmId = std::stoi(req.path_params.at("id"));
            
            json targetAlarm;
            if (!readAlarmRecordById(alarmId, targetAlarm)) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Alarm not found";
                res.status = 404;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }
            
            // 触发远程上报
            reportAlarmToRemote(targetAlarm);
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Alarm report triggered";
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 清理孤立的告警图片（没有对应告警记录的图片）
    svr.Post("/api/alarms/cleanup-images", [](const Request &req, Response &res)
    {
        try {
            // 读取现有告警记录
            json alarmsJson;
            {
                std::lock_guard<std::mutex> lock(alarmFileMutex);
                std::ifstream fileIn("/data/lintech/celectronicfence/alarms.json");
                if (fileIn.is_open()) {
                    fileIn >> alarmsJson;
                    fileIn.close();
                } else {
                    alarmsJson = json::array();
                }
            }
            
            // 收集所有告警记录中的媒体文件名
            std::set<std::string> validMediaFiles;
            for (const auto& alarm : alarmsJson) {
                if (alarm.contains("imageUrl") && alarm["imageUrl"].is_string()) {
                    std::string imageUrl = alarm["imageUrl"].get<std::string>();
                    size_t lastSlash = imageUrl.find_last_of('/');
                    if (lastSlash != std::string::npos) {
                        std::string filename = imageUrl.substr(lastSlash + 1);
                        validMediaFiles.insert(filename);
                    }
                }
                if (alarm.contains("videoUrl") && alarm["videoUrl"].is_string()) {
                    std::string videoUrl = alarm["videoUrl"].get<std::string>();
                    size_t lastSlash = videoUrl.find_last_of('/');
                    if (lastSlash != std::string::npos) {
                        std::string filename = videoUrl.substr(lastSlash + 1);
                        validMediaFiles.insert(filename);
                    }
                }
            }
            
            // 遍历告警图片目录，删除孤立文件
            std::string alarmDir = "/data/lintech/celectronicfence/static/upload/alarm";
            std::string listCmd = "ls " + alarmDir + " 2>/dev/null";
            std::string fileList = executeCommand(listCmd.c_str());
            
            int deletedCount = 0;
            std::istringstream iss(fileList);
            std::string filename;
            
            while (std::getline(iss, filename)) {
                if (filename.empty()) continue;
                if (!isSupportedRecordingMediaFile(filename)) continue;
                
                if (validMediaFiles.find(filename) == validMediaFiles.end()) {
                    std::string filepath = alarmDir + "/" + filename;
                    if (std::remove(filepath.c_str()) == 0) {
                        deletedCount++;
                        std::cout << "[清理孤立媒体] 已删除: " << filename << std::endl;
                    }
                }
            }
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Cleanup completed";
            responseJson["deletedCount"] = deletedCount;
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 上传告警图片
    svr.Post("/api/alarms/upload", [](const Request &req, Response &res)
    {
        try {
            auto file = req.get_file_value("image");
            if (file.content.empty()) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "No image file uploaded";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }
            
            // 生成文件名
            auto now = std::chrono::system_clock::now();
            std::time_t now_c = std::chrono::system_clock::to_time_t(now);
            std::tm* now_tm = std::localtime(&now_c);
            char timeBuffer[32];
            std::strftime(timeBuffer, sizeof(timeBuffer), "%Y%m%d_%H%M%S", now_tm);
            
            std::string filename = "alarm_" + std::string(timeBuffer) + "_" + std::to_string(rand() % 10000) + ".jpg";
            std::string filepath = "/data/lintech/celectronicfence/static/upload/alarm/" + filename;
            
            // 保存文件
            std::ofstream ofs(filepath, std::ios::binary);
            ofs << file.content;
            ofs.close();
            
            std::string imageUrl = "/upload/alarm/" + filename;
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["imageUrl"] = imageUrl;
            responseJson["filename"] = filename;
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    // 参数配置API
    
    // 获取所有参数
    svr.Get("/api/params", [](const Request &req, Response &res)
    {
        try {
            std::ifstream file("/data/lintech/celectronicfence/params.json");
            json paramsJson;
            
            if (file.is_open()) {
                file >> paramsJson;
                file.close();
            } else {
                paramsJson = json::object();
            }
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["params"] = paramsJson;
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 保存所有参数
    svr.Post("/api/params", [](const Request &req, Response &res)
    {
        try {
            json requestData = json::parse(req.body);
            json params = requestData["params"];
            
            // 保存到文件
            std::ofstream fileOut("/data/lintech/celectronicfence/params.json");
            fileOut << params.dump(4);
            fileOut.close();
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Parameters saved successfully";
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 更新单个参数
    svr.Put("/api/params/:key", [](const Request &req, Response &res)
    {
        try {
            std::string paramKey = req.path_params.at("key");
            json requestData = json::parse(req.body);
            
            // 读取现有参数
            std::ifstream fileIn("/data/lintech/celectronicfence/params.json");
            json paramsJson;
            
            if (fileIn.is_open()) {
                fileIn >> paramsJson;
                fileIn.close();
            } else {
                paramsJson = json::object();
            }
            
            // 更新参数
            paramsJson[paramKey] = requestData["value"];
            
            // 保存到文件
            std::ofstream fileOut("/data/lintech/celectronicfence/params.json");
            fileOut << paramsJson.dump(4);
            fileOut.close();
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Parameter updated successfully";
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 获取单个参数
    svr.Get("/api/params/:key", [](const Request &req, Response &res)
    {
        try {
            std::string paramKey = req.path_params.at("key");
            
            // 读取参数
            std::ifstream file("/data/lintech/celectronicfence/params.json");
            json paramsJson;
            
            if (file.is_open()) {
                file >> paramsJson;
                file.close();
            } else {
                paramsJson = json::object();
            }
            
            json responseJson;
            responseJson["status"] = "success";
            
            if (paramsJson.contains(paramKey)) {
                responseJson["value"] = paramsJson[paramKey];
            } else {
                responseJson["value"] = nullptr;
            }
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    // 获取NPU和VPU使用率（通过ion堆信息）
    svr.Get("/api/system/npu", [](const Request &req, Response &res)
    {
        try {
            int npuUsage = 0;
            int vpuUsage = 0;
            
            // 获取NPU堆使用情况（不使用head，获取完整输出）
            std::string npuCommand = "cat /sys/kernel/debug/ion/cvi_npu_heap_dump/summary 2>&1";
            std::string npuResult = executeCommand(npuCommand.c_str());
            
            std::cout << "=== NPU堆信息输出 ===" << std::endl;
            std::cout << npuResult << std::endl;
            std::cout << "===================" << std::endl;
            
            // 解析NPU使用率 - 查找 "used:XXX bytes"
            size_t npuSizePos = npuResult.find("heap size:");
            size_t npuUsedPos = npuResult.find("used:");
            
            if (npuSizePos != std::string::npos && npuUsedPos != std::string::npos) {
                // 提取heap size
                size_t sizeStart = npuSizePos + 10; // "heap size:" 长度
                size_t sizeEnd = npuResult.find(" bytes", sizeStart);
                
                // 提取used
                size_t usedStart = npuUsedPos + 5; // "used:" 长度
                size_t usedEnd = npuResult.find(" bytes", usedStart);
                
                if (sizeEnd != std::string::npos && usedEnd != std::string::npos) {
                    try {
                        std::string sizeStr = npuResult.substr(sizeStart, sizeEnd - sizeStart);
                        std::string usedStr = npuResult.substr(usedStart, usedEnd - usedStart);
                        
                        long long heapSize = std::stoll(sizeStr);
                        long long heapUsed = std::stoll(usedStr);
                        
                        if (heapSize > 0) {
                            npuUsage = (int)((heapUsed * 100) / heapSize);
                        }
                        
                        std::cout << "NPU堆大小: " << heapSize << " bytes, 已使用: " << heapUsed << " bytes" << std::endl;
                        std::cout << "NPU使用率: " << npuUsage << "%" << std::endl;
                    } catch (const std::exception& e) {
                        std::cout << "NPU数据解析失败: " << e.what() << std::endl;
                    }
                }
            }
            
            // 获取VPU堆使用情况（不使用head，获取完整输出）
            std::string vpuCommand = "cat /sys/kernel/debug/ion/cvi_vpp_heap_dump/summary 2>&1";
            std::string vpuResult = executeCommand(vpuCommand.c_str());
            
            std::cout << "=== VPU堆信息输出 ===" << std::endl;
            std::cout << vpuResult << std::endl;
            std::cout << "===================" << std::endl;
            
            // 解析VPU使用率 - 查找 "used:XXX bytes"
            size_t vpuSizePos = vpuResult.find("heap size:");
            size_t vpuUsedPos = vpuResult.find("used:");
            
            if (vpuSizePos != std::string::npos && vpuUsedPos != std::string::npos) {
                // 提取heap size
                size_t sizeStart = vpuSizePos + 10;
                size_t sizeEnd = vpuResult.find(" bytes", sizeStart);
                
                // 提取used
                size_t usedStart = vpuUsedPos + 5;
                size_t usedEnd = vpuResult.find(" bytes", usedStart);
                
                if (sizeEnd != std::string::npos && usedEnd != std::string::npos) {
                    try {
                        std::string sizeStr = vpuResult.substr(sizeStart, sizeEnd - sizeStart);
                        std::string usedStr = vpuResult.substr(usedStart, usedEnd - usedStart);
                        
                        long long heapSize = std::stoll(sizeStr);
                        long long heapUsed = std::stoll(usedStr);
                        
                        if (heapSize > 0) {
                            vpuUsage = (int)((heapUsed * 100) / heapSize);
                        }
                        
                        std::cout << "VPU堆大小: " << heapSize << " bytes, 已使用: " << heapUsed << " bytes" << std::endl;
                        std::cout << "VPU使用率: " << vpuUsage << "%" << std::endl;
                    } catch (const std::exception& e) {
                        std::cout << "VPU数据解析失败: " << e.what() << std::endl;
                    }
                }
            }
            
            json responseJson;
            responseJson["status"] = "success";
            responseJson["npuUsage"] = npuUsage;
            responseJson["vpuUsage"] = vpuUsage;
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            std::cout << "获取NPU/VPU使用率异常: " << e.what() << std::endl;
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            errorJson["npuUsage"] = 0;
            errorJson["vpuUsage"] = 0;
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    // 固件更新API：上传文件并替换指定目标文件
    svr.Post("/api/firmware/upload", [](const Request &req, Response &res)
    {
        try {
            std::lock_guard<std::mutex> lock(firmwareUpdateMutex);

            if (!req.has_file("firmware")) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "No firmware file uploaded";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            auto file = req.get_file_value("firmware");
            if (file.content.empty()) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Uploaded firmware file is empty";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            auto getFormText = [&](const std::string &key) -> std::string {
                if (req.has_param(key)) {
                    return trimString(req.get_param_value(key));
                }
                if (req.has_file(key)) {
                    return trimString(req.get_file_value(key).content);
                }
                return "";
            };

            std::string uploadedFilename = trimString(file.filename);
            size_t nameSlashPos = uploadedFilename.find_last_of("/\\");
            if (nameSlashPos != std::string::npos) {
                uploadedFilename = uploadedFilename.substr(nameSlashPos + 1);
            }
            if (uploadedFilename.empty() ||
                uploadedFilename == "." ||
                uploadedFilename == ".." ||
                uploadedFilename.find('/') != std::string::npos ||
                uploadedFilename.find('\\') != std::string::npos ||
                uploadedFilename.find('\n') != std::string::npos ||
                uploadedFilename.find('\r') != std::string::npos) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Invalid uploaded filename";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::string targetDir = getFormText("targetDir");
            std::string targetPath = getFormText("targetPath");

            if (!targetDir.empty()) {
                if (targetDir[0] != '/') {
                    json errorJson;
                    errorJson["status"] = "error";
                    errorJson["message"] = "Invalid targetDir, must be an absolute directory path";
                    res.status = 400;
                    res.set_content(errorJson.dump(), "application/json");
                    return;
                }
                if (targetDir.find('\n') != std::string::npos || targetDir.find('\r') != std::string::npos) {
                    json errorJson;
                    errorJson["status"] = "error";
                    errorJson["message"] = "Invalid targetDir";
                    res.status = 400;
                    res.set_content(errorJson.dump(), "application/json");
                    return;
                }
                while (targetDir.size() > 1 && targetDir.back() == '/') {
                    targetDir.pop_back();
                }
                if (!isDirectoryPath(targetDir)) {
                    json errorJson;
                    errorJson["status"] = "error";
                    errorJson["message"] = "Target directory does not exist";
                    res.status = 400;
                    res.set_content(errorJson.dump(), "application/json");
                    return;
                }
                targetPath = targetDir + "/" + uploadedFilename;
            } else {
                if (targetPath.empty()) {
                    targetPath = FIRMWARE_TARGET_PATH;
                }
                if (targetPath.empty() || targetPath[0] != '/' || targetPath.back() == '/') {
                    json errorJson;
                    errorJson["status"] = "error";
                    errorJson["message"] = "Invalid targetPath, must be an absolute file path";
                    res.status = 400;
                    res.set_content(errorJson.dump(), "application/json");
                    return;
                }
                if (targetPath.find('\n') != std::string::npos || targetPath.find('\r') != std::string::npos) {
                    json errorJson;
                    errorJson["status"] = "error";
                    errorJson["message"] = "Invalid targetPath";
                    res.status = 400;
                    res.set_content(errorJson.dump(), "application/json");
                    return;
                }
            }

            size_t slashPos = targetPath.find_last_of('/');
            std::string parentDir = (slashPos == 0) ? "/" : targetPath.substr(0, slashPos);
            if (parentDir.empty() || !isDirectoryPath(parentDir)) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Target directory does not exist";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            std::string tempPath = targetPath + ".upload.tmp";
            std::remove(tempPath.c_str());

            std::ofstream tempOut(tempPath.c_str(), std::ios::binary | std::ios::trunc);
            if (!tempOut.is_open()) {
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Cannot create temporary firmware file";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            tempOut.write(file.content.data(), static_cast<std::streamsize>(file.content.size()));
            tempOut.close();
            if (!tempOut.good()) {
                std::remove(tempPath.c_str());
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Failed to write temporary firmware file";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            struct stat targetStat;
            bool hadOldTarget = (stat(targetPath.c_str(), &targetStat) == 0);
            if (hadOldTarget && !S_ISREG(targetStat.st_mode)) {
                std::remove(tempPath.c_str());
                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "targetPath is not a regular file";
                res.status = 400;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }
            mode_t fileMode = hadOldTarget ? (targetStat.st_mode & 0777) : 0755;
            chmod(tempPath.c_str(), fileMode);

            if (std::rename(tempPath.c_str(), targetPath.c_str()) != 0) {
                std::remove(tempPath.c_str());

                json errorJson;
                errorJson["status"] = "error";
                errorJson["message"] = "Failed to replace firmware file";
                res.status = 500;
                res.set_content(errorJson.dump(), "application/json");
                return;
            }

            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Firmware file replaced successfully";
            responseJson["targetPath"] = targetPath;
            responseJson["targetDir"] = parentDir;
            responseJson["savedFilename"] = uploadedFilename;
            responseJson["filename"] = file.filename;
            responseJson["size"] = static_cast<long long>(file.content.size());

            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    // 重启设备API
    svr.Post("/api/reboot", [](const Request &req, Response &res)
    {
        try {
            json responseJson;
            responseJson["status"] = "success";
            responseJson["message"] = "Device is rebooting...";
            
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_content(responseJson.dump(), "application/json");
            
            // 在后台线程中执行重启命令
            std::thread rebootThread([]() {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                std::system("sudo reboot");
            });
            rebootThread.detach();
        } catch (const std::exception& e) {
            json errorJson;
            errorJson["status"] = "error";
            errorJson["message"] = e.what();
            res.status = 500;
            res.set_content(errorJson.dump(), "application/json");
        }
    });
    
    //----------------------------------------------------------------------------------------------------------------------------
    route_polygon(svr);

    stopRegionAlarmMonitor.store(false);
    std::thread regionAlarmMonitorThread(regionAlarmMonitorThreadFunction);
    regionAlarmMonitorThread.detach();

    // 加载 JSON 文件到全局变量
    json data;
    JsonFile2::loadJsonData(data);
    svr.listen("0.0.0.0", 8088);

    return 0;
}
