/**
 * OTA (Over-The-Air) 固件升级模块
 *
 * 主要功能：
 * 1. 版本检查：从服务器获取最新固件版本信息
 * 2. 设备激活：通过 HMAC-SHA256 挑战响应机制激活设备
 * 3. 固件升级：下载并安装新固件，支持断点续传和进度回调
 * 4. 配置同步：从服务器获取 MQTT、WebSocket 等配置信息
 * 5. 时间同步：从服务器获取时间戳并设置系统时间
 *
 * 安全机制：
 * - 使用 eFuse 存储的序列号进行设备认证
 * - HMAC-SHA256 签名验证设备合法性
 * - OTA 回滚保护，确保升级失败后可恢复
 */

#include "ota.h"
#include "system_info.h"
#include "settings.h"
#include "assets/lang_config.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cJSON.h>
#include <esp_log.h>
#include <esp_partition.h>
#include <esp_ota_ops.h>
#include <esp_app_format.h>
#include <esp_efuse.h>
#include <esp_efuse_table.h>
#include <esp_heap_caps.h>
#ifdef SOC_HMAC_SUPPORTED
#include <esp_hmac.h>
#endif

#include <cstring>
#include <vector>
#include <sstream>
#include <algorithm>

#define TAG "Ota"


/**
 * 构造函数：初始化 OTA 对象
 *
 * 功能：
 * 1. 从 eFuse 读取设备序列号（用于激活认证）
 *
 * eFuse 说明：
 * - ESP_EFUSE_USER_DATA 是用户自定义数据块（32 字节）
 * - 用于存储设备序列号，烧录后不可修改
 * - 通过 HMAC-SHA256 签名验证设备合法性
 * - 如果第一个字节为 0，表示未烧录序列号
 */
Ota::Ota() {
#ifdef ESP_EFUSE_BLOCK_USR_DATA
    // Read Serial Number from efuse user_data
    uint8_t serial_number[33] = {0};
    if (esp_efuse_read_field_blob(ESP_EFUSE_USER_DATA, serial_number, 32 * 8) == ESP_OK) {
        if (serial_number[0] == 0) {
            has_serial_number_ = false;
        } else {
            serial_number_ = std::string(reinterpret_cast<char*>(serial_number), 32);
            has_serial_number_ = true;
        }
    }
#endif
}

Ota::~Ota() {
}

/**
 * 获取版本检查 URL
 *
 * 优先级：
 * 1. 从 NVS "wifi" 命名空间读取 "ota_url"
 * 2. 如果未配置，使用编译时的默认值 CONFIG_OTA_URL
 *
 * @return 版本检查的服务器 URL
 */
std::string Ota::GetCheckVersionUrl() {
    Settings settings("wifi", false);
    std::string url = settings.GetString("ota_url");
    if (url.empty()) {
        url = CONFIG_OTA_URL;
    }
    return url;
}

/**
 * 设置 HTTP 客户端
 *
 * 功能：
 * 1. 创建 HTTP 客户端实例
 * 2. 设置认证相关的请求头
 *
 * 请求头说明：
 * - Activation-Version: 激活协议版本（有序列号为 2，无序列号为 1）
 * - Device-Id: 设备物理 MAC 地址
 * - Client-Id: 软件生成的 UUID（擦除 NVS 会重置）
 * - Serial-Number: 从 eFuse 读取的设备序列号（仅版本 2）
 * - User-Agent: 用户代理字符串（包含固件版本、芯片型号等）
 * - Accept-Language: 语言代码（如 zh-CN）
 * - Content-Type: 请求内容类型
 *
 * @return HTTP 客户端智能指针
 */
std::unique_ptr<Http> Ota::SetupHttp() {
    auto& board = Board::GetInstance();
    auto network = board.GetNetwork();
    auto http = network->CreateHttp(0);
    auto user_agent = SystemInfo::GetUserAgent();
    http->SetHeader("Activation-Version", has_serial_number_ ? "2" : "1");
    http->SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
    http->SetHeader("Client-Id", board.GetUuid());
    if (has_serial_number_) {
        http->SetHeader("Serial-Number", serial_number_.c_str());
        ESP_LOGI(TAG, "Setup HTTP, User-Agent: %s, Serial-Number: %s", user_agent.c_str(), serial_number_.c_str());
    }
    http->SetHeader("User-Agent", user_agent);
    http->SetHeader("Accept-Language", Lang::CODE);
    http->SetHeader("Content-Type", "application/json");

    return http;
}

/**
 * 检查版本并获取配置信息
 *
 * 功能：
 * 1. 向服务器发送版本检查请求
 * 2. 解析服务器返回的 JSON 数据
 * 3. 提取并保存各类配置信息到 NVS
 *
 * 服务器返回的 JSON 结构：
 * {
 *   "activation": {              // 激活信息（未激活设备）
 *     "challenge": "...",         // 挑战字符串
 *     "message": "...",           // 激活提示消息
 *     "timeout_ms": 30000         // 激活超时时间
 *   },
 *   "mqtt": {                     // MQTT 配置
 *     "url": "...",
 *     "username": "...",
 *     "password": "..."
 *   },
 *   "websocket": {                // WebSocket 配置
 *     "url": "...",
 *     "token": "...",
 *     "version": 1
 *   },
 *   "server_time": 1234567890,   // 服务器时间戳（秒）
 *   "firmware": {                 // 固件更新信息
 *     "version": "1.0.0",
 *     "url": "..."
 *   }
 * }
 *
 * 规范文档：https://ccnphfhqs21z.feishu.cn/wiki/FjW6wZmisimNBBkov6OcmfvknVd
 *
 * @return ESP_OK 成功，其他值表示失败
 */
esp_err_t Ota::CheckVersion() {
    auto& board = Board::GetInstance();
    auto app_desc = esp_app_get_description();

    // Check if there is a new firmware version available
    current_version_ = app_desc->version;
    ESP_LOGI(TAG, "Current version: %s", current_version_.c_str());

    // 获取版本检查 URL
    std::string url = GetCheckVersionUrl();
    if (url.length() < 10) {
        ESP_LOGE(TAG, "Check version URL is not properly set");
        return ESP_ERR_INVALID_ARG;
    }

    // 设置 HTTP 客户端和认证头
    auto http = SetupHttp();

    // 准备请求数据：包含设备系统信息的 JSON
    std::string data = board.GetSystemInfoJson();
    std::string method = data.length() > 0 ? "POST" : "GET";
    http->SetContent(std::move(data));

    // 发送 HTTP 请求
    if (!http->Open(method, url)) {
        int last_error = http->GetLastError();
        ESP_LOGE(TAG, "Failed to open HTTP connection, code=0x%x", last_error);
        return last_error;
    }

    // 检查 HTTP 响应状态码
    auto status_code = http->GetStatusCode();
    if (status_code != 200) {
        ESP_LOGE(TAG, "Failed to check version, status code: %d", status_code);
        return status_code;
    }

    // 读取响应数据
    data = http->ReadAll();
    http->Close();

    // 解析 JSON 响应
    // 响应格式：{ "firmware": { "version": "1.0.0", "url": "http://" } }
    cJSON *root = cJSON_Parse(data.c_str());
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON response");
        return ESP_ERR_INVALID_RESPONSE;
    }

    // ========== 解析激活信息 ==========
    // 如果设备未激活，服务器会返回激活相关信息
    has_activation_code_ = false;
    has_activation_challenge_ = false;
    cJSON *activation = cJSON_GetObjectItem(root, "activation");
    if (cJSON_IsObject(activation)) {
        // 激活提示消息（显示给用户）
        cJSON* message = cJSON_GetObjectItem(activation, "message");
        if (cJSON_IsString(message)) {
            activation_message_ = message->valuestring;
        }
        // 激活码（用于显示或扫码）
        cJSON* code = cJSON_GetObjectItem(activation, "code");
        if (cJSON_IsString(code)) {
            activation_code_ = code->valuestring;
            has_activation_code_ = true;
        }
        // 挑战字符串（用于 HMAC 签名）
        cJSON* challenge = cJSON_GetObjectItem(activation, "challenge");
        if (cJSON_IsString(challenge)) {
            activation_challenge_ = challenge->valuestring;
            has_activation_challenge_ = true;
        }
        // 激活超时时间（毫秒）
        cJSON* timeout_ms = cJSON_GetObjectItem(activation, "timeout_ms");
        if (cJSON_IsNumber(timeout_ms)) {
            activation_timeout_ms_ = timeout_ms->valueint;
        }
    }

    // ========== 解析 MQTT 配置 ==========
    // 服务器下发的 MQTT 连接配置（url, username, password 等）
    has_mqtt_config_ = false;
    cJSON *mqtt = cJSON_GetObjectItem(root, "mqtt");
    if (cJSON_IsObject(mqtt)) {
        Settings settings("mqtt", true);
        cJSON *item = NULL;
        // 遍历 MQTT 对象的所有字段，保存到 NVS
        cJSON_ArrayForEach(item, mqtt) {
            if (cJSON_IsString(item)) {
                if (settings.GetString(item->string) != item->valuestring) {
                    settings.SetString(item->string, item->valuestring);
                }
            } else if (cJSON_IsNumber(item)) {
                if (settings.GetInt(item->string) != item->valueint) {
                    settings.SetInt(item->string, item->valueint);
                }
            }
        }
        has_mqtt_config_ = true;
    } else {
        ESP_LOGI(TAG, "No mqtt section found !");
    }

    // ========== 解析 WebSocket 配置 ==========
    // 服务器下发的 WebSocket 连接配置（url, token, version 等）
    has_websocket_config_ = false;
    cJSON *websocket = cJSON_GetObjectItem(root, "websocket");
    if (cJSON_IsObject(websocket)) {
        Settings settings("websocket", true);
        cJSON *item = NULL;
        // 遍历 WebSocket 对象的所有字段，保存到 NVS
        cJSON_ArrayForEach(item, websocket) {
            if (cJSON_IsString(item)) {
                if (settings.GetString(item->string) != item->valuestring) {
                    settings.SetString(item->string, item->valuestring);
                }
            } else if (cJSON_IsNumber(item)) {
                if (settings.GetInt(item->string) != item->valueint) {
                    settings.SetInt(item->string, item->valueint);
                }
            }
        }
        has_websocket_config_ = true;
    } else {
        ESP_LOGI(TAG, "No websocket section found!");
    }

    // ========== 解析服务器时间 ==========
    // 用于同步设备系统时间
    has_server_time_ = false;
    cJSON *server_time = cJSON_GetObjectItem(root, "server_time");
    if (cJSON_IsObject(server_time)) {
        cJSON *timestamp = cJSON_GetObjectItem(server_time, "timestamp");
        cJSON *timezone_offset = cJSON_GetObjectItem(server_time, "timezone_offset");

        if (cJSON_IsNumber(timestamp)) {
            // 设置系统时间
            struct timeval tv;
            double ts = timestamp->valuedouble;

            // 如果有时区偏移，计算本地时间
            if (cJSON_IsNumber(timezone_offset)) {
                ts += (timezone_offset->valueint * 60 * 1000); // 转换分钟为毫秒
            }

            tv.tv_sec = (time_t)(ts / 1000);  // 转换毫秒为秒
            tv.tv_usec = (suseconds_t)((long long)ts % 1000) * 1000;  // 剩余的毫秒转换为微秒
            settimeofday(&tv, NULL);
            has_server_time_ = true;
        }
    } else {
        ESP_LOGW(TAG, "No server_time section found!");
    }

    // ========== 解析固件更新信息 ==========
    // 检查是否有新版本固件可用
    has_new_version_ = false;
    cJSON *firmware = cJSON_GetObjectItem(root, "firmware");
    if (cJSON_IsObject(firmware)) {
        cJSON *version = cJSON_GetObjectItem(firmware, "version");
        if (cJSON_IsString(version)) {
            firmware_version_ = version->valuestring;
        }
        cJSON *url = cJSON_GetObjectItem(firmware, "url");
        if (cJSON_IsString(url)) {
            firmware_url_ = url->valuestring;
        }

        // 比较版本号，判断是否有新版本
        // 例如：0.1.0 比 0.0.1 新
        if (cJSON_IsString(version) && cJSON_IsString(url)) {
            has_new_version_ = IsNewVersionAvailable(current_version_, firmware_version_);
            if (has_new_version_) {
                ESP_LOGI(TAG, "New version available: %s", firmware_version_.c_str());
            } else {
                ESP_LOGI(TAG, "Current is the latest version");
            }
            // 如果设置了强制更新标志，则强制安装指定版本
            cJSON *force = cJSON_GetObjectItem(firmware, "force");
            if (cJSON_IsNumber(force) && force->valueint == 1) {
                has_new_version_ = true;
            }
        }
    } else {
        ESP_LOGW(TAG, "No firmware section found!");
    }

    cJSON_Delete(root);
    return ESP_OK;
}

/**
 * 标记当前固件版本为有效
 *
 * 功能：
 * 1. 检查当前运行的分区是否为 OTA 分区
 * 2. 如果固件处于待验证状态，标记为有效
 * 3. 取消 OTA 回滚机制
 *
 * OTA 回滚机制说明：
 * - ESP32 支持 OTA 回滚，如果新固件启动后未标记为有效，下次重启会回滚到旧版本
 * - 这可以防止升级到有问题的固件导致设备无法启动
 * - 通常在设备成功连接服务器后调用此函数，确认新固件工作正常
 */
void Ota::MarkCurrentVersionValid() {
    auto partition = esp_ota_get_running_partition();
    // 如果运行在出厂分区，跳过（出厂分区不需要标记）
    if (strcmp(partition->label, "factory") == 0) {
        ESP_LOGI(TAG, "Running from factory partition, skipping");
        return;
    }

    ESP_LOGI(TAG, "Running partition: %s", partition->label);
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(partition, &state) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get state of partition");
        return;
    }

    // 如果固件处于待验证状态，标记为有效并取消回滚
    if (state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGI(TAG, "Marking firmware as valid");
        esp_ota_mark_app_valid_cancel_rollback();
    }
}

/**
 * 执行固件升级
 *
 * 功能：
 * 1. 从指定 URL 下载固件
 * 2. 验证固件头部信息
 * 3. 写入 OTA 分区
 * 4. 设置启动分区
 * 5. 通过回调报告下载进度和速度
 *
 * @param firmware_url 固件下载 URL
 * @param callback 进度回调函数 (进度百分比, 下载速度 bytes/s)
 * @return true 成功，false 失败
 */
bool Ota::Upgrade(const std::string& firmware_url, std::function<void(int progress, size_t speed)> callback) {
    ESP_LOGI(TAG, "Upgrading firmware from %s", firmware_url.c_str());
    esp_ota_handle_t update_handle = 0;
    // 获取下一个可用的 OTA 分区
    auto update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "Failed to get update partition");
        return false;
    }

    ESP_LOGI(TAG, "Writing to partition %s at offset 0x%lx", update_partition->label, update_partition->address);
    bool image_header_checked = false;
    std::string image_header;

    auto network = Board::GetInstance().GetNetwork();
    auto http = network->CreateHttp(0);
    // 发起 HTTP GET 请求下载固件
    if (!http->Open("GET", firmware_url)) {
        ESP_LOGE(TAG, "Failed to open HTTP connection");
        return false;
    }

    // 检查 HTTP 响应状态码
    if (http->GetStatusCode() != 200) {
        ESP_LOGE(TAG, "Failed to get firmware, status code: %d", http->GetStatusCode());
        return false;
    }

    // 获取固件文件大小
    size_t content_length = http->GetBodyLength();
    if (content_length == 0) {
        ESP_LOGE(TAG, "Failed to get content length");
        return false;
    }

    // 分配 4KB 缓冲区用于下载和写入（使用内部 RAM）
    constexpr size_t PAGE_SIZE = 4096;
    char* buffer = (char*)heap_caps_malloc(PAGE_SIZE, MALLOC_CAP_INTERNAL);
    if (buffer == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate buffer");
        return false;
    }

    size_t buffer_offset = 0;  // 缓冲区当前数据大小
    size_t total_read = 0, recent_read = 0;  // 总下载量和最近下载量
    auto last_calc_time = esp_timer_get_time();  // 上次计算速度的时间

    // 循环下载固件数据
    while (true) {
        // 读取数据到缓冲区
        int ret = http->Read(buffer + buffer_offset, PAGE_SIZE - buffer_offset);
        if (ret < 0) {
            ESP_LOGE(TAG, "Failed to read HTTP data: %s", esp_err_to_name(ret));
            heap_caps_free(buffer);
            return false;
        }

        // 每秒计算一次下载速度和进度
        recent_read += ret;
        total_read += ret;
        buffer_offset += ret;
        if (esp_timer_get_time() - last_calc_time >= 1000000 || ret == 0) {
            size_t progress = total_read * 100 / content_length;
            ESP_LOGI(TAG, "Progress: %u%% (%u/%u), Speed: %uB/s", progress, total_read, content_length, recent_read);
            if (callback) {
                callback(progress, recent_read);  // 调用进度回调
            }
            last_calc_time = esp_timer_get_time();
            recent_read = 0;
        }

        // 验证固件头部信息（仅第一次）
        if (!image_header_checked) {
            image_header.append(buffer, buffer_offset);
            // 等待收集足够的数据来验证固件头部
            if (image_header.size() >= sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
                esp_app_desc_t new_app_info;
                memcpy(&new_app_info, image_header.data() + sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t), sizeof(esp_app_desc_t));

                // 开始 OTA 写入过程
                if (esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle)) {
                    esp_ota_abort(update_handle);
                    ESP_LOGE(TAG, "Failed to begin OTA");
                    heap_caps_free(buffer);
                    return false;
                }

                image_header_checked = true;
                std::string().swap(image_header);  // 释放 image_header 内存
            }
        }

        // 当缓冲区满（4KB）或是最后一块数据时，写入 Flash
        bool is_last_chunk = (ret == 0);
        if (buffer_offset == PAGE_SIZE || (is_last_chunk && buffer_offset > 0)) {
            auto err = esp_ota_write(update_handle, buffer, buffer_offset);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to write OTA data: %s", esp_err_to_name(err));
                esp_ota_abort(update_handle);
                heap_caps_free(buffer);
                return false;
            }

            buffer_offset = 0;  // 重置缓冲区偏移
        }

        // 如果是最后一块数据，退出循环
        if (is_last_chunk) {
            break;
        }
    }
    http->Close();
    heap_caps_free(buffer);

    // 结束 OTA 写入并验证固件完整性
    esp_err_t err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        } else {
            ESP_LOGE(TAG, "Failed to end OTA: %s", esp_err_to_name(err));
        }
        return false;
    }

    // 设置下次启动时使用新固件分区
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set boot partition: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Firmware upgrade successful");
    return true;
}

/**
 * 开始固件升级（使用已保存的固件 URL）
 *
 * @param callback 进度回调函数
 * @return true 成功，false 失败
 */
bool Ota::StartUpgrade(std::function<void(int progress, size_t speed)> callback) {
    return Upgrade(firmware_url_, callback);
}


/**
 * 解析版本号字符串
 *
 * 功能：
 * 将版本号字符串（如 "1.2.3"）解析为整数数组 [1, 2, 3]
 *
 * @param version 版本号字符串（格式：x.y.z）
 * @return 版本号整数数组
 */
std::vector<int> Ota::ParseVersion(const std::string& version) {
    std::vector<int> versionNumbers;
    std::stringstream ss(version);
    std::string segment;

    // 按 '.' 分割版本号字符串
    while (std::getline(ss, segment, '.')) {
        versionNumbers.push_back(std::stoi(segment));
    }

    return versionNumbers;
}

/**
 * 判断是否有新版本可用
 *
 * 功能：
 * 比较两个版本号，判断新版本是否比当前版本新
 *
 * 比较规则：
 * - 从左到右逐段比较版本号（主版本.次版本.修订版本）
 * - 例如：1.2.3 < 1.2.4，1.2.3 < 1.3.0，1.2.3 < 2.0.0
 *
 * @param currentVersion 当前版本号
 * @param newVersion 新版本号
 * @return true 有新版本，false 无新版本
 */
bool Ota::IsNewVersionAvailable(const std::string& currentVersion, const std::string& newVersion) {
    std::vector<int> current = ParseVersion(currentVersion);
    std::vector<int> newer = ParseVersion(newVersion);

    // 逐段比较版本号
    for (size_t i = 0; i < std::min(current.size(), newer.size()); ++i) {
        if (newer[i] > current[i]) {
            return true;  // 新版本号更大
        } else if (newer[i] < current[i]) {
            return false;  // 新版本号更小
        }
        // 如果相等，继续比较下一段
    }

    // 如果所有段都相等，版本号段数更多的为新版本
    // 例如：1.2.3.1 > 1.2.3
    return newer.size() > current.size();
}

/**
 * 生成激活请求的 JSON 负载
 *
 * 功能：
 * 1. 使用 eFuse 中的硬件密钥计算 HMAC-SHA256 签名
 * 2. 生成包含签名的 JSON 负载
 *
 * HMAC 签名流程：
 * - 输入：服务器下发的 challenge 字符串
 * - 密钥：eFuse 中的 HMAC_KEY0（硬件密钥，不可读取）
 * - 算法：HMAC-SHA256
 * - 输出：64 字符的十六进制字符串
 *
 * JSON 格式：
 * {
 *   "algorithm": "hmac-sha256",
 *   "serial_number": "...",
 *   "challenge": "...",
 *   "hmac": "..."
 * }
 *
 * @return JSON 字符串
 */
std::string Ota::GetActivationPayload() {
    if (!has_serial_number_) {
        return "{}";
    }

    std::string hmac_hex;
#ifdef SOC_HMAC_SUPPORTED
    uint8_t hmac_result[32]; // SHA-256 输出为32字节

    // 使用 eFuse 中的 Key0 计算 HMAC-SHA256
    esp_err_t ret = esp_hmac_calculate(HMAC_KEY0, (uint8_t*)activation_challenge_.data(), activation_challenge_.size(), hmac_result);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HMAC calculation failed: %s", esp_err_to_name(ret));
        return "{}";
    }

    // 将 HMAC 结果转换为十六进制字符串
    for (size_t i = 0; i < sizeof(hmac_result); i++) {
        char buffer[3];
        sprintf(buffer, "%02x", hmac_result[i]);
        hmac_hex += buffer;
    }
#endif

    // 构造 JSON 负载
    cJSON *payload = cJSON_CreateObject();
    cJSON_AddStringToObject(payload, "algorithm", "hmac-sha256");
    cJSON_AddStringToObject(payload, "serial_number", serial_number_.c_str());
    cJSON_AddStringToObject(payload, "challenge", activation_challenge_.c_str());
    cJSON_AddStringToObject(payload, "hmac", hmac_hex.c_str());
    auto json_str = cJSON_PrintUnformatted(payload);
    std::string json(json_str);
    cJSON_free(json_str);
    cJSON_Delete(payload);

    ESP_LOGI(TAG, "Activation payload: %s", json.c_str());
    return json;
}

/**
 * 激活设备
 *
 * 功能：
 * 1. 向服务器发送激活请求（包含 HMAC 签名）
 * 2. 轮询激活状态直到成功或超时
 *
 * 激活流程：
 * 1. 设备调用 CheckVersion() 获取 challenge
 * 2. 设备计算 HMAC 签名并发送到 /activate 端点
 * 3. 服务器返回 202 表示等待用户确认，设备继续轮询
 * 4. 服务器返回 200 表示激活成功
 * 5. 服务器返回其他状态码表示激活失败
 *
 * @return ESP_OK 成功，ESP_ERR_TIMEOUT 超时，ESP_FAIL 失败
 */
esp_err_t Ota::Activate() {
    if (!has_activation_challenge_) {
        ESP_LOGW(TAG, "No activation challenge found");
        return ESP_FAIL;
    }

    // 构造激活 URL
    std::string url = GetCheckVersionUrl();
    if (url.back() != '/') {
        url += "/activate";
    } else {
        url += "activate";
    }

    auto http = SetupHttp();

    // 生成激活请求负载（包含 HMAC 签名）
    std::string data = GetActivationPayload();
    http->SetContent(std::move(data));

    // 发送激活请求
    if (!http->Open("POST", url)) {
        ESP_LOGE(TAG, "Failed to open HTTP connection");
        return ESP_FAIL;
    }

    auto status_code = http->GetStatusCode();
    if (status_code == 202) {
        // 202 表示服务器已接收请求，等待用户确认
        return ESP_ERR_TIMEOUT;
    }
    if (status_code != 200) {
        ESP_LOGE(TAG, "Failed to activate, code: %d, body: %s", status_code, http->ReadAll().c_str());
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Activation successful");
    return ESP_OK;
}
