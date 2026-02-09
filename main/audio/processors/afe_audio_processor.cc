#include "afe_audio_processor.h"
#include <esp_log.h>

// 处理器运行状态标志位
#define PROCESSOR_RUNNING 0x01

#define TAG "AfeAudioProcessor"

// 构造函数：初始化AFE数据指针和事件组
AfeAudioProcessor::AfeAudioProcessor()
    : afe_data_(nullptr) {
    event_group_ = xEventGroupCreate();
}

/**
 * 初始化AFE音频处理器
 * @param codec 音频编解码器指针
 * @param frame_duration_ms 音频帧时长（毫秒）
 * @param models_list 语音识别模型列表（可选）
 */
void AfeAudioProcessor::Initialize(AudioCodec* codec, int frame_duration_ms, srmodel_list_t* models_list) {
    codec_ = codec;
    // 根据帧时长和采样率（16kHz）计算每帧采样点数
    frame_samples_ = frame_duration_ms * 16000 / 1000;

    // 预分配输出缓冲区容量，避免频繁内存分配
    output_buffer_.reserve(frame_samples_);

    // 计算参考通道数量（用于回声消除）
    int ref_num = codec_->input_reference() ? 1 : 0;

    // 构建输入格式字符串：'M'表示麦克风通道，'R'表示参考通道
    std::string input_format;
    for (int i = 0; i < codec_->input_channels() - ref_num; i++) {
        input_format.push_back('M');
    }
    for (int i = 0; i < ref_num; i++) {
        input_format.push_back('R');
    }

    // 初始化或使用传入的模型列表
    srmodel_list_t *models;
    if (models_list == nullptr) {
        models = esp_srmodel_init("model");
    } else {
        models = models_list;
    }

    // 从模型列表中筛选噪声抑制和VAD模型
    char* ns_model_name = esp_srmodel_filter(models, ESP_NSNET_PREFIX, NULL);
    char* vad_model_name = esp_srmodel_filter(models, ESP_VADN_PREFIX, NULL);

    // 初始化AFE配置：语音通信类型，高性能模式
    afe_config_t* afe_config = afe_config_init(input_format.c_str(), NULL, AFE_TYPE_VC, AFE_MODE_HIGH_PERF);
    afe_config->aec_mode = AEC_MODE_VOIP_HIGH_PERF;  // 回声消除：VoIP高性能模式
    afe_config->vad_mode = VAD_MODE_0;                // 语音活动检测模式
    afe_config->vad_min_noise_ms = 100;               // VAD最小噪声时长
    if (vad_model_name != nullptr) {
        afe_config->vad_model_name = vad_model_name;
    }

    // 配置噪声抑制
    if (ns_model_name != nullptr) {
        afe_config->ns_init = true;
        afe_config->ns_model_name = ns_model_name;
        afe_config->afe_ns_mode = AFE_NS_MODE_NET;    // 使用神经网络噪声抑制
    } else {
        afe_config->ns_init = false;
    }

    afe_config->agc_init = false;                      // 禁用自动增益控制
    afe_config->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;  // 优先使用PSRAM

#ifdef CONFIG_USE_DEVICE_AEC
    // 使用设备AEC时，启用AEC，禁用VAD
    afe_config->aec_init = true;
    afe_config->vad_init = false;
#else
    // 不使用设备AEC时，禁用AEC，启用VAD
    afe_config->aec_init = false;
    afe_config->vad_init = true;
#endif

    // 创建AFE接口和数据实例
    afe_iface_ = esp_afe_handle_from_config(afe_config);
    afe_data_ = afe_iface_->create_from_config(afe_config);

    // 创建音频处理任务
    xTaskCreate([](void* arg) {
        auto this_ = (AfeAudioProcessor*)arg;
        this_->AudioProcessorTask();
        vTaskDelete(NULL);
    }, "audio_communication", 4096, this, 3, NULL);
}

// 析构函数：清理AFE资源和事件组
AfeAudioProcessor::~AfeAudioProcessor() {
    if (afe_data_ != nullptr) {
        afe_iface_->destroy(afe_data_);
    }
    vEventGroupDelete(event_group_);
}

// 获取AFE需要的输入数据块大小
size_t AfeAudioProcessor::GetFeedSize() {
    if (afe_data_ == nullptr) {
        return 0;
    }
    return afe_iface_->get_feed_chunksize(afe_data_);
}

/**
 * 向AFE输入音频数据
 * @param data 音频数据（移动语义，避免拷贝）
 *
 * 线程安全：使用互斥锁保护输入缓冲区
 */
void AfeAudioProcessor::Feed(std::vector<int16_t>&& data) {
    if (afe_data_ == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(input_buffer_mutex_);
    // 在锁内检查运行状态，避免与Stop()的竞态条件
    if (!IsRunning()) {
        return;
    }
    // 将数据追加到输入缓冲区
    input_buffer_.insert(input_buffer_.end(), data.begin(), data.end());
    size_t chunk_size = afe_iface_->get_feed_chunksize(afe_data_) * codec_->input_channels();
    // 当缓冲区数据足够时，分块喂给AFE处理
    while (input_buffer_.size() >= chunk_size) {
        afe_iface_->feed(afe_data_, input_buffer_.data());
        input_buffer_.erase(input_buffer_.begin(), input_buffer_.begin() + chunk_size);
    }
}

// 启动音频处理器
void AfeAudioProcessor::Start() {
    xEventGroupSetBits(event_group_, PROCESSOR_RUNNING);
}

/**
 * 停止音频处理器
 * 清空输入缓冲区并重置AFE内部缓冲区
 */
void AfeAudioProcessor::Stop() {
    xEventGroupClearBits(event_group_, PROCESSOR_RUNNING);

    std::lock_guard<std::mutex> lock(input_buffer_mutex_);
    if (afe_data_ != nullptr) {
        afe_iface_->reset_buffer(afe_data_);
    }
    input_buffer_.clear();
}

// 检查处理器是否正在运行
bool AfeAudioProcessor::IsRunning() {
    return xEventGroupGetBits(event_group_) & PROCESSOR_RUNNING;
}

// 设置音频输出回调函数
void AfeAudioProcessor::OnOutput(std::function<void(std::vector<int16_t>&& data)> callback) {
    output_callback_ = callback;
}

// 设置VAD状态变化回调函数
void AfeAudioProcessor::OnVadStateChange(std::function<void(bool speaking)> callback) {
    vad_state_change_callback_ = callback;
}

/**
 * 音频处理任务主循环
 * 在独立的FreeRTOS任务中运行，负责：
 * 1. 从AFE获取处理后的音频数据
 * 2. 检测VAD状态变化并触发回调
 * 3. 将音频数据按帧输出
 */
void AfeAudioProcessor::AudioProcessorTask() {
    auto fetch_size = afe_iface_->get_fetch_chunksize(afe_data_);
    auto feed_size = afe_iface_->get_feed_chunksize(afe_data_);
    ESP_LOGI(TAG, "Audio communication task started, feed size: %d fetch size: %d",
        feed_size, fetch_size);

    while (true) {
        // 等待处理器启动，外部每调用start()一次，启动一次本任务循环的音频处理
        xEventGroupWaitBits(event_group_, PROCESSOR_RUNNING, pdFALSE, pdTRUE, portMAX_DELAY);

        // 从AFE获取处理后的音频数据（带延迟等待）
        auto res = afe_iface_->fetch_with_delay(afe_data_, portMAX_DELAY);
        // 再次检查运行状态，避免在停止时处理数据
        if ((xEventGroupGetBits(event_group_) & PROCESSOR_RUNNING) == 0) {
            continue;
        }
        if (res == nullptr || res->ret_value == ESP_FAIL) {
            if (res != nullptr) {
                ESP_LOGI(TAG, "Error code: %d", res->ret_value);
            }
            continue;
        }

        // 检测VAD状态变化并触发回调，应用中检测到语音端点变化，上层回调函数只做了LED状态指示
        if (vad_state_change_callback_) {
            if (res->vad_state == VAD_SPEECH && !is_speaking_) { //检测到语音，标识为VAD_SPEECH态
                is_speaking_ = true;
                vad_state_change_callback_(true);  // 开始说话
            } else if (res->vad_state == VAD_SILENCE && is_speaking_) { //未检测到语音，标识为VAD_SILENCE态
                is_speaking_ = false;
                vad_state_change_callback_(false);  // 停止说话
            }
        }

        // 处理输出音频数据
        if (output_callback_) {
            size_t samples = res->data_size / sizeof(int16_t);

            // 将数据添加到输出缓冲区
            output_buffer_.insert(output_buffer_.end(), res->data, res->data + samples);

            // 当缓冲区有足够数据时，输出完整的音频帧
            while (output_buffer_.size() >= frame_samples_) {
                if (output_buffer_.size() == frame_samples_) {
                    // 缓冲区大小正好等于帧大小，移动整个缓冲区（避免拷贝）
                    output_callback_(std::move(output_buffer_));
                    output_buffer_.clear();
                    output_buffer_.reserve(frame_samples_);
                } else {
                    // 缓冲区大小超过帧大小，拷贝一帧并移除
                    output_callback_(std::vector<int16_t>(output_buffer_.begin(), output_buffer_.begin() + frame_samples_));
                    output_buffer_.erase(output_buffer_.begin(), output_buffer_.begin() + frame_samples_);
                }
            }
        }
    }
}

/**
 * 启用或禁用设备AEC（回声消除）
 * @param enable true启用设备AEC，false禁用
 *
 * 注意：启用设备AEC时会禁用VAD，反之亦然
 */
void AfeAudioProcessor::EnableDeviceAec(bool enable) {
    if (enable) {
#if CONFIG_USE_DEVICE_AEC
        afe_iface_->disable_vad(afe_data_);
        afe_iface_->enable_aec(afe_data_);
#else
        ESP_LOGE(TAG, "Device AEC is not supported");
#endif
    } else {
        afe_iface_->disable_aec(afe_data_);
        afe_iface_->enable_vad(afe_data_);
    }
}
