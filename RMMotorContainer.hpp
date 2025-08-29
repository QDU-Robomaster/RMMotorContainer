#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 大疆电机驱动
constructor_args:
   - motor0:
         model: RMMotorContainer::Model::MOTOR_NONE
         reverse: false
   - motor1:
         model: RMMotorContainer::Model::MOTOR_NONE
         reverse: false
   - motor2:
         model: RMMotorContainer::Model::MOTOR_NONE
         reverse: false
   - motor3:
         model: RMMotorContainer::Model::MOTOR_NONE
         reverse: false
   - motor4:
         model: RMMotorContainer::Model::MOTOR_NONE
         reverse: false
   - motor5:
         model: RMMotorContainer::Model::MOTOR_NONE
         reverse: false
   - motor6:
         model: RMMotorContainer::Model::MOTOR_NONE
         reverse: false
   - motor7:
         model: RMMotorContainer::Model::MOTOR_NONE
         reverse: false
   - motor8:
         model: RMMotorContainer::Model::MOTOR_NONE
         reverse: false
   - motor9:
         model: RMMotorContainer::Model::MOTOR_NONE
         reverse: false
   - motor10:
         model: RMMotorContainer::Model::MOTOR_NONE
         reverse: false
required_hardware:
  - can
depends: []
=== END MANIFEST === */
// clang-format on

#include "app_framework.hpp"
#include "can.hpp"
#include "cycle_value.hpp"

/* RMMotor id */
/* id     feedback id     control id */
/* 1-4    0x205 to 0x208  0x1fe */
/* 5-6    0x209 to 0x20B  0x2fe */
#define GM6020_FB_ID_BASE (0x205)
#define GM6020_FB_ID_EXTAND (0x209)
#define GM6020_CTRL_ID_BASE (0x1fe)
#define GM6020_CTRL_ID_EXTAND (0x2fe)

/* id     feedback id		  control id */
/* 1-4		0x201 to 0x204  0x200 */
/* 5-6		0x205 to 0x208  0x1ff */
#define M3508_M2006_FB_ID_BASE (0x201)
#define M3508_M2006_FB_ID_EXTAND (0x205)
#define M3508_M2006_CTRL_ID_BASE (0x200)
#define M3508_M2006_CTRL_ID_EXTAND (0x1ff)
#define M3508_M2006_ID_SETTING_ID (0x700)

#define MOTOR_CTRL_ID_NUMBER (4)

#define GM6020_MAX_ABS_LSB (16384)
#define M3508_MAX_ABS_LSB (16384)
#define M2006_MAX_ABS_LSB (10000)

/* 电机最大电流绝对值 */
#define GM6020_MAX_ABS_CUR (3)
#define M3508_MAX_ABS_CUR (20)
#define M2006_MAX_ABS_CUR (10)

#define MOTOR_ENC_RES (8192)  /* 电机编码器分辨率 */
#define MOTOR_CUR_RES (16384) /* 电机转矩电流分辨率 */


class RMMotorContainer : public LibXR::Application {
public:
  static inline uint8_t motor_tx_buff_[MOTOR_CTRL_ID_NUMBER][8];

  static inline uint8_t motor_tx_flag_[MOTOR_CTRL_ID_NUMBER];

  static inline uint8_t motor_tx_map_[MOTOR_CTRL_ID_NUMBER];

  /**
   * @brief 电机型号
   */
  enum class Model {
    MOTOR_NONE = 0,
    MOTOR_M2006,
    MOTOR_M3508,
    MOTOR_GM6020,
  };

  /**
   * @brief 电机参数
   */
  typedef struct {
    Model model;
    bool reverse;
  } Param;

  class RMMotor {
  public:
    struct ConfigParam {
      uint32_t id_feedback;
      uint32_t id_control;
    };

    struct Feedback {
      float rotor_abs_angle = 0.0f;
      float rotor_rotation_speed = 0.0f;
      float torque_current = 0.0f;
      float temp = 0.0f;
    };

    /**
     * @brief RMMotor 类的构造函数
     * @param container 指向父 RMMotorContainer 实例的指针
     * @param param 电机的基本参数（型号、是否反转）
     * @param config 电机的配置参数（反馈ID、控制ID）
     */
    RMMotor(RMMotorContainer* container, const Param& param,
            const ConfigParam& config)
      : param_(param), config_param_(config), container_(container) {
    }

    /**
     * @brief 解码来自CAN总线的电机反馈数据包
     * @param pack 包含电机反馈数据的CAN数据包
     */
    void Decode(LibXR::CAN::ClassicPack& pack) {
      uint16_t raw_angle = static_cast<uint16_t>(
        (pack.data[0] << 8) | pack.data[1]);
      int16_t raw_current = static_cast<int16_t>(
        (pack.data[4] << 8) | pack.data[5]);

      this->feedback_.rotor_abs_angle =
          LibXR::CycleValue<float>(raw_angle) / MOTOR_ENC_RES * M_2PI;
      this->feedback_.rotor_rotation_speed =
          static_cast<int16_t>((pack.data[2] << 8) | pack.data[3]);
      this->feedback_.torque_current =
          static_cast<float>(raw_current) * M3508_MAX_ABS_CUR / MOTOR_CUR_RES;
      this->feedback_.temp = pack.data[6];
    }

    /**
     * @brief 更新电机状态，处理接收队列中的所有反馈数据包
     * @return bool 总是返回 true
     */
    bool Update() {
      LibXR::CAN::ClassicPack pack;

      while (container_->recv_.Pop(pack) == ErrorCode::OK) {
        if ((pack.id == config_param_.id_feedback) &&
            (Model::MOTOR_NONE != this->param_.model)) {
          this->Decode(pack);
        }
      }

      return true;
    }

    float GetTorque() {
      switch (this->param_.model) {
        case Model::MOTOR_M2006:
          return 0.18f;
        case Model::MOTOR_M3508:
          return 0.3f;
        case Model::MOTOR_GM6020:
          return 0.6f;
        default:
          return 0.0f;
      }
    }

    float GetCurrentMAX() {
      switch (this->param_.model) {
        case Model::MOTOR_M2006:
          return M2006_MAX_ABS_CUR;
        case Model::MOTOR_M3508:
          return M3508_MAX_ABS_CUR;
        case Model::MOTOR_GM6020:
          return GM6020_MAX_ABS_CUR;
        default:
          return 0.0f;
      }
    }

    /**
     * @brief 根据电机型号获取电流控制指令的转换系数 (LSB)
     * @return float 返回对应型号的LSB值，若型号未知则返回0.0f
     */
    float GetLSB() {
      switch (this->param_.model) {
        case Model::MOTOR_M2006:
          return M2006_MAX_ABS_LSB;

        case Model::MOTOR_M3508:
          return M3508_MAX_ABS_LSB;

        case Model::MOTOR_GM6020:
          return GM6020_MAX_ABS_LSB;

        default:
          return 0.0f;
      }
    }

    float GetSpeed() {
      if (param_.reverse) {
        return -this->feedback_.rotor_rotation_speed;
      } else {
        return this->feedback_.rotor_rotation_speed;
      }
    }

    float GetCurrent() {
      if (param_.reverse) {
        return -this->feedback_.torque_current;
      } else {
        return this->feedback_.torque_current;
      }
    }

    float GetTemp() { return this->feedback_.temp; }

    /**
     * @brief 设置电机的电流控制指令
     * @details 将归一化的输出值转换为16位整数指令，存入共享发送缓冲区。
     *          当同一控制ID下的所有电机都更新指令后，触发CAN报文发送。
     * @param out 归一化的电机输出值，范围 [-1.0, 1.0]
     */
    void CurrentControl(float out) {
      if (this->feedback_.temp > 75.0f) {
        out = 0.0f;
        XR_LOG_WARN("motor %d high temperature detected", index_);
      }

      out = std::clamp(out, -1.0f, 1.0f);
      if (param_.reverse) {
        this->output_ = -out;
      } else {
        this->output_ = out;
      }
      float lsb = this->GetLSB();

      if (lsb != 0.0f) {
        int16_t ctrl_cmd = static_cast<int16_t>(this->output_ * lsb);
        motor_tx_buff_[this->index_][2 * this->num_] =
            static_cast<uint8_t>((ctrl_cmd >> 8) & 0xFF);
        motor_tx_buff_[this->index_][2 * this->num_ + 1] =
            static_cast<uint8_t>(ctrl_cmd & 0xFF);
        motor_tx_flag_[this->index_] |= 1 << (this->num_);

        if (((~motor_tx_flag_[this->index_]) &
             (motor_tx_map_[this->index_])) == 0) {
          this->SendData();
        }
      }
    }

    void TorqueControl(float out) {
      if (this->feedback_.temp > 75.0f) {
        out = 0.0f;
        XR_LOG_WARN("motor %d high temperature detected", index_);
      }

      float kt = this->GetTorque();
      float max_current = this->GetCurrent();
      float lsb = this->GetLSB();

      float torque = out / kt;
      float out_ = torque / max_current;

      out = std::clamp(out_, -1.0f, 1.0f);
      if (param_.reverse) {
        this->output_ = -out;
      } else {
        this->output_ = out;
      }

      if (lsb != 0.0f) {
        int16_t ctrl_cmd = static_cast<int16_t>(this->output_ * lsb);
        motor_tx_buff_[this->index_][2 * this->num_] =
            static_cast<uint8_t>((ctrl_cmd >> 8) & 0xFF);
        motor_tx_buff_[this->index_][2 * this->num_ + 1] =
            static_cast<uint8_t>(ctrl_cmd & 0xFF);
        motor_tx_flag_[this->index_] |= 1 << (this->num_);

        if (((~motor_tx_flag_[this->index_]) &
             (motor_tx_map_[this->index_])) == 0) {
          this->SendData();
        }
      }
    }

    /**
     * @brief 发送打包好的CAN控制报文
     * @details 从共享缓冲区拷贝数据到CAN包，通过CAN总线发送，并清零发送标志位。
     * @return bool 总是返回 true
     */
    bool SendData() {
      LibXR::CAN::ClassicPack tx_buff{};

      tx_buff.id = this->config_param_.id_control;
      tx_buff.type = LibXR::CAN::Type::STANDARD;

      memcpy(tx_buff.data, motor_tx_buff_[this->index_],
             sizeof(tx_buff.data));

      container_->can_->AddMessage(tx_buff);

      motor_tx_flag_[this->index_] = 0;

      return true;
    }

    /**
     * @brief 将电机反馈数据清零，用于电机离线状态
     */
    void Offline() {
      // 使用显式赋值替代 memset
      this->feedback_.rotor_abs_angle = 0.0f;
      this->feedback_.rotor_rotation_speed = 0.0f;
      this->feedback_.torque_current = 0.0f;
      this->feedback_.temp = 0.0f;
    }

    uint8_t index_;
    uint8_t num_;
    float output_ = 0.0f;

    Param param_;
    ConfigParam config_param_;
    Feedback feedback_;

  private:
    RMMotorContainer* container_;
  };

public:
  /**
   * @brief RMMotorContainer 类的构造函数
   * @details 初始化所有电机实例，并根据其型号和索引自动分配CAN ID和在报文中的位置。
   * @param hw LibXR::HardwareContainer 的引用，用于查找硬件资源
   * @param app LibXR::ApplicationManager 的引用
   * @param param_0 电机0的参数
   * @param param_1 电机1的参数
   * @param param_2 电机2的参数
   * @param param_3 电机3的参数
   * @param param_4 电机4的参数
   * @param param_5 电机5的参数
   * @param param_6 电机6的参数
   * @param param_7 电机7的参数
   * @param param_8 电机8的参数
   * @param param_9 电机9的参数
   * @param param_10 电机10的参数
   */
 RMMotorContainer(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                   Param param_0 = {Model::MOTOR_NONE, false},
                   Param param_1 = {Model::MOTOR_NONE, false},
                   Param param_2 = {Model::MOTOR_NONE, false},
                   Param param_3 = {Model::MOTOR_NONE, false},
                   Param param_4 = {Model::MOTOR_NONE, false},
                   Param param_5 = {Model::MOTOR_NONE, false},
                   Param param_6 = {Model::MOTOR_NONE, false},
                   Param param_7 = {Model::MOTOR_NONE, false},
                   Param param_8 = {Model::MOTOR_NONE, false},
                   Param param_9 = {Model::MOTOR_NONE, false},
                   Param param_10 = {Model::MOTOR_NONE, false})
    : can_(hw.template FindOrExit<LibXR::CAN>({"can1"})) {
    memset(motor_tx_map_, 0, sizeof(motor_tx_map_));
    size_t index = 0;
    for (const auto param : std::initializer_list<Param*>{
             &param_0, &param_1, &param_2, &param_3, &param_4, &param_5,
             &param_6, &param_7, &param_8, &param_9, &param_10}) {
      if (param->model != Model::MOTOR_NONE) {
        RMMotor::ConfigParam config{};
        uint8_t motor_num = 0;
        uint8_t motor_index = 0;

        switch (param->model) {
          case Model::MOTOR_M2006:
          case Model::MOTOR_M3508:
            if (index <= 3) {
              config.id_control = M3508_M2006_CTRL_ID_BASE;
              config.id_feedback = M3508_M2006_FB_ID_BASE + index;
            } else if (index >= 4 && index <= 7) {
              config.id_control = M3508_M2006_CTRL_ID_EXTAND;
              config.id_feedback = M3508_M2006_FB_ID_EXTAND + (index - 4);
            }
            break;
          case Model::MOTOR_GM6020:
            if (index >= 4 && index <= 7) {
              config.id_control = GM6020_CTRL_ID_BASE;
              config.id_feedback = GM6020_FB_ID_BASE + (index - 4);
            } else if (index >= 8) {
              config.id_control = GM6020_CTRL_ID_EXTAND;
              config.id_feedback = GM6020_FB_ID_EXTAND + (index - 8);
            }
            break;
          default:
            break;
        }

        switch (config.id_control) {
          case M3508_M2006_CTRL_ID_BASE:
            motor_index = 0;
            motor_num = config.id_feedback - M3508_M2006_FB_ID_BASE;
            break;
          case M3508_M2006_CTRL_ID_EXTAND:
            motor_index = 1;
            motor_num = config.id_feedback - M3508_M2006_FB_ID_EXTAND;
            break;
          case GM6020_CTRL_ID_BASE:
            motor_index = 2;
            motor_num = config.id_feedback - GM6020_FB_ID_BASE;
            break;
          case GM6020_CTRL_ID_EXTAND:
            motor_index = 3;
            motor_num = config.id_feedback - GM6020_FB_ID_EXTAND;
            break;
          default:
            break;
        }

        motor_tx_map_[motor_index] |= (1 << motor_num);

        motors_[index] = new RMMotor(this, *param, config);
        motors_[index]->index_ = motor_index;
        motors_[index]->num_ = motor_num;

      } else {
        // 如果电机模型为 NONE, 使用默认空配置创建
        RMMotor::ConfigParam empty_config{};
        motors_[index] = new RMMotor(this, *param, empty_config);
        motors_[index]->index_ = 0; // 赋予明确的默认值
        motors_[index]->num_ = 0;   // 赋予明确的默认值
      }

      index++;
    }

    motor_count_ = index;
    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, RMMotorContainer* self,
           const LibXR::CAN::ClassicPack& pack) {
          RxCallback(in_isr, self, pack);
        }, this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE,
                   M3508_M2006_FB_ID_BASE, M3508_M2006_FB_ID_EXTAND + 3);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE,
                   GM6020_FB_ID_BASE, GM6020_FB_ID_EXTAND + 2);
  }

  /**
   * @brief 通过索引获取电机实例的指针
   * @param index 要获取的电机的索引
   * @return RMMotor* 指向电机实例的指针，如果索引越界则返回 nullptr
   */
  RMMotor* GetMotor(size_t index) {
    if (index >= motor_count_) {
      return nullptr;
    }
    return motors_[index];
  }

  /**
   * @brief CAN 接收回调的静态包装函数
   * @details 将接收到的CAN数据包推入无锁队列中，供后续处理。如果队列已满，则丢弃最旧的数据包。
   * @param in_isr 指示是否在中断服务程序中调用
   * @param self 用户提供的参数，这里是 RMMotorContainer 实例的指针
   * @param pack 接收到的 CAN 数据包
   */
  static void RxCallback(bool in_isr, RMMotorContainer* self,
                         const LibXR::CAN::ClassicPack& pack) {
    while (self->recv_.Push(pack) != ErrorCode::OK) {
      self->recv_.Pop(); // 如果队列满了，尝试弹出一个包
    }
  }

  /**
   * @brief 监控函数，由应用框架周期性调用
   */
  void OnMonitor() override {
  }

private:
  RMMotor* motors_[11] = {};
  size_t motor_count_ = 0;
  LibXR::CAN* can_;

  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_ =
      LibXR::LockFreeQueue<LibXR::CAN::ClassicPack>(1);
};