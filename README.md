# RMMotorContainer 模块

## 模块描述
`RMMotorContainer` 是一个专为大疆系列电机（M3508, M2006, GM6020）设计的高级驱动模块。它通过 CAN 总线与电机通信，实现了对多达11个电机的统一、索引化管理。该模块自动处理底层CAN报文的打包和解析，并向上层应用提供简洁的控制接口，包括基于归一化电流的控制和基于物理单位（牛·米）的直接扭矩控制。

## 依赖硬件
*   CAN 总线 (在 `LibXR` 硬件容器中默认名称为 `can1`)
*  大疆系列电机（M3508, M2006, GM6020）
*  电调（C620，C610，GM6020自带电调）

## 构造参数
模块通过构造函数初始化，最多可配置11个电机。每个电机都需要一个 `Param` 结构体进行配置。

*   `Param`: 电机初始化参数结构体。
    *   `model`: 电机型号，从 `RMMotorContainer::Model` 枚举中选择。
    *   `reverse`: `bool` 类型，设置 `true` 将反转电机的运动方向和控制方向。

## 主要功能
*   **多电机统一管理**: 通过单个实例管理多达11个电机。
*   **索引化访问**: 使用从0开始的索引来访问和控制任意电机，无需关心其CAN ID。
*   **多型号支持**: 无缝支持 M3508, M2006, 和 GM6020 电机。
*   **双重控制模式**:
    *   `CurrentControl(float out)`: 按最大电流的百分比进行控制 (`[-1.0, 1.0]`)。
    *   `TorqueControl(float torque)`: 直接使用物理单位牛·米 (N·m) 进行精确扭矩控制。
*   **自动CAN报文分组**: 自动将具有相同控制ID的电机指令打包到同一个CAN报文中，高效利用总线带宽。
*   **实时数据反馈**: 解析并提供每个电机的实时数据，包括转子角度、转速、扭矩电流和温度。
*   **过热保护**: 当电机温度超过75℃时，自动停止对该电机的动力输出以防止损坏。

## 核心类与结构体
*   `RMMotorContainer`: 顶层管理类，负责与应用框架交互和电机集合管理。
*   `RMMotorContainer::RMMotor`: 代表单个电机的类，包含了控制和反馈逻辑。
*   `RMMotorContainer::Param`: 用于构造函数配置的结构体。
*   `RMMotor::Feedback`: 存储电机反馈数据的结构体。
    *   `rotor_abs_angle` (float): 转子绝对角度 (弧度)。
    *   `rotor_rotation_speed` (float): 转子转速 (RPM)。
    *   `torque_current` (float): 实际扭矩电流 (安培)。
    *   `temp` (float): 电机温度 (摄氏度)。

## 枚举说明
*   `enum class Model`: 定义了支持的电机型号。
    *   `MOTOR_NONE`: 空置，不代表任何电机。
    *   `MOTOR_M2006`: 代表 M2006 电机。
    *   `MOTOR_M3508`: 代表 M3508 电机。
    *   `MOTOR_GM6020`: 代表 GM6020 电机。

## CAN ID 分配与分组机制
模块内部根据电机型号和在构造函数中的顺序（索引）自动分配CAN的**反馈ID**和**控制ID**。

| 电机型号 | 电机索引 (从0开始) | 控制ID | 反馈ID范围 |
| :--- | :--- | :--- | :--- |
| M3508 / M2006 | 0 - 3 | `0x200` | `0x201` - `0x204` |
| M3508 / M2006 | 4 - 7 | `0x1ff` | `0x205` - `0x208` |
| GM6020 | 4 - 7 | `0x1fe` | `0x205` - `0x208` |
| GM6020 | 8 - 10 | `0x2fe` | `0x209` - `0x20B` |

**注意**: 构造函数中参数的顺序决定了电机的索引（从0开始）。请确保物理接线与软件配置的顺序一致。例如，第一个被配置为 `MOTOR_M3508` 的电机，其索引为0，将被分配反馈ID `0x201` 和控制ID `0x200`。

## 主要方法
#### RMMotorContainer
*   `RMMotor* GetMotor(size_t index)`: 通过索引获取 `RMMotor` 对象的指针。
*   `static void RxCallback(...)`: CAN接收回调函数，用于接收和处理电机反馈报文。

#### RMMotor
*   `void CurrentControl(float out)`: 设置电机输出，`out` 范围为 `[-1.0, 1.0]`，代表最大电流的百分比。
*   `void TorqueControl(float torque)`: 设置电机输出扭矩，`torque` 单位为牛·米 (N·m)。
*   `bool Update()`: 在循环中调用，用于处理该电机接收到的反馈数据。
*   `void Offline()`: 将电机置于离线状态，清空所有反馈数据。
*   `float GetSpeed()`: 获取转子转速 (RPM)，已处理反向设置。
*   `float GetCurrent()`: 获取扭矩电流 (安培)，已处理反向设置。
*   `float GetTemp()`: 获取电机温度 (摄氏度)。
*   `float GetTorque()`: 获取电机的扭矩常数 (**此函数中的反馈值为经验值，根据具体的数据调整，带减速比**)。
*   `float GetCurrentMAX()`: 获取电机的最大电流。
*   `float GetLSB()`: 获取电流控制指令的转换系数。
*   `const Feedback& feedback_`: (公开成员变量) 获取包含电机原始状态的 `Feedback` 结构体。