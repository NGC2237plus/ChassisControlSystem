# ChassisControlSystem
# 智能车底盘控制程序

| 工程                     | F407_Chassis2.0                    |
|--------------------------|-------------------------------------|
| 作者                     | 早上坏 (star32349@outlook.com)     |

## 详细描述
智能车底盘控制程序，根据串口数据控制移动。

## 功能描述
- **硬件**: STM32F407VET6
- **开发环境**: STM32 HAL库 + FreeRTOS
- **底盘**: 采用麦克纳姆轮
- **运动校正**: 陀螺仪校正运动误差

## 用法描述
- 具体串口协议请参考相关协议文档。
- [doxygen文件](http://htmlpreview.github.io/?https://github.com/NGC2237plus/ChassisControlSystem/blob/main/app/html/index.html)建议clone本地后打开

## 固件更新

| 日期        | 版本 | 作者                             | 描述                   |
|-------------|------|----------------------------------|------------------------|
| 2023-12-05  | 1.0  | 早上坏 (star32349@outlook.com)  | 初始版本               |
| 2024-04-01  | 2.0  | 早上坏 (star32349@outlook.com)  | 算法优化，代码完善     |

