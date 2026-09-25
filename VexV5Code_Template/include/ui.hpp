#pragma once
/**
 * @brief 打印电机数据函数。显示电机温度和连接状态。
 *
 */
void printMotorData();

/**
 * @brief 主控UI界面线程函数。在此函数中，主控UI界面会不断更新显示电机的温度和安装状态等信息。
 *
 */
void ui_task();