# STM32 Microcomputer Lab

本仓库整理了浙江大学光电信息科学与工程专业《微机原理与接口技术》(教改班)课程中的四个硬件实验项目.(以及我暑假自己学习时的一些练手项目).

由于老师提供的例程主要基于LL开发,和我之前学习的不太适应,并且老旧的Keil5也实在不太符合当今的时代,所以我改为通过STM32CubeMX与CLion开发环境完成,完全基于HAL库开发.

前面01-16是基于正点原子的STM32F103C8T6的开发,后面F04-F07是课程的硬件实验.

## 实验目录

- 01_FlashingLED:LED闪烁控制实验
- 02_Buzzer:蜂鸣器输出控制实验
- 03_Button:按键检测与处理实验
- 04_EXTI:外部中断检测实验
- 05_USART_Polling:USART轮询收发实验
- 06_USART_IT:USART中断收发实验
- 07_IWDG:独立看门狗定时器实验
- 09_BTIM:基本定时器BTIM使用实验
- 10_GTIM:通用定时器GTIM使用实验
- 11_GTIM_IC:通用定时器输入捕获实验
- 12_ATIM:高级定时器ATIM使用实验
- 13_OLED:OLED显示模块实验
- 15_RTC:实时时钟RTC实验
- 16_START:课程项目起步实验
- F00_TEST: 请忽略
- F01_BUTTON_LED: 请忽略
- F02_CLION_MX_TEST: 请忽略
- F03_MX_CLION_TEST: 使用CLion + Cubemx开发测试
- F04_EXTI: 微机原理课程硬件实验一
- F05_TIM: 微机原理课程硬件实验二
- F06_KEY: 微机原理课程硬件实验三
## 开发环境(F字头实验)

- 开发板:STM32F446RE Nucleo-64
- IDE:CLion + STM32CubeMX
- 编译器:Arm GCC
- 调试工具:OpenOCD + ST-Link

## 许可证

本项目采用 [MIT License](LICENSE) 授权，欢迎学习、使用和修改，但请保留作者署名。
