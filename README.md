STM32 FreeRTOS Projects

A collection of STM32 microcontroller projects demonstrating FreeRTOS features, including tasks, queues, semaphores, mutexes, and round-robin scheduling.

📂 Projects Included
Project	Description
Project_1_Task	Basic FreeRTOS task creation and scheduling example.
Project_2_Queues	Single queue communication between tasks.
Project_3_Queue_Multiple_Users	Queue shared by multiple tasks with different priorities.
Project_4_Binary_Semaphore	Synchronization between tasks using binary semaphore.
Project_5_Counting_Semaphore	Demonstrates counting semaphore for resource management.
Project_6_Mutex	Mutex usage for preventing data corruption in shared resources.
Project_7_Round_Robin	Round-robin scheduling example with multiple tasks.
⚡ Features

FreeRTOS kernel integration on STM32F0 series.

Task creation, scheduling, and prioritization.

Inter-task communication via queues.

Task synchronization using binary and counting semaphores.

Resource protection using mutexes.

Demonstration of round-robin task scheduling.

Uses STM32 HAL libraries for hardware abstraction.

🛠️ Hardware Requirements

STM32F072RB microcontroller (other STM32F0 series may also work).

STM32CubeIDE for building and flashing projects.

ST-LINK debugger/programmer.

Optional: LEDs, buttons, and sensors for extended demos.

📦 Software Requirements

STM32CubeIDE

FreeRTOS (bundled with STM32CubeIDE or added manually)

STM32 HAL drivers (included in each project)

🚀 How to Run

Clone this repository:

git clone https://github.com/arjun123445678/stm32-freertos.git
cd stm32-freertos


Open the project folder in STM32CubeIDE.

Build the project.

Flash the firmware to your STM32F072RB.

Observe the results on LEDs, UART console, or peripherals depending on the example.

📝 Notes

Each project contains:

Core/Inc/ → Header files

Core/Src/ → Source files

Core/Startup/ → Startup assembly file

FreeRTOSConfig is customized per project for task priorities and system tick.

You can modify tasks, semaphores, or queues to experiment with real-time scheduling behavior.

🔗 References

FreeRTOS Official Documentation: https://www.freertos.org/

STM32 HAL Documentation : https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html


📌 License

This repository is MIT licensed – feel free to use and modify it for learning and experimentation.
