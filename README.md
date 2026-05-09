# 🚗 Smart Parking System (Embedded C + RTOS)

## 📌 Overview

This project implements a **Smart Parking System** using embedded C on a TM4C microcontroller, developed with Keil uVision and powered by FreeRTOS.
The system manages parking slots efficiently, monitors availability, and provides real-time feedback using LEDs and sensors.

---

## 🎯 Features

* ✅ Real-time parking slot monitoring
* ✅ Multi-tasking using FreeRTOS
* ✅ LED indicators for slot status (Available / Occupied)
* ✅ Queue & semaphore-based task synchronization
* ✅ Efficient resource management
* ✅ Scalable embedded system design

---

## 🛠️ Technologies Used

* **Programming Language:** C
* **Microcontroller:** TM4C123 (Tiva C Series)
* **IDE:** Keil uVision
* **RTOS:** FreeRTOS
* **Hardware:** Sensors + LEDs

---

## 🧠 System Design

The system is built using multiple RTOS tasks:

* 🚘 **Entry Task:** Detects incoming cars
* 🚗 **Exit Task:** Updates available slots
* 💡 **Display Task:** Controls LED indicators
* 🔄 **Control Task:** Manages system logic

Synchronization is handled using:

* Queues
* Semaphores

---

## 📂 Project Structure

```
Smart-Parking-System/
│
├── Core/
│   ├── main.c
│   └── system.c
│
├── Drivers/
│   └── (MCU drivers)
│
├── FreeRTOS/
│   └── (RTOS source files)
│
├── Config/
│   └── (System configuration)
│
├── .gitignore
├── README.md
└── project.uvprojx
```

---

## ⚙️ How to Run

1. Open the project in **Keil uVision**
2. Build the project (`F7`)
3. Flash it to the TM4C board
4. Run and monitor LEDs behavior

---

## 📸 Demo (Optional)

> Add images or videos here showing the system working

---

## 🚀 Future Improvements

* Add LCD display for slot count
* Integrate mobile app / IoT monitoring
* Use ultrasonic sensors for accuracy
* Cloud-based parking management

---

## 👨‍💻 Author

**Mahmoud Nashaat**

* Fullstack .NET Developer
* Embedded Systems Enthusiast

---

## 📜 License

This project is open-source and available under the MIT License.

---

## ⭐ Support

If you like this project, consider giving it a ⭐ on GitHub!
