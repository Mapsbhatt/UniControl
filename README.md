# 🧠 UniControl – Gyro & Pressure-Based Controller for Accessibility

**A low-latency, sensor-fused controller system for diplegic patients that converts head and foot gestures into computer inputs.**

---

## 🚀 Project Overview

UniControl is a hybrid controller integrating:
- **Gyroscope-based head movements** (for direction)
- **Foot pressure sensors** (for discrete actions)

It maps real-time gestures to **keyboard and mouse inputs**, enhancing accessibility and enabling even gaming or interactive control.

---

## 💡 Key Features
- Cross-platform: Supports macOS and Windows
- Real-time precision mapping with low latency
- Fully customizable control mapping
- Modular and Arduino-compatible design

---

## 🛠 Tech Stack
- Arduino R4 Minima + MPU6050
- Foot pressure sensors (FSRs)
- Python Serial Communication
- OpenCV (for optional visual feedback)

---

## 🛠️ Hardware Notes

- **MPU6050**:  
  Add external 10kΩ pull-up resistors on the **SDA** and **SCL** lines to ensure I2C signal integrity — especially important when using longer wire lengths.

- **FSRs (Force-Sensitive Resistors)**:  
  Connected via **voltage dividers**. Adjust the fixed resistor value based on required force sensitivity range.  
  Example: Use 10kΩ for moderate sensitivity; lower values for higher sensitivity.

> ⚠️ Make sure to debounce sensor readings and filter noise (moving average or low-pass filtering recommended).
