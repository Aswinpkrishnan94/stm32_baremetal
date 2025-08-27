# Smart Presence & Motion Monitoring

An **IoT solution** for real-time motion and presence detection, built with:
- **STM32F446RE + FreeRTOS** (Edge sensing & processing)
- **PIR motion sensor**

---

## 🛠 Hardware Setup
### Required Components
- **STM32F446RE Nucleo Board**
- **PIR Motion Sensor**
- Breadboard, jumper wires

**Sensor Connections:**
| Sensor        | STM32 Pin  | Notes |
|--------------|-----------|-------|
| PIR Motion   | PA0 (EXTI) | Interrupt-based |

---

## 📂 Project Structure

/firmware
├── Core/ # STM32CubeIDE source
├── FreeRTOS/ # Task and ISR management
├── drivers/ # HAL and BSP drivers

---

## 🚀 Getting Started

### 1️⃣ Firmware
1. Open firmware in **STM32CubeIDE**
2. Flash to STM32F446RE
3. Connect the sensor as per the wiring table

