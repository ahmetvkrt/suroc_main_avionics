# SUROC Main Avionics (STM32F103C8T6)

**Purpose:**  
This project aims to develop the **main avionics firmware** for a rocket flight computer based on **STM32F103C8T6** (“BluePill”-class MCU).  
It integrates the following sensors and modules:

- ✅ **BME680** — environmental sensor (I²C) *(currently implemented)*
- 🧭 **BNO085** — 9-DoF IMU with fusion (SPI preferred, I²C fallback) *(to do)*
- 🛰 **Ultimate GPS v3** — UART + PPS *(to do)*

The firmware is structured for robustness, clarity, and easy collaboration. It separates **CubeMX-generated code** from **application code**, allowing safe regeneration without losing logic.

---

## 🧰 Current Status

| Module  | Transport             | Status       | Notes                                                 |
|---------|------------------------|-------------|-------------------------------------------------------|
| BME680  | I²C                   | ✅ Working  | Bosch `bme68x` driver; basic periodic sampling         |
| BNO085  | SPI (preferred) / I²C | ⏳ Planned  | INTN interrupt + DMA transport (SH-2/SHTP)            |
| GPS     | UART + PPS            | ⏳ Planned  | PPS timestamping with EXTI, UBX/NMEA message parsing |

---

## 📂 Repository Structure

```

suroc_main_avionics/
├─ Core/              # CubeMX generated code (keep your edits in USER CODE blocks)
├─ Drivers/           # HAL/CMSIS, versioned
├─ App/               # Application code
│  ├─ Inc/
│  │  ├─ board_config.h      # Pin map, bus speeds, feature flags
│  │  └─ app.h
│  │  └─ sensor headers…
│  └─ Src/
│     ├─ app.c               # app_init(), app_loop()
│     └─ bme68x_port.c, etc.
├─ third_party/
│  └─ bosch_bme68x/          # Vendor driver + license
├─ suroc_main_avionics.ioc   # CubeMX configuration
├─ STM32F103C8TX_FLASH.ld    # Linker script (memory layout)
├─ .project / .cproject      # IDE project files
└─ .gitignore
```

Other optional folders that can be added:
- `docs/` — wiring, timing, calibration procedures  
- `hardware/` — schematics/PCBs  
- `tools/` — host utilities, plotting, flashing scripts  
- `logs/` — small sample logs

---

## 🧭 Prerequisites

- **STM32CubeIDE** (recommended version: latest LTS or the version used for this repo)
- **STM32Cube MCU Packages** for STM32F1
- **ST-LINK** or equivalent SWD programmer
- Git

---

## 🪛 How to Clone and Open in STM32CubeIDE

1. **Clone the repository**
   ```bash
   git clone https://github.com/ahmetvkrt/suroc_main_avionics.git
   cd suroc_main_avionics
   ```

2. **Import into STM32CubeIDE**
   - Go to **File → Import… → Existing Projects into Workspace**
   - Browse to `firmware/suroc_main_avionics`
   - Select the project and click **Finish**

3. **Build the project**
   - Select the project in Project Explorer
   - Click **Project → Build Project** or press **Ctrl+B**

   *(CubeIDE will generate a local `Debug/` folder automatically — not tracked in Git.)*

4. **Flash the firmware**
   - Connect ST-LINK
   - Open **STM32CubeProgrammer** app
   - Find the built **.elf** file under **Debug** or **Release** folder and upload

5. **View serial output**
   - Use CubeIDE terminal or external tool (PuTTY, Serial Studio)
   - Default baud rate: check `board_config.h` (e.g., `115200`)

---

## ✅ First Run Checklist

- Power with **3.3 V** (no 5 V I/O!)
- BME680 has 2.2–4.7 kΩ pull-ups on SDA/SCL
- Verify BME680 I²C address in `board_config.h` (0x76 or 0x77)
- Build and flash → serial should display sensor readings

---

## ⚙️ Configuration

`App/Inc/board_config.h` controls:
- Pin mappings for SPI, I²C, UART, INT, RST, PPS
- Bus speeds (e.g. SPI 2–4 MHz, I²C 400 kHz)
- Feature flags (e.g. `USE_BNO085_SPI`, `USE_GPS_UART`)

> After modifying the `.ioc` file, regenerate code in CubeMX and rebuild.

---

## 🧠 Code Structure

- `Core/*` — Cube-generated init code (clocks, GPIO, DMA, etc.)
- `App/*` — application logic, sensor drivers, fusion, logging
- `third_party/*` — external drivers (e.g., Bosch BME68x)
- Regeneration-safe design: all your logic stays outside of Cube-generated code.

---

## 📡 Default Pinout (BluePill Example)

| Peripheral | Signal | Pin   | Notes                |
|------------|--------|-------|----------------------|
| BME680     | SDA/SCL | PB7/PB6 | I²C1, 400 kHz         |
| BNO085     | SPI1   | PA5/6/7 + PA4 (CS) | INTN PB0, RST PB1 |
| GPS        | UART2  | PA2 (TX), PA3 (RX) | PPS on PA8        |

Adjust in `board_config.h` as needed.

---

## 🧱 Regenerating Code Safely

1. Open `.ioc` in CubeMX
2. Edit pins / peripherals
3. **Project Manager → Code Generator**:
   - ☑ Keep User Code when re-generating
   - ☑ Generate peripheral init in separate .c/.h files
4. Click **Generate Code**
5. Rebuild

Commit `.ioc` and generated diffs together:
```bash
git add .
git commit -m "hw: add SPI1 for BNO085 and regenerate code"
```

---

## 🤝 Contributing

- Keep logic in `App/*`  
- Don’t modify Cube-generated code outside `USER CODE` blocks  
- Separate hardware commits (ioc/regens) from feature commits  
- Include wiring notes for new sensor work

---

## 📜 Licensing

- Own code (`App/*`) → Apache-2.0*  
- STMicroelectronics (`Core/*`, `Drivers/*`) → under ST license (kept in file headers)  
- Bosch BME68x → vendor BSD-like license in `third_party/bosch_bme68x/`

Documented third-party licenses in `NOTICE.md`.

---

## 🛠 Roadmap

- [ ] Implement BNO085 SPI transport + INTN (DMA, SH-2/SHTP)
- [ ] Implement GPS UART receiver with PPS timestamping
- [ ] Add unified telemetry logger (CSV/Binary)
- [ ] Add host visualization tool (`tools/host/`)
- [ ] Add fault handling & flight state machine

---

## 🆘 Support

If you encounter issues, please include:
- MCU / board revision
- Toolchain versions
- Wiring photo or schematic
- Serial output logs

---

✈ **Happy flying!**
