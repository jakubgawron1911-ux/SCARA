# 🤖 SCARA Robot – Embedded System

System wbudowany dla robota typu SCARA zaprojektowany i zaimplementowany w ramach pracy inżynierskiej.

Projekt obejmuje kompletny system mechatroniczny integrujący:
- konstrukcję mechaniczną,
- elektronikę,
- oprogramowanie sterujące,
- komunikację z chmurą IoT.

---

## 🎥 Demo projektu

<p align="center">
  <a href="https://www.youtube.com/watch?v=6avzCS4PHWA">
    <img src="https://img.shields.io/badge/▶️_Zobacz_demo_na_YouTube-red?style=for-the-badge&logo=youtube" />
  </a>
</p>

---

## 📌 Opis projektu

Celem projektu było stworzenie systemu wbudowanego do sterowania robotem typu SCARA (Selective Compliance Assembly Robot Arm), umożliwiającego:

- precyzyjne pozycjonowanie manipulatora,
- sterowanie chwytakiem,
- komunikację bezprzewodową (Wi-Fi),
- zdalne sterowanie przez chmurę,
- implementację zaawansowanych algorytmów sterowania.

Robot został zaprojektowany jako platforma do dalszych badań nad:
- algorytmami sterowania,
- filtracją danych (filtr Kalmana),
- automatyzacją procesów.

---

## 🏗️ Architektura systemu

### 🔧 Hardware

- **Mikrokontroler:** STM32 F072RB (Nucleo)
- **Moduł komunikacyjny:** Raspberry Pi Pico 2 W (Wi-Fi)
- **Sterowniki silników:**
  - TMC2209 (silniki krokowe)
  - DRV8835 (silnik DC – chwytak)
- **Czujniki:**
  - AS5600 – enkodery magnetyczne
  - VL53L0X – czujnik odległości (ToF)
  - INA226 – pomiar prądu
- **Zasilanie:** 24V DC + przetwornice step-down

---

### 🧠 Software

#### STM32 (C)
- sterowanie silnikami krokowymi (PWM + STEP/DIR),
- implementacja regulatorów,
- filtr Kalmana,
- obsługa czujników (I2C),
- komunikacja UART + DMA + CRC.

#### Raspberry Pi Pico W (MicroPython)
- komunikacja MQTT (Blynk Cloud),
- konwersja danych MQTT → UART,
- budowa ramek z CRC32,
- funkcja bramy IoT (gateway).

---

## ☁️ Komunikacja

System działa w architekturze trójwarstwowej:

1. **STM32** – sterowanie niskopoziomowe
2. **Pico W** – brama IoT (UART ↔ MQTT)
3. **Blynk Cloud** – interfejs użytkownika

Funkcjonalności:
- sterowanie pozycją (slider),
- sterowanie chwytakiem,
- monitoring statusu (LED-y),
- komunikacja w czasie rzeczywistym.

---

## 🎯 Funkcjonalności

- ✔️ Sterowanie 3 osiami robota SCARA  
- ✔️ Sterowanie chwytakiem z kontrolą siły (pomiar prądu)  
- ✔️ Filtracja pomiarów (filtr Kalmana)  
- ✔️ Generowanie trajektorii (S-curve)  
- ✔️ Komunikacja MQTT (IoT)  
- ✔️ Zdalne sterowanie przez aplikację  
- ✔️ Detekcja błędów transmisji (CRC32)  

---

## 📊 Sterowanie

Zastosowane elementy sterowania:

- regulator ze sprzężeniem od stanu,
- filtr Kalmana (estymacja pozycji i prędkości),
- trajektoria typu S-curve (płynny ruch),
- model obiektu wyznaczony eksperymentalnie.

---

## 🧪 Badania

Projekt obejmował testy:

- dokładności pozycjonowania osi liniowej,
- dokładności pomiaru kąta,
- detekcji obciążenia chwytaka,
- stabilności układu sterowania.

---

## 🛠️ Technologie

- C (STM32)
- MicroPython (Pico W)
- MATLAB / Simulink
- MQTT / TLS
- Blynk IoT
- EasyEDA (PCB)
- Fusion 360 (CAD)

---

## 📷 Konstrukcja

Robot został wykonany z użyciem:
- elementów drukowanych 3D,
- pasków GT2,
- prowadnic liniowych,
- silników krokowych,
- łożysk precyzyjnych.

---

## 🚀 Możliwe rozszerzenia

- sterowanie odwrotną kinematyką,
- wizja komputerowa,
- ROS / ROS2,
- automatyczne planowanie trajektorii,
- sterowanie adaptacyjne.

---

## 👨‍🎓 Autor

**Jakub Gawron**  
Kierunek: Automatyka i Robotyka  
Rok: 2026  

---

## 📄 Licencja

Projekt edukacyjny – do wykorzystania w celach naukowych i rozwojowych.
