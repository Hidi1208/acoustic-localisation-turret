# Real-Time Acoustic Localization System

## Overview

This is a real-time acoustic localization system designed to detect impulsive sound events (e.g., claps or gunshots) and orient a pan-tilt mechanism toward the source.

The system leverages a distributed **4-microphone array (INMP441)** across two ESP32 microcontrollers and performs **3D spatial localization** using a combination of amplitude-based estimation and Time Difference of Arrival (TDOA) analysis.

---

## Key Features

* Real-time acoustic event detection with adaptive noise gating
* 3D sound source localization using weighted vector methods
* TDOA-based refinement for improved angular accuracy
* Distributed sensing using dual ESP32 nodes with ESP-NOW communication
* Low-latency servo actuation for directional tracking
* Dynamic baseline noise calibration for robustness

---

## Hardware Configuration

* **Microcontrollers:** 2 × ESP32
* **Sensors:** 4 × INMP441 MEMS microphones
* **Actuation:** 2 × SG90 servo motors (pan and tilt)
* **Communication:** ESP-NOW (peer-to-peer wireless protocol)

---

## System Architecture

### Master Node (ESP32)

* Acquires audio from Mic1 and Mic2 via I2S
* Receives processed data from the slave node
* Performs localization and direction estimation
* Controls pan and tilt servos

### Slave Node (ESP32)

* Acquires audio from Mic3 and Mic4
* Computes peak amplitudes and timestamps
* Transmits data to the master node via ESP-NOW

---

## Methodology

### Signal Acquisition

Audio signals are sampled at 44.1 kHz using the I2S interface, ensuring high temporal resolution for localization.

### Noise Modeling and Filtering

Each microphone maintains a dynamically updated baseline representing ambient noise. Incoming signals are compared against a noise threshold to distinguish valid acoustic events.

### Peak Detection

For each buffer window, the maximum amplitude and corresponding timestamp are extracted to characterize the acoustic event.

### Energy Estimation

The effective signal energy is computed relative to the baseline:

E = max(0, amplitude − baseline)

### Gain Calibration

Due to hardware and communication inconsistencies, signals from Mic3 and Mic4 are scaled using empirically derived gain factors to align with Mic1 and Mic2.

### 3D Localization

Each microphone is associated with a predefined unit direction vector. The source direction is estimated using a weighted centroid approach:

D = (Σ Ei · Di) / Σ Ei

where Ei represents the energy contribution of each microphone and Di represents its direction vector.

### Angle Extraction

The resulting 3D direction vector is converted into:

* **Pan (azimuth angle)**
* **Tilt (elevation angle)**

### TDOA Refinement

Cross-correlation between Mic1 and Mic2 is used to compute time delay, refining horizontal localization accuracy.

---

## Dataset

* Approximately 3500 samples were collected for initial system calibration and validation
* A smoothed amplitude dataset is included in this repository
* Dataset limitations contributed to reduced model generalization

---

## Challenges and Limitations

* Gain mismatch between microphones across different ESP32 nodes
* Synchronization constraints in distributed acquisition
* Limited dataset size affecting model accuracy
* Initial servo response latency

---

## Implemented Improvements

* Separation of gain scaling from noise gate logic
* Empirical calibration of microphone gains
* Transition to 3D vector-based localization
* Direct servo actuation for improved responsiveness

---

## Future Work

* Collection of larger and more diverse datasets (≥5000 samples)
* Integration of machine learning models for enhanced localization
* Improved synchronization mechanisms between nodes
* Fusion with vision-based tracking systems

---

## Repository Structure

```
firmware/
  master/
  slave/

data/
```

---

## Demonstration

The system detects an acoustic event and dynamically orients the pan-tilt mechanism toward the estimated source direction in real time.

---

## Team

Developed by **Team SIXSEVEN** for SELECT Makeathon 2026

---

## License

This project is released under the MIT License.
