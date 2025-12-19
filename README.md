# Portable-Wave-Maker
Portable Wave Maker is a low-cost, compact, open-source device for teaching and small-scale hydrodynamic experiments. It generates controlled waves with adjustable amplitude and frequency, supporting practical classes, laboratory tests, and academic projects.

🌊 Portable Wave Maker

Portable Wave Maker is a low-cost, compact, and open-source device designed for controlled wave generation in small-scale hydrodynamic experiments. The system is intended for use in practical classes, laboratory demonstrations, and experimental research in ocean and offshore engineering.

The project focuses on accessibility, reproducibility, and educational value, enabling students and researchers to study wave generation, propagation, and interaction with reduced-scale models without requiring complex infrastructure.

📌 Objectives

Support hands-on teaching of hydrodynamics through experimental demonstrations

Provide a low-cost and portable alternative to conventional wave tanks

Enable small-scale experimental research for TCCs, dissertations, and theses

Encourage replication, improvement, and collaboration through open-source sharing

⚙️ System Overview

The Portable Wave Maker consists of:

Mechanical wave paddle driven by a stepper motor

Linear guide system with low-friction bearings

Raspberry Pi–based control unit

Python software with graphical user interface (GUI)

Modular structure for easy assembly and transport

Wave parameters such as frequency, amplitude, and motion profile can be adjusted in real time.

🧠 Software

Language: Python

GUI: Tkinter

Features:

Sinusoidal motion with configurable ramp-up and ramp-down

Unit conversion from displacement (mm) to motor steps

Multithreading for responsive interface

Manual jog, homing, and emergency stop functions

🧩 3D Modeling & Fabrication

All mechanical components were modeled in Fusion360, including:

Full paddle and sliding support geometry

Interference checks between motor, electronics, and structure

Exploded views and technical drawings for assembly

The design prioritizes ease of fabrication using common machining or 3D printing methods.

📊 Simulations & Validation

ANSYS Fluent simulations for deep-water sinusoidal waves

Extraction of pressure and velocity fields for damping calibration

Python post-processing of experimental data:

Actuator displacement analysis

Frequency spectra extraction

Comparison with linear wave theory

Results were validated through laboratory tests and practical classes.
