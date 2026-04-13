# Automated-Load-Transient-Testing-for-a-Power-distribution-network
The PDN Load Transient Test Script is a Python-based automated test program that controls four bench instruments over USB using the PyVISA library and SCPI (Standard Commands for Programmable Instruments) protocol.
It executes a defined load transient test sequence on each voltage rail of the Power Distribution Network, captures oscilloscope waveforms, extracts key transient metrics, logs all results to a CSV file and generates a structured test report.


This repo consists of a simulated test report, a simulated CSV file having time stamp, Overshoot, Undershoot, Recovery time, settling time, and a Pass/Fail for each of the mentioned coulumns. It also has some simulated CSV files representing the Time vs Volatge values of the capacitors at each output rails.

