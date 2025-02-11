# Postfault Operation Strategy for n-Phase CHB VSIs with Reduced Stator Copper Losses

This repository contains MATLAB source code demonstrating the postfault operation strategy for n-phase Cascaded H-Bridge Voltage Source Inverters (CHB VSIs) that dramatically reduces the stator copper losses by minimizing the injection of low-order xy harmonics in the stator voltage [1].

This work was supported by [MCIN/AEI/10.13039/501100011033/FEDER, UE](https://www.aei.gob.es/en) under Project [PID2021-124136OB-I00](http://olopez.webs.uvigo.es/pgc2021.html).

## Description

This MATLAB code implements the postfault control strategy described in [1].  The strategy focuses on minimizing low-order xy harmonics in the stator voltage during postfault operation of n-phase CHB VSIs, which leads to a significant reduction in stator copper losses.

## Source Code

The MATLAB source code is provided in the `src` directory.  Key files include:

* `[MXYFivePhases.m]` : The main script to run the simulation and demonstrate the postfault strategy. Running this file has no particular requirements. It includes comments that explain the code. The script produces three image files: "MXY_cold.png", "MXY_warm.png" and "MIN.png". 

## Installation

1. Clone the repository: `git clone https://github.com/olopezuvigo/CHB_MXY.git`
2. Add the `src` directory to your MATLAB path. You can do this programmatically within MATLAB:
   ```matlab
   addpath('/path/to/cloned/repository/src'); % Replace with the actual path

## References
[1]: Ó. López, J. Álvarez, A. G. Yepes, M. Medina-Sánchez and J. Doval-Gandoy, "Postfault Operation Strategy With Minimum Harmonic Injection for Cascaded H-Bridge Inverters Driving a Multiphase Motor," in IEEE Transactions on Power Electronics, vol. 40, no. 1, pp. 8-22, Jan. 2025, [doi: 10.1109/TPEL.2024.3456390](https://doi.org/10.1109/TPEL.2024.3456390)
