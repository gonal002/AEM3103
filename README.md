# Paper Airplane Numerical Study
  Final Project: AEM 3103 Spring 2024

  - By: Rafaela Goncalves da Silva

  ## Summary of Findings
  **Flight Trajectories Varying Initial Velocity:**
  |                | V_0 = 2 [m/s] | V_0 = 3.55 [m/s] | V_0 = 7.5 [m/s] |
  :---------------:|:-------------:|:----------------:|:--------------:
  | Max Height [m] |     2.00      |       2.00       |     3.58      
  | Max Range [m]  |     20.6      |       20.9       |     19.5     

   **Flight Trajectories Varying Flight Path Angle:**
  |                | Gam = -0.5 [rad] | Gam = -0.18 [rad] | Gam = 0.4 [rad] |
  :---------------:|:-------------:|:----------------:|:--------------:
  | Max Height [m] |       2.00       |       2.00        |     2.55      
  | Max Range [m]  |       21.1       |       20.9        |     20.3     

  - As seen above, as V and Gamma increases, max Height or maintains the same (start point) or gets higher. Now, for Range, the sweet spot (Max Range) is both at the nominal force and Gamma, and if you deviate from that, range will decrease.
  
  # Code Listing
  - [EqMotion.m](https://github.com/gonal002/AEM3103/blob/adf1484c6375420a543d85aa3ff47d8254ffd786/EqMotion.m) --> Fourth-Order Equations of Aircraft Motion
  - [PaperPlane.m](https://github.com/gonal002/AEM3103/blob/4e11cbc5770e34cbc52dfcc1889a9921612532f1/PaperPlane.m) --> Original code from study
  - [final_project.m](https://github.com/gonal002/AEM3103/blob/0aba278d9aaa581f2f4b4300cc929e7bbf92dc80/final_project.m) --> Adapted PaperPlane.m to calculate the desired simulations

  # Figures

  ## Fig. 1: Single Parameter Variation
  ![single_parameter_variation](https://github.com/gonal002/AEM3103/assets/167819730/d019e017-c8cf-4695-935c-72e6a452ae40)

  This figure contain 2 subplots of 2D Flight Trajectories. The top one shows how 3 different speeds affect trajectorie and lower one does the same thing, now varying Gamma.

  ## Fig. 2: Monte Carlo Simulation
  ![monte_carlo_simulation](https://github.com/gonal002/AEM3103/assets/167819730/b6077c43-7fc6-445d-ae6b-0d153bfc205d)

  This figure contains 100 plots of flight trjectories that vary both Gamma(Flight Path Angle) and Velocity in the bounds defined on the previous plot. It contains too a majenta plot that is a polynomial of 4th order fitted using the 'polyfit' MATLAB command. 

  ## Fig. 3: Time Derivatives 
  ![time_derivatives](https://github.com/gonal002/AEM3103/assets/167819730/9e6a9d83-55ac-4179-b143-50a16467fc3b)

 This is a graph of vertical and horizontal velocity of the polynomial fit from past calculation. Got it by taking the time derivative of both range and altitude values.
