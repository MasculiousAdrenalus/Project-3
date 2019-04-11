for part 2 change line 426 
from:
Noisy_GyroZ =ContextSimulation.GyroZ+ContextSimulation.stdDevGyro*randn(1) + deg2rad(1);
to:
Noisy_GyroZ =ContextSimulation.GyroZ+ContextSimulation.stdDevGyro*randn(1) - deg2rad(1);