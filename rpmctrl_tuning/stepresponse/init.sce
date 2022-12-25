clear()
clc()

measurementFile = "data/response2.csv" // Step response

encoderSteps = 30                      // Encoder ticks
sampleTime_sec = 0.2                   // Update period
ignorePhiRes = 0                       // Don't optimize for phase margin (T_d = 0)
overshoot_percent = 5                  // Maximum overshoot in %
dest_percent = 30                      // Step response destination in % for optimization
swing_sec = 1.0                        // Stabilization time in seconds
tolerance_percent = 10                 // Tolerance band in %

mprintf("***INIT***\n")
disp(measurementFile)
disp(encoderSteps)
disp(sampleTime_sec)
disp(dest_percent)
disp(overshoot_percent)

