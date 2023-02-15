/**
 @brief Optimize for 1st order plant
 @param sys            Plant parameters
 @param sampletime_sec Controller update period in seconds
 @param phi_res        Goal phase margin
 @param wd             Goal crossover angular frequency
 @retval gPI           Closed loop system function
 @retval gO            Open loop system function
 @retval params_t      Controller parameters
 @retval phi           Actual phase margin
 **/
function [gPI, gO, params_t, phi] = tunePT1(sys, sampletime_sec, phi_res, wd)
    params_t = struct('Kp', 'Tn', 'Td', 'Ka', 'wd', '')
    
    params_t.Tn = sys.T1

    phi = ((-180 + phi_res) / 180) * %pi
    params_t.wd = wd

    if ignorePhiRes == 0 then
        params_t.Td = abs(tan(-0.5*%pi - phi) / params_t.wd)
    else
        params_t.Td = 0
    end
   
    phi = ((90 / 180) * %pi - atan(params_t.wd * params_t.Td)) / %pi * 180
    
    params_t.Kp = (params_t.wd * params_t.Tn * sqrt(1 + params_t.wd^2*params_t.Td^2)) / sys.Kp
    params_t.Ka = 1 / (2 * params_t.Kp)// * sys.Kp)
    num = poly([params_t.Kp, params_t.Kp*params_t.Tn],'s','coeff')
    den = poly([0 params_t.Tn params_t.Td*params_t.Tn],'s','coeff')
    gPI = syslin('c', num/den)
    
    num = poly([sys.Kp],'s','coeff')
    den = poly([1 sys.T1],'s','coeff')
    gO = gPI * syslin('c', num/den)  
      
endfunction

/**
 @brief Optimize for 2nd order plant
 @param sys            Plant parameters
 @param sampletime_sec Controller update period in seconds
 @param phi_res        Goal phase margin
 @param wd             Goal crossover angular frequency
 @retval gPI           Closed loop system function
 @retval gO            Open loop system function
 @retval params_t      Controller parameters
 @retval phi           Actual phase margin
 **/
function [gPI, gO, params_t, phi] = tunePT2(sys, sampletime_sec, phi_res, wd)
    params_t = struct('Kp', 'Tn', 'Td', 'Ka', 'wd', '')

    if sys.T1 < sys.T2
        params_t.Tn = sys.T2
        T3 = sys.T1
    else
        params_t.Tn = sys.T1
        T3 = sys.T2
    end
        
    phi = ((-180 + phi_res) / 180) * %pi
    params_t.wd = wd  
    
    if ignorePhiRes == 0 then
        params_t.Td = abs(tan(-0.5*%pi - atan(T3) - phi) / params_t.wd)
    else
        params_t.Td = 0
    end
    phi = ((90 / 180) * %pi - atan(params_t.wd * params_t.Td) - atan(params_t.wd * T3)) / %pi * 180
    
    params_t.Kp = (params_t.wd*params_t.Tn * sqrt(1 + params_t.wd^2*params_t.Td^2) * sqrt(1 + params_t.wd^2*T3^2)) / sys.Kp
    params_t.Ka = 1 / (2 * params_t.Kp)// * sys.Kp)
        
    num = poly([params_t.Kp, params_t.Kp*params_t.Tn],'s','coeff')
    den = poly([0 params_t.Tn params_t.Td*params_t.Tn],'s','coeff')
    gPI = syslin('c', num/den)
    
    num = poly([sys.Kp],'s','coeff')
    den = poly([1 (sys.T1 + sys.T2) sys.T1 * sys.T2],'s','coeff');
    gO = gPI * syslin('c', num/den)   
    
endfunction

/**
 @brief Diskretize using trapezoidal rule
 @param ctrl            Controller parameters
 @param sampletime_sec  Controller update period in seconds
 @retval coefficients_t Discrete filter coefficients
 **/
function [coefficients_t] = digitize(ctrl, sampletime_sec)
    coefficients_t = struct('d0', 'd1', 'd2', 'c1', 'c2', 'T')
    T = sampletime_sec
    coefficients_t.d0 = ctrl.Kp * ((ctrl.Tn + T/2) * (0 + T/2)) / (ctrl.Tn * (ctrl.Td + T/2))
    coefficients_t.d1 = -2 * ctrl.Kp * (ctrl.Tn * 0 - (T/2)^2) / (ctrl.Tn * (ctrl.Td + T/2))
    coefficients_t.d2 = ctrl.Kp * ((ctrl.Tn - T/2) * (0 - T/2)) / (ctrl.Tn * (ctrl.Td + T/2))
    coefficients_t.c1 = (2 * ctrl.Td) / (ctrl.Td + T/2)
    coefficients_t.c2 = -(ctrl.Td - T/2) / (ctrl.Td + T/2)
    coefficients_t.T = T
endfunction

d = (-log(overshoot_percent/dest_percent))/(sqrt(%pi^2 + log(overshoot_percent/dest_percent)^2))
e = sqrt(sqrt(4*d^4+1)-2*d^2)
wd = (1/swing_sec*d)*log(100/(0.5*tolerance_percent*sqrt(1-d^2)))*e // Calculate desired crossover frequency
phiRes_deg = 90 - (180 / %pi)*atan((1/2 * 1/d)*e) // Calculate desired phase margin

// Optimieren
[ctrlLeftPT1, gOLeftPT1, ctrlParamsLeftPT1, phiLeftPT1] = tunePT1(sysLeftPT1, sampleTime_sec, phiRes_deg, wd)
[ctrlLeftPT2, gOLeftPT2, ctrlParamsLeftPT2, phiLeftPT2] = tunePT2(sysLeftPT2, sampleTime_sec, phiRes_deg, wd)
[ctrlRightPT1, gORightPT1, ctrlParamsRightPT1, phiRightPT1] = tunePT1(sysRightPT1, sampleTime_sec, phiRes_deg, wd)
[ctrlRightPT2, gORightPT2, ctrlParamsRightPT2, phiRightPT2] = tunePT2(sysRightPT2, sampleTime_sec, phiRes_deg, wd)

// Diskretisieren
digLeftPT1 = digitize(ctrlParamsLeftPT1, sampleTime_sec)
digLeftPT2 = digitize(ctrlParamsLeftPT2, sampleTime_sec)
digRightPT1 = digitize(ctrlParamsRightPT1, sampleTime_sec)
digRightPT2 = digitize(ctrlParamsRightPT2, sampleTime_sec)

mprintf("***Left System PI ctrl***\n")
mprintf("PT1\n")
mprintf("Phase margin:")
disp(phiLeftPT1)
disp(ctrlLeftPT1)
disp(ctrlParamsLeftPT1)
mprintf("PT2\n")
mprintf("Phase margin:")
disp(phiLeftPT2)
disp(ctrlLeftPT2)
disp(ctrlParamsLeftPT2)
mprintf("digital")
disp(digLeftPT1)
disp(digLeftPT2)
mprintf("***Right System PI ctrl***\n")
mprintf("PT1\n")
mprintf("Phase margin:")
disp(phiRightPT1)
disp(ctrlRightPT1)
disp(ctrlParamsRightPT1)
mprintf("PT2\n")
mprintf("Phase margin:")
disp(phiRightPT2)
disp(ctrlRightPT2)
disp(ctrlParamsRightPT2)
mprintf("digital")
disp(digRightPT1)
disp(digRightPT2)
