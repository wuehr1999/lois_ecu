raw = csvRead(measurementFile, [], [], 'double') // Load data

tLeft = []
rpmLeft = []
tRight = []
rpmRight = []

dutycycleLeft = raw(1, 1)
dutycycleRight = raw(1, 2)

/**
 @brief Interpolate first y-Value that reaches a threshold
 @param vecX x-coordinates
 @param vecY y-coordinates
 @param threshHold 
 @retval x x-coordinate von threshHold
 @retval y y-coordinate von threshHold
 **/
function [x, y] = interpolateThreshhold(vecX, vecY, threshHold)
     maxY = max(vecY)

    for m = 1:size(vecY)(1)
        if vecY(m) >= threshHold * maxY
            x = [vecX(m - 1), vecX(m)]
            y = [vecY(m -1), vecY(m)]
            xx = linspace(vecX(m -1), vecX(m), 100)
            yy = linear_interpn(xx, x, y, "natural")
            [y ix] = min(abs(threshHold * maxY - yy))
            y = yy(ix)
            x = xx(ix)
            break
        end
    end      
endfunction

/**
 @brief Approximate 1st order system
 @param vecT_sec  Time axis
 @param vecY      RPM values
 @param stepRatio Stepresponse amplitude
 @retval g        Transfer function
 @retval params_t Coefficients of transfer function
 **/
function [g, params_t] = approximatePT1(vecT_sec, vecY, stepRatio)
    params_t = struct('Kp', 'T1')
    [t632 y632] = interpolateThreshhold(vecT_sec, vecY, 0.632)
    
    params_t.Kp = max(vecY) / stepRatio
    params_t.T1 = t632
    
    num = poly([params_t.Kp],'s','coeff');
    den = poly([1 params_t.T1],'s','coeff');
    g = syslin('c', num/den)
    
endfunction

/**
 @brief Approximate 2nd order system
 @param vecT_sec  Time axis
 @param vecY      RPM values
 @param stepRatio Stepresponse amplitude
 @retval g        Transfer function
 @retval params_t Coefficients of transfer function
 **/
function [g, params_t] = approximatePT2(vecT_sec, vecY, stepRatio)
    params_t = struct('Kp', 'T1', 'T2', 'N')
    [t25, y25] = interpolateThreshhold(vecT_sec, vecY, 0.25)
    [t75, y75] = interpolateThreshhold(vecT_sec, vecY, 0.75)
    
    params_t.Kp = max(vecY) / stepRatio
    
    params_t.T1 = t25
    params_t.T2 = t75
    
    num = poly([params_t.Kp],'s','coeff');
    den = poly([1 (params_t.T1 + params_t.T2) params_t.T1 * params_t.T2],'s','coeff');
    g = syslin('c',num/den)
    
endfunction

// Preprocess dataset
for m = 2:size(raw)(1)
    if m < 3
        tLeft(m - 1) = 0;
        rpmLeft(m - 1) = 0;
        tRight(m - 1) = 0;  
        rpmRight(m - 1) = 0;    
    else
        tLeft(m - 1) = tLeft(m - 2) + raw(m, 1);
        rpmLeft(m - 1) = 60 / (raw(m, 1) * encoderSteps / 1000 / 1000);
        tRight(m - 1) = tRight(m - 2) + raw(m, 2);
        rpmRight(m - 1) = 60 / (raw(m, 2) * encoderSteps / 1000 / 1000);
    end
end

tLeft = tLeft ./ 1000 ./ 1000;
tRight = tRight ./ 1000 ./ 1000;

// Approximation
[gLeftPT1 sysLeftPT1] = approximatePT1(tLeft, rpmLeft, dutycycleLeft)
[gRightPT1 sysRightPT1] = approximatePT1(tRight, rpmRight, dutycycleRight)
[gLeftPT2 sysLeftPT2] = approximatePT2(tLeft, rpmLeft, dutycycleLeft)
[gRightPT2 sysRightPT2] = approximatePT2(tRight, rpmRight, dutycycleRight)

// Calculate system stepresponse
len = size(tLeft)(1)
tsLeft = 0:0.05:tLeft(len - 1);
gsLeftPT1 = csim('step', tsLeft, 100 * gLeftPT1)
gsLeftPT2 = csim('step', tsLeft, 100 * gLeftPT2)
len = size(tRight)(1)
tsRight = 0:0.05:tRight(len - 1);
gsRightPT1 = csim('step', tsRight, 100 * gRightPT1)
gsRightPT2 = csim('step', tsRight, 100 * gRightPT2)

figure(1)
clf()
subplot(121)
plot(tLeft, rpmLeft, 'r-')
plot(tsLeft, gsLeftPT1, 'g-')
plot(tsLeft, gsLeftPT2, 'b-')
xlabel('time [sec]')
ylabel('[rpm]')
xgrid()
legend(['measurement', 'PT1', 'PT2'])
title(msprintf('step response left to %03d %%', dutycycleLeft))

subplot(122)
plot(tRight, rpmRight, 'r-')
plot(tsRight, gsRightPT1, 'g-')
plot(tsRight, gsRightPT2, 'b-')
xgrid()
legend(['measurement', 'PT1', 'PT2'])
xlabel('time [sec]')
ylabel('[rpm]')
title(msprintf('step response right to %03d %%', dutycycleRight))

clear(['P' 'T1' 'T2' 'Tg' 'Tu' 'ans' 'dutycycleLeft' 'dutycycleRight' 'len' 'm' 'raw' 'rpmLeft' 'rpmRight' 'tLeft' 'tRight' 'tsLeft' 'tsRight'])

mprintf("***Left SISO approx***\n")
mprintf("PT1\n")
disp(gLeftPT1)
mprintf("\nPT2\n")
disp(gLeftPT2)
mprintf("\n\n\n***Right SISO approx***\n")
mprintf("PT1\n")
disp(gRightPT1)
mprintf("\nPT2\n")
disp(gRightPT2)
