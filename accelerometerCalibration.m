close all 
clc
clear all

%% On board accelerometer calibration
file = load("zaccCalibration.txt", "-ascii");
file1 = load("xyaccCalibration.txt", "-ascii");

onBoardXacc = file1(:,1);
onBoardYacc = file1(:,2);
onBoardZacc = file(:,3);

[gAccX, gAccXTh] = calibrateAxis(onBoardXacc);
[gAccY, gAccYTh] = calibrateAxis(onBoardYacc);
[gAccZ, gAccZTh] = calibrateAxis(onBoardZacc);

[cx,gof,fx] = fit(gAccX,gAccXTh,"poly1");
[cy,gof,fy] = fit(gAccY,gAccYTh,"poly1");
[cz,gof,fz] = fit(gAccZ,gAccZTh,"poly1");


figure
subplot(1,3,1)
scatter(gAccX, gAccXTh, 'r')
hold on
plot(cx, 'b')
title('Ax Calibration')
l1 = legend('ax', extractFunction(cx))
fontsize(l1, 14, 'points')

subplot(1,3,2)
scatter(gAccY, gAccYTh, 'r')
hold on
plot(cy, 'b')
title('Ay Calibration')
l1 = legend('ay', extractFunction(cy))
fontsize(l1, 14, 'points')

subplot(1,3,3)
scatter(gAccZ, gAccZTh, 'r')
hold on
plot(cz, 'b')
title('Az Calibration')
l1 = legend('az',extractFunction(cz))
fontsize(l1, 14, 'points')

function [gAcc, gTh]  = calibrateAxis(accData)

    gMinus = accData(accData <= -9.71 & accData >= -9.91);
    gPlus = accData(accData >= 9.71 & accData <= 9.91);

    gPositive = ones(length(gPlus),1);
    gNegative = -1 * ones(length(gMinus),1);
    
    gAcc = cat(1, gMinus, gPlus);
    gTh = cat(1, gNegative, gPositive);
    gAcc = gAcc./9.81;
end

function [eq] = extractFunction(c)
    if(c.p2 > 0)
        eq = "y = " + c.p1 + "x + " + c.p2
    else
        eq = "y = " + c.p1 + "x - " + c.p2
    end
    
end


