clc
clear all
close all

dataNames = ['OnBoard xAcc','OnBoard yAcc','OnBoard zAcc','Yaw','Pitch','Roll','OffBoard xAcc','OffBoard yAcc','OffBoard zAcc'];
file = load("drinkTest12.txt", "-ascii");

onBoardXacc = file(:,1);
onBoardYacc = file(:,2);
onBoardZacc = file(:,3);

yaw = file(:,4);
pitch = file(:,5);
roll = file(:,6);

offBoardXacc = file(:,7)./9.8;
offBoardYacc = file(:,8)./9.8;
offBoardZacc = file(:,9)./9.8;

nSamples = length(file);
samplingFreq = 20;
ts = 1/samplingFreq;
t = (1:nSamples).*ts;

p1 = atand(onBoardZacc./(sqrt(onBoardXacc.^2+onBoardYacc.^2)));

%% Activity
actF = [];
t1 = [];

act = 0;
cnt = 0;
lim = floor(length(t)/samplingFreq);
while cnt < lim
    for i=1:samplingFreq - 1
        act = act + abs(sqrt(onBoardXacc(20 * cnt + i)^2 + onBoardYacc(20*cnt + i)^2 + onBoardZacc(20*cnt + i)^2) - 1);
    end
    actF = [actF, act];
    cnt = cnt + 1;
    t1 = [t1, cnt];
    act = 0;
end


figure(1)
plot(t1, actF, 'black', 'LineWidth',1.5)
hold on
plot(t, onBoardZacc, 'r', t, onBoardYacc, 'b', t, onBoardXacc, 'g')
legend('dm', 'ax', 'ay', 'az')
title('Activity vs On-Board Accelerometer')
ylabel('G')
xlabel('Time (s)')
xlim([0 30])
hold off

%% Subplotting
%upper plot ax ay az
%lower plot dm and activity line and marker 'ro'
lim = floor(length(t)/samplingFreq)
act1 = [];
for i=1:length(offBoardXacc)
    da = sqrt(onBoardXacc(i)^2 + onBoardYacc(i)^2 + onBoardZacc(i)^2) - 1 - 0.0106;
    act1 = [act1, da];
end

activity = [];
t1 = [];

for i=0:lim-1
    tx = 0;
    for j=1:20
        tx = tx + abs(act1(20*i + j));
    end
    activity = [activity, tx];
    t1 = [t1, i];
end

t1 = t1 + 1

figure
subplot(2,1,1)
plot(t, onBoardZacc, 'r', t, onBoardYacc, 'b', t, onBoardXacc, 'g')
legend('ax', 'ay', 'az')
title('Acceleration components')
subplot(2,1,2)
plot(t, act1, t1, activity, 'r')
legend('dm', 'Activity')
title('Dynamic Acceleration and Activity')

%% PLotting
figure(2)
subplot(2,1,1)
plot(t, onBoardZacc, 'r', t, onBoardYacc, 'b', t, onBoardXacc, 'g')
title("On-board accelerometer")
legend("az", "ay", "ax")
ylabel("Acceleration (G)")
xlabel("Time (s)")

subplot(2,1,2)
plot(t, offBoardZacc, 'r', t, offBoardYacc, 'b', t, offBoardXacc, 'g')
title("Off-board Accelerometer")
legend("az", "ay", "ax")
ylabel("Acceleration (G)")
xlabel("Time (s)")
ylim([-1 1.5])


subplot(4,1,3)
%plot(t, yaw, 'r', t, pitch, 'b', t, roll, 'g')
title("Heading")
hold on
%legend('Yaw', 'Pitch', 'Roll')

%plot local minima of pitch angles
localMin = islocalmin(pitch);
plot(t, pitch, 'b',t(localMin), pitch(localMin), 'r*')
ylabel("Heading (º)")
xlabel("Time (s)")
ylim([-10 90])

subplot(4,1,4)
%plot(t, yaw, 'r', t, pitch, 'b', t, roll, 'g')
localMin = islocalmin(p1);
plot(t,p1, 'b',t(localMin), p1(localMin), 'r*')
hold on
title("Heading (On Board Acc)")
ylabel("Heading (º)")
xlabel("Time (s)")
ylim([-10 90])

%% cup measures
%legend('Yaw', 'Pitch', 'Roll')
r = 6.3/2;
H = 15;
V = pi * r^2 * H; % cm3
angInit = 90.0;


%% Estimating Final Volume in cup based on angle
vf = [318 272 217 205 87 0 270 256 239 218 199 390 379 370 361 348 332 320 309 299 290 281 272 445 438 433 428 421 415 409 404 400 389 190 178 155 142 127 106 85 62 40 V 30 15];
angMinOff = [25.54 20.2 16.08 17.05 6.07 0 22.37 22.88 20.37 19.16 15.69 55.7 49.64 48.37 43.34 39.77 36.45 32.12 31.14 29.63 27.27 25.13 24.86 72.45 73.58 70.65 67.74 64.72 60.91 58.96 56.41 51.23 49.66 17.98 19.05 14.62 14.38 13.35 11.68 9.85 7.6 5.49 90 3 2];

%find total displacement based on original position


angMinOff = angInit - angMinOff;

figure(3)
subplot(2,1,2)
scatter(angMinOff, vf, 'b*')
% title('Off-board accelerometer angles')
ylabel("Final Volume (ml)")
xlabel("Angle Difference (º)")
hold on

%% Mathematical model

% Measures of Cylinder (glass)

% Full cup (theoretical)
V = pi * r^2 * H; % cm3
alpha = [90]; %Initial reading when full
h2 = (2*r)./tand((alpha(1)));
h1 = H - h2;

volF = pi * r^2 * (H+h1)/2; % cm3 = ml

vols = [volF];

%First part of the model will work while water touches both walls
%shape is a cut cylinder
while(h2 <= H)
    alpha = [alpha, (alpha(end) - 0.001)];

    h2 = (2*r)/tand(alpha(end));

    h1 = H - h2;

    volF = pi * r^2 * (H+h1)/2; % cm3 = ml
    
    vols = [vols, volF];
end



% second part is a cylindrical wedge
% volF = 2/3*H^3*tand(theta(end))^2;

% we need to know the angles of mu
alpha = [alpha, (alpha(end) - 0.001)];
r1 = tand(alpha(end)) * H; %length of base segment of water
r3 = r1 - r;
a = acosd(r3/r);
beta = 180 - a;
% source: https://mathworld.wolfram.com/CylindricalWedge.html
volF = H * r^2 * (3 * sind(beta) - 3*deg2rad(beta)*cosd(beta) - (sind(beta))^3) / (3* (1 - cosd(beta)));
vols = [vols, volF];
fit1 = alpha(end);

while(beta >= 90)
    alpha = [alpha, (alpha(end) - 0.001)];
    r1 = tand(alpha(end)) * H; %length of base segment of water
    r3 = r1 - r;
    a = acosd(r3/r);
    beta = 180 - a;
    % source: https://mathworld.wolfram.com/CylindricalWedge.html
    volF = H * r^2 * (3 * sind(beta) - 3*deg2rad(beta)*cosd(beta) - (sind(beta))^3) / (3* (1 - cosd(beta)));
    vols = [vols, volF];
end


alpha = [alpha, (alpha(end) - 0.001)];
r1 = tand(alpha(end)) * H; %length of base segment of water
r3 = r - r1;
beta = acosd(r3/r);
volF = H * r^2 * (3 * sind(beta) - 3*deg2rad(beta)*cosd(beta) - (sind(beta))^3) / (3* (1 - cosd(beta)));
vols = [vols, volF];

while(beta > 0)
    alpha = [alpha, (alpha(end) - 0.001)];
    r1 = tand(alpha(end)) * H; %length of base segment of water
    r3 = r - r1;
    beta = acosd(r3/r);
    volF = H * r^2 * (3 * sind(beta) - 3*deg2rad(beta)*cosd(beta) - (sind(beta))^3) / (3* (1 - cosd(beta)));
    vols = [vols, volF];
end

angMinOn = [30.42 24.08 19.30 20.35 9.62 0 26.15 25.94 24.41 22.65 19.13 60.20 53.50 52.74 47.19 44.01 39.59 35.58 35.21 32.74 30.53 28.43 27.61 78.16 75.17 71.99 69.07 66.21 64.67 61.86 54.94 53.87 49.66 18.95 19.09 15.53 15.32 13.62 12.19 8.95 8.09 4.29 90 3 2];
angMinOn = angInit - angMinOn;


theta = 90 - alpha; %displacement
plot(theta, vols, 'r', 'LineWidth',1)
scatter(angMinOn, vf, 'g*')
title("Theoretical Model vs Real Measures")
ylabel("Final Volume (ml)")
xlabel("Angle Difference (º)")
legend('Off-board sensor', 'Model', 'On-board sensor')

subplot(2, 1,1)
plot(theta, vols, 'r', 'LineWidth',1)
title("Theoretical Model")
ylabel("Final Volume (ml)")
xlabel("Angle Difference (º)")

%% Data statistics of sensors vs model
onBoardAngleIdx = int32(angMinOn.*1000 + 1);
offBoardAngleIdx = int32(angMinOff.*1000 + 1);

errorOffBoard = zeros(1, length(vf));
errorOnBoard = zeros(1, length(vf));

for i=1:length(errorOffBoard)
    errorOffBoard(i) = vols(offBoardAngleIdx(i)) - vf(i);
    errorOnBoard(i) = vols(onBoardAngleIdx(i)) - vf(i);
end

statsOnBoardTh = datastats(errorOnBoard');
statsOffBoardTh = datastats(errorOffBoard');

%% Model vs data subplot

figure(5)
subplot(2,1,1)
plot(theta, vols, 'r', 'LineWidth', 1)
hold on
scatter(angMinOn, vf, 'b*')
title("Theoretical Model vs On-board Accelerometer")
ylabel("Final Volume (ml)")
xlabel("Angle Difference (º)")

subplot(2,1,2)
plot(theta, vols, 'r', 'LineWidth', 1)
hold on
scatter(angMinOff, vf, 'b*')
title("Theoretical Model vs Off-board Accelerometer")
ylabel("Final Volume (ml)")
xlabel("Angle Difference (º)")
%% Fit lines off board
%off board accelerometer
fit1 = 54.7319;

fit2 = 43.66; %to use with later part of graph
fit3 = 57.37; %to use with first part of graph

line1Angle = angMinOff(angMinOff <= fit3);
line1Volume = vf(find(angMinOff <= fit3));

line2Angle = angMinOff(angMinOff >= fit2);
line2Volume = vf(find(angMinOff >= fit2));

[yin ind] = ismember(line2Angle, line1Angle);
ind = ind(ind>0);


figure
scatter(line1Angle, line1Volume, 'r*')
hold on
scatter(line2Angle, line2Volume, 'b*')
scatter(line1Angle(ind), line1Volume(ind), 'g*')
title('Piece-wise separation for off-board sensor')
ylabel("Final Volume (ml)")
xlabel("Angle Difference (º)")
legend('First line', 'Second line', 'Common points')
hold off

%% Add symbolic solving for intersect

syms x
l1 = -0.02626*x^2 - 1.039*x + 466.1;
l2 = 0.005094*x^3 - 1.205*x^2 + 82.79*x -1412;

inters = solve(l1 - l2, x);

%% Plot fitted line
lInt = double(inters(2))

x1 = 0:0.0001:lInt;
x2 = lInt:0.0001:angInit;

l1 = -0.02626.*x1.^2 - 1.039.*x1 + 466.1;
l2 = 0.005094.*x2.^3 - 1.205.*x2.^2 + 82.79.*x2 -1412;

fittedOffBoard = [l1 l2];
anglesOffBoard = [x1 x2];

figure
plot(x2,l2, 'r')
hold on
plot(x1, l1, 'r')
ylim([0 500])
scatter(line1Angle, line1Volume, 'b*')
scatter(line2Angle, line2Volume, 'b*')
title("Fitted Off-Board sensor")
ylabel("Final Volume (ml)")
xlabel("Angle Difference (º)")

%% Error calculation
fittedOffBoardVolume = [l1 l2];
fittedAnglesOff = [x1 x2];

angDif = zeros(1, length(angMinOff));
volDif = zeros(1, length(angMinOff));

for i=1:length(angMinOff)
    j = cast(angMinOff(i)*10000,'uint32');
    volDif(i) = fittedOffBoardVolume(j+1) - vf(i);
end

statsOffBoard = datastats(volDif');
%% Fit lines on board
%on board accelerometer
fit1 = 54.7319;

fit2 = 43.66; %to use with later part of graph
fit3 = 57.37; %to use with first part of graph

line1Angle = angMinOn(angMinOn <= fit3);
line1Volume = vf(find(angMinOn <= fit3));

line2Angle = angMinOn(angMinOn >= fit2);
line2Volume = vf(find(angMinOn >= fit2));

% save('angles.csv', line2Angle, line2Volume, '-ascii')

[yin ind] = ismember(line2Angle, line1Angle);
ind = ind(ind>0);


figure
scatter(line1Angle, line1Volume, 'r*')
hold on
scatter(line2Angle, line2Volume, 'b*')
scatter(line1Angle(ind), line1Volume(ind), 'g*')
title('Piece-wise separation for on-board sensor')
legend('First line', 'Second line', 'Common points')
ylabel("Final Volume (ml)")
xlabel("Angle Difference (º)")
hold off

%% Add symbolic solving for intersect

syms x
l1 = -0.03106*x^2 - 1.018*x + 463;
l2 = 0.004996*x^3 - 1.13*x^2 + 74.3*x - 1176;

inters = solve(l1 - l2, x);

%% Plot fitted line
lInt = double(inters(2))

x1 = 0:0.0001:lInt;
x2 = lInt:0.0001:angInit;

l1 = -0.03106.*x1.^2 - 1.018.*x1 + 463;
l2 = 0.004996.*x2.^3 - 1.13.*x2.^2 + 74.3.*x2 - 1176;

fittedOnBoard = [l1 l2];
anglesOnBoard = [x1 x2];

figure
plot(x2,l2, 'r')
title('Fitted On-board sensor')
ylim([0 500])
hold on
plot(x1, l1, 'r')
scatter(line1Angle, line1Volume, 'b*')
scatter(line2Angle, line2Volume, 'b*')
ylabel("Final Volume (ml)")
xlabel("Angle Difference (º)")
hold off

%% Error calculation
fittedOnBoardVolume = [l1 l2];
fittedAnglesOn = [x1 x2];

angDif = zeros(1, length(angMinOn));
volDif = zeros(1, length(angMinOn));

for i=1:length(angMinOff)
    j = cast(angMinOn(i)*10000,'uint32');
    volDif(i) = fittedOnBoardVolume(j+1) - vf(i);
end


statsOnBoard = datastats(volDif');

%% Different models
figure
plot(theta,vols)
hold on
plot(fittedAnglesOn, fittedOnBoardVolume)
plot(fittedAnglesOff, fittedOffBoardVolume)
ylim([0 500])
xlim([0 90])
title('Theoretical vs Fitted Lines')
legend('Theoretical', 'On-board sensor', 'Off-board sensor')
ylabel("Final Volume (ml)")
xlabel("Angle Difference (º)")

%% Angles plotter






