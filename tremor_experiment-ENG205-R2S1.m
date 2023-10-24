clc
clear all
close all
file = load("tremor2.txt", "-ascii");

onBoardXacc = file(:,1);
onBoardYacc = file(:,2);
onBoardZacc = file(:,3);

yaw = file(:,4);
pitch = file(:,5);
roll = file(:,6);

offBoardXacc = file(:,7);
offBoardYacc = file(:,8);
offBoardZacc = file(:,9);

nSamples = length(file);
samplingFreq = 100;
ts = 1/samplingFreq;
t = (1:nSamples).*ts;

p1 = atand(onBoardZacc./(sqrt(onBoardXacc.^2+onBoardYacc.^2)));

%% PLotting
figure(2)
subplot(2,1,1)
plot(t, onBoardZacc, 'r', t, onBoardYacc, 'b', t, onBoardXacc, 'g')
title("On-board accelerometer")
legend("az", "ay", "ax")
ylabel("Acceleration (G)")
xlabel("Time (s)")
grid

subplot(2,1,2)
plot(t, offBoardZacc, 'r', t, offBoardYacc, 'b', t, offBoardXacc, 'g')
title("Off-board Accelerometer")
legend("az", "ay", "ax")
ylabel("Acceleration (G)")
xlabel("Time (s)")
ylim([-0.5 2])

%% Activity o Board
actF = [];
t1 = [];

act = 0;
cnt = 0;
lim = floor(length(t)/samplingFreq);
while cnt < lim
    for i=1:samplingFreq
        act = act + abs(sqrt(onBoardXacc(samplingFreq * cnt + i)^2 + onBoardYacc(samplingFreq*cnt + i)^2 + onBoardZacc(samplingFreq*cnt + i)^2) - 1);
    end
    actF = [actF, act];
    cnt = cnt + 1;
    t1 = [t1, cnt];
    act = 0;
end


figure
% plot(t1, actF)
hold on
plot(t, onBoardZacc, 'r', t, onBoardYacc, 'b', t, onBoardXacc, 'g')
legend('dm', 'ax', 'ay', 'az')
title('Activity vs On-Board Accelerometer')
grid


%% Activity off board
actF = [];
t1 = [];

act = 0;
cnt = 0;
lim = floor(length(t)/samplingFreq);
while cnt < lim
    for i=1:samplingFreq
        act = act + abs(sqrt(offBoardXacc(samplingFreq * cnt + i)^2 + offBoardYacc(samplingFreq*cnt + i)^2 + offBoardZacc(samplingFreq*cnt + i)^2) - 1);
    end
    actF = [actF, act];
    cnt = cnt + 1;
    t1 = [t1, cnt];
    act = 0;
end


figure
plot(t1, actF)
hold on
plot(t, offBoardZacc, 'r', t, offBoardYacc, 'b', t, offBoardXacc, 'g')
legend('dm', 'ax', 'ay', 'az')
title('Activity vs Off-Board Accelerometer')
grid
% ylim([-0.5 2])