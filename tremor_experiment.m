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
lim = floor(length(t)/samplingFreq)

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

%% act1
act1 = [];
for i=1:length(onBoardXacc)
    da = sqrt(onBoardXacc(i)^2 + onBoardYacc(i)^2 + onBoardZacc(i)^2) - 1 -  0.0106;
    act1 = [act1, da];
end

activity = [];
t1 = [];

for i=0:lim-1
    tx = 0;
    for j=1:100
        tx = tx + abs(act1(100*i + j));
    end
    activity = [activity, tx];
    t1 = [t1, i];
end

t1 = t1 +1;

f = [zeros(1, 21) 4 3.8835 3.8095 3.9216 4.1237 3.9216 4.1237 3.7736 3.8835 3.9216 zeros(1,3)];
tf = 1:34;
figure


figure
subplot(2,1,1)
plot(t, onBoardZacc, 'r', t, onBoardYacc, 'b', t, onBoardXacc, 'g')
legend('ax', 'ay', 'az')
title('Acceleration components')
ylabel('Acc [G]')
xlabel('time [s]')
subplot(2,1,2)
plot(t1, activity, 'b')
ylabel('Activity')
yyaxis right
plot(tf, f, 'r')
legend('Activity', 'Tremor Frequency')
title('Activity and Tremor Frequency')
xlabel('time [s]')
ylabel('Frequency [Hz]')

grid

%%
%21.88 22.91 23.96 24.98 25.95 26.97 29 30.03 31.05
% tf = [22 23 24 25 26 27 28 29 30 31];



%% Activity off board
actF = [];
t1 = [];

act = 0;
cnt = 0;
lim = floor(length(t)/samplingFreq);
while cnt < lim
    for i=1:samplingFreq-1
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