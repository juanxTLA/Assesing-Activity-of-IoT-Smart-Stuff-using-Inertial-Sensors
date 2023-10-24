close all

samplingFreq = 20;
ts = 1/samplingFreq;
t = (1:nSamples).*ts;

drinks = [];
times = [];
s = 120;
s1 = 119;
%% Process files
file12 = load("drinkTest12.txt", "-ascii");
pitch12 = file12(:,5);
nSamples = length(file12);
t = (1:nSamples).*ts;

file11 = load("drinkTest11.txt", "-ascii");
pitch11 = file11(:,5);
nSamples = length(file11);
t = (1:nSamples).*ts;

file10 = load("drinkTest10.txt", "-ascii");
pitch10 = file10(:,5);
nSamples = length(file10);
t = (1:nSamples).*ts;

file9 = load("drinkTest9.txt", "-ascii");
pitch9 = file9(:,5);
nSamples = length(file9);
t = (1:nSamples).*ts;


%% Plot
figure
title("Angles for single drinking events")
xlabel("time (s)")
ylabel("Angle(deg)")
hold on
plot(t(1:s), pitch12(410:410+s1))
plot(t(1:s), pitch12(661:661+s1))
plot(t(1:s), pitch12(1035:1035+s1))
plot(t(1:s), pitch12(1696:1696+s1))
plot(t(1:s), pitch12(2388:2388+s1))
plot(t(1:s), pitch12(3079:3079+s1))

plot(t(1:s), pitch11(103:103+s1))
plot(t(1:s), pitch11(739:739+s1))
plot(t(1:s), pitch11(1027:1027+s1))
plot(t(1:s), pitch11(1225:1225+s1))
plot(t(1:s), pitch11(1408:1408+s1))
plot(t(1:s), pitch11(1951:1951+s1))
plot(t(1:s), pitch11(2168:2168+s1))

plot(t(1:s), pitch10(35:35+s1))
plot(t(1:s), pitch10(375:375+s1))
plot(t(1:s), pitch10(1339:1339+s1))
plot(t(1:s), pitch10(1560:1560+s1))
plot(t(1:s), pitch10(1738:1738+s1))
plot(t(1:s), pitch10(1944:1944+s1))
plot(t(1:s), pitch10(2215:2215+s1))
plot(t(1:s), pitch10(2486:2486+s1))
plot(t(1:s), pitch10(2740:2740+s1))

plot(t(1:s), pitch9(312:312+s1))
plot(t(1:s), pitch9(644:644+s1))
plot(t(1:s), pitch9(975:975+s1))
plot(t(1:s), pitch9(1338:1338+s1))

