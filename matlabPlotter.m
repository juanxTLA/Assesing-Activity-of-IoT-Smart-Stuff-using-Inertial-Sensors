clear all
close all
clc
% Specify the file path
file_path = 'capacitance_experiment_1.txt';

% Use importdata to read the file with space delimiter
data = importdata(file_path, ' ');

% Check if 'data' is a structure with fields 'data' and 'textdata'
if isstruct(data)
    % If the data is stored as a structure, extract the numeric data
    data = data.data;
else
    % If the data is stored directly as a matrix, assign it to 'numeric_data'
    data = data;
end


data = data';

volume = data(1,:);
cap1 = data(2,:);
cap2 = data(3, :);

figure()
plot(volume, cap1)

figure()
plot(volume, cap2)

figure()
plot(volume, cap1, volume, cap2)
%%
c12=cap1-cap1(1)+cap2-cap2(1);
plot(volume, cap1-cap1(1), volume, cap2-cap2(1),volume,c12,'r')