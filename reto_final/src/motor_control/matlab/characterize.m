clf
clear
close all
motor_data = readmatrix('motor_data.csv');
data = iddata(motor_data(:,2), motor_data(:, 3));
data.OutputName = 'Motor Velocity';
data.OutputUnit = 'RAD/S';
data.InputName = 'Motor Percent';
data.InputUnit = '%';
figure
plot(data)

sysTF = tfest(data, 2, 0, nan);
figure
resid(sysTF, data);
figure
compare(data, sysTF)
sysTF

[C_pi, info] = pidtune(sysTF, 'PI')
T_pi = feedback(C_pi * sysTF, 1);
figure
step(T_pi)