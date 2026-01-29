% setup

clc; close all; clear;

% Set LaTeX as default interpreter for axis labels, ticks and legends
set(0,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

%set(0,'DefaultFigureWindowStyle','docked');
%set(0,'DefaultFigureWindowStyle','modal');

set(0,'defaultAxesFontSize',20)
set(0,'DefaultLegendFontSize',20)

vdd = 5.; 
% shunt_resistor = 1.; 
measurement_time = 60.; % 60 sec
decimate_factor = 1500;


% --- ESP32 ---

file = load("data/esp.mat");

measurement_n_samples = int32(measurement_time / file.Tinterval) + 1;

voltage_measured = transpose(file.A(1:measurement_n_samples));

current = voltage_measured;
v_drop_shunt = (2.e-5 * current) + 1.e-5;
power = (vdd - v_drop_shunt) .* current;

fprintf("\nESP32\n");
fprintf("Avg current is: %f mA\n", 1000 * mean(current));
fprintf("Avg power is: %f mW\n", 1000 * mean(power));

t = 0:file.Tinterval:measurement_time;
plot(t, power);

esp_current = current; % useful later

% --- display 100 lux ---

file = load("data/esp_display_100lux.mat");

measurement_n_samples = int32(measurement_time / file.Tinterval) + 1;

voltage_measured = transpose(file.A(1:measurement_n_samples));

current = voltage_measured - esp_current;
v_drop_shunt = (2.e-5 * current) + 1.e-5;
power = (vdd - v_drop_shunt) .* current;

fprintf("\nDisplay 100lux\n");
fprintf("Avg current is: %f mA\n", 1000 * mean(current));
fprintf("Avg power is: %f mW\n", 1000 * mean(power));

% --- display 450 lux ---

file = load("data/esp_display_450lux.mat");

measurement_n_samples = int32(measurement_time / file.Tinterval) + 1;

voltage_measured = transpose(file.A(1:measurement_n_samples));

current = voltage_measured - esp_current;
v_drop_shunt = (2.e-5 * current) + 1.e-5;
power = (vdd - v_drop_shunt) .* current;

fprintf("\nDisplay 450lux\n");
fprintf("Avg current is: %f mA\n", 1000 * mean(current));
fprintf("Avg power is: %f mW\n", 1000 * mean(power));

esp_display_450lux_current = current + esp_current; % useful later

% --- display 3000 lux ---

file = load("data/esp_display_3000lux.mat");

measurement_n_samples = int32(measurement_time / file.Tinterval) + 1;

voltage_measured = transpose(file.A(1:measurement_n_samples));

current = voltage_measured - esp_current;
v_drop_shunt = (2.e-5 * current) + 1.e-5;
power = (vdd - v_drop_shunt) .* current;

fprintf("\nDisplay 3000lux\n");
fprintf("Avg current is: %f mA\n", 1000 * mean(current));
fprintf("Avg power is: %f mW\n", 1000 * mean(power));

% --- gyro 450 lux ---
file = load("data/esp_acc_gyro_450lux.mat");

measurement_n_samples = int32(measurement_time / file.Tinterval) + 1;

voltage_measured = transpose(file.A(1 : measurement_n_samples));

current = voltage_measured - esp_display_450lux_current;
v_drop_shunt = (2.e-5 * current) + 1.e-5;
power = (vdd - v_drop_shunt) .* current;

fprintf("\nGyroscope/accelerometer\n");
fprintf("Avg current is: %f mA\n", 1000 * mean(current));
fprintf("Avg power is: %f mW\n", 1000 * mean(power));

% --- temp/humid 450lux ---
file = load("data/esp_temp_450lux.mat");

measurement_n_samples = int32(measurement_time / file.Tinterval) + 1;

voltage_measured = transpose(file.A(1 : measurement_n_samples));

current = voltage_measured - esp_display_450lux_current;
v_drop_shunt = (2.e-5 * current) + 1.e-5;
power = (vdd - v_drop_shunt) .* current;

fprintf("\nTemperature/Humidity sensor\n");
fprintf("Avg current is: %f mA\n", 1000 * mean(current));
fprintf("Avg power is: %f mW\n", 1000 * mean(power));

% --- magnetometer 450lux ---
file = load("data/esp_magnet_450lux.mat");

measurement_n_samples = int32(measurement_time / file.Tinterval) + 1;

voltage_measured = transpose(file.A(1 : measurement_n_samples));

current = voltage_measured - esp_display_450lux_current;
v_drop_shunt = (2.e-5 * current) + 1.e-5;
power = (vdd - v_drop_shunt) .* current;

fprintf("\nMagnetometer\n");
fprintf("Avg current is: %f mA\n", 1000 * mean(current));
fprintf("Avg power is: %f mW\n", 1000 * mean(power));

% --- brightness 450lux ---
file = load("data/esp_brightness_450lux.mat");

measurement_n_samples = int32(measurement_time / file.Tinterval) + 1;

voltage_measured = transpose(file.A(1 : measurement_n_samples));

current = voltage_measured - esp_display_450lux_current;
v_drop_shunt = (2.e-5 * current) + 1.e-5;
power = (vdd - v_drop_shunt) .* current;

fprintf("\nBrightness sensor\n");
fprintf("Avg current is: %f mA\n", 1000 * mean(current));
fprintf("Avg power is: %f mW\n", 1000 * mean(power));

% --- Wifi 450lux ---
file = load("data/esp_wifi_450lux.mat");

measurement_n_samples = int32(measurement_time / file.Tinterval) + 1;

voltage_measured = transpose(file.A(1 : measurement_n_samples));

current = voltage_measured;
v_drop_shunt = (2.e-5 * current) + 1.e-5;
power = (vdd - v_drop_shunt) .* current;

fprintf("\nWifi\n");
fprintf("Avg current is: %f mA\n", 1000 * mean(current));
fprintf("Avg power is: %f mW\n", 1000 * mean(power));

t = 0 : file.Tinterval:measurement_time;
hold on;
plot(decimate(t, decimate_factor), decimate(power, decimate_factor));

legend("ESP32", "ESP32+Wi-Fi")