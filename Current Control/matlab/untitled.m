% clear; clc;
% data = readmatrix("measurements/step20_mat_rs_10ms.csv"); inp20 = data(:, 3); out20 = data(:, 2); inp20 = (inp20-1000)/1000;
% data = readmatrix("measurements/step60_mat_rs_10ms.csv"); inp60 = data(:, 3); out60 = data(:, 2); inp60 = (inp60-1000)/1000;
% data = readmatrix("measurements/test3_mat_rs_10ms.csv"); inp3 = data(:, 3); out3 = data(:, 2); inp3 = (inp3-1000)/1000;
% data = readmatrix("measurements/test4_mat_rs_10ms.csv"); inp4 = data(:, 3); out4 = data(:, 2); inp4 = (inp4-1000)/1000;
% data = readmatrix("measurements/test5_mat_rs_10ms.csv"); inp5 = data(:, 3); out5 = data(:, 2); inp5 = (inp5-1000)/1000;
function untitled()
    load matlab.mat model;
    step(model)
