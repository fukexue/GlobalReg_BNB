clc;clear;close all

read(3d);
read(2d);

set center_rots;

set epsilon;
set plane_z;

get G&T-goudtruth;

tic
[best_r,best_t] = POSE(point_2d,point_3d,epsilon,center_rots,plane_z)
toc

