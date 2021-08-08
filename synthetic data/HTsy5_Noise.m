
clc;
clear;
close all;
load('result/pose_noise_rate_0to1.mat')
translation_error_our=mean(error_trans_refine,2);
load('result/pose_noise_rate_0to1_double.mat')
translation_error_our_double=mean(error_trans_refine,2);


figure(1);
plot(noise_rate,translation_error_our,'o-','LineWidth',8);
hold on 
plot(noise_rate,translation_error_our_double,'x-','LineWidth',8);
ylim([0,2.6])
ylabel('Translation Error')
xlabel('Noise')
legend({'Our\_SE3\_S','Our\_SE3\_D'},'FontSize',45)
set(gca,'FontSize',45,'Fontname', 'Times New Roman'); 
set(get(gca,'XLabel'),'FontSize',45); 
set(get(gca,'YLabel'),'FontSize',45);
set(gca,'linewidth',7)
