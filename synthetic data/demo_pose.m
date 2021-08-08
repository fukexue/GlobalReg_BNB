clc;clear;close all;
rng('default')

point3d_num = 20;       %3D点的数目
point3d_center=[0;0;500];%3D旋转中心
Epslion=5;              %允许误差
Sigma=10;                %refine参数
plane_z=1000;            %投影平面1
Project_center=[0;-500;500];
plane_y=500;
ts_line=-5:1:5;

ts=getSeed([-5;-5;-5],[10;10;10],3);
noise_rate=[0.5,1,1.5,2,2.5];
error_degree=[];
error_trans=[];
error_degree_refine=[];
error_trans_refine=[];
error_meantre=[];
error_meantre_2d=[];
error_everytre=[];
error_meantre_zq=[];
error_everytre_zq=[];
time_all=[];

for j=1:size(noise_rate,2)
    error_degree_temp=[];
    error_transla_temp=[];
    error_degree_refine_temp=[];
    error_transla_refine_temp=[];
    error_everytre_temp=[];
    error_meantre_temp=[];
    error_meantre_temp_2d=[];
    error_everytre_temp_zq=zeros(20,point3d_num);
    error_meantre_temp_zq=[];
    time_noise_rate=[];
    for i=1:50
        disp([j,i])
        Point_3d=rand(3,point3d_num)*100+point3d_center+[-50;-50;-50];%生成�?3D�?
        r_d=rand(3,1);
        r=r_d/norm(r_d)*pi*(360*rand-180)/180; %生成�?个随机的旋转
        r_degree=norm(r)*180/pi;
        R=rotationVectorToMatrix(r);
        Ground_Truth_T=rand(3,1)*10-5;%生成�?个随机的平移
        % Ground_Truth_T=[0;0;0];
        center_rot=mean(Point_3d,2);
        y_=R*(Point_3d-center_rot)+center_rot+Ground_Truth_T;%得到旋转以后的加噪声�?3D�?

        Point_2d_Z=Project_point(y_,plane_z);%得到投影平面1�?2D�?
        Point_2d_Z_NOISE=normrnd(0,noise_rate(j),[2,size(Point_2d_Z,2)]);
        Point_2d_Z=Point_2d_Z_NOISE+Point_2d_Z;
        Point_2d_Y=Project_point_Y(y_,Project_center,plane_y);
        Point_2d_Y=Point_2d_Z_NOISE+Point_2d_Y;
        
        save(['data/',num2str(j),num2str(i),'data'],'Point_3d','r','Ground_Truth_T')

        tic_1=tic;
        [best_r,best_t] = pose(Point_2d_Z,Point_3d,Epslion,center_rot,ts,plane_z);
        best_t(3)=pose_t(Point_2d_Y,Point_3d,best_r,best_t,center_rot,ts_line,Project_center,plane_y,Epslion);
        pose_time = toc(tic_1);
        rotation1_erro=acosd(0.5*(trace(R'*rotationVectorToMatrix(best_r))-1));
        trans1_erro=norm(Ground_Truth_T-best_t);
        error_degree_temp=[error_degree_temp,rotation1_erro];
        error_transla_temp=[error_transla_temp,trans1_erro];

        tic_2=tic;
        [r_out,t_out,E] = refine(best_r,best_t,Point_3d,Point_2d_Z,Point_2d_Y,center_rot,Project_center,plane_z,plane_y,[-5;-5;-5;5;5;5],Epslion,Sigma);
        refine_time=toc(tic_2);

        rotation_erro=acosd(0.5*(trace(R'*rotationVectorToMatrix(r_out))-1));
        trans_erro=norm(Ground_Truth_T-t_out);
        error_degree_refine_temp=[error_degree_refine_temp,rotation_erro];
        error_transla_refine_temp=[error_transla_refine_temp,trans_erro];

%         best_R=rotationVectorToMatrix(best_r);
%         y_best_r=best_R*(Point_3d-center_rot)+center_rot+best_t;%得到旋转以后的加噪声�?3D�?
%         figure;
%         Point_2d_yuan=Project_point(y_best_r,plane_z);%得到投影平面1�?2D�?
%         plot(Point_2d(1,:),Point_2d(2,:),'or')
%         hold on
%         plot(Point_2d_yuan(1,:),Point_2d_yuan(2,:),'ob')
        R_best=rotationVectorToMatrix(best_r);
        R_point3d=R*(Point_3d-center_rot)+center_rot+Ground_Truth_T;
        R_out_point3d=R_best*(Point_3d-center_rot)+center_rot+best_t;
        every_TRE=sqrt(sum((R_point3d-R_out_point3d).^2));
        meanTRE=sum(every_TRE)/size(every_TRE,2);
        error_meantre_temp_zq=[error_meantre_temp_zq,meanTRE];
        error_everytre_temp_zq=[error_everytre_temp_zq;every_TRE];

        R_out=rotationVectorToMatrix(r_out);
        R_point3d=R*(Point_3d-center_rot)+center_rot+t_out;
        R_out_point3d=R_out*(Point_3d-center_rot)+center_rot+t_out;
        every_TRE_r=sqrt(sum((R_point3d-R_out_point3d).^2));
        meanTRE_r=sum(every_TRE_r)/size(every_TRE_r,2);
        error_meantre_temp=[error_meantre_temp,meanTRE_r];
        error_everytre_temp=[error_everytre_temp;every_TRE_r];
        time_noise_rate=[time_noise_rate,[pose_time;refine_time]];
        
        point_2d_best=Project_point(R_out_point3d,plane_z);
        error_meantre_temp_2d=[error_meantre_temp_2d,sum(sqrt(sum((Point_2d_Z-point_2d_best).^2)))/size(point_2d_best,2)];
    end
    error_degree=[error_degree;error_degree_temp];
    error_trans=[error_trans;error_transla_temp];
    error_degree_refine=[error_degree_refine;error_degree_refine_temp];
    error_trans_refine=[error_trans_refine;error_transla_refine_temp];
    error_meantre=[error_meantre;error_meantre_temp];
    error_meantre_2d=[error_meantre_2d;error_meantre_temp_2d];
    error_everytre=[error_everytre;error_everytre_temp];
    error_meantre_zq=[error_meantre_zq;error_meantre_temp_zq];
    error_everytre_zq=[error_everytre_zq;error_everytre_temp_zq];
    time_all=[time_all;time_noise_rate];
end
save('result/pose_noise_rate_0to1_double')

