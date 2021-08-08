%%
%真实数据实验
%%
%可用实验代码
%采用真实数据中的TP数据
%case1
%%
clc;clear;close all;
rng('default')
% case_index=1;
for case_index=1:10
iter_epoch=10;
error_r=zeros(1,iter_epoch);
error_t=zeros(1,iter_epoch);
error_r_refine=zeros(1,iter_epoch);
error_t_refine=zeros(1,iter_epoch);
time_every=zeros(1,iter_epoch);
time_every_refine=zeros(1,iter_epoch);
tre_mean_pose=zeros(1,iter_epoch);
tre_mean_refine=zeros(1,iter_epoch);
tre_max_pose=zeros(1,iter_epoch);
tre_max_refine=zeros(1,iter_epoch);
for i=1:iter_epoch
    disp('----------------设置实验参数----------------')
    epsilon=20;
    ts=getSeed([-20;-20;-20],[40;40;40],5,5,5);
    ts_line=-20:2:20;
    Sigma=5;
    disp('----------------开始加载实验数据----------------')
    load(['Final_data/case',num2str(case_index)])
    Point_3d=PC_3D;%世界坐标系下
    Point_2d_AP=PC_2D_AP;
    Point_2d_LAT=PC_2D_LAT;
    if i==1
        disp('-------开始下采样--------')
        [idx1,Point_3d_D] = kmeans(Point_3d',60);%采样得到的未旋转未平移的数据
        Point_3d_D=Point_3d_D';
        [idx2,Point_2d_D_AP] = kmeans(Point_2d_AP',90);%采样得到的未旋转未平移的数据
        Point_2d_D_AP=Point_2d_D_AP';
        [idx3,Point_2d_D_LAT] = kmeans(Point_2d_LAT',90);%采样得到的未旋转未平移的数据
        Point_2d_D_LAT=Point_2d_D_LAT';
        disp('-------降采样完成--------')
    end
    %% 随机产生变化矩阵
    disp('-------随机产生变化矩阵并对数据旋转投影到AP/LAT平面--------')
    r1_dirction=2*rand(3,1)-1;
    r1=r1_dirction/norm(r1_dirction)*pi*(rand*20-10)/180;
    INITTRANS=[40*rand-20;40*rand-20;40*rand-20];
%     r1=[0;0;0];
%     INITTRANS=[0;0;0];
    INITROT=rotationVectorToMatrix(r1);
    
    [Point_3d_D_AP,f_AP,tans_s_AP] = Proj_extract2(Point_3d_D,Tx_AP,xs_AP,ys_AP,zs_AP);%转换到X-ray坐标系
    center_rot_AP=mean(Point_3d_D_AP,2);
    Point_3d_D_R_AP=inv(INITROT)*(Point_3d_D_AP-center_rot_AP-INITTRANS)+center_rot_AP; %旋转
    Point_3d_D_R_AP_inv=INITROT*(Point_3d_D_R_AP-center_rot_AP)+center_rot_AP+INITTRANS;%再旋转回来
    [Point_3d_D_LAT,f_LAT,tans_s_LAT] = Proj_extract2(Point_3d_D,Tx_LAT,xs_LAT,ys_LAT,zs_LAT);
    center_rot_LAT=mean(Point_3d_D_LAT,2);

    disp('-------数据随机旋转并投影完成--------')
    %%
%     figure;
%     plot3(Point_3d_D_AP(1,:),Point_3d_D_AP(2,:),Point_3d_D_AP(3,:),'.b'); hold on
%     plot3(Point_3d_D_R_AP(1,:),Point_3d_D_R_AP(2,:),Point_3d_D_R_AP(3,:),'.r');
% %     plot3(Point_3d_D_AP2LAT(1,:),Point_3d_D_AP2LAT(2,:),Point_3d_D_AP2LAT(3,:),'.g');
%     legend('原始','旋转后')
%     title('配准前3D') 
    %%
    Point_3d_D_AP_2d=(ProjectPoint(Point_3d_D_AP,f_AP)+tans_s_AP)/0.1540;
    Point_3d_D_R_AP_2d=(ProjectPoint(Point_3d_D_R_AP,f_AP)+tans_s_AP)/0.1540;
    Point_3d_D_R_AP_2d_inv=(ProjectPoint(Point_3d_D_R_AP_inv,f_AP)+tans_s_AP)/0.1540;
    Point_3d_D_LAT_2d=(ProjectPoint(Point_3d_D_LAT,f_AP)+tans_s_AP)/0.1540;
%     figure,
%     imshow(X_2D_AP);
%     hold on;
%     plot(Point_2d_D_AP(1,:),Point_2d_D_AP(2,:),'b.');%真实2d点
%     plot(Point_3d_D_R_AP_2d(1,:),Point_3d_D_R_AP_2d(2,:),'r.');%旋转以后的投影2d点
%     plot(Point_3d_D_R_AP_2d_inv(1,:),Point_3d_D_R_AP_2d_inv(2,:),'g.');%旋转再旋转以后的投影2d点
%     plot(Point_3d_D_AP_2d(1,:),Point_3d_D_AP_2d(2,:),'k.');%无旋转投影2d点
%     legend('真实2d','反旋转后','反旋转＋正旋转后','无旋转3d投影')
%     title('配准前AP2D')
%     axis equal
%     figure,
%     hold on;
%     plot(Point_2d_D_LAT(1,:),Point_2d_D_LAT(2,:),'b.');%真实2d点
%     plot(Point_3d_D_LAT_2d(1,:),Point_3d_D_LAT_2d(2,:),'k.');%无旋转投影2d点
%     legend('真实2d','无旋转3d投影')
%     title('配准前LAT2D')
%     axis equal
    %%
    disp('-------开始求解最优的旋转和最优的平移--------')
    tic_1=tic;
    %传入的数据：真实2D数据（像素），3D点（世界坐标系下旋转过，物理尺寸）
    [best_r,best_t] = pose(Point_2d_D_AP,Point_3d_D_R_AP,epsilon,center_rot_AP,ts,f_AP,tans_s_AP);
    R_best=rotationVectorToMatrix(best_r);
    Point_3d_D_R_AP_recover=R_best*(Point_3d_D_R_AP-center_rot_AP)+center_rot_AP+best_t;
    best_t(3)=pose_t(Point_2d_D_LAT,Point_3d_D_R_AP,R_best,best_t,center_rot_AP,epsilon,ts_line,Tx_LAT,xs_LAT,ys_LAT,zs_LAT,Tx_AP,xs_AP,ys_AP,zs_AP,f_LAT,tans_s_LAT);
    pose_time = toc(tic_1);

    Point_3d_D_AP_2d_recover=(ProjectPoint(Point_3d_D_R_AP_recover,f_AP)+tans_s_AP)/0.1540;
    near_pose_idx=knnsearch(Point_2d_D_AP',Point_3d_D_AP_2d_recover');
    tre_2d_pose_iter=sqrt(sum((Point_2d_D_AP(:,near_pose_idx)-Point_3d_D_AP_2d_recover).^2))*0.1540;
    
    Point_3d_D_R_AP_recover_1=R_best*(Point_3d_D_R_AP-center_rot_AP)+center_rot_AP+best_t;
    Point_3d_AP2LAT=inv(Tx_LAT)*(Tx_AP*[Point_3d_D_R_AP_recover_1;zeros(1,size(Point_3d_D_R_AP_recover_1,2))]+[xs_AP;ys_AP;zs_AP;1]-[xs_LAT;ys_LAT;zs_LAT;1]);
    Point_3d_AP2LAT2d_recover=ProjectPoint(Point_3d_AP2LAT,f_LAT);
    
    disp('-------AP面粗配准完成--------')
    disp(['pose最优旋转:','(',num2str(best_r(1)),',',num2str(best_r(2)),',',num2str(best_r(3)),')'])
    disp(['金标准旋转:','(',num2str(r1(1)),',',num2str(r1(2)),',',num2str(r1(3)),')']);
    disp(['pose最优平移:','(',num2str(best_t(1)),',',num2str(best_t(2)),',',num2str(best_t(3)),')'])
    disp(['金标准平移:','(',num2str(INITTRANS(1)),',',num2str(INITTRANS(2)),',',num2str(INITTRANS(3)),')'])
    disp(['POSEtime: ',num2str(pose_time),'s']);
    error_degree=acosd(0.5*(trace(INITROT'*rotationVectorToMatrix(best_r))-1));
    disp(['角度误差: ',num2str(error_degree),'°']);
    translation_error=norm(best_t-INITTRANS);
    disp(['平移误差: ',num2str(translation_error),'(mm)']);
    disp(['2d_tre: ',num2str(mean(tre_2d_pose_iter))]);
    
%     figure;
%     plot3(Point_3d_D_AP(1,:),Point_3d_D_AP(2,:),Point_3d_D_AP(3,:),'.b'); hold on
%     plot3(Point_3d_D_R_AP_recover(1,:),Point_3d_D_R_AP_recover(2,:),Point_3d_D_R_AP_recover(3,:),'.g');
%     legend('原始','旋转后恢复')
%     title('POSE配准后3D')
%     figure;
%     plot(Point_2d_D_AP(1,:),Point_2d_D_AP(2,:),'b.');%真实2d点
%     hold on;
%     plot(Point_3d_D_AP_2d_recover(1,:),Point_3d_D_AP_2d_recover(2,:),'g.');
%     axis equal
%     legend('真实2d','旋转后恢复2d')
%     title('POSE配准后2D')
%     figure;
%     plot(Point_2d_D_AP(1,near_pose_idx),Point_2d_D_AP(2,near_pose_idx),'b.');%真实2d点
%     hold on;
%     plot(Point_3d_D_AP_2d_recover(1,:),Point_3d_D_AP_2d_recover(2,:),'g.');
%     axis equal
%     legend('真实2d','旋转后恢复2d')
%     title('POSE配准后最近邻2D')
    
    
    Point_2d_LAT_nopixel=Point_2d_D_LAT*0.1540-tans_s_LAT;
%     figure;
%     plot(Point_2d_LAT_nopixel(1,:),Point_2d_LAT_nopixel(2,:),'b.');%真实2d点
%     hold on;
%     plot(Point_3d_AP2LAT2d_recover(1,:),Point_3d_AP2LAT2d_recover(2,:),'g.');
%     axis equal
%     legend('真实2d','旋转后恢复2d')
%     title('POSE配准后LAT-2D')
    
    
    tic_2=tic;
    [r_out,t_out,E] = refine(Point_2d_D_AP,Point_2d_D_LAT,Point_3d_D_R_AP,best_r,best_t,center_rot_AP,[-20;-20;-20;20;20;20],epsilon,Sigma,f_AP,f_LAT,tans_s_AP,tans_s_LAT,Tx_LAT,xs_LAT,ys_LAT,zs_LAT,Tx_AP,xs_AP,ys_AP,zs_AP);
    refine_time=toc(tic_2);
    
    R_out=rotationVectorToMatrix(r_out);
    Point_3d_D_R_AP_recover=R_out*(Point_3d_D_R_AP-center_rot_AP)+center_rot_AP+t_out;
    Point_3d_D_AP_2d_recover=(ProjectPoint(Point_3d_D_R_AP_recover,f_AP)+tans_s_AP)/0.1540;
    near_pose_idx=knnsearch(Point_2d_D_AP',Point_3d_D_AP_2d_recover');
    tre_2d_refine_iter=sqrt(sum((Point_2d_D_AP(:,near_pose_idx)-Point_3d_D_AP_2d_recover).^2))*0.1540;
    
%     figure;
%     plot3(Point_3d_D_AP(1,:),Point_3d_D_AP(2,:),Point_3d_D_AP(3,:),'.b'); hold on
%     plot3(Point_3d_D_R_AP_recover(1,:),Point_3d_D_R_AP_recover(2,:),Point_3d_D_R_AP_recover(3,:),'.g');
%     legend('原始','旋转后恢复')
%     title('refine配准后3D')
%     figure;
%     plot(Point_2d_D_AP(1,:),Point_2d_D_AP(2,:),'b.');%真实2d点
%     hold on;
%     plot(Point_3d_D_AP_2d_recover(1,:),Point_3d_D_AP_2d_recover(2,:),'g.');
%     axis equal
%     legend('真实2d','旋转后恢复2d')
%     title('refine配准后2D')
    
    
    Point_3d_AP2LAT_refine=inv(Tx_LAT)*(Tx_AP*[Point_3d_D_R_AP_recover;zeros(1,size(Point_3d_D_R_AP_recover,2))]+[xs_AP;ys_AP;zs_AP;1]-[xs_LAT;ys_LAT;zs_LAT;1]);
    Point_3d_AP2LAT2d_recover_refine=ProjectPoint(Point_3d_AP2LAT_refine,f_LAT);
    
    
%     figure;
%     plot(Point_2d_LAT_nopixel(1,:),Point_2d_LAT_nopixel(2,:),'b.');%真实2d点
%     hold on;
%     plot(Point_3d_AP2LAT2d_recover_refine(1,:),Point_3d_AP2LAT2d_recover_refine(2,:),'g.');
%     axis equal
%     legend('真实2d','旋转后恢复2d')
%     title('refine配准后LAT-2D')
    
    %%
    %画图
    [Point_3d_AP,f_AP,tans_s_AP] = Proj_extract2(Point_3d,Tx_AP,xs_AP,ys_AP,zs_AP);%转换到X-ray坐标系
    center_rot_AP=mean(Point_3d_AP,2);
    Point_3d_R_AP=inv(INITROT)*(Point_3d_AP-center_rot_AP-INITTRANS)+center_rot_AP; %旋转
    Point_3d_AP_2d=(ProjectPoint(Point_3d_AP,f_AP)+tans_s_AP)/0.1540;
    Point_3d_R_AP_2d=(ProjectPoint(Point_3d_R_AP,f_AP)+tans_s_AP)/0.1540;
    
    figure(1),
    imshow(X_2D_AP);
    hold on;
    plot(Point_2d_AP(1,:),Point_2d_AP(2,:),'k.');%真实2d点
    plot(Point_3d_AP_2d(1,:),Point_3d_AP_2d(2,:),'b.');%无旋转投影2d点
    title('ground_truth_AP2D')
    axis equal
    
    figure(2),
    imshow(X_2D_AP);
    hold on;
    plot(Point_2d_AP(1,:),Point_2d_AP(2,:),'k.');%真实2d点
    plot(Point_3d_R_AP_2d(1,:),Point_3d_R_AP_2d(2,:),'r.');%旋转以后的投影2d点
    title('配准前AP2D')
    axis equal
    
    figure(2),
    imshow(X_2D_AP);
    hold on;
    plot(Point_2d_AP(1,:),Point_2d_AP(2,:),'k.');%真实2d点
    plot(Point_3d_R_AP_2d(1,:),Point_3d_R_AP_2d(2,:),'r.');%旋转以后的投影2d点
    title('配准前AP2D')
    axis equal
    
    
    %%
    
    

    rotation_error_refine=acosd(0.5*(trace(INITROT'*rotationVectorToMatrix(r_out))-1));
    trans_error_refine=norm(INITTRANS-t_out);
    disp(['refine最优旋转:','(',num2str(r_out(1)),',',num2str(r_out(2)),',',num2str(r_out(3)),')'])
    disp(['refine最优平移:','(',num2str(t_out(1)),',',num2str(t_out(2)),',',num2str(t_out(3)),')'])
    disp(['refine time: ',num2str(refine_time),'s']);
    disp(['角度误差: ',num2str(rotation_error_refine),'°']);  
    disp(['平移误差: ',num2str(trans_error_refine),'(mm)']);  
    disp(['2d_tre: ',num2str(mean(tre_2d_refine_iter))]);
    disp('-------搜索变换完成--------')

%     save(['every_epoch_result/epoch',num2str(i),'_',num2str(case_index),'_result'])
    error_r(i)=error_degree;
    error_r_refine(i)=rotation_error_refine;
    error_t(i)=translation_error;
    error_t_refine(i)=trans_error_refine;
    time_every(i)=pose_time;
    time_every_refine(i)=refine_time;
    tre_mean_pose(i)=mean(tre_2d_pose_iter);
    tre_mean_refine(i)=mean(tre_2d_refine_iter);
    tre_max_pose(i)=max(tre_2d_pose_iter);
    tre_max_refine(i)=max(tre_2d_refine_iter);
    close all;
end
error_r_mean=mean(error_r);
error_r_refine_mean=mean(error_r_refine);
error_t_mean=mean(error_t);
error_t_refine_mean=mean(error_t_refine);
time_every_mean=mean(time_every);
time_every_refine_mean=mean(time_every_refine);
tre_2d_pose=mean(tre_mean_pose);
tre_2d_refine=mean(tre_mean_refine);
near_gold_idx=knnsearch(Point_2d_D_AP',Point_3d_D_AP_2d');
gold_2dtre=sqrt(sum((Point_2d_D_AP(:,near_gold_idx)-Point_3d_D_AP_2d).^2))*0.1540;
excel_all_pose=[error_r_mean,error_t_mean,tre_2d_pose,time_every_mean];
excel_all_refine=[error_r_refine_mean,error_t_refine_mean,tre_2d_refine,time_every_mean+time_every_refine_mean];
% save(['exp_result/result_case',num2str(case_index)])
end


