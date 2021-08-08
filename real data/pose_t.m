function [best_tz] = pose_t(Point_2d_LAT,Point_3d_AP,R_best,best_t,center_rot_AP,epsilon,ts_line,Tx_LAT,xs_LAT,ys_LAT,zs_LAT,Tx_AP,xs_AP,ys_AP,zs_AP,f_LAT,tans_s_LAT)
% find the translation in z

Point_2d_LAT_nopixel=Point_2d_LAT*0.1540-tans_s_LAT;
epsilon=epsilon*0.1540;
best_inlier=0;
best_tz=0;
for t_z=ts_line
    Point_3d_AP_recover=R_best*(Point_3d_AP-center_rot_AP)+center_rot_AP+[best_t(1:2);t_z];
    Point_3d_AP2LAT=inv(Tx_LAT)*(Tx_AP*[Point_3d_AP_recover;zeros(1,size(Point_3d_AP_recover,2))]+[xs_AP;ys_AP;zs_AP;1]-[xs_LAT;ys_LAT;zs_LAT;1]);
    Point_3d_AP2LAT2d_recover=ProjectPoint(Point_3d_AP2LAT,f_LAT);
    near_pose_idx=knnsearch(Point_2d_LAT_nopixel',Point_3d_AP2LAT2d_recover');
    current_inlier=sum(sqrt(sum((Point_2d_LAT_nopixel(:,near_pose_idx)-Point_3d_AP2LAT2d_recover).^2))<epsilon);
%     Point_3d_D_LAT_2d=ProjectPoint(Point_3d_D_LAT,f_LAT);
%     figure;
%     plot(Point_2d_LAT_nopixel(1,:),Point_2d_LAT_nopixel(2,:),'b.');%真实2d点
%     hold on;
%     plot(Point_3d_AP2LAT2d_recover(1,:),Point_3d_AP2LAT2d_recover(2,:),'g.');
%     plot(Point_3d_D_LAT_2d(1,:),Point_3d_D_LAT_2d(2,:),'r.');
%     axis equal
%     legend('真实2d','旋转后恢复2d')
%     title('t配准后2D')
    
    if current_inlier>best_inlier
        best_inlier=current_inlier;
        best_tz=t_z;
    end
end
end

