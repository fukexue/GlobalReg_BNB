function [best_tz] = pose_t(Point_2d,Point_3d,best_r,best_t,center_rot,ts_line,Project_center,plane_y,epsilon)
% find the translation in z

best_inlier=0;
best_tz=0;
R_best=rotationVectorToMatrix(best_r);
for t_z=ts_line
    Point_3d_R=R_best*(Point_3d-center_rot)+center_rot+[best_t(1:2);t_z];
    Point_2d_Y=Project_point_Y(Point_3d_R,Project_center,plane_y);
    near_pose_idx=knnsearch(Point_2d',Point_2d_Y');
    current_inlier=sum(sqrt(sum((Point_2d(:,near_pose_idx)-Point_2d_Y).^2))<epsilon);
    
    if current_inlier>best_inlier
        best_inlier=current_inlier;
        best_tz=t_z;
    end
end
end

