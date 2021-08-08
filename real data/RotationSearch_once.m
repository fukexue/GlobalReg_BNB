function [best_r,branch,stop_flag,best_Q,max_Q_u,check_flag] = RotationSearch_once(point_2d_ex_rep,point_2d_ex_norm_rep_reshapex,point_2d_ex_norm_rep_reshapey,point_2d_ex_norm_rep_reshapez,point_2d_w,point_3d,epsilon,center_rot,t,branch,best_E,best_Q,best_Q_u,best_r_,plane_z,check_flag)
%ROTATIONSEARCH_ONECE 
stop_flag=0;
best_r=best_r_;
branch(:,branch(end,:)<best_E)=[];

if(size(branch,2)==0)
    best_r=[];
    branch=[];
    max_Q_u=0; 
    best_Q=0;
    return
end

[max_Q_u,index_best_bran]=max(branch(end,:));

if check_flag
    
    best_branch=branch(:,index_best_bran);

    best_bran_center=0.5*(best_branch(1:3)+best_branch(4:6));
    [Q] = Q_fun(point_2d_ex_rep,point_2d_ex_norm_rep_reshapex,point_2d_ex_norm_rep_reshapey,point_2d_ex_norm_rep_reshapez,point_2d_w,point_3d,epsilon,best_bran_center,center_rot,t,plane_z);



    if(Q>best_Q)
        best_Q=Q;best_r=best_bran_center;
    %     if norm(best_branch(1:3)-best_branch(4:6))<pi*90/180
    %         plotUpper(Point_2d,Point_2d_h,Point3d_to_center,epsilon,best_branch(1:6),center_rot,Project_center,plane_z,plane_y,t);
    %     end
    end

    if(max_Q_u==best_Q)

        if(max_Q_u==best_Q_u)
            stop_flag=1;
            disp('get best rotation!')
            disp(max_Q_u)
        else
            stop_flag=0; 
        end

        return;
    end


    branch(:,index_best_bran)=[];

    new_branch=Branch(best_branch(1:6));

    Q_u=zeros(1,8);
    for i=1:8
        [Q_u(i)] = Q_upper(point_2d_ex_rep,point_2d_ex_norm_rep_reshapex,point_2d_ex_norm_rep_reshapey,point_2d_ex_norm_rep_reshapez,point_2d_w,point_3d,epsilon,new_branch(:,i),center_rot,t,plane_z);
    end

    branch=[branch,[new_branch;Q_u]];
else
    if(best_Q==best_Q_u)
        stop_flag=1;
        disp(['get best rotation! inlier:',num2str(best_Q)])
        return; 
    end
end

end

