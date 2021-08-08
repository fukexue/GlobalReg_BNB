function [best_r,best_t] = pose(Point_2d,Point_3d,epsilon,center_rot,ts,plane_z,tans_s)
%POSE find the best rotation and translation
%   point_2d:       input 2D point
%   point_3d:       input 3D point
%   epsilon:        input tolerable error
%   center_rots:    input many rotation center
%   palne_z:        input X-Y plane position: z=palne_z
%   plot_flag:      input display level,0:off|1:show upper and lower
%                   |2:show all
%   best_r        output global optimal rotation�� 2d=P(R*(3d-t)+t)
%   best_t

point_2d_w=size(Point_2d,2);
point_3d_w=size(Point_3d,2);

if point_2d_w>point_3d_w
    Point_3d(:,size(Point_3d,2)+1:point_2d_w)=NaN;
else
    point_2d_w=point_3d_w;
    Point_2d(:,size(Point_2d,2)+1:point_3d_w)=NaN;
end

point_2d_ex=[Point_2d*0.1540-tans_s;ones(1,point_2d_w)*plane_z];
epsilon=epsilon*0.1540;

% figure;
% plot3(Point_3d(1,:),Point_3d(2,:),Point_3d(3,:),'r.');
% hold on;
% for j=1:size(point_2d_ex,2)
%     plot3([0,point_2d_ex(1,j)],[0,point_2d_ex(2,j)],[0,point_2d_ex(3,j)],'b');
% end

point_2d_ex_rep=repmat(point_2d_ex,point_2d_w,1);
point_2d_ex_norm=point_2d_ex./sqrt(sum(point_2d_ex.^2));
point_2d_ex_norm_rep=repmat(point_2d_ex_norm,point_2d_w,1);
point_2d_ex_norm_rep_reshape=reshape(point_2d_ex_norm_rep,3,[]);
point_2d_ex_norm_rep_reshapex=point_2d_ex_norm_rep_reshape(1,:);
point_2d_ex_norm_rep_reshapey=point_2d_ex_norm_rep_reshape(2,:);
point_2d_ex_norm_rep_reshapez=point_2d_ex_norm_rep_reshape(3,:);


num_rot_cent=size(ts,2);
% branches=repmat({[-pi,-pi,-pi,pi,pi,pi,size(Point_3d,2)]'},num_rot_cent,1);
branches=repmat({[-pi*1/18,-pi*1/18,-pi*1/18,pi*1/18,pi*1/18,pi*1/18,size(Point_3d,2)]'},num_rot_cent,1);
best_rs=cell(num_rot_cent,1);
best_Qs=zeros(num_rot_cent,1);
stop_flags=zeros(num_rot_cent,1);
check_flag=ones(num_rot_cent,1);%检查标准
best_E=0;
best_Q_us=zeros(num_rot_cent,1);
best_Q_u=size(Point_3d,2);
iter=0;



while(1)
    for i=1:num_rot_cent %parfor for Multithreading
        if(size(branches{i},2)~=0)
            [best_rs{i},branches{i},stop_flags(i),best_Qs(i),best_Q_us(i),check_flag(i)] = RotationSearch_once(point_2d_ex_rep,point_2d_ex_norm_rep_reshapex,point_2d_ex_norm_rep_reshapey,point_2d_ex_norm_rep_reshapez,point_2d_w,Point_3d,epsilon,center_rot,ts(:,i),branches{i},best_E,best_Qs(i),best_Q_u,best_rs{i},plane_z,check_flag(i));
        end
    end
    if(sum(stop_flags)>0)
        k=find(stop_flags==1);
        best_r=best_rs{k(1)};
        best_t=ts(:,k(1));
        return;
    end
    
    best_Q_u=max(best_Q_us);
    best_E=max(best_Qs);
    iter=iter+1;
end

end

