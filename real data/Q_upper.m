function [Q_u] = Q_upper(point_2d_rep,x1,y1,z1,point_2d_w,point_3d,epsilon,r_lu,center_rot,t,plane_z)
%Q_UPPER 

[b_c,b_r] = get_ball(point_3d,r_lu,center_rot,t);

b_c_T=reshape(b_c,[],1);
b_r_T=b_r';
point_2d_vector=b_c_T-point_2d_rep;%每一列都是同一个2D点与不同的球心对应的vector

point_2d_vector_reshape=reshape(point_2d_vector,3,[]);%每N列都是同一个2D点与不同的球心对应的vector
% corss_2d_3d=cross(point_2d_ex_norm_rep_reshape,point_2d_vector_reshape);
% corss_2d_3d_length=sqrt(sum(corss_2d_3d.^2));

corss_2d_3d_length=sqrt((y1.*point_2d_vector_reshape(3,:)-point_2d_vector_reshape(2,:).*z1).^2+(point_2d_vector_reshape(1,:).*z1-x1.*point_2d_vector_reshape(3,:)).^2+(x1.*point_2d_vector_reshape(2,:)-point_2d_vector_reshape(1,:).*y1).^2);

corss_2d_3d_length_reshape=reshape(corss_2d_3d_length,point_2d_w,[]);%每一行都是代表同一个球心
epsilon_3d=center_rot(3)/plane_z*epsilon;
ball_flag=sum(corss_2d_3d_length_reshape<(b_r_T+epsilon_3d),2);

Q_u=sum(ball_flag>0);


end

