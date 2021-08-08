function Point_2D=ProjectPoint(Point_3D,f)
%Project points to a plane from origin point
%
%f        :focal length( x-y plane : z=f )
%Point_3D :3D points
%Point-2D :2D points
%------------------------------------------

Point_2D=zeros(2,size(Point_3D,2));
Point_2D(1,:)=Point_3D(1,:)./Point_3D(3,:).*f;
Point_2D(2,:)=Point_3D(2,:)./Point_3D(3,:).*f;

end