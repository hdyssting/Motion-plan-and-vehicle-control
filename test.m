% clc;clear;
% ego_car = struct('x',0,'y',0);
% surrend_car = struct('x',48,'y',0);
% boundary = struct('low',-1.9,'high',5.7);
% lane_center = struct('yrctr1',0,'yrctr2',3.8);
% s = -10:1:60;
% d = -7.6:0.5:7.6;
% for i=1:length(s)
%     for j=1:length(d)
%         data(i,j) = struct('x',s(i),'y',d(j));
%         p(i,j) = potential_vehicle(data(i,j),surrend_car);
%         p_b(i,j) = potential_road_boundary(data(i,j),boundary);
%         p_lane(i,j) = potential_lane_center(data(i,j),lane_center,p(i,j));
%     end
% end
% 
% for i=1:length(d)
%     xx(i,:) = s;
% end
% for j=1:length(s)
%     yy(:,j) = d;
% end
% 
% figure;
% zz = p';
% contourf(xx,yy,zz,20,'lines','no');
% colorbar;
% 
% coff = cubicline(2,50,5,10);
% x = 0:50;
% for i=1:size(coff,1)
%     y = coff(i,1)*x.^3+coff(i,2)*x.^2+coff(i,3)*x+2;
%     plot(x,y);hold on;
% end
% 
% figure;
% zz_b = p_b';
% contourf(xx,yy,zz_b,20,'lines','no');
% colorbar;
% 
% figure;
% zz_lane = p_lane';
% contourf(xx,yy,zz_lane,20,'lines','no');
% colorbar;
% 
% figure;
% zz_total = zz+zz_b+zz_lane;
% contourf(xx,yy,zz_total,20,'lines','no');
% colorbar;
% grid on;
% 
% 
% originalcoor = struct('s',1:10,'d',zeros(1,10));
% reftrajectory = [0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0;
%                  0.0, -4.0,  1.0,  6.5,  8.0, 10.0,  6.0 ];
% coor = frenet_cartesian_conversion(reftrajectory,originalcoor);

