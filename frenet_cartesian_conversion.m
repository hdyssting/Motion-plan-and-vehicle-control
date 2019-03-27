function coorline = frenet_cartesian_conversion(reftrajectory,originalcoorline)

X = reftrajectory(1,:);
Y = reftrajectory(2,:);

dX = diff(X);
dY = diff(Y);
ds = sqrt(dX.^2+dY.^2);

s = [0, cumsum(ds)];

pps_X = csape(s,X);
pps_Y = csape(s,Y);

for i=1:length(originalcoorline.s)
    
    x_ref = fnval(pps_X,originalcoorline.s(i));
    dx_ref = fnval(fnder(pps_X,1),originalcoorline.s(i));
    ddx_ref = fnval(fnder(pps_X,2),originalcoorline.s(i));
    y_ref = fnval(pps_Y,originalcoorline.s(i));
    dy_ref = fnval(fnder(pps_Y,1),originalcoorline.s(i));
    ddy_ref = fnval(fnder(pps_Y,2),originalcoorline.s(i));
    
    yaw = atan2(dy_ref,dx_ref);
    k = (ddy_ref*dx_ref - ddx_ref*dy_ref)/(dx_ref^2 + dy_ref^2);
    
    
    coorline.x(i) = x_ref+originalcoorline.d(i)*cos(yaw+pi/2);
    coorline.y(i) = y_ref+originalcoorline.d(i)*sin(yaw+pi/2);
    
end

dx = diff(coorline.x);
dy = diff(coorline.y);
coorline.yaw = [atan2(dy,dx),atan2(dy(end),dx(end))];
coorline.ds = [sqrt(dx.^2+dy.^2),sqrt(dx(end).^2+dy(end).^2)];
coorline.k = [diff(coorline.yaw),0]./coorline.ds;
coorline.s = originalcoorline.s;
coorline.s_dot = originalcoorline.s_dot;
coorline.s_ddot = originalcoorline.s_ddot;
coorline.s_dddot = originalcoorline.s_dddot;
coorline.d = originalcoorline.d;
coorline.d_dot = originalcoorline.d_dot;
coorline.d_ddot = originalcoorline.d_ddot;
coorline.d_dddot = originalcoorline.d_dddot;
coorline.cost = originalcoorline.cost;



