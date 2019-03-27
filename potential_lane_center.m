function p = potential_lane_center(ego_car,lane_center,p_vehicle)

Actr = 7;
sigmactr = 0.2;
p_veh_thres = 1;

if p_vehicle<p_veh_thres
    p1 = -Actr*exp(-(ego_car.y-lane_center.yrctr1)^2/(2*sigmactr^2));
    p2 = -Actr*exp(-(ego_car.y-lane_center.yrctr2)^2/(2*sigmactr^2));
    p = p1+p2;
else
    p = 0;
end


