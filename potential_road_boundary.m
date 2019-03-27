function p = potential_road_boundary(ego_car,road_boundary)

Arb = 40;
sigmarb = 0.4;

if ego_car.y>=road_boundary.low
    p1 = Arb*exp(-(ego_car.y-road_boundary.low)^2/(2*sigmarb^2));
else
    p1 = Arb;
end

if ego_car.y<=road_boundary.high
    p2 = Arb*exp(-(ego_car.y-road_boundary.high)^2/(2*sigmarb^2));
else
    p2 = Arb;
end

p = p1+p2;

