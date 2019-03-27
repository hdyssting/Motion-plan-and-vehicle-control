function p = potential_vehicle(ego_car,surround_car)

D0 = 10;
tao = 0.02;

sigma_x = (D0+tao*surround_car.s_dot);
sigma_y = 1.1;%?
A_ego = 30;
c=2;
for i=1:length(ego_car.s)
    p_detail(i) = A_ego*exp(-((ego_car.s(i)-surround_car.s(i))^2/(2*sigma_x^2)+(ego_car.d(i)-surround_car.d(i))^2/(2*sigma_y^2)).^c);
end
p = sum(p_detail);


