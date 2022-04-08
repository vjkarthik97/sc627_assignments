function flag = Velocity_Obstacle_Collision_check(Obstacle_x,Obstacle_y,Obstacle_V,Obstacle_R,Robot_x,Robot_y,Robot_V)

R = Obstacle_R;

A = (Obstacle_x - Robot_x)/(Obstacle_y - Robot_y);
B = (Obstacle_R^2)/(Obstacle_y - Robot_y);

a1 = ((-A*B) + ( (A*B)^2 - (1+A^2)*(B^2 - R^2) )^(0.5))/(1+A^2);
a2 = ((-A*B) - ( (A*B)^2 - (1+A^2)*(B^2 - R^2) )^(0.5))/(1+A^2);

b1 = -B - A*a1;
b2 = -B - A*a2;

xt1 = a1 + Obstacle_x;
yt1 = b1 + Obstacle_y;

xt2 = a2 + Obstacle_x;
yt2 = b2 + Obstacle_y;

VO_apex = [Robot_x Robot_y] + Obstacle_V;

test_candidate_velocity_vector = [Robot_x Robot_y] + Robot_V - VO_apex;
T1 = [xt1 yt1] + Obstacle_V - VO_apex;
T2 = [xt2 yt2] + Obstacle_V - VO_apex;

cross_product_T1 = cross([test_candidate_velocity_vector 0],[T1 0]);
cross_product_T2 = cross([test_candidate_velocity_vector 0],[T2 0]);

if(sign(cross_product_T1(3)*cross_product_T2(3)) <= 0)
    flag = 1;
else
    flag = 0;
end


end