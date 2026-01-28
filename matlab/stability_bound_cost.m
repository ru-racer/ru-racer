function j = stability_bound_cost(beta,r,bound)

j=beta^2+r^2;
beta_bound=.2; %shoud computed according to the speed, and friction feedback
beta_m=.2;
yaw_rate_bound=.6;
if (beta>(beta_m*r+beta_bound) || beta<(beta_m*r-beta_bound)) || (r>yaw_rate_bound || r<-yaw_rate_bound);
    j=10*j;
else
    j=0;
end
j=min(j,2);