function[p]=measurement_model_map(lidRanges,lidAngles,X,m)
% X is a sample point in the cloud.
% m is its corresponding map.
w_hit=0.95;
w_short=0.01;
w_max=0.02;
w_rand=0.02;
w=[w_hit,w_short,w_max,w_rand]; % Weights of PDFs
lambda=0.3; 
sensor=[30,1]; % Zmax = 30 meters and STD = 1m

%20x20meters with 200x200pixels-Each pixel is 0.1m 

Q=21; %nb of beams
[rayAngles,rayRanges]=castrays(X(1),X(2),pi-X(3),m,Q-1,5,200,1);
%rayRanges is inputted to flipud in the raycast fct to become clockwise
%like lidRanges.
% select the lidar beams that match the angles of the raycasted beams.
z(1)=lidRanges(1);
for i=2:20 %k-1
    z(i)=lidRanges((i-1)*36+1);
end
z(21)=lidRanges(720);
%z(1) and z(21) are the lidar beams at the 2 extremes -235 and +235

%% Calculation Probabilities / Likelihood
zmax=sensor(1);
p=1;
for j=1:Q
    % Convert all raycast and lidar readings that are equal to inf to Zmax=
    % max lidar range.
    
    if z(j)==inf
        z(j)=zmax;
    end
    if rayRanges(j)==inf
        rayRanges(j)=zmax;
    end
    
    % Probability of Hit
    c=1/sqrt(2*pi*sensor(2)^2);
    N=c*exp(-((z(j)-rayRanges(j))^2)/(2*sensor(2)^2));
    if z(j)>0 && z(j)<zmax
        n=1;
        pHit=N*n;
    else
        pHit=0;
    end
    
    % Probability of short
    n=1/(1-exp(-lambda*rayRanges(j)));
    if z(j)>0 && z(j)<rayRanges(j)
        pShort=n*lambda*exp(-lambda*z(j));
    else
        pShort=0;
    end
    
    % Probability of Faulty Measurement
    if z(j)==zmax
        pMax=1;
    else
        pMax=0;
    end
    
    % Probability of Unexplained Errors
    if z(j)>0 && z(j)<zmax
        pRand=1/zmax;
    else
        pRand=0;
    end
    
    % Combination
    pf=w(1)*pHit+w(2)*pShort+w(3)*pMax+w(4)*pRand;
    p=p*pf*100;
end  
p = p*1000;
end