function [angle,range]=castrays(xr,yr,thetac,map,n,lidarMin,lidarRange,DEBUG)
% Note that both the lidarMin and lidarRange inputs should be in pixel
% values (20m = 200 pixels)
% Transforming robot position into pixel map position
xc=round((yr+10)/0.1);
yc=round((xr+10)/0.1);
thetac=thetac/pi*180;
MatrixSize = size(map);
angle = [];

% due to odometric errors, sometime converting the robot odometric position
% into pixels gives pixel values outside of the borders. Thus we cap xc and
% yc between [1,199]. 1 if xc,yc <=0 (min pixel value) and 199 if xc,yc>200
% (max pixel value).
if(xc<=0) 
    xc = 1;
end
if(yc<=0) 
    yc = 1 ;
end

if (xc>=200)
    xc=199;
end
if (yc>=200)
    yc=199;
end

% % Conversion factor from pixels to meters.
pixeltom=0.1;
if map(floor(yc),floor(xc))==1
    range=zeros(n,1);
    return;
end

if DEBUG == 1
figure(2);
hold off;
imshow(map);
hold on;
plot(xc,yc,'b*');
end

range=zeros(n+1,1);
angle = zeros(n+1,1);

% we changed it from 360/n to 270/n so that the raycast scope matches the
% lidar scope of angles.
thetastep=270/n;

for i=1:n+1
    pause(0.01)
    % the bracketed term is the offset needed to make theta match the lidar
    % beam angles with respect to the global coordinate frame.
    theta=-thetastep*i + (45 + thetac + thetastep);
    r=linspace(lidarMin,lidarRange,2000);
    x=xc+(r*cosd(theta));
    y=yc+(r*sind(theta));
    
    % Removing points out of map
    temp=[];
    for k=1:numel(x)
        if x(k)>MatrixSize(2) || y(k)>MatrixSize(1) || x(k)<=0 || y(k)<=0
            temp=[temp;k];
        end
    end
    
    x(temp)=[];
    y(temp)=[];

    if DEBUG == 1 
    figure(2);
    plot(x,y,'r');
    end

    % Computing Intersections
    xint=round(x);
    yint=round(y);
    % Correcting zero map indexing
    for l=1:numel(xint)
        if xint(l)==0
            xint(l)=1;
        end
        if yint(l)==0
            yint(l)=1;
        end
    end
    
    b=[];
    for j=1:numel(xint)
        b=[b;map(yint(j),xint(j))];
    end
    
    % if the castray doesn't hit any obstacles, give its range a value =
    % inf. this value will then be transformed into max sensor range in the
    % measurement model function before calculating weights and
    % likelihoods.
    for c=1:length(b)
        if b(c)~=0
            dist=inf;
            range(i)=dist;
            angle(i)=-theta/360*2*pi;
        end
    end
    
    %in presence of an obstacle
    ind=find(b==0);
    if ~isempty(ind)
    xb=x(ind(1));
    yb=y(ind(1));
    
    if DEBUG == 1 
    figure(2);
    plot(xb,yb,'g*');
    end

    dist=sqrt((xc-xb).^2 + (yc-yb).^2);
    range(i)=dist;   
    angle(i)=-theta/360*2*pi;
    
%     Note: the below comment is an alternative to the loop c = 1:length(b)
%     in case the raycast beam doesn't hit an obstacle. Both methods give
%     the same result.

%     else
%         dist = 30; % max lidar range is 30m.
%         range(i) = dist ; % raycast reads max lidar range = 30m.
%         angle(i) =-theta/360*2*pi;
    end

    pause(0.000001);
end

% Converting to m from pixels.
range=range*1*pixeltom;
% flipud only works since the range vector is a column vector. If it were a
% row vector, we should use fliplr.
% we use  flipud to make the elements of the range vector in a CW manner
% similat to the lidar ranges.
range=flipud(range);
range=range';
angle=flipud(angle);
angle=angle';
hold off
end