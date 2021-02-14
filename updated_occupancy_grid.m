function[m]=updated_occupancy_grid(lidRanges,lidAngles,xt,mt_1)
beta=3*pi/180;
alpha=0.05;
zmax=30;
Map=mt_1;
% playpen map is 20x20 meters
% 100 is white (free space, no obstacles)
% 0 is black (obstacle / occupied)

%for the same position repeat the map update
for b=1:5
    %loop over all pixels of map
    for I=1:200
        for J=1:200
            xi=(I-100)*0.1-0.05; % 0.1 resolution and the 0.05 offset is to reach center of pixel.
            yi=(J-100)*0.1-0.05;
            r=sqrt((xi-xt(1))^2+(yi-xt(2))^2);
            phi=atan2(yi-xt(2),xi-xt(1))-xt(3);
            [d k]=min(abs(phi-lidAngles)); 

            if r>min(zmax,lidRanges(k)+alpha/2) || abs(phi-lidAngles(k))>beta/2
                Map(I,J)=Map(I,J); % keep the map as is   

            elseif lidRanges(k)<zmax && abs(r-lidRanges(k))<alpha/2 && Map(I,J)>=10
                Map(I,J)=Map(I,J)-10; % deduct 10 to show more likelihood of presence of obstacle (darker shade)

            elseif r<=lidRanges(k) && Map(I,J)<=95
                Map(I,J)=Map(I,J)+5; % add 5 to show more likelihood of free space (lighter shade)
            end

        end
    end
end

% Threshold for a (0,100) black/white map without different degrees of
% shade.
bw_map=Map;
for i=1:200  
    for j=1:200
        if bw_map(i,j)<=80 % 80 is our chosen threshold. choose based on convinience and experimentation.
            bw_map(i,j)=0;
        else
            bw_map(i,j)=100;
        end
    end
end
m=bw_map;
end