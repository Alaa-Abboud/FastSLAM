function [newCloud] = Sample_motion_model(prevOdomPose, newOdomPose, oldCloud)

alphas = [1 1 1 1];
prevX = prevOdomPose(1);
prevY = prevOdomPose(2);
prevTheta = prevOdomPose(3);

newX = newOdomPose(1);
newY = newOdomPose(2);
newTheta = newOdomPose(3);

% Works for any number of points in the cloud - A Generalization
[N,m] = size(oldCloud); % in our case, N = 40 points.

deltaRot1 = atan2(newY - prevY, newX - prevX) - prevTheta;
deltaTrans = sqrt((newY - prevY)^2 + (newX - prevX)^2 );
deltaRot2 = newTheta - prevTheta - deltaRot1;

for i= 1:N
    
    deltaRot1_hat = deltaRot1 - CalcSample(alphas(1) * deltaRot1 + alphas(2) * deltaTrans);
    deltaTrans_hat = deltaTrans - CalcSample(alphas(3) * deltaTrans + alphas(4) * (deltaRot1 + deltaRot2));
    deltaRot2_hat = deltaRot2 - CalcSample(alphas(1) * deltaRot2 + alphas(2) * deltaTrans);
    
    X_err = oldCloud(i,1) + deltaTrans_hat * cos(oldCloud(i,3) + deltaRot1_hat);
    Y_err = oldCloud(i,2) + deltaTrans_hat * sin(oldCloud(i,3) + deltaRot1_hat);
    Theta_err = oldCloud(i,3) + deltaRot1_hat + deltaRot2_hat;
    
    newCloud(i,:) = [X_err, Y_err, Theta_err];
    
end

end
