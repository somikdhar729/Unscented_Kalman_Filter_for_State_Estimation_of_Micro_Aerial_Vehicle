clear; % Clear variables
addpath('../data')
datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime, proj2Data] = init(datasetNum);
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = 0.01*eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %Just for saving state his.
prevTime = 0;
vel = proj2Data.linearVel;
angVel2 = proj2Data.angVel;
%% Calculate Kalmann Filter
for i = 1:length(sampledTime)
    %% FILL IN THE FOR LOOP
    dt = sampledData(i).t - prevTime;

    angVel = sampledData(i).omg;
    acc = sampledData(i).acc;
    [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt);
    
    z_t = [vel(i,:)';angVel2(i,:)'];
    [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);
    
    savedStates(:,i) = uCurr;
    prevTime = sampledTime(i);
    covarPrev = covar_curr;
    uPrev = uCurr;
    
end

plotData(savedStates, sampledTime, sampledVicon, 2, datasetNum);