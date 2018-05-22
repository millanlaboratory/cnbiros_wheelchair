clearvars; 


SelectedProfile     = 'profile1';
SelectedOdometry    = 'visual_odometry';

rootfolder  = 'wheelchair_velocity_profiles';
rootpath    = [rootfolder '/' SelectedProfile '/' SelectedOdometry '/01_trial/'];
pattern     = ['wheelchair.velocity.' SelectedProfile ]; 
extension   = '.bag';

VelocityTypes = {'linear.forward', 'linear.backward', 'angular.clockwise', 'angular.counterclockwise'};
NumVelocities = length(VelocityTypes);
ModalityTypes = {'command'};
NumModalities = length(ModalityTypes);

ResampleFreq = 10;      % [Hz]
TimePad = 5;            % [samples]
TimeShift = 10;         % [samples] Remove the first N samples to compute the maximum

files  = util_getfile(rootpath, extension, pattern);
nfiles = length(files); 


PosXIndex = 4;          % Pose position around x-axis
PosYIndex = 5;          % Pose position around y-axis
PosOIndex = 9;          % Pose orientation around z-axis
VelXIndex = 11;         % Twist linear  velocity around x-axis
VelYIndex = 12;         % Twist linear  velocity around y-axis
VelOIndex = 16;         % Twist angular velocity around z-axis

Vk = zeros(nfiles, 1);
Mk = zeros(nfiles, 1);

ts = cell(nfiles, 1);
max_velocities    = zeros(nfiles, 3);
max_accelerations = zeros(nfiles, 3);

for fId = 1:nfiles
   cfile =  files{fId};
   util_bdisp(['[io] - Import ' num2str(fId) '/' num2str(nfiles) ' bag: ' cfile]);
   
   % Extract modality info from filename
   cmodality = 0;
   for mId = 1:NumModalities
       if(contains(cfile, ModalityTypes{mId}))
           cmodality = mId;
       end
   end
   
   % Extract velocity type info from filename
   cvelocity = 0;
   for vId = 1:NumVelocities
       if(contains(cfile, VelocityTypes{vId}))
           cvelocity = vId;
       end
   end
   
   % Extract the current bag
   cbag = rosbag(cfile);
   cbagsel = select(cbag, 'Topic', '/odom');
   
    
   % Extract timeseries
   cts = timeseries(cbagsel);
   
   % Resample timeseries
   rts = resample(cts, cts.TimeInfo.Start:1/ResampleFreq:cts.TimeInfo.End);
   NumSamples = length(rts.Time);
   
   % Extract the data
   pose     = zeros(NumSamples, 3);
   velocity = zeros(NumSamples, 3);
   pose(:, 1) = rts.Data(:, PosXIndex);
   pose(:, 2) = rts.Data(:, PosYIndex);
   pose(:, 3) = rts.Data(:, PosOIndex);
   velocity(:, 1) = rts.Data(:, VelXIndex);
   velocity(:, 2) = rts.Data(:, VelYIndex);
   velocity(:, 3) = rts.Data(:, VelOIndex);
   
   % Smothing pose
   spose = zeros(size(pose));
   for cId = 1:size(pose, 2)
        spose(:, cId) = smooth(pose(:, cId), TimePad);
   end
   
   % Smothing velocity
   svelocity = zeros(size(velocity));
   for cId = 1:size(velocity, 2)
        svelocity(:, cId) = smooth(velocity(:, cId), TimePad);
   end
   
   % Computing acceleration
   acceleration = zeros(size(svelocity));
   for cId = 1:size(svelocity, 2)
       acceleration(:, cId) = diff([0; svelocity(:, cId)])./diff([rts.Time(1)-ResampleFreq;  rts.Time]);
   end
   
   % Smoothing acceleration
   sacceleration = zeros(size(acceleration));
   for cId = 1:size(acceleration, 2)
       sacceleration(:, cId) = smooth(acceleration(:, cId), TimePad);
   end
  
   
   
   for cId = 1:size(svelocity, 2)
        switch(cvelocity)
            case 1
                cmaxv = max(svelocity (TimeShift:end, cId));
                cmaxa = max(sacceleration(TimeShift:end, cId));
            case 3
                cmaxv = min(svelocity (TimeShift:end, cId));
                cmaxa = min(sacceleration(TimeShift:end, cId));
            case 2
                cmaxv = min(svelocity (TimeShift:end, cId));
                cmaxa = min(sacceleration(TimeShift:end, cId));
            case 4
                cmaxv = max(svelocity (TimeShift:end, cId));
                cmaxa = max(sacceleration(TimeShift:end, cId));
        end
        max_velocities(fId, cId) = cmaxv;
        max_accelerations(fId, cId) = cmaxa;
   end
   
   
   ts{fId} = rts;
   Vk(fId) = cvelocity;
   Mk(fId) = cmodality;
   
end



%% Re-arrange for plots
Coordinates = [1 1 3 3];
MeanVelModality = zeros(NumVelocities, NumModalities);
StdVelModality  = zeros(NumVelocities, NumModalities);

MeanVel = zeros(NumVelocities, 1);
StdVel  = zeros(NumVelocities, 1);
MeanAcc = zeros(NumVelocities, 1);
StdAcc  = zeros(NumVelocities, 1);

for vId = 1:NumVelocities
    for mId = 1:NumModalities
        MeanVelModality(vId, mId) = mean(max_velocities(Vk == vId & Mk == mId, Coordinates(vId)));
        StdVelModality(vId, mId) = mean(max_velocities(Vk == vId & Mk == mId, Coordinates(vId)));
    end
    
    MeanVel(vId) = mean(max_velocities(Vk == vId, Coordinates(vId)));
    StdVel(vId) = std(max_velocities(Vk == vId, Coordinates(vId)));
    
    MeanAcc(vId) = mean(max_accelerations(Vk == vId, Coordinates(vId)));
    StdAcc(vId) = std(max_accelerations(Vk == vId, Coordinates(vId)));
end

svelocities = [max_velocities(Vk == 3 | Vk == 4, 3); max_velocities(Vk == 1 | Vk == 2, 1)];
saccelerations = [max_accelerations(Vk == 3 | Vk == 4, 3); max_accelerations(Vk == 1 | Vk == 2, 1)];
sVk = [Vk(Vk == 3 | Vk == 4); Vk(Vk == 1 | Vk == 2)];
sMk = [Mk(Vk == 3 | Vk == 4); Mk(Vk == 1 | Vk == 2)];

%% Plots
fig1 = figure;
fig_set_position(fig1, 'Top');

ylimit = 0.9;


subplot(1, 2, 1);
boxplot(svelocities, sVk, 'labels', VelocityTypes, 'factorseparator', 1);
grid on;
title('Average velocities [m/s]');
ylabel('[m/s]');

util_bdisp('Average velocities:');
for vId = 1:NumVelocities
    disp(['  - ' VelocityTypes{vId} ': ' num2str(MeanVel(vId), 3) '+/-' num2str(StdVel(vId), 2) ' [m/s]']);
end

subplot(1, 2, 2);
boxplot(saccelerations, sVk, 'labels', VelocityTypes, 'factorseparator', 1);
grid on;
title('Average accelerations [m/s]');
ylabel('[m/s]');

util_bdisp('Average accelerations:');
for vId = 1:NumVelocities
    disp(['  - ' VelocityTypes{vId} ': ' num2str(MeanAcc(vId), 3) '+/-' num2str(StdAcc(vId), 2) ' [m/s]']);
end

%     
% for vId = 1:2
%     subplot(2, 2, vId);
%     boxplot(svelocities(sVk == vId), Mk(sVk==vId), 'labels', ModalityTypes); 
%     grid on;
%     title([VelocityTypes{vId} ': ' num2str(MeanVel(vId), 3) '+/-' num2str(StdVel(vId), 2) ' [m/s]']);
%     ylabel('[m/s]');
% end
% 
% suptitle([SelectedProfile ' - ' strrep(SelectedOdometry, '_', ' ') ' - Velocities']);
% 
% fig2 = figure;
% fig_set_position(fig2, 'All');
% 
% 
% 
% for aId = 1:NumVelocities
%     subplot(2, 2, aId);
%     boxplot(saccelerations(sVk == aId), Mk(sVk==aId), 'labels', ModalityTypes); 
%     grid on;
%     title([VelocityTypes{aId} ': ' num2str(MeanAcc(aId), 3) '+/-' num2str(StdAcc(aId), 2) ' [m/s^2]']);
%     ylabel('[m/s]');
% end
% 
% suptitle([SelectedProfile ' - ' strrep(SelectedOdometry, '_', ' ') ' - Accelerations']);

% 
% ModalitiesNames = {'joystick', 'command'};
% ResampleFreq = 10;
% TimePad = 5;
% position = [];
% velocity = [];
% max_velocity = zeros(nfiles, 3);
% Mk   = zeros(nfiles, 1);
% raw_ts = cell(nfiles, 1);
% res_ts = cell(nfiles, 1);
% for fId = 1:nfiles
%    cfile =  files{fId};
%    util_bdisp(['[io] - Import bag: ' cfile]);
%    
%    cbag = rosbag(cfile);
%    cbagsel = select(cbag, 'Topic', '/odom');
%    cnmsg = cbagsel.NumMessages;
%    
%    if(contains(cfile, 'joystick'))
%        cmod = 1;
%    elseif(contains(cfile, 'command'))
%        cmod = 2;
%    else
%        cmod = -1;
%        warning('chk:mod', 'Unknown modality for this file');
%    end
%    
%    cts = timeseries(cbagsel);
%    
%    raw_ts{fId} = cts;
%    res_ts{fId} = resample(cts, cts.TimeInfo.Start:1/ResampleFreq:cts.TimeInfo.End);
%    
%    position(fId).x = smooth(res_ts{fId}.Data(:, PosXIndex), TimePad);
%    position(fId).y = smooth(res_ts{fId}.Data(:, PosYIndex), TimePad);
%    position(fId).o = smooth(res_ts{fId}.Data(:, PosOIndex), TimePad);
%    velocity(fId).x = smooth(res_ts{fId}.Data(:, VelXIndex), TimePad);
%    velocity(fId).y = smooth(res_ts{fId}.Data(:, VelYIndex), TimePad);
%    velocity(fId).o = smooth(res_ts{fId}.Data(:, VelOIndex), TimePad);
%    
%    max_velocity(fId, :) = [max(velocity(fId).x) max(velocity(fId).y) max(velocity(fId).o)];
%  
%    
%    
%    
%    Mk(fId) = cmod;
%    
% end
% 
