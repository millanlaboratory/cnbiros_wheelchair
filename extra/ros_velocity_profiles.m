clearvars; clc;


SelectedProfile = 'profile1';
SelectedOdometry = 'visual_odometry';
SelectedType = 'linear';
SelectedDirection = 'forward';

rootfolder = 'wheelchair_velocity_profiles';

rootpath = [rootfolder '/' SelectedProfile '/' SelectedOdometry '/'];
pattern = ['wheelchair.velocity.' SelectedType '.' SelectedDirection '.' SelectedProfile ]; 
extension = '.bag';

files  = util_getfile(rootpath, extension, pattern);
nfiles = length(files); 

PosXIndex = 4;          % Pose position around x-axis
PosYIndex = 5;          % Pose position around y-axis
PosOIndex = 9;          % Pose orientation around z-axis
VelXIndex = 11;         % Twist linear  velocity around x-axis
VelYIndex = 12;         % Twist linear  velocity around y-axis
VelOIndex = 16;         % Twist angular velocity around z-axis

ModalitiesNames = {'joystick', 'command'};
ResampleFreq = 10;
TimePad = 5;
position = [];
velocity = [];
max_velocity = zeros(nfiles, 3);
Mk   = zeros(nfiles, 1);
raw_ts = cell(nfiles, 1);
res_ts = cell(nfiles, 1);
for fId = 1:nfiles
   cfile =  files{fId};
   util_bdisp(['[io] - Import bag: ' cfile]);
   
   cbag = rosbag(cfile);
   cbagsel = select(cbag, 'Topic', '/odom');
   cnmsg = cbagsel.NumMessages;
   
   if(contains(cfile, 'joystick'))
       cmod = 1;
   elseif(contains(cfile, 'command'))
       cmod = 2;
   else
       cmod = -1;
       warning('chk:mod', 'Unknown modality for this file');
   end
   
   cts = timeseries(cbagsel);
   
   raw_ts{fId} = cts;
   res_ts{fId} = resample(cts, cts.TimeInfo.Start:1/ResampleFreq:cts.TimeInfo.End);
   
   position(fId).x = smooth(res_ts{fId}.Data(:, PosXIndex), TimePad);
   position(fId).y = smooth(res_ts{fId}.Data(:, PosYIndex), TimePad);
   position(fId).o = smooth(res_ts{fId}.Data(:, PosOIndex), TimePad);
   velocity(fId).x = smooth(res_ts{fId}.Data(:, VelXIndex), TimePad);
   velocity(fId).y = smooth(res_ts{fId}.Data(:, VelYIndex), TimePad);
   velocity(fId).o = smooth(res_ts{fId}.Data(:, VelOIndex), TimePad);
   
   max_velocity(fId, :) = [max(velocity(fId).x) max(velocity(fId).y) max(velocity(fId).o)];
 
   
   
   
   Mk(fId) = cmod;
   
end

