%% Testing Teensy-Matlab Comms

%% Clear Everything if Neccessary

clc
clear all
close all

%% Connect to the HC-06 module
clc
bt = Bluetooth("HC-06",1);
fopen(bt);

%% Disconnect from the HC-06 module
clc
fclose(bt);

%% Define Struct to/from Teensy

% need to count bytes for receiving data
% see function at bottom for data types
% _10 means array of size 10 of data type

toTeensyInfo.startFlag = 'int8';
toTeensyInfo.time = 'single';
toTeensyInfo.voltInput = 'int16';
% toNanoInfo.referenceVec = 'int16_10';

fromTeensyInfo = {'TeensyCounter' 'int16'; 'time' 'single'; 'robotState' 'single_6'};
% fromNanoInfo = {'TeensyCounter' 'single'; 'time' 'single'};
% Should not have to change this unless there is something else we need

numBytes = 0;
sizeFTI = size(fromTeensyInfo);
for i = 1:sizeFTI(1)
    numBytes = numBytes + NumBytes(fromTeensyInfo{i,2});
end

fromTeensy.time = 0;
fromTeensy.TeensyCounter = -1;
fromTeensy.robotState = zeros(1,6);

%% Send/Receive from Nano
clc
close all

t = [];
tStart = -99;
robotStates = [];

simTime = 10;
tic
flushinput(bt);
flushoutput(bt);

% Start sending data
toTeensy.startFlag = int8(1);
toTeensy.time = single(0.0);
toTeensy.voltInput = int16(111);
% toNano.referenceVec = int16(4*ones(1,10));

sendData(toTeensy,bt); 

count = -1;
counterSum = 0; % 1 extra send on Teensy side
while (1)
    count = count + 1;
    counterSum = counterSum + count;
    currentTime = toc;
    if currentTime > simTime
        break
    end

    % Read Data
    fromTeensy = readData(fromTeensy,bt,fromTeensyInfo,numBytes);

    % Send Data
    toTeensy.startFlag = int8(1);
    toTeensy.time = single(currentTime);
    toTeensy.voltInput = int16(count);
    
    sendData(toTeensy,bt);    
    
    % store data in arrays for later
    if tStart == -99
       tStart = fromTeensy.time; 
    end
    
    t = [t (fromTeensy.time) - tStart];
    robotStates = [robotStates fromTeensy.robotState];
    disp(fromTeensy.TeensyCounter)
end
counterSum = counterSum - count;
disp(counterSum)

pause(0.2);

% Stop sending data
toTeensy.startFlag = int8(0);
toTeensy.time = single(0.0);
toTeensy.voltInput = single(0.0);
% toNano.referenceVec = int16(0*ones(1,10));

sendData(toTeensy,bt);

%% Plots

figure
subplot(231)
plot(t,robotStates(1,:),'r-')
xlim([0 simTime])
ylabel('x')
subplot(232)
plot(t,robotStates(2,:),'r-')
xlim([0 simTime])
ylabel('xDot')
subplot(233)
plot(t,robotStates(5,:),'g-')
xlim([0 simTime])
ylabel('motorV')
subplot(234)
plot(t,robotStates(3,:),'b-')
xlim([0 simTime])
ylabel('theta')
subplot(235)
plot(t,robotStates(4,:),'b-')
xlim([0 simTime])
ylabel('thetaDot')

figure
plot(t,robotStates(3,:),'b-')
xlim([0 simTime])
ylabel('theta')

%% Functions

function data = readData(data,bt,dataInfo,numBytes)
byteStream = uint8(fread(bt,numBytes));
    
index = 1;
for i = 1:size(dataInfo,1)
   num_bytes = NumBytes(dataInfo{i,2});
   eval_str = ['data.', dataInfo{i,1}, ' = typecast( uint8(byteStream(index:index+num_bytes-1)), ''', dataInfo{i,2}, ''' );'];
   eval(eval_str);
   index = index + num_bytes;
end
end

function [] = sendData(data,bt)
dataFields = fields(data);
buffer = [];
for i = 1:numel(dataFields)
    buffer = [buffer typecast(data.(dataFields{i}),'uint8')];
end

% disp(sprintf('%02X ',buffer));
fprintf(bt,char(buffer));
% fwrite(bt,buffer,'async');
end

function num_bytes = NumBytes(data_type)

  dtype = strsplit(data_type,'_');

  switch lower(dtype{1})
    case {'double', 'int64', 'uint64'}
      num_bytes = 8;
    case {'single', 'int32', 'uint32'}
      num_bytes = 4;
    case {'char', 'int16', 'uint16'}
      num_bytes = 2;
    case {'logical', 'boolean'}
      num_bytes = 1;
    case {'int8', 'uint8'}
      num_bytes = 1;
    otherwise
      warning('Class "%s" is not supported.', dtype{1});
      num_bytes = NaN;
  end
  
  if ( numel(dtype) == 2 )
    num_bytes = num_bytes * str2double(dtype{2});
  end
end