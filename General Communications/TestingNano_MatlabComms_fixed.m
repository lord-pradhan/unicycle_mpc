%% Testing Nano-Matlab Comms

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

%% Define Struct to/from Nano

% need to count bytes for receiving data
% see function at bottom for data types
% _10 means array of size 10 of data type

toNanoInfo.startFlag = 'int8';
toNanoInfo.time = 'single';
toNanoInfo.voltInput = 'single';
toNanoInfo.referenceVec = 'int16_10';

fromNanoInfo = {'time' 'single'; 'robotState' 'single_5'};
% Need to define numBytes to be how much data we expect from the nano
% Should not have to change this unless there is something else we need
numBytes = 24;

fromNano.time = 0;
fromNano.robotState = zeros(1,5);

%% Send/Receive from Nano
clc

% step amplitude (meters)
stepSize = -0.2;

t = [];
robotStates = [];

simTime = 15;
tic
flushinput(bt);
flushoutput(bt);

% Start sending data
toNano.startFlag = int8(1);
toNano.time = single(0.0);
toNano.voltInput = single(0.0);
toNano.referenceVec = int16(0*ones(1,10));

sendData(toNano,bt); 

count = -1;
while (1)
    count = count + 1;
    currentTime = toc;
    if currentTime > simTime
        break
    end

    
    % Read Data
    fromNano = readData(fromNano,bt,fromNanoInfo,numBytes);
    
    % store data in arrays for later
    t = [t fromNano.time];
    robotStates = [robotStates fromNano.robotState];

    % Send Data
    toNano.startFlag = int8(1);
    toNano.time = single(currentTime);
    toNano.voltInput = single(5.72);
    if count > 25
        stepSizeActual = stepSize;
        disp(stepSize)
    else
        stepSizeActual = 0;
    end
    toNano.referenceVec = int16(stepSizeActual*1000*ones(1,10));
    
    sendData(toNano,bt);    
end

t = t - t(1);

pause(0.2);

% Stop sending data
toNano.startFlag = int8(0);
toNano.time = single(0.0);
toNano.voltInput = single(0.0);
toNano.referenceVec = int16(0*ones(1,10));

sendData(toNano,bt); 

%% Plots
figure 
subplot(221)
hold on
plot(t,robotStates(1,:))
plot([0 15],[0.2 0.2])
ylabel('x')
subplot(222)
plot(t,robotStates(2,:))
ylabel('xDot')
subplot(223)
plot(t,robotStates(3,:))
ylabel('theta')
subplot(224)
plot(t,robotStates(4,:))
ylabel('thetaDot')

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