clc;clear;close all

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         Part I: Radar Simulation                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
fc = 24e9;    % 24GHz, 60GHz, 77GHz, etc.
c = 3e8;
lambda = c/fc;

%%
tm = 5e-4;   % Chirp Cycle
bw = 500e6;    % FMCW Bandwidth
range_max = 50;     % Max detection Range 1~100 meters
% range_max = fs*c/2/sweep_slope
v_max = 2.5;         % Max Velocity
% v_max = lambda/4/tm;
%
range_res = c/2/bw;
sweep_slope = bw/tm;
fr_max = range2beat(range_max,sweep_slope,c);
fd_max = speed2dop(2*v_max,lambda);
fb_max = fr_max+fd_max;
fs = max(2*fb_max,bw);

% Use Phased Array System Toolbox to generate an FMCW waveform
waveform = phased.FMCWWaveform('SweepTime',tm,'SweepBandwidth',bw,...
'SampleRate',fs);

%% Antenna Array
Nty = 2;                    % Tx antenna number y axis
Nry = 2;                    % Rx antenna number y axis
Nrz = 2;                    % Rx antenna number z axis

dty = lambda;
dry = lambda/2;
drz = lambda/2;

txarray = phased.ULA(Nty,dty);               % Tx Array 1x2
rxarray = phased.URA([Nrz Nry],[drz dry]);   % Rx Array 2x2
varray = phased.URA([Nrz Nty*Nry],[drz dry]);

figure;viewArray(txarray,'ShowIndex','All')
figure;viewArray(rxarray,'ShowIndex','All')
figure;viewArray(varray,'ShowIndex','All')
pause(0.1)

%%
transmitter = phased.Transmitter('PeakPower',0.001,'Gain',26);
receiver = phased.ReceiverPreamp('Gain',20,'NoiseFigure',4.5,'SampleRate',fs);

txradiator = phased.Radiator('Sensor',txarray,'OperatingFrequency',fc,...
'PropagationSpeed',c,'WeightsInputPort',true);
rxcollector = phased.Collector('Sensor',rxarray,'OperatingFrequency',fc,...
'PropagationSpeed',c);

rng(2020);
Nsweep = 64;               % Number of Chirps (IF signal) of this simulation
Dn = fix(fs/fr_max/2);      % Decimation factor
fs_d = fix(fs/Dn*tm)/tm;    % IF signal sample rate
% fs_d = 128/tm; %256000;
% Dn = fix(fs/fs_d);

%%
radar_s = phased.Platform;                % Stationary Radar


%% Targets
%simulation of 5 persons/targets at different positions, speed, directions.
target_dist = [5.23 18.23 31.20];              % Distance between sensor and 3 targets (meters)
target_speed = [-0.7 0.47 0.5];            % m/s, only simulate x axis direction motions.
target_az = [15 -15.1 40];                  % azimuth angles, degrees
target_zpos = [1.2 1.25 1.25];          % z axis positons
target_rcs = [0.3 0.3 0.3];            % RCS of targets
target_pos = [target_dist.*cosd(target_az);target_dist.*sind(target_az);target_zpos];   % initial positons

targets = phased.RadarTarget('MeanRCS',target_rcs,'PropagationSpeed',c,'OperatingFrequency',fc);
targetmotion = phased.Platform('InitialPosition',target_pos,...
    'Velocity',[target_speed;-0.1 0.1 0.1;0 0 0]);


%% Signal Propogation
% simulation of fog propagtion
channel = phased.LOSChannel('PropagationSpeed',c,...
'OperatingFrequency',fc,'SampleRate',fs,'TwoWayPropagation',true,...
'SpecifyAtmosphere',true,'Temperature',15,...
'LiquidWaterDensity',500);

%%
% Generate Time Domain Waveforms of Chirps
% xr is the data received at rx array

chirp_len = fix(fs_d*waveform.SweepTime);
xr = complex(zeros(chirp_len,Nry,Nsweep));
w0 = [0;1];  % weights to enable/disable radiating elements

disp('The simulation will take some time. Please wait...')
for m = 1:Nsweep
    if mod(m,1)==0
        disp([num2str(m),'/',num2str(Nsweep)])
    end
    
    % Update radar and target positions
    [radar_pos,radar_vel] = radar_s(waveform.SweepTime);
    [tgt_pos,tgt_vel] = targetmotion(waveform.SweepTime);
    [~,tgt_ang] = rangeangle(tgt_pos,radar_pos);
    
    % Transmit FMCW waveform
    sig = waveform();
    txsig = transmitter(sig);
    
    % Toggle transmit element
    w0 = 1-w0;
    txsig = txradiator(txsig,tgt_ang,w0);
    
    % Propagate the signal and reflect off the target
    txsig = channel(txsig,radar_pos,tgt_pos,radar_vel,tgt_vel);
    txsig = targets(txsig);
    
    % Dechirp the received radar return
    rxsig = rxcollector(txsig,tgt_ang);
    rxsig = receiver(rxsig);
    dechirpsig = dechirp(rxsig,sig);
    
    % Decimate the return to reduce computation requirements
    for n = size(xr,2):-1:1
        xr(:,n,m) = decimate(dechirpsig(1:chirp_len*Dn,n),Dn,'FIR');
    end
end

range_res = range_res*size(dechirpsig,1)/Dn/size(xr,1);

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         Part II: Signal Processing                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
% Virtual Array Chirp Data
Nvsweep = Nsweep/2;
xr1 = xr(:,:,1:2:end);
xr2 = xr(:,:,2:2:end);

xrv = cat(2,xr1,xr2);   % Use 2T4R

%%
% FFT points
nfft_r = 2^nextpow2(size(xrv,1));
nfft_d = 2^nextpow2(size(xrv,3));
nfft_mul = 2;

% RDM Algorithm
rngdop = phased.RangeDopplerResponse('PropagationSpeed',c,...
    'DopplerOutput','Speed','OperatingFrequency',fc,'SampleRate',fs_d,...
    'RangeMethod','FFT','PRFSource','Property',...
    'RangeWindow','Hann','PRF',1/(Nty*waveform.SweepTime),...
    'SweepSlope',waveform.SweepBandwidth/waveform.SweepTime,...
    'RangeFFTLengthSource','Property','RangeFFTLength',nfft_r*nfft_mul,...
    'DopplerFFTLengthSource','Property','DopplerFFTLength',nfft_d*nfft_mul,...
    'DopplerWindow','Hann');

% RD Map
% Range-Energy Calibration
[resp,r,sp] = rngdop(xrv);
for k=size(resp,1)/2+1:size(resp,1)
    resp(k,:,:) = resp(k,:,:) * (k-size(resp,1)/2)^3;
end

figure;plotResponse(rngdop,squeeze(xrv(:,1,:)));axis([-v_max v_max 0 range_max-0.05]);title('Fog');
