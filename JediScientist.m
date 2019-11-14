%% STOP: DO NOT RUN THIS SCRIPT UNLESS YOU HAVE MuJoCo OPEN AND RUNNING %%
%% Background Information
% The following code is split into three distinct sections. You can run the
% entire script using the "Run Command" (green arrow) or run each section
% individually using the "Run Section Command" (green arrow over yellow
% background). The three sections are as follow:
%
% Section 1: Setting up MuJoCo
%   This section will start up connection between MATLAB and MuJoCo
% Section 2: Setting up Backyard Brains Arduino
%   This section will start up connection between MATLAB and Backyard
%   Brains Arduino
% Section 3: Recording and Controlling in Real-time
%   This section will continuously acquire signals from the Backyard Brains
%   Arduino, plot them, and use them to control the hand.
clear all; clc;
%% Section 1: Set Up Virtual Environment (MuJoCo)
% You should have MuJoCo open with a model loaded and running before
% starting this code!  If your code is crashing during this section, try
% the following: Close MuJoCo, Open MuJoCo, Open Model, Play MuJoCo, Run
% MATLAB Code.

try
    % CONNECT TO MUJOCO
    hx_close            %disconnect if previously connected
    hx_connect('')      %connects to MuJoCo with a local host (local host is specified by an empty single input)
    model_info = hx_robot_info       %displays info about the loaded model

    % SET UP MOVEMENTS
    movements = [0; ...     %(1) wrist pronation/supination
                 0; ...     %(2) wrist deviation
                 0; ...     %(3) wrist flexion/extension
                 1.62; ...  %(4) thumb abduction/adduction
                 0; ...     %(5) thumb flexion/extension (at base)
                 0; ...     %(6) thumb flexion/extension (at first joint - most proximal)
                 0; ...     %(7) thumb flexion/extension (at last joint - most distal)
                 0; ...     %(8) index abduction/adduction
                 0; ...     %(9) index flexion/extension
                 0; ...     %(10) middle flexion/extension
                 0; ...     %(11) ring flexion/extension
                 0; ...     %(12) pinky abduction/adduction
                 0;];       %(13) pinky flexion/extension

    % SET UP COMMAND
    command = struct('ref_pos', movements, ...        %reference position
                'ref_vel', zeros(13,1), ...
                'gain_pos', zeros(13,1), ...            
                'gain_vel', zeros(13,1), ...
                'ref_pos_enabled', 1, ...
                'ref_vel_enabled', 0, ...
                'gain_pos_enabled', 0, ...
                'gain_vel_enabled', 0);

    % The following code determine what control you provide to the hand model. 
    % Currently it's configured so that you have control over the fingers and
    % the wrist. The value you send to the hand determine the position of the
    % fingers and wrist. A value of 0 for C1 is an open hand (full extension of
    % all fingers) and a value of 1 for C1 is a closed hand (full flexion of all
    % fingers). Similarly, for C2, a value of 0 is an extended wrist, whereas a
    % value of 1 is a fully flexed wrist.

    % SET UP CONTROLLABLE DEGREES OF FREEDOM
    selectedDigits_Set1 = [5,6,7,9,10,11,13]; %fingers only (see indexes above)
    selectedDigits_Set2 = [3];                %wrist flexion only
    VREconnected = 1; %flag showing connection is complete
    disp('VRE connected! :D')
catch
    VREconnected = 0;
    disp('VRE failed to connect. :(')
end
%% Section 2: Set Up Arduino Board
% If you have trouble connecting to, or communicating with, the Arduino, 
% make sure the the JediScientist.ino file is loaded onto the Arduino!
try
    % SET UP ARDUINO COMMUNICATION
    if(~isempty(instrfindall))  %if some connection is already present:
        fclose(instrfindall)    %close it and
        delete(instrfindall)    %delete it
    end
    uno = serial(seriallist,'BaudRate',115200); %setup connection to arduino
    fopen(uno); %open connection and start communication
    ArduinoConnected = 1;
    disp('Arduino connected! :)')
catch
    ArduinoConnected = 0;
    disp('Arduino failed to connect. :(')
end
%% Section 3: Plot and Control in Real-time
% DETERMINE HOW LONG TO RUN DATA COLLECTION
Tmax = 30; %duration (in seconds) to record for (also scales plot)
displayPlots = true; %visualize EMG and control values in real-time
displayVRE = true; %see hand move in real-time
delay = 0.02; % SEE NOTES BELOW
% This delay setting is necessary for computers with limited processing
% power. Use the general guidelines to start:
% Delay = 0.2;  FOR VIRTUAL REALITY AND PLOTTING IN REAL-TIME
% Delay = 0.1;  FOR VIRTUAL REALITY ONLY
% Delay = 0.05; FOR PLOTTING ONLY

% SET UP PLOT
if(~ArduinoConnected)
    displayPlots = false;
end
fig = figure('units','normalized'); %open figure
if(displayPlots)
    set(fig,'outerposition',[0,0,.5,1]) %move figure to left half of screen
    subplot(411) %top plot is voltage 1
    v1 = animatedline;
    ylim([0,5]);
    xlim([0,Tmax])
    ylabel('Flexor Muscle Activity')
    xticks([]);
    subplot(412) %second plot is control value 1
    c1 = animatedline;
    ylim([-.25,1.25]);
    yticks([0,1]);
    yticklabels({'OPEN','CLOSE'})
    xlim([0,Tmax])
    ylabel('HAND')
    xticks([]);
    subplot(413) %second plot is voltage 2
    v2 = animatedline('Color','r');
    ylim([0,5]);
    xlim([0,Tmax])
    ylabel('Extensor Muscle Activity')
    xticks([]);
    subplot(414) %bottom plot is control value 2
    c2 = animatedline('Color','r');
    ylim([-.25,1.25]);
    yticks([0,1]);
    yticklabels({'FLEX','EXTEND'})
    xlim([0,Tmax])
    ylabel('WRIST')
    xlabel('Time (seconds)')
end
% INITIALIZATION
nSamp = 10000; %abritrarily large buffer for data
data = NaN(nSamp,2);
control = NaN(nSamp,2);
samp = 0;
prevSamp = 1;
previousTimeStamp = 0;
pause(0.5)
tic
while(toc < Tmax && ishandle(fig)) %run until time is out or figure closes
    % SAMPLE ARDUINO
    samp = samp + 1; %update sample count
    try
        data(samp,:) = fscanf(uno,'%d %d')/1023*5; %sample data, scale to voltage
    catch
    end
    timeStamp = toc; %get timestamp
    % UPDATE
    if(timeStamp > previousTimeStamp + delay) %limit rate
        % CALCULATE CONTROL VALUES
        try
            %% START OF YOUR CODE
             currentDataSample = data(samp,:); %gets current samples
             control = currentDataSample; %sets control values to current samples
            %% END OF YOUR CODE
        catch
        end
        % UPDATE PLOT
        if(displayPlots)
            addpoints(v1,timeStamp,max(data(prevSamp:samp,1)));
            addpoints(v1,timeStamp,min(data(prevSamp:samp,1)));
            addpoints(v2,timeStamp,max(data(prevSamp:samp,2)));
            addpoints(v2,timeStamp,min(data(prevSamp:samp,2)));
            addpoints(c1,timeStamp,control(samp,1))
            addpoints(c2,timeStamp,control(samp,2))
            drawnow limitrate %update the plot, but limit update rate to 20 Hz
        end
        % UPDATE HAND
        if(VREconnected && displayVRE) %if connected
            controlValues = control(samp,:);
            controlValues(controlValues>1) = 1; %check bounds
            controlValues(controlValues<-1) = -1; %check bounds
            controlValues(isnan(controlValues)) = 0; %check data
            % Threshold
            if(THRESH > 0)
                controlValues(controlValues > THRESH) = 1;
                controlValues(controlValues <= THRESH) = 0;
            end
            movements(selectedDigits_Set1) = controlValues(1); %set finger control to c1 value
            movements(selectedDigits_Set2) = .2-controlValues(2); %set wrist control to c2 value
            command.ref_pos = movements; %update command
            status = hx_update(command); %send command to model
        end
        previousTimeStamp = timeStamp;
        prevSamp = samp;
    end 
end