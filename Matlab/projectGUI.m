function varargout = projectGUI(varargin)
% projectGUI MATLAB code for projectGUI.fig
%      projectGUI, by itself, creates a new projectGUI or raises the existing
%      singleton*.
%
%      H = projectGUI returns the handle to a new projectGUI or the handle to
%      the existing singleton*.
%
%      projectGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in projectGUI.M with the given input arguments.
%
%      projectGUI('Property','Value',...) creates a new projectGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before projectGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to projectGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help projectGUI

% Last Modified by GUIDE v2.5 20-Apr-2016 19:55:59

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @projectGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @projectGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before projectGUI is made visible.
function projectGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to projectGUI (see VARARGIN)

% Choose default command line output for projectGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% check com ports
set(handles.SelComPort, 'String', getAvailableComPort());
global iK_L1_store iK_L2_store iK_L3_store
iK_L1_store = 0;
iK_L2_store = 0;
iK_L3_store = 0;

% display image of sensor
axes(handles.imuImage)
matlabImage = imread('sensorOrientation.jpg');
image(matlabImage)
axis off
axis image

% UIWAIT makes projectGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = projectGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function popupmenu_ArdSer_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function edit_link1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function edit_link2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function edit_link3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton_StartRec.
function pushbutton_StartRec_Callback(hObject, eventdata, handles)
%% ===================== Recording Setup =====================
% Set up global variables
global recording dataList s;   % allows recording to be a global variable
recording = 1;      % indicates recording has started
dataList = [];      % data to store Arduino readings

% Plot to the plotFinger window
axes(handles.plotFinger);

% Get link lengths
link1Length = str2double(get(handles.edit_link1,'String'));
link2Length = str2double(get(handles.edit_link2,'String'));
link3Length = str2double(get(handles.edit_link3,'String'));

% create initial plot (not saved in recording)
h = plotFinger3D(link1Length, link2Length, link3Length, 0,0,0,0);

% starts timer
tic;

%% ===================== Recording Loop ======================
while recording == 1
    % send arduino data request
    fprintf(s, '%s', 'a');
    pause(0.01);
    
    % read the arduino response
    sensor1 = cell2mat(textscan(fscanf(s),'%n %n %n'));
    sensor2 = cell2mat(textscan(fscanf(s),'%n %n %n'));
    sensor3 = cell2mat(textscan(fscanf(s),'%n %n %n'));
    sensor4 = cell2mat(textscan(fscanf(s),'%n %n %n'));
    
    % =============== DETERMINE JOINT ANGLES ===================
    % get rotation matrix of each euler angle
    rot1 = euler2rot(sensor1);
    rot2 = euler2rot(sensor2);
    rot3 = euler2rot(sensor3);
    rot4 = euler2rot(sensor4);

    % Compute theta1 and theta2
    rot1to2 = rot1' * rot2;
    theta1 = rad2deg(atan2(-rot1to2(1,2), rot1to2(2,2)));
    theta2 = rad2deg(atan2(-rot1to2(3,1), rot1to2(3,3)));

    % Compute theta3
    rot2to3 = rot2' * rot3;
    theta3 = rad2deg(atan2(-rot2to3(3,1), abs(rot2to3(3,3))));

    % Compute theta4
    rot3to4 = rot3' * rot4;
    theta4 = rad2deg(atan2(-rot3to4(3,1), abs(rot3to4(3,3))));
    % ==========================================================
    
    % plot the finger ball/stick model
    [x, y, z] = updatePlotFinger3D(link1Length, link2Length, link3Length, theta1, theta2, theta3, theta4);
    h.XData = x;
    h.YData = y;
    h.ZData = z;
    
    % Display the Joint Values
    set(handles.pbQ1,'String',num2str(round(theta1, 1)));
    set(handles.pbQ2,'String',num2str(round(theta2, 1)));
    set(handles.pbQ3,'String',num2str(round(theta3, 1)));
    set(handles.pbQ4,'String',num2str(round(theta4, 1)));
    
    % Display the Fingertip XYZ coordinates
    set(handles.stX,'String',num2str(round(x(5), 1)));
    set(handles.stY,'String',num2str(round(y(5), 1)));
    set(handles.stZ,'String',num2str(round(z(5), 1)));
    
    % write the data to a continuously increasing list
    dataList(end+1,:) = [theta1, theta2, theta3, theta4, x, y, z];
end

%% =============== Store data and compute run time ===============
% end timer
recordTime = toc;

% create time steps column based upon recordTime
[numReadings, ~] = size(dataList);
timestamps = linspace(0, recordTime, numReadings);
timestamps = timestamps';

% add time stamps to columnized data
dataList = [timestamps, dataList];

% --- Executes on button press in pushbutton_EndRec.
function pushbutton_EndRec_Callback(hObject, eventdata, handles)
global recording
recording = 0;
display('Recording Ended')

% --- Executes on button press in pushbutton_saveRec.
function pushbutton_saveRec_Callback(hObject, eventdata, handles)
global dataList;
if size(dataList) ~= 0;
    [file,path] = uiputfile('dataList.mat','Save Data File');
    if isnumeric(file)  && isnumeric(path)
        display('Save Cancelled');
    else
        save([path, file],'dataList');
        display(['Recording "', file, '" Saved To Working Directory']);
    end
else
    display('No Recording to save');
end

% --- Executes on button press in pushbutton_ResetRec.
function pushbutton_ResetRec_Callback(hObject, eventdata, handles)
display('refresh recorded plot')
global dataList
dataList = [];

% --- Executes on button press in pbConnect.
function pbConnect_Callback(hObject, eventdata, handles)
global s;
list = get(handles.SelComPort,'String');
val = get(handles.SelComPort,'Value');
port = list{val};

s = serial(port);
disp(get(hObject,'String'))
baudRate = str2double(get(handles.efBaudRate,'String'));
set(s,'BaudRate',baudRate);
fopen(s);
pause(1);

status = fscanf(s);
while(not(status == 'GO!\n')) %Wait for ready signal from arduino 
    %wait for 'GO\n'
    clc;
    status = fscanf(s);
    disp(sprintf('Status: %s',status));
end

set(handles.pbConnect,'Enable','off');
set(handles.pbDisconnect,'Enable','on');
set(handles.pushbutton_StartRec,'Enable','on');
set(handles.pushbutton_EndRec,'Enable','on');
set(handles.pushbutton_saveRec,'Enable','on');
set(handles.pushbutton_ResetRec,'Enable','on');


% --- Executes on button press in pbDisconnect.
function pbDisconnect_Callback(hObject, eventdata, handles)
global s;
fprintf(s, '%s', 'R'); %Reset the arduino before disconnecting
fclose(s);
delete(s);
clear s;
pause(1);
set(handles.pbDisconnect,'Enable','off');
set(handles.pbConnect,'Enable','on');


% --- Executes during object creation, after setting all properties.
function efBaudRate_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pbLoad.
function pbLoad_Callback(hObject, eventdata, handles)
global loadList playbackPlot IKPlot
[FileName,PathName,~] = uigetfile('*.mat','Load the Recording');    % gets the filename and path of a selected file
if isnumeric(FileName) && isnumeric(PathName)
    display('Load Cancelled');
else
    tempStruct = load([PathName, FileName]);  % creates a structure
    display(FileName(1:size(FileName,2)-4))
    % loadList = getfield(tempStruct, FileName(1:size(FileName,2)-4));    % makes loadList the loaded .mat file
    loadList = getfield(tempStruct, 'dataList');    % makes loadList the loaded .mat file
    display(['Recording "', FileName, '" Loaded']);
    
    %% determine length
    length = loadList(end,1);
    set(handles.sPlayback,'Max',length);

    %% create initial plot
    axes(handles.playbackAxes);
    playbackPlot = plot3(loadList(1, 6:10), loadList(1, 11:15), loadList(1, 16:20), 'ko-');
    
    hold on
    IKPlot = plot3([0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],'ko-');
    hold off
    
    grid on;
    xlabel('x mm');
    ylabel('y mm');
    zlabel('z mm');
    
    % find max length
    tipTo4 = sqrt((loadList(1, 10) - loadList(1, 9))^2 + (loadList(1, 15) - loadList(1, 14))^2 + (loadList(1, 20) - loadList(1, 19))^2);
    dist4to3 = sqrt((loadList(1, 9) - loadList(1, 8))^2 + (loadList(1, 14) - loadList(1, 13))^2 + (loadList(1, 19) - loadList(1, 18))^2);
    dist3to2 = sqrt((loadList(1, 8) - loadList(1, 7))^2 + (loadList(1, 13) - loadList(1, 12))^2 + (loadList(1, 18) - loadList(1, 17))^2);
    
    maxLength = tipTo4 + dist4to3 + dist3to2;
    axis ([-1.5*maxLength 1.5*maxLength -1.5*maxLength 1.5*maxLength -1.5*maxLength 1.5*maxLength]); axis vis3d;
    cameratoolbar('show') 
    
    %% activate slider
    set(handles.sPlayback,'Enable','on');
    
    %% activate inverse kinematics controls
    set(handles.rbOn, 'Enable', 'on');
    set(handles.rbOff, 'Enable', 'on');
    set(handles.efIKLink1, 'Enable', 'on');
    set(handles.efIKLink2, 'Enable', 'on');
    set(handles.efIKLink3, 'Enable', 'on');
end


% --- Executes during object creation, after setting all properties.
function SelComPort_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pbCOMUpdate.
function pbCOMUpdate_Callback(hObject, eventdata, handles)
set(handles.SelComPort, 'String', getAvailableComPort());


% --- Executes on button press in pb_GoRecord.
function pb_GoRecord_Callback(hObject, eventdata, handles)
set(handles.uipanel_Playback,'visible','off')
set(handles.uipanel_Record,'visible','on')
set(handles.pb_goPlayback,'Enable','on');
set(handles.pb_GoRecord,'Enable','off');

% --- Executes on button press in pb_goPlayback.
function pb_goPlayback_Callback(hObject, eventdata, handles)
set(handles.uipanel_Playback,'visible','on')
set(handles.uipanel_Record,'visible','off')
set(handles.pb_goPlayback,'Enable','off');
set(handles.pb_GoRecord,'Enable','on');


%% Inverse Kinematics
% --- Executes on button press in rbOn.
function rbOn_Callback(hObject, eventdata, handles)
global IKtable iK_L1_store iK_L2_store iK_L3_store
% run the inverse Kinematics (getInvKinLookupTable) (getJointAngles)
iK_L1 = str2double(get(handles.efIKLink1,'String'));
iK_L2 = str2double(get(handles.efIKLink2,'String'));
iK_L3 = str2double(get(handles.efIKLink3,'String'));

if (iK_L1==iK_L1_store) && (iK_L1==iK_L1_store) && (iK_L1==iK_L1_store)
    % dont create a new table
    set(handles.efIKLink1, 'Enable', 'off');
    set(handles.efIKLink2, 'Enable', 'off');
    set(handles.efIKLink3, 'Enable', 'off');
    set(handles.stTableStatus,'String','Table Created');
else
    % create a new table
    iK_L1_store = iK_L1;
    iK_L2_store = iK_L2;
    iK_L3_store = iK_L3;
    
    set(handles.efIKLink1, 'Enable', 'off');
    set(handles.efIKLink2, 'Enable', 'off');
    set(handles.efIKLink3, 'Enable', 'off');
    set(handles.stTableStatus,'String','Creating Table');
    drawnow

    IKtable = getInvKinLookupTable(iK_L1, iK_L2, iK_L3, 1);
    set(handles.stTableStatus,'String','Table Created');
end



% --- Executes on slider movement.
function sPlayback_Callback(hObject, eventdata, handles)
global loadList playbackPlot IKPlot IKtable

% read the slider value
frameTime = get(handles.sPlayback,'Value');

% create initial variables
timeCheck = 0;
frame = 1;

% determine frame based on closest time of slider
while timeCheck < frameTime
   frame = frame + 1;
   timeCheck = loadList(frame,1);
end

% update the time indicator (static text)
set(handles.stTime,'String',num2str(loadList(frame,1)));

% read inverse kinematics inputs
iK_L1 = str2double(get(handles.efIKLink1,'String'));
iK_L2 = str2double(get(handles.efIKLink2,'String'));
iK_L3 = str2double(get(handles.efIKLink3,'String'));
rbOn = get(handles.rbOn,'Value');

% find xyz point of the tip
tipX = loadList(frame,10);
tipY = loadList(frame,15);
tipZ = loadList(frame,20);

% plot IK parts if ON radial button is selected
if rbOn == 1
    % In the Y-Z plane, so look at the last frame of Y and Z
    [IKtheta1, IKtheta2, IKtheta3, IKtheta4] = getJointAngles(tipX, tipY, tipZ, IKtable);
    
    % update the joint angle numbers
    set(handles.stPlayBackIKQ1,'String',num2str(round(IKtheta1)));
    set(handles.stPlayBackIKQ2,'String',num2str(round(IKtheta2)));
    set(handles.stPlayBackIKQ3,'String',num2str(round(IKtheta3)));
    set(handles.stPlayBackIKQ4,'String',num2str(round(IKtheta4)));

    % plot inverse kinematics
    [ikx, iky, ikz] = updatePlotFinger3D(iK_L1, iK_L2, iK_L3, IKtheta1, IKtheta2, IKtheta3, IKtheta4);
    IKPlot.XData = ikx;
    IKPlot.YData = iky;
    IKPlot.ZData = ikz;
    IKPlot.MarkerEdgeColor = 'r';
    IKPlot.Color = 'r';
    hold on
else
    IKPlot.XData = [0, 0, 0, 0, 0];
    IKPlot.YData = [0, 0, 0, 0, 0];
    IKPlot.ZData = [0, 0, 0, 0, 0];
    IKPlot.MarkerEdgeColor = 'k';
    IKPlot.Color = 'k';
    hold on
end

% plot playback data
playbackPlot.XData = loadList(frame, 6:10);
playbackPlot.YData = loadList(frame, 11:15);
playbackPlot.ZData = loadList(frame, 16:20);
hold off

% update the joint angle numbers
set(handles.stPlayBackQ1,'String', round(loadList(frame, 2), 1));
set(handles.stPlayBackQ2,'String', round(loadList(frame, 3), 1));
set(handles.stPlayBackQ3,'String', round(loadList(frame, 4), 1));
set(handles.stPlayBackQ4,'String', round(loadList(frame, 5), 1));


% --- Executes during object creation, after setting all properties.
function sPlayback_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in rbOff.
function rbOff_Callback(hObject, eventdata, handles)
set(handles.efIKLink1, 'Enable', 'on');
set(handles.efIKLink2, 'Enable', 'on');
set(handles.efIKLink3, 'Enable', 'on');
set(handles.stTableStatus,'String','No Table Created');
