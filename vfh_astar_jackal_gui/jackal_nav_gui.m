function varargout = jackal_nav_gui(varargin)
% JACKAL_NAV_GUI MATLAB code for jackal_nav_gui.fig
%      JACKAL_NAV_GUI, by itself, creates a new JACKAL_NAV_GUI or raises the existing
%      singleton*.
%
%      H = JACKAL_NAV_GUI returns the handle to a new JACKAL_NAV_GUI or the handle to
%      the existing singleton*.
%
%      JACKAL_NAV_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in JACKAL_NAV_GUI.M with the given input arguments.
%
%      JACKAL_NAV_GUI('Property','Value',...) creates a new JACKAL_NAV_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before jackal_nav_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to jackal_nav_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help jackal_nav_gui

% Last Modified by GUIDE v2.5 13-Nov-2019 21:05:37

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @jackal_nav_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @jackal_nav_gui_OutputFcn, ...
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

% --- Executes just before jackal_nav_gui is made visible.
function jackal_nav_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to jackal_nav_gui (see VARARGIN)

% Choose default command line output for jackal_nav_gui
handles.output = hObject;

% Update handles structure

guidata(hObject, handles);

% UIWAIT makes jackal_nav_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = jackal_nav_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes during object creation, after setting all properties.
function init_CreateFcn(hObject, eventdata, handles)
% hObject    handle to time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes on button press in init.
function init_Callback(hObject, eventdata, handles)
% hObject    handle to init (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rosshutdown;    
ipaddress = 'localhost';
rosinit(ipaddress);
global pose0 map vfh laser odom velmsg jackal rate ...
    x_length y_length resolution stop_flag map_time

stop_flag = false;
set(handles.status_disp,'String','Standing by');

laser = rossubscriber('/front/scan');
jackal = rospublisher('/jackal_velocity_controller/cmd_vel');
odom = rossubscriber('/odometry/filtered');
velmsg = rosmessage(jackal);

x_length=50;    % meter
y_length=50;
resolution=5;   % grid/meter
map = robotics.OccupancyGrid(x_length,y_length,resolution);
map.GridLocationInWorld=[-x_length/2,-y_length/2];
map.OccupiedThreshold=0.8;

vfh = robotics.VectorFieldHistogram;
vfh.NumAngularSectors = 180;
vfh.DistanceLimits = [0 2];
vfh.RobotRadius = 0.3;
vfh.MinTurningRadius = 0.3;
vfh.SafetyDistance = 0.1;
vfh.HistogramThresholds = [1 1];

rate = robotics.Rate(10);
map_time = 0;
pose = receive(odom);
quat = [pose.Pose.Pose.Orientation.W, pose.Pose.Pose.Orientation.X, ...
    pose.Pose.Pose.Orientation.Y, pose.Pose.Pose.Orientation.Z];
eul = quat2eul(quat);
pose0 = [pose.Pose.Pose.Position.X; pose.Pose.Pose.Position.Y; eul(1)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on button press in start_mapping.
function start_mapping_Callback(hObject, eventdata, handles)
% hObject    handle to start_mapping (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global map laser odom velmsg jackal rate occval ...
    row col stop_flag map_time
set(handles.status_disp,'String','Exploring...');
reset(rate);
% automatic exploring
stop_flag = false;
while rate.TotalElapsedTime < map_time && ~stop_flag
    % Get laser scan data
    laserScan = receive(laser);
    pose = receive(odom);
    [v,w] = VFH_nav(0, laserScan);
    % Assign and send velocity commands
    velmsg.Linear.X = v;
    velmsg.Angular.Z = w;
    send(jackal,velmsg);
    waitfor(rate);
    occval=mapping2(laserScan,pose);
    show(map)
end

occval=fliplr(occval');
index=occval==0;
occval(occval==1)=0;
occval(index)=1;
[row,col] = find(occval == 0);
set(handles.status_disp,'String','Standing by...');

% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
datacursormode on;
% Hint: place code in OpeningFcn to populate axes3



function enter_goal_x_Callback(hObject, eventdata, handles)
% hObject    handle to enter_goal_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global goal_x
goal_x = str2double(get(hObject, 'String'));
% Hints: get(hObject,'String') returns contents of enter_goal_x as text
%        str2double(get(hObject,'String')) returns contents of enter_goal_x as a double


% --- Executes during object creation, after setting all properties.
function enter_goal_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to enter_goal_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function enter_goal_y_Callback(hObject, eventdata, handles)
% hObject    handle to enter_goal_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global goal_y
goal_y = str2double(get(hObject, 'String'));
% Hints: get(hObject,'String') returns contents of enter_goal_y as text
%        str2double(get(hObject,'String')) returns contents of enter_goal_y as a double


% --- Executes during object creation, after setting all properties.
function enter_goal_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to enter_goal_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in generate_path.
function generate_path_Callback(hObject, eventdata, handles)
global map odom row col OptimalPath goal_x goal_y
inflate(map,0.2);

pose = receive(odom);
x=pose.Pose.Pose.Position.X;
y=pose.Pose.Pose.Position.Y;

Start = [x,y]
Goal = [goal_x, goal_y]
OptimalPath = astar2(Start,Goal);

cla(handles.axes4);
axis(handles.axes4,'equal');
hold(handles.axes4,'on');
plot(handles.axes4, row,col,'.k')
plot(handles.axes4,OptimalPath(end,1),OptimalPath(end,2),'xb')
plot(handles.axes4,OptimalPath(:,1),OptimalPath(:,2),'-r')
% set(handles.axes4,gca,'xtick',[],'xticklabel',[])
% set(handles.axes4,gca,'ytick',[],'yticklabel',[])

% hObject    handle to generate_path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function axes4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes4


% --- Executes on button press in navigate.
function navigate_Callback(hObject, eventdata, handles)
global OptimalPath odom velmsg jackal laser ...
     x_length y_length goal_theta resolution stop_flag
% set arrival condition
n=1;
nmax=length(OptimalPath);
tic;
stop_flag = false;
while ~stop_flag
    % receive from odom
    pose = receive(odom);
    odom_pose = odometry(pose);
    x = odom_pose(1);
    y = odom_pose(2);
    theta_odom = odom_pose(3);
    
    % read from lidar
    laserScan = receive(laser);
    
    % convert goal from global to local
    R_goal = [cos(theta_odom),sin(theta_odom);...
        -sin(theta_odom),cos(theta_odom)];
    
    local = R_goal*(([OptimalPath(n,1);OptimalPath(n,2)] -...
        [(x+x_length/2)*resolution;(y+y_length/2)*resolution]));
    
    x_local = local(1);
    y_local = local(2);
    bearing_goal = atan2(y_local,x_local);
    
    [v,w] = VFH_nav(bearing_goal,laserScan);
    
    plot(handles.axes4,(x+x_length/2)*resolution,(y+y_length/2)*resolution,'.b');
    hold(handles.axes4,'on');
    
    % send speed command
    velmsg.Linear.X = v;
    velmsg.Angular.Z = w;
    send(jackal,velmsg);
    
    % update arrival condition
    range_goal = sqrt((OptimalPath(n,1)-(x+x_length/2)*resolution)^2 +...
        ((OptimalPath(n,2)-(y+y_length/2)*resolution))^2);
    if n < nmax
        if range_goal <= 0.6*resolution
            n = n + 1;      % update waypoint
        end
    elseif n == nmax
        if range_goal <= 0.4*resolution
            break             % arrive goal
        end
    end
    
    set(handles.status_disp,'String','Navigating...');
end

while ~stop_flag    % spin to desired orientation
    pose = receive(odom);
    odom_pose = odometry(pose);
    theta_odom = odom_pose(3);
    
    error = deg2rad(goal_theta) - theta_odom;
    w = 0.25*error;
    if w > 0.6
        w = 0.6;
    elseif w < -0.6
        w = -0.6;
    end
    
    set(handles.status_disp,'String','Spin ...');
    
    if abs(error) <= 0.05
        break
    end
    
    velmsg.Linear.X = 0;
    velmsg.Angular.Z = w;
    send(jackal,velmsg);
end

velmsg.Linear.X = 0;
velmsg.Angular.Z = 0;
send(jackal,velmsg);
total_time = toc;
set(handles.time,'String',num2str(total_time));
set(handles.status_disp,'String','Standing by');
% hObject    handle to navigate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in estop.
function estop_Callback(hObject, eventdata, handles)
global stop_flag
stop_flag = true;

set(handles.status_disp,'String','Standing by');
% hObject    handle to estop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function enter_map_time_Callback(hObject, eventdata, handles)
% hObject    handle to enter_map_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global map_time
map_time = str2double(get(hObject, 'String'))
% Hints: get(hObject,'String') returns contents of enter_map_time as text
%        str2double(get(hObject,'String')) returns contents of enter_map_time as a double


% --- Executes during object creation, after setting all properties.
function enter_map_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to enter_map_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function enter_theta_Callback(hObject, eventdata, handles)
% hObject    handle to enter_theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global goal_theta
goal_theta = str2double(get(hObject, 'String'));
% Hints: get(hObject,'String') returns contents of enter_theta as text
%        str2double(get(hObject,'String')) returns contents of enter_theta as a double


% --- Executes during object creation, after setting all properties.
function enter_theta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to enter_theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
