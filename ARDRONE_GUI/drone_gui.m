function varargout = drone_gui(varargin)
% DRONE_GUI MATLAB code for drone_gui.fig
%      DRONE_GUI, by itself, creates a new DRONE_GUI or raises the existing
%      singleton*.
%
%      H = DRONE_GUI returns the handle to a new DRONE_GUI or the handle to
%      the existing singleton*.
%
%      DRONE_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DRONE_GUI.M with the given input arguments.
%
%      DRONE_GUI('Property','Value',...) creates a new DRONE_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before drone_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to drone_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help drone_gui

% Last Modified by GUIDE v2.5 27-Jan-2013 18:04:23

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @drone_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @drone_gui_OutputFcn, ...
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


% --- Executes just before drone_gui is made visible.
function drone_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to drone_gui (see VARARGIN)

% Choose default command line output for drone_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

%++++++++++++++++++++++++++++++++
% Drone variables
global droneObject;
droneObject = drone;

% UIWAIT makes drone_gui wait for user response (see UIRESUME)
% uiwait(handles.droneGui);


% --- Outputs from this function are returned to the command line.
function varargout = drone_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in configurationButton.
function configurationButton_Callback(hObject, eventdata, handles)
% hObject    handle to configurationButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in resetDroneButton.
function resetDroneButton_Callback(hObject, eventdata, handles)
% hObject    handle to resetDroneButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
display('Reset Drone');
droneObject.reset;


% --- Executes on button press in takeOffButton.
function takeOffButton_Callback(hObject, eventdata, handles)
% hObject    handle to takeOffButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
display('Take Off')
droneObject.takeoff;

% --- Executes on button press in landButton.
function landButton_Callback(hObject, eventdata, handles)
% hObject    handle to landButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
display('Land')
droneObject.land;


% --- Executes on button press in calibrateButton.
function calibrateButton_Callback(hObject, eventdata, handles)
% hObject    handle to calibrateButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in moveForwardButton.
function moveForwardButton_Callback(hObject, eventdata, handles)
% hObject    handle to moveForwardButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
display('Move Forward');
droneObject.move_forward(0.5);

% --- Executes on button press in moveLeftButton.
function moveLeftButton_Callback(hObject, eventdata, handles)
% hObject    handle to moveLeftButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
display('Move Left');
droneObject.roll_left(0.5);

% --- Executes on button press in moveRightButton.
function moveRightButton_Callback(hObject, eventdata, handles)
% hObject    handle to moveRightButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
display('Move Right');
droneObject.roll_right(0.5);

% --- Executes on button press in moveBackwardsButton.
function moveBackwardButton_Callback(hObject, eventdata, handles)
% hObject    handle to moveBackwardsButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
display('Move Backward');
droneObject.move_backward(0.5);

% --- Executes on button press in moveUpButton.
function moveUpButton_Callback(hObject, eventdata, handles)
% hObject    handle to moveUpButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
display('Move Up');
droneObject.go_up(0.5);

% --- Executes on button press in rotateClockwiseButton.
function rotateClockwiseButton_Callback(hObject, eventdata, handles)
% hObject    handle to rotateClockwiseButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
display('Counter Clockwise');
droneObject.rotate_right(0.5);

% --- Executes on button press in rotateCounterClockwiseButton.
function rotateCounterClockwiseButton_Callback(hObject, eventdata, handles)
% hObject    handle to rotateCounterClockwiseButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
display('Rotate Counter Clockwise');
droneObject.rotate_left(0.5);

% --- Executes on button press in moveDownButton.
function moveDownButton_Callback(hObject, eventdata, handles)
% hObject    handle to moveDownButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
display('Move Down');
droneObject.go_down(0.5);

% --- Executes on button press in flatTrimButton.
function flatTrimButton_Callback(hObject, eventdata, handles)
% hObject    handle to flatTrimButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
display('Flat Trim');
droneObject.ftrim;


% --- Executes on button press in batteryCheckButton.
function batteryCheckButton_Callback(hObject, eventdata, handles)
% hObject    handle to batteryCheckButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
droneObject.get_battery;
droneObject.battery


% --- Executes on button press in roundelTrackButton.
function roundelTrackButton_Callback(hObject, eventdata, handles)
% hObject    handle to roundelTrackButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
display('Track Roundel');
droneObject.track_roundel;


% --- Executes on button press in closeConnectionButton.
function closeConnectionButton_Callback(hObject, eventdata, handles)
% hObject    handle to closeConnectionButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global droneObject;
display('Close Connection');
droneObject.terminate;


% --- Executes on key press with focus on droneGui or any of its controls.
function droneGui_WindowKeyPressFcn(hObject, eventdata, handles)
% hObject    handle to droneGui (see GCBO)
% eventdata  structure with the following fields (see FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
% --- Executes on key press with focus on moveRightButton and none of its controls.
if get(handles.droneGui,'currentcharacter') == 'w';
    moveForwardButton_Callback(hObject, eventdata, handles);
end

if get(handles.droneGui,'currentcharacter') == 'd';
    moveRightButton_Callback(hObject, eventdata, handles);
end

if get(handles.droneGui,'currentcharacter') == 'a';
    moveLeftButton_Callback(hObject, eventdata, handles);
end

if get(handles.droneGui,'currentcharacter') == 's';
    moveBackwardButton_Callback(hObject, eventdata, handles);
end

if get(handles.droneGui,'currentcharacter') == 'q';
    rotateClockwiseButton_Callback(hObject, eventdata, handles);
end

if get(handles.droneGui,'currentcharacter') == 'e';
    rotateCounterClockwiseButton_Callback(hObject, eventdata, handles);
end

if get(handles.droneGui,'currentcharacter') == 'r';
    moveUpButton_Callback(hObject, eventdata, handles);
end

if get(handles.droneGui,'currentcharacter') == 'f';
    moveDownButton_Callback(hObject, eventdata, handles);
end

if get(handles.droneGui,'currentcharacter') == 't';
    takeOffButton_Callback(hObject, eventdata, handles);
end

if get(handles.droneGui,'currentcharacter') == 'l';
    landButton_Callback(hObject, eventdata, handles);
end

if get(handles.droneGui,'currentcharacter') == 'b';
    batteryCheckButton_Callback(hObject, eventdata, handles);
end