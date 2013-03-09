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

% Last Modified by GUIDE v2.5 10-Feb-2013 20:04:28

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
function drone_gui_OpeningFcn(hObject, ~, handles, varargin)
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
function varargout = drone_gui_OutputFcn(~, ~, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in resetDroneButton.
function resetDroneButton_Callback(~, ~, handles)
% hObject    handle to resetDroneButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('resetDrone', handles);

% --- Executes on button press in takeOffButton.
function takeOffButton_Callback(~, ~, handles)
% hObject    handle to takeOffButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('takeOff', handles);

% --- Executes on button press in landButton.
function landButton_Callback(~, ~, handles)
% hObject    handle to landButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('land', handles);
    

% --- Executes on button press in calibrateButton.
function calibrateButton_Callback(hObject, eventdata, handles)

% --- Executes on button press in calibrateCheckBox.
function calibrateCheckBox_Callback(hObject, eventdata, handles)
% hObject    handle to calibrateCheckBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of calibrateCheckBox    

% --- Executes on button press in moveForwardButton.
function moveForwardButton_Callback(~, ~, handles)
% hObject    handle to moveForwardButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('moveForward', handles);

% --- Executes on button press in moveLeftButton.
function moveLeftButton_Callback(~, ~, handles)
% hObject    handle to moveLeftButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('moveLeft', handles);

% --- Executes on button press in moveRightButton.
function moveRightButton_Callback(~, ~, handles)
% hObject    handle to moveRightButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('moveRight', handles);

% --- Executes on button press in moveBackwardButton.
function moveBackwardButton_Callback(~, ~, handles)
% hObject    handle to moveBackwardButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('moveBackward', handles);

% --- Executes on button press in moveUpButton.
function moveUpButton_Callback(~, ~, handles)
% hObject    handle to moveUpButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('moveUp', handles);

% --- Executes on button press in rotateClockwiseButton.
function rotateClockwiseButton_Callback(~, ~, handles)
% hObject    handle to rotateClockwiseButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('clockwise', handles);

% --- Executes on button press in rotateCounterClockwiseButton.
function rotateCounterClockwiseButton_Callback(~, ~, handles)
% hObject    handle to rotateCounterClockwiseButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('counterClockwise', handles);

% --- Executes on button press in moveDownButton.
function moveDownButton_Callback(~, ~, handles)
% hObject    handle to moveDownButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('moveDown', handles);

% --- Executes on button press in flatTrimButton.
function flatTrimButton_Callback(~, ~, handles)
% hObject    handle to flatTrimButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('flatTrim', handles);

% --- Executes on button press in batteryCheckButton.
function batteryCheckButton_Callback(~, ~, handles)
% hObject    handle to batteryCheckButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('batteryCheck', handles);

% --- Executes during object creation, after setting all properties.
function batteryEditText_CreateFcn(hObject, ~, ~)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

function batteryEditBox_Callback(~, ~, ~)

% --- Executes on button press in roundelTrackButton.
function roundelTrackButton_Callback(~, ~, handles)
% hObject    handle to roundelTrackButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('trackRoundel', handles);

% --- Executes on button press in closeConnectionButton.
function closeConnectionButton_Callback(~, ~, handles)
% hObject    handle to closeConnectionButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
controlCenter('closeConnection', handles);

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
 
% --- Executes during object creation, after setting all properties.
function debugOutput_Text_CreateFcn(hObject, ~, ~)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

function debugOutput_Text_Callback(~, ~, ~)

% --- Executes on button press in height_Button.
function height_Button_Callback(~, ~, handles)
% hObject    handle to height_Button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
height = str2double(get(handles.heightEditBox, 'string'));
    if( height < 500 || height > 2000 )
        return
    end

disp(['Height ' num2str(height/1000) ' meters'])
global droneObject;
droneObject.z_moveTo( height );

% --- Executes during object creation, after setting all properties.
function heightEditBox_CreateFcn(hObject, ~, ~)
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

function heightEditBox_Callback(~, ~, ~)

%Function that sends all the commands
function controlCenter(action, handles)

global droneObject;
dateString = datestr(clock);

    if( strcmp(action,'resetDrone') )
        actionMessage = { strcat( dateString, '        Reset Drone') };
        droneObject.reset;    
    elseif( strcmp(action, 'takeOff') )
        actionMessage = { strcat( dateString, '        Take Off') };
        droneObject.takeoff;
    elseif( strcmp(action, 'land') )
        actionMessage = { strcat( dateString, '        Land') };
        droneObject.land;    
    elseif( strcmp(action, 'moveForward') )
        actionMessage = { strcat( dateString, '        Move Forward') };
        droneObject.move_forward(0.5);
    elseif( strcmp(action, 'moveLeft') )
        actionMessage = { strcat( dateString, '        Move Left') };
        droneObject.roll_left(0.5);
    elseif( strcmp(action, 'moveRight') )
        actionMessage = { strcat( dateString, '        Move Right') };
        droneObject.roll_right(0.5);        
    elseif( strcmp(action, 'moveBackward') )
        actionMessage = { strcat( dateString, '        Move Backward') };
        droneObject.move_backward(0.5);
    elseif( strcmp(action, 'moveUp') )
        actionMessage = { strcat( dateString, '        Move Up') };
        droneObject.go_up(0.5);
    elseif( strcmp(action, 'clockwise') )
        actionMessage = { strcat( dateString, '        Clockwise') };
        droneObject.rotate_right(0.5);
    elseif( strcmp(action, 'counterClockwise') )
        actionMessage = { strcat( dateString, '        Counter Clockwise') };
        droneObject.rotate_left(0.5);    
    elseif( strcmp(action, 'moveDown') )
        actionMessage = { strcat( dateString, '        Move Down') };
        droneObject.go_down(0.5);     
    elseif( strcmp(action, 'flatTrim') )
        actionMessage = { strcat( dateString, '        Flat Trim') };
        droneObject.ftrim;
    elseif( strcmp(action, 'batteryCheck') )
        actionMessage = { strcat( dateString, '        Battery Check') };
        droneObject.get_battery;
        set(handles.batteryEditText, 'string', droneObject.battery);
    elseif( strcmp(action, 'trackRoundel') )
        actionMessage = { strcat( dateString, '        Track Roundel') };
        droneObject.track_roundel;
    elseif( strcmp(action, 'closeConnection') )
        actionMessage = { strcat( dateString, '        Close COnnection') };
        droneObject.terminate;
    end

display(action);
set(handles.debugOutput_Text, 'string', cat(1, actionMessage, get(handles.debugOutput_Text, 'string')) );





