function varargout = gui1(varargin)
% GUI1 MATLAB code for gui1.fig
%      GUI1, by itself, creates a new GUI1 or raises the existing
%      singleton*.
%
%      H = GUI1 returns the handle to a new GUI1 or the handle to
%      the existing singleton*.
%
%      GUI1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI1.M with the given input arguments.
%
%      GUI1('Property','Value',...) creates a new GUI1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui1

% Last Modified by GUIDE v2.5 01-Mar-2017 13:56:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui1_OpeningFcn, ...
                   'gui_OutputFcn',  @gui1_OutputFcn, ...
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


% --- Executes just before gui1 is made visible.
function gui1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui1 (see VARARGIN)

% Choose default command line output for gui1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%load data
[name path]=uigetfile('*.*');
newData1 = importdata(fullfile(path, name));
header = genvarname(newData1.colheaders);

for i = 1: size(header, 2)
    eval([header{i} ' = newData1.data(:,i);']);
end

%Index
index1=find(SensorId==2)
handles.Time1=TimeStamp0x28s0x29(index1);
Gyrz1=GyroZ0x28deg0x2Fs0x29(index1);

% filter design 0.05 high pass filter of RA
fs=100;		        % Sampling rate
filterOrder=5;		% Order of filter
cutOffFreq=0.05; %預設
cutOffFreq=str2num(get(handles.edit5,'String')); % Cutoff frequency	  
[b, a]=butter(filterOrder, cutOffFreq/(fs/2), 'high');

% 0.05 high pass filter use of LA
 handles.Gyrz1_HighPass=filter(b, a,Gyrz1);


% plot raw data of gyro
horizen_value=0;%預設
horizen_value=str2num(get(handles.edit4,'String'));
for i=1:size(handles.Time1)
  handles.Time1_0(i)=horizen_value;
end  

n=1;
m=1;

start=400;%預設
finish=1500;%預設
start=str2num(get(handles.edit2,'String'));
finish=str2num(get(handles.edit3,'String'));

for i=start:finish
    if  handles.Gyrz1_HighPass(i)<horizen_value
        if handles.Gyrz1_HighPass(i+1)>horizen_value
            handles.swing_start(n)=i*0.01;    %save
            guidata(hObject, handles);
            n=n+1;
        end
    end
   
    
    if  handles.Gyrz1_HighPass(i)>horizen_value
        if handles.Gyrz1_HighPass(i+1)<horizen_value
            handles.stance_start(m)=i*0.01;   %save
            guidata(hObject, handles);
            m=m+1;
        end
    end
end  
% save the n&m
handles.n=n;
guidata(hObject, handles);
handles.m=m;
guidata(hObject, handles);

%計算stance phase (swing_start先開始,swing 跟stance數量同)
for i=1:n-2;
     handles.step_time(i)=handles.swing_start(i+1)-handles.swing_start(i);   %save
     guidata(hObject, handles);
     handles.stance_time(i)=handles.swing_start(i+1)-handles.stance_start(i);   %save
     guidata(hObject, handles);
     handles.stance_phase(i)=(handles.stance_time(i)/handles.step_time(i))*100;   %save
     guidata(hObject, handles);
end 
stance_phase_mean=sum(handles.stance_phase)/(n-2);
step_time_meam=sum(handles.step_time)/(n-2);

set(handles.text1, 'String', stance_phase_mean);
set(handles.text3, 'String',step_time_meam);

% plot the phase line and data
%{
for i=1:handles.swing_start(1)*100-1;    
    phase_line(i)=0;  
end
for i=1:handles.n-2; 
    for j=handles.swing_start(i)*100:handles.stance_start(i)*100-1;
        phase_line(j)=100;
    end
    for k=handles.stance_start(i)*100:handles.swing_start(i+1)*100-1;
        phase_line(k)=-100;
    end    
end
for i=handles.swing_start(handles.n-1)*100:size(Time1);
    phase_line(i)=0;
end
%}
for i=1:size(handles.Time1);    
    phase_line(i)=0; 
end
for i=handles.swing_start(1)*100:handles.swing_start(handles.n-1)*100;
    phase_line(i)=-50;
end
for j=1:handles.n-2;
    for i=1:size(handles.Time1);
        if i>=handles.swing_start(j)*100 && i<=handles.stance_start(j)*100
        phase_line(i)=50; 
        end
    end    
end
plot(handles.axes1,handles.Time1,handles.Gyrz1_HighPass,'b',handles.Time1,handles.Time1_0,'g',handles.Time1,phase_line,'r');  
title('Gyr');


function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%load data
[name path]=uigetfile('*.*');
newData1 = importdata(fullfile(path, name));
header = genvarname(newData1.colheaders);

for i = 1: size(header, 2)
    eval([header{i} ' = newData1.data(:,i);']);
end

%Index
index1=find(SensorId==2)
Time1=TimeStamp0x28s0x29(index1);
Gyrz1=GyroZ0x28deg0x2Fs0x29(index1);
% plot raw data of gyro

for i=1:size(Time1)
  Time1_0(i)=0;
end  

plot(handles.axes1,Time1,Gyrz1,'b',Time1,Time1_0,'r');
title('Gyr');




% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%load data
[name path]=uigetfile('*.*');
newData1 = importdata(fullfile(path, name));
header = genvarname(newData1.colheaders);

for i = 1: size(header, 2)
    eval([header{i} ' = newData1.data(:,i);']);
end

%Index
index1=find(SensorId==2)
Time1=TimeStamp0x28s0x29(index1);
Gyrz1=GyroZ0x28deg0x2Fs0x29(index1);

% filter design 0.05 high pass filter of RA
fs=100;		        % Sampling rate
filterOrder=5;		% Order of filter
cutOffFreq=0.05 %預設
cutOffFreq=str2num(get(handles.edit5,'String')); % Cutoff frequency	    
[b, a]=butter(filterOrder, cutOffFreq/(fs/2), 'high');

% 0.05 high pass filter use of LA
 Gyrz1_HighPass=filter(b, a,Gyrz1);
% plot raw data of gyro

for i=1:size(Time1)
  Time1_0(i)=0;
end  

plot(handles.axes1,Time1,Gyrz1_HighPass,'b',Time1,Time1_0,'r');
title('Gyr');



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Num_change=str2num(get(handles.edit6,'String')); % Cutoff frequency	
[new_time,new_y]=ginput(1);
set(handles.text6, 'String',new_y);
set(handles.text7, 'String',new_time);
handles.swing_start(Num_change)=new_time;
guidata(hObject, handles);

%caculate again
for i=1:handles.n-2;
     handles.step_time(i)=handles.swing_start(i+1)-handles.swing_start(i);   %save
     guidata(hObject, handles);
     handles.stance_time(i)=handles.swing_start(i+1)-handles.stance_start(i);   %save
     guidata(hObject, handles);
     handles.stance_phase(i)=(handles.stance_time(i)/handles.step_time(i))*100;   %save
     guidata(hObject, handles);
end 
stance_phase_mean=sum(handles.stance_phase)/(handles.n-2);
step_time_meam=sum(handles.step_time)/(handles.n-2);

set(handles.text1, 'String', stance_phase_mean);
set(handles.text3, 'String',step_time_meam);

%plot new phase_line
for i=1:size(handles.Time1);    
    phase_line(i)=0; 
end
for i=fix(handles.swing_start(1)*100):fix(handles.swing_start(handles.n -1)*100);
    phase_line(i)=-50;
end
for j=1:handles.n-2;
    for i=1:size(handles.Time1);
        if i>=handles.swing_start(j)*100 && i<=handles.stance_start(j)*100
        phase_line(i)=50; 
        end
    end    
end
plot(handles.axes1,handles.Time1,handles.Gyrz1_HighPass,'b',handles.Time1,handles.Time1_0,'g',handles.Time1,phase_line,'r');  
title('Gyr');

function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Num_change=str2num(get(handles.edit7,'String')); % Cutoff frequency	
[new_time,new_y]=ginput(1);
set(handles.text8, 'String',new_y);
set(handles.text9, 'String',new_time);
handles.stance_start(Num_change)=new_time;
guidata(hObject, handles);

%caculate again
for i=1:handles.n-2;
     handles.step_time(i)=handles.swing_start(i+1)-handles.swing_start(i);   %save
     guidata(hObject, handles);
     handles.stance_time(i)=handles.swing_start(i+1)-handles.stance_start(i);   %save
     guidata(hObject, handles);
     handles.stance_phase(i)=(handles.stance_time(i)/handles.step_time(i))*100;   %save
     guidata(hObject, handles);
end 
stance_phase_mean=sum(handles.stance_phase)/(handles.n-2);
step_time_meam=sum(handles.step_time)/(handles.n-2);

set(handles.text1, 'String', stance_phase_mean);
set(handles.text3, 'String',step_time_meam);

%plot new phase_line
for i=1:size(handles.Time1);    
    phase_line(i)=0; 
end
for i=fix(handles.swing_start(1)*100):fix(handles.swing_start(handles.n -1)*100);
    phase_line(i)=-50;
end
for j=1:handles.n-2;
    for i=1:size(handles.Time1);
        if i>=handles.swing_start(j)*100 && i<=handles.stance_start(j)*100
        phase_line(i)=50; 
        end
    end    
end
plot(handles.axes1,handles.Time1,handles.Gyrz1_HighPass,'b',handles.Time1,handles.Time1_0,'g',handles.Time1,phase_line,'r');  
title('Gyr');

function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
