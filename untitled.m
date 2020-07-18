function varargout = untitled(varargin)
% UNTITLED MATLAB code for untitled.fig
%      UNTITLED, by itself, creates a new UNTITLED or raises the existing
%      singleton*.
%
%      H = UNTITLED returns the handle to a new UNTITLED or the handle to
%      the existing singleton*.
%
%      UNTITLED('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UNTITLED.M with the given input arguments.
%
%      UNTITLED('Property','Value',...) creates a new UNTITLED or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before untitled_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to untitled_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitled

% Last Modified by GUIDE v2.5 18-Jul-2020 14:38:45

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitled_OpeningFcn, ...
                   'gui_OutputFcn',  @untitled_OutputFcn, ...
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


% --- Executes just before untitled is made visible.
function untitled_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to untitled (see VARARGIN)

% Choose default command line output for untitled
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes untitled wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = untitled_OutputFcn(hObject, eventdata, handles) 
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


stopper = 0;
j=0;
fid = fopen('data.txt', 'wt'); % открыли файл дл€ записи
while 1

    f0 = 465; %465 к√ц
    fs = 1000; %1 ћ√ц
    T = 1/fs;

    t = 0:T:10-T; 

    % rng(40);
    num = randi(3); %число прин€тых сигналов
    % num = 1;
    % num = 2;
    % num = 3;

    x = nan(num,length(t));

    n=50*num; 
    noise = zeros(num,length(t));

    for i=1:1:length(t)
        noise(:,i) = normrnd(0,n,[num,1]);
    end

    % rng(26);
    
    for i=1:1:num
        A=randi(1000);
        A=1.1*A;
        x(i,:) = A*sin(2*pi*f0*t);
    end

    Et1 = 0; 
    Et2 = 0;
    Et3 = 0;
    for i=1:1:length(t)
        if num==1
            Et1 = Et1 + T*(x(1,i)^2);
        elseif num==2
            Et1 = Et1 + T*(x(1,i)^2);
            Et2 = Et2 + T*(x(2,i)^2);
        else
            Et1 = Et1 + T*(x(1,i)^2);
            Et2 = Et2 + T*(x(2,i)^2);
            Et3 = Et3 + T*(x(3,i)^2);
        end
    end

    E_ref = (Et1/3600)*1e6+(Et2/3600)*1e6+(Et3/3600)*1e6; 
    E_ref = E_ref/(4*pi*25)/1e4; 

    y = x+noise;
    
    Et_y1 = 0; 
    Et_y2 = 0;
    Et_y3 = 0;
    for i=1:1:length(t)
        if num==1
            Et_y1 = Et_y1 + T*(y(1,i)^2);
        elseif num==2
            Et_y1 = Et_y1 + T*(y(1,i)^2);
            Et_y2 = Et_y2 + T*(y(2,i)^2);
        else
            Et_y1 = Et_y1 + T*(y(1,i)^2);
            Et_y2 = Et_y2 + T*(y(2,i)^2);
            Et_y3 = Et_y3 + T*(y(3,i)^2);
        end
    end

    E_noise = (Et_y1/3600)*1e6+(Et_y2/3600)*1e6+(Et_y3/3600)*1e6; 
    E_noise = E_noise/(4*pi*25)/1e4; 

    if num==1
            u1=wiener_filter(x(1,:),y(1,:),t,n); 
        elseif num==2
            u1=wiener_filter(x(1,:),y(1,:),t,n); 
            u2 = wiener_filter(x(2,:),y(2,:),t,n); 
        else
            u1=wiener_filter(x(1,:),y(1,:),t,n); 
            u2 = wiener_filter(x(2,:),y(2,:),t,n); 
            u3 = wiener_filter(x(3,:),y(3,:),t,n); 
    end

    Et_u1 = 0; 
    Et_u2 = 0;
    Et_u3 = 0;
    for i=1:1:length(t)
        if num==1
            Et_u1 = Et_u1 + T*(u1(1,i)^2);
        elseif num==2
            Et_u1 = Et_u1 + T*(u1(1,i)^2);
            Et_u2 = Et_u2 + T*(u2(1,i)^2);
        else
            Et_u1 = Et_u1 + T*(u1(1,i)^2);
            Et_u2 = Et_u2 + T*(u2(1,i)^2);
            Et_u3 = Et_u3 + T*(u3(1,i)^2);
        end

    end
    E_u = (Et_u1/3600)*1e6+(Et_u2/3600)*1e6+(Et_u3/3600)*1e6; 
    E_u = E_u/(4*pi*25)/1e4; 


    fprintf(fid,num2str(round(E_u,2)));
    fprintf(fid,'\n');
    
       
    pause(10)
    
    %¬ывод оценки энергетической экспозиции
    handles.edit2.String = [num2str(round(E_u,2)) ' мк¬т*ч/см^2'];

    %—игнал индикатора
    if E_u >= 200
        sound(x(1,:),48e3,24); %можно прослушать
    end

    %÷вет индикатора
    if E_u == 0 
        handles.edit1.BackgroundColor = 'b'
    end
    if E_u > 0 & E_u <= 200
        handles.edit1.BackgroundColor = 'g'
    end
    if E_u >= 200
        handles.edit1.BackgroundColor = 'r'
    end
    
    stopper = str2num(handles.edit4.String);
    if stopper == 1
        break
    end
end
fclose(fid);


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.edit4.String = num2str(1);




% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


f0 = 465; %465 к√ц
fs = 1000; %1 ћ√ц
T = 1/fs;

t = 0:T:10-T; 

%rng(40);
num = randi(3); %число прин€тых сигналов
% num = 1;
% num = 2;
% num = 3;

x = nan(num,length(t));

n=50*num; 
noise = zeros(num,length(t));


for i=1:1:length(t)
    noise(:,i) = normrnd(0,n,[num,1]);
end


%rng(26);
for i=1:1:num
    A=randi(1000);
    A=1.1*A;
    x(i,:) = A*sin(2*pi*f0*t);
end


Et1 = 0; 
Et2 = 0;
Et3 = 0;
for i=1:1:length(t)
    if num==1
        Et1 = Et1 + T*(x(1,i)^2);
    elseif num==2
        Et1 = Et1 + T*(x(1,i)^2);
        Et2 = Et2 + T*(x(2,i)^2);
    else
        Et1 = Et1 + T*(x(1,i)^2);
        Et2 = Et2 + T*(x(2,i)^2);
        Et3 = Et3 + T*(x(3,i)^2);
    end
end

E_ref = (Et1/3600)*1e6+(Et2/3600)*1e6+(Et3/3600)*1e6; 
E_ref = E_ref/(4*pi*25)/1e4; 

y = x+noise;
Et_y1 = 0; 
Et_y2 = 0;
Et_y3 = 0;
for i=1:1:length(t)
    if num==1
        Et_y1 = Et_y1 + T*(y(1,i)^2);
    elseif num==2
        Et_y1 = Et_y1 + T*(y(1,i)^2);
        Et_y2 = Et_y2 + T*(y(2,i)^2);
    else
        Et_y1 = Et_y1 + T*(y(1,i)^2);
        Et_y2 = Et_y2 + T*(y(2,i)^2);
        Et_y3 = Et_y3 + T*(y(3,i)^2);
    end
end

E_noise = (Et_y1/3600)*1e6+(Et_y2/3600)*1e6+(Et_y3/3600)*1e6; 
E_noise = E_noise/(4*pi*25)/1e4; 

if num==1
        u1=wiener_filter(x(1,:),y(1,:),t,n); 
    elseif num==2
        u1=wiener_filter(x(1,:),y(1,:),t,n); 
        u2 = wiener_filter(x(2,:),y(2,:),t,n); 
    else
        u1=wiener_filter(x(1,:),y(1,:),t,n); 
        u2 = wiener_filter(x(2,:),y(2,:),t,n); 
        u3 = wiener_filter(x(3,:),y(3,:),t,n); 
end

Et_u1 = 0; 
Et_u2 = 0;
Et_u3 = 0;
for i=1:1:length(t)
    if num==1
        Et_u1 = Et_u1 + T*(u1(1,i)^2);
    elseif num==2
        Et_u1 = Et_u1 + T*(u1(1,i)^2);
        Et_u2 = Et_u2 + T*(u2(1,i)^2);
    else
        Et_u1 = Et_u1 + T*(u1(1,i)^2);
        Et_u2 = Et_u2 + T*(u2(1,i)^2);
        Et_u3 = Et_u3 + T*(u3(1,i)^2);
    end
    
end
E_u = (Et_u1/3600)*1e6+(Et_u2/3600)*1e6+(Et_u3/3600)*1e6; 
E_u = E_u/(4*pi*25)/1e4; 

pause(10)

%¬ывод оценки энергетической экспозиции
handles.edit2.String = [num2str(round(E_u,2)) ' мк¬т*ч/см^2'];

%—игнал индикатора
if E_u >= 200
    sound(x(1,:),48e3,24); %можно прослушать
end

%÷вет индикатора
if E_u == 0 
    handles.edit1.BackgroundColor = 'b'
end
if E_u > 0 & E_u <= 200
    handles.edit1.BackgroundColor = 'g'
end
if E_u >= 200
    handles.edit1.BackgroundColor = 'r'
end





function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



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
