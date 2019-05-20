function varargout = tubesfauzan(varargin)
% TUBESFAUZAN MATLAB code for tubesfauzan.fig
%      TUBESFAUZAN, by itself, creates a new TUBESFAUZAN or raises the existing
%      singleton*.
%
%      H = TUBESFAUZAN returns the handle to a new TUBESFAUZAN or the handle to
%      the existing singleton*.
%
%      TUBESFAUZAN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TUBESFAUZAN.M with the given input arguments.
%
%      TUBESFAUZAN('Property','Value',...) creates a new TUBESFAUZAN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before tubesfauzan_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to tubesfauzan_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help tubesfauzan

% Last Modified by GUIDE v2.5 20-May-2019 21:33:32

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @tubesfauzan_OpeningFcn, ...
                   'gui_OutputFcn',  @tubesfauzan_OutputFcn, ...
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


% --- Executes just before tubesfauzan is made visible.
function tubesfauzan_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to tubesfauzan (see VARARGIN)

% Choose default command line output for tubesfauzan
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes tubesfauzan wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = tubesfauzan_OutputFcn(hObject, eventdata, handles) 
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
[filename,pathname] = uigetfile({'*.jpg';'*.bmp';'*.png';'*.tif';},'Open Image');  
handles.data1 = imread(fullfile(pathname,filename));
guidata(hObject,handles);
axes(handles.axes1);
imshow(handles.data1);

% --- Executes on button press in gray.
function gray_Callback(hObject, eventdata, handles)
% hObject    handle to gray (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
image=handles.data1;
axes(handles.axes2);
red = image(:,:,1);
green = image(:,:,2);
blue = image(:,:,3);
gray = .299*red + .587*green + .114*blue;
guidata(hObject,handles);
imshow(gray);


% --- Executes on button press in histo.
function histo_Callback(hObject, eventdata, handles)
% hObject    handle to histo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I =handles.data1;
h=rgb2gray(I);
[M,N]=size(h);
t=1:256;
n=0:255;
count=0;
for z=1:256
    for i=1:M
        for j=1:N
            if h(i,j)==z-1
                count=count+1;
            end
        end
    end
            t(z)=count;
            count=0;
end
axes(handles.axes2);
stem(n,t); 
grid on;


% --- Executes on button press in contKali.
function contKali_Callback(hObject, eventdata, handles)
% hObject    handle to contKali (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
image=handles.data1;
x = str2double(get(handles.edit1,'String'));
axes(handles.axes2);
contrast = image*x;
guidata(hObject,handles);
imshow(contrast);

% --- Executes on button press in contBagi.
function contBagi_Callback(hObject, eventdata, handles)
% hObject    handle to contBagi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
image=handles.data1;
x = str2double(get(handles.edit1,'String'));
axes(handles.axes2);
contrast = image/x;
guidata(hObject,handles);
imshow(contrast);

% --- Executes on button press in contTambah.
function contTambah_Callback(hObject, eventdata, handles)
% hObject    handle to contTambah (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA
image=handles.data1;
x = str2double(get(handles.edit1,'String'));
axes(handles.axes2);
contrast = image+x;
guidata(hObject,handles);
imshow(contrast);

% --- Executes on button press in contKurang.
function contKurang_Callback(hObject, eventdata, handles)
% hObject    handle to contKurang (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
image=handles.data1;
x = str2double(get(handles.edit1,'String'));
axes(handles.axes2);
contrast = image-x;
guidata(hObject,handles);
imshow(contrast);


% --- Executes on button press in blur.
function blur_Callback(hObject, eventdata, handles)
% hObject    handle to blur (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
image=handles.data1;
axes(handles.axes2);
[t, l, p] = size(image);
imgnew = double(image);
m = 1/9;
x = [m m m ; m m m ; m m m];
for p=1 : 3
    for i=2 : t-2
        for j=2 : l-2
            sum=imgnew(i-1,j-1,p)*x(1,1)+imgnew(i,j-1,p)*x(2,1)...
                + imgnew(i+1,j-1,p)*x(3,1)+imgnew(i-1,j,p)*x(1,2)...
                + imgnew(i,j,p)*x(2,2)+imgnew(i+1,j,p)*x(3,2)...
                + imgnew(i-1,j+1,p)*x(1,3)+imgnew(i,j+1,p)*x(2,3)...
                + imgnew(i+1,j+1,p)*x(3,3);
            img(i-1,j-1,p)=sum;
        end
    end
end
imshow(uint8(img));


% --- Executes on button press in sharp.
function sharp_Callback(hObject, eventdata, handles)
% hObject    handle to sharp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
image=handles.data1;
axes(handles.axes2);
[t, l, p] = size(image);
imgnew = double(image);
x = [0 -1 0 ; -1 5 -1 ; 0 -1 0];
for p=1 : 3
    for i=2 : t-2
        for j=2 : l-2
            sum=imgnew(i-1,j-1,p)*x(1,1)+imgnew(i,j-1,p)*x(2,1)...
                + imgnew(i+1,j-1,p)*x(3,1)+imgnew(i-1,j,p)*x(1,2)...
                + imgnew(i,j,p)*x(2,2)+imgnew(i+1,j,p)*x(3,2)...
                + imgnew(i-1,j+1,p)*x(1,3)+imgnew(i,j+1,p)*x(2,3)...
                + imgnew(i+1,j+1,p)*x(3,3);
            img(i-1,j-1,p)=sum;
        end
    end
end
imshow(uint8(img));

% --- Executes on button press in edge.
function edge_Callback(hObject, eventdata, handles)
% hObject    handle to edge (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
image=handles.data1;
axes(handles.axes2);
[t, l, p] = size(image);
imgnew = double(image);
x = [1 1 1 ; 1 -8 1 ; 1 1 1];
for p=1 : 3
    for i=2 : t-2
        for j=2 : l-2
            sum=imgnew(i-1,j-1,p)*x(1,1)+imgnew(i,j-1,p)*x(2,1)...
                + imgnew(i+1,j-1,p)*x(3,1)+imgnew(i-1,j,p)*x(1,2)...
                + imgnew(i,j,p)*x(2,2)+imgnew(i+1,j,p)*x(3,2)...
                + imgnew(i-1,j+1,p)*x(1,3)+imgnew(i,j+1,p)*x(2,3)...
                + imgnew(i+1,j+1,p)*x(3,3);
            img(i-1,j-1,p)=sum;
        end
    end
end
imshow(uint8(img));


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
image=handles.data1;
axes(handles.axes2);
%blurFilter=[0.1 0.111 0.111;0.111 0.111 0.111;0.111 0.111 0.111];
blurFilter=fspecial('gaussian',20,4);
blur = imfilter(image, blurFilter, 'replicate');
imshow(blur);


% --- Executes on button press in zoomin.
function zoomin_Callback(hObject, eventdata, handles)
% hObject    handle to zoomin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
I = getimage(handles.axes1);
m=1; n=1;
o=1; p=1;
red = I(:,:,1);
green = I(:,:,2);
blue = I(:,:,3);
for i=1:size(I,1)
    for j=1:size(I,2)
        rednew(m,n)=red(o,p);
        rednew(m,n+1)=red(o,p);
        rednew(m+1,n)=red(o,p);
        rednew(m+1,n+1)=red(o,p);
        greennew(m,n)=green(o,p);
        greennew(m,n+1)=green(o,p);
        greennew(m+1,n)=green(o,p);
        greennew(m+1,n+1)=green(o,p);
        bluenew(m,n)=blue(o,p);
        bluenew(m,n+1)=blue(o,p);
        bluenew(m+1,n)=blue(o,p);
        bluenew(m+1,n+1)=blue(o,p);
        if(n+2<=size(I,2))
            n=n+2;
        end
        p=p+1;
    end
    if(m+2<=size(I,1))
        m=m+2;
    end
    o=o+1;
    p=1;
    n=1;
end
potong = cat(3,rednew,greennew,bluenew);
axes(handles.axes2);
imshow(potong);

% --- Executes on button press in zoomout.
function zoomout_Callback(hObject, eventdata, handles)
% hObject    handle to zoomout (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



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


% --- Executes on button press in tubesfauzan.
function compress_Callback(hObject, eventdata, handles)
% hObject    handle to tubesfauzan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
gambar = handles.data1;
F = dct2(image);
ff = idct2(F);

[m, n] = size(image);
DF = zeros(m, n);
DFF = DF;
IDF = DF;
IDFF = DF;
depth = 4;
N = 8;

for i=1 : N : m
    for j=1 : N : n
        f = gambar(i:i+N-1,j:j+N-1);
        df = dct2(f);
        DF(i:i+N-1,j:j+N-1) = df;
        dff = idct2(df);
        DFF(i:i+N-1,j:j+N-1) = dff;
        
        df(N:-1:depth+1,:) = 0;
        df(:,N:-1:depth+1) = 0;
        IDF(i:i+N-1,j:j+N-1) = df;
        dff = idct2(df);
        IDFF(i:i+N-1,j:j+N-1) = dff;
    end
end

A = DFF/255;
imwrite(A,'hasil.jpg');
B=IDFF/255;
imwrite(B,'hasil2.jpg');
axes(handles.axes2)
imshow(B);
