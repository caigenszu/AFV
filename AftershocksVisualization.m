function varargout = AftershocksVisualization(varargin)
% AFTERSHOCKSVISUALIZATION MATLAB code for AftershocksVisualization.fig
%      AFTERSHOCKSVISUALIZATION, by itself, creates a new AFTERSHOCKSVISUALIZATION or raises the existing
%      singleton*.
%
%      H = AFTERSHOCKSVISUALIZATION returns the handle to a new AFTERSHOCKSVISUALIZATION or the handle to
%      the existing singleton*.
%
%      AFTERSHOCKSVISUALIZATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in AFTERSHOCKSVISUALIZATION.M with the given input arguments.
%
%      AFTERSHOCKSVISUALIZATION('Property','Value',...) creates a new AFTERSHOCKSVISUALIZATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before AftershocksVisualization_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to AftershocksVisualization_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help AftershocksVisualization

% Last Modified by Chisheng Wang v2.5 30-Jan-2019 13:24:18

% Aftershock Visualization [AFV1.0]
% Software for the Visualization of Aftershock Point Cloud Data.
% Copyright: Chisheng Wang, 2018
%
% Email: sherwoodwang88@gmail.com

% Begin initialization code - DO NOT EDIT
global cc lofvalue  p1 ind fmatrix dist; %cc: aftershocks point cloud; lofvalue: lof of the points; p1: projected points on the fault plane; ind: the points for display; fmatrix: the index matrix; dist: distance to plane


gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @AftershocksVisualization_OpeningFcn, ...
    'gui_OutputFcn',  @AftershocksVisualization_OutputFcn, ...
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


% --- Executes just before AftershocksVisualization is made visible.
function AftershocksVisualization_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to AftershocksVisualization (see VARARGIN)

% Choose default command line output for AftershocksVisualization
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes AftershocksVisualization wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = AftershocksVisualization_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --------------------------------------------------------------------
function importdata_Callback(hObject, eventdata, handles)
% hObject    handle to importdata (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
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


% --------------------------------------------------------------------
function import_Callback(hObject, eventdata, handles)
% hObject    handle to import (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cc
[filename, pathname] = uigetfile( ...
    {'*.txt;*.tif;*.png;*.gif','All Image Files';...
    '*.*','All Files' },...
    '请选择要修改的图片（可多选）', ...
    'MultiSelect', 'on');
cc=importdata(filename);
cc(:,3)=-cc(:,3);
scatter3(handles.axes1,cc(:,1),cc(:,2),cc(:,3),1,cc(:,3),'filled');axis equal;
hold on;

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cc p1;
ind=find(cc(:,7)==1);
axes(handles.axes1);
hold on;
plot3(cc(ind,1),cc(ind,2),cc(ind,3),'kp','MarkerSize',10, 'MarkerFaceColor','k');colormap(jet);
axes(handles.axes3);
hold on;
plot(p1(ind,1),p1(ind,2),'kp','MarkerSize',10, 'MarkerFaceColor','k');axis equal;


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cc p1 dist
%[az, el] = view(handles.axes1);
% set(handles.edit2,'String',num2str(90+az));
% set(handles.edit3,'String',num2str(90-el));

strike=str2num(get(handles.edit2,'String'));
dip=str2num(get(handles.edit3,'String'));
proj=str2num(get(handles.edit5,'String'));

n=[-sind(dip)*sind(strike-90),-sind(dip)*cosd(strike-90),cosd(dip)];
nlin=[-sind(dip)*sind(proj),-sind(dip)*cosd(proj),cosd(dip)];

% define the plane
XYZ=median(cc(:,1:3));
D=-sum(n.*XYZ);
% find the distance to the plane
for i=1:size(cc,1)
    [I,check]=plane_line_intersect(n,XYZ,cc(i,1:3),cc(i,1:3)-10*nlin);
    p0(i,1:3)=I;
    dist(i)=norm(I-cc(i,1:3));
end
cc(:,8)=dist;

% rotate along z
a=strike-90;
Rz=[cosd(a) -sind(a) 0;sind(a) cosd(a) 0;0 0 1];
% rotate along x
a=-dip;
Rx=[1 0 0;0 cosd(a) -sind(a);0 sind(a) cosd(a)];
axes(handles.axes3);
hold off;
p1 = Rx*Rz*p0(:,1:3)';%*;
p1=p1';

replot( handles);


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
re

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global cc
[filename, pathname] = uigetfile( ...
    {'*.dat;*.txt','All fault Files';...
    '*.*','All Files' },...
    'Please select fault file', ...
    'MultiSelect', 'on');
fault=importdata(filename);
for i=1:size(fault,1)
    strike=90-fault(i,1);dip=90-fault(i,2);x_utm=fault(i,3);y_utm=fault(i,4);len=fault(i,5);dep_up=fault(i,6);dep_down=fault(i,7);
    lef_cs=[x_utm-0.5*len*cosd(strike) y_utm-0.5*len*sind(strike)];
    rig_cs=[x_utm+0.5*len*cosd(strike) y_utm+0.5*len*sind(strike)];
    
    lef_cu=lef_cs+[dep_up*tand(dip)*sind(strike) -dep_up*tand(dip)*cosd(strike)];
    rig_cu=rig_cs+[dep_up*tand(dip)*sind(strike) -dep_up*tand(dip)*cosd(strike)];
    lef_cd=lef_cs+[dep_down*tand(dip)*sind(strike) -dep_down*tand(dip)*cosd(strike)];
    rig_cd=rig_cs+[dep_down*tand(dip)*sind(strike) -dep_down*tand(dip)*cosd(strike)];
    
    corner=[lef_cu;rig_cu;rig_cd;lef_cd;lef_cu];corner(:,3)=[dep_up;dep_up;dep_down;dep_down;dep_up];
    axes(handles.axes1);
    hold on;
    plot3(corner(1:2,1),corner(1:2,2),-corner(1:2,3),'k-','linewidth',2);
    plot3(corner(2:5,1),corner(2:5,2),-corner(2:5,3),'k.-','linewidth',1);
    hold on;
end



% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cc lofvalue
[tt,lofvalue]=LOFa(cc(:,1:3),30);
cc(:,6)=lofvalue;

function edit2_Callback(hObject, eventdata, handles)

% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
aa=str2num(get(handles.edit2,'String'))-90;
set(handles.edit5,'String',num2str(aa));

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


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: place code in OpeningFcn to populate axes1


% --- Executes on mouse press over axes background.
function axes1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global cc p1 ind fmatrix;
k=get(hObject,'value');
set(handles.text7,'String',num2str(100*k));
k1=str2num(get(handles.text7,'String'));
k2=str2num(get(handles.text8,'String'));
m=sort([k1 k2]);
k1=0.001+m(1);k2=m(2);
popup_sel_index = get(handles.popupmenu2,'Value');
num=size(cc,1);
switch popup_sel_index
    case 1 %depth
        fmatrix(1,:)=[k1 k2];
        [B,I] = sort(cc(:,3));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),3));
        set(handles.text12,'String',cc(indt(end),3));
    case 2 %lof
        fmatrix(2,:)=[k1 k2];
        [B,I] = sort(cc(:,6));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),6));
        set(handles.text12,'String',cc(indt(end),6));
    case 3 %magnitude
        fmatrix(3,:)=[k1 k2];
        [B,I] = sort(cc(:,4));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),4));
        set(handles.text12,'String',cc(indt(end),4));
    case 4 %time
        fmatrix(4,:)=[k1 k2];
        [B,I] = sort(cc(:,5));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        stime=datetime(get(handles.edit4,'String'),'InputFormat','uuuu-MM-dd''T''HH:mm:ss');
        set(handles.text11,'String',datestr(stime+cc(indt(1),5)));
        set(handles.text12,'String',datestr(stime+cc(indt(end),5)));
    case 5 %utm-x
        fmatrix(5,:)=[k1 k2];
        [B,I] = sort(cc(:,1));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),1));
        set(handles.text12,'String',cc(indt(end),1));
    case 6 %utm-y
        fmatrix(6,:)=[k1 k2];
        [B,I] = sort(cc(:,2));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),2));
        set(handles.text12,'String',cc(indt(end),2));
end


cal_ind(handles)
replot(handles)

function cal_ind(handles)
global cc p1 ind fmatrix;
num=size(cc,1);
% handles    empty - handles not created until after all CreateFcns called
[B,I] = sort(cc(:,3));
ind1=I(ceil(num*fmatrix(1,1)/100):ceil(num*fmatrix(1,2)/100));
[B,I] = sort(cc(:,6));
ind2=I(ceil(num*fmatrix(2,1)/100):ceil(num*fmatrix(2,2)/100));
[B,I] = sort(cc(:,4));
ind3=I(ceil(num*fmatrix(3,1)/100):ceil(num*fmatrix(3,2)/100));
[B,I] = sort(cc(:,5));
ind4=I(ceil(num*fmatrix(4,1)/100):ceil(num*fmatrix(4,2)/100));
[B,I] = sort(cc(:,1));
ind5=I(ceil(num*fmatrix(5,1)/100):ceil(num*fmatrix(5,2)/100));
[B,I] = sort(cc(:,2));
ind6=I(ceil(num*fmatrix(6,1)/100):ceil(num*fmatrix(6,2)/100));
ind=intersect(intersect(intersect(intersect(intersect(ind1,ind2),ind3),ind4),ind5),ind6);
% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on key press with focus on slider2 and none of its controls.
function slider2_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
global cc p1;

replot(handles)
% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global cc p1 ind fmatrix;
k=get(hObject,'value');

set(handles.text8,'String',num2str(100*k));
k1=str2num(get(handles.text7,'String'));
k2=str2num(get(handles.text8,'String'));
m=sort([k1 k2]);
k1=0.001+m(1);k2=m(2);
popup_sel_index = get(handles.popupmenu2,'Value');
num=size(cc,1);
switch popup_sel_index
    case 1 %depth
        fmatrix(1,:)=[k1 k2];
        [B,I] = sort(cc(:,3));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),3));
        set(handles.text12,'String',cc(indt(end),3));
    case 2 %lof
        fmatrix(2,:)=[k1 k2];
        [B,I] = sort(cc(:,6));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),6));
        set(handles.text12,'String',cc(indt(end),6));
    case 3 %magnitude
        fmatrix(3,:)=[k1 k2];
        [B,I] = sort(cc(:,4));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),4));
        set(handles.text12,'String',cc(indt(end),4));
    case 4 %time
        fmatrix(4,:)=[k1 k2];
        [B,I] = sort(cc(:,5));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        stime=datetime(get(handles.edit4,'String'),'InputFormat','uuuu-MM-dd''T''HH:mm:ss');
        set(handles.text11,'String',datestr(stime+cc(indt(1),5)));
        set(handles.text12,'String',datestr(stime+cc(indt(end),5)));
    case 5 %utm-x
        fmatrix(5,:)=[k1 k2];
        [B,I] = sort(cc(:,1));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),1));
        set(handles.text12,'String',cc(indt(end),1));
    case 6 %utm-y
        fmatrix(6,:)=[k1 k2];
        [B,I] = sort(cc(:,2));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),2));
        set(handles.text12,'String',cc(indt(end),2));
end
cal_ind(handles)
replot(handles)

function replot(handles)
global cc p1 ind;
popup_sel_index = get(handles.popupmenu1,'Value');
ptsize = (get(handles.popupmenu3,'Value'));
switch popup_sel_index
    
    
    case 1 %depth
        axes(handles.axes1);
        hold off;
        scatter3(cc(ind,1),cc(ind,2),cc(ind,3),ptsize,cc(ind,3),'filled');colormap(jet);axis equal;axis([min(cc(:,1)), max(cc(:,1)),min(cc(:,2)), max(cc(:,2)),min(cc(:,3)),0]);box on;
        
        axes(handles.axes3);
        hold off;
        
        scatter(p1(ind,1),p1(ind,2),ptsize,cc(ind,3),'filled');axis equal;axis([min(p1(:,1)), max(p1(:,1)),min(p1(:,2)), max(p1(:,2))]);box on;
    case 2 %lof
        axes(handles.axes1);
        hold off;
        scatter3(cc(ind,1),cc(ind,2),cc(ind,3),ptsize,cc(ind,6),'filled');colormap(jet);caxis([0 0.5]);axis equal;axis([min(cc(:,1)), max(cc(:,1)),min(cc(:,2)), max(cc(:,2)),min(cc(:,3)),0]); caxis([0 2]);  box on;
        axes(handles.axes3);
        hold off;
        scatter(p1(ind,1),p1(ind,2),ptsize,cc(ind,6),'filled');caxis([0 0.5]);axis equal;axis([min(p1(:,1)), max(p1(:,1)),min(p1(:,2)), max(p1(:,2))]);caxis([0 2]);box on;
        
    case 3 %magnitude
        axes(handles.axes1);
        hold off;
        scatter3(cc(ind,1),cc(ind,2),cc(ind,3),ptsize,cc(ind,4),'filled');colormap(jet);axis equal;axis([min(cc(:,1)), max(cc(:,1)),min(cc(:,2)), max(cc(:,2)),min(cc(:,3)),0]);  box on;
        axes(handles.axes3);
        hold off;
        scatter(p1(ind,1),p1(ind,2),ptsize,cc(ind,4),'filled');axis equal; axis([min(p1(:,1)), max(p1(:,1)),min(p1(:,2)), max(p1(:,2))]);box on;
        
    case 4 %time
        axes(handles.axes1);
        hold off;
        scatter3(cc(ind,1),cc(ind,2),cc(ind,3),ptsize,cc(ind,5),'filled');colormap(jet);axis equal;axis([min(cc(:,1)), max(cc(:,1)),min(cc(:,2)), max(cc(:,2)),min(cc(:,3)),0]);box on;
        axes(handles.axes3);
        hold off;
        scatter(p1(ind,1),p1(ind,2),ptsize,cc(ind,5),'filled');axis equal;axis([min(p1(:,1)), max(p1(:,1)),min(p1(:,2)), max(p1(:,2))]);box on;
        
    case 5 %distance to plane
        axes(handles.axes1);
        hold off;
        scatter3(cc(ind,1),cc(ind,2),cc(ind,3),ptsize,cc(ind,8),'filled');colormap(jet);axis equal;axis([min(cc(:,1)), max(cc(:,1)),min(cc(:,2)), max(cc(:,2)),min(cc(:,3)),0]);box on;
        axes(handles.axes3);
        hold off;
        scatter(p1(ind,1),p1(ind,2),ptsize,cc(ind,8),'filled');axis equal;axis([min(p1(:,1)), max(p1(:,1)),min(p1(:,2)), max(p1(:,2))]);box on;
        
end

% for animation
function replotani(handles)
global cc p1 ind;
popup_sel_index = get(handles.popupmenu1,'Value');
ptsize = (get(handles.popupmenu3,'Value'));
switch popup_sel_index
    
    
    case 1 %depth
        axes(handles.axes1);axiss=axis;
        hold off;
        scatter3(cc(ind,1),cc(ind,2),cc(ind,3),ptsize,cc(ind,3),'filled');colormap(jet);axis equal;axis(axiss);box on;
        
        axes(handles.axes3);axiss=axis;
        hold off;
        
        scatter(p1(ind,1),p1(ind,2),ptsize,cc(ind,3),'filled');axis equal;axis(axiss);box on;
    case 2 %lof
        axes(handles.axes1);axiss=axis;
        hold off;
        scatter3(cc(ind,1),cc(ind,2),cc(ind,3),ptsize,cc(ind,6),'filled');colormap(jet);axis equal;axis(axiss); caxis([0 2]);  box on;
        axes(handles.axes3);axiss=axis;
        hold off;
        scatter(p1(ind,1),p1(ind,2),ptsize,cc(ind,6),'filled');axis equal;axis(axiss);caxis([0 2]);box on;
        
    case 3 %magnitude
        axes(handles.axes1);axiss=axis;
        hold off;
        scatter3(cc(ind,1),cc(ind,2),cc(ind,3),ptsize,cc(ind,4),'filled');colormap(jet);axis equal;axis(axiss);  box on;
        axes(handles.axes3);axiss=axis;
        hold off;
        scatter(p1(ind,1),p1(ind,2),ptsize,cc(ind,4),'filled');axis equal;axis(axiss);box on;
        
    case 4 %time
        axes(handles.axes1);axiss=axis;
        hold off;
        scatter3(cc(ind,1),cc(ind,2),cc(ind,3),ptsize,cc(ind,5),'filled');colormap(jet);axis equal;axis(axiss);box on;
        axes(handles.axes3);axiss=axis;
        hold off;
        scatter(p1(ind,1),p1(ind,2),ptsize,cc(ind,5),'filled');axis equal;axis(axiss);box on;
        
    case 5 %distance to plane
        axes(handles.axes1);axiss=axis;
        hold off;
        scatter3(cc(ind,1),cc(ind,2),cc(ind,3),ptsize,cc(ind,8),'filled');colormap(jet);axis equal;axis(axiss);box on;
        axes(handles.axes3);axiss=axis;
        hold off;
        scatter(p1(ind,1),p1(ind,2),ptsize,cc(ind,8),'filled');axis equal;axis(axiss);box on;
        
end
% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2
global cc p1 ind fmatrix;
popup_sel_index = get(handles.popupmenu2,'Value');
num=size(cc,1);

switch popup_sel_index
    case 1 %depth
        k1=fmatrix(1,1);k2=fmatrix(1,2);
        set(handles.text7,'String',num2str(k1));
        set(handles.text8,'String',num2str(k2));
        set(handles.slider2,'value',k1/100);
        set(handles.slider3,'value',k2/100);
        [B,I] = sort(cc(:,3));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),3));
        set(handles.text12,'String',cc(indt(end),3));
    case 2 %lof
        k1=fmatrix(2,1);k2=fmatrix(2,2);
        set(handles.text7,'String',num2str(k1));
        set(handles.text8,'String',num2str(k2));
        set(handles.slider2,'value',k1/100);
        set(handles.slider3,'value',k2/100);
        [B,I] = sort(cc(:,6));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),6));
        set(handles.text12,'String',cc(indt(end),6));
    case 3 %magnitude
        k1=fmatrix(3,1);k2=fmatrix(3,2);
        set(handles.text7,'String',num2str(k1));
        set(handles.text8,'String',num2str(k2));
        set(handles.slider2,'value',k1/100);
        set(handles.slider3,'value',k2/100);
        [B,I] = sort(cc(:,4));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),4));
        set(handles.text12,'String',cc(indt(end),4));
    case 4 %time
        k1=fmatrix(4,1);k2=fmatrix(4,2);
        set(handles.text7,'String',num2str(k1));
        set(handles.text8,'String',num2str(k2));
        set(handles.slider2,'value',k1/100);
        set(handles.slider3,'value',k2/100);
        [B,I] = sort(cc(:,5));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        stime=datetime(get(handles.edit4,'String'),'InputFormat','uuuu-MM-dd''T''HH:mm:ss');
        set(handles.text11,'String',datestr(stime+cc(indt(1),5)));
        set(handles.text12,'String',datestr(stime+cc(indt(end),5)));
    case 5 %utm-x
        k1=fmatrix(5,1);k2=fmatrix(5,2);
        set(handles.text7,'String',num2str(k1));
        set(handles.text8,'String',num2str(k2));
        set(handles.slider2,'value',k1/100);
        set(handles.slider3,'value',k2/100);
        [B,I] = sort(cc(:,1));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),1));
        set(handles.text12,'String',cc(indt(end),1));
    case 6 %utm-y
        k1=fmatrix(6,1);k2=fmatrix(6,2);
        set(handles.text7,'String',num2str(k1));
        set(handles.text8,'String',num2str(k2));
        set(handles.slider2,'value',k1/100);
        set(handles.slider3,'value',k2/100);
        [B,I] = sort(cc(:,2));
        indt=I(ceil(num*k1/100):ceil(num*k2/100));
        set(handles.text11,'String',cc(indt(1),2));
        set(handles.text12,'String',cc(indt(end),2));
end



% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
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


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cc ind fmatrix dist lofvalue
[filename, pathname] = uigetfile( ...
    {'*.txt;','All Aftershock Files';...
    '*.*','All Files' },...
    'Select the aftershock text file', ...
    'MultiSelect', 'on');
cc=importdata(filename);
cc(:,3)=-cc(:,3);
dist=0*cc(:,3);

[tt,lofvalue]=LOFa(cc(:,1:3),30);
cc(:,6)=lofvalue;

fmatrix=[0 1;0 1;0 1;0 1;0 1;0 1];fmatrix(:,1)=0.00001;fmatrix(:,2)=100;
ind=1:size(cc,1);
scatter3(handles.axes1,cc(:,1),cc(:,2),cc(:,3),1,cc(:,3),'filled');axis equal;
hold on;
strike=str2num(get(handles.edit2,'String'));
dip=str2num(get(handles.edit3,'String'));
proj=str2num(get(handles.edit5,'String'));

n=[-sind(dip)*sind(strike-90),-sind(dip)*cosd(strike-90),cosd(dip)];
nlin=[-sind(dip)*sind(proj),-sind(dip)*cosd(proj),cosd(dip)];

% define the plane
XYZ=median(cc(:,1:3));
D=-sum(n.*XYZ);
% find the distance to the plane
for i=1:size(cc,1)
    [I,check]=plane_line_intersect(n,XYZ,cc(i,1:3),cc(i,1:3)-10*nlin);
    p0(i,1:3)=I;
    dist(i)=norm(I-cc(i,1:3));
end
cc(:,8)=dist;

% rotate along z
a=strike-90;
Rz=[cosd(a) -sind(a) 0;sind(a) cosd(a) 0;0 0 1];
% rotate along x
a=-dip;
Rx=[1 0 0;0 cosd(a) -sind(a);0 sind(a) cosd(a)];
axes(handles.axes3);
hold off;
p1 = Rx*Rz*p0(:,1:3)';%*;
p1=p1';
replot( handles);

% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cc p1 ind;
axes(handles.axes3);
[x,y] = ginput(2);
hold on;plot(x,y,'k-');hold on;plot(x(1),y(1),'r.');
for i=1:length(ind)
    a=p1(ind(i),1:2)-[x(1) y(1)];b=[x(2) y(2)]-[x(1) y(1)];
    dist(i)=norm(p1(ind(i),1:2)-[x(1) y(1)]).*(dot(a,b)/(norm(a)*norm(b)));
end
set(handles.text14,'String','Distance-time function');
axes(handles.axes4);
hold off;
plot(cc(ind,5),dist,'k.');xlabel(strcat('days after ',get(handles.edit4,'String')));ylabel('distance (km)');


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cc p1 ind;
axes(handles.axes3);
[x,y] = ginput(4);
x=[x;x(1)];y=[y;y(1)];
hold on;plot(x,y,'k-.');hold on;
in=inpolygon(p1(ind,1),p1(ind,2),x,y);
indin=find(in==1);
axes(handles.axes4);
hold off;
plot3(cc(ind(indin),1),cc(ind(indin),2),cc(ind(indin),3),'k.');
[A,B,C,D]=fit_point(cc(ind(indin),1:3));
hold on;
disp(strcat('plane equation:',num2str([A B C D])));
normal=[A B C]*sign(C);
normal = normal/norm(normal); n_e = [0 0 1];
dip = acosd(abs(dot(normal,n_e)));
theta = atan2d(normal(1), normal(2)) -90;
disp(strcat('dip:',num2str(dip)));
disp(strcat('strike:',num2str(theta)));

dep=-min(cc(ind(indin),3));
corner(1,1)=min(cc(ind(indin),1));corner(1,2)=max(cc(ind(indin),2));corner(1,2)=(D-A*corner(1,1))/B;
corner(4,1)=max(cc(ind(indin),1));corner(4,2)=max(cc(ind(indin),2));corner(4,2)=(D-A*corner(4,1))/B;
fault(1)=theta;fault(2)=dip;fault(3)=(corner(1,1)+corner(4,1))/2;fault(4)=(corner(1,2)+corner(4,2))/2;fault(5)=norm(corner(1,:)-corner(4,:));fault(6)=0;fault(7)=dep;
disp(strcat('plane parameter:',num2str(fault)));

strike=90-fault(1);dip=90-fault(2);x_utm=fault(3);y_utm=fault(4);len=fault(5);dep_up=fault(6);dep_down=fault(7);
lef_cs=[x_utm-0.5*len*cosd(strike) y_utm-0.5*len*sind(strike)];
rig_cs=[x_utm+0.5*len*cosd(strike) y_utm+0.5*len*sind(strike)];

lef_cu=lef_cs+[dep_up*tand(dip)*sind(strike) -dep_up*tand(dip)*cosd(strike)];
rig_cu=rig_cs+[dep_up*tand(dip)*sind(strike) -dep_up*tand(dip)*cosd(strike)];
lef_cd=lef_cs+[dep_down*tand(dip)*sind(strike) -dep_down*tand(dip)*cosd(strike)];
rig_cd=rig_cs+[dep_down*tand(dip)*sind(strike) -dep_down*tand(dip)*cosd(strike)];

corner=[lef_cu;rig_cu;rig_cd;lef_cd;lef_cu];corner(:,3)=[dep_up;dep_up;dep_down;dep_down;dep_up];
hold on;
plot3(corner(:,1),corner(:,2),-corner(:,3),'k-','linewidth',2);
set(handles.text14,'String','Inferred fault');
axes(handles.axes1);
hold on;
plot3(corner(:,1),corner(:,2),-corner(:,3));
hold on;



% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cc p1 ind fmatrix;

fmatrix=[0 1;0 1;0 1;0 1;0 1;0 1];fmatrix(:,1)=0.00001;fmatrix(:,2)=100;
ind=1:size(cc,1);
popupmenu2_Callback(hObject, eventdata, handles)
replot(handles);



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


% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu3
replot(handles);

% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cc p1 ind fmatrix;

kk1=get(handles.slider2,'value');
kk2=get(handles.slider3,'value');


% set(handles.text8,'String',num2str(100*k));
% k1=str2num(get(handles.text7,'String'));
% k2=str2num(get(handles.text8,'String'));

writerObj=VideoWriter('ani.avi');  %// 定义一个视频文件用来存动画
writerObj2=VideoWriter('ani2.avi');  %// 定义一个视频文件用来存动画
open(writerObj);                    %// 打开该视频文件
open(writerObj2);
%动画部分代码
for kk=1:300*(kk2-kk1)
    set(handles.slider3,'value',kk1+kk/300);
    
    k=get(handles.slider3,'value');
    
    set(handles.text8,'String',num2str(100*k));
    k1=str2num(get(handles.text7,'String'));
    k2=str2num(get(handles.text8,'String'));
    m=sort([k1 k2]);
    k1=0.001+m(1);k2=m(2);
    popup_sel_index = get(handles.popupmenu2,'Value');
    num=size(cc,1);
    switch popup_sel_index
        case 1 %depth
            fmatrix(1,:)=[k1 k2];
            [B,I] = sort(cc(:,3));
            indt=I(ceil(num*k1/100):ceil(num*k2/100));
            set(handles.text11,'String',cc(indt(1),3));
            set(handles.text12,'String',cc(indt(end),3));
        case 2 %lof
            fmatrix(2,:)=[k1 k2];
            [B,I] = sort(cc(:,6));
            indt=I(ceil(num*k1/100):ceil(num*k2/100));
            set(handles.text11,'String',cc(indt(1),6));
            set(handles.text12,'String',cc(indt(end),6));
        case 3 %magnitude
            fmatrix(3,:)=[k1 k2];
            [B,I] = sort(cc(:,4));
            indt=I(ceil(num*k1/100):ceil(num*k2/100));
            set(handles.text11,'String',cc(indt(1),4));
            set(handles.text12,'String',cc(indt(end),4));
        case 4 %time
            fmatrix(4,:)=[k1 k2];
            [B,I] = sort(cc(:,5));
            indt=I(ceil(num*k1/100):ceil(num*k2/100));
            stime=datetime(get(handles.edit4,'String'),'InputFormat','uuuu-MM-dd''T''HH:mm:ss');
            set(handles.text11,'String',datestr(stime+cc(indt(1),5)));
            set(handles.text12,'String',datestr(stime+cc(indt(end),5)));
        case 5 %utm-x
            fmatrix(5,:)=[k1 k2];
            [B,I] = sort(cc(:,1));
            indt=I(ceil(num*k1/100):ceil(num*k2/100));
            set(handles.text11,'String',cc(indt(1),1));
            set(handles.text12,'String',cc(indt(end),1));
        case 6 %utm-y
            fmatrix(6,:)=[k1 k2];
            [B,I] = sort(cc(:,2));
            indt=I(ceil(num*k1/100):ceil(num*k2/100));
            set(handles.text11,'String',cc(indt(1),2));
            set(handles.text12,'String',cc(indt(end),2));
    end
    cal_ind(handles)
    replotani(handles)
    
    
    frame = getframe(handles.axes3);
    writeVideo(writerObj,frame);
    
    frame2 = getframe(handles.axes1);
    writeVideo(writerObj2,frame2);
end
set(handles.slider3,'value',kk2);
cal_ind(handles)
replotani(handles)
frame = getframe;
writeVideo(writerObj,frame);
close(writerObj);


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
new_f_handle=figure('visible','off');colormap(jet);
set(new_f_handle,'units','normalized','position',[0.1 0.1 0.8 0.8]);
new_axes=copyobj(handles.axes1,new_f_handle); %picture是GUI界面绘图的坐标系句柄
set(new_axes,'units','default','position',[0.1 0.1 0.8 0.8]);
[filename,pathname]=uiputfile({'*.png'},'save picture as');
if ~filename
    return
else
    file=strcat(pathname,strcat('1',filename));
    print(new_f_handle,'-dpng','-r300',file);
end

delete(new_f_handle);

new_f_handle=figure('visible','off');colormap(jet);
new_axes=copyobj(handles.axes3,new_f_handle);
set(new_axes,'units','default','position','default');

if ~filename
    return
else
    file=strcat(pathname,strcat('2',filename));
    print(new_f_handle,'-dpng','-r300',file);
end
delete(new_f_handle);

new_f_handle=figure('visible','off');colormap(jet);
new_axes=copyobj(handles.axes4,new_f_handle);
set(new_axes,'units','default','position','default');

if ~filename
    return
else
    file=strcat(pathname,strcat('3',filename));
    print(new_f_handle,'-dpng','-r300',file);
end
delete(new_f_handle);
