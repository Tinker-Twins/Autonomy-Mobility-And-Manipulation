function varargout = Problem_1_GUI(varargin)
% PROBLEM_1_GUI MATLAB code for PROBLEM_1_GUI.fig
%      PROBLEM_1_GUI, by itself, creates a new PROBLEM_1_GUI or raises the existing
%      singleton*.
%
%      H = PROBLEM_1_GUI returns the handle to a new PROBLEM_1_GUI or the handle to
%      the existing singleton*.
%
%      PROBLEM_1_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PROBLEM_1_GUI.M with the given input arguments.
%
%      PROBLEM_1_GUI('Property','Value',...) creates a new PROBLEM_1_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the PROBLEM_1_GUI before GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PROBLEM_1_GUI

% Last Modified by GUIDE v2.5 08-Oct-2022 18:18:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_OutputFcn, ...
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


% --- Executes just before PROBLEM_1_GUI is made visible.
function GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PROBLEM_1_GUI (see VARARGIN)

% Choose default command line output for PROBLEM_1_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes PROBLEM_1_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Solve1A.
function Solve1A_Callback(hObject, eventdata, handles)
% hObject    handle to Solve1A (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Interpolate individual angles (relative ZYZ)
z11=linspace(0,((pi/4)),100);
y12=linspace(0,((pi/6)),100);
z13=linspace(0,((pi/9)),100);

% Compute rotation for each of the interpolation
for i=1:length(z11)
Rz11(:,:,i) = [cos(z11(i)) -sin(z11(i)) 0;
               sin(z11(i)) cos(z11(i))  0;
               0           0            1];

Ry12(:,:,i) = [cos(y12(i))  0 sin(y12(i));
               0            1 0          ;
               -sin(y12(i)) 0 cos(y12(i))];

Rz13(:,:,i) = [cos(z13(i)) -sin(z13(i)) 0;
               sin(z13(i)) cos(z13(i))  0;
               0           0            1];
Rzyz(:,:,i) = Rz11(:,:,i)*Ry12(:,:,i)*Rz13(:,:,i);
end

% Plot
axes(handles.Plot1)
trplot(Rzyz(:,:,1),'rgb','frame',num2str(0)) % Using PCRTB ONLY for plotting
%poseplot(Rzyz(:,:,1)) % Another way to plot rotation matrix, but the convention is weird
hold on
for i=1:length(Rzyz)
    if mod(i,20)==0
        trplot(Rzyz(:,:,i),'rgb','frame',num2str(i/20)) % Using PCRTB ONLY for plotting
        %poseplot(Rzyz(:,:,i)) % Another way to plot rotation matrix, but the convention is weird
        hold on
    end
end
hold off

% Animation
axes(handles.Plot2)
tranimate(Rzyz,'rgb','movie','Results\Problem 1\Animation 1-A.mp4');


% --- Executes on button press in Solve1B.
function Solve1B_Callback(hObject, eventdata, handles)
% hObject    handle to Solve1B (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Compute relative Rzyz from given angles
Rz11 = [cos(pi/4) -sin(pi/4) 0;
        sin(pi/4) cos(pi/4)  0;
        0         0          1];
Ry12 = [cos(pi/6)  0 sin(pi/6);
        0          1 0        ;
        -sin(pi/6) 0 cos(pi/6)];
Rz13 = [cos(pi/9) -sin(pi/9) 0;
        sin(pi/9) cos(pi/9)  0;
        0         0          1];
Rzyz = Rz11*Ry12*Rz13;

% Compute absolute Rxyz from relative Rzyz 
phi1 = atan2(Rzyz(3,2),Rzyz(3,3));
phi2 = atan2(-Rzyz(3,2),-Rzyz(3,3));
theta1 = atan2(-Rzyz(3,1),sqrt((Rzyz(1,1)^2)+(Rzyz(2,1)^2)));
theta2 = atan2(-Rzyz(3,1),-sqrt((Rzyz(1,1)^2)+(Rzyz(2,1)^2)));
psi1 = atan2(Rzyz(2,1),Rzyz(1,1));
psi2 = atan2(-Rzyz(2,1),-Rzyz(1,1));

% Interpolate individual angles (one set of absolute XYZ)
phi=linspace(0,phi1,100);
theta=linspace(0,theta1,100);
psi=linspace(0,psi1,100);

% Compute rotation for each of the interpolation
for i=1:length(phi)
Rx_phi(:,:,i) = [1 0           0           ;
                 0 cos(phi(i)) -sin(phi(i));
                 0 sin(phi(i)) cos(phi(i)) ];

Ry_theta(:,:,i) = [cos(theta(i))  0 sin(theta(i));
                   0              1 0            ;
                   -sin(theta(i)) 0 cos(theta(i))];

Rz_psi(:,:,i) = [cos(psi(i)) -sin(psi(i)) 0;
                 sin(psi(i)) cos(psi(i))  0;
                 0           0            1];
Rxyz(:,:,i) = Rz_psi(:,:,i)*Ry_theta(:,:,i)*Rx_phi(:,:,i);
end

% Plot
axes(handles.Plot1)
trplot(Rxyz(:,:,1),'rgb','frame',num2str(0)) % Using PCRTB ONLY for plotting
%poseplot(Rxyz(:,:,1)) % Another way to plot rotation matrix, but the convention is weird
hold on
for i=1:length(Rxyz)
    if mod(i,20)==0
        trplot(Rxyz(:,:,i),'rgb','frame',num2str(i/20)) % Using PCRTB ONLY for plotting
        %poseplot(Rxyz(:,:,i)) % Another way to plot rotation matrix, but the convention is weird
        hold on
    end
end
hold off

% Animation
axes(handles.Plot2)
tranimate(Rxyz,'rgb','movie','Results\Problem 1\Animation 1-B.mp4');


% --- Executes on button press in Solve1C.
function Solve1C_Callback(hObject, eventdata, handles)
% hObject    handle to Solve1C (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% ZYZ relative rotation
Ri = rotz(0)*roty(0)*rotz(0);
Rf = rotz(rad2deg(pi/4))*roty(rad2deg(pi/6))*rotz(rad2deg(pi/9));

% Convert to homogenous transformation
Ti = r2t(Ri);
Tf = r2t(Rf);

% Interpolate and plot
Tint = trinterp(Ti, Tf, 100);
axes(handles.Plot1)
trplot(Ti,'rgb','frame',num2str(0))
hold on
for i=1:length(Tint)
    if mod(i,20)==0
        trplot(Tint(:,:,i),'rgb','frame',num2str(i/20))
    end
end
hold off

% Animation
axes(handles.Plot2)
tranimate(Tint,'rgb','movie','Results\Problem 1\Animation 1-C.mp4');


% --- Executes on button press in Solve1D.
function Solve1D_Callback(hObject, eventdata, handles)
% hObject    handle to Solve1D (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% ZYZ relative rotation
Ri = rotz(0)*roty(0)*rotz(0);
Rf = rotz(rad2deg(pi/4))*roty(rad2deg(pi/6))*rotz(rad2deg(pi/9));

% Convert to PRY absolute roll-pitch-yaw angles
RPYi = rad2deg(tr2rpy(Ri)); % PCRTB v10 default is absolute XYZ convention (i.e., relative ZYX)
RPYf = rad2deg(tr2rpy(Rf)); % PCRTB v10 default is absolute XYZ convention (i.e., relative ZYX)

% Convert to homogenous transformation
Ti = rpy2tr(RPYi);
Tf = rpy2tr(RPYf);

% Interpolate and plot
Tint = trinterp(Ti, Tf, 100);
axes(handles.Plot1)
trplot(Ti,'rgb','frame',num2str(0))
hold on
for i=1:length(Tint)
    if mod(i,20)==0
        trplot(Tint(:,:,i),'rgb','frame',num2str(i/20))
    end
end
hold off

% Animation
axes(handles.Plot2)
tranimate(Tint,'rgb','movie','Results\Problem 1\Animation 1-D.mp4');
