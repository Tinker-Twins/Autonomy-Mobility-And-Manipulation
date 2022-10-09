function varargout = Problem_2_GUI(varargin)
% PROBLEM_2_GUI MATLAB code for PROBLEM_2_GUI.fig
%      PROBLEM_2_GUI, by itself, creates a new PROBLEM_2_GUI or raises the existing
%      singleton*.
%
%      H = PROBLEM_2_GUI returns the handle to a new PROBLEM_2_GUI or the handle to
%      the existing singleton*.
%
%      PROBLEM_2_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PROBLEM_2_GUI.M with the given input arguments.
%
%      PROBLEM_2_GUI('Property','Value',...) creates a new PROBLEM_2_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the PROBLEM_2_GUI before GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PROBLEM_2_GUI

% Last Modified by GUIDE v2.5 08-Oct-2022 18:14:49

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


% --- Executes just before PROBLEM_2_GUI is made visible.
function GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PROBLEM_2_GUI (see VARARGIN)

% Choose default command line output for PROBLEM_2_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes PROBLEM_2_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Solve2A.
function Solve2A_Callback(hObject, eventdata, handles)
% hObject    handle to Solve2A (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Robot parameters
a1 = 2;
a2 = 1;

% Robot definition using PCRTB (DH parametrization)
L(1) = Link([0 0 2 0 0],'R'); % First '0' doesn't mean 0. MATLAB takes it in as a variable 'theta' because we specify 'R', which defines a revolute joint
L(2) = Link([0 0 1 0 0],'R');
Robot = SerialLink(L, 'name', '2-R Planar Robot'); % Concatenate the links as a serial robot

% Cubic polynomial interpolation for x coordinate
ti = 0; % Initial time
xi = -1; % Initial x position
vi = 0; % Initial x velocity
tf = 10; % Final time
xf = 2; % Final x position
vf = 0; % Final x velocity
%x(t) = a0 + a1t + a2t^2 + a3t^3
a = [1 ti ti^2 ti^3; % Matrix of time (t^0 - t^3) terms
     0 1  2*ti 3*ti^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2];
c = [xi; vi; xf; vf]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a3)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
x_2a = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3; % x trajectory

% Cubic polynomial interpolation for y coordinate
ti = 0; % Initial time
yi = 1; % Initial y position
vi = 0; % Initial y velocity
tf = 10; % Final time
yf = 1; % Final y position
vf = 0; % Final y velocity
%y(t) = b0 + b1t + b2t^2 + b3t^3
a = [1 ti ti^2 ti^3; % Matrix of time (t^0 - t^3) terms
     0 1  2*ti 3*ti^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2];
c = [yi; vi; yf; vf]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a3)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
y_2a = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3; % y trajectory

% IK of 2R planar robot to find theta1 and theta2 for all (x,y) positions
% NOTE: Using only 1 (elbow down) of 2 possible solutions
for i=1:length(t)
    theta2_2a(i) = acos((x_2a(i)^2+y_2a(i)^2-a1^2-a2^2)/(2*a1*a2));
    theta1_2a(i) = atan2(y_2a(i),x_2a(i)) - atan2((a2*sin(theta2_2a(i))),(a1+a2*cos(theta2_2a(i))));
end

% Plots
% x vs. t
axes(handles.Plot1)
plot(t,x_2a)
xlabel('Time (s)')
ylabel('Position (x)')
% y vs. t
axes(handles.Plot2)
plot(t,y_2a)
xlabel('Time (s)')
ylabel('Position (y)')
% y vs. x
axes(handles.Plot3)
plot(x_2a,y_2a)
xlabel('Position (x)')
ylabel('Position (y)')
% theta1 vs. t
axes(handles.Plot4)
plot(t,rad2deg(theta1_2a))
xlabel('Time (s)')
ylabel('\theta_1 (deg)')
% theta2 vs. t
axes(handles.Plot5)
plot(t,rad2deg(theta2_2a))
xlabel('Time (s)')
ylabel('\theta_2 (deg)')
% theta2 vs. theta1
axes(handles.Plot6)
plot(rad2deg(theta1_2a),rad2deg(theta2_2a))
xlabel('\theta_1 (deg)')
ylabel('\theta_2 (deg)')
% Robot animation
axes(handles.Plot7)
plot(x_2a,y_2a);
Robot.plot([theta1_2a', theta2_2a'],'movie','Results\Problem 2\Animation 2-A.mp4');


% --- Executes on button press in Solve2B.
function Solve2B_Callback(hObject, eventdata, handles)
% hObject    handle to Solve2B (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Robot parameters
a1 = 2;
a2 = 1;

% Robot definition using PCRTB (DH parametrization)
L(1) = Link([0 0 2 0 0],'R'); % First '0' doesn't mean 0. MATLAB takes it in as a variable 'theta' because we specify 'R', which defines a revolute joint
L(2) = Link([0 0 1 0 0],'R');
Robot = SerialLink(L, 'name', '2-R Planar Robot'); % Concatenate the links as a serial robot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cubic polynomial interpolation for x coordinate
ti = 0; % Initial time
xi = -1; % Initial x position
vi = 0; % Initial x velocity
tf = 10; % Final time
xf = 2; % Final x position
vf = 0; % Final x velocity
%x(t) = a0 + a1t + a2t^2 + a3t^3
a = [1 ti ti^2 ti^3; % Matrix of time (t^0 - t^3) terms
     0 1  2*ti 3*ti^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2];
c = [xi; vi; xf; vf]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a3)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
x_2a = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3; % x trajectory

% Cubic polynomial interpolation for y coordinate
ti = 0; % Initial time
yi = 1; % Initial y position
vi = 0; % Initial y velocity
tf = 10; % Final time
yf = 1; % Final y position
vf = 0; % Final y velocity
%y(t) = b0 + b1t + b2t^2 + b3t^3
a = [1 ti ti^2 ti^3; % Matrix of time (t^0 - t^3) terms
     0 1  2*ti 3*ti^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2];
c = [yi; vi; yf; vf]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a3)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
y_2a = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3; % y trajectory

% IK of 2R planar robot to find theta1 and theta2 for all (x,y) positions
% NOTE: Using only 1 (elbow down) of 2 possible solutions
for i=1:length(t)
    theta2_2a(i) = acos((x_2a(i)^2+y_2a(i)^2-a1^2-a2^2)/(2*a1*a2));
    theta1_2a(i) = atan2(y_2a(i),x_2a(i)) - atan2((a2*sin(theta2_2a(i))),(a1+a2*cos(theta2_2a(i))));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Quintic polynomial interpolation for joint angle 1 (theta1)
ti = 0; % Initial time
qi = theta1_2a(1); % Initial theta1 position
vi = 0; % Initial theta1 velocity
ai = 0; % Initial theta1 acceleration
tf = 10; % Final time
qf = theta1_2a(end); % Final theta1 position
vf = 0; % Final theta1 velocity
af = 0; % Final theta1 acceleration
%theta_1(t) = a0 + a1t + a2t^2 + a3t^3 + a4t^4 + a5t^5
a = [1 ti ti^2 ti^3   ti^4    ti^5; % Matrix of time (t^0 - t^5) terms
     0 1  2*ti 3*ti^2 4*ti^3  5*ti^4;
     0 0  2    6*ti   12*ti^2 20*ti^3;
     1 tf tf^2 tf^3   tf^4    tf^5;
     0 1  2*tf 3*tf^2 4*tf^3  5*tf^4;
     0 0  2    6*tf   12*tf^2 20*tf^3;];
c = [qi; vi; ai; qf; vf; af]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a5)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
theta1_2b = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3 + b(5).*t.^4 + b(6).*t.^5; % theta1 trajectory

% Quintic polynomial interpolation for joint angle 2 (theta2)
ti = 0; % Initial time
qi = theta2_2a(1); % Initial theta2 position
vi = 0; % Initial theta2 velocity
ai = 0; % Initial theta2 acceleration
tf = 10; % Final time
qf = theta2_2a(end); % Final theta2 position
vf = 0; % Final theta2 velocity
af = 0; % Final theta2 acceleration
%theta_2(t) = b0 + b1t + b2t^2 + b3t^3 + b4t^4 + b5t^5
a = [1 ti ti^2 ti^3   ti^4    ti^5; % Matrix of time (t^0 - t^5) terms
     0 1  2*ti 3*ti^2 4*ti^3  5*ti^4;
     0 0  2    6*ti   12*ti^2 20*ti^3;
     1 tf tf^2 tf^3   tf^4    tf^5;
     0 1  2*tf 3*tf^2 4*tf^3  5*tf^4;
     0 0  2    6*tf   12*tf^2 20*tf^3;];
c = [qi; vi; ai; qf; vf; af]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a5)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
theta2_2b = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3 + b(5).*t.^4 + b(6).*t.^5; % theta2 trajectory

% FK of 2R planar robot to find (x,y) positions for all theta1 and theta2
for i=1:length(t)
    x_2b(i) = a1*cos(theta1_2b(i))+a2*cos(theta1_2b(i)+theta2_2b(i));
    y_2b(i) = a1*sin(theta1_2b(i))+a2*sin(theta1_2b(i)+theta2_2b(i));
end

% Plots
% x vs. t
axes(handles.Plot1)
plot(t,x_2b)
xlabel('Time (s)')
ylabel('Position (x)')
% y vs. t
axes(handles.Plot2)
plot(t,y_2b)
xlabel('Time (s)')
ylabel('Position (y)')
% y vs. x
axes(handles.Plot3)
plot(x_2b,y_2b)
xlabel('Position (x)')
ylabel('Position (y)')
% theta1 vs. t
axes(handles.Plot4)
plot(t,rad2deg(theta1_2b))
xlabel('Time (s)')
ylabel('\theta_1 (deg)')
% theta2 vs. t
axes(handles.Plot5)
plot(t,rad2deg(theta2_2b))
xlabel('Time (s)')
ylabel('\theta_2 (deg)')
% theta2 vs. theta1
axes(handles.Plot6)
plot(rad2deg(theta1_2b),rad2deg(theta2_2b))
xlabel('\theta_1 (deg)')
ylabel('\theta_2 (deg)')
% Robot animation
axes(handles.Plot7)
plot(x_2b,y_2b);
Robot.plot([theta1_2b', theta2_2b'],'movie','Results\Problem 2\Animation 2-B.mp4');


% --- Executes on button press in Solve2C.
function Solve2C_Callback(hObject, eventdata, handles)
% hObject    handle to Solve2C (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Robot parameters
a1 = 2;
a2 = 1;

% Robot definition using PCRTB (DH parametrization)
L(1) = Link([0 0 2 0 0],'R'); % First '0' doesn't mean 0. MATLAB takes it in as a variable 'theta' because we specify 'R', which defines a revolute joint
L(2) = Link([0 0 1 0 0],'R');
Robot = SerialLink(L, 'name', '2-R Planar Robot'); % Concatenate the links as a serial robot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cubic polynomial interpolation for x coordinate
ti = 0; % Initial time
xi = -1; % Initial x position
vi = 0; % Initial x velocity
tf = 10; % Final time
xf = 2; % Final x position
vf = 0; % Final x velocity
%x(t) = a0 + a1t + a2t^2 + a3t^3
a = [1 ti ti^2 ti^3; % Matrix of time (t^0 - t^3) terms
     0 1  2*ti 3*ti^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2];
c = [xi; vi; xf; vf]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a3)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
x_2a = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3; % x trajectory

% Cubic polynomial interpolation for y coordinate
ti = 0; % Initial time
yi = 1; % Initial y position
vi = 0; % Initial y velocity
tf = 10; % Final time
yf = 1; % Final y position
vf = 0; % Final y velocity
%y(t) = b0 + b1t + b2t^2 + b3t^3
a = [1 ti ti^2 ti^3; % Matrix of time (t^0 - t^3) terms
     0 1  2*ti 3*ti^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2];
c = [yi; vi; yf; vf]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a3)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
y_2a = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3; % y trajectory

% IK of 2R planar robot to find theta1 and theta2 for all (x,y) positions
% NOTE: Using only 1 (elbow down) of 2 possible solutions
for i=1:length(t)
    theta2_2a(i) = acos((x_2a(i)^2+y_2a(i)^2-a1^2-a2^2)/(2*a1*a2));
    theta1_2a(i) = atan2(y_2a(i),x_2a(i)) - atan2((a2*sin(theta2_2a(i))),(a1+a2*cos(theta2_2a(i))));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Quintic polynomial interpolation for x coordinate
x_2c = tpoly(-1,2,100);
% Quintic polynomial interpolation for y coordinate
y_2c = tpoly(1,1,100);

% IK of 2R planar robot to find theta1 and theta2 for all (x,y) positions
% NOTE: Using only 1 (elbow down) of 2 possible solutions by initializing
%       'q0' values for each joint angle using IK solution from 2 - A.
for i=1:length(t)
    theta_2c(i,:) = Robot.ikine(transl(x_2c(i),y_2c(i), 0),'q0',[theta1_2a(i),theta2_2a(i)],'mask',[1,1,0,0,0,0]); % Mask vector (1x6 - xyzrpy) specifies the Cartesian DOF that will be ignored in reaching IK solution
end

% Plots
% x vs. t
axes(handles.Plot1)
plot(t,x_2c)
xlabel('Time (s)')
ylabel('Position (x)')
% y vs. t
axes(handles.Plot2)
plot(t,y_2c)
xlabel('Time (s)')
ylabel('Position (y)')
% y vs. x
axes(handles.Plot3)
plot(x_2c,y_2c)
xlabel('Position (x)')
ylabel('Position (y)')
% theta1 vs. t
axes(handles.Plot4)
plot(t,rad2deg(theta_2c(:,1)))
xlabel('Time (s)')
ylabel('\theta_1 (deg)')
% theta2 vs. t
axes(handles.Plot5)
plot(t,rad2deg(theta_2c(:,2)))
xlabel('Time (s)')
ylabel('\theta_2 (deg)')
% theta2 vs. theta1
axes(handles.Plot6)
plot(rad2deg(theta_2c(:,1)),rad2deg(theta_2c(:,2)))
xlabel('\theta_1 (deg)')
ylabel('\theta_2 (deg)')
% Robot animation
axes(handles.Plot7)
plot(x_2c,y_2c);
Robot.plot([theta_2c(:,1), theta_2c(:,2)],'movie','Results\Problem 2\Animation 2-C.mp4');

% --- Executes on button press in Solve2D.
function Solve2D_Callback(hObject, eventdata, handles)
% hObject    handle to Solve2D (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Robot parameters
a1 = 2;
a2 = 1;

% Robot definition using PCRTB (DH parametrization)
L(1) = Link([0 0 2 0 0],'R'); % First '0' doesn't mean 0. MATLAB takes it in as a variable 'theta' because we specify 'R', which defines a revolute joint
L(2) = Link([0 0 1 0 0],'R');
Robot = SerialLink(L, 'name', '2-R Planar Robot'); % Concatenate the links as a serial robot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cubic polynomial interpolation for x coordinate
ti = 0; % Initial time
xi = -1; % Initial x position
vi = 0; % Initial x velocity
tf = 10; % Final time
xf = 2; % Final x position
vf = 0; % Final x velocity
%x(t) = a0 + a1t + a2t^2 + a3t^3
a = [1 ti ti^2 ti^3; % Matrix of time (t^0 - t^3) terms
     0 1  2*ti 3*ti^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2];
c = [xi; vi; xf; vf]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a3)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
x_2a = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3; % x trajectory

% Cubic polynomial interpolation for y coordinate
ti = 0; % Initial time
yi = 1; % Initial y position
vi = 0; % Initial y velocity
tf = 10; % Final time
yf = 1; % Final y position
vf = 0; % Final y velocity
%y(t) = b0 + b1t + b2t^2 + b3t^3
a = [1 ti ti^2 ti^3; % Matrix of time (t^0 - t^3) terms
     0 1  2*ti 3*ti^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2];
c = [yi; vi; yf; vf]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a3)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
y_2a = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3; % y trajectory

% IK of 2R planar robot to find theta1 and theta2 for all (x,y) positions
% NOTE: Using only 1 (elbow down) of 2 possible solutions
for i=1:length(t)
    theta2_2a(i) = acos((x_2a(i)^2+y_2a(i)^2-a1^2-a2^2)/(2*a1*a2));
    theta1_2a(i) = atan2(y_2a(i),x_2a(i)) - atan2((a2*sin(theta2_2a(i))),(a1+a2*cos(theta2_2a(i))));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Quintic polynomial interpolation for joint angle 1 (theta1)
ti = 0; % Initial time
qi = theta1_2a(1); % Initial theta1 position
vi = 0; % Initial theta1 velocity
ai = 0; % Initial theta1 acceleration
tf = 10; % Final time
qf = theta1_2a(end); % Final theta1 position
vf = 0; % Final theta1 velocity
af = 0; % Final theta1 acceleration
%theta_1(t) = a0 + a1t + a2t^2 + a3t^3 + a4t^4 + a5t^5
a = [1 ti ti^2 ti^3   ti^4    ti^5; % Matrix of time (t^0 - t^5) terms
     0 1  2*ti 3*ti^2 4*ti^3  5*ti^4;
     0 0  2    6*ti   12*ti^2 20*ti^3;
     1 tf tf^2 tf^3   tf^4    tf^5;
     0 1  2*tf 3*tf^2 4*tf^3  5*tf^4;
     0 0  2    6*tf   12*tf^2 20*tf^3;];
c = [qi; vi; ai; qf; vf; af]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a5)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
theta1_2b = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3 + b(5).*t.^4 + b(6).*t.^5; % theta1 trajectory

% Quintic polynomial interpolation for joint angle 2 (theta2)
ti = 0; % Initial time
qi = theta2_2a(1); % Initial theta2 position
vi = 0; % Initial theta2 velocity
ai = 0; % Initial theta2 acceleration
tf = 10; % Final time
qf = theta2_2a(end); % Final theta2 position
vf = 0; % Final theta2 velocity
af = 0; % Final theta2 acceleration
%theta_2(t) = b0 + b1t + b2t^2 + b3t^3 + b4t^4 + b5t^5
a = [1 ti ti^2 ti^3   ti^4    ti^5; % Matrix of time (t^0 - t^5) terms
     0 1  2*ti 3*ti^2 4*ti^3  5*ti^4;
     0 0  2    6*ti   12*ti^2 20*ti^3;
     1 tf tf^2 tf^3   tf^4    tf^5;
     0 1  2*tf 3*tf^2 4*tf^3  5*tf^4;
     0 0  2    6*tf   12*tf^2 20*tf^3;];
c = [qi; vi; ai; qf; vf; af]; % Vector of initial & final conditions
b = inv(a)*c; % Vector of polynomial coefficiants (a0 - a5)
t = linspace(ti, tf, 100); % Time steps/increments for smooth transition
theta2_2b = b(1).*ones(size(t)) + b(2).*t + b(3).*t.^2 + b(4).*t.^3 + b(5).*t.^4 + b(6).*t.^5; % theta2 trajectory

% FK of 2R planar robot to find (x,y) positions for all theta1 and theta2
for i=1:length(t)
    x_2b(i) = a1*cos(theta1_2b(i))+a2*cos(theta1_2b(i)+theta2_2b(i));
    y_2b(i) = a1*sin(theta1_2b(i))+a2*sin(theta1_2b(i)+theta2_2b(i));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Quintic polynomial interpolation for theta1 position
theta1_2d = jtraj(theta1_2b(1),theta1_2b(end),100);
% Quintic polynomial interpolation for theta2 position
theta2_2d = jtraj(theta2_2b(1),theta2_2b(end),100);

% FK of 2R planar robot to find (x,y) positions for all theta1 and theta2
for i=1:length(t)
    T_2d = Robot.fkine([theta1_2d,theta2_2d]);
    xyz_2d(:,i) = transl(T_2d(i));
end

% Plots
% x vs. t
axes(handles.Plot1)
plot(t,xyz_2d(1,:))
xlabel('Time (s)')
ylabel('Position (x)')
% y vs. t
axes(handles.Plot2)
plot(t,xyz_2d(2,:))
xlabel('Time (s)')
ylabel('Position (y)')
% y vs. x
axes(handles.Plot3)
plot(xyz_2d(1,:),xyz_2d(2,:))
xlabel('Position (x)')
ylabel('Position (y)')
% theta1 vs. t
axes(handles.Plot4)
plot(t,rad2deg(theta1_2d))
xlabel('Time (s)')
ylabel('\theta_1 (deg)')
% theta2 vs. t
axes(handles.Plot5)
plot(t,rad2deg(theta2_2d))
xlabel('Time (s)')
ylabel('\theta_2 (deg)')
% theta2 vs. theta1
axes(handles.Plot6)
plot(rad2deg(theta1_2d),rad2deg(theta2_2d))
xlabel('\theta_1 (deg)')
ylabel('\theta_2 (deg)')
% Robot animation
axes(handles.Plot7)
plot(xyz_2d(1,:),xyz_2d(2,:));
Robot.plot([theta1_2d, theta2_2d],'movie','Results\Problem 2\Animation 2-D.mp4');
