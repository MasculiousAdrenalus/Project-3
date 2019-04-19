%Author: <Bryce Gossling>, Z3424655

%Program: Solution for AAS, S1.2018, Project2
function brycegossling_3C()
% close all;
global ABCD;
ABCD.flagPause=0;
% In case the caller does not specify the input argument, we propose a
% default one, assumed to be in the same folder where we run this example from.
% if ~exist('file','var'), file ='DataForProject02/Laser__2C.mat'; load(file);end;
% if ~exist('file','var'), file ='DataForProject02/IMU_dataC.mat'; load(file); end;
% if ~exist('file','var'), file ='DataForProject02/Speed_dataC.mat'; load(file); end;
load('DataForProject02/IMU_dataC.mat','IMU');
load('DataForProject02/Speed_dataC.mat','Vel');
load('DataForProject02/Laser__2C.mat', 'dataL');

%imu setup init

accel_imu = IMU.DATAf(1:3,:)';
omega_imu = IMU.DATAf(4:6,:)';
omega_imu(:,3) = -omega_imu(:,3); %since we take yaw to be negative
time_imu = double(IMU.times(:)-IMU.times(1))/10000;

yaw_rate = -IMU.DATAf(6,:);% - mean( IMU.DATAf(6,1:5000))) );
N_imu = length(time_imu);
bias = -mean(IMU.DATAf(6,1:1000));

yaw = zeros(N_imu,1);
X = zeros(N_imu,1);
Y = zeros(N_imu,1);
yaw(1) = pi/2;


%laser data
time_laser =  double(dataL.times(:)-dataL.times(1))/10000;
N_laser = dataL.N;
laser.angles = dataL.angles;
%

r.N = 0;
r.Centers = [];
r.Sizes   = [];
OOI =[];

OOI.global.id = [];
OOI.global.N = [];

OOI.local.id = [];
OOI.local.N = [];

OOI.MeasuredRanges = [];
OOI.MeasuredBearings = [];

%speed data setup
speed = Vel.speeds;


%KALMAN VARS
stdDevGyro = deg2rad(1.4) ;%radians        
stdDevSpeed = 0.4;%m/s 
std_rangeMeasurement = 0.16;%meters
std_angleMeasurement = deg2rad(1.1);%radians

P = zeros(3,3) ;

Q = diag( [ (0.1)^2 ,(0.1)^2 , (2*pi/180)^2]) ;
Q_u = diag([stdDevGyro stdDevSpeed]);

R = diag([std_rangeMeasurement*std_rangeMeasurement*4,...
      std_angleMeasurement*std_angleMeasurement*4]);

Xe_History = zeros(3,length(time_imu));
  
%%
% --------------------------------------
% Create graphical object for refreshing data during program execution.
figure(1) ; clf();
MyGUIHandles.plot1 = plot(0,0,'b.',...
                          0,0,'r.',...
                          0,0,'go',...
                          0,0,'m+');      % to be used for showing the laser points
hold on;
                   % focuses the plot on this region (of interest, close to the robot)
axis([-10,10,0,20]);                        % focuses the plot on this region (of interest, close to the robot)
xlabel('X (meters)');
ylabel('Y (meters)');

MyGUIHandles.plot1_title = title('');   
% create an empty title..
zoom on ;  grid on;


fprintf('\nThere are [ %d ] laser scans in this dataset (file [%s])\n',dataL.N,'Laser__2C.mat');
uicontrol('Style','pushbutton','String','Pause/Cont.','Position',[10,1,80,20],'Callback',{@MyCallBackA,1});
uicontrol('Style','pushbutton','String','END Now','Position',[90,1,80,20],'Callback',{@MyCallBackA,2});
hold off;

figure(2); clf();
grid on; hold on;
MyGUIHandles.plot2 = plot(0,0,'m.',...
                          0,0,'ro',...
                          0,0,'b+',...
                          0,0,'g+');
axis([-8,8,-8,8]);
MyGUIHandles.plot2_title = title(''); 
fprintf('\nThere are [ %d ] laser scans in this dataset (file [%s])\n',dataL.N,'Laser__2C.mat');
MyGUIHandles.labels = text(zeros(1,5),zeros(1,5),'','Color','k'); 
hold off;
% figure(3); clf();
% grid on; hold on;
% MyGUIHandles.plot3 = plot(0,0,'b.',...
%                           0,0,'r.',...
%                           0,0,'go',...
%                           0,0,'m+');  
% hold off;
% figure(4); clf();
% grid on; hold on;
% MyGUIHandles.plot4 = plot(0,0,'b.',...
%                           0,0,'r.',...
%                           0,0,'go',...
%                           0,0,'m+'); 
% hold off;
% figure(5); clf();
% grid on; hold on;
% MyGUIHandles.plot5 = plot(0,0,'b.',...
%                           0,0,'r.',...
%                           0,0,'go',...
%                           0,0,'m+'); 
% hold off;
%%
% Gets first graphs
for k = 2:N_imu
    dt = time_imu(k) - time_imu(k-1);
    yaw(k) = yaw(k-1) + dt*(yaw_rate(k-1)-bias);
    X(k) = X(k-1) + dt*speed(k-1) * cos(yaw(k-1));
    Y(k) = Y(k-1) + dt*speed(k-1) * sin(yaw(k-1));
end
%set(MyGUIHandles.plot3(1),'xdata',time_imu,'ydata', -(yaw_rate-bias));
%set(MyGUIHandles.plot4(1),'xdata',time_imu,'ydata', rad2deg(yaw));
%set(MyGUIHandles.plot5(1),'xdata',X,'ydata', Y);

%scan first frame
scan_i = dataL.Scans(:,1);
scan = ExtractScan(scan_i);



r = ExtractOOI(scan.ranges, scan.intensity, MyGUIHandles);
OOI.global = ExtractOOIHR(r);
OOI.global.N = length(OOI.global.x);
%fills ids
OOI.global.id = zeros(OOI.global.N,1);
for i=1:OOI.global.N
    OOI.global.id(i)=i;
end

alpha = yaw(1) - (pi/2);
OOI.local = Transform(alpha, X(1), Y(1), OOI.global.x, OOI.global.y);
set(MyGUIHandles.plot2(3),'xdata',OOI.global.x,'ydata', OOI.global.y);
set(MyGUIHandles.plot2(2),'xdata',OOI.local.x,'ydata', OOI.local.y);



skip=3;

%-------------resets variables
% yaw = zeros(N_imu,1);
% X = zeros(N_imu,1);
% Y = zeros(N_imu,1);
% yaw(1) = pi/2;
j=1;
for i = 2:N_imu-2             % in this example I skip some of the laser scans.
    if (ABCD.flagPause), pause(0.2) ; continue ; end;
    if i>=N_imu, break ;  end;
    
%     dt = time_imu(i) - time_imu(i-1);
%     yaw(i) = yaw(i-1) + dt*(yaw_rate(i-1)-bias);
%     X(i) = X(i-1) + dt*Speed(i-1) * cos(yaw(i-1));
%     Y(i) = Y(i-1) + dt*Speed(i-1) * sin(yaw(i-1));
    dt = time_imu(i) - time_imu(i-1);
    
    if ( (time_laser(j) < time_imu(i)) && (j<N_laser))
        scan_i = dataL.Scans(:,j);
        scan = ExtractScan(scan_i);
        r = ExtractOOI(scan.ranges, scan.intensity, MyGUIHandles);   % some function to use the data...
        PlotScan(scan.ranges, scan.intensity, r, MyGUIHandles,j,time_laser(j));

        OOI.local = ExtractOOIHR(r);
        alpha = yaw(i) - (pi/2);
        OOI.local = Transform(alpha, X(i), Y(i), OOI.local.x, OOI.local.y);
        set(MyGUIHandles.plot2(3),'xdata',OOI.local.x,'ydata', OOI.local.y);
        OOI = AssociateIOO(OOI, MyGUIHandles, j, time_laser(j));
        %OOI.kalman = 
        

        j= j+1;
    end
    set(MyGUIHandles.plot2(1),'xdata',X(1:i),'ydata', Y(1:i));
    
    J = [ [1,0,-dt*speed(i)*sin(yaw(i))]; [0,1,dt*speed(i)*cos(yaw(i))]; [ 0,0,1 ] ] ;
    A = dt*[[speed(i)*cos(yaw(i)) 0]; [speed(i)*sin(yaw(i)) 0]; [0 1];];
    P = J*P*J'+ (Q + A*Q_u*A');
    
    Xe = [X(i) Y(i) yaw(i)];
    Xe = RunProcessModel(Xe,speed(i),double(omega_imu(i,3)),dt) ;
    for k = 1:OOI.local.N
        landmark_x = OOI.global.x(OOI.local.id(k));
        landmark_y = OOI.global.y(OOI.local.id(k));

	    eDX = (landmark_x-Xe(1)) ;      % (xu-x)
	    eDY = (landmark_y-Xe(2)) ;      % (yu-y)
	    eDD = sqrt( eDX*eDX + eDY*eDY ) ; 
	
	    % New 2D measurement matrix:
	    H = [[  -eDX/eDD , -eDY/eDD , 0 ]; 
		 [eDY/(eDX^2 + eDY^2), -eDX/(eDX^2 + eDY^2), -1]];

	    ExpectedRange = eDD ;  
	    ExpectedBearing = wrapToPi(atan2(eDY,eDX) - Xe(3) + pi/2);

	    z = [OOI.MeasuredRanges(k) - ExpectedRange; OOI.MeasuredBearings(k) - ExpectedBearing]; 


%	    disp(MeasuredBearings)

	    % ------ covariance of the noise/uncetainty in the measurements
% 	    R = diag([sdev_rangeMeasurement*sdev_rangeMeasurement*4,...
% 		      sdev_angleMeasurement*sdev_angleMeasurement*4]);

	    S = R + H*P*H' ;
	    iS=inv(S);
	    K = P*H'*iS ;           % Kalman gain

	    % ----- finally, we do it...We obtain  X(k+1|k+1) and P(k+1|k+1)
	    Xe = Xe+K*z            % update the  expected value
	    P = P-P*H'*iS*H*P ;     % update the Covariance
    end
    
    %Xe_History(:,i) = Xe;
    
    set(MyGUIHandles.plot2(3),'xdata',OOI.local.x,'ydata', OOI.local.y);
    pause(0.0001);
    %i=i+skip;
end
    fprintf('\nDONE!\n');

return;
end
%%
function Xnext=RunProcessModel(Xe,speed,GyroZ,dt) 
    Xnext = Xe + [ dt*speed*cos(Xe(3)) + Xe(1);
                   dt*speed*sin(Xe(3)) + Xe(2); 
                   GyroZ + Xe(3); ] ;
return ;
end
%%
function OOI = AssociateIOO(OOI, H, t,q)
    %populates ids 
    OOI.local.N = length(OOI.local.x);
    OOI.local.id = zeros(OOI.local.N,1);

    %finds min dist
    for i = 1:OOI.local.N
       dist_x = OOI.global.x - OOI.local.x(i);
       dist_y = OOI.global.y - OOI.local.y(i);
       dist = sqrt(dist_x.^2 + dist_y.^2);
       [dist_min,id] = min(dist)
       OOI.local.id(i) = id;
       %OOI.dist = [OOI.dist dist];
        assignin('base','dist', dist);
        if dist_min < 1.1
            OOI.local.id(i) = id;
            set(H.labels(i),...
            'position',[OOI.local.x(i)-1,OOI.local.y(i)-0.5],...
            'String',['Best match: #',num2str(id),', Error: ', num2str(dist_min), ' m']);
        
            OOI.MeasuredRanges = [OOI.MeasuredRanges, sqrt(OOI.local.x(i)^2 + ...
                                    OOI.local.y(i)^2)];
            OOI.MeasuredBearings = [OOI.MeasuredBearings, atan2(OOI.local.x(i),...
                                    OOI.local.y(i))];
            %OOI.ObservedLandmarks = [ObservedLandmarks, local_OOI_list.global_ID(j)];
            %num_measurements = num_measurements + 1;
        else
            set(H.labels(i),...
            'position',[0,0],...
            'String','');
        end
    end
    for i = OOI.local.N+1:length(H.labels)
        set(H.labels(i),...
        'position',[0,0],...
        'String','');
    end
    s= sprintf('Laser scan # [%d] at time [%.3f] secs',t,q);
    set(H.plot2_title,'string',s);

    return;
end
%%
function result = Transform(alpha, Xr, Yr, X, Y)
    temp = [cos(alpha) -sin(alpha); sin(alpha) cos(alpha)]*...
        [X; Y];
    temp = temp + [Xr; Yr];
    result.x = temp(1,:);
    result.y = temp(2,:);
end
%%
function PlotScan(ranges, intensity, r, mh,i,t)
    angles = [0:360]'*0.5* pi/180 ;         % associated angle, for each individual range in a scan
    X = -cos(angles).*ranges;
    Y = sin(angles).*ranges+0.46;
%     OOI.x=[];
%     OOI.y=[];
    set(mh.plot1(1),'xdata',X,'ydata',Y);
    OOI = ExtractOOIHR(r);
    set(mh.plot1(3),'xdata',OOI.x,'ydata',OOI.y);
    ii = find(intensity~=0);
    Xi = X(ii);
    Yi = Y(ii);
    
    set(mh.plot1(2),'xdata',Xi,'ydata',Yi);
    s= sprintf('Laser scan # [%d] at time [%.3f] secs',i,t);
    set(mh.plot1_title,'string',s); 
end
%%
function OOI = ExtractOOIHR(r)
    OOI=[];
    ii = find(r.Color~=0);
    OOI.x = r.Centers(1,ii);
    OOI.y = r.Centers(2,ii);
    return;
end
%%
% ---------------------------------------
% Callback function. I defined it, and associated it to certain GUI button,
function MyCallBackA(~,~,x)
global ABCD;

if (x==1)
    ABCD.flagPause = ~ABCD.flagPause; %Switch ON->OFF->ON -> and so on.
    return;
end;
if (x==2)
    
    disp('you pressed "END NOW"');
    %uiwait(msgbox('Ooops, you still need to implement this one!','?','modal'));
    %set(x.start, 'UserData', true);
    % students complete this.
    close;
    %set(gcbo,'userdata',1);
    return;
end;
return;
end
