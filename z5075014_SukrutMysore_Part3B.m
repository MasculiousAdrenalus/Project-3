%% MTRN4010 Project 3B - Sukrut Mysore

%Loading file data into the program
close all; clear all;
MyFile = '.\DataForProject02\IMU_dataC.mat';

load(MyFile);
times = double(IMU.times);
times = (times-times(1));

k = 180/pi;

%currentYaw = 0;
currentYaw_unbias = 90;

turn_point = find(times>200000,1);
wz = double(IMU.DATAf(6,:))*k;
wz = wz;

bias = mean(wz(1:turn_point));

wz_unbias = wz;
%newYaw = zeros(1,length(times));
newYaw_unbias = zeros(1,length(times));

% for i = 1:(length(times))
% newYaw(i) = currentYaw - (times(2)/10000)*wz(i);
% currentYaw = newYaw(i);
% end

for j = 1:(length(times))
newYaw_unbias(j) = currentYaw_unbias - (times(2)/10000)*wz_unbias(j);
currentYaw_unbias = newYaw_unbias(j);
end

figure(1);
hold on;
subplot(2,1,1)
plot (times/10000, wz,'g')
title('Yaw rate(degrees/sec)')
ylabel('deg/s')
xlabel('time')

subplot(2,1,2)
grid on;
% plot (times/10000, newYaw,'b')
plot (times/10000,newYaw_unbias,'r')
title('Estimated Yaw(degrees)')
ylabel('deg')
xlabel('time')
%% Part 2b

MyFile2 = '.\DataForProject02\speed_dataC.mat';

load(MyFile2);

velocity = double(Vel.speeds);

currentX_pos = 0;
currentY_pos = 0;
x_pos = zeros(1,length(times));
y_pos = zeros(1,length(times));

for k = 1:length(times)
    x_pos(k) = currentX_pos + velocity(k)*cosd(newYaw_unbias(k))*(times(2)/10000);
    currentX_pos = x_pos(k);
    
    y_pos(k) = currentY_pos + velocity(k)*sind(newYaw_unbias(k))*(times(2)/10000);
    currentY_pos = y_pos(k);
end

figure(2);
hold on;
plot(x_pos,y_pos,'.b');
plot(x_pos(1),y_pos(1),'+r');
plot(x_pos(end),y_pos(end),'+g');
axis([-4,2,-1,5]);
hold off;

%% Part 2c - Feature Detection

MyFile3 = '.\DataForProject02\Laser__2C.mat';
load(MyFile3);

figure(3);
hold on
myHandles1.h1 = plot(0,0,'.b');
myHandles1.h2 = plot(0,0,'.r');
myHandles1.h3 = plot(0,0,'+g');
axis([-10,10,0,20]);
hold off

figure(4);
hold on;
myHandles2.h1 = plot(0,0,'*b');
myHandles2.h2 = plot(0,0,'+g');
myHandles2.h3 = quiver(0,0,0,0,'r');
myHandles2.h4 = plot(0,0,'b');
myHandles2.h5 = plot(0,0,'k');
for i = 1:5
myHandles2.h6(i) = text(0,0,'');
end
axis([-6,4,-1,7]);
hold off;

figure(5);
hold on;
myHandles3.h1 = plot(0,0,'*b');
myHandles3.h2 = plot(0,0,'+g');
myHandles3.h3 = quiver(0,0,0,0,'r');
myHandles3.h4 = plot(0,0,'b');
myHandles3.h5a = plot(0,0,'.k');
myHandles3.h5b = plot(0,0,'.r');
for i = 1:5
myHandles3.h6(i) = text(0,0,'');
end
axis([-6,4,-1,7]);
hold off;

numFrame = dataL.N;

IMU_times = times;
Laser_times = double(dataL.times);
Laser_times = Laser_times-Laser_times(1);

counter1 = 1;
counter2 = 1;

% VehiclePosA = [x_pos(counter1)+cosd(newYaw_unbias(counter1)-90)*0.1,y_pos(counter1)+sind(newYaw_unbias(counter1)-90)*0.1];
% VehiclePosB = [x_pos(counter1)-cosd(newYaw_unbias(counter1)-90)*0.1,y_pos(counter1)-sind(newYaw_unbias(counter1)-90)*0.1];
% VehiclePosC = [x_pos(counter1)+cosd(newYaw_unbias(counter1)-90)*hypot(0.1,0.46),y_pos(counter1)+sind(newYaw_unbias(counter1)-90)*hypot(0.1,0.46)];
% VehiclePosD = [x_pos(counter1)-cosd(newYaw_unbias(counter1)-90)*hypot(0.1,0.46),y_pos(counter1)-sind(newYaw_unbias(counter1)-90)*hypot(0.1,0.46)];

vehicle_pos = [x_pos(counter1),y_pos(counter1)];
scan_i = dataL.Scans(:,counter2);
[OOIs_initial,new_pos_initial] = ProcessScan(scan_i,vehicle_pos,(newYaw_unbias(counter1)-90),myHandles1,myHandles2);
pause(0.01);
POI = find(OOIs_initial.Colors > 0);
set(myHandles2.h1,'xdata',OOIs_initial.Centers(POI,1),'ydata',OOIs_initial.Centers(POI,2));
POI_initial = OOIs_initial.Centers(POI,:);

% for incrementer = 1:IMU.N
% if IMU_times(counter1)>=Laser_times(counter2)
%     vehicle_pos = [x_pos(counter1),y_pos(counter1)];
%     scan_i = dataL.Scans(:,counter2);
%     [OOIs,new_pos] = ProcessScan(scan_i,vehicle_pos,(newYaw_unbias(counter1)-90),myHandles1,myHandles2);
%     pause(0.01);
%     
%     point = [];
%     if isempty(new_pos) == 0
%     if length(new_pos(1,:)) > 1
%         point=zeros(1,length(new_pos(1,:)));
%         for num2 = 1:length(new_pos(1,:))
%             for num1 = 1:length(POI)
%             proximity = sqrt((new_pos(1,num2)-OOIs_initial.Centers(POI(num1),1))^2 + (new_pos(2,num2)-OOIs_initial.Centers(POI(num1),2))^2);
%                 if proximity <0.4
%                 point(num2)=num1;
%                 end
%             end
%         end
%     end
%     end
%     counter2 = counter2+1;
%     for num3=1:length(point)
%         set(myHandles2.h6(num3),'position',[new_pos(1,num3),new_pos(2,num3),0],'string',num2str(point(num3)));
%     end
% else
%     rotation = [cosd(newYaw_unbias(counter1)-90) -1*sind(newYaw_unbias(counter1)-90); sind(newYaw_unbias(counter1)-90) cosd(newYaw_unbias(counter1)-90)];
%     local_posX = [x_pos(counter1)+0.1*cosd(newYaw_unbias(counter1)-90),x_pos(counter1)-0.1*cosd(newYaw_unbias(counter1)-90),x_pos(counter1)+(hypot(0.46,0.1)*cosd(newYaw_unbias(counter1)+atand(0.1/0.46))),x_pos(counter1)+(hypot(0.46,0.1)*cosd(newYaw_unbias(counter1)-90+atand(0.46/0.1))),x_pos(counter1)+0.1*cosd(newYaw_unbias(counter1)-90)];
%     local_posY = [y_pos(counter1)+0.1*sind(newYaw_unbias(counter1)-90),y_pos(counter1)-0.1*sind(newYaw_unbias(counter1)-90),y_pos(counter1)+(hypot(0.46,0.1)*sind(newYaw_unbias(counter1)+atand(0.1/0.46))),y_pos(counter1)+(hypot(0.46,0.1)*sind(newYaw_unbias(counter1)-90+atand(0.46/0.1))),y_pos(counter1)+0.1*sind(newYaw_unbias(counter1)-90)];
%     position = [local_posX;local_posY];
%     vehicle_pos = rotation*position;
%     vector_pos = rotation*[x_pos(counter1);y_pos(counter1)];
%     set(myHandles2.h4,'xdata',local_posX,'ydata',local_posY);
%     set(myHandles2.h3,'xdata',mean(local_posX(3:4)),'ydata',mean(local_posY(3:4)),'udata',cosd(newYaw_unbias(counter1)),'vdata',sind(newYaw_unbias(counter1)));
%     set(myHandles2.h5,'xdata',x_pos(1:counter1),'ydata',y_pos(1:counter1));
%     counter1 = counter1+1;
% end
% end

%% Setting up and working on EKF stuff

stdDevGyro = 1.4*pi/180 ;        
stdDevSpeed = 0.4 ; 
sdev_rangeMeasurement = 0.16;
sdev_bearingMeasurement = 1.1*pi/180; 

Xe = [0; 0; pi/2; 0]; 
Xdr = [0; 0; pi/2; -bias*(pi/180)];
P = zeros(4,4);
P(4,4) = 4*pi/180;

Xe_History= zeros(4,length(times)) ;
Xdr_History= zeros(4,length(times)) ;

Q = diag( [ (0.005)^2 ,(0.005)^2 , (0.005*pi/180)^2, (0.005*pi/180*(1/600))^2]) ;
Pu = diag([stdDevSpeed^2,stdDevGyro^2]);
initial_state = [0 0 pi/2 0]; %Initial x, y, theta, bias

state = zeros(length(times),4);

state(1,:) = initial_state;

for i = 1:length(times)-1
	dt = (times(i+1) - times(i))/10000;
    state(i+1,:) = RunProcessModel(velocity(i),wz_unbias(i)*(pi/180),dt,state(i,:));
end

%use OOIs_initial.Centers(POI,:)

%Make a function called EKF_transformation
[OOIs_ekfInitial]=ProcessScan_ekf(dataL.Scans(:,1),myHandles1);
globalOOIs_initial = EKF_transformation(OOIs_ekfInitial,Xe);

set(myHandles3.h1,'xdata',globalOOIs_initial(1,:),'ydata',globalOOIs_initial(2,:));
for num3=1:5
    set(myHandles3.h6(num3),'position',[globalOOIs_initial(1,num3),globalOOIs_initial(2,num3),0],'string',num2str(num3));
end
% use counter1 and counter2 like you did above
k = 1;
for counter1 = 2:length(IMU_times)-1
    dt = (IMU_times(counter1) - IMU_times(counter1-1))/10000;
    gyro = -wz_unbias(counter1)*(pi/180);
    speed = velocity(counter1);
    MeasuredRanges = [];
    MeasuredBearings = [];
    ObservedLandmarks = [];
    point = [];
    num_measurements = 0;
    
    J = [[1,0,-dt*speed*sin(Xe(3)),0];[0,1,dt*speed*cos(Xe(3)),0];[0,0,1,-dt];[0,0,0,1]]; 
    Ju = [dt*cos(Xe(3)), 0; dt*sin(Xe(3)), 0; 0, dt; 0, 0];
    
    P = J*P*J'+(Ju*Pu*Ju')+Q ;
    Xe = RunProcessModel(speed,gyro,dt,Xe)';
    Xdr = RunProcessModel(speed,gyro,dt,Xdr)';
    
    if (IMU_times(counter1) > Laser_times(counter2) && counter2 < length(Laser_times))
        
        scan_i = dataL.Scans(:,counter2);
        [OOIs_ekf] = ProcessScan_ekf(scan_i,myHandles1);
        %plot the points here
        
         point = [];
         set(myHandles3.h2,'xdata',0,'ydata',0);
        if isempty(OOIs_ekf) == 0
            global_pos = EKF_transformation(OOIs_ekf,Xe);
            if length(global_pos(1,:)) > 1
            point=zeros(1,length(global_pos(1,:)));
                for num2 = 1:length(global_pos(1,:))
                    for num1 = 1:length(globalOOIs_initial(1,:))
                        proximity = sqrt((global_pos(1,num2)-globalOOIs_initial(1,num1))^2 + (global_pos(2,num2)-globalOOIs_initial(2,num1))^2);
                        if proximity <0.4
                            point(num2)=num1;
                            MeasuredRanges = sqrt((global_pos(1,num2)-(Xe(1)))^2+(global_pos(2,num2)-Xe(2))^2);
                            MeasuredBearings = atan2((global_pos(2,num2)-Xe(2)),(global_pos(1,num2)-Xe(1)))-Xe(3)+pi/2;
                            ObservedLandmarks(:,num2) = global_pos(:,num2);
                            num_measurements = num_measurements+1;
                            set(myHandles3.h2,'xdata',ObservedLandmarks(1,:),'ydata',ObservedLandmarks(2,:));
                            
                            landmark_x = globalOOIs_initial(1,num1);
                            landmark_y = globalOOIs_initial(2,num1);

                            eDX = (landmark_x-Xe(1)) ;      % (xu-x)
                            eDY = (landmark_y-Xe(2)) ;      % (yu-y)
                            eDD = sqrt( eDX*eDX + eDY*eDY ) ; 
	
                % New 2D measurement matrix:
                            H = [[  -eDX/eDD , -eDY/eDD , 0, 0 ]; 
                            [eDY/(eDD^2), -eDX/(eDD^2), -1, 0]];

                            ExpectedRange = eDD ;  
                            ExpectedBearing = (atan2(eDY,eDX) - Xe(3) + pi/2);
                            %disp(point(j))
                            z = [MeasuredRanges-ExpectedRange ; wrapToPi(MeasuredBearings-ExpectedBearing)]; 
        %disp(z)
	    % ------ covariance of the noise/uncetainty in the measurements
                            R = diag([sdev_rangeMeasurement*sdev_rangeMeasurement,sdev_bearingMeasurement*sdev_bearingMeasurement]);

                            S = R + H*P*H' ;
                            iS=inv(S);
                            K = P*H'*iS;           % Kalman gain

	    % ----- finally, we do it...We obtain  X(k+1|k+1) and P(k+1|k+1)
                            Xe = Xe+K*z;            % update the  expected value
                            P = P-K*H*P ;     % update the Covariance
                            z_History(:,k) = z;
                            k = k+1;
        %disp(max(z_History(:)));
                            disp(iS);
                        end
                    end
                end
            end
            for j = 1:length(point)
                
            end
       
        end
    counter2 = counter2+1; 
    local_posX = [  Xe(1)+0.1*cos(Xe(3)-pi/2),...
                    Xe(1)-0.1*cos(Xe(3)-pi/2),...
                    Xe(1)+(hypot(0.46,0.1)*cos(Xe(3)+atan(0.1/0.46))),...
                     Xe(1)+(hypot(0.46,0.1)*cos(Xe(3)-(pi/2)+atan(0.46/0.1))),...
                    Xe(1)+0.1*cos(Xe(3)-pi/2)];
              
    local_posY = [  Xe(2)+0.1*sin(Xe(3)-(pi/2)),...
                    Xe(2)-0.1*sin(Xe(3)-(pi/2)),...
                    Xe(2)+(hypot(0.46,0.1)*sin(Xe(3)+atan(0.1/0.46))),...
                    Xe(2)+(hypot(0.46,0.1)*sin(Xe(3)-(pi/2)+atan(0.46/0.1))),...
                    Xe(2)+0.1*sin(Xe(3)-pi/2)];
    
    position = [local_posX;local_posY];
    rotation = [cos(Xe(3)-pi/2) -1*sin(Xe(3)-pi/2); sin(Xe(3)-pi/2) cos(Xe(3)-pi/2)];
    vehicle_pos = rotation*position;
    vector_pos = rotation*[Xe(1);Xe(2)];
    set(myHandles3.h4,'xdata',local_posX,'ydata',local_posY);
    set(myHandles3.h3,'xdata',mean(local_posX(3:4)),'ydata',mean(local_posY(3:4)),'udata',cos(Xe(3)),'vdata',sin(Xe(3)));
    %plots ekf
    set(myHandles3.h5a,'xdata',Xe_History(1,1:counter1-1),'ydata',Xe_History(2,1:counter1-1));
    
    set(myHandles3.h5b,'xdata',Xdr_History(1,1:counter1-1),'ydata',Xdr_History(2,1:counter1-1));
    Xe_History(:,counter1) = Xe;
    Xdr_History(:,counter1) = Xdr;
    end

    pause(0.001);
end
figure
    hold on
    plot(Xe_History(1,:),Xe_History(2,:));
    plot(Xdr_History(1,:),Xdr_History(2,:));
    hold off
%% Functions used

function [OOIs,updated_pos] = ProcessScan(scan,pos,angle,mh1,mh2)

% Extract range and intensity information, from raw measurements.
% Each "pixel" is represented by its range and intensity of reflection.
% It is a 16 bits number whose bits 0-12 define the distance (i.e. the range)
% in cm (a number 0<=r<2^13), and bits 13-15 indicate the intensity 
%( a number 0<=i<8 ).

% We extract range and intensity, here.
%useful masks, for dealing with bits.
mask1FFF = uint16(2^13-1);
maskE000 = bitshift(uint16(7),13)  ;

intensities = bitand(scan,maskE000);

ranges    = single(bitand(scan,mask1FFF))*0.01; 
% Ranges expressed in meters, and unsing floating point format (e.g. single).

% 2D points, expressed in Cartesian. From the sensor's perpective.
angles = [0:360]'*0.5* pi/180 ;         % associated angle, for each individual range in a scan
X = cos(angles).*ranges;
Y = sin(angles).*ranges;    
X = -X;
Y = Y+0.46;
% Plot. "BRUTE FORCE" plot (see note 1).
set (mh1.h1,'xdata',X,'ydata',Y);                     % all points
ii = find(intensities~=0);          % find those "pixels" that had intense reflection (>0) (aka: Highly Reflective pixels, HR)
set(mh1.h2,'xdata',X(ii),'ydata',Y(ii));             % plot highly reflective ones
                                    % focuses plot on this region ( of interest in L220)

    idx = zeros(1,length(X));
    idx(1) = 1;
    distance = zeros(1,length(X));
    x = 1;
    for n = 2:length(idx)
        distance(n) = hypot((X(n)-X(n-1)),(Y(n)-Y(n-1)));
        if distance(n) > 0.29
            x=x+1;
        end
        idx(n) = x;
    end

% To be done (by you)
OOIs = ExtractOOIs(idx,intensities,X,Y) ;

f = find(OOIs.Colors > 0);
updated_pos=[];
if f > 0
     rotation = [cosd(angle) -1*sind(angle); sind(angle) cosd(angle)];
     new_pos = rotation*OOIs.Centers(f,:)';
    set(mh1.h3,'xdata',OOIs.Centers(f,1),'ydata',OOIs.Centers(f,2));
    set(mh2.h2,'xdata',new_pos(1,:)+pos(1),'ydata',new_pos(2,:)+pos(2));
    updated_pos=[new_pos(1,:)+pos(1);new_pos(2,:)+pos(2)];
end

return;
end

function [OOIs] = ProcessScan_ekf(scan,mh1)

% Extract range and intensity information, from raw measurements.
% Each "pixel" is represented by its range and intensity of reflection.
% It is a 16 bits number whose bits 0-12 define the distance (i.e. the range)
% in cm (a number 0<=r<2^13), and bits 13-15 indicate the intensity 
%( a number 0<=i<8 ).

% We extract range and intensity, here.
%useful masks, for dealing with bits.
mask1FFF = uint16(2^13-1);
maskE000 = bitshift(uint16(7),13)  ;

intensities = bitand(scan,maskE000);

ranges    = single(bitand(scan,mask1FFF))*0.01; 
% Ranges expressed in meters, and unsing floating point format (e.g. single).

% 2D points, expressed in Cartesian. From the sensor's perpective.
angles = [0:360]'*0.5* pi/180 ;         % associated angle, for each individual range in a scan
X = cos(angles).*ranges;
Y = sin(angles).*ranges;    
X = -X;
Y = Y+0.46;
% Plot. "BRUTE FORCE" plot (see note 1).
set (mh1.h1,'xdata',X,'ydata',Y);                     % all points
ii = find(intensities~=0);          % find those "pixels" that had intense reflection (>0) (aka: Highly Reflective pixels, HR)
set(mh1.h2,'xdata',X(ii),'ydata',Y(ii));             % plot highly reflective ones
                                    % focuses plot on this region ( of interest in L220)

    idx = zeros(1,length(X));
    idx(1) = 1;
    distance = zeros(1,length(X));
    x = 1;
    for n = 2:length(idx)
        dx = X(n) - X(n-1);
        dy = Y(n) - Y(n-1);
        distance(n) = sqrt(dx^2 + dy^2);
        if distance(n) > 0.10
            x=x+1;
        end
        idx(n) = x;
    end

% To be done (by you)
OOIs = ExtractOOIs(idx,intensities,X,Y) ;

f = find(OOIs.Colors > 0);
OOIs = OOIs.Centers(f,:);
% updated_pos=[];
 if f > 0
%      rotation = [cosd(angle) -1*sind(angle); sind(angle) cosd(angle)];
%      new_pos = rotation*OOIs.Centers(f,:)';
     set(mh1.h3,'xdata',OOIs(:,1),'ydata',OOIs(:,2));
%     set(mh2.h2,'xdata',new_pos(1,:)+pos(1),'ydata',new_pos(2,:)+pos(2));
%     updated_pos=[new_pos(1,:)+pos(1);new_pos(2,:)+pos(2)];
 end

return;
end

function r = ExtractOOIs(idx,intensities,X,Y)
    r.N = 0;
    r.Centers = [];
    r.Sizes   = [];
    r.Colors = [];
    % your part....
%     angles = [0:360]'*0.5* pi/180 ;     % associated angle, for each individual range in a scan
%     for j=1:length(ranges)
%         X(j) = cos(angles(j)).*ranges(j);
%         Y(j) = sin(angles(j)).*ranges(j);
%     end
    %[idx,C]=kmeans([X';Y']',60);
        
counter = 0;
            
for i=1:max(idx)
    point = find(idx==i);
    if length(point) > 3
        circle = CircleFitByPratt([X(point)';Y(point)']');
        if circle(3) > 0.05 && circle(3) < 0.2
            counter = counter+1;
            r.N = counter;
            r.Centers(counter,:) = [circle(1);circle(2)]';
            r.Sizes(counter) = [circle(3)*2];
            r.Colors(counter) = 0;
            colour_counter=0;
            for m=1:length(point)
                if intensities(point(m)) > 0
                    colour_counter=colour_counter+1;
                end
                if colour_counter >= 2
                    r.Colors(counter) = 1;
                end
            end
         end
    end
end
return;
end

function new_state = RunProcessModel(speed, omega, dt, old_state)
        new_state = zeros(1,4);
		new_state(1) = speed*cos(old_state(3))*dt + old_state(1); 
		new_state(2) = speed*sin(old_state(3))*dt + old_state(2); 
		new_state(3) = (omega-old_state(4))*dt + old_state(3); 
        new_state(4) = old_state(4);
end

function [global_pos] = EKF_transformation(local_OOI, state)
       
       rotation = [cos(state(3)-pi/2) -1*sin(state(3)-pi/2); sin(state(3)-pi/2) cos(state(3)-pi/2)];     
       local_pos = [local_OOI(:,1)'; local_OOI(:,2)'];
       global_pos = rotation*local_pos;
       global_pos = global_pos + [state(1);state(2)];
end