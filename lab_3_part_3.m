%Master run file for robot localization lab (MTRN4010) 
%Part 3 of Project 1 - Full Kalman Filter
%Ian Bartlett

function [] = lab_3_part_3()

time_conv_factor = 10000;

IMU_data = load('DataForProject02/IMU_dataC.mat');
IMU_times = double(IMU_data.IMU.times)/time_conv_factor;
IMU_accel = IMU_data.IMU.DATAf(1:3,:)';
IMU_omega = IMU_data.IMU.DATAf(4:6,:)';

IMU_times = IMU_times - IMU_times(1);

% 2D change of attitude representation
IMU_omega(:,3) = -1*IMU_omega(:,3);

speed_data = load('DataForProject02/Speed_dataC.mat');
velocity = speed_data.Vel.speeds;

laser_data = load('DataForProject02/Laser__2.mat');
laser_times = double(laser_data.dataL.times)/time_conv_factor;
laser_times = laser_times - laser_times(1);
laser_scans = laser_data.dataL.Scans;

% Kalman filter parameters

Xe = [ 0; 0; pi/2 ] ;        
P = zeros(3,3) ;

Xe_History= zeros(3,length(IMU_times)) ;

stdDevGyro = 2*pi/180 ;        
stdDevSpeed = 0.15 ; 
sdev_rangeMeasurement = 0.25;
sdev_angleMeasurement = 0.1; 

Q = diag( [ (0.1)^2 ,(0.1)^2 , (2*pi/180)^2]) ;
Q_u = diag([stdDevGyro stdDevSpeed]);

% Begin configuration initialization

handles = initialize_plots(IMU_times);

%Extract linear trend
start_index = find(IMU_times - IMU_times(1) < 20,1,'last');
omega_offset = mean(IMU_omega(1:start_index,3));
IMU_omega(:,3) = IMU_omega(:,3) - omega_offset;

%Integrate state to obtain an initial estimate of the robot's path. 
%This is the old Part A and Part B from Lab 2- we will use this 
%to provide a point of comparison.

initial_state = [0 0 pi/2]; %Initial x, y, theta

state = zeros(length(IMU_times),3);

state(1,:) = initial_state;

for i = 1:length(IMU_times)-1
	dt = IMU_times(i+1) - IMU_times(i);
    state(i+1,:) = int_state(IMU_omega(i,3),velocity(i),dt,state(i,:));
end

set(handles.att_trace,'xdata',IMU_times(1:i),'ydata',(180/pi)*state(1:i,3));
set(handles.xy_trace,'xdata',state(:,1),'ydata',state(:,2));

% Parts B and C - data fusion and object associtaion

% Initialize and plot starting positions of OOIs
% Process first scan
[range_0, intensity_0] = extract_laser_data(laser_scans(:,1));
plot_scan(range_0, intensity_0, state(1,:), handles);
global_OOI_list = ExtractOOIs(range_0, intensity_0);

[global_OOI_list.Centers(1,:),global_OOI_list.Centers(2,:)] = ...
            transform_position(global_OOI_list.Centers(1,:)', ...
                               global_OOI_list.Centers(2,:)', ...
                               Xe);

set(handles.object_global_centers,'xdata',global_OOI_list.Centers(1,:),...
                                  'ydata',global_OOI_list.Centers(2,:));
                              
for i = 1:global_OOI_list.N
    set(handles.object_global_labels(i),...
        'position',[global_OOI_list.Centers(1,i)+0.2,global_OOI_list.Centers(2,i)],...
        'String',['#',num2str(i)]);
end

%Initialize
local_OOI_list = global_OOI_list;                             
                              
% Iterate through data set, using IMU times as clock
current_scan = 1;

for i = 2:length(IMU_times)-1

    Dt = IMU_times(i) - IMU_times(i-1);

    %Process new IMU data
    imu_gyro = double(IMU_omega(i,3));
    encoder_speed = double(velocity(i));
    MeasuredRanges = [];
    MeasuredBearings = []; 
    ObservedLandmarks = [];
    num_measurements = 0;

    % If a new laser scan has arrived, process it.
    if (IMU_times(i) > laser_times(current_scan) && current_scan < length(laser_times))

        current_scan = current_scan + 1;
        [range_i, intensity_i] = ...
            extract_laser_data(laser_scans(:,current_scan));
        
        %disp(state(i,:))
        
        local_OOI_list = ExtractOOIs(range_i,intensity_i);
        
        [adjusted_centers_X,adjusted_centers_Y] = ...
        transform_position(local_OOI_list.Centers(1,:)', ...
                           local_OOI_list.Centers(2,:)', ...
                           Xe);

        plot_scan(range_i, intensity_i, Xe, handles);
                       
        set(handles.object_local_centers,...
        'xdata',adjusted_centers_X,...
        'ydata',adjusted_centers_Y);
        
        %Associate objects

	%Shouldn't just be inventing this attribute here
	local_OOI_list.global_ID = zeros(local_OOI_list.N, 1);

        for j = 1:local_OOI_list.N
            x_dists = global_OOI_list.Centers(1,:) - adjusted_centers_X(j);
            y_dists = global_OOI_list.Centers(2,:) - adjusted_centers_Y(j);
            euclidian_dists = sqrt(x_dists.^2 + y_dists.^2);
            [mindist,mini] = min(euclidian_dists);
            if mindist < 0.4
                set(handles.object_local_labels(j),...
                    'position',[adjusted_centers_X(j)-1,adjusted_centers_Y(j)-0.5],...
                    'String',['Best match: #',num2str(mini),', Error: ', num2str(mindist), ' m']);
	        local_OOI_list.global_ID(j) = mini;

		MeasuredRanges = [MeasuredRanges, sqrt(local_OOI_list.Centers(1,j)^2 + ...
		                                       local_OOI_list.Centers(2,j)^2)];
		MeasuredBearings = [MeasuredBearings, atan2(local_OOI_list.Centers(2,j),...
		                    local_OOI_list.Centers(1,j))];

		ObservedLandmarks = [ObservedLandmarks, local_OOI_list.global_ID(j)];
	        num_measurements = num_measurements + 1;

            else
                 set(handles.object_local_labels(j),...
                    'position',[0,0],...
                    'String','');
	        local_OOI_list.global_ID(j) = -1;
            end
            
        end
        
        for j = local_OOI_list.N+1:length(handles.object_local_labels)
              set(handles.object_local_labels(j),...
                    'position',[0,0],...
                    'String','');
        end
    
    end
        
    %Compute new P matrix and new predicted value

    J = [ [1,0,-Dt*encoder_speed*sin(Xe(3))  ]  ; [0,1,Dt*encoder_speed*cos(Xe(3))] ;    [ 0,0,1 ] ] ; 

    F_u = Dt*[[encoder_speed*cos(Xe(3)) 0]; [encoder_speed*sin(Xe(3)) 0]; [0 1];];

    % MODIFIED: added J*Q_u*J' term to include input uncertainty
    P = J*P*J'+ (Q + F_u*Q_u*F_u');
    % -----------------------------------------------
    % and the predicted expected value. 
    Xe = int_state(imu_gyro, encoder_speed, Dt, Xe)';  

    %If any landmarks have been detected and assocated, use them to compute new 
    %range/bearing measurements, and perform an EKF update. 
    for j = 1:num_measurements
	    disp(local_OOI_list.global_ID(j))
	    landmark_x = global_OOI_list.Centers(1,ObservedLandmarks(j));
	    landmark_y = global_OOI_list.Centers(2,ObservedLandmarks(j));

	    eDX = (landmark_x-Xe(1)) ;      % (xu-x)
	    eDY = (landmark_y-Xe(2)) ;      % (yu-y)
	    eDD = sqrt( eDX*eDX + eDY*eDY ) ; 
	
	    % New 2D measurement matrix:
	    H = [[  -eDX/eDD , -eDY/eDD , 0 ]; 
		 [eDY/(eDX^2 + eDY^2), -eDX/(eDX^2 + eDY^2), -1]];

	    ExpectedRange = eDD ;  
	    ExpectedBearing = wrapToPi(atan2(eDY,eDX) - Xe(3) + pi/2);

	    z = [MeasuredRanges(j) - ExpectedRange; MeasuredBearings(j) - ExpectedBearing]; 

	    disp('Measured bearings')
	    disp(MeasuredBearings)

	    % ------ covariance of the noise/uncetainty in the measurements
	    R = diag([sdev_rangeMeasurement*sdev_rangeMeasurement*4,...
		      sdev_angleMeasurement*sdev_angleMeasurement*4]);

	    S = R + H*P*H' ;
	    iS=inv(S);
	    K = P*H'*iS ;           % Kalman gain

	    % ----- finally, we do it...We obtain  X(k+1|k+1) and P(k+1|k+1)
	    Xe = Xe+K*z            % update the  expected value
	    P = P-P*H'*iS*H*P ;     % update the Covariance
    end

    Xe_History(:,i) = Xe;
    
    s = sprintf('Processed: IMU scan %d, Laser scan %d', i, current_scan);
    set(handles.object_title, 'string', s);
    
    set(handles.vehicle_trace,'xdata',state(1:i,1),'ydata',state(1:i,2));
    set(handles.kalman_trace,'xdata',Xe_History(1,1:i),'ydata',Xe_History(2,1:i));
    pause(0.0001);
end

end

function new_state = int_state(omega, speed, dt, old_state)
        new_state = zeros(1,3);
		new_state(1) = speed*cos(old_state(3))*dt + old_state(1); 
		new_state(2) = speed*sin(old_state(3))*dt + old_state(2); 
		new_state(3) = omega*dt + old_state(3); 
end

function [ranges, intensity] = extract_laser_data(scan)
         mask_low_13_bits = uint16(2^13-1);
         rangesA = bitand(scan,mask_low_13_bits);
         ranges = 0.01*double(rangesA);

         %Extract intensity data
         maskE000 = bitshift(uint16(7),13);
         intensity = bitand(scan, maskE000);
end

function plot_scan(ranges, intensity, state, handles)
        laser_offset = 0.46;
        angles = [0:360]'*0.5*pi/180;
        X = -cos(angles).*ranges;
        Y = sin(angles).*ranges + laser_offset;
        
        [X_t,Y_t] = transform_position(X,Y,state);
        
        ii_bright = find(intensity~=0);
        set(handles.object_data_points,'xdata',X_t,'ydata',Y_t);
        set(handles.object_bright_points,'xdata',X_t(ii_bright),...
            'ydata',Y_t(ii_bright));
        
end

function [transformed_X, transformed_Y] = transform_position(X, Y, state)
       
       transform_matrix = rotz(state(3)*180/pi - 90) + [[0 0 state(1)];...
                                         [0 0 state(2)];...
                                         [0 0 0]];
       homogenous_coords = [X'; Y'; ones(1,length(X))];
       transformed_coords = transform_matrix*homogenous_coords;
       transformed_X = transformed_coords(1,:);
       transformed_Y = transformed_coords(2,:);
   
end
