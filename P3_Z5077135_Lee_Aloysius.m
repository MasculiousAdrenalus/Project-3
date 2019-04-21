% Author: Aloysius Jie Wen Lee, z5077135 
% 
% Program: Solution for AAS, T1.2019 Project3, Part 3
% 
% Use (1) for implementing the EKF based on the data and code used in Project2.Part4.
% This program is based on the data used in Project 2. It is OFF-LINE but using real data, in the same way we solved Project2. You are requested to adapt your solution for Project2.Part4, adding the EKF component. The estimates should be more accurate than those obtained in P2.4, which were based on simple dead-reckoning.
% Assume the following realistic conditions:
% Noise in angular rate measurements: standard deviation = 1.4 degrees/second.
% Noise in speed sensor: standard deviation = 0.4m/s.
% Noise in range measurements: standard deviation = 0.16m.
% Noise in bearing measurements: standard deviation = 1.1 degree
% You need also to consider that the LIDAR sensor is located at the front of the platform. There is a document, in the lecture notes, which describes how to treat that matter.
% Note: you must remove the bias which is present in the angular rate measurements (this is not required in part 4)

function P3_Z5077135_Lee_Aloysius(file)
    % In case the caller does not specify the input argument, we proDR_Pose a
    % default one, assumed to be in the same folder where we run this example from.
    if ~exist('file','var')
        % IMU data
        file1 ='DataForProject02/IMU_dataC.mat';
        load(file1); 
        % Information on Data Recieved 
        % Accelerations X,Y,Z, expressed in local frame : IMU.DATAf(1:3,:)
        % Units: Accelerations in Gravities
        %
        % Angular rates (local roll, pitch and yaw rates): IMU.DATAf(4:6,:) 
        % Angular rates in radians/secondIMUtimes units:  1 count =0.1ms; 
        %
        % "IMUtimes" expressed via class "uint32";
        % you may need to convert it to real format (e.g. "double")
        
        % Speed Encoder data
        file2 = 'DataForProject02/Speed_dataC.mat';
        load(file2);
        % Speed measurements in Vel.speeds
        % Expressed in m/sTime units:  1 count =0.1ms; 
        % "IMUtimes" expressed using class "uint32"; 
        % you may need to convert it to real format (e.g. "double")
        
        % Laser data
        file3 = 'DataForProject02/Laser__2C.mat';
        load(file3);
        % Field "dataL.N" : number of scans;
        % "dataL.Scans(:,i)": scan number i;
        % "dataL.IMUtimes(i)": IMUtimestamp of scan #i. 
        % Each scan, "dataL.Scans(:,i)", contains 361 samples,covering 180 degrees of FoV 
        % (@ 1/2 degree resolution).
        % 
        % IMUtimestamps are class uint32, where 1 unit = 0.1ms
        %
        % "dataL.angles": are the 361 angles associated to the beams in the scan."
        % 
        % Scans"  is class uint16. 
        % Scans(k,i) is the range in direction "angles(k)" of the scan #i.
        % The first 13 bits, (0-12), of the range are actually the range, in cm;
        % while the rest of the bits (bits 13,14,15), are the intensity.
    end
    
    %% --------------------------------------------------------------------
    % Time data 
    IMUtimes  = double(IMU.times)*10^-4;
    IMUtimes = IMUtimes - IMUtimes(1);          % shift the IMUtimes data to start at t = 0
    
    laser_times  = double(dataL.times)*10^-4;
    laser_times = laser_times - laser_times(1);

    % yaw data are expressed using convention A. Where positive is downward
    % However, for the navigation we will use Convention B. Where positive
    % is upward. i.e. we filp the data set
    yaw = -1*(IMU.DATAf(6,:));
    % Integrate yaw rate to get altitude
    % 1. Eliminate Bias in yaw rate data 
    % Get average of initial samples, because the machine was not moving during that period of time; 
    % we take advantage of that fact, so we estimate the bias using this OFF_LINE approach/trick.
    start = 4000; % the last indice where time is under 20 seconds. = 4000 As vehicle changed altitude after 20s
    yaw_Bias = mean(yaw(1,1:start));
    % yaw in radians/second
    yaw = yaw - yaw_Bias;   
    speed = Vel.speeds;
    
    %% --------------------------------------------------------------------
    % plot setup 
    global ABCD;
    ABCD.flagPause=0;
    ABCD.Exit=0;
    figure(3);
    clf(); hold on;
    MyGUIHandles.handle1 = plot(0,0,'b.');                       % local laser points
    % MyGUIHandles.handle3 = plot(0,0,'ko','MarkerSize',10);       % used to show ALL OOI
    MyGUIHandles.handle4 = plot(0,0,'g*','MarkerSize',10,'LineWidth',2);       % used to show the brillant OOIs 
    MyGUIHandles.handle5 = plot(0,0,'r*','MarkerSize',5);        % used to show ALL intense points  
    MyGUIHandles.handle6 = plot(0,0,'k.');                       % global laser points
    MyGUIHandles.handle7 = plot(0,0,'b');                        % vehicle position/path taken DR
    MyGUIHandles.handle8 = plot(0,0,'r');                        % vehicle position/path taken EKF
    %----------------------------------------------------------------------
    % plot setup D
    globalhandle.plotdata = plot(0,0,'b*','MarkerSize',10);      % global OOI
    globalhandle.Errortext = text(zeros(1,8),zeros(1,8),'','color','red' );
    globalhandle.NumberOOI = text(zeros(1,8),zeros(1,8),'','color','red');
    %----------------------------------------------------------------------
    axis([-10,10,-10,10]);                         % focuses the plot on this region (of interest, close to the robot)
    xlabel('Horizontal Position (meters)');
    ylabel('Vertical Position (meters)');
    MyGUIHandles.handle2 = title('');           % create an empty title..
    zoom on ;  grid on;
    disp('Showing laser scans in Cartesian.');
    fprintf('\nThere are [ %d ] laser scans in this dataset (file [%s])\n',dataL.N, file3);
    uicontrol('Style','pushbutton','String','Pause/Cont.','Position',[10,1,80,20],'Callback',{@MyCallBackA,1});
    uicontrol('Style','pushbutton','String','END Now','Position',[90,1,80,20],'Callback',{@MyCallBackA,2});
    
    %% -------------------------------------------------------------------- 

    
    
    
    
    
    
    
    
    
    
    %% ....................................................................
    % Measurement errors and Noise
    % Magnitude of the "noises",Polluting mesurements, models, etc.  
    
    % - Measurement error: in the prediction (i.e. from rate of change sensors)
    %   Standard deviation of the error in the angular rate sensor. 
    stdDevGyro = 1.4*pi/180 ;          
    %   2 degrees/second , standard deviation of the gyros' noise

    % - Standard deviation of the error in the speed's measurements
    stdDevSpeed = 0.4 ;   
    %   Speed-meter's error = 0.15m/sec; actually, a low quality speed sensor! 
    
    % - Measurement error: in the update/observations 
    %   Errors in the range measurements (25cm, standard dev.)
    sdev_rangeMeasurement = 0.16;          
    %   std. of noise in range measurements. 0.25m
    
    % - Error measuring the bearing of landmarks 0.5 degrees 
    sdev_bearingMeasurement = 1.1*pi/180; % in rads 
    % .....................................................................
 
    % EKF Variables 
    % "Xe" and "P" : EKF ESTIMATES (Expected value and covariance matrix)
    % initial quality --> perfect (covariance =zero )
    P = zeros(3,3) ;            
    
    
    % Size of IMUdata stored in Vel.N
    XeDR_History = zeros(3,Vel.N);
    Xdr = [ 0; 0;pi/2 ] ; %Initial Condition (x, y, heading), in(meters, meters, radians)
    Xe_History = zeros(3,Vel.N);

    Xe = [ 0; 0;pi/2] ; 
    
    % .....................................................................
    % process model uncertainty 
    % I assume that every time we apply the process model to predict the 
    % evolution of the system for a perdiod of time of 
    % we introduce uncertainty
    
    % - Q matrix -> Represent the covariance of the uncertainty about the 
    %   process model.
    Q = diag( [ (0.01)^2 ,(0.01)^2 , (1*pi/180)^2]) ;
    %   Q(4,4) = 0 as bias doesnt change over time

    % - Pu -> covariance of the du (error between measured the actual state 
    %   inputs)
    Pu = diag([stdDevGyro^2, stdDevSpeed^2]);
    % .....................................................................
    
%     % For Modifying the Observation Function H to account for the laser
%     % displacement 
%     tic
%     syms xa ya x y heading_ang 
%     h1 = sqrt( (xa - x -0.46*cos(heading_ang))^2 + (ya - y -0.46*sin(heading_ang))^2 );
%     h1x = diff(h1,x);
%     h1y = diff(h1,y);
%     h1head = diff(h1,heading_ang);
% 
%     h2 = atan2( ya - y -0.46*sin(heading_ang), xa - x -0.46*cos(heading_ang) ) - heading_ang + pi/2;
%     h2x = diff(h2,x);
%     h2y = diff(h2,y);
%     h2head = diff(h2, heading_ang);
% 
%     H = [h1x, h1y, h1head; h2x, h2y, h2head];
%     toc
    
    
    %----------------------------------------------------------------------
    % part c) get OOI as Global OOI reference 
    [ranges, intensities,angles] = getlaserdata(dataL.Scans(:,1));
    [GOOIs] = processScan(ranges, intensities,angles);
    GlobalOOI = GOOIs.Centers(:,find(GOOIs.IntenseClusters));
    [GlobalOOI(1,:),GlobalOOI(2,:)]  = transform_coordinate(GlobalOOI(1,:)',GlobalOOI(2,:)',Xe);
    set(globalhandle.plotdata,'xdata',GlobalOOI(1,:),'ydata',GlobalOOI(2,:));
    % add error text and numbering 
    for i = 1:length(GlobalOOI(1,:))
        set(globalhandle.NumberOOI(i),'FontSize',8,'String',['N',num2str(i)],'position',[GlobalOOI(1,i)-0.5,GlobalOOI(2,i)-0.5]);
        set(globalhandle.Errortext(i),'FontSize',10,'String',['Error: ', num2str(0), ' m.'],'position',[GlobalOOI(1,i)-0.5,GlobalOOI(2,i)-1]);
    end 
    %----------------------------------------------------------------------
    
    
    % Now, loop through the avaialable laser data scans and match it up
    % with laser scans 
    scan_num = 1;
    for i = 1:length(IMUtimes)-1 
        % ------------- run process model to do prediction ----------------
        Dt = IMUtimes(i+1) - IMUtimes(i);
        Xdr = PredictVehiclePose(Xdr, yaw(i), speed(i) ,Dt);
        
        
        % Add to history plot: overwritten if scan runs and OOi detected 
        XeDR_History(:,i+1) = Xdr;

        % ----------------- THIS IS THE ACTUAL EKF! -----------------------
        Xe = PredictVehiclePose(Xe, yaw(i), speed(i) ,Dt);
        
        
        %   EKF's prediction: 
        % - J -> linearization of the function f about the system states X=X(k|k) 
        J = [ 
            [1,0,-Dt*speed(i)*sin(Xe(3))]; 
            [0,1,Dt*speed(i)*cos(Xe(3)) ];
            [ 0,        0,              1]; 
            ];
        % - Fu-> linearization of the function f about the inputs (velocity and angular velocity) 
        Fu = [
             [Dt*cos(Xe(3)), 0;]
             [Dt*sin(Xe(3)), 0;]
             [  0         , Dt;]
             ];
        % - Qu -> Covariance of the uncertainty/noise of the process model, 
        % exclusively as consequence of the uncertainty that pollutes 
        % our knowledge about the inputs
        Qu = Fu*Pu*Fu';
        % then I calculate the new covariance, after the prediction P(K+1|K) = J*P(K|K)*J'+Q ;
        P = J*P*J' + Q + Qu;
        % Calculate the predicted expected value. 

        Xe_History(:,i+1)    = Xe ;
        % .................................................................
        % Get Laser Data (if available)
        %---------------------------
        % dont iterate until scans are similar timing 
        
        
%         if (IMUtimes(i) < 50 && IMUtimes(i) > laser_times(scan_num))
%             scan_num = scan_num + 1;
%         end 
%         if (IMUtimes(i) > laser_times(scan_num) && scan_num < length(laser_times) && IMUtimes(i) > 50 )
%             % pause(0.5)


        % skip first 20 seconds 
        if (IMUtimes(i) < 20 && IMUtimes(i) > laser_times(scan_num))
            scan_num = scan_num + 1;
        end 
        
        
        if (IMUtimes(i) > laser_times(scan_num) && scan_num < length(laser_times) && IMUtimes(i) > 20 )
            
            %---------------------------
            % GUI CONTROLS
            if (ABCD.flagPause)
                pause(0.2);
                continue; 
            end
            if (ABCD.Exit ==1)
                close all;
                return;
            end
            %---------------------------
            % process laser scan: contains an estimate of the vehicle's current position 
            current_laser_time =  laser_times(scan_num);
            % t: time expressed in seconds, relative to the time of the first scan.
            scan_i = dataL.Scans(:,scan_num);
            % get laser Data
            [ranges, intensities,angles] = getlaserdata(scan_i);
                % tic
            % PARTC 2) feature extraction 
            [OOIs,Data] = processScan(ranges, intensities,angles);   % process scan 1 to 361, but pass in IMU scan as well
                % toc
            %---------------------------
            % transform co-ordinate frame for all points (not necessary) 
            [TransformedData.Xpts,TransformedData.Ypts]  = transform_coordinate(Data.LocalXpts,Data.LocalYpts,Xe);
            % transform co-ordinate frame for all Local OOI centers 
            [OOIs.Centers(1,:),OOIs.Centers(2,:)]  = transform_coordinate(OOIs.Centers(1,:)',OOIs.Centers(2,:)',Xe);
            %---------------------------
            scan_num = scan_num + 1; % -> basically will go from 1 to 8292
            
            % ----------------------------------
            % temp for visual
            VehicleData.VehiDR_PoseX = XeDR_History(1,1:i);
            VehicleData.VehiDR_PoseY = XeDR_History(2,1:i);
            VehicleData.VehiDR_Head = XeDR_History(3,1:i);
            VehicleData.VehiEKF_PoseX = Xe_History(1,1:i);
            VehicleData.VehiEKF_PoseY = Xe_History(2,1:i);
            VehicleData.VehiEKF_Head = Xe_History(3,1:i);
            PlotOOIs(OOIs,Data,VehicleData,TransformedData, MyGUIHandles,scan_num,current_laser_time,i);
            % ----------------------------------

            %---------------------------
            k = find(OOIs.IntenseClusters);
            KeyOOIs = [OOIs.Centers(1,k);OOIs.Centers(2,k)];    % local OOI

            numEKFlandmarks = 0;
            %---------------------------
            %   EKF's Update: 
            % if measurements are avaiable ==> we perform the related updates.
            
            if   ~isempty(KeyOOIs(1,:))         % CHECK IF LARGER THAN 1 TO AID EKF PERFORMANCE
                EKFlandmarkGlobalIndex = zeros(1,length(KeyOOIs(1,:)));
                EKFlandmarkLocalIndex = zeros(1,length(KeyOOIs(1,:)));
                % ******************************* 
                % Associate OOI's 
                % Associate by comparing local with global
                % for c = 1:length(KeyOOIs(1,:))
                %     distX = GlobalOOI(1,:) - KeyOOIs(1,c);
                %     distY = GlobalOOI(2,:) - KeyOOIs(2,c);
                %     distance =  sqrt(distX.^2 + distY.^2);
                %     % Data Association: comparing the global with the local
                %     [minDistance,index] = min(distance);
                % 
                %     if (minDistance < 0.5) % dodgy way to ensure that the distance isnt used for another OOI
                %         % associate the OOI 
                %         numEKFlandmarks = numEKFlandmarks + 1;
                %         EKFlandmarkGlobalIndex(1,numEKFlandmarks) = index; 
                %         EKFlandmarkLocalIndex(1,numEKFlandmarks) = c; 
                %         set(globalhandle.Errortext(c),'FontSize',10,'String',['Error:#', num2str(c), ' ', num2str(minDistance), ' m.'],'position',[GlobalOOI(1,c)-0.5,GlobalOOI(2,c)-1]);  
                %     else
                %         set(globalhandle.Errortext(c),'FontSize',10,'String','');
                %     end
                % end %------------ GG BREAKPOINT
                
                % Associate by comparing global with locals
                for c = 1:length(GlobalOOI(1,:))
                    distX = KeyOOIs(1,:) - GlobalOOI(1,c);
                    distY = KeyOOIs(2,:) - GlobalOOI(2,c);
                    distance =  sqrt(distX.^2 + distY.^2);
                    % Data Association: comparing the global with the local
                    [minDistance,index] = min(distance);
                    
                    if (minDistance < 0.5) % dodgy way to ensure that the distance isnt used for another OOI
                        % associate the OOI 
                        numEKFlandmarks = numEKFlandmarks + 1;
                        EKFlandmarkGlobalIndex(1,numEKFlandmarks) = c; 
                        EKFlandmarkLocalIndex(1,numEKFlandmarks) = index; 
                        set(globalhandle.Errortext(c),'FontSize',10,'String',['Error:#', num2str(c), ' ', num2str(minDistance), ' m.'],'position',[GlobalOOI(1,c)-0.5,GlobalOOI(2,c)-1]);  
                    else
                        set(globalhandle.Errortext(c),'FontSize',10,'String','');
                    end
                end %------------ GG BREAKPOINT
                
                tic
                % apply prediction 
                for j = 1:numEKFlandmarks

                    c = EKFlandmarkGlobalIndex(1,j);
                    b = EKFlandmarkLocalIndex(1,j);
                    
                    % hacky method to stop some glitches
                    if not(i>17500 && i <19000) % not(numEKFlandmarks == 1 && c == 1)

                        eDX = (GlobalOOI(1,c)-Xe(1)) ;      % (xu-x)
                        eDY = (GlobalOOI(2,c)-Xe(2)) ;      % (yu-y)
                        eDD = sqrt( eDX*eDX + eDY*eDY ) ; %   so : sqrt( (xu-x)^2+(yu-y)^2 ) 

                        ExpectedRange = eDD ;   
                        ExpectedAngle = atan2(eDY,eDX) - Xe(3) + deg2rad(90);
                        % Evaluate residual (innovation)  "Y-h(Xe)" 
                        %(measured output value - expected output value)
                        % - Z-> innovation: difference between measured and expected observations 
                        eDX_M = (KeyOOIs(1,b)-Xe(1)) ;      % (xu-x)
                        eDY_M = (KeyOOIs(2,b)-Xe(2)) ;      % (yu-y)
                        eDD_M = sqrt( eDX_M*eDX_M + eDY_M*eDY_M ) ; %   so : sqrt( (xu-x)^2+(yu-y)^2 ) 
                        MeasuredRanges = eDD_M;
                        MeasuredBearings = atan2(eDY_M,eDX_M) - Xe(3) + deg2rad(90);

                        % Jacobian of h(X);
                        % the expected distances to this landmark ( "h(Xe)" )
                        HHH = [-eDX/eDD   , -eDY/eDD      , 0 ;
                               eDY/(eDD^2), -eDX/(eDD^2)  , -1];  


                        %------------ GG BREAKPOINT
                        z  = [MeasuredRanges - ExpectedRange ; wrapToPi(MeasuredBearings - ExpectedAngle) ];
                        % ------ covariance of the noise/uncetainty in the measurements
                        % - R-> uncertainty in the measurements from the observations
                        R = [4*sdev_rangeMeasurement*sdev_rangeMeasurement,0 ;
                            0                     ,4*sdev_bearingMeasurement*sdev_bearingMeasurement];

                        % - intermediate steps for the EKF 
                        % - S-> estimated covariance of Z 
                        S = R + HHH*P*HHH' ;
                        % iS=1.\S;                 % iS = inv(S) ;   % in this case S is 1x1 so inv(S) is just 1/S
                        iS = inv(S) ;
                        % - K -> gain matrix 
                        K = P*HHH'*iS;           % Kalman gain
                        %   Obtain  X(k+1|k+1) and P(k+1|k+1)
                        Xe = Xe+K*z ;       % update the  expected value
                        P = P-K*HHH*P ;       % update the Covariance % i.e. "P = P-P*H'*iS*H*P"  )
                        % individual EKF update done 
                        % .................................................................
                        % Loop to the next observation based on available measurements..
                    end
                end
                toc
            end
            fprintf("X before prediction\n")
            disp(Xdr)
            fprintf("X after prediction\n")
            disp(Xe)
            fprintf("z\n")
            disp(z)
            Xe_History(:,i+1)    = Xe ;

            %---------------------------
            % vehicle position data: old data 
%             VehicleData.VehiDR_PoseX = XeDR_History(1,1:i);
%             VehicleData.VehiDR_PoseY = XeDR_History(2,1:i);
%             VehicleData.VehiDR_Head = XeDR_History(3,1:i);
%             VehicleData.VehiEKF_PoseX = Xe_History(1,1:i);
%             VehicleData.VehiEKF_PoseY = Xe_History(2,1:i);
%             VehicleData.VehiEKF_Head = Xe_History(3,1:i);
            %---------------------------
            % plot data 
            PlotOOIs(OOIs,Data,VehicleData,TransformedData, MyGUIHandles,scan_num,current_laser_time,i);
            
            pause(0.001);                   % wait 
        end
    end
    fprintf('\nDONE!\n');    
    % Read Matlab Help for explanation of FOR loops, and function double( ) and pause()
    return;
end
%% ------------------------------------------------------------------------


%% ------------------------------------------------------------------------
% Part A and B Function 
% X0 current state ( [x; y; heading] )
% X estimated next state ( [x; y; heading] at time t+dt)
% speed : current speed (m/s)
% steering : current steering angle (at time t)(in radians)
% dt is the "integration" horizon (should be a fraction of second)
% Tricycle / Ackermann model, discrete version
function X = PredictVehiclePose(X0,steering,speed,dt)
    % X = [x; y; heading]
    X=X0;
    X(1) = speed*cos(X0(3))*dt + X0(1);
    X(2) = speed*sin(X0(3))*dt + X0(2); 
    X(3) = steering*dt + X0(3);
    return;
end
%% ------------------------------------------------------------------------




%% ========================================================================
% part C functions (1)
function [ranges, intensities,angles] = getlaserdata(scan)
    angles = [0:360]'*0.5 ;         % Associated angle for each range of scan
    % same as in "dataL.angles".
    % scan data is provided as a array of class uint16, which encodes range
    % and intensity (that is the way the sensor provides the data, in that
    % mode of operation)
    MaskLow13Bits = uint16(2^13-1); % mask for extracting the range bits.
    % the lower 13 bits are for indicating the range data (as provided by this sensor)
    %----------------------------------------------------------------------
    % INTENSITIES data for intensities 
    maskE000 = bitshift(uint16(7),13) ;
    intensities = bitand(scan,maskE000);
    %----------------------------------------------------------------------
    rangesA = bitand(scan,MaskLow13Bits) ; 
    % rangesA now contains the range data of the scan, expressed in CM, having uint16 format.
    % now I convert ranges to meters, and also to floating point format
    ranges    = 0.01*double(rangesA);
    % convert to Cartesian 
end
% ------------------------------------------------------------------------
%
%
%
%
% ------------------------------------------------------------------------
% part C functions (2)
% process scan and find OOI and local X,Y data and intensity data
function [OOIs,Data] = processScan(ranges, intensities,angles)
    X = ranges.*cos(degtorad(angles));
    Y = ranges.*sin(degtorad(angles)); 
    %----------------------------------------------------------------------
    % Find OOI
    % data structure for custers 
    distPT = zeros(1,361);              % distance between each adjacent point
    % store cluster numbering order 
    ClusterOrder = zeros(1,361); % - > (1 , 6, 15, 18, 50, ...) 
    ClusterOrder(1,1) = 1; %  account for the first cluster data pt 
    % Store cluster 
    % number of total clusters 
    numCluster = 0;
    % Seperate points into clusters 
    for i = 1:360       
        distPT(1,i) = sqrt( (X(i+1) - X(i))^2 + (Y(i+1) - Y(i))^2);
        % this will count cluster groups from point numbers 
        if distPT(1,i) > 0.2 
            numCluster = numCluster + 1;
            ClusterOrder(1,numCluster+1) = i+1;       % as numCluster is indexed from 1 to ... (1,2)
        end 
    end 
    % account for the last cluster 
    numCluster = numCluster + 1;
    ClusterOrder(1,numCluster+1) = 362;       % as numCluster is indexed from 1 to ... (1,2)
    %----------------------------------------------------------------------
    % Capture Data from Clusters 
    Nu = 0;
    ColourTemp = zeros(1,361);  % 0 or 1 only !  % assert 
    CentersTemp = zeros(2,361);
    SizesTemp = zeros(1,361);
    % Analyse distance within the cluster: determine if ooi 
    for i = 1:numCluster
        firstPtClust = ClusterOrder(1,i);
        lastPTClust = ClusterOrder(1,i+1) - 1 ;
        % Method 1: 
        if lastPTClust - firstPtClust > 2    % get rid of small data points 
            % fprintf("Cluster num: %i, Order %i:%i\n",i, firstPtClust,lastPTClust)
            % Find distance 
            distFirstLast = sqrt( (X(lastPTClust) - X(firstPtClust) )^2 + (Y(lastPTClust) - Y(firstPtClust))^2);
            if (distFirstLast <= 0.2) && (distFirstLast >= 0.05)
            % find center via the first and last points center 
                % Find centers 
                % X intersect
                CentersTemp(1,Nu+1) = (X(lastPTClust) + X(firstPtClust))/2.0;
                % Y intersect
                CentersTemp(2,Nu+1) = (Y(lastPTClust) + Y(firstPtClust))/2.0;
                % Find diameter 
                % From first and last data points to determine this 
                SizesTemp(1,Nu+1) = distFirstLast;
                Nu = Nu + 1;
                % Find intense Coloured objects 
                for c = firstPtClust:lastPTClust
                    % fprintf("point %d ",c)
                    % fprintf(" %.2f \n",intensities(c))
                    if intensities(c) > 0
                         ColourTemp(1,Nu) = 1;
                         % fprintf("intense point found",c)
                         % pause()
                         break;
                    end
                    % intensities(firstPtClust:lastPTClust)
                end 
            end 
        end    
    end 
    % fprintf("Total OOi %d\n",Nu )
    % fprintf("\n")
    %----------------------------------------------------------------------
    % find only intense points and return it 
    Data.LocalXpts = X;
    Data.LocalYpts = Y;
    Data.allIntensePts = find(intensities~=0);
    %----------------------------------------------------------------------
    % return struct and declaration
    OOIs.NumOfOOI = Nu;
    OOIs.Centers = CentersTemp(:,1:Nu);         % returns a non transformed co-ordinate 
    OOIs.Sizes = SizesTemp(1,1:Nu);             % just returns object size 
    OOIs.IntenseClusters = ColourTemp(1,1:Nu);  % just a flag set if its intense
    %----------------------------------------------------------------------
    return;
end
% ------------------------------------------------------------------------
%
%
%
%
% ------------------------------------------------------------------------
% part C functions (3): from Lab whiteboard
function [transformed_X, transformed_Y] = transform_coordinate(X, Y, state)
    XL = -X;
    YL = Y + 0.46;
    angle = (state(3) - pi/2);
    R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
    transform = R*[XL'; YL'];
    transformed_X = transform(1,:) + state(1);
    transformed_Y = transform(2,:) + state(2);
end
% ------------------------------------------------------------------------
%
%
%
%
% ------------------------------------------------------------------------
% part C functions (4) PLOT
function PlotOOIs(OOIs,Data,VehicleData,TransformedData, mh,i,t, imu_num)
    if OOIs.NumOfOOI<1 
        return ; 
    end
    %----------------------------------------------------------------------
    % asserts 
    [cx, cy] = size(OOIs.Centers);
    assert(OOIs.NumOfOOI == cy, "Incorrect number of centers (2xN)");
    assert(cx == 2, "Centers should have two points (2xN)");
    assert(OOIs.NumOfOOI == length(OOIs.Sizes), "Incorrect number of sizes");
    assert(OOIs.NumOfOOI == length(OOIs.IntenseClusters), "Incorrect number of colors");
    assert(OOIs.NumOfOOI == length(find(OOIs.Sizes >= 0.05 & OOIs.Sizes <= 0.2)), "OOIs are not all >= 0.05, <= 0.2");
    assert(OOIs.NumOfOOI == length(find(OOIs.IntenseClusters == 1 | OOIs.IntenseClusters == 0)), "Intensity should be either 0 or 1");
    %----------------------------------------------------------------------
    % plot 
    % set(mh.handle1,'xdata',Data.Xpts,'ydata',Data.Ypts); % shows all laser points 
    set(mh.handle6,'xdata',TransformedData.Xpts,'ydata',TransformedData.Ypts);  % shows all transformed laser points 
    set(mh.handle7,'xdata',VehicleData.VehiDR_PoseX,'ydata',VehicleData.VehiDR_PoseY); % shows vehicle path taken (dead reckoning path) 
    set(mh.handle8,'xdata',VehicleData.VehiEKF_PoseX,'ydata',VehicleData.VehiEKF_PoseY); % shows vehicle path taken (dead reckoning path) 
    
    % and some text... as a variable title 
    s= sprintf('Laser scan # [%d] at time [%.3f] secs IMU data # [%d]',i,t,imu_num );
    set(mh.handle2,'string',s); 
    % set(mh.handle3,'xdata',OOIs.Centers(1,:),'ydata',OOIs.Centers(2,:));    % plot transformed  OOI
    k = find(OOIs.IntenseClusters);
    set(mh.handle4,'xdata',OOIs.Centers(1,k),'ydata',OOIs.Centers(2,k));    % plot transformed local Coloured OOI
    set(mh.handle5,'xdata',TransformedData.Xpts(Data.allIntensePts),'ydata',TransformedData.Ypts(Data.allIntensePts)); % plot all transformed intense points 
    return;
end
% -------------------------------------------------------------------------
%
%
%
%
% ------------------------------------------------------------------------
% part C functions (5)
% Callback function. I defined it, and associated it to certain GUI button,
function MyCallBackA(~,~,x)   
    global ABCD;
        
    if (x==1)
       ABCD.flagPause = ~ABCD.flagPause; %Switch ON->OFF->ON -> and so on.
       return;
    end;
    if (x==2)
        ABCD.Exit = ~ABCD.Exit;
        disp('you pressed "END NOW"');
        return;
    end;
    return;    
end
%% ------------------------------------------------------------------------