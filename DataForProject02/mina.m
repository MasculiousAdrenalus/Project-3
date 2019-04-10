
function mina

    
    load('IMU_dataC.mat');
    load('Speed_dataC.mat');
    load('Laser__2C.mat');
    %First we must change the counts to seconds 
     time = double(IMU.times);
     
     time = time - time(1);
     time = time/10000;
    
     Yaw = -IMU.DATAf(6,:);
%      plot(time,Yaw);
%      grid on;
     
     %removing the bias
     bias = 0;
     D = 0.46; %change to meters
%      figure(2);
%      plot(time,Yaw);
%      grid on;

    speed = Vel.speeds;
    X = zeros(Vel.N,1);
    Y = zeros(Vel.N,1);

     angle = zeros(IMU.N,1);
     angle(1) = pi/2;
     
      scan_i = dataL.Scans(:,1);
    DataL = MyProcessingOfScan(scan_i,0,1);
    
    XLd = -DataL.OOIX;
     YLd = DataL.OOIY;
     
     %Now we want to keep the same X coordinate we just obtained but add
     %the distance d of the laser scanner from the back to the back of the
     %vehicle
     XLdd = XLd;
     YLdd = YLd + D;
     
     
    %Now rotate the coordinat system by an angle of alpha 
    alpha = angle(1) - pi/2;
    R = [cos(alpha) -sin(alpha);
         sin(alpha) cos(alpha)]   
    
    temp = R * [XLdd; YLdd];
    
    % Now we translate according to the robots position
    temp = [temp(1,:); temp(2,:)] + [X(1); Y(1)]
    assignin('base', 'temp', temp);
    Xg = temp(1,:);
    Yg = temp(2,:);
    
    figure(2); %clf();
    plot(Xg, Yg, '*g');
    axis([-10,10,-10,10]); hold on;
    
     for i=1:IMU.N-1 
         if(time(i) < 10)
             bias = mean(Yaw(1:i));
         end
        dt=time(i+1)-time(i);
        angle(i+1) = angle(i) + dt*(Yaw(i) - bias); 
        X(i+1) = X(i) + dt*speed(i)*cos(angle(i));
        Y(i+1) = Y(i) + dt*speed(i)*sin(angle(i));
        assignin('base','angle',angle);
        assignin('base','X',X);
        assignin('base','Y',Y);
       
        
    t =  double(dataL.times(i)-dataL.times(1))/10000;
    % t: time expressed in seconds, relative to the time of the first scan.
    
    scan_i = dataL.Scans(:,i);
    DataL = MyProcessingOfScan(scan_i,t,i);
     
    %First we flip the X cordinate so its in the same direction as the
    %global X coordinate but the Y coordinate remains the same 
     XLd = -DataL.OOIX;
     YLd = DataL.OOIY;
     
     %Now we want to keep the same X coordinate we just obtained but add
     %the distance d of the laser scanner from the back to the back of the
     %vehicle
     XLdd = XLd;
     YLdd = YLd + D;
     
     
    %Now rotate the coordinat system by an angle of alpha 
    alpha = angle(i) - pi/2;
    R = [cos(alpha) -sin(alpha);
         sin(alpha) cos(alpha)]   
    
    temp = R * [XLdd; YLdd];
    
    % Now we translate according to the robots position
    temp = [temp(1,:); temp(2,:)] + [X(i); Y(i)]
    Xg = temp(1,:);
    Yg = temp(2,:);

     plot(Xg, Yg,'.b');
     axis([-10,10,-10,10]);
     end
     
     plot(X,Y,'*b');
     grid on;
     figure(2);
     
     plot(time, rad2deg(angle));
     grid on;figure(3);
     plot(time, Yaw - bias);
     grid on;
     figure(4);
     plot(Xg, Yg);
     
end



function DataL = MyProcessingOfScan(scan,t,i)

       DataL.XL = [];
       DataL.YL = [];
       DataL.OOIX = [];
       DataL.OOIY = [];
       
    % I made this function, to receive the following parameters/variables:
    % 'scan' : scan measurements to be shown.
    % 't':  associated time.    
    % 'i' : scan number.
    % 'mh'  : struct contaning handles of necessary graphical objects.
    
    angles = [0:360]'*0.5* pi/180 ;              % Associated angle for each range of scan
    % same as in "dataL.angles".
    
    % scan data is provided as a array of class uint16, which encodes range
    % and intensity (that is the way the sensor provides the data, in that
    % mode of operation)
     
    MaskLow13Bits = uint16(2^13-1); % mask for extracting the range bits.
    % the lower 13 bits are for indicating the range data (as provided by this sensor)
    maskE000 = bitshift(uint16(7),13)  ;
    rangesA = bitand(scan,MaskLow13Bits) ; 
    % rangesA now contains the range data of the scan, expressed in CM, having uint16 format.
    intensities = bitand(scan,maskE000);
    % now I convert ranges to meters, and also to floating point format
    ranges    = 0.01*double(rangesA); 
    DataL.XL = cos(angles).*ranges;
    DataL.YL = sin(angles).*ranges;
    ii = find(intensities~=0);
    
    OOIs = ExtractOOIs(ranges,intensities);
    
   DataL.OOIX = OOIs.Centers(1,:);
   DataL.OOIY = OOIs.Centers(2,:);
   
  
    return;
end

function OOIs = ExtractOOIs(ranges,intensities)
OOIs.N = 0;
OOIs.Centers = [];
OOIs.Sizes   = [];
OOIs.colour = [];
% your part....

angles = [0:360]'*0.5* pi/180 ;         % associated angle, for each individual range in a scan
X = cos(angles).*ranges;
Y = sin(angles).*ranges;
A = [X Y];

Dist_btw_pts = sqrt(sum(abs(diff(A)).^2,2));


cluster_index = [0 (find(Dist_btw_pts > 0.075))'];
OOIs.N = length(cluster_index) -1;

for i = 1:OOIs.N
    temp_cluster_x = X(cluster_index(i) + 1:cluster_index(i+1));
    temp_cluster_y = Y(cluster_index(i) + 1:cluster_index(i+1));
    [OOIs.Centers(:,i), OOIs.Sizes(i)]=circle_fit(temp_cluster_x, temp_cluster_y);

    
    if any(intensities(cluster_index(i) + 1:cluster_index(i+1))) > 0  
        OOIs.Colour(i) = 1;
    else
        OOIs.Colour(i)=0;
    end 
    
    if OOIs.Sizes(i) < 0.05 || OOIs.Sizes(i) > 0.20 || OOIs.Colour(i) == 0
        OOIs.Sizes(i) = 0;
       
    end
    
end
 t =(find(OOIs.Sizes == 0));
 
 OOIs.Centers(:,t) = [];
 OOIs.Sizes(t) = [];
 OOIs.Colour(t) = [];
return;
end


function [c,d] = circle_fit(x,y)

c = [mean(x) mean(y)];
d = sqrt(range(x)^2+range(y)^2);

end