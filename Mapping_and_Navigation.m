function Mapping_and_Navigation()
clear;
clc;    
close all;  
workspace;  
format long g;
format compact;
global figureHandle
figureHandle = figure;
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
set(gcf, 'Toolbar', 'none', 'Menu', 'none');
imshow('Title.png');
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
set(gcf, 'Toolbar', 'none', 'Menu', 'none');



button = menu('Select an option?', 'Calibration', 'Map','Navigate', 'Exit');


while button ~= 5
	if button > 1
		% Let the user choose the task, once they have calibrated.
		button = menu('Select an Option', 'Calibrate','Map','Navigate', 'Exit');
	end
	switch button
		case 1
			success = Calibrate();
			% Keep trying if the user didn't click properly.
			while ~success
				success = Calibrate();
			end
			% If the user get to here, he clicked properly
			% Change to something else so it will ask him
			% for the task on the next time through the loop.
			button = 99;
            
        case 2
			Map(); 
        case 3
            Navigate();
        case 4
            close all;
            
		otherwise
			break;
	end
end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function success = Calibrate()
global lastDrawnHandle;
global calibration;
global originalImage; 


clear;
fontSize = 10;
rpi = raspi('192.168.1.102','pi','raspberry');
mycam = webcam(rpi);

originalImage = snapshot(mycam);

[~, ~, ~] = size(originalImage);

subplot(1,2, 1);
imshow(originalImage, []);
axis on;
title('Original Image', 'FontSize', fontSize);
set(gcf,'name','Image Calibration','numbertitle','off')


message = sprintf('Caliberation of the camera to the real world values.');
reply = questdlg(message, 'Calibrate Now', 'OK', 'Cancel', 'OK');
if strcmpi(reply, 'Cancel')
	% User said Cancel, so exit.
	return;
end
try
	success = false;
	instructions = sprintf('Line marking');
    title(instructions);
	uiwait(msgbox(instructions,'Help','help'));
	
	[~, ~, rgbValues, xi,yi] = improfile(1000);
	rgbValues = squeeze(rgbValues);
	distanceInPixels = sqrt( (xi(2)-xi(1)).^2 + (yi(2)-yi(1)).^2);
	if length(xi) < 2
		return;
	end
	
	hold on;
	lastDrawnHandle = plot(xi, yi, 'y-', 'LineWidth', 2);
	
	
	userPrompt = {'Enter real world units (e.g. meters):','Enter distance in those units:'};
    dialogTitle = 'Specify calibration information';
    width = 1;
    height = 60;
	
	numberOfLines = [width height] ;
	def = {'m', '1'};
    
	answer = inputdlg(userPrompt, dialogTitle, numberOfLines, def);
	if isempty(answer)
		return;
    end
    
    
	calibration.units = answer{1};
	calibration.distanceInPixels = distanceInPixels;
	calibration.distanceInUnits = str2double(answer{2});
	calibration.distancePerPixel = calibration.distanceInUnits / distanceInPixels;
    
    if calibration.distanceInUnits>1
        setDIP(calibration.distanceInUnits*calibration.distanceInPixels);
    else
        setDIP(calibration.distanceInPixels/calibration.distanceInUnits);
        
    end
    
    %setDIP(calibration.distanceInPixels);
    
	success = true;
	
	message = sprintf('The distance you drew is %.2f pixels = %f %s.\nThe number of %s per pixel is %f.\nThe number of pixels per %s is %f',...
		distanceInPixels, calibration.distanceInUnits, calibration.units, ...
		calibration.units, calibration.distancePerPixel, ...
		calibration.units, 1/calibration.distancePerPixel);
	uiwait(msgbox(message,'Pixel to Real World','modal'));
catch ME
	errorMessage = sprintf('Error in function Calibrate().\nDid you first left click and then right click?\n\nError Message:\n%s', ME.message);
	fprintf(1, '%s\n', errorMessage);
	warning(errorMessage); %#ok<SPWRN>
end

return;	
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function Map()

clc;
clear;
close all;
tic;


X = 0;
Y = 0;

t =0;
rpi = raspi('192.168.1.102','pi','raspberry');
mycam = webcam(rpi);


subplot(1,2,2);
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
%set(gcf, 'Toolbar', 'none', 'Menu', 'none');
h = animatedline(0,0,'Marker','o','MarkerEdgeColor','m');
axis([-5,5,-5,5]);
title('2D Map', 'FontSize', 24);
xlabel('Direction X', 'FontSize', 13);
ylabel('Direction Y', 'FontSize', 13);


frames1 = snapshot(mycam);
pause(0.2);

for k = 1:200
  
  
  frames2 = snapshot(mycam);
  
  subplot(1,2,1);
  imshow(frames2);
  title('Real Feed', 'FontSize', 24);
  img3=rgb2gray(frames1);
  img4=rgb2gray(frames2);

  [rows, columns] = size(img3);
  middleCol = floor(columns/2);
  img5 = img3(:,1:middleCol);
  img6 = img3(: , middleCol+1:end);

  [rows1, columns1] = size(img4);
  middleCol1 = floor(columns1/2);
  img7 = img4(:,1:middleCol1);
  img8 = img4(: , middleCol1+1:end);

  bf = fftn(double(img5));
  af = fftn(double(img7));

  df = fftn(double(img6));
  cf = fftn(double(img8));
  
  [Out1, Grad1] = dftreg(bf,af,1);
  [Out2, Grad2] = dftreg(df,cf,1);
  
  deltaX1 = Out1(4);
  deltaX2 = Out2(4);
  deltaY1 = Out1(3);
  deltaY2 = Out2(3);
  
  
  if abs(Out1(1))> abs(Out2(1))
      Xn = deltaX2;
      Yn = deltaY2;
      
  else 
      Xn = deltaX1;
      Yn = deltaY1;
      
  end
  

  t0 = atan((deltaY1-deltaY2)/(160+deltaX2-deltaX1));
 
  KC = [cos(t0) -sin(t0) X;sin(t0) cos(t0) Y;0 0 1]*[Xn;Yn;1];
  
  r = getDIP;
  
  
    
  XX = KC(1)/r;
  YY = KC(2)/r;
  X = KC(1);
  Y = KC(2);
    
  addpoints(h,XX,-YY);
  drawnow
    
    t=t+t0;
    frames1 = frames2;
    
end
toc;
display(t);
display(r);

return 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Navigate()
clc;
clear;
tic;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X = 0;
Y = 0;

t =0;


%%%%%%%%%%%%%%%%%%% Occupancy Map %%%%%%%%%%%%%%%%%%%%%%%%
map = binaryOccupancyMap(30, 30, 1);
occ = zeros(30, 30);
occ(1,:) = 1;
occ(:,end) = 1;
occ(10:30,1:20) = 1;
occ(15:20,22:25) = 1;
occ(12:14,21:25) = 1;

setOccupancy(map, occ)

subplot (1,2,1);
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
set(gcf, 'Toolbar', 'none', 'Menu', 'none');
show(map)
title('Road Plan')




%%%%%%%%%%%%%%%%%%% Location Setting %%%%%%%%%%%%%%%%%%%%%

%StartLocation =  [28 2 pi/2];   
%EndLocation = [40 990 -pi/2];
prompt = {'Enter X Position:','Enter Y Position:','Enter Pose Angle:'};
dlgtitle = 'Input Start Location';
dims = [1 50];
definput = {'0','0','pi/2'};
answer1 = inputdlg(prompt,dlgtitle,dims,definput);

prompt = {'Enter X Position:','Enter Y Position:','Enter Pose Angle:'};
dlgtitle = 'Input End Location';
dims = [1 50];
definput = {'0','0','pi'};
answer2 = inputdlg(prompt,dlgtitle,dims,definput);

Start1 = cellfun(@str2num,answer1);
StartLocation =  [Start1(1) Start1(2) Start1(3)];

End1 = cellfun(@str2num,answer2);
EndLocation =  [End1(1) End1(2) End1(3)];



%%%%%%%%%%%%%%%%%%% Device Connection %%%%%%%%%%%%%%%%%%%%
rpi = raspi('192.168.1.102','pi','raspberry');
mycam = webcam(rpi);



%%%%%%%%%%%%%%%%%%% Image Processing %%%%%%%%%%%%%%%%%%%%%

frames1 = snapshot(mycam);
pause(0.2);
toc;


for k = 1:50
    
  frames2 = snapshot(mycam);
  img3=rgb2gray(frames1);
  img4=rgb2gray(frames2);

  [rows, columns] = size(img3);
  middleCol = floor(columns/2);
  img5 = img3(:,1:middleCol);
  img6 = img3(: , middleCol+1:end);

  [rows1, columns1] = size(img4);
  middleCol1 = floor(columns1/2);
  img7 = img4(:,1:middleCol1);
  img8 = img4(: , middleCol1+1:end);

  bf = fftn(double(img5));
  af = fftn(double(img7));

  df = fftn(double(img6));
  cf = fftn(double(img8));
  
  [Out1, Grad1] = dftreg(bf,af,1);
  [Out2, Grad2] = dftreg(df,cf,1);
  
  deltaX1 = Out1(4);
  deltaX2 = Out2(4);
  deltaY1 = Out1(3);
  deltaY2 = Out2(3);
  
  
  if abs(Out1(1))> abs(Out2(1))
      Xn = deltaX2;
      Yn = deltaY2;
      
  else 
      Xn = deltaX1;
      Yn = deltaY1;
      
  end
  

  t0 = atan((deltaY1-deltaY2)/(160+deltaX2-deltaX1));

  KC = [cos(t0) -sin(t0) X;sin(t0) cos(t0) Y;0 0 1]*[Xn;Yn;1];
  
  
    
  r = getDIP;
  
  XX = KC(1)/r;
  YY = KC(2)/r;
  X = KC(1);
  Y = KC(2);  
  
    
      
%%%%%%%%%%%%%%%%%%%%% Navigation Map %%%%%%%%%%%%%%%%%%%%%

if rem(k,5) == 0
   
    if (StartLocation(1)~= EndLocation(1) && StartLocation(2)~= EndLocation(2))
        % Create map that will be updated with sensor readings
    estMap = occupancyMap(occupancyMatrix(map));

    % Create a map that will inflate the estimate for planning
    inflateMap = occupancyMap(estMap);

    vMap = validatorOccupancyMap;
    vMap.Map = inflateMap;
    vMap.ValidationDistance = .1;
    planner = plannerHybridAStar(vMap, 'MinTurningRadius', 4);

    route = plan(planner, StartLocation, EndLocation);
    route = route.States;

    % Get poses from the route.
    rsConn = reedsSheppConnection('MinTurningRadius', planner.MinTurningRadius);
    startPoses = route(1:end-1,:);
    endPoses = route(2:end,:);

    rsPathSegs = connect(rsConn, startPoses, endPoses);
    poses = [];
    for i = 1:numel(rsPathSegs)
        lengths = 0:0.1:rsPathSegs{i}.Length;
        [pose, ~] = interpolate(rsPathSegs{i}, lengths);
        poses = [poses; pose]; %#ok<AGROW>
    end

        subplot (1,2,2);
    
        show(planner)
        drawnow;
        title('Initial Route to Package')


        StartLocation = [StartLocation(1)+XX StartLocation(2)-YY StartLocation(3)];
    
    
    
    else 
    
        %subplot (1,2,2);

        %occ(8,16) = 1;
        %setOccupancy(map, occ)
        %show(map)

        %title('Road Plan')
        %uiwait(msgbox('Target Reached the Destination','Success','modal'));
    
        
    end
end



t=t+t0;
frames1 = frames2;
toc;
end


toc;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function setDIP(val)
global x
x = val;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function r = getDIP
global x
r = x;
end