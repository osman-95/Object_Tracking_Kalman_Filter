%Note
%Change the location address in line 6,8,198,214 and 221 accordingly
clear all
close all
%video file location (Change the location address accordingly)
file_loc='C:/Users/osman/Desktop/tracking_video.mp4';
%create a folder to save the extracted images (Change the location address accordingly)
mkdir C:/Users/osman/Documents/MATLAB/kalman_work/image_file
%a function to convert the video to frames
num = vid2frame(file_loc);
 
%number of frames to work on
Img_count = 65;
%initialization of matrices
Measured_val  = zeros(2, Img_count);
computed_val1 = zeros(2, Img_count);
computed_val2 = zeros(2, Img_count);
computed_val3 = zeros(2, Img_count);
 
%implementing Kalman algorithm frame by frame
for k = 1:Img_count
   %extraction of measured values from the video
  [x_measured,   y_measured] = vid_analysis(k);
  %kalman logic with first configuration
  [x_computed_1, y_computed_1] = kalman_logic(x_measured, y_measured);
  %kalman logic with second configuration
  [x_computed_2, y_computed_2] = kalman_logic_2(x_measured, y_measured);
   %kalman logic with second configuration
  [x_computed_3, y_computed_3] = kalman_logic_3(x_measured, y_measured);
  %error calculation
  %error of x-coordinate in first configuration
   x_error=abs(x_measured-x_computed_1);
   %error of y-coordinate in first configuration
   y_error=abs(y_measured-y_computed_1);
   %error of x-coordinate in second configuration
   x_error_2=abs(x_measured-x_computed_2);
   %error of y-coordinate in second configuration
   y_error_2=abs(y_measured-y_computed_2);
    %error of x-coordinate in third configuration
   x_error_3=abs(x_measured-x_computed_3);
   %error of y-coordinate in third configuration
   y_error_3=abs(y_measured-y_computed_3);
   
  %plot of the tracking operation in each frame 
  hold on
  plot(x_measured,  y_measured,  'o','MarkerSize',70,'MarkerEdgeColor','red')
  plot(x_computed_1, y_computed_1, 'o','MarkerSize',70,'MarkerEdgeColor','blue')
  title( 'tracking of kalman filter demonstration ');
  legend('measured values','predicted kalman vlaues');
  xlabel('x-vlaue') 
  ylabel('y-value') 
  pause(1)
  
  %storing the x and y values in each frame
  Measured_val(:, k)  = [x_measured   y_measured]';
  computed_val1(:, k) = [x_computed_1, y_computed_1];
  computed_val2(:, k) = [x_computed_2, y_computed_2];
  computed_val3(:, k) = [x_computed_3, y_computed_3];
  %storing the error values in each frame
  error(:,k)  = [x_error y_error];
  error_2(:,k)  = [x_error_2 y_error_2];
  error_3(:,k)  = [x_error_3 y_error_3];
end
%summation of all values of x and y coordinates for each case
M_t=sum(Measured_val,'All');
C1=sum(computed_val1,'All');
C2=sum(computed_val2,'All');
C3=sum(computed_val3,'All');
%computation of percentage errors for each configuration
percent_error1=((M_t-C1)/C1)*100;
percent_error2=((M_t-C2)/C2)*100;
percent_error3=((M_t-C3)/C3)*100;
%display of the percentage errors for each congfiguration
fprintf('Error in first configuration = %f\n',percent_error1);
fprintf('Error in second configuration = %f\n',percent_error2);
fprintf('Error in third configuration = %f\n',percent_error3);
 
%plotting of cumulative tracking operation and error function for each
%configuration ///////////////////////////////////////////////////////
 
%plotting of cumulative tracking operation and error function for first configuration
%cumulative tracking operation plot for first configuration
figure(2)
hold on
plot(Measured_val(1,:),  Measured_val(2,:),  '*')
plot(computed_val1(1,:), computed_val1(2,:), 'o')
title( 'tracking of kalman filter for first configuration ');
legend('measured values','predicted kalman vlaues');
xlabel('x-vlaue') 
ylabel('y-value') 
hold off
k = 1:Img_count;
 
%plotting the tracking operation for the 1st configuration with repect to time t
figure (3)
plot(k,Measured_val(1,:),'--m' ,k, Measured_val(2,:),  '--r')
hold on
plot(k,computed_val1(1,:),'--k' ,k,computed_val1(2,:), '--g')
title( 'kalman filter tracking for 1st configuration with respect to time ');
legend('predicted kalman vlaues x', 'predicted kalman vlaues y','measured values x','measured values y');
xlabel('time') 
ylabel('measured and computed position') 
hold off
 
%error/deviation plot for first configuration
figure(4)
plot(k,error(1,:),'-s')
hold on
plot(k,error(2,:),'--s')
title( 'error/deviation for first configuration ');
legend('error in x-coordinate','error in y-coordinate');
xlabel('time') 
ylabel('error value') 
 
%plotting of cumulative tracking operation and error function for second configuration
%cumulative tracking operation plot for second configuration
figure(5)
hold on
plot(Measured_val(1,:),  Measured_val(2,:),  '*')
plot(computed_val2(1,:), computed_val2(2,:), 'o')
title( 'tracking of kalman filter for second configuration ');
legend('measured values','predicted kalman vlaues');
xlabel('x-vlaue') 
ylabel('y-value') 
hold off
k = 1:Img_count;
 
%plotting the tracking operation for 2nd configuration with repect to time t
figure (6)
plot(k,Measured_val(1,:),'--m' ,k, Measured_val(2,:),  '--r')
hold on
plot(k,computed_val2(1,:),'--k' ,k,computed_val2(2,:), '--g')
title( 'kalman filter tracking for 2nd configuration with respect to time ');
legend('predicted kalman vlaues x', 'predicted kalman vlaues y','measured values x','measured values y');
xlabel('time') 
ylabel('measured and computed position') 
hold off
 
%error/deviation plot for second configuration
figure(7)
plot(k,error_2(1,:),'-s')
hold on
plot(k,error_2(2,:),'--s')
title( 'error/deviation for second configuration ');
legend('error in x-coordinate','error in y-coordinate');
xlabel('time') 
ylabel('error value') 
 
%plotting of cumulative tracking operation and error function for third configuration
%cumulative tracking operation plot for third configuration
figure(8)
hold on
plot(Measured_val(1,:),  Measured_val(2,:),  '*')
plot(computed_val3(1,:), computed_val3(2,:), 'o')
title( 'tracking of kalman filter for third configuration ');
legend('measured values','predicted kalman vlaues');
xlabel('x-vlaue') 
ylabel('y-value') 
hold off
k = 1:Img_count;
 
%plotting the tracking operation for 3rd configuration with repect to time t
figure (9)
plot(k,Measured_val(1,:),'--m', k, Measured_val(2,:),  '--r')
hold on
plot(k,computed_val3(1,:),'--k', k,computed_val3(2,:), '--g')
title( 'kalman filter tracking for 3rd configuration with respect to time ');
legend('measured values x','measured values y','predicted kalman vlaues x','predicted kalman vlaues y');
xlabel('time') 
ylabel('measured and computed position') 
hold off
 
%error/deviation plot for third configuration
figure(10)
plot(k,error_3(1,:),'-s')
hold on
plot(k,error_3(2,:),'--s')
title( 'error/deviation for third configuration ');
legend('error in x-coordinate','error in y-coordinate');
xlabel('time') 
ylabel('error value') 
 
%\\\\\\\\\\\\\\\\\\\\\functions///////////////////////////
 
%converting the video into frames
function [n]=vid2frame(filename)
%vid=VideoReader('C:/Users/osman/Desktop/590-118-234.mp4');
%Read the videa
vid_save=VideoReader(filename);
%determine the number of frames
 Frame_count = vid_save.NumFrames;
 n=Frame_count;
 kk=1;
 %save each frame as an image in the designated location
 for i = 1:2:n
 frames = read(vid_save,i);
 %(Change the location address accordingly)
 imwrite(frames,['C:/Users/osman/Documents/MATLAB/kalman_work/image_file/Image' int2str(kk), '.jpg']);
 im(kk)=image(frames);
 kk=kk+1;
 end
end
 
%Extracting the position of the object on each frame
%takes the frame number as an input
%gives the x and y coordinate of the object (measured values)
function [x_center, y_center] = vid_analysis(index)
%back_img is the background image and init_loop is the loop counter
persistent back_img
persistent init_loop
%loading the background image and initializing the loop counter
if isempty(init_loop)
    %Background image is the first frame in the video(Change the location address accordingly)
  back_img = imread('C:/Users/osman/Documents/MATLAB/kalman_work/image_file/image1.jpg');
  init_loop = 1;
end
%initializing the x and y coordinates
x_center = 0;
y_center = 0;
%reading all the images in an iterative manner calling each image by its index value (Change the location address accordingly) 
All_img = imread(['C:/Users/osman/Documents/MATLAB/kalman_work/image_file/image', int2str(index+1), '.jpg']); 
imshow(All_img)
%computing the difference between the frame and background image to
%determine the position of the object
img_diff = imabsdiff(All_img, back_img);
img_diff = (img_diff(:,:,1) > 10) | (img_diff(:,:,2) > 10) | (img_diff(:,:,3) > 10);
 
img_ext     = logical(img_diff);
stats = regionprops(img_ext, 'area', 'centroid');
Area_vect = [stats.Area];
[nn, des] = max(Area_vect);
 
centroid = stats(des(1)).Centroid;
%update the position of the object   
x_center = centroid(1) + 15*randn;
y_center = centroid(2) + 15*randn;
end
 
%Kalman filer with configuration 1
%takes measured values as input
%gives updated belief computed by kalman filter as an output
function [x_computed y_computed] = kalman_logic(x_measured, y_measured)
%retaining the values of the matrices from the previous function call
persistent A_matrix H_matrix Q_matrix R_matrix
persistent x P_matrix 
persistent firstRun
%initialization of matrices required in the algorithm
if isempty(firstRun)  
  dt = 1; %time step
  A_matrix = [ 1  dt  0   0;        0  1   0   0;      0  0   1  dt;      0  0   0   1 ];
  H_matrix = [ 1  0  0  0;      0  0  1  0 ];
  %noise matrices  
  Q_matrix = 1.0*eye(4);
  R_matrix = 0.1*[ 50  0; 0 50 ];
  x = [0, 0, 0, 0]';
  P_matrix = 100*eye(4);
  
  firstRun = 1;
end
%step2
x_pred = A_matrix*x;
%step3
Pred_P = A_matrix*P_matrix*A_matrix' + Q_matrix;
%step4
K_gain = Pred_P*H_matrix'*inv(H_matrix*Pred_P*H_matrix' + R_matrix);
%step5
z = [x_measured y_measured]';
x = x_pred + K_gain*(z - H_matrix*x_pred);
%step6
P_matrix = Pred_P - K_gain*H_matrix*Pred_P;
%Step7
x_computed = x(1);
y_computed = x(3);
end
 
%Kalman filer with configuration 2
%takes measured values as input
%gives updated belief computed by kalman filter as an output
function [x_computed y_computed] = kalman_logic_2(x_measured, y_measured)
%retaining the values of the matrices from the previous function call
persistent A_matrix H_matrix Q_matrix R_matrix
persistent x P_matrix 
persistent firstRun
%initialization of matrices required in the algorithm
if isempty(firstRun)
  dt = 1; %time step 
  A_matrix = [ 1  dt  0   0;        0  1   0   0;        0  0   1  dt;      0  0   0   1 ];  
  H_matrix = [ 1  0  0  0;      0  0  1  0 ];
 %noise matrices  
  Q_matrix = 5*eye(4);
  R_matrix = [ 50  0;  0 50 ];
  x = [0, 0, 0, 0]';
  P_matrix = 100*eye(4);
  
  firstRun = 1;
end
%step2
x_pred = A_matrix*x;
%step3
Pred_P = A_matrix*P_matrix*A_matrix' + Q_matrix;
%step4
K_gain = Pred_P*H_matrix'*inv(H_matrix*Pred_P*H_matrix' + R_matrix);
%step5
z = [x_measured y_measured]';
x = x_pred + K_gain*(z - H_matrix*x_pred);
%step6
P_matrix = Pred_P - K_gain*H_matrix*Pred_P;
%step7
x_computed = x(1);
y_computed = x(3);
end
 
%Kalman filer with configuration 3
%takes measured values as input
%gives updated belief computed by kalman filter as an output
function [x_computed y_computed] = kalman_logic_3(x_measured, y_measured)
%retaining the values of the matrices from the previous function call
persistent A_matrix H_matrix Q_matrix R_matrix
persistent x P_matrix 
persistent firstRun
%initialization of matrices required in the algorithm
if isempty(firstRun)
  dt = 1;  %time step
  A_matrix = [ 1  dt  0   0;      0  1   0   0;        0  0   1  dt;      0  0   0   1 ];  
  H_matrix = [ 1  0  0  0;        0  0  1  0 ];
 %noise matrices
  Q_matrix = 1.0*eye(4);
  R_matrix = 0.01*[ 20  0;   0 20 ];
  x = [0, 0, 0, 0]';
  P_matrix = 500*eye(4);
  
  firstRun = 1;
end
%step2
x_pred = A_matrix*x;
%step3
Pred_P = A_matrix*P_matrix*A_matrix' + Q_matrix;
%step4
K_gain = Pred_P*H_matrix'*inv(H_matrix*Pred_P*H_matrix' + R_matrix);
%step5
z = [x_measured y_measured]';
x = x_pred + K_gain*(z - H_matrix*x_pred);
%step6
P_matrix = Pred_P - K_gain*H_matrix*Pred_P;
%step7
x_computed = x(1);
y_computed = x(3);
end

