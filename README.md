# Object Tracking using Kalman Filter
Tracking and analysis of a moving object on a 2-Dimenional space (video) using Kalman filter Algorithm.
## 1. Introduction
Object tracking is a complex field that has witnessed a significant development in the last decades. It is a branch of computer vision implemented in vast number of practical applications such as surveillance, navigation of a robot, human-to-computer interaction, etc. Its main goal is to automate any given operation and to reduce human effort by replacing working humans with computers to process and analyze digital images or videos to gather (or use “collect”) desired information. 


The aim of this project is to locate, track and analyze the object displayed on the video, frame by frame, with the implementation of Kalman Filter Algorithm logic using MATLAB. The processes of analysis refer to the reduction of error of the evaluated results with variation of certain parameters in the algorithm such as noises to achieve better tracking of the algorithm with respect to the actual values.

### 1.1 Kalman Filter

Kalman filtering is a classic state estimation technique extensively used in various application areas such as signal processing and autonomous control of vehicles.[1] The algorithm predicts an estimate of undetermined variables with use of the measurement variables of the previous time interval of a system. The algorithm is simple and computationally inexpensive compared to other filtering algorithms with complex structures and computation. 


![Algorithm](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/algorithm.png)



Tracking an object position on video or a 2D space requires the determination of the position coordinates (x and y for our case as its 2 dimensional). So, the state can be represented as

![x](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/x.PNG)

Where x and y are the positions coordinate of the object on the 2-dimensional plane, x ̇ and y ̇ are the velocity values. The measurements are given as 

![z](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/z.PNG)

And the control input is given as

![u](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/u.PNG)

Where x ̈ and y ̈ are the acceleration values.
The deterministic part of the state model of the Kalman filter is given as 

![xt](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/xt.PNG)

the measurement depends on the state with some noise δ

![zt](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/zt.PNG)

Where

	    A ∈ R4x4

	    B ∈ R4x2
	    
	    C ∈ R2x4

the equation for the new position is given as 

![xznew](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/xznew.PNG)

So, from equation 2.6 to 2.9 we get the following state model 

![xxt](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/xxt.PNG)

and the measurement 

![zzt](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/zzt.PNG)

since in our situation we are not providing any control action to the system, the matrix B becomes zero. So, A, B and C are as follows

![ABC](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/ABC.PNG)

The system was evaluated and analyzed with multiple configurations, with constant A, B and C matrices and variation of processes error covariance Q, measurement error covariance R and initial uncertainty covariance matrix respectively. Three configurations were finally chosen to be discussed in the report and exhibited the following parameters shown below

![cap_2](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/Capture_2.PNG)

## 2. Code Structure

The Kalman filter was implemented on sample video of a single object (tomato in our case) moving on the screen using MATLAB. The code was initially divided into three parts:

-	Main code
-	Video processing function  
-	Kalman logic function

### 2.1 Main Code

The main code contains the main initialization and the video address along with the main loop incorporating all the other functions for processing the video and applying the filter to the data in addition to the code plot of all the acquired outputs from the algorithm. The graphs plot the tracking process between the computed coordinate values and the actual measured values. The main code can be expressed in a flowchart shown below

![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/Main_code.png)

### 2.2 Video processing function

This function processes the video to present the data input in an acceptable format that can be utilized in the filtering algorithm. It takes the frame index address as an input and reads the desired frame to analyze and operate the algorithm on it. The function processes each frame individually and determines the position of the object by subtracting the background image with each frame. The output of the function is an array of the measured coordinate of the desired object in each frame of the video.   

![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/vid_process.jpg)

### 2.3 Kalman Logic Function

The Kalman logic function consist of the initialization of all the matrices required for performing the Kalman filtering in addition to all the seven steps of the algorithm. This function takes the data coordinates (measurement values) as inputs obtained from the video through video processing function and calculate the coordinates using the algorithm based on the previous beliefs and measurement values. The flow-chart representing the Kalman filtering function is shown below

![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/Kalman_function.png)

## 3. Results

As discussed earlier the data/video was evaluated on multiple configuration and 3 configurations were chosen for discussion. The deviation/error was calculated by using the following formula.

![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/err.PNG)

### 3.1 Output


![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/Output/Capture1.PNG)
![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/Output/Capture2.PNG)
![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/Output/Capture3.PNG)
![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/Output/Capture4.PNG)


### 3.2 Configuration 1


![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/C1_1.png)
![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/C1_2.png)
![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/C1_3.png)

### 3.3 Configuration 2

![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/C2_1.png)
![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/C2_2.png)
![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/C2_3.png)

### 3.4 Configuration 3


![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/C3_1.png)
![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/C3_2.png)
![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/C3_3.png)




From all the configuration testing results we can conclude that the major impact on the tracking and results the Kalman filter was the tuning of the measurement error covariance R. The tracking of the filter was negatively affected with increase in measurement error covariance R resulting in significant deviation of the coordinates computed by the Kalman filter as shown in 2nd configuration case with R=diag(50,50). The accuracy of the filtering was improved significantly when R matrix was reduced as shown in 1st and 3rd configuration case with R matrix taken as R=diag(5,5) and R=(0.2,0.2) respectively. The initial uncertainty covariance matrix P matrix didn’t play any significant impact on the tracking results when varied through wide range. Finally, the process error covariance Q improved the results with increase in its value but not as significant as measurement error covariance. The complete analysis and results of all the configuration is shown in the table below



![](https://github.com/osman-95/Object_Tracking_Kalman_Filter/blob/master/ReadMe_Img/Capture_1.PNG)
