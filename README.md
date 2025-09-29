# Lane-Following-Car
Autonomous Lane Following Algorithm using MPC controller. Autonomous carmakers use principle of making mask image from front cameras to follow lanes on highways. After making Mask Image we are using Hough Lines so as to make our car follow those lines. The MPC controller from matlab helps us in generating steering angles when lanes marker takes a turn. Do remember in this project we are only concerned with pixels (meaning pixel based MPC parameters) not with real distance (as I have taken sample of a video rather not a real time lane), since MPC takes real distance value and generate relevant lateral position and steer angle. The main purpose of this project is to have an essence how autonomous lane following algorithms being made. 

<img width="1305" height="688" alt="Figure_1" src="https://github.com/user-attachments/assets/b3ff9b15-cfd1-43f6-abae-a6eb94c67254" />

Gray Scale Image

<img width="1305" height="706" alt="Figure_2" src="https://github.com/user-attachments/assets/f97a12d1-ceb3-40b8-ae1b-dd7c3b812b10" />

Canny Mask Image

<img width="1305" height="706" alt="mask_houghLines" src="https://github.com/user-attachments/assets/165d28b7-9a57-4cbb-836f-38a903e8ada8" />

Hough Lines on Mask Image

<img width="2107" height="1107" alt="HoughLines2" src="https://github.com/user-attachments/assets/e1050fb1-1047-4116-a075-245aee9b0bae" />

Hough lines on RGB Image



