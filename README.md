# DPM developmtnt

## System composition
- DynaFlash
- High-speed camera Basler
- ExDCM marker-based tracking
- homography method by Yanagisawa-san
- Bunny

## Procedure

- Camera calibration

- Capture
    - Using basler um to capture 720x540 images with 200 FPS
        - mono-color camera makes the tracking more stable
    - Modified the capture thread in ExDCM code
    - Output: captured frame
    
- Tracking
    - Using marker-based tracking method: ExDCM
    - Input: captured frame, camera intrinsic matrix, camera distortion coefficients
    - Output: Target's rotation and translation vector

- Rendering
    - Rendering part in ExDCM
    - Input: Target's rotation and translation vector (to calculate Model matrix)
    - Projection matrix and View matrix is adjusted manually
    - Output: rendering result
    - Phong reflection model
        - no ambient light, 2 parellel lights from front left and front right
        - diffuse reflection constant $k_d=0.5$
        - specular reflection constant $k_s=1$
        - shininess $\alpha=2$
        - ![1](https://github.com/newblush/DPM_2022/blob/main/img/1.png)



- ProCam calibration
    - Homography calibration
    - Get homography matrix after set the position of camera and projector
    - Input: homography matrix and rendering result
    - Output: warpped 1024x768 image

- Projection    
    - Dynaflash V3
    - 940 fps
 
- Demo Making
    - projected color is too light under visible illumination
    - IR illumination
        - ![2](https://github.com/newblush/DPM_2022/blob/main/img/2.png)
    - ![3](https://github.com/newblush/DPM_2022/blob/main/img/3.png)


## Reference
- [Homography code](https://github.com/watanabelaboratory/GetHomographyMatrix)
- [ExDCM code (original code)](https://github.com/watanabelaboratory/ExDCM)

## Obj
- [Bunny](https://github.com/watanabelaboratory/developmtntDPM2022_teamA/tree/main/exp_data)

## Log
### 06/16
 - Assign tasks to team members.

### 07/27
- Implement the procam calibration by homography method.

### 07/29
- Discuss with Peng-san about the feasibility and specific implementation plan of dynamic projection using the ExDCM tracking method.

### 08/03
- Complete real-time capturing with high-speed cameras and combine this part with the tracking method.
- Test the projection part.

### 08/04
- Combine the projection part with the capture, tracking, rendering part.
- Adjust rendering details and obtained preliminary results.
- Discuss existing problems and possible solutions.

### 08/05
- Adjust the rendering and projection slightly by adjusting the fov and view matrix through trial.

### 08/09
- Record Demo.


## Demo

[Google drive](https://drive.google.com/file/d/1gEhtAhlshs5KFxiOt6rCF58KEnvOKcLG/view?usp=share_link)
