see3cam
=======

ROS driver for the e-consystems See3CAM_Stereo (Tara) camera based on the 
[uvc_camera](https://github.com/ktossell/camera_umd/tree/master/uvc_camera) package.

Creates a stereo image node pair (`left/image_raw` and `right/image_raw`) from any connected Tara devices. This is for compatibility with any other ROS modules which support stereo cameras.
Added support for setting and getting exposure and brightness.
Added support for getting IMU data.

The following nodes will be created upon launching this driver.
```
    /stereo/concat
    /stereo/image_raw
    /stereo/left/image_raw
    /stereo/left/camera_info
    /stereo/right/image_raw
    /stereo/right/camera_info
    /stereo/get_brightness
    /stereo/set_brightness
    /stereo/get_exposure
    /stereo/set_exposure
    /stereo/get_IMU
```
# zeabus vision 2018

## Table of Contents

**[Parameters](#parameters)**<br>
**[Camera calibration](#camera-calibration)**<br>
**[Description](#description)**<br>
**[CameraInfo Message](#camerainfo-message)**<br>


## Camera calibration
    
* Prepare chessboard where one side contains an even number of squares, both black and white, and the other contains an odd number of squares and define size of square that use for calibration.  

* Set up stereo camera have a same clock for capture the images that use arduino send the trigger to two cameras.
    
* You have 2 ways to calibration

    
1. The first way is collect 10 - 20 images for accurate calibration in MATLAB, After you have cameraParameters to remove lens distortion from the image. 

   
2. (Recommanded) The second way is ROS stereo camera calibration [**camera_calibration**](https://wiki.ros.org/camera_calibration/)
    
    *  Run the cameracalibrator.py node for calibrate a stereo camera:
        
        ```
        rosrun camera_calibration cameracalibrator.py --size 8x7 --approximate=0.01 --square 0.90 right:=/my_stereo/right/image_raw left:=/my_stereo/left/image_raw right_camera:=/my_stereo/right left_camera:=/my_stereo/left _approximate_sync:=True _queue_size:=1
        ```
    * After finished the calibration, click `SAVE`. The images and calibration parameter are compressed and save to  `/tmp/calibrationdata.tar.gz`.
 
    * Then extract `calibrationdata.tar.gz` that have `left.yaml` and `right.yaml`, copy them into directory `camera_info/` in your camera parameter directory (e.g. zeabus_vision/camera_info)
    
    * Convert file `.yaml` to `.ini` by
        
        ```
        rosrun camera_calibration_parsers convert *.yaml *.ini
        ```
    * Move `*.ini` into directory `camera_conf/`
    
    
3. Create `.launch` file for open stereo camera.

    * Use stereo_image_proc pacakge
    
    ```
    <node pkg="stereo_image_proc" name="stereo_image_proc" type="stereo_image_proc" output="screen" ns="stereo" args="_approximate_sync:=True _queue_size:=10"></node>
    ```


## Description

#### Focal length in millimeter

* Focal length of Kowa is 6 mm 
* Focal length have influence to *Field of view (FOV)*
* If number of Focal length is a low, number of FOV is high
* **Ps. In intrinic camera matrix use Focal length in pixel unit not mm unit.** 


#### Optical center or Principal point

* Optical center is the central of a lens. It's represented *O* in physics.

	
#### Skew coefficient

* Skew coefficient is non-zero if the image axes are not perpendiuclar *(From MATLAB)*.


#### Field of view (FOV)

* FOV *(HxV)* of Kowa is 96.8° x 79.4°
* FOV have vertical and horizontal FOV, that I use vFOV and hFOV instead of them.
	* vFOV = *2 * arctan( height of image [px] / (2 * working distance [mm])*
	* hFOV = *2 * arctan( width of image [px] / (2 * working distance [mm])*


### Intrinic camera

* Intrinic camera is a 3x3 matrix that used to map between coordinates in the image to physical world coordinates.
	
	
### CameraInfo Message

* sensor_msgs/CameraInfo.msg is the message of ROS ,contains information about *calibration parameter* that requires D, K, R, and P

* D is distortion parameters.

```
	D = [k1, k2, t1, t2, k3]
```
	* k1, k2 and k3 are radial distortion coeffients.
	* t1 and t2 are tangenial distortion coefficients.
	
	
* K is intrinsic camera metrix.

```
	    [Fx  0   Cx]
	K = [S   Fy  Cy]	
	    [0   0   1 ]
```

	* Fx, Fy are Focal length in px.
	* Cx, Cy are Optical center.
	* S is skew coefficient.

* R is rectification matrix or rotation matrix.

* P is projection matrix or camera matrix. 

## References

[**[1] ROS Camera calibration**](http://wiki.ros.org/camera_calibration)

[**[2] ROS CameraInfo**](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)

[**[3] MATLAB Camera calibration**](https://www.mathworks.com/help/vision/ug/camera-calibration.html)

[**[4] MATLAB Stereo Camera calibrator**](https://www.mathworks.com/help/vision/ug/stereo-camera-calibrator-app.html)

[**[5] how-to-photograpy-6-focal-length**](http://www.tamemo.com/post/116/how-to-photograpy-6-focal-length/)

