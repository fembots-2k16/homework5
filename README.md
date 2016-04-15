# homework 5 (search and rescue apriltags)

-----------------------------------------------------

# 1. download dependencies

    ### NOTE: http://people.csail.mit.edu/kaess/apriltags/ if you want to test separate from ros (a good idea to see if webcam is working)

    $ sudo apt-get install ros-indigo-usb-cam ros-indigo-image-pipeline
    
    $ sudo apt-get install ros-indigo-apriltags ros-indigo-apriltags-ros
    
# 2. download homework 5 launch files and controllers

    $ cd ~

    $ git clone https://github.com/fembots-2k16/homework5
   
# 3. Set up usb_camera configuration
## 3.1  (IF JUST USING THE WEBCAM and satisfied with configuration) (skip step 3.2)

    $ mkdir /home/<username>/.ros/camera_info

    $ cd /home/<username>/.ros/camera_info/
    
    $ ln -s ~/homework5/head_camera.yaml

## 3.2. (OPTIONAL) configure your camera?? (using the printout) (skip step 3.1)

    --------TAB 1------------------------------
    
    ### NOTE: make sure you change param name="video_device" to the right value
    
    ### value="/dev/video1" typically if you're using a usb_camera and you have
    
    ### a built in laptop webcam that you don't want to use
    
    $ roslaunch ~/homework5/usb_cam-test.launch
    
    --------TAB 2------------------------------
    
    $ cd ~/homework5
    
    $ ./cameracalibrator.py --size 8x6 --square 0.0246 image:=/usb_cam/image_raw camera:=/usb_cam
    
    ## move the checkerboard around as specified and commit here: http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
    
    $ cd /home/<username>/.ros/camera_info/
    
    $ mv head_camera.yaml ~/homework5/
    
    $ ln -s ~/homework5/head_camera.yaml
  
# 4. Set up soft links (make sure you have https://github.com/fembots-2k16/piberry-launch set up)
    
    $ cd ~/homework5
    
    $ ln -s ~/piberry-launch/pioneer.launch
    
    $ ln -s ~/piberry-launch/navigation.launch
    
    $ ln -s ~/piberry-launch/usb_cam-test.launch
    
    $ ln -s ~/piberry-launch/apriltags.launch
    
    $ ln -s ~/piberry-launch/master.launch
    
# 5. (optional) test to see that usb_cam/apriltags are working in ros?
    ### note, if previous steps worked, it should work. but camera config files may not work if you're using a different webcame or something
    ### i just pulled up https://april.eecs.umich.edu/wiki/images/9/94/Tagsampler.png on my phone

    --------TAB 1------------------------------
    
    $ roslaunch ~/homework5/usb_cam-test.launch
    
    --------TAB 2------------------------------
    
    $ roslaunch ~/homework5/apriltags.launch
    
    --------TAB 3------------------------------
    
    $ rostopic echo /tag_detections_pose
    
    ### note, should see poses that are non zero!!! (if poses are zero, need to redo step 3 somehow

# 6. Running the pioneer and navigation and usb_cam and apriltags!!!

    $ roslaunch ~/homework5/master.launch
