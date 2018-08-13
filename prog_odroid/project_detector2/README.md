# project_detector

Execute project_detector to detect square and send subtented angles to gumstix via Serial Communication.

UPDATE :
- Add time-to-collision computation for gap width estimation

## To run :
1) Edit GIT_REPO in makefile
2) make / make debug
3) Plug RX/TX CON10 (pins 6/8) to gumstix
4) Run ./project_detector
5) Run Gumstix model

## Communication :
### INPUT from UART
- uint16_t t_polling : Time between 2 frames
- uint16_t MaxNumBuffer : Number of frames max per buffer
- uint16_t width/height : Width/Height for camera
- uint16_t offset_x/y : offset_x/y for camera center
- uint16_t binning : Binning on/off
- uint16_t expoAutoMax_us : Maximum exposure time for expoAuto (in us)
- uint16_t grabTimeOut_ms : TimeOut to grab a frame (in ms)
- float FOV_x/y : Field Of View on x/y (in rad)
- uint16_t max_no_detection : Maximum of false detections before reinitialization of the detector
- uint16_t blur_size : Matrix size for blurring (pre-processing of the image)
- uint16_t bin_square : Threshold to digitize the image (in pixel 8bits-intensity)
- float k_approx : Coefficient to approximate lines (percent/100 of contour length)
- uint16_t thresh_area : Minimal size of a detected area (in px^2)
- float thresh_cos : Threshold on cos between 2 segments to define a right-angle
- uint16_t thresh_diff : Distance maximum between 2 square centers
- float thresh_ratio : Maximal difference between 2 square ratios
- float thresh_Et : Threshold on brightness time derivative for ttc computation
- uint16_t n_median : Median filter order for Time-To-Collision

### OUTPUT to UART
- float sub_angles[4] : Subtented angles left, up, right, down (in rad)
- uint64_t frame_time : Frame time (in camera ticks)
- uint16_t no_detect : Number of false detections
- uint16_t init_detect : Initialization done/not done
- float ttc : Time-To-Collision filtered
