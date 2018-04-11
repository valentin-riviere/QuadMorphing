# X4_detector

Use [Suzuki 85] and Ramer Douglas Peucker algorithm to extract squares from image.
Then, the squares detected are compared in pairs :
1) Distance between centers
2) Ratio (width/height) between both
3) Selection of the minimal area square

## Parameters :
All parameters are define by global variables in x4_detector
- uint16_t t_polling : Time between 2 frames
- uint8_t MaxNumBuffer : Number of frames max per buffer
- uint16_t width/height : Width/Height for camera
- uint16_t offset_x/y : offset_x/y for camera center
- uint8_t binning : Binning on/off
- uint16_t expoAutoMax_us : Maximum exposure time for expoAuto (in us)
- uint16_t grabTimeOut_ms : TimeOut to grab a frame (in ms)
- float FOV_x/y : Field Of View on x/y (in rad)
- uint8_t max_no_detection : Maximum of false detections before reinitialization of the detector
- uint8_t blur_size : Matrix size for blurring (pre-processing of the image)
- uint8_t bin_square : Threshold to digitize the image (in pixel 8bits-intensity)
- float k_approx : Coefficient to approximate lines (percent/100 of contour length)
- uint16_t thresh_area : Minimal size of a detected area (in px^2)
- float thresh_cos : Threshold on cos between 2 segments to define a right-angle
- uint16_t thresh_diff : Distance maximum between 2 square centers
- float thresh_ratio : Maximal difference between 2 square ratios

## Flags in def.h :
- TIME_EXEC : Execute algo and measure time of execution
- VIEWER_ON : Active viewer (need disabling TIME_EXEC)
- DEBUG : Use to debug only when miss detection
- ROI : Use ROI with roi_offsets parameters
- BLUR : Blur image before detection
- UNDISTORT : Not implemented yet (path_dist_coeffs is relative path to saved coeffs)
- CANNY : Use Canny before detection (not tested yet)
