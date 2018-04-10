# X4_detector

Execute x4_detector to detect 2 squares with same ratio (width/height) and same center.
Use [Suzuki 85] and Ramer Douglas Peucker algorithm to extract squares from image.

## Parameters :
- Global variables in x4_detector

## Flags in def.h :
- TIME_EXEC : Execute algo and measure time of execution
- VIEWER_ON : Active viewer (need disabling TIME_EXEC)
- DEBUG : Use to debug only when miss detection
- ROI : Use ROI with roi_offsets parameters
- BLUR : Blur image before detection
- UNDISTORT : Not implemented yet (path_dist_coeffs is relative path to saved coeffs)
- CANNY : Use Canny before detection (not tested yet)
