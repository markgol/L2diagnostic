# How to calibrate your Unitree L2 4D LiDar

Revision V2.0.1

## The issue:

Each L2 has a set of builtin calibration parameters that are used to convert the point cloud packets to an x,y,z position in a point cloud space.  Accurate conversion is needed to properly map point clouds for use in SLAM and odometry prediction.  The builtin calibration parameters are set at time of manufacture.  There is no explanation how this calibration is done, under what conditions, with what objectives.  

## Calibration parameters used in the conversion of point cloud data:

This list of builtin calibration parameters used in the conversion of the point cloud packet to point cloud space (x,y,z in meters).

- RangeBias (Offset in mm for measured range, applied before RangeScale)
  in mm, L2 unit specific

- RangeScale (Converts range mm to meters, scalar adjust for precise conversion)
  unitless, same across multiple units

- AlphaAngleBias (Elevation offset of the Z axis tilt, as manufactured)
  radian, L2 unit specific

- AlphaAngleStepSize (incrmental elevation angle between fast scan points)
  radian, same across multiple units

- ThetaAngleBias (Azimuth offset to align L2 to XY angular plane)
  radians, same across multiple units

- XiAngle (unpublished value, appears related to laser alignment, as manufactured)
  radian, L2 unit specific

- BetaAngle (unpublished value, appears related to laser alignment, as manufactured)
  radian, L2 unit specific

- MinRange (point with range distance less than this value is removed)
  meters, same across multiple units

- Max Range (point with range distance greater than this value is removed)
  meters, same across multiple units

- A axis distance (z distance from L2 mounting surface)
  meters, same across multiple units

- B axis distance (unpublished value)
  meters, same across multiple units

The Unitree SDK does not provide for overriding these values without rewriting the conversion utilities.
Observations:

-------------

It has been observed that calibration values on some L2 units are inaccurate and the correct value may be significantly different that what is sent in the point cloud data packets.  For example on my L2 the Alpha Angle Bias builtin value is 1.86 deg but the correct value is actually close to 1.25 degrees, RangeBias builtin value -365 mm but the correct value is close to -535 mm.  Having the incorrect values results in distortions of the point cloud scan which becomes pronounced when accumlating point cloud scans.

The L2diagnostic application allows the user to override these builtin calibration values in order to reduce distortion in the accumulated point cloud from mulitple scans.

## Adjusting the parameters:

When the L2diagnostic starts select Switch to Calibration mode and you will see the following:

![518430fa-1f62-4bc0-8f64-396396a0178c](file:///C:/Users/photo/Pictures/Typedown/518430fa-1f62-4bc0-8f64-396396a0178c.png)

The live data controls lets you connect/disconnect from the L2, Clear the point cloud, load a point cloud, and save a point cloud and switch to diagnostic mode.

When you select "L2 connect" the point cloud window gets displayed and live point cloud data is shown. Once connected and the Enable Calibration Override is enabled you can see in real-time the effects of changing the over-ride parameters. The point cloud accumulates the scan overtime so you may want to clear the point display to clearly see the effects.

This describes the parameters and how to adjust them.  Range correction will be discussed in the next section.

This lists the parameters in the order they should be adjusted.

**Range Bias**

This parameter effects the barrel/pincushion distortion of the point cloud.  RangeBias is a negative number.  If you are closer to 0 than the distortion moves toward increasing barrel distortion.  As the value moves toward -1000 the distortion moves toward a pinchushion distortion.

![2ed84019-6bcc-456e-a96e-ad57c2bc8e55](file:///C:/Users/photo/Pictures/Typedown/2ed84019-6bcc-456e-a96e-ad57c2bc8e55.png)

Image of barrel distortion

![c7775dcb-92c2-44c0-8d34-0b8ce5403d28](file:///C:/Users/photo/Pictures/Typedown/c7775dcb-92c2-44c0-8d34-0b8ce5403d28.png)

Image of pinchushion distortion

It is more obvious looking at the bottom edge of this scene.

This parameter should be adjusted to minimize the this distortion.

**Alpha Angle Bias**

This is the tilt angle of the Z axis relative to the XY plane.  This tilt is the result of the manufacturing assembly tolerances.  This can cause significant distorion in an accumlated point cloud.  For example if the L2 is mounted parallel to the floor and you looked at ceiling then the ceiling should appear flat.  If this is not set correctly then the ceiling will appear to have 2 diverging planes that meet directly above the L2.

![d941de90-7e6a-4933-a2e7-661071745808](file:///C:/Users/photo/Pictures/Typedown/d941de90-7e6a-4933-a2e7-661071745808.png)

Example image of diverging planes with grossly wrong Alpha Angle Bias.

![a4beddc5-3a9d-41f6-9987-ef9e41e66274](file:///C:/Users/photo/Pictures/Typedown/a4beddc5-3a9d-41f6-9987-ef9e41e66274.png)

Same view with correct Alpha Angle Bias.

The value of this parameter should be adjusted to minimize this affect over the typical range for your scenes.  There is sufficent non-linear range distortion that it may not be possible to perfectly align this over the full range of the sensor.  This is unique to each L2.

**Alpha Angle Step size**

This is the evelation angle step size between fast scan points.  There are always 300 elevation index points in every scan.  The step size determines the total hemispherical fast scan angle.  The wrong value causes distortion looking almost identical to the Alpha Angle Bias with the distinguishing difference that this parameter changes the extent of the fast scan hemisphere.  There appears to be a discrepancy from the factory 0.600 degrees which is 180 degrees / 300 points instead 0.602 which is the 180 degrees / 299 (number of scan increments in the fast scan).  Suggest using 0.602 or adjusting from that value.  The goal is to minimize the  diverging planes using both Alpha Angle Biad and Alpha Angle Step Size.  If you have a way of veryfing the hemisphere scan total angle you can also use this to estimate the step size (total hemispherical angle/299).



**Xi Angle**

This parameter does not have a published definition but a guess is the Laser beam not lying exactly in the mirror's scan plane (optical skew).  This is an angular correction.

**Adjustment of parmeter should be done after RangeBias, Alpha Angle Bias and initial range correction calibration have been performed.**

The Xi angle is one of the more challenging corrections to make. One should start with the builtin values.

Adjustment of parmeter should be done after RangeBias, Alpha Angle Bias and range correction calibration have been performed.

Grossly incorrect values of these parameters will result in double image that appears shifted in theta. Smaller incorrect erros will result in smearing image.  The goal is to minimize the smear

![e19f090f-ecef-4aed-935f-43f4592f971f](file:///C:/Users/photo/Pictures/Typedown/e19f090f-ecef-4aed-935f-43f4592f971f.png)

Image of double target with exagerated value of Xi.

**Beta Angle**

This parameter does not have a published definition but a guess is the mechanical tilt of the fast scan assembly relative to the rotating head.  This is an angular correction.

**Adjustment of parmeter should be done after RangeBias, Alpha Angle Bias and initial range correction calibration have been performed.**

The Xi angle and Beta Angle are more challenging corrections to make. One should start with the builtin values.

Adjusting this parameter can benfit from using a small target, 0.25 to 0.5 meters square or round, at some distance (10 meters or more)

Grossly incorrect values of these parameters will result in double image (this can appear as one object behind the another) Smaller incorrect errors will result in smearing image of the target. The goal is to minimize the smear for the targets.

****Range Scale****

RangeScale does not have a distortion effect. It affects the accuracy of the distances measured in the point cloud. The default value is 0.00978. If distances measured are shorter than expected then increase this number. If distances are longer than expected the decrease this number. Unity is 0.001 which just converts range mm to meters.

**Theta angle Bias**

This is an offset to align the XY axis for azimuth (rotation in the XY plane). When you mount the L2 on your platform you expect the this angular aligment is such that you can which direction is forward. The builtin value for this is 119 degrees. You may want to adjust this to take into account the XY rotation (Azimuth angle) of your physical mount.

This adjustment does not cause distortion. It only causes rotation in the XY plane of the point cloud.

## Range correction calibration

The range correction calibration isused to generate a calibration file that can be used to correct for the non-linear range response of the L2.  It is important to note what it can not correct.  It can not correct measured range values that have more than 1 solution for true range.  This is primarily been observed only very close range values (<1.5 meters).  The result of this issue is adding a new parameter call MinTrustedRange.  This is shoterest range were this effect is never observed.  This is useful for processing like ICP which can use then this value to exclude fitting of points closer than this range.  This does not mean there isn't useful data at shorter ranges that could be used for collision avoidance.  It means that a reported range of there could be uncertainty of what the range acutally is.  For exmaple: A measured range of 358mm could have a true range of 457mm or 505mm with now ability to know which is correct.  The calibration process does save .csv files that can illustrated this issue.  It can help set the MinTrustedValue.

![d054adc4-7f34-4912-8d6a-3e2b8468f582](file:///C:/Users/photo/Pictures/Typedown/d054adc4-7f34-4912-8d6a-3e2b8468f582.png)

This is a more invloved process than just adjusting the calibration parameters.  This is a process that has been broken into 5 stages.  You do not have to be connected to L2 if you already saved a flattened point cloud file for processing.

This requires having flat calibration targets that cover a representative range of interest.  The simplest is a vertical flat wall and a horizintal flat ceiling.  It can be complicated than that. Like having multiple flat targets to cover the needed minimum range correction to the maximum range correction needed.  The requirement is as least one flat surface.

The processes allows you to move back and forth between the stages until a satisfactory solution is found. 

Note: This process uses flattened scan accumulation.  This were all scans are collapsed to the vertical plane at the position StartScanAngle +ScanAngleWidth/2 perpendicular to XY plane.  Correction for Beta and Xi angle are set to 0.0 and A axis and B axis distances are set to 0.0.

**Stage 1 Meta data for calibration**

Before this Stage is enabled th euser must select the 'Init L2 for Range Cal.' button.

![e9606ae5-7226-417c-bcf8-b54d0501abff](file:///C:/Users/photo/Pictures/Typedown/e9606ae5-7226-417c-bcf8-b54d0501abff.png)

This allows the user to enter some of the metadata that will be saved in the calibration file.

If you are not connected to the L2 then the Firmware field will be labelled 'unkown'.

If you are connected to the L2 and the FW version shows 0.0.0.0 then application has not received the version information from the L2.  You should select the 'Init L2 for Range Cal.' button again and restart Stage 1.

The Min Trusted range is set by the user but is not used in any of the correction calculations.

Other metadata values included in the calibration file are all the override biases.



**Stage 2 Point cloud acquisition**

If you have not saved a flattened point cloud already it is recommended that you use this stage to capture one.

You must be connected to the L2 to enter this stage.

![0b1ec0a9-4a79-4975-8613-b76e54b54059](file:///C:/Users/photo/Pictures/Typedown/0b1ec0a9-4a79-4975-8613-b76e54b54059.png)

It is recommended that you capture at least 50K points at a minimum.  In this dialog you specify a start angle and a scan angle width.  The scan angle width should be narrow 2-5 degrees.  The narrower scan width will be more accurate but it takes longer to accumultae points.
You L2 should be tangential in azimuth to your flat targets.

The first thing you can do is look at your alignment to your target using the 'View full 360' button.  This can quickly confirm you aligment.  The example here is a vertical column .15m away  at the base.
![48eecd1b-88dc-4238-97d8-9cd9e4aa5138](file:///C:/Users/photo/Pictures/Typedown/48eecd1b-88dc-4238-97d8-9cd9e4aa5138.png)

The next is to look at the 3d scene using the narrow scan parameters to verify correct angles.

![64c034b4-3726-43b8-bd5b-b3f2c4832219](file:///C:/Users/photo/Pictures/Typedown/64c034b4-3726-43b8-bd5b-b3f2c4832219.png)

If this looks good you can then select the flattended scan.  This will aslo start an acqusition.  You can pan, zoom, rotate the same as live data.  Is it useful to look at the side veiw of the flattened scan.  You can click (Re)Start Acq if you see an a pertubation in you acquisition.

![c2003e83-5b9d-4d32-9786-4d07c5f944ce](file:///C:/Users/photo/Pictures/Typedown/c2003e83-5b9d-4d32-9786-4d07c5f944ce.png)

Once the minimum number of points is collected then the 'Save acq.'' will enable and you can save the current point cloud.

When you click Exit the normal scan parameters and calibration overrides will restored.

**Stage 3 Analysis of data**

![dd822e2d-c8cc-41aa-8a0b-2996ead7ece7](file:///C:/Users/photo/Pictures/Typedown/dd822e2d-c8cc-41aa-8a0b-2996ead7ece7.png)

The default parameters cluster bin size, convolution filter sigma, and filter radius sigma should be sufficent for the L2.  A CSV base filename can be specified to store results of the analysis.  These files will have various names appended to distinguish what they contain.  These files are generated for stage 3, 4, and 5.

This stage is interactive with the user. You load the flattened point cloud file you saved in stage 2. You then select the 'Analyze' button.

 This brings up a graphical interface that you can add line segments and exclusion boxes.  These exlcusion boxes are used to highlight the points in the graph that are not used in the range correction.  The line segments cover the range of your flat surface targets.

The derived point spread function will be color coded green, yellow, and red.  Red dots are highly suspect and likely need to be excluded.  Green dots are points will reasonable statical measurements.  The yellow dots are points that may be question as whether to include them or not in the range correction calibration.

Line segments are used to define the what are the flat surfaces.  They should not overlap or mutiply define range points in the graph.  If they do the lines conflicting line segments will be highlighted in magenta.  The normal color of a line is yellow.

![ca3502f7-77bb-4952-bd43-545b3b535e6d](file:///C:/Users/photo/Pictures/Typedown/ca3502f7-77bb-4952-bd43-545b3b535e6d.png)

**Stage 4  Creating the piecewise cubic spline fit**

If you have run stage 1 yet then the stage 1 dialog will be presented to user to fill out.

The only parameter for stage 4 is the number of cubic spline steps to use on the fit.

![a8838bef-1e8f-44be-974b-4acadcca4cae](file:///C:/Users/photo/Pictures/Typedown/a8838bef-1e8f-44be-974b-4acadcca4cae.png)



**Stage 5 Save Calibration file.**

Will prompt the user for the name of the CSV calibration file to save.

This completes the range correction calibration process.  You should load the calibration you just created and enable it to see the effect on the point cloud.  You can repeat any of the stages as needed but if you do you must finish the stages in sequence.

Once you load the calibration file the Range Calibration Info dock window will look similar to:
![0f62d383-d184-4919-ba68-e5f88a374a65](file:///C:/Users/photo/Pictures/Typedown/0f62d383-d184-4919-ba68-e5f88a374a65.png)

## Revision History

V2.0.0    2026-8-23 RC1 Initial release,  missing  image for Beta adjustment.

V2.0.1    2026-8-26 Added Alpha Angle Step Size, reorganized GUI
