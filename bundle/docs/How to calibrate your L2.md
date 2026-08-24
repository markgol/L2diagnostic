# How to calibrate your Unitree L2 4D LiDar

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

![60dfda40908a471c978f42220e40105a](file:///C:/Users/photo/Pictures/Typedown/60dfda40-908a-471c-978f-42220e40105a.png?msec=1787522253277)

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

**Xi Angle**

This parameter does not have a published definition but the likely is the Laser beam not lying exactly in the mirror's scan plane (optical skew).  This is an angular correction.

**Adjustment of parmeter should be done after RangeBias, Alpha Angle Bias and initial range correction calibration have been performed.**

The Xi angle and Beta Angle are more challenging corrections to make. One should start with the builtin values.

Adjustment of parmeter should be done after RangeBias, Alpha Angle Bias and range correction calibration have been performed.

Adjusting this parameter requires using a small target, 0.25 to 0.5 meters square or round,  at some distance (10 meters or more)

Xi and Beta angle should be adjusted togehter.  Grossly incorrect values of these parameters will result in double image (this can also appear as one object behind the another)  Smaller incorrect erros will result in smearing image of the target.  The goal is to minimize the smear in all directions(x,y,and z) for the target.

add image of double target

**Beta Angle**

This parameter does not have a published definition but the likely is the mechanical tilt of the fast scan assembly relative to the rotating head.  This is an angular correction.

**Adjustment of parmeter should be done after RangeBias, Alpha Angle Bias and initial range correction calibration have been performed.**

The Xi angle and Beta Angle are more challenging corrections to make. One should start with the builtin values.

Adjusting this parameter requires using a small target, 0.25 to 0.5 meters square or round, at some distance (10 meters or more)

Xi and Beta angle should be adjusted togehter. Grossly incorrect values of these parameters will result in double image (this can also appear as one object behind the another) Smaller incorrect erros will result in smearing image of the target. The goal is to minimize the smear in all directions(x,y,and z) for the target

add image of double target

****Range Scale****

RangeScale does not have a distortion effect. It affects the accuracy of the distances measured in the point cloud. The default value is 0.00978. If distances measured are shorter than expected then increase this number. If distances are longer than expected the decrease this number. Unity is 0.001 which just converts range mm to meters.

**Theta angle Bias**

This is an offset to align the XY axis for azimuth (rotation in the XY plane). When you mount the L2 on your platform you expect the this angular aligment is such that you can which direction is forward. The builtin value for this is 119 degrees. You may want to adjust this to take into account the XY rotation (Azimuth angle) of your physical mount.

This adjustment does cause distortion. It only causes rotation in the XY plane of the point cloud.

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



**Stage 4  Creating the piecewise cubic spline fit**



**Stage 5 Save Calibration file.**
