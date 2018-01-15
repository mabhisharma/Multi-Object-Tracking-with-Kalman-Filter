# Multi Object Tracking with Kalman-Filter

This project demonstrates Multi Object Tracking with Kalman-Filter when we already have detections. In this particular project I already had detections (got detections from image processing which is not mentioned here). Multi Object tracking is done using Kalman Filter where we estimate the next position of a particular object using the detection in the previous frame. Once we have the prediction of the object in the next frame we associate this estimate with the actual detection and now using hungarian algorithm we assign each detection with the tracked objects. Sample output from this Multi Object tracker is below where Black colored objects are the actual detections and colored are predictions -

<p align="center">
  <img src="Multi-Object-Tracking.gif" alt="Multi Object Tracking with Kalman-Filter" height="300" width="400" />
  <p align="center">Sample Multi Obejct Tracking output</p>
</p>

# References -
http://studentdavestutorials.weebly.com/multi-bugobject-tracking.html
