# Colorblob Information
### Quick Rundown of What Colorblob Does
Assuming it has the topic of an actively published Image message and corresponding CameraInfo, Colorblob will produce a realworld coordinate (x,y,z) of the centroid of the colorblob. 

Service calls can be used to change the color, and blob-size thresholding. There are two service calls available:
* **color_blob_find** 
  * this is a simplistic implementation and usable for 90% of use cases that allows the user to change the minimum blob size they would expect for a given colorblob. This can be used to erase background noise of the chosen color.
* **color_set_blob_dimensions**
  * This is a much more advanced version of the prior service call that lets you threshold both the minimum and maximum size of the expected blob as well as the aspect ratio
  
  It is important here to note that a minimum "size" of the colorblob refers to the amount of pixels a blob consists of in the view of the camera, and it will use whichever is less between width or height. The size you set is completely dependent to how close/rotated your object is relative to the camera view and not a realworld size of the object itself. E.g. a red thread would be a larger colorblob than a red bowling ball if the bowling ball is far in the background and only taking up 15 pixels height/width and the thread is dangling right in frnot of the camera.

  The real world coordinate is calculated using the focal length from the camerainfo message for X & Y along with the aligned_depth image for the Z coordinate


## Colorblob Troubleshooting
Colorblob was initially implemented to manipulate various handles/objects and as such has a somewhat rudimentary approximation of angle of the blob. This angle is produced by calculating a bounding rectangle around the blob and the rotation of that rectangle to approximate its rotation. A few limitations can arise from this:

1) The rotation is calculated using the optical frame of the camera + a yaw rotation yielded from the blobs rotation. Thus the **blobs orientation will always share the same Z axis as the camera's optical frame**.
2) The blob must be assymetrical to properly calculate its angle. Square/circular blobs will have unusable rotation. The blob will always have the **longer edges of its bounding rectangle side be considered the "height", and the shorter sides be the "width"**.
3) Due to there being no true top or bottom to any arbitrary blob of color, if you flip an assymetrical blob of color 180 degrees, the bottom of the bounding rectangle will become the new top, and the top the new bottom. **There is a tipping point in which the transform produced will flip back and forth in 180 increments (this occurs where the long end of the bounding rectangle is the same as the camera's horizon)**

If the OpenCV window freezes on an image or doesnt show at all, it almost always means that youre not getting any data from the realsense. Check that the realsense didnt get unplugged in motion and that the realsense node is publishing on the topic that you expect. Sometimes the realsense node will randomly prefix all its published topics with "camera/" and its painful.