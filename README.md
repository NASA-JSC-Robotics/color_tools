# ColorNames implementation

## Summary

This is an implementation of the *color names* algorithm.
It depends on the OpenCV library.
The algorithm uses a lookup table to determine which of eleven named colors each pixel represents.

## Build

This project uses cmake to create the build files.
If you are using make for compiling:

```bash
mkdir build
cd build
cmake ..
make
```

Otherwise it can be included in a colcon workspace and built with `colcon`.

## Run

Three executables and a library are created.

*color_image* intakes a color image from the camera and displays a false colored image with each pixel given it's *named* color.

```bash
./color_image -camera_id=0
```

Camera id defaults to 0, and can be omitted from the command line.

*color_mask* intakes color image from the camera and a color defined on the command line.
It displays an image where the pixel that match the defined color retain their original color; the other pixels are set to their greyscale value.

```bash
./color_mask -camera_id=0 -color=blue
```

The default camera_id is 0 and the default color is yellow.
If the either default is acceptable, it can be omitted from the command line.
Colors are always lower case and can be one of the following:

- black
- blue
- brown
- grey
- green
- orange
- pink
- purple
- red
- white
- yellow

## References

J. Van De Weijer, C. Schmid, J. Verbeek, and D. Larlus, “Learning
color names for real-world applications,” *IEEE Trans. Image Processing*,
vol. 18, no. 7, pp. 1512–1523, 2009.
