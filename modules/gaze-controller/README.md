# gaze-controller usage

## Request to controller:

The request to the gaze controller must contain:
- control-frame
- target-type (cartesian, angular, image)
- target-location (list of x, y, z)


### Example

`(control-frame gaze) (target-type image) (target-location 0.2 0.3 1.0)`
