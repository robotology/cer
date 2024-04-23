# imagePatternRecognition

**This module is currently under development/testing and should be not used in a user application.**

## Description
This module is used to find a pattern image within an another image coming from a RGB camera.

## Usage
An input port receives a string containing the path to the jpg file of the pattern image.
An output port returns the pixel coordinates of the better match between the pattern image and the camera image.

**Biggest limitation: the pattern image must have the same pixel size of its correspondence within the camera image**
