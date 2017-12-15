# NVM Tools

**Author:** [Alex Locher](http://www.vision.ee.ethz.ch/~alocher)

For my 3D project I often use nvm files for storing the data of 3D models. NVM files are originally used in VisualSfM from [Changchang Wu](http://ccwu.me/). 
This project contains a bunch of tools I often use for manipulating these files and are just here in case its of any use to somebody..

## NVM_V3 file format

The specification of the original NVM version 3 file format can be found [here](http://ccwu.me/vsfm/doc.html#nvm)

## NVM_V4 file format

The original file format only allows for a very trivial pinhole camera model with a single radial distortion. 
I extended the format into V4, which allows for flexible camera models (see CameraModel.h). 

In addition measurements of features are stored in absolute image coordinates (with top left pixel beeing (0,0) ) rather than relative to the principal point as in NVM_V3. 




## Licence
Copyright 2017 Alex Locher

nvmtools is realeased under the [GPLv3 Licence](https://www.gnu.org/licenses/gpl-3.0.txt).

