opencv-ar
=========

AR marker recognition with OpenCV forked from http://opencv-ar.sourceforge.net by Allen Choong

Introduction
------------

Quoted from README in http://opencv-ar.sourceforge.net

> OpenCV-AR,(I tried to pronounce as OpenCVAR, for easier pronunciation), is a software library, used for Augmented Reality development. It is a software library targeted on Linux, though it is also able to works on Windows platform. It is an alternative of ARToolKit, which is the most popular open source library toolkit for Augmented Reality application development. However, ARToolKit has dual license, GPL and commercial license. OpenCV-AR uses OpenCV as its core computer vision algorithm for the fiduciary marker detection. The result of the computer vision calculation is converted to the 3D Computer Graphics matrix. Therefore, it is possible to integrated with any 3D comptuer graphics engine, especially OpenGL.

> One of the differences between OpenCV-AR and ARToolKit is the license. OpenCV-AR is using BSD License, as a gratitude towards OpenCV library. Next, OpenCV-AR is improved with fiduciary marker detection which is similar to ARTag. However, the binary information within ARTag marker consists of 6x6, yet OpenCV-AR uses 8x8 as a different algorithm.

Installation
------------

Dependencies:
* **OpenCV 2.3** or newer

~~~ sh
$ cmake .
$ make
$ bin/ARTest 0 template/2x2-01.png
~~~
