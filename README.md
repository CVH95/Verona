# ROVI1 Visual Servoing

This repository contains the final project for ROVI 1 (SDU).

### Downloading and compiling the RobWorkStudio Pluggin:

```sh
  $ mkdir ~/workspace
  $ git clone https://github.com/CVEarp/ViServ_ROVI1/
  $ mv ViServ_ROVI1 RoVi1-Final-Project
  $ cd RoVi1-Final-Project/RWStudio_plug-ins/SamplePluginPA10/
  $ mkdir build
  $ cd build/
  $ cmake ../
  $ make
```

### Compiling the tests for both feature extractors:

```sh
  $ cd ~/workspace/RoVi1-Final-Project/feature_extraction/marker2/src
  $ cmake .
  $ make
  $ cd ~/workspace/RoVi1-Final-Project/feature_extraction/marker3/src
  $ cmake .
  $ make
```

The RWStudio pluggin only uses the color marker for real feature-extraction visual servoing.

### Visual Servoing Block Diagram

![alt text](diagram.png)

### Documentation

- [Robwork](http://www.robwork.dk/apidoc/nightly/rw/) installation and documentation.
- [OpenCV](https://docs.opencv.org/3.4.0/d9/df8/tutorial_root.html) installation and documentation.
