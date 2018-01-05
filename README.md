This repo contains the final project for ROVI 1 (SDU).

# Compiling:

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

# To compile the tests for both feature extractors:

```sh
  $ cd ~/workspace/RoVi1-Final-Project/feature_extraction/marker2/src
  $ cmake .
  $ make 
  $ cd ~/workspace/RoVi1-Final-Project/feature_extraction/marker3/src
  $ cmake . 
  $ make 
```

The RWStudio plug-in only uses the color marker for real feature-extraction visual servoing.
