# ant\_trails #

Implementations of an embodied ant algorithm for foraging in [Stage](https://github.com/rtv/stage). Includes an implementation of [LOST](https://autonomylab.github.io/doc/vaughan_LOST.pdf) and [SO-LOST](https://autonomylab.github.io/doc/sadat_alife2010.pdf).

* Author: [Jacob Perron](http://jacobperron.ca) ([Autonomy Lab](http://autonomylab.org), [Simon Fraser University](http://sfu.ca))

## Install (Ubuntu) ##

#### Install [Stage](https://github.com/rtv/stage) ####

```bash
# Install dependencies
$ sudo apt-get install git cmake g++ fltk1.1-dev libjpeg8-dev libpng12-dev libglu1-mesa-dev libltdl-dev git
# Clone Stage in home directory
$ cd && git clone https://github.com/rtv/stage.git
$ cd stage
$ mkdir build && cd build
$ cmake -DCMAKE_INSTALL_PREFIX=$HOME/stage-lib ..
$ make
$ make install
```

To set up environment add the following to the file `~/.bashrc`:  
```bash
STG=$HOME/stage-lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$STG/lib64  # note: may be 'lib'
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$STG/lib64/pkgconfig
export PATH=$PATH:$STG/bin
```

Make sure to source: `$ source ~/.bashrc`

#### Install ant_trails ####

```bash
# Clone this repository
$ cd && git clone https://bitbucket.org/jacobperron/ant_trails.git
$ cd ant_trails
$ mkdir build && cd build
# Build
$ cmake ..
$ make
```

## Run ##

By default the code is configured to use the SO-LOST algorithm.

```bash
# Go to the worlds directory
$ cd ../worlds
# Run example world file
$ stage ant.world
```

Press 'P' to start/pause and '[' and ']' to decrease or increase the rate at which time passes.
