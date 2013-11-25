# BasicDemo

## ビルド方法

cmakeにBULLET_PHYSICS_SOURCE_DIRとUSE_GLUTを-Dオプションで渡すとビルド可能となります。

    cd Demos/BasicDemo
    mkdir build
    cd build
    cmake -G Ninja -D BULLET_PHYSICS_SOURCE_DIR=`pwd`/../../../ -D USE_GLUT=1 ..
    ninja

