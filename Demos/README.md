# 共通する変更点

## CMakeLists.txt > INCLUDE_DIRECTORIES

Linux Mint 15 KDEの標準構成に併せて/usr/include/bulletを追加。

    INCLUDE_DIRECTORIES(
    ${BULLET_PHYSICS_SOURCE_DIR}/src 
    )

↓

    INCLUDE_DIRECTORIES(
    ${BULLET_PHYSICS_SOURCE_DIR}/src 
    /usr/include/bullet
    )

## CMakeLists.txt > add_definitions

-std=c++11を定義するため追加

    add_definitions("-std=c++11")
