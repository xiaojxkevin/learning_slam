# 记录各种包的安装

## My Device

Ubuntu 22.04, x86-64, Intel i7-12700H

## Eigen3

这个应该很常见了，随便找一个博客跟着就行。我的放在了 `/usr/include/eigen3` 下。比如可以尝试 `sudo apt install libeigen3-dev`

## Sophus

1. [github](https://github.com/strasdat/Sophus) clone 下来
   
2. 看很多人都说需要 `fmt` 这个库，但我自己似乎通过 `sudo apt install libfmt-dev` 安装过了，直接使用也是没有问题的, so, what ever。
   
3. 我的 `Eigen` 库是 `3.4.0` 的版本，因此将 `CMakeLists.txt` 第56-58行修改为
   ```cmake
   if(NOT TARGET Eigen3::Eigen)
        find_package(Eigen3 REQUIRED)
    endif()
   ```

4. 随后就是一般程序
   ```bash
   mkdir build && cd build
   cmake ..
   sudo make install -j8
   ```

5. 最后install的文件在 `/usr/local/share/sophus` 和 `/usr/local/include/sophus/` 下 (注意如果要删除的话需要手动进行)

6. **有关(cmake)** 正如我们第二条所述，`fmt` 不是通过cmake得到的，于是乎，我们需要手动link fmt. 例如：
   ```cmake
   find_package( Sophus REQUIRED )
   include_directories( 
       ${Sophus_INCLUDE_DIRS}
   )
   target_link_libraries(your_target_name fmt)
   ```

## Ceres and G2O

找到了一篇不错的[blog](https://blog.csdn.net/weixin_53660567/article/details/120295824), 照着上面来就好了。**注意要修改CMakeLists的find Eigen(如果Eigen是3.4.0的话)**

1. 对于`Ceres`, install的文件在 `/usr/local/lib/`, `/usr/local/lib/cmake/Ceres`, `/usr/local/include/ceres/`.
   
2. 对于`g2o`,  `/usr/local/include/g2o/`, `/usr/local/lib/cmake/g2o/`, `/usr/local/lib/`, `/usr/local/bin/`

3. 可以在 [这个CMakeLists.txt](../pose-graph/project/g2o-optimize/CMakeLists.txt) 找到一份关于g2o使用有关的cmake files.