# gs101
计算机图形学GAMES101

Code5的编译问题
1.删除CMakeLists.txt中的-fsanitize=undefined语句

2.在终端输入：mkdir build	创建文件

3.在终端输入：cd build	进入build文件

4.在终端输入：cmake ..	..代表CMakeList.txt文件位置，即返回上一个文件夹

5.删除build文件夹中的CMakeCache.txt文件

6.在终端输入：cmake .. -G "MinGW Makefiles"

7.在终端输入：mingw32-make
