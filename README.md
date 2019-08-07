###　1. 项目结构

~~~ｓｈｅｌｌ
.
├── 5PCL3drestruction
│   ├── pcltest.cpp
│   ├── rabbit.pcd
│   ├── 三维重建.cpp
│   └── 配置环境依赖项.txt
├── calibration_and_rectify #　相机标定和纠正
│   ├── calibration_and_rectify.cpp
│   └── img_list.xml
├── distanct #　双目测距
│   └── distanct.cpp
├── img　#　folder to save img
├── imgCaputer　＃ 图片捕获
│   └── imgCaputer.cpp
├── useCamera　＃ 相机使用
│   └── useCamera.cpp
├── utils　
│   ├── chmodCamera.sh　#　摄像头切换
│   └── intrinsics.yml　#　内参矩阵　&　畸变系数
└── README.md

45 directories, 141 files
~~~

### 2. Usage

##### Demo1 useCamera

~~~shell
$ cd utils
$ chmod 755 chmodCamera.sh
$ cd ../useCamera
$ g++ `pkg-config opencv --cflags` useCamera.cpp -o useCamera `pkg-config opencv --libs`
$ ./useCamera
# Press Esc to Exit!
~~~

##### Demo2 imgCaputer

~~~shell
$ cd imgCaputer
$ g++ `pkg-config opencv --cflags` imgCaputer.cpp -o imgCaputer `pkg-config opencv --libs`
# Press Esc to Exit
# Press 'S' to save img #　大写
~~~

##### Demo3 calibration_and_rectify

~~~shell
# 先将图片填写在 calibration_and_rectify/img_list/xml文件中
$ cd calibration_and_rectify
$ g++ `pkg-config opencv --cflags` calibration_and_rectify.cpp -o calibration_and_rectify `pkg-config opencv --libs`
$ ./calibration_and_rectify
~~~

##### Demo4 distanct

~~~shell
$ cd distanct
$ g++ `pkg-config opencv --cflags` distanct.cpp -o distanct `pkg-config opencv --libs`
$ ls
$ ./distanct
~~~

##### Demo5

~~~shell
### 
~~~



###　其他

##### １．如何使用网络摄像头

~~~ｓｈｅｌｌ
#　读取摄像头时候设置相应的地址即可
cap.open("http://192.168.1.100:8080/?action=stream?dummy=param.mjpg");
~~~

##### 2.　uvcdynctrl

github 链接： https://github.com/cshorler/webcam-tools/tree/master/uvcdynctrl

######　（１）安装

~~~
sudo apt-get install uvcdynctrl
~~~

###### （２）关闭uvcdynctrl-udev.log

在linux系统中使用USB摄像头时，都会用到这个uvcdynctrl，但是uvcdynctrl也会一些问题引发输出巨量的log文件，会把系统占用达到100%。关闭方法为：

修改　`/lib/udev`　目录下的 `uvcdynctrl` 中第16行的  `debug=1` 为 `debug = 0` 即可

###### (3)　如何通过 uvcdynctrl来改变摄像头模式

使用如下脚本

~~~
uvcdynctrl -d /dev/video0 -S 6:8  '(LE)0x50ff'
uvcdynctrl -d /dev/video0 -S 6:15 '(LE)0x00f6'
uvcdynctrl -d /dev/video0 -S 6:8  '(LE)0x2500'
uvcdynctrl -d /dev/video0 -S 6:8  '(LE)0x5ffe'
uvcdynctrl -d /dev/video0 -S 6:15 '(LE)0x0003'
uvcdynctrl -d /dev/video0 -S 6:15 '(LE)0x0002'
uvcdynctrl -d /dev/video0 -S 6:15 '(LE)0x0012'
uvcdynctrl -d /dev/video0 -S 6:15 '(LE)0x0004'
uvcdynctrl -d /dev/video0 -S 6:8  '(LE)0x76c3'
uvcdynctrl -d /dev/video0 -S 6:10 '(LE)0x0400'
~~~

可以通过修改最后一行的　`0x0400`　切换摄像头模式：

 `0x0100`, `0x0200`, `0x0300`, `0x0400` 指令分别表示切换到左单目，右单目，红蓝模式，双目模式。

###### (4)　帮助文档

~~~ｓｈｅｌｌ
zhaozhichao@zhaozhichao-MS-7B24:/lib/udev$ uvcdynctrl -h
uvcdynctrl 0.2.4

Manage dynamic controls in uvcvideo

Usage: uvcdynctrl [OPTIONS]... [VALUES]...

  -h, --help               Print help and exit
  -V, --version            Print version and exit
  -l, --list               List available cameras
  -i, --import=filename    Import dynamic controls from an XML file
  -a, --addctrl=vid        Import dynamic controls for vid from default location
  -v, --verbose            Enable verbose output  (default=off)
  -d, --device=devicename  Specify the device to use  (default=`video0')
  -c, --clist              List available controls
  -g, --get=control        Retrieve the current control value
  -G, --get_raw=unit_id:selector        Retrieve the current raw control value
  -s, --set=control        Set a new control value
                            (For negative values: -s 'My Control' -- -42)
  -S, --set_raw=unit_id:selector        Set the current raw control value
                            (value is a hex string of control size,
                             default is little endian- '(BE)' prefix to change:
                             -S 13:1 0x01100c or -S 13:1 '(BE)0x0c1001' )

  -f, --formats            List available frame formats
  -W, --save=filename      Save device controls state to a file
  -L, --load=filename      Load device controls state from a file
~~~


##### 3. 一些常见的术语

~~~ｓｈｅｌｌ
binocular 双目
Stereo 立体
disparity 视差
~~~

