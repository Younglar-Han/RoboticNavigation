# Notes of ROS

## 问题

###  rosrun tf view_frames 时报错

```shell
Listening to /tf for 5.0 seconds
Done Listening
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/tf/view_frames", line 119, in <module>
    generate(dot_graph)
  File "/opt/ros/noetic/lib/tf/view_frames", line 75, in generate
    with open('frames.gv', 'w') as outfile:
PermissionError: [Errno 13] Permission denied: 'frames.gv'
```

解决：

```shell
cd /opt/ros/noetic/lib/tf/

sudo vim view_frames 
```

在

```shell
            vstr = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[1]
```

后加上 .decode('utf-8')

```
            vstr = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()[1].decode('utf-8')
```



### Git push 时超时 

```shell
fatal: unable to access ‘https://XXX: Failed onnect to github. com port 443: Timed out
```

解决

```shell
git config --unset https.proxy #不设置git的代理，根据项目域名选择设置http或者https
```



### Python 在函数内修改全局变量

在函数起始用global 声明要修改的全局变量

### noetic编译turtlebot3_applications报错

安装ar-track-alvar

在src下

```shell
it clone https://github.com/ros-perception/ar_track_alvar.git -b noetic-devel
```

此时编译会报错，需要删除turtlebot3_applications中的turtlebot3_panorama，此时编译即可成功



### base_link和实际的朝向是相反的
