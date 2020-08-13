### 1. 

```bash
sudo gedit ~/.bashrc

#在bashrc文件中加入
export PATH="/home/sqg/下载/CLion-2020.1.2/clion-2020.1.2/bin:$PATH" #地址根据自己改
#保存退出
source ~/.bashrc
```

### 2.

![](/home/sqg/图片/clion.png)

修改File -> Setting -> Build,Execution,Deployment -> CMake 中的配置，将CMake options一栏修改为：

```
-DCATKIN_DEVEL_PREFIX:PATH=/home/sqg/catkin_ws/devel
```


将Gerenation path一栏修改为：

```
/home/sqg/catkin_ws/build
```

### 3.

如果此时cmake还是匪夷所思的报错，有一种可能是：clion需通过终端打开，而不能通过图标打开，如

```bash
~/下载/CLion-2020.1.2/clion-2020.1.2/bin$ sh clion.sh　#切换到你的clion目录，通过命令行打开clion.sh
```



[参考链接](https://blog.csdn.net/Jay_2018/article/details/102666342?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.nonecase&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.nonecase)