# Reconstruction

Reconstruction是一个用于多视角三维重建的项目，该项目参考[Beeler10](https://graphics.ethz.ch/publications/papers/paperBee10.php)的工作完成，使用10台佳能D1200相机、6盏闪光灯同步采集，最终由算法计算得到人脸三维模型。项目有VS2010和VS2015两个版本。

# 项目依赖

- [PCL](http://pointclouds.org/) 静态库。由于库文件过于庞大，未包含在项目中，需要额外下载本人已编译好的[静态库]()，有精简版（仅包含该项目所需）和完整版，解压至项目同一级目录，VS2010使用1.6.0版本，VS2015使用1.7.2版本。请用户自行编译，可参考本人[博客](http://ld99.top/2015/07/07/%E9%A1%B9%E7%9B%AE%E4%BB%8EVS2010%E8%BF%81%E7%A7%BB%E5%88%B0VS2015/#%E9%9D%99%E6%80%81%E7%BC%96%E8%AF%91_PCL)。
- OpenCV静态库，已包含在项目中，请在`lib\`目录下解压**opencv.7z**
- [Armadillo](http://arma.sourceforge.net/)，已包含在项目中
- [MeshLab](http://meshlab.sourceforge.net/)，请用户自行下载后解压到`Demo\meshlab\`目录下

注：由于库文件体积太大，并未上传debug版，如有需要请单独索取。

# 项目框架

项目分为4个模块，核心为reconstruction，完成了重建的主程序，CloudOptimization用于生成涉及PCL函数的链接库，其他两个项目为个人使用。

Demo文件夹下放有已编译好的可执行文件和demo资源，可直接运行`reconstruction.exe INPUT\myself\config_myself.yml`或`reconstruction.exe INPUT\ETH\config_ETH.yml`，生成ply文件用MeshLab打开。Demo需要将解压MeshLab到`Demo\meshlab\`目录下。