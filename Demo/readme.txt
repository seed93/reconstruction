1.图像分割
在matlab中运行 "segmentation\CutImageDir_canon.m"，该函数输入为图片路径，该路径下必须包含25组图片，处理完会在当前目录产生mask、result两个目录，以及对原始图片进行自动旋转

2.标定
运行"calibration\calibration.exe"，可以修改config.yml里的filepath为标定图片的目录。一般每个visibility应该在25以上，最后误差在0.01附近

3.生成模型
a)单个模型
运行"reconstruction.exe config.yml"，可以修改config.yml的参数。

b)批量生成
运行 "BatchProcess.exe path.txt" 可以对路径下的照片批处理，要求在每个路径下包含calib_camera.yml的标定文件
path.txt每一行为"input_path output_path"，生成模型放在输出目录。

reconstruction参数介绍
i)config.yml中影响结果的有金字塔层数、底层图片长宽，层数越多、图片越大质量会变好，时间变久
ii)在CReconstruction.cpp中17行最后一个数变大会使得点云更平滑，第18行第1 3 4个数不要变，第2个变大点云滤波时会条件宽松，第5个数增大会使点云平滑

