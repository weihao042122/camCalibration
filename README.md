# camCalibration
张正友标定法与畸变校正模型与双目摄像头外参标定 示例代码

### 代码主要组成：
1. 张正友摄像机标定。
2. 畸变校正。分别包含普通摄像机模型（CV）和鱼眼摄像机模型(fisheye)下两种标定和校正方法。
3. 外参标定。
4. shot目录存放着demo数据

cmake 编译完成后可以使用下面脚本来完成标定
>calibration.sh PATH1 PATH2 OUT_PATH

##### PATH1 
左侧摄像头拍摄的图片集目录，n.bmp(默认是bmp格式的，可以修改宏来使用jpeg格式)。
##### PATH2
右侧摄像头拍摄的图片集目录，n.bmp(默认是bmp格式的，可以修改宏来使用jpeg格式)。
##### OUT_PATH 是参数的保存目录。
1. calibResult0.txt、calibResult1.txt 分别保存两个摄像头的内参。
2. extrinsics.tx 是保存两个摄像头之间的外参
