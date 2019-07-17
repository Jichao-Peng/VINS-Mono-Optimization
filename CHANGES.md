## line_vins_estimator
- 2019-07-16 
修改了gitignore；
加入了vins_pro.launch文件，以后启动调用这个launch文件；
加入了line_local_parameterization和line_projection_factor
在utility中加入了普吕克坐标和正交坐标的转换函数

- 2019-07-17
line_projection_factor还差一点
utility里面添加了修改了几个函数

## line_feature_tracker
- 修改了parameter.cpp中读取配置文件的内容

## estimator & line_feature_manager
- 添加了vector2double和double2vector两个函数中从优化变量和featuremanager中的变量之间的转换

## line_feature_tracker
- 2019-07-17
将发布的直线端点改成了归一化平面

## estimator
-初步完成了marg和slidWindow部分，待测试 
