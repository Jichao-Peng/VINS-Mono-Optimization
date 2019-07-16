## line_vins_estimator
- 2019-07-16 
修改了gitignore；
加入了vins_pro.launch文件，以后启动调用这个launch文件；
加入了line_local_parameterization和line_projection_factor

## line_feature_tracker
- 修改了parameter.cpp中读取配置文件的内容

## estimator & line_feature_manager
- 添加了vector2double和double2vector两个函数中从优化变量和featuremanager中的变量之间的转换