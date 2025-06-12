        # In your_ros_package/msg/SingleDetection.msg
        int32 x1
        int32 y1
        int32 x2
        int32 y2
        float64 intensity  # 或者 int64, 具体取决于 pixel_sum 的数据类型和范围
        int32 id