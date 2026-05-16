2026.5.16 测试方法：打开6个终端，其中5个依次 source /opt/tros/humble.setup.bash，
另一个 
cd rdk_ws 
source install/setup.bash
ros2 run camera_detector camera_detector_node

剩下5个：

ros2 launch hobot_usb_cam hobot_usb_cam.launch.py usb_video_device:=/dev/video8 usb_pixel_format:=yuyv2rgb usb_image_width:=640 usb_image_height:=480

ros2 launch hobot_codec hobot_codec_encode.launch.py codec_in_mode:=ros codec_in_format:=rgb8 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/image codec_pub_topic:=/jpeg_img

ros2 launch hobot_codec hobot_codec_encode.launch.py codec_in_mode:=ros codec_in_format:=jpeg codec_out_mode:=ros codec_out_format:=nv12 codec_sub_topic:=/jpeg_img codec_pub_topic:=/nv12_img

ros2 launch hobot_codec hobot_codec_encode.launch.py codec_in_mode:=ros codec_in_format:=bgr8 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/detection_img codec_pub_topic:=/detection_jpeg_img

ros2 launch websocket websocket.launch.py websocket_image_topic:=/detection_jpeg_img websocket_only_show_image:=true

今天能看到 web上有图像，但是没有识别到东西，可能
1.第3条命令是这样的：
[hobot_codec_republish-1] [WARN] [1778943326.858569883] [hobot_codec_encoder_26bcfc89]: Pub img fps [30.06]
[hobot_codec_republish-1] [WARN] [1778943331.877136255] [hobot_codec_encoder_26bcfc89]: Pub img fps [30.09]
......
只有pub没有sub
但是之前
ros2 launch hobot_codec hobot_codec_encode.launch.py codec_in_mode:=shared_mem codec_in_format:=jpeg codec_out_mode:=ros codec_out_format:=nv12 codec_sub_topic:=/tmp_img codec_pub_topic:=/hbmem_img
这是旧命令，它有pub和sub
[hobot_codec_republish-1] [WARN] [1778941974.568665141] [hobot_codec_encoder_fc055384]: sub jpeg 640x480, fps: 33.4975, pub nv12, fps: 33.4975, comm delay [495.3235]ms, codec delay [2.8824]ms
[hobot_codec_republish-1] [WARN] [1778941978.460594763] [hobot_codec_encoder_fc055384]: Pub img fps [30.07]

并且[官方教程这个地方](https://developer.d-robotics.cc/rdk_doc/Robot_development/tros_dev/ai_predict#1-%E5%88%9B%E5%BB%BApackage)，虽然是rdkx3，但是用的是hbm_img_msgs::msg::HbmMsg1080P这个消息类型，所以可以尝试改类型为 shared_mem

2.同样是官方教程，它用的是hbm的消息类型，说不定有帮助