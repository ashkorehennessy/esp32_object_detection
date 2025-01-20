# 移植记录
将[Yolo-FastestV2](https://github.com/dog-qiuqiu/Yolo-FastestV2)模型利用[ncnn](https://github.com/nihui/ncnn_on_esp32)框架部署到esp32s3，在240x240输入下推理一帧RGB888需要6500ms，在96x96输入下推理一帧RGB888需要1000ms。
