# 移植记录

移植来自edge impulse的FOMO物体检测模型(mobilenetv2 0.1)到esp32s3，输入96x96灰度图，在arduino框架下推理一帧耗时116ms，但是在esp-idf框架下推理一帧耗时2067ms，暂不清楚原因。

这是esp-idf版本。
