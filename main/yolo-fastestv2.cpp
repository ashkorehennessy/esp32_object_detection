#include <math.h>
#include <algorithm>
#include "yolo-fastestv2.h"
#include <esp_log.h>
#include <stdio.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "ncnn/net.h"
#include "ncnn/cpu.h"
#include "yolo_fastestv2_224-opt.mem.h"
extern camera_fb_t *pic;

//模型的参数配置
yoloFastestv2::yoloFastestv2()
{
    // printf("Creat yoloFastestv2 Detector...\n");
    //输出节点数
    numOutput = 2;
    //推理线程数
    numThreads = 1;
    //anchor num
    numAnchor = 3;
    //类别数目
    numCategory = 1;
    //NMS阈值
    nmsThresh = 0.1f;

    //模型输入尺寸大小
    inputWidth = 224;
    inputHeight = 224;

    //模型输入输出节点名称
    inputName = 0; // const int BLOB_input_1 = 0;
    outputName1 = 162; //22x22，const int BLOB_770 = 162;
    outputName2 = 164; //11x11，const int BLOB_772 = 164;

    net.opt.num_threads = numThreads;
    net.opt.use_sgemm_convolution = true;
    net.opt.use_local_pool_allocator = false;


    //打印初始化相关信息
    // printf("numThreads:%d\n", numThreads);
    // printf("inputWidth:%d inputHeight:%d\n", inputWidth, inputHeight);
    ESP_LOGI("yoloFastestv2", "numThreads:%d", numThreads);
    ESP_LOGI("yoloFastestv2", "inputWidth:%d inputHeight:%d", inputWidth, inputHeight);

    //anchor box w h
    std::vector<float> bias {18.85f,24.07f, 26.19f,32.52f, 33.93f,44.26f, 44.36f,53.52f, 59.13f,68.30f, 103.13f,132.61f};

    anchor.assign(bias.begin(), bias.end());
}

yoloFastestv2::~yoloFastestv2()
{
    // printf("Destroy yoloFastestv2 Detector...\n");
    ESP_LOGI("yoloFastestv2", "Destroy yoloFastestv2 Detector...");
}

//ncnn 模型加载
int yoloFastestv2::loadModel(const unsigned char* paramPath, const unsigned char* binPath)
{
    // printf("Ncnn mode init:\n%s\n%s\n", paramPath, binPath);
    ESP_LOGI("yoloFastestv2", "Ncnn model init");

    net.load_param(paramPath);
    net.load_model(binPath);

    // printf("Ncnn model init sucess...\n");
    ESP_LOGI("yoloFastestv2", "Ncnn model init sucess...");

    return 0;
}

float intersection_area(const TargetBox &a, const TargetBox &b)
{
    if (a.x1 > b.x2 || a.x2 < b.x1 || a.y1 > b.y2 || a.y2 < b.y1)
    {
        // no intersection
        return 0.f;
    }

    float inter_width = std::min(a.x2, b.x2) - std::max(a.x1, b.x1);
    float inter_height = std::min(a.y2, b.y2) - std::max(a.y1, b.y1);

    return inter_width * inter_height;
}

bool scoreSort(TargetBox a, TargetBox b)
{
    return (a.score > b.score);
}

//NMS处理
int yoloFastestv2::nmsHandle(std::vector<TargetBox> &tmpBoxes,
                             std::vector<TargetBox> &dstBoxes)
{
    std::vector<int> picked;

    sort(tmpBoxes.begin(), tmpBoxes.end(), scoreSort);

    for (int i = 0; i < tmpBoxes.size(); i++) {
        int keep = 1;
        for (int j = 0; j < picked.size(); j++) {
            //交集
            float inter_area = intersection_area(tmpBoxes[i], tmpBoxes[picked[j]]);
            //并集
            float union_area = tmpBoxes[i].area() + tmpBoxes[picked[j]].area() - inter_area;
            float IoU = inter_area / union_area;

            if(IoU > nmsThresh && tmpBoxes[i].cate == tmpBoxes[picked[j]].cate) {
                keep = 0;
                break;
            }
        }

        if (keep) {
            picked.push_back(i);
        }
    }

    for (int i = 0; i < picked.size(); i++) {
        dstBoxes.push_back(tmpBoxes[picked[i]]);
    }

    return 0;
}

//检测类别分数处理
int yoloFastestv2::getCategory(const float *values, int index, int &category, float &score)
{
    float tmp = 0;
    float objScore  = values[4 * numAnchor + index];

    for (int i = 0; i < numCategory; i++) {
        float clsScore = values[4 * numAnchor + numAnchor + i];
        clsScore *= objScore;

        if(clsScore > tmp) {
            score = clsScore;
            category = i;

            tmp = clsScore;
        }
    }

    return 0;
}

//特征图后处理
int yoloFastestv2::predHandle(const ncnn::Mat *out, std::vector<TargetBox> &dstBoxes,
                              const float scaleW, const float scaleH, const float thresh)
{    //do result
    for (int i = 0; i < numOutput; i++) {
        int stride;
        int outW, outH, outC;

        outH = out[i].c;
        outW = out[i].h;
        outC = out[i].w;

        assert(inputHeight / outH == inputWidth / outW);
        stride = inputHeight / outH;

        for (int h = 0; h < outH; h++) {
            const float* values = out[i].channel(h);

            for (int w = 0; w < outW; w++) {
                for (int b = 0; b < numAnchor; b++) {
                    //float objScore = values[4 * numAnchor + b];
                    TargetBox tmpBox;
                    int category = -1;
                    float score = -1;

                    getCategory(values, b, category, score);

                    if (score > thresh) {
                        float bcx, bcy, bw, bh;

                        bcx = ((values[b * 4 + 0] * 2. - 0.5f) + w) * stride;
                        bcy = ((values[b * 4 + 1] * 2. - 0.5f) + h) * stride;
                        bw = pow((values[b * 4 + 2] * 2.), 2) * anchor[(i * numAnchor * 2) + b * 2 + 0];
                        bh = pow((values[b * 4 + 3] * 2.), 2) * anchor[(i * numAnchor * 2) + b * 2 + 1];

                        tmpBox.x1 = (bcx - 0.5f * bw) * scaleW;
                        tmpBox.y1 = (bcy - 0.5f * bh) * scaleH;
                        tmpBox.x2 = (bcx + 0.5f * bw) * scaleW;
                        tmpBox.y2 = (bcy + 0.5f * bh) * scaleH;
                        tmpBox.score = score;
                        tmpBox.cate = category;

                        dstBoxes.push_back(tmpBox);
                    }
                }
                values += outC;
            }
        }
    }
    return 0;
}

int yoloFastestv2::detection(const camera_fb_t * srcImg, std::vector<TargetBox> &dstBoxes, const float thresh)
{
    dstBoxes.clear();

    float scaleW = (float)srcImg->width / (float)inputWidth;
    float scaleH = (float)srcImg->height / (float)inputHeight;
    ESP_LOGI("detection", "scaleW:%f scaleH:%f", scaleW, scaleH);
    //resize of input image data
    ncnn::Mat inputImg = ncnn::Mat::from_pixels_resize(srcImg->buf, ncnn::Mat::PIXEL_BGR,\
                                                       scaleW, scaleH, inputWidth, inputHeight);
    ESP_LOGI("detection", "inputImg:%d %d %d", inputImg.w, inputImg.h, inputImg.c);
    //Normalization of input image data, RGB888 format
    // const float mean_vals[3] = {0.f, 0.f, 0.f};
    // const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};

    //Normalization of input image data, RGB565 format
    const float mean_vals[3] = {0.f, 0.f, 0.f};
    const float norm_vals[3] = {1/31.f, 1/63.f, 1/31.f};

    inputImg.substract_mean_normalize(mean_vals, norm_vals);
    ESP_LOGI("detection", "mean_normalize");
    //creat extractor
    ncnn::Extractor ex = net.create_extractor();
    ESP_LOGI("detection", "create_extractor");
    //set input tensor
    ex.input(inputName, inputImg);
    ESP_LOGI("detection", "set input");
    //forward
    ex.extract(outputName1, out[0],1); //22x22
    ESP_LOGI("detection", "extract1");
    ex.extract(outputName2, out[1],1); //11x11
    ESP_LOGI("detection", "extract2");
    std::vector<TargetBox> tmpBoxes;
    //特征图后处理
    predHandle(out, tmpBoxes, scaleW, scaleH, thresh);
    ESP_LOGI("detection", "predHandle");
    //NMS
    nmsHandle(tmpBoxes, dstBoxes);
    ESP_LOGI("detection", "nmsHandle");
    return 0;
}


extern "C" void yolo_inference(void *arg){
    yoloFastestv2 api;
    api.loadModel(yolo_fastestv2_224_opt_param_bin, yolo_fastestv2_224_opt_bin);

    while (true) {
        // 检测
        std::vector<TargetBox> boxes;
        if(!pic){
            ESP_LOGE("YOLO", "Frame buffer could not be acquired");
            vTaskDelay(1);
            continue;
        }
        int64_t start = esp_timer_get_time();
        api.detection(pic, boxes);
        int64_t end = esp_timer_get_time();
        ESP_LOGI("YOLO", "inference time: %lld ms", (end - start) / 1000);
        for (const TargetBox& box : boxes) {
            ESP_LOGI("YOLO", "category: %s, score: %f, x1: %d, y1: %d, x2: %d, y2: %d",
                "face", box.score, box.x1, box.y1, box.x2, box.y2);
        }
        vTaskDelay(1);
    }
}
