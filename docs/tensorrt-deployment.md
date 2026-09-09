# YOLOv5 / TensorRT 部署与性能测量

## 模型系列必须匹配

原版 `ultralytics/yolov5` 训练的权重使用 `model_family: yolov5`，由本地 YOLOv5 的 Hub/AutoShape 加载 `.pt` 或 `.engine`。这条路径符合 [YOLOv5 官方导出与加载说明](https://docs.ultralytics.com/yolov5/tutorials/model-export/)。

通过新版 `ultralytics` 包训练的兼容模型使用 `model_family: ultralytics`，采用其 [TensorRT 导出接口](https://docs.ultralytics.com/integrations/tensorrt/)。不要仅凭文件名将原版 YOLOv5 权重交给新版 `YOLO()`。

## 目标环境

在实际运行推理的 Jetson 上导出引擎，匹配 JetPack、CUDA、TensorRT 和 PyTorch。引擎不能视为跨 GPU/跨 TensorRT 版本的通用文件。使用与模型训练相匹配的、已检查的 YOLOv5 版本，并记录其 Git 提交；加载器不自动下载远程代码。

模型权重单独配置；以下步骤使用本地训练权重进行导出和部署。

## 导出 FP16 引擎

准备本地 YOLOv5 仓库及依赖后，在本项目根目录执行：

```bash
python tools/export_tensorrt.py \
  --weights /absolute/path/best.pt \
  --family yolov5 --yolov5-repo ~/yolov5 --imgsz 640 --device 0
```

默认 FP16、batch 1，输出与权重同目录的 `best.engine`。如需 FP32，添加 `--fp32`。输出已存在时工具拒绝覆盖，避免误把旧引擎当成新导出。新版 Ultralytics 模型改用 `--family ultralytics`。

修改自己的 `robot.yaml`：

```yaml
yolo_detector:
  ros__parameters:
    model_path: /absolute/path/best.engine
    model_family: yolov5
    yolov5_repo: /absolute/path/yolov5
    device: '0'
    imgsz: 640
    display: false
```

`imgsz` 必须匹配导出设置。模型类别应与 `dry_classes` / `wet_classes` 对应；未知类别会被拒绝抓取。

## 实测与记录

准备包含足够连续帧的代表性视频，在目标硬件执行：

```bash
python tools/benchmark_detection.py \
  --model /absolute/path/best.engine --video /absolute/path/test.mp4 \
  --family yolov5 --yolov5-repo ~/yolov5 --warmup 30 --frames 300 \
  --output output/benchmark.json
```

工具逐帧读取视频，执行预处理、推理、后处理与 CPU 结果获取，在 CUDA 同步后计时。输出 JSON 汇总及逐帧 CSV，包括模型哈希、环境版本、平均 FPS、P50/P95/P99、最大延迟、33.33 ms 内帧比例。

该指标覆盖检测处理，不包括视频解码、相机等待、ROS 传输和显示。实时链路可用 `ros2 topic hz /detection_boxes` 观察发布频率，配合帧时间戳记录延迟；抓取期间检测会暂停，纯检测阶段与完整任务阶段分别统计。

TensorRT 前后还应在相同验证集比较检测精度，防止只提升速度却损失分类效果。
