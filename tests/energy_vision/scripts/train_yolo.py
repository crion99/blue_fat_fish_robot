#!/usr/bin/env python3
"""训练检测“待激活/已激活”靶板的小 YOLO（需要 pip install ultralytics）。

数据目录结构（用 labelImg / Roboflow 导出 YOLO 格式）：
    yolo/
      data.yaml
      images/train/*.jpg
      images/val/*.jpg
      labels/train/*.txt      # 每行: cls cx cy w h（归一化）
      labels/val/*.txt

用法：
    python3 scripts/train_yolo.py --data yolo/data.yaml --epochs 80
"""
import argparse

from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', default='yolo/data.yaml')
    ap.add_argument('--model', default='yolov8n.pt')
    ap.add_argument('--epochs', type=int, default=80)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--batch', type=int, default=8)
    ap.add_argument('--name', default='rune')
    args = ap.parse_args()

    model = YOLO(args.model)
    model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        project='yolo/runs',
        name=args.name,
        # 小数据集：开强增强，对付角度/亮度变化
        degrees=180.0,
        fliplr=0.5,
        flipud=0.0,
        hsv_h=0.02,
        hsv_s=0.5,
        hsv_v=0.5,
        scale=0.5,
        translate=0.2,
        mosaic=1.0,
    )
    print('最佳权重: yolo/runs/%s/weights/best.pt' % args.name)


if __name__ == '__main__':
    main()
