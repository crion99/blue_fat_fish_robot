// 轻量绘图：矩形、文字（内置 5×7 点阵字体，支持大写字母/数字/常用符号）。
// 不依赖 OpenCV，标定与自检程序也能直接出带标注的图片。

#pragma once

#include <string>

#include "bff/image.hpp"

namespace bff {

void fillRect(Image& img, const Rect& r, uint8_t b, uint8_t g, uint8_t r8);

void drawRect(Image& img, const Rect& r, uint8_t b, uint8_t g, uint8_t r8, int thickness = 2);

void drawLine(Image& img, int x0, int y0, int x1, int y1, uint8_t b, uint8_t g, uint8_t r8);

// 文字左下角为 (x, y)，scale 为像素放大倍数
void drawText(Image& img, int x, int y, const std::string& text, uint8_t b, uint8_t g, uint8_t r8,
              int scale = 2);

int textWidth(const std::string& text, int scale = 2);
int textHeight(int scale = 2);

}  // namespace bff
