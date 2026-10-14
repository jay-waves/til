

<img src="../../assets/vision/color-space.svg" alt="wiki">

<img src="../../assets/vision/yuv.webp" alt="wiki https://en.wikipedia.org/wiki/Y%E2%80%B2UV">

* for printing industry, CMYK color is preferred 
* for dispaly, RGB(A) color is wideyl used 
* for imgae compression, YUV color has been predominant


**色彩度量**: 
- RGB ($255\times 255\times 255)$, 加色模式, 需要主动发出三色光.
- CMYK (印刷彩色), 减色模式, 反射部分波长的光. (黑暗中看不见)
- YIG, Y: 颜色透视度(luminance), 灰度值; I: 色调; Q: 饱和度. 可以线性地转化为RGB值. 用于北美广播电视(NTSC).
- YUV, Y: 亮度; UV: 色度 (U蓝色到绿色的色差, V红色到绿色的色差), 人眼对绿色亮度更加敏感 (高于红蓝色), 所以可以压缩色度信息. 被用于欧洲广播电视的视频压缩中.
