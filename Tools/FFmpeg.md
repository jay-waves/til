## 音视频格式

音频编码:
- AAC, 兼容广泛, MP4 容器的默认音频编码, MP3 升级.
- Opus, 有损压缩. 低延迟 + 实时通信.
- MP3 (MPEG-1 Audio Layer III), 通用格式. 
- FLAC, 无损压缩. 开源. 文件比 WAV 小 50% 左右.
- AC3, 多音道
- WAV. 无损压缩. 

视频编码:
- MPEG-2, 过时格式.
- H.264 (AVC): 广泛使用. 有损压缩.
- HEVC (H.265). 有损压缩, 比 H.264 小 50%.
- AV1. 开源 (一般指没有专利版权). 压缩率接近 H.265, 用于 YouTube 等流媒体.
- VP9. 谷歌为浏览器开发. 效果类似 AV1

图像编码格式: 现代格式的编解码性能要求高 (需要更新的硬件), 但压缩效率和画质更好.
- JPEG. 有损压缩 + DCT + 最高 RGB24b + 灰度 8b + sRGB 色域 + SDR (标准动态范围)
	- JPEG XT 支持 最高 RGB36b, 但由于版权问题, 兼容性差.
- PNG. 无损压缩 + 最高 RGBA64b (16 * 4) + 灰度 16b + sRGB 色域 + SDR.
- GIF. 无损压缩 + 固定 RGB8b + 动画 (多帧序列) + sRGB 色域 + SDR.
- HEIF. 基于现代 H.265 技术. 有损压缩 + RGBA40b (10 * 4) + 宽色域 (sRGB, DCI-P3, Rec.2020) + HDR + 动画 (HEIC 格式)
- AVIF. 基于现代 AV1 技术. 有损压缩 + 最高 RGBA48b (12 * 4) + 宽色域 (sRGB, DCI-P3, Rec.2020) + HDR + 
- WebP. 谷歌为浏览器开发, 压缩率超过 JPEG. 有损/无损压缩 + RGBA32b (8 * 4) + 动画 + 宽色域 (sRGB, DCI-P3) + SDR 

| 格式    | 压缩     | 色深                       | 灰度色深 | 色域                              | 动画支持    | 动态范围 | 描述            |
| ------- | -------- | -------------------------- | -------- | --------------------------------- | --- | -------- | --------------- |
| JPEG    | 有损压缩 |                            |          |                                   |     |          | 基于 DCT        |
| JPEG XT |          |                            |          |                                   |     |          |                 |
| PNG     |          |                            |          |                                   |     |          |                 |
| GIF     |          |                            |          |                                   |     |          |                 |
| HEIF    |          |                            |          |                                   |     |          | 基于 H.265 技术 |
| AVIF    |          |                            |          |                                   |     |          |                 |
| WebP    |          |                            |          |                                   |     |          |                 |
| TIFF    |          | RGBA/CMYK 32b * 4 (含浮点) |          | sRGB, Adobe RGB, DCI-P3, Rec.2020 |     | SDR      |                 |

封装格式:
- MP4, 通用容器, 兼容性最强. H.264/H.265 + AAC.
- MKV, 开源容器, 灵活性更高, 结构较复杂 (兼容性较低).
- AVI, 微软开发的早期容器, 结构简单 (兼容性好), 文件体积大.
- WebM. 互联网流媒体格式, 一般用于 VP9/Opus 视频. 
- TS (MPEG Transport Stream), 用于广播和流传输, 容错性强 (丢包仍可以播放)

滤镜 (Filters):
- Detelecine: 逆转电视电影化 (24fps --> 30fps)
- Interlace Detection (隔行检测): 用于分析视频是否为隔行扫描格式.
- Deinterlace (去隔行): 将隔行扫描转化为逐行扫描, 消除画面"锯齿". 效果 Yadif < Decomb < BWDIF 
- Denoise (降噪): 减少噪点和颗粒感. 效果 Chroma Smooth (仅减少色彩噪点) < HQDN3D < NLMeans 
- Shaerpen (锐化): 增加边缘清晰度. 效果 Unsharp < LapShap 
- Deblock (去块): 减少或消除视频压缩中的*块状伪影 (block artifacts)*. 一些有损压缩格式, 将视频按矩阵 (块) 编码, 导致块边界出现马赛克效果.
- Colourspace (色彩空间转换): 
	- *色彩空间*指颜色存储方式 (RGB -> 显示器用, YUV -> 视频压缩用).
	- *色域*决定了颜色返回 (sRGB, DCI-P3, Rec.2020)
	- *动态范围*影响亮度表现 (SDR, HDR)
	- *伽马曲线*控制亮度色彩映射. (BT.709, PQ)
- Rotation/Cropping (旋转/裁剪)
- Color Adjustment (调色) 调整*色调 (Hue)*, 如白平衡错误, 调整色彩*饱和度 (Saturation)*
- Lenscorrection (镜头校正, 镜头校正). 修复镜头畸变等.

## ffmpeg 操作

```sh
# -i 输入的文件流
ffmpeg -i hello.mp4
ffmpeg -i in.mp4 out.avi

# -b:
# -b:v 设定视频的码率
# -b:a 设定音频的码率

# -c: -codec  编解码器
# -c:v 视频编解码器
	# libx264 H.264 编码器, 编码 MP4
# -c:a aac 音频编解码器
ffmpeg -i in.avi -c:v libx264 out.mp4

# -r 30 指定视频帧率为 30fps
# -s 1280x720 指定输出视频的分辨率
# -vf 视频滤镜, 如亮度/对比度/剪切
```

剪辑从第 30 秒开始的 10 秒.
- `ss` 指定输入文件从某个时间开始处理
```sh
ffmpeg -i in.mp4 -ss 00:00:30 -t 10 out.mp4
```

合并多个视频:
```sh
ffmpeg -f concat -i filelist.txt -c copy out.mp4
```

仅保留视频中的音频:
- `-vn` video no, 去掉视频流. 同理 `-an` 指删除音频流.
- `-acodec copy` 编码器不重新编码, 直接复制. 此时, 使用 mp4 音频容器格式 `.m4a`, 就是无损的.
```sh
ffmpeg -i "xxx.mp4" -vn -acodec copy "xxx.m4a"
```

提取音频的另一种方法:
- `-q`, qscale, 指定编码质量. 质量因子越小质量越高.
- `q:a 0` 指定视频质量因子为 `0`, 因此该方法有损.
```sh
ffmepg -i in.mp4 -q:a 0 -map a out.mp3
```