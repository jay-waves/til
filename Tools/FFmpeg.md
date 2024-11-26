

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