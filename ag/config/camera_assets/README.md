把你的相机标定文件放在这里，默认文件名建议：

```text
camera_calibration.npz
```

要求至少包含：

- `camera_matrix`
- `dist_coeffs`

如果你不放标定文件，GUI 仍可显示 RTSP 原始图像，只是 BEV 投影会退化为普通背景图层。
