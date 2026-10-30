# 大鱼写字机 Z 轴升降机械爪工程

保存日期：2026-10-30。当前阶段：**布局预览，尚未制造定版**。

## 当前方案
- 保留齿轮齿条升降和槽形导向，不使用 Z 轴光轴。
- 下部替换为用户提供的齿轮连杆夹爪，预览机构按25%缩放；圆爪保持原尺寸，开孔长柄朝上。
- 升降齿轮暂定模数2、18齿、节圆直径36 mm、外径40 mm。
- 假定150度舵机可用转角，理论位移47.12 mm；这不是已验证有效行程。
- 当前静态预览总高263.15 mm。安装孔、连接件、舵机接口和运动干涉仍待校核。

## 查看与恢复
GitHub连接器仅支持文本提交，本机Git直连不可用，因此模型、原生SolidWorks文件及PNG采用**无损ZIP的Base64分片**保存。文件没有丢弃，也不是可直接改后缀使用的模型。

1. 在仓库页面选择 Code → Download ZIP，完整解压。
2. 在此文件夹运行 `python restore.py`（Python 3.9以上，无额外依赖）。
3. 生成 `restored_project`，自动验证全部100个文件的SHA-256。
4. 打开 `restored_project/preview_revision/layout_perspective.png` 和 `claw_detail.png` 查看本次预览。

`source/`、`docs/` 与参数文件可直接在GitHub阅读；`archive/`保存完整快照分片；`MANIFEST.json`列出每个文件的大小与校验值。

## 完整快照内容
- `preview_revision/`：最新预览图及参数。
- `outputs/`：此前80 mm行程、双齿条夹爪版本的STL与校验记录。**这是旧方案，有已知干涉，不是最新预览的制造模型。**
- `reference/original_gripper/`：用户提供的新夹爪SLDASM、SLDPRT及参考JPG。
- `reference/new_gripper/`：从该装配提取的组件STL及变换记录。
- `reference/圆爪.STL`、`圆爪.SLDPRT`：用户圆爪。
- `source/`：旧版生成器与独立预览生成器。仓库快照中的路径已改为项目相对路径。
- 中文交接文件、装配说明、BOM及历史状态文档。旧文档由最新交接文件中的说明取代。

## 重新生成预览
恢复后，在 `restored_project` 中使用Node.js 22或更新版本执行 `npm install`、`npm run preview`。`npm run build`生成的是旧版，不能用来生成新夹爪制造定版。

本次未重新进行运动、强度、夹持力及实物舵机验证。`source/export_donor.ps1`是历史实验脚本，存在活动文档导出及锁文件遍历问题，不要直接运行。网上获取的参考模型保留原来源属性，本仓库不为其额外授予许可证。

本快照不含node_modules、临时锁文件、重复组件导出、39 MB演示GIF或整个E盘零件库；包含本次工程所需模型和生成输入。
