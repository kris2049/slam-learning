# Level 07: 双目 SLAM

> **目标**: 双目相机: 立体匹配, 基线, 视差→深度, S-PTAM, ORB-SLAM2/3 stereo
> **预计时间**: 2-3 周
> **前置要求**: Level 2-3

## 核心系统
S-PTAM, ORB-SLAM2 Stereo, LDSO, Stereo DSO

## 学习资源
- 查阅 [visual-slam-roadmap](https://github.com/changh95/visual-slam-roadmap) 对应级别的论文列表
- arXiv 搜索具体系统名获取最新论文

## 学习方式
本级别为高级专题，建议:
1. 先读对应级别的核心论文 (1-2篇)
2. 运行开源代码 (GitHub)
3. 在标准数据集上评测 (KITTI / EuRoC / TUM)

```bash
cd tests/ && /usr/bin/python3 test_07_concepts.py
```
