# Level 5 教材: 深度学习 + SLAM

## A. 深度学习前端 (Perception)

### 特征检测与匹配
- **SuperPoint** (2018): 自监督关键点+描述子，Homographic Adaptation
- **SuperGlue** (2020): GNN + Attention + Sinkhorn 最优传输匹配
- **LightGlue** (2023): 自适应深度，5-10x 加速
- **LoFTR** (2021): Detector-free 稠密匹配，Transformer 粗到细

### 深度估计
- **MiDaS** (2020): 多数据集混合训练，相对深度
- **DPT** (2021): ViT 架构，全局上下文
- **ZoeDepth** (2023): 零样本度量深度
- **Depth Anything V2** (2024): 大规模基础模型

### 光流
- **RAFT** (2020, ECCV Best Paper): All-Pairs Correlation + ConvGRU迭代
- **FlowFormer** (2022): Transformer on cost volume

### 位姿回归与重定位
- **PoseNet** (2015): CNN 直接回归 6-DoF 位姿
- **ACE** (2023): 加速坐标编码，每场景5分钟训练
- **hloc**: NetVLAD(粗) → SuperGlue(细) 层次定位

## B. 深度学习后端 (Optimization)

- **BA-Net** (2019): 可微分 LM 层
- **DROID-SLAM** (2021): 可微分 BA + 稠密光流
- **Theseus** (Meta, 2022): 可微分非线性优化库
- **SE-Sync** (2019): 可验证的位姿图优化 (SDP+黎曼优化)

## C. 端到端系统

- **DeepVO** (2017): 监督 VO
- **SfM-Learner** (2017): 无监督深度+位姿
- **TANDEM** (2021): 实时跟踪+稠密建图
- **DPVO** (2023): 补丁级 DROID, 实时 30+ FPS

## D. 场景理解

- **SAM** (2023): 提示式分割基础模型
- **Grounding DINO**: 文本提示检测 → Grounded SAM 流水线
- **ConceptFusion** (2023): CLIP 特征融合到 3D 地图
- **ConceptGraphs** (2023): SAM + CLIP + LLM 3D 场景图
- **Clio** (2024, MIT): 任务驱动的场景图
- **Hydra** (2022, MIT): 实时层次化 3D 场景图
