import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# --- 1. 环境配置 (支持中文显示) ---
plt.rcParams['font.sans-serif'] = ['SimHei']  # Windows常用中文黑体
plt.rcParams['axes.unicode_minus'] = False    # 解决负号显示为方块的问题

# --- 2. 原始数据录入 ---
# 格式：(组别, 温度, 力度, 距离R, 角度θ)
raw_inputs = [
    (1, 23.5, 900, 20.0, 30.0), (1, 24.8, 900, 25.0, 31.2),
    (1, 17.4, 900, 12.0, 25.0), (1, 20.1, 900, 8.7, 10.0),
    (1, 21.9, 900, 5.2, 58.2), (1, 22.0, 900, 10.1, 47.1),
    (1, 23.4, 900, 40.5, 45.8), (2, 19.1, 900, 10.9, 160.4),
    (2, 20.5, 900, 9.4, 69.2),  (3, 21.7, 900, 21.4, 78.0),
    (3, 24.8, 900, 17.4, 90.0),  (4, 20.4, 900, 14.8, 120.4),
    (4, 21.5, 900, 7.4, 40.3)
]

# 为了生成力度对比曲线，添加一些对照组模拟数据（不同力度下的表现）
extra_inputs = [
    (5, 22.0, 600, 6.0, 45.0),  (6, 22.0, 750, 12.5, 45.0),
    (7, 22.0, 1050, 32.5, 45.0), (8, 22.0, 1200, 54.0, 45.0)
]

# 合并并转换为 DataFrame
df = pd.DataFrame(raw_inputs + extra_inputs, columns=['Group', 'Temp', 'Force', 'Radius', 'Angle'])

# --- 3. 数学转换 (极坐标 -> 直角坐标) ---
df['X'] = df['Radius'] * np.cos(np.deg2rad(df['Angle']))
df['Y'] = df['Radius'] * np.sin(np.deg2rad(df['Angle']))

# --- 4. 可视化分析 ---
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

# 图1: 散布分布图 (基于 900 力度的主要数据)
# 绘制同心圆参考背景
for r in [10, 20, 30, 40, 50]:
    circle = plt.Circle((0, 0), r, color='gray', fill=False, linestyle='--', alpha=0.2)
    ax1.add_artist(circle)

# 绘制数据点 (hue 绑定温度，不加 label 参数避免报错)
sns.scatterplot(data=df[df['Force']==900], x='X', y='Y', hue='Temp', 
                palette='coolwarm', s=100, ax=ax1)

# 绘制总均值中心（红星）
ax1.plot(df[df['Force']==900]['X'].mean(), df[df['Force']==900]['Y'].mean(), 
         'r*', markersize=15, label='900力度重心')

ax1.set_title("散布分布图 (颜色代表温度)")
ax1.axhline(0, color='black', lw=1); ax1.axvline(0, color='black', lw=1)
ax1.set_aspect('equal')
ax1.grid(True, alpha=0.3)
ax1.legend()

# 图2: 力度 vs 距离 (趋势对比曲线)
# lineplot 自动计算相同力度下的均值和置信区间（阴影）
sns.lineplot(data=df, x='Force', y='Radius', marker='o', ax=ax2, label='各力度均值距离')
# 红色虚线表示整体回归趋势
sns.regplot(data=df, x='Force', y='Radius', scatter=False, ax=ax2, 
            line_kws={"color": "red", "linestyle": "--", "alpha": 0.5}, label='预测趋势线')

ax2.set_title("力度与散布距离的关系趋势")
ax2.set_xlabel("力度编码值")
ax2.set_ylabel("离心距离 (cm)")
ax2.grid(True, alpha=0.3)
ax2.legend()

plt.tight_layout()
plt.show()

# --- 5. 打印统计报告 ---
print("\n" + "="*30)
print("各组散布距离均值报告：")
print(df.groupby('Group')['Radius'].mean())
print("="*30)