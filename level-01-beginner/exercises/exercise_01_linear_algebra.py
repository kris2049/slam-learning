"""
练习 1: 线性代数 (交互式)
=============================
脚本会逐个出题，你输入代码，当场判对错。
"""

import numpy as np
import sys

correct = 0
total = 0

def ask(task_name, prompt_text, check_fn, hint, setup_code=""):
    """出一道题，等用户输入代码，执行后检查结果"""
    global correct, total
    total += 1

    print(f"\n{'─'*50}")
    print(f"  {task_name}")
    print(f"{'─'*50}")
    print(f"  {prompt_text}")
    print()

    # 准备环境
    namespace = {"np": np, "__builtins__": {}}

    if setup_code:
        for line in setup_code.strip().split("\n"):
            line = line.strip()
            if line and not line.startswith("#"):
                try:
                    exec(line, namespace)
                except Exception as e:
                    pass  # 忽略 setup 中的错误

    attempts = 0
    while attempts < 5:
        attempts += 1
        try:
            user_input = input("  >>> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n  已退出。")
            sys.exit(0)

        if user_input.lower() in ("hint", "提示", "?"):
            print(f"  💡 {hint}")
            continue

        # 在命名空间中执行用户代码
        try:
            exec(user_input, namespace)
        except Exception as e:
            print(f"  ❌ 执行错误: {e}")
            continue

        # 检查结果
        try:
            ok, msg = check_fn(namespace)
            if ok:
                correct += 1
                print(f"  ✅ {msg}")
                return True
            else:
                print(f"  ❌ {msg}")
                if attempts >= 2:
                    print(f"  💡 {hint}")
        except Exception as e:
            print(f"  ❌ 结果检查失败: {e}")
            print(f"  💡 {hint}")

    print(f"  ⏭ 跳过 (答案: {hint})")
    return False


# ══════════════════════════════════════════
# 任务 1: 向量运算
# ══════════════════════════════════════════

def task_01():
    env = {"np": np, "a": np.array([3., 0., 0.]), "b": np.array([0., 4., 0.])}

    print("\n" + "═"*55)
    print("  任务1: 向量运算")
    print("  a = [3, 0, 0],  b = [0, 4, 0]")
    print("═"*55)

    # 1a: 模长
    print("\n  1a. 计算 a 的 L2 范数，把结果存入变量 n")
    print("      例如: n = np.linalg.norm(a)")
    for _ in range(5):
        try: user = input("  >>> ").strip()
        except: return
        if user in ("hint","?"): print("  💡 n = np.linalg.norm(a)"); continue
        local = dict(env)
        try: exec(user, local)
        except Exception as e: print(f"  ❌ {e}"); continue
        if "n" not in local: print("  ❌ 请定义变量 n"); continue
        if abs(local["n"] - 3.0) < 0.1:
            correct_track(); env["n"] = local["n"]; print("  ✅ |a| = 3.0 正确!"); break
        else: print(f"  ❌ n={local['n']:.2f}, 应该是 3.0")

    # 1b: 点积
    print("\n  1b. 计算 a 和 b 的点积，存入变量 d")
    for _ in range(5):
        try: user = input("  >>> ").strip()
        except: return
        if user in ("hint","?"): print("  💡 d = np.dot(a, b)"); continue
        local = dict(env)
        try: exec(user, local)
        except Exception as e: print(f"  ❌ {e}"); continue
        if "d" not in local: print("  ❌ 请定义变量 d"); continue
        if abs(local["d"]) < 0.01:
            correct_track(); env["d"] = local["d"]; print("  ✅ a·b = 0 (正交) 正确!"); break
        else: print(f"  ❌ d={local['d']:.2f}, 应该是 0")

    # 1c: 叉积
    print("\n  1c. 计算 a 和 b 的叉积，存入变量 c (应该是 [0,0,12])")
    for _ in range(5):
        try: user = input("  >>> ").strip()
        except: return
        if user in ("hint","?"): print("  💡 c = np.cross(a, b)"); continue
        local = dict(env)
        try: exec(user, local)
        except Exception as e: print(f"  ❌ {e}"); continue
        if "c" not in local: print("  ❌ 请定义变量 c"); continue
        if np.allclose(local["c"], [0, 0, 12]):
            correct_track(); print("  ✅ a×b = [0, 0, 12] 正确!"); break
        else: print(f"  ❌ c={local['c']}, 应该是 [0, 0, 12]")


def correct_track():
    global correct; correct += 1


# ══════════════════════════════════════════
# 任务 2: 旋转矩阵
# ══════════════════════════════════════════

def task_02():
    import math
    env = {"np": np, "math": math, "theta": math.radians(30),
           "v": np.array([1.0, 0.0])}

    print("\n" + "═"*55)
    print("  任务2: 旋转矩阵")
    print("  theta = 30° (已转为弧度), v = [1, 0]")
    print("═"*55)

    # 2a: 构造旋转矩阵
    print("\n  2a. 构造 2x2 旋转矩阵 R (绕Z轴转 theta)")
    print("      存为: R = np.array([[cosθ, -sinθ], [sinθ, cosθ]])")
    for _ in range(5):
        try: user = input("  >>> ").strip()
        except: return
        local = dict(env)
        try: exec(user, local)
        except Exception as e: print(f"  ❌ {e}"); continue
        if "R" not in local: print("  ❌ 请定义 R"); continue
        R = local["R"]
        if isinstance(R, np.ndarray) and R.shape == (2,2):
            detR = np.linalg.det(R)
            if abs(detR - 1.0) < 0.01:
                correct_track(); env["R"] = R; print(f"  ✅ det(R)={detR:.4f} ≈ 1 正确!"); break
            else: print(f"  ❌ det(R)={detR:.4f}, 旋转矩阵必须是1")
        else: print("  ❌ R 必须是 2x2 数组")

    # 2b: 旋转向量
    print("\n  2b. 用 R 旋转 v，存到 rotated")
    for _ in range(5):
        try: user = input("  >>> ").strip()
        except: return
        local = dict(env)
        try: exec(user, local)
        except Exception as e: print(f"  ❌ {e}"); continue
        if "rotated" not in local: print("  ❌ 请定义 rotated"); continue
        r = local["rotated"]
        if abs(r[0] - 0.866) < 0.01 and abs(r[1] - 0.5) < 0.01:
            correct_track(); print(f"  ✅ rotated = [{r[0]:.3f}, {r[1]:.3f}] 正确!"); break
        else: print(f"  ❌ rotated={r}, 应≈[0.866, 0.5]. 提示: R @ v")


# ══════════════════════════════════════════
# 任务 3: SVD
# ══════════════════════════════════════════

def task_03():
    A = np.random.RandomState(42).randn(9, 9)
    A[:, -1] = A[:, 0] * 0.5 + A[:, 1] * 0.3
    env = {"np": np, "A": A}

    print("\n" + "═"*55)
    print("  任务3: SVD 解 Ax=0")
    print("  A 是 9x9 秩为8的矩阵，求其零空间向量")
    print("═"*55)

    print("\n  3a. 对 A 做 SVD 分解，取最小奇异值对应的右奇异向量")
    print("      存到 x (应该是9维向量)")
    print("      提示: U,S,Vt = np.linalg.svd(A); x = Vt[-1]")
    for _ in range(5):
        try: user = input("  >>> ").strip()
        except: return
        local = dict(env)
        try: exec(user, local)
        except Exception as e: print(f"  ❌ {e}"); continue
        if "x" not in local: print("  ❌ 请定义 x"); continue
        x = np.asarray(local["x"]).flatten()
        if len(x) != 9: print(f"  ❌ x 应该是9维, 实际{len(x)}维"); continue
        residual = np.linalg.norm(A @ x)
        if residual < 0.001:
            correct_track(); print(f"  ✅ ||Ax|| = {residual:.2e} ≈ 0 正确!"); break
        else: print(f"  ❌ ||Ax||={residual:.2e}, 应≈0. 提示: x=Vt[-1]")


# ══════════════════════════════════════════
# 任务 4: 齐次变换
# ══════════════════════════════════════════

def task_04():
    theta = np.radians(45)
    env = {"np": np, "theta": theta}

    print("\n" + "═"*55)
    print("  任务4: 齐次变换矩阵")
    print("  绕Z轴转45°, 然后平移 [1,2,3]")
    print("═"*55)

    # 4a: 构造 T
    print("\n  4a. 构造 4x4 齐次变换矩阵 T")
    print("      提示: T = np.eye(4); T[:3,:3] = ...; T[:3,3] = [1,2,3]")
    for _ in range(5):
        try: user = input("  >>> ").strip()
        except: return
        local = dict(env)
        try: exec(user, local)
        except Exception as e: print(f"  ❌ {e}"); continue
        if "T" not in local: print("  ❌ 请定义 T"); continue
        T = np.asarray(local["T"])
        if T.shape != (4, 4): print(f"  ❌ T 应该是 4x4"); continue
        # 检查是否是有效的 SE(3)
        R_part = T[:3, :3]
        if abs(np.linalg.det(R_part) - 1.0) > 0.01:
            print(f"  ❌ det(R) ≠ 1. 检查旋转矩阵"); continue
        correct_track(); env["T"] = T; print("  ✅ T 构造正确!"); break

    # 4b: 变换一个点
    print("\n  4b. 用 T 变换点 p=[1,1,1] (齐次坐标 [1,1,1,1]), 存到 pt")
    for _ in range(5):
        try: user = input("  >>> ").strip()
        except: return
        local = dict(env, p=np.array([1,1,1,1.0]))
        try: exec(user, local)
        except Exception as e: print(f"  ❌ {e}"); continue
        if "pt" not in local: print("  ❌ 请定义 pt"); continue
        pt = np.asarray(local["pt"])
        # 手动验证: R=[cos45,-sin45,0; sin45,cos45,0; 0,0,1], t=[1,2,3]
        # p_trans = R@[1,1,1] + t = [0, 1.414, 1] + [1,2,3] = [1, 3.414, 4]
        if abs(pt[0]-1.0)<0.1 and abs(pt[1]-3.414)<0.1 and abs(pt[2]-4.0)<0.1:
            correct_track(); print(f"  ✅ pt = {pt[:3].round(2)} 正确!"); break
        else: print(f"  ❌ pt={pt[:3]}, 应≈[1, 3.414, 4]. 提示: pt = T @ p")


# ══════════════════════════════════════════
# 主流程
# ══════════════════════════════════════════

if __name__ == "__main__":
    print("═"*55)
    print("  SLAM 练习 1: 线性代数")
    print("  输入你的代码，输 hint 看提示，Ctrl+C 退出")
    print("═"*55)

    task_01()
    task_02()
    task_03()
    task_04()

    total_q = 8  # 总共8个小问
    print(f"\n{'═'*55}")
    print(f"  完成: {correct}/{total_q}")
    if correct == total_q:
        print("  🎉 全部正确！进入下一个练习。")
    else:
        print(f"  还有 {total_q-correct} 题，重新运行再试。")
