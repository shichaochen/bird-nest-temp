# 伪蛋参数化模型 v1 -- 椭球近似, 长轴沿 Z
# 用法: ~/workspace/venvs/cad/bin/python build_egg.py [--length 32 --width 23]
# 量好真蛋再改数: L=蛋长, W=蛋最大直径, WALL=壳厚 1.2mm
# 输出: egg_bottom.stl (装电子件的下半杯) + egg_top.stl (盖)
# 打印: PETG 或树脂, 0.2mm 层高, 15% 填充; 壳内侧涂环氧密封, 合缝处环氧封死, 否则不防水
# 注意: 这是椭球近似, 真蛋一头钝一头尖; 要更像真蛋, 下一版用回转轮廓线做不对称蛋形
import argparse, pathlib
import numpy as np
from manifold3d import Manifold

def mesh_to_trimesh(m):
    import trimesh
    mesh = m.to_mesh()
    verts = np.asarray(mesh.vert_properties[:, :3], dtype=float)
    faces = np.asarray(mesh.tri_verts, dtype=np.int64)
    return trimesh.Trimesh(vertices=verts, faces=faces, process=False)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--length", type=float, default=32.0, help="蛋长 mm")
    ap.add_argument("--width", type=float, default=23.0, help="蛋最大直径 mm")
    ap.add_argument("--wall", type=float, default=1.2, help="壳厚 mm")
    ap.add_argument("--split", type=float, default=5.0, help="分模面距中心高度 mm, 正值=盖小杯大")
    ap.add_argument("--out", type=str, default=".")
    a = ap.parse_args()
    out = pathlib.Path(a.out); out.mkdir(parents=True, exist_ok=True)
    L, W, WALL = a.length, a.width, a.wall
    outer = Manifold.sphere(1.0, 96).scale([W/2, W/2, L/2])
    inner = Manifold.sphere(1.0, 96).scale([(W-2*WALL)/2, (W-2*WALL)/2, (L-2*WALL)/2])
    shell = outer - inner
    # 分模: trim_by_plane(normal, offset) 返回法向一侧
    top = shell.trim_by_plane([0, 0, 1], a.split)
    bottom = shell.trim_by_plane([0, 0, -1], -a.split)
    for name, part in [("egg_top", top), ("egg_bottom", bottom)]:
        tm = mesh_to_trimesh(part)
        print(name, "watertight:", tm.is_watertight, "bbox mm:", np.round(tm.bounds[1]-tm.bounds[0], 2), "volume mm3:", round(float(tm.volume), 1))
        assert tm.is_watertight, name + " 不水密, 不发货"
        tm.export(str(out / (name + ".stl")))
    print(f"OK: L={L} W={W} wall={WALL} split_z={a.split} -> {out}/egg_top.stl + egg_bottom.stl")

if __name__ == "__main__":
    main()
