"""离线几何检查与预览；真实端口坐标由MATLAB脚本另行检查。"""
from pathlib import Path
from zipfile import ZipFile
import argparse
import xml.etree.ElementTree as E

from build_checked import TARGETS
from check_checked_model import ResolvedGraph, check_visible_graphs
from model_geometry import Geometry


def main():
    folder = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=Path, default=folder / 'reduce_emf_load_fixed.slx')
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    graph = ResolvedGraph(args.model.read_bytes())
    _, line_count = check_visible_graphs(graph)
    print(f'PASS {len(TARGETS)}个可见图，{line_count}条信号线：无悬空活动端口、斜线、重叠或穿模块')
    print('端点为序列化排布估计；本结果不代表已检查Simulink实际端口坐标。')
    if args.output_dir is None:
        return

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with ZipFile(args.model) as archive:
        for ref in TARGETS:
            system = E.fromstring(archive.read(f'simulink/systems/system_{ref}.xml'))
            geometry = Geometry(system)
            fig, ax = plt.subplots(figsize=(16, 11))
            for wire in system.findall('Line'):
                if wire.get('LineType') == 'Connection':
                    continue
                for _, _, points in geometry.polylines(wire):
                    ax.plot(*zip(*points), linewidth=0.8, color='#426c8a')
            for sid, item in geometry.blocks.items():
                left, top, right, bottom = geometry.position(sid)
                ax.add_patch(Rectangle((left, top), right-left, bottom-top,
                                       edgecolor='#294459', facecolor='#eef4f8', linewidth=0.7))
                ax.text((left+right)/2, (top+bottom)/2,
                        sid + '\n' + item.get('BlockType'), fontsize=5, ha='center', va='center')
            ax.autoscale()
            ax.invert_yaxis()
            ax.set_aspect('equal')
            ax.set_axis_off()
            fig.tight_layout()
            fig.savefig(args.output_dir / f'checked_{ref}.png', dpi=140)
            plt.close(fig)


if __name__ == '__main__':
    main()
