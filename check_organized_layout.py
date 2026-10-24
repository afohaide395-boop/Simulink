"""检查整理后顶层及四个新子系统的信号线几何；电气拓扑另由结构检查验证。"""
from pathlib import Path
import zipfile,xml.etree.ElementTree as E,re,ast
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import argparse
parser=argparse.ArgumentParser()
parser.add_argument('--output-dir',type=Path)
args=parser.parse_args()
HERE=Path(__file__).resolve().parent
OUTPUT=args.output_dir
if OUTPUT is not None:OUTPUT.mkdir(parents=True,exist_ok=True)
with zipfile.ZipFile(HERE/'reduce_emf_organized.slx') as z:
 for ref in ['root','13210','13220','13230','13240']:
  r=E.fromstring(z.read(f'simulink/systems/system_{ref}.xml'));blocks={b.get('SID'):b for b in r.findall('Block')}
  ins={};outs={}
  for l in r.findall('Line'):
   if l.get('LineType')=='Connection':continue
   s=l.find("P[@Name='Src']").text;bid,n=s.split('#out:');outs[bid]=max(outs.get(bid,0),int(n))
   for p in l.findall(".//P[@Name='Dst']"):
    bid,n=p.text.split('#in:');ins[bid]=max(ins.get(bid,0),int(n))
  coords={sid:ast.literal_eval(b.find("P[@Name='Position']").text) for sid,b in blocks.items()}
  def endpoint(text):
   bid,typ=text.split('#');side,n=typ.split(':');n=int(n);bb=blocks[bid];xy=coords[bid];pc=bb.find('PortCounts')
   num=int(pc.get(side,str((outs if side=='out' else ins).get(bid,1)))) if pc is not None else (outs if side=='out' else ins).get(bid,1)
   return (xy[2] if side=='out' else xy[0],xy[1]+(n-.5)*(xy[3]-xy[1])/max(num,1))
  fig,ax=plt.subplots(figsize=(20,12) if ref=='12300' else (14,8));cross=[]
  for l in r.findall('Line'):
   if l.get('LineType')=='Connection':continue
   src=l.find("P[@Name='Src']").text
   for node in [l]+l.findall('.//Branch'):
    dest=node.find("P[@Name='Dst']")
    if dest is None:continue
    dest=dest.text;pts=[endpoint(src)];
    if node is not l:
     trunk=l.find("P[@Name='Points']")
     assert trunk is not None, 'Branch has no main geometry'
     for pair in trunk.text.strip('[]').split(';'):
      dx,dy=map(float,pair.split(','));x,y=pts[-1];pts.append((x+dx,y+dy))
    q=node.find("P[@Name='Points']")
    if q is not None:
     for pair in q.text.strip('[]').split(';'):
      dx,dy=map(float,pair.split(','));x,y=pts[-1];pts.append((x+dx,y+dy))
    pts.append(endpoint(dest));ax.plot(*zip(*pts),lw=.7,color='#477799',zorder=1)
    for sid,rect in coords.items():
     if sid in [src.split('#')[0],dest.split('#')[0]]:continue
     x0,y0,x1,y1=rect
     for (x,y),(xx,yy) in zip(pts,pts[1:]):
      horizontal=abs(y-yy)<.1 and y0+1<y<y1-1 and max(min(x,xx),x0+1)<min(max(x,xx),x1-1)
      vertical=abs(x-xx)<.1 and x0+1<x<x1-1 and max(min(y,yy),y0+1)<min(max(y,yy),y1-1)
      if horizontal or vertical:cross.append((src,dest,sid))
  for sid,b in blocks.items():
   x,y,xx,yy=coords[sid]
   ax.add_patch(Rectangle((x,y),xx-x,yy-y,facecolor='#e8f0f5',edgecolor='#203848',lw=.6,zorder=2))
   ax.text((x+xx)/2,(y+yy)/2,sid+'\n'+b.get('BlockType'),fontsize=5.5,ha='center',va='center',zorder=3)
  ax.autoscale();ax.invert_yaxis();ax.set_aspect('equal');ax.set_axis_off();fig.tight_layout();
  if OUTPUT is not None:fig.savefig(OUTPUT/f'layout_{ref}.png',dpi=130)
  plt.close(fig)
  overlaps=[]
  for i,(aa,a) in enumerate(coords.items()):
   for bb,br in list(coords.items())[i+1:]:
    if max(a[0],br[0])<min(a[2],br[2]) and max(a[1],br[1])<min(a[3],br[3]):overlaps.append((aa,bb))
  print(ref,'overlapping blocks:',overlaps)
  assert not overlaps
  assert not cross
  print(ref,'segments crossing other block interiors:',len(set(cross)))
  for item in sorted(set(cross)):print(item)
