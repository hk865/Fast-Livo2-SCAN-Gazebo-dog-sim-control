#!/usr/bin/env python3
"""Export the authored Markdown documents with system Pango/Cairo only.
No application import, simulation, download, or environment mutation is performed.
"""
from pathlib import Path
import re, html, json, math
# Use the system C libraries directly: this host lacks the optional gi-cairo bridge.
import ctypes as C, ctypes.util as U
from types import SimpleNamespace
import xml.etree.ElementTree as ET
cai=C.CDLL(U.find_library('cairo'));pan=C.CDLL(U.find_library('pango-1.0'));pc=C.CDLL(U.find_library('pangocairo-1.0'));obj=C.CDLL(U.find_library('gobject-2.0'));rsvg=C.CDLL(U.find_library('rsvg-2'))
def sig(lib,name,result,args):
 f=getattr(lib,name);f.restype=result;f.argtypes=args;return f
ptr=C.c_void_p;dbl=C.c_double;integer=C.c_int;char=C.c_char_p
for name,res,args in [('cairo_pdf_surface_create',ptr,[char,dbl,dbl]),('cairo_pdf_surface_set_metadata',None,[ptr,integer,char]),('cairo_surface_finish',None,[ptr]),('cairo_surface_destroy',None,[ptr]),('cairo_create',ptr,[ptr]),('cairo_destroy',None,[ptr]),('cairo_set_source_rgb',None,[ptr,dbl,dbl,dbl]),('cairo_rectangle',None,[ptr,dbl,dbl,dbl,dbl]),('cairo_fill',None,[ptr]),('cairo_move_to',None,[ptr,dbl,dbl]),('cairo_line_to',None,[ptr,dbl,dbl]),('cairo_set_line_width',None,[ptr,dbl]),('cairo_stroke',None,[ptr]),('cairo_show_page',None,[ptr]),('cairo_save',None,[ptr]),('cairo_restore',None,[ptr]),('cairo_translate',None,[ptr,dbl,dbl]),('cairo_scale',None,[ptr,dbl,dbl])]:sig(cai,name,res,args)
class Rect(C.Structure):_fields_=[('x',integer),('y',integer),('width',integer),('height',integer)]
for name,res,args in [('pango_font_description_from_string',ptr,[char]),('pango_font_description_set_absolute_size',None,[ptr,dbl]),('pango_font_description_free',None,[ptr]),('pango_layout_set_font_description',None,[ptr,ptr]),('pango_layout_set_width',None,[ptr,integer]),('pango_layout_set_wrap',None,[ptr,integer]),('pango_layout_set_spacing',None,[ptr,integer]),('pango_layout_set_markup',None,[ptr,char,integer]),('pango_layout_set_alignment',None,[ptr,integer]),('pango_layout_get_extents',None,[ptr,C.POINTER(Rect),C.POINTER(Rect)])]:sig(pan,name,res,args)
sig(pc,'pango_cairo_create_layout',ptr,[ptr]);sig(pc,'pango_cairo_show_layout',None,[ptr,ptr]);sig(obj,'g_object_unref',None,[ptr]);sig(rsvg,'rsvg_handle_new_from_file',ptr,[char,ptr]);sig(rsvg,'rsvg_handle_render_cairo',integer,[ptr,ptr])
class Surface:
 def __init__(self,path,w,h):self.ptr=cai.cairo_pdf_surface_create(path.encode(),w,h)
 def set_metadata(self,key,value):cai.cairo_pdf_surface_set_metadata(self.ptr,key,value.encode())
 def finish(self):cai.cairo_surface_finish(self.ptr);cai.cairo_surface_destroy(self.ptr)
class Context:
 def __init__(self,surface):self.ptr=cai.cairo_create(surface.ptr)
 def __getattr__(self,name):
  f=getattr(cai,'cairo_'+name)
  return lambda *args:f(self.ptr,*args)
class Layout:
 def __init__(self,p):self.ptr=p
 def get_extents(self):
  ink,logical=Rect(),Rect();pan.pango_layout_get_extents(self.ptr,C.byref(ink),C.byref(logical));return ink,logical
 def __del__(self):
  if self.ptr:obj.g_object_unref(self.ptr)
cairo=SimpleNamespace(PDFSurface=Surface,Context=Context,PDFMetadata=SimpleNamespace(TITLE=0,AUTHOR=1))
Pango=SimpleNamespace(SCALE=1024,Alignment=SimpleNamespace(RIGHT=2))
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'output/pdf'
W,H=595.276,841.89
L,R,TOP,BOTTOM=52,52,66,57
CW=W-L-R
NAVY=(.07,.17,.24);TEAL=(.0,.43,.46);GREY=(.34,.40,.44)
FONT='Noto Sans CJK SC'
SOURCES=[('第一部分 · 技术报告',ROOT/'docs/technical_report.zh-CN.md'),('第二部分 · 源码结构与数据流',ROOT/'docs/source_architecture.zh-CN.md'),('第三部分 · 移植构建运行',ROOT/'docs/移植构建运行.md'),('第四部分 · 本轮验证状态',ROOT/'docs/验证状态.md')]
LINKS={}
def plain(s):
 s=re.sub(r'!\[([^\]]*)\]\([^)]*\)',r'\1',s)
 def link(m):
  if m[2].startswith('http'):LINKS[m[2]]=m[1]
  return m[1]
 s=re.sub(r'\[([^\]]+)\]\(([^)]+)\)',link,s)
 return s.replace('**','').replace('`','')
def markup(s):
 s=plain(s)
 return html.escape(s)

def parse(path):
 lines=path.read_text().splitlines();out=[];i=0
 while i<len(lines):
  line=lines[i]
  if not line.strip():i+=1;continue
  if line.startswith('```'):
   kind=line[3:].strip();i+=1;rows=[]
   while i<len(lines) and not lines[i].startswith('```'):rows.append(lines[i]);i+=1
   i+=1
   if kind=='mermaid':
    # The authored SVG already renders the system and correction diagrams.
    if rows and rows[0].strip()=='sequenceDiagram' and any('start.py' in x for x in rows):
     out.append(('code','启动：准备world/参数 → Actor ready → ROS桥/节点 → Gazebo\n运行：传感器/clock → SLAM/控制；state64 ↔ 12关节目标＋协议字段\n退出：SIGINT launch owner → 子进程有序退出/必要时升级 → 释放锁'))
    continue
   out.append(('code','\n'.join(rows)));continue
  if line.startswith('|'):
   rows=[]
   while i<len(lines) and lines[i].startswith('|'):
    cells=[c.strip() for c in lines[i].strip().strip('|').split('|')]
    if not all(re.fullmatch(r'[:\- ]+',c or '-') for c in cells):rows.append(cells)
    i+=1
   out.append(('table',rows));continue
  m=re.match(r'^(#{1,4})\s+(.*)',line)
  if m:out.append(('h'+str(len(m[1])),m[2]));i+=1;continue
  m=re.match(r'!\[([^\]]*)\]\(([^)]+)\)',line)
  if m:out.append(('image',(m[1],path.parent/m[2])));i+=1;continue
  if line.startswith('> '):out.append(('quote',line[2:]));i+=1;continue
  if re.match(r'^(\d+\. |[-*] )',line):out.append(('list',line));i+=1;continue
  rows=[line];i+=1
  while i<len(lines) and lines[i].strip() and not re.match(r'^(#|\||```|>|!\[|\d+\. |[-*] )',lines[i]):rows.append(lines[i]);i+=1
  out.append(('p',' '.join(rows)))
 return out
class Renderer:
 def __init__(self,path,total=0,toc=None):
  self.surface=cairo.PDFSurface(str(path),W,H);self.ctx=cairo.Context(self.surface)
  self.surface.set_metadata(cairo.PDFMetadata.TITLE,'四足机器人导航：技术报告与源码数据流')
  self.surface.set_metadata(cairo.PDFMetadata.AUTHOR,'Project documentation')
  self.page=1;self.y=TOP;self.total=total;self.part='';self.toc=toc or [];self.headings=[];self.bounds=[]
 def color(self,rgb):self.ctx.set_source_rgb(*rgb)
 def layout(self,text,width,size=10.3,bold=False,mono=False,align=None):
  lo=Layout(pc.pango_cairo_create_layout(self.ctx.ptr));desc=pan.pango_font_description_from_string((('Noto Sans Mono CJK SC' if mono else FONT)+(' Bold' if bold else '')).encode())
  pan.pango_font_description_set_absolute_size(desc,size*1024);pan.pango_layout_set_font_description(lo.ptr,desc);pan.pango_font_description_free(desc);pan.pango_layout_set_width(lo.ptr,int(width*1024));pan.pango_layout_set_wrap(lo.ptr,2);pan.pango_layout_set_spacing(lo.ptr,int(size*.23*1024));pan.pango_layout_set_markup(lo.ptr,markup(text).encode(),-1)
  if align is not None:pan.pango_layout_set_alignment(lo.ptr,align)
  return lo
 def measure(self,text,width,size=10.3,bold=False,mono=False):
  lo=self.layout(text,width,size,bold,mono);_,r=lo.get_extents();return lo,r.height/1024
 def draw(self,lo,x,y,color=NAVY):
  self.color(color);self.ctx.move_to(x,y);pc.pango_cairo_show_layout(self.ctx.ptr,lo.ptr)
 def footer(self):
  self.color((.75,.81,.83));self.ctx.set_line_width(.5);self.ctx.move_to(L,H-43);self.ctx.line_to(W-R,H-43);self.ctx.stroke()
  lo=self.layout('算法基线 b29d3eef · 模块化源码与新构建 · 验证范围单列',CW-70,7.6);self.draw(lo,L,H-34,GREY)
  lo=self.layout(f'{self.page}'+(f' / {self.total}' if self.total else ''),65,8,align=Pango.Alignment.RIGHT);self.draw(lo,W-R-65,H-34,GREY)
 def header(self):
  lo=self.layout('FAST-LIVO2 / SCAN / GO2',230,8.2,bold=True);self.draw(lo,L,29,TEAL)
  lo=self.layout(self.part,260,8.2,align=Pango.Alignment.RIGHT);self.draw(lo,W-R-260,29,GREY)
 def pagebreak(self):
  self.footer();self.ctx.show_page();self.page+=1;self.y=TOP;self.header()
 def ensure(self,height):
  if self.y+height>H-BOTTOM:self.pagebreak()
 def text(self,text,size=10.3,bold=False,space=8,mono=False,bg=False):
  lo,height=self.measure(text,CW-16 if bg else CW,size,bold,mono)
  if height>H-BOTTOM-TOP:
   # Split long preformatted blocks at paragraph/line boundaries.
   for row in text.splitlines():self.text(row,size,bold,2,mono,bg)
   return
  self.ensure(height+space+(12 if bg else 0))
  if bg:
   self.color((.95,.97,.98));self.ctx.rectangle(L-2,self.y-5,CW+4,height+12);self.ctx.fill();self.draw(lo,L+8,self.y)
  else:self.draw(lo,L,self.y)
  self.bounds.append((self.page,self.y,self.y+height));self.y+=height+space+(7 if bg else 0)
 def cover(self):
  self.color(NAVY);self.ctx.rectangle(0,0,W,245);self.ctx.fill()
  self.draw(self.layout('四足机器人\n仿真导航工程',CW,29,bold=True),L,65,(1,1,1))
  self.draw(self.layout('技术报告 · 源码结构 · 数据流',CW,15),L,172,(.67,.88,.88))
  self.y=282
  self.text('FAST-LIVO2 / SCAN / Student45-GRU',17,True,18)
  self.text('从传感器与坐标开始，沿定位、地图、规划、学习控制与物理执行，解释一条可以检查的工程闭环。',13,space=25)
  self.text('算法基线  原正式工程 b29d3eef\n交付范围  模块化源码、必要资产与公共环境构建\n实验范围  旧第26轮与本轮验证分别陈述',10.6,space=25,bg=True)
  self.text('使用本报告',14,True,10)
  self.text('第一部分解释算法、公式、实际启用与历史证据；第二部分提供源码目录与阅读路线；第三、四部分说明移植方法和本轮验证。模块README与详细数据流保留在Markdown文档包中。',11,space=20)
  self.text('边界先行',14,True,10)
  self.text('停车漂移不是绝对定位精度。目标Z不是真值。当前仍属仿真适配。干净构建及一次原场景闭环已通过；训练复现、真值对齐、重复可靠率和跨硬件迁移仍有证据缺口。',11)
  self.pagebreak();self.part='目录';self.header();self.text('目录',24,True,20)
  self.text('章节页码对应本PDF；Markdown中的相对链接用于仓库阅读。',9.5,space=16)
  last=''
  for part,title,page in self.toc:
   if part!=last:self.text(part,12,True,9);last=part
   lo,h=self.measure(title,CW-55,9.7);self.draw(lo,L,self.y);self.draw(self.layout(str(page),45,9.7,align=Pango.Alignment.RIGHT),W-R-45,self.y);self.y+=h+6
  self.pagebreak()
 def table(self,rows):
  if not rows:return
  n=max(map(len,rows));weights=([.24,.26,.50] if n==3 else [.18,.25,.28,.29] if n==4 else [.34,.66] if n==2 else [1/n]*n);widths=[CW*x for x in weights]
  def row(cells,head=False):
   layouts=[self.measure(c,widths[j]-12,8.6,head) for j,c in enumerate(cells)]
   height=max([h for _,h in layouts]+[12])+13
   self.ensure(height+3)
   self.color((.88,.94,.95) if head else (.975,.98,.985));self.ctx.rectangle(L,self.y,CW,height);self.ctx.fill()
   x=L
   for j,(lo,h) in enumerate(layouts):self.draw(lo,x+6,self.y+6);x+=widths[j]
   self.color((.78,.84,.86));self.ctx.set_line_width(.4);self.ctx.move_to(L,self.y+height);self.ctx.line_to(W-R,self.y+height);self.ctx.stroke()
   self.bounds.append((self.page,self.y,self.y+height));self.y+=height
  row(rows[0],True)
  for cells in rows[1:]:
   if self.y+max([self.measure(c,widths[j]-12,8.6)[1] for j,c in enumerate(cells)]+[12])+16>H-BOTTOM:self.pagebreak();row(rows[0],True)
   row(cells)
  self.y+=10
 def image(self,label,path):
  root=ET.parse(path).getroot();width,height=float(root.attrib['width']),float(root.attrib['height']);display_height=CW*height/width
  self.ensure(display_height+26);handle=rsvg.rsvg_handle_new_from_file(str(path).encode(),None)
  if not handle:raise RuntimeError('SVG load failed')
  self.ctx.save();self.ctx.translate(L,self.y);self.ctx.scale(CW/width,CW/width);ok=rsvg.rsvg_handle_render_cairo(handle,self.ctx.ptr);self.ctx.restore();obj.g_object_unref(handle)
  if not ok:raise RuntimeError('SVG render failed')
  self.y+=display_height+5;self.text(label,8.6,space=12)
 def body(self):
  for part,path in SOURCES:
   self.part=part
   if self.y>TOP+1:self.pagebreak()
   self.header();self.text(part,22,True,16)
   blocks=parse(path)
   def next_height(index):
    if index>=len(blocks):return 0
    kind,value=blocks[index]
    if kind in ('h2','h3'):return 32+next_height(index+1)
    if kind=='table':return 55
    if kind=='image':return CW*.75+25
    return min(210,self.measure(value,CW,10.3)[1]+12)
   for index,(kind,value) in enumerate(blocks):
    if kind=='h1':continue
    if kind=='h2':
     self.ensure(52+next_height(index+1));self.y+=11;self.headings.append((part,plain(value),self.page));self.text(value,17,True,13)
    elif kind=='h3':self.ensure(40+next_height(index+1));self.y+=5;self.text(value,12.6,True,9)
    elif kind=='p':self.text(value)
    elif kind=='list':self.text(value,10,space=6)
    elif kind=='quote':self.text(value,9.2,space=12,bg=True)
    elif kind=='code':self.text(value,8.6,space=10,mono=True,bg=True)
    elif kind=='table':self.table(value)
    elif kind=='image':self.image(*value)
  self.part='外部来源与使用说明';self.pagebreak();self.text('外部来源与使用说明',20,True,15)
  self.text('以下是正文核对的外部原理来源。具体工程行为仍以本地源码、实际有效配置和已明确范围的运行报告为准。历史H1-H4仅保留标题和结论，未复制私有宿主路径。',10.5,space=15)
  for url,label in LINKS.items():self.text(label+'\n'+url,9,space=12)
  self.text('Markdown与模块导航',13,True,10)
  self.text('主源：docs/technical_report.zh-CN.md、docs/source_architecture.zh-CN.md。根README适合第一次使用；主要模块README解释职责、输入输出、关键入口、配置和调试。完整文档包保留SVG图及所有README，不包含模型、运行日志、凭据或应用代码。',10.5)
 def finish(self):self.footer();self.surface.finish();return self.page

def main():
 OUT.mkdir(parents=True,exist_ok=True)
 draft=Path('/tmp/navigation_documentation_layout.pdf');r=Renderer(draft);r.cover();r.body();pages=r.finish();headings=r.headings
 out=OUT/'navigation_technical_report.zh-CN.pdf';r=Renderer(out,pages,headings);r.cover();r.body();final_pages=r.finish()
 if final_pages!=pages:raise RuntimeError('Page count changed on final render')
 if any(bottom>H-BOTTOM+.1 for _,_,bottom in r.bounds):raise RuntimeError('Content outside page bounds')
 (OUT/'layout_check.json').write_text(json.dumps({'pages':pages,'headings':headings,'content_blocks':len(r.bounds),'page_bounds_passed':True,'font':FONT,'engine':'system Pango/Cairo','source_files':[str(p.relative_to(ROOT)) for _,p in SOURCES]},ensure_ascii=False,indent=2)+'\n')
 print(json.dumps({'pdf':str(out.relative_to(ROOT)),'pages':pages,'blocks':len(r.bounds),'bounds_passed':True}))
if __name__=='__main__':main()
