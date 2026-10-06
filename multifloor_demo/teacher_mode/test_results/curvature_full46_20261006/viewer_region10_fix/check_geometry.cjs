const fs=require('fs'),vm=require('vm'),assert=require('assert'),crypto=require('crypto'),path=require('path');
const root=path.resolve(__dirname,'../../..');
const html=fs.readFileSync(path.join(root,'web/index.html'),'utf8');
const funcs=html.slice(html.indexOf('function goalXYFootprint('),html.indexOf('function navigationPlot('));
const scope={finite:Number.isFinite};vm.createContext(scope);vm.runInContext(funcs,scope);
const fixture=JSON.parse(fs.readFileSync(path.join(__dirname,'ACTUAL_REQUEST_FIXTURE.json'),'utf8'));
let count=0;
function check(name,fn){fn();count++;}
check('all actual 18 goal footprints drawable',()=>assert.equal(fixture.goals.filter(g=>scope.goalXYFootprint(g)).length,18));
const g=fixture.goals[9],shape=scope.goalXYFootprint(g);
check('region10 box retained',()=>assert.equal(shape.kind,'polygon'));
check('human numbering without mission alteration',()=>{assert.equal(scope.goalDisplayLabel(g),'第10区');assert.equal(g.goal_id,'exploration:9')});
check('3D axes are world vectors; every original corner enclosed',()=>{
 const cross=(a,b,c)=>(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
 for(const sx of [-1,1])for(const sy of [-1,1])for(const sz of [-1,1]){
  const signs=[sx,sy,sz],p=[0,1].map(k=>g.center[k]+signs.reduce((v,s,i)=>v+s*g.arrival.half_extents_m[i]*g.arrival.axes[i][k],0));
  for(let i=0;i<shape.points.length;i++)assert(cross(shape.points[i],shape.points[(i+1)%shape.points.length],p)>=-1e-12);
 }
});
check('axis-aligned box footprint exact dimensions',()=>{const s=scope.goalXYFootprint({center:[0,0,0],arrival:{axes:[[1,0,0],[0,1,0],[0,0,1]],half_extents_m:[2,1,.1]}});assert.equal(s.points.length,4);assert.equal(Math.max(...s.points.map(p=>p[0])),2);assert.equal(Math.min(...s.points.map(p=>p[1])),-1)});
check('disc presentation preserved',()=>{const s=scope.goalXYFootprint({center:[1,2,3],arrival:{radius_m:.4}});assert.equal(s.kind,'circle');assert.equal(s.radius,.4)});
check('invalid dimensions safely excluded',()=>{assert.equal(scope.goalXYFootprint({center:[0,0,0],arrival:{axes:[[1,0,0]],half_extents_m:[1,1,1]}}),null);assert.equal(scope.goalXYFootprint({center:[NaN,0,0]}),null)});
const receipt={status:'passed',tests:count,scope:'display-only; actual 3D arrival and mission untouched',html_sha256:crypto.createHash('sha256').update(html).digest('hex'),actual_fixture_goal10:g,footprint:shape};
fs.writeFileSync(path.join(__dirname,'GEOMETRY_RECEIPT.json'),JSON.stringify(receipt,null,2)+'\n');
console.log(JSON.stringify({status:receipt.status,tests:count,html_sha256:receipt.html_sha256}));
