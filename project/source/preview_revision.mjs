// Layout preview only. Does not modify production outputs or original CAD.
import {M,C,fs,path,OUT,box,cylY,cylZ,union,importSTL,render} from './preview_helpers.mjs';
const root=path.resolve(import.meta.dirname,'..');
const items=[];
async function existing(file,name,offset,c){const m=await importSTL(path.join(root,'outputs',file+'.stl'));items.push({name,m:m.translate(offset),c});}
await existing('z_lift_frame','fixed_frame',[0,0,100],'#75889e');
await existing('z_lift_carriage','moving_carriage',[0,0,102],'#d49b48');
await existing('z_rack','rack',[0,0,98],'#d49b48');
await existing('z_rack_guide','captured_guides',[0,0,100],'#75889e');
await existing('z_claw_platform_2xM3','platform',[0,0,94],'#d49b48');
// Reuse the current involute generator with a smaller tooth count.
const original=await fs.readFile(path.join(root,'source/build.mjs'),'utf8');
const gearFunction=original.slice(original.indexOf('function liftPinion()'),original.indexOf('const rackLong = compactRack();'));
const extrudeXZ=(cs,t,y=0)=>cs.extrude(t).translate([0,0,-t/2]).rotate([90,0,0]).translate([0,y,0]);
const cylX=(r,x0,x1,y=0,z=0,n=48)=>M.cylinder(x1-x0,r,r,n).rotate([0,90,0]).translate([x0,y,z]);
const P={lift_pinion_teeth:18,lift_module:2,lift_pinion_pitch_radius:18};
const gear=new Function('P','C','extrudeXZ','cylY','cylX','union',gearFunction+';return liftPinion();')(P,C,extrudeXZ,cylY,cylX,union);
items.push({name:'smaller_lift_pinion',m:gear.translate([12,-89,190]),c:'#e4a63e'});
items.push({name:'lift_servo_placeholder',m:box([-8,-140,172],[32,-120,208]),c:'#318b9b'});
items.push({name:'lift_servo_shaft',m:cylY(4,-122,-84,12,190),c:'#b5bcc7'});
// Donor STL files are already in the SLDASM assembly coordinates (millimetres).
const donorDir=path.join(root,'reference/new_gripper');
const claw=[];
for(const file of (await fs.readdir(donorDir)).filter(f=>f.startsWith('1 - '))){
 let m=await importSTL(path.join(donorDir,file));
 // Retain the two pivot holes of each donor finger; replace its long contact tip.
 if(file.includes('6-'))m=m.intersect(box([-500,-200,200],[300,195,300]));
 m=m.translate([87.346024636,0,-250]).scale(.25).rotate([-90,0,0]).translate([0,-46,143.25]);
 const c=file.includes('6-')?'#db9e48':file.includes('2-')?'#cb6689':file.includes('7-')?'#8d98a4':file.includes('3-')?'#c4cbd2':file.includes('4-')?'#62a574':'#7962b2';
 claw.push({name:'donor_'+file,m,c});
}
const round=await importSTL(path.join(root,'reference','圆爪.STL'));
const roundBottom=16.85;
claw.push({name:'round_left_handle_up',m:round.rotate([90,0,0]).rotate([0,0,90]).translate([-67.5,-74,roundBottom]),c:'#e6ab4e'});
claw.push({name:'round_right_handle_up',m:round.rotate([90,0,0]).rotate([0,0,-90]).translate([67.5,-18,roundBottom]),c:'#e6ab4e'});
// Proposed rear mounting bridge: an envelope only, pending detailed hole design.
let bridge=union(box([-18,-81,94],[18,-54,102]),box([-18,-65,100],[-12,-60,169]),box([12,-65,100],[18,-60,169]),box([-18,-65,161],[18,-54,169]));
for(const x of [-12,12])bridge=bridge.subtract(cylZ(1.7,93,103,x,-74));
items.push({name:'proposed_mount_bridge',m:bridge,c:'#5ba4aa'});
items.push(...claw);
const bb=M.compose(items.map(x=>x.m)).boundingBox();
await render(items,path.join(OUT,'layout_front.png'),{width:1300,height:1500,target:[0,-75,148],az:180,el:5,label:'新夹爪布局预览 · 正面',sub:'原夹爪机构按25%展示｜圆爪原尺寸、开孔柄向上｜升降齿轮外径40 mm'});
await render(items,path.join(OUT,'layout_perspective.png'),{width:1400,height:1500,target:[0,-75,148],az:148,el:18,label:'新夹爪布局预览 · 斜视',sub:'青色为拟定连接桥；布局草案，尚未完成安装孔、啮合和干涉校核'});
await render(claw,path.join(OUT,'claw_detail.png'),{width:1400,height:1100,target:[0,-46,98],az:24,el:12,label:'新夹爪与圆爪 · 方向细节',sub:'保留齿轮及双侧连杆；原长指尖截短，圆爪开孔柄竖直向上'});
const report={scope:'preview only',donor_scale:0.25,round_claw_scale:1,round_orientation:'source +Y handle points to world +Z',lift_pinion:{module:2,teeth:18,pitch_diameter_mm:36,outside_diameter_mm:40,assumed_servo_angle_deg:150,theoretical_stroke_mm:18*150*Math.PI/180},bbox_mm:bb,limitations:['Concept mounting bridge, not a finished mounting interface','Donor holes and gear module shrink with preview scale and require redesign before fabrication','No collision, motion, servo torque or grip validation','Original project outputs and native CAD remain unchanged']};
await fs.writeFile(path.join(OUT,'preview_parameters.json'),JSON.stringify(report,null,2));
console.log(JSON.stringify(report,null,2));
