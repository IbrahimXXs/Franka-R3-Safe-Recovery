'use strict';
const D=JSON.parse(document.getElementById('study-data').textContent);
const $=id=>document.getElementById(id);
const esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const finite=v=>typeof v==='number'&&Number.isFinite(v);
const fmt=v=>v===null||v===undefined?'—':typeof v==='number'?(v===0?'0':Math.abs(v)<.001?v.toExponential(3):v.toLocaleString(undefined,{maximumFractionDigits:4})):String(v);
const recoveryPhases=new Set(['realign','retreat','clear_hold']);
const colors=['#087e8b','#c07324','#7152aa','#287e4c','#c44965','#3b6fbe'];
const scenarios=[...new Set(D.trials.map(t=>t.summary.scenario))];
const color=t=>colors[scenarios.indexOf(t.summary.scenario)%colors.length];
const label=t=>D.phase2?`${t.summary.scenario} · ${t.id}`:`${t.summary.scenario} / ${t.summary.recovery}`;
const selected=new Set();
const completed=D.trials.filter(t=>t.eligible);
const defaultScenario=completed.some(t=>t.summary.scenario==='loaded_deep')?'loaded_deep':completed[0]?.summary.scenario;
completed.filter(t=>t.summary.scenario===defaultScenario).forEach(t=>selected.add(t.id));
let activeTab='profiles', sortKey='scenario', sortAscending=true, drawing=Promise.resolve(), syncZoom=false;
const signals={fz:['Signed axial force Fz','Force [N]'],resistance:['Phase-aware axial resistance','Axial resistance [N]'],force_norm_n:['Net contact force magnitude','Net force [N]'],fx:['Force Fx','Force [N]'],fy:['Force Fy','Force [N]'],normal_fz:['Normal component Fz','Normal force [N]'],friction_fz:['Friction component Fz','Friction force [N]'],normal_load_n:['Total normal contact load','Normal load [N]'],min_separation_mm:['Minimum contact separation','Separation [mm]']};
const metrics=D.phase2?{max_force:"Peak insertion net force [N]",max_torque:"Peak insertion torque [N m]",max_normal_load:"Peak normal load [N]",max_penetration:"Maximum overlap [mm]",max_depth:"Maximum depth [mm]"}:{retreat_peak_resistance_n:'Peak retreat resistance [N]',retreat_peak_50ms_resistance_n:'50 ms peak retreat resistance [N]',retreat_resistance_impulse_ns:'Retreat resistance impulse [N s]',recovery_peak_force_n:'Peak recovery net force [N]',recovery_peak_torque_nm:'Peak recovery torque [N m]',recovery_resistive_work_j:'Recovery resisting-work proxy [J]',time_to_clear_s:'Time to sustained clearance [s]',insertion_peak_resistance_n:'Peak insertion resistance [N]'};
const columns=D.phase2?[["scenario","Family"],["status","Numerical status"],["insertion_success","Inserted"],["max_force","Peak force [N]"],["max_torque","Peak torque [N m]"],["max_normal_load","Peak normal load [N]"],["max_penetration","Max overlap [mm]"],["max_depth","Max depth [mm]"],["stalled","Stalled"],["force_budget_exceeded","Force budget exceeded"],["torque_budget_exceeded","Torque budget exceeded"],["numerically_valid","Numerically valid"]]:[['scenario','Scenario'],['recovery','Recovery'],['status','Outcome'],['depth_before_recovery_mm','Depth before recovery [mm]'],['tilt_before_recovery_deg','Tilt before recovery [°]'],['retreat_peak_resistance_n','Peak retreat [N]'],['recovery_peak_force_n','Peak recovery net force [N]'],['recovery_peak_torque_nm','Peak recovery torque [N m]'],['recovery_resistive_work_j','Recovery work [J]'],['time_to_clear_s','Clearance time [s]'],['max_penetration_mm','Max overlap [mm]'],['penetration_screen_passed','Overlap screen']];
const config={responsive:true,displaylogo:false,scrollZoom:true,toImageButtonOptions:{format:'png',scale:2},modeBarButtonsToRemove:['lasso2d','select2d']};
function options(element, items){for(const [v,n] of Object.entries(items)){const o=document.createElement('option');o.value=v;o.textContent=Array.isArray(n)?n[0]:n;element.append(o);}}
options($('signal'),signals);options($('metric'),metrics);
for(const p of [...new Set(D.trials.flatMap(t=>t.series.phase||[]))].filter(Boolean)){const o=document.createElement('option');o.value=p;o.textContent=p;$('phase').append(o);}
$('study-name').textContent=D.name;
$('subtitle').textContent=`${D.manifest.study||'FR3 study'} · ${D.manifest.status||'unknown status'} · ${D.manifest.parameters?.physics_hz??'unknown'} Hz physics`;
const invalid=D.trials.filter(t=>!t.eligible).length, review=completed.filter(t=>t.summary.penetration_screen_passed===false).length;
$('cards').innerHTML=[[D.trials.length,'Trial outcomes'],[completed.length,'Completed trials'],[invalid,'Invalid / incomplete'],[review,'Completed · overlap review']].map(([n,l])=>`<div class="card"><strong>${n}</strong><span>${l}</span></div>`).join('');
const notices=[...D.warnings];
if(invalid)notices.unshift(`${invalid} trial(s) are invalid or incomplete. Their costs are excluded; partial traces are available only when diagnostic viewing is enabled.`);
if(review)notices.unshift(`${review} completed trial(s) exceeded the overlap review threshold. Treat their force costs as preliminary until numerical convergence is established.`);
if(D.manifest.status==='invalid_sensor_configuration')notices.unshift('INVALID SENSOR CONFIGURATION: this study cannot support force or recovery-cost conclusions.');
$('warnings').innerHTML=notices.map(s=>`<div class="notice">${esc(s)}</div>`).join('');
function choices(){
 $('trials').replaceChildren();
 for(const t of D.trials){const l=document.createElement('label');l.className='trial'+(!t.eligible&&!$('include-invalid').checked?' off':'');
 const cb=document.createElement('input');cb.type='checkbox';cb.checked=selected.has(t.id);cb.disabled=!t.eligible&&!$('include-invalid').checked;cb.dataset.trial=t.id;cb.onchange=()=>{cb.checked?selected.add(t.id):selected.delete(t.id);refresh();};
 const dot=document.createElement('span');dot.className='dot';dot.style.background=color(t);l.append(cb,dot,document.createTextNode(label(t)));
 if(!t.eligible){const tag=document.createElement('em');tag.textContent=t.status;l.append(tag);}
 $('trials').append(l);}
}
function chosen(){return D.trials.filter(t=>selected.has(t.id)&&(t.eligible||$('include-invalid').checked));}
function baseLayout(ytitle, xtitle){return {paper_bgcolor:'white',plot_bgcolor:'white',font:{family:'system-ui, sans-serif',color:'#263f49',size:12},margin:{l:76,r:30,t:32,b:60},hovermode:'closest',legend:{orientation:'h',y:1.16,font:{size:11}},xaxis:{title:{text:xtitle},gridcolor:'#e4edef',zerolinecolor:'#bccdd2'},yaxis:{title:{text:ytitle},gridcolor:'#e4edef',zerolinecolor:'#bccdd2'}};}
function xTitle(){return $('x-mode').value==='depth'?'Actual insertion depth [mm]':$('x-mode').value==='recovery'?'Time from recovery start [s]':'Experiment time [s]';}
function recoveryStart(s){const i=(s.phase||[]).findIndex(p=>recoveryPhases.has(p));return i<0?null:i>0?s.time_s[i-1]:s.time_s[0]-1/(D.manifest.parameters?.physics_hz||480);}
function value(s,key,i){if(key==='resistance'){const f=s.fz?.[i],p=s.phase?.[i];return !finite(f)?null:p==='insert'?Math.max(0,f):p==='retreat'?Math.max(0,-f):null;}return s[key]?.[i]??null;}
function makeTrace(t,key){
 const s=t.series, mode=$('x-mode').value, phase=$('phase').value, start=mode==='recovery'?recoveryStart(s):0;
 if(!s.time_s||start===null)return null;
 const x=[],y=[],customdata=[];let previous=-2;
 for(let i=0;i<s.time_s.length;i++){
  const p=s.phase?.[i]||'unknown';if(phase!=='all'&&!(phase==='recovery'?recoveryPhases.has(p):phase===p))continue;
  const xx=mode==='depth'?s.depth_mm?.[i]:s.time_s[i]-start, yy=value(s,key,i);
  if(!finite(xx)||!finite(yy))continue;
  if(previous!==i-1&&x.length){x.push(null);y.push(null);customdata.push(null);}
  x.push(xx);y.push(yy);customdata.push([s.time_s[i],esc(p),s.depth_mm?.[i],s.tilt_deg?.[i],i]);previous=i;
 }
 if(!x.length)return null;
 return {type:'scatter',mode:'lines',name:esc(label(t))+(!t.eligible?' [INVALID / PARTIAL]':''),x,y,customdata,
  line:{color:color(t),width:t.eligible?1.5:2,dash:t.eligible?(t.summary.recovery==='realign'?'dash':'solid'):'dot',simplify:false},connectgaps:false,
  hovertemplate:'%{fullData.name}<br>Time: %{customdata[0]:.4f} s<br>Phase: %{customdata[1]}<br>Depth: %{customdata[2]:.4f} mm<br>Value: %{y:.6g}<extra></extra>'};
}
function phaseShapes(t){
 if($('x-mode').value==='depth'||!t?.series.time_s)return [];
 const s=t.series, start=$('x-mode').value==='recovery'?recoveryStart(s):0;if(start===null)return [];
 const shapes=[];let i=0;
 while(i<s.time_s.length){let j=i;while(j+1<s.time_s.length&&s.phase?.[j+1]===s.phase?.[i])j++;
  shapes.push({type:'rect',xref:'x',yref:'paper',x0:(i?s.time_s[i-1]:0)-start,x1:s.time_s[j]-start,y0:0,y1:1,fillcolor:recoveryPhases.has(s.phase?.[i])?'#def1ec':'#e5edf8',opacity:shapes.length%2?.38:.65,line:{width:0},layer:'below',label:{text:esc(s.phase?.[i]||''),textposition:'top left',font:{size:10,color:'#657b82'}}});i=j+1;
 }return shapes;
}
async function profiles(){
 const ts=chosen();const missing=ts.filter(t=>!t.series.time_s).length;
 $('plot-warning').innerHTML=(ts.some(t=>!t.eligible)?'<div class="notice bad">Diagnostic view includes partial or invalid traces. These are not valid recovery-cost evidence.</div>':'')+(missing?`<div class="notice">${missing} selected trial(s) have no raw samples. See the summary comparison and original figures.</div>`:'');
 const motion=$('x-mode').value==='depth'?'tilt_deg':'depth_mm';
 for(const [id,key,title] of [['force-chart',$('signal').value,signals[$('signal').value][1]],['torque-chart','torque_norm_nm','Contact torque magnitude [N m]'],['motion-chart',motion,motion==='depth_mm'?'Actual insertion depth [mm]':'Actual tilt [°]']]){
  const traces=ts.map(t=>makeTrace(t,key)).filter(Boolean);const layout=baseLayout(title,xTitle());layout.shapes=ts.length===1?phaseShapes(ts[0]):[];
  const budget=key==='force_norm_n'?D.manifest.parameters?.force_budget:key==='torque_norm_nm'?D.manifest.parameters?.torque_budget:null;
  if(finite(budget))layout.shapes.push({type:'line',xref:'paper',x0:0,x1:1,y0:budget,y1:budget,line:{color:'#b87523',dash:'dot',width:1},label:{text:'Evaluation budget',font:{size:10},textposition:'top right'}});
  if(!traces.length)layout.annotations=[{text:'No recorded samples for this selection / phase / axis.',xref:'paper',yref:'paper',x:.5,y:.5,showarrow:false}];
  await Plotly.react($(id),traces,layout,config);
  if(!$(id)._viewerEvents){$(id)._viewerEvents=true;
   $(id).on('plotly_hover',event=>{const p=event.points[0],c=p.customdata;if(c)$('sample').textContent=`${p.data.name.replace(/&[^;]+;/g,' ')} | t = ${fmt(c[0])} s | ${c[1]} | depth = ${fmt(c[2])} mm | tilt = ${fmt(c[3])}° | value = ${fmt(p.y)} | sample ${c[4]}`;});
   $(id).on('plotly_relayout',async event=>{if(syncZoom||$('x-mode').value==='depth')return;let update;
    if(event['xaxis.autorange'])update={'xaxis.autorange':true};else if(event['xaxis.range[0]']!==undefined)update={'xaxis.range':[event['xaxis.range[0]'],event['xaxis.range[1]']]};else if(Array.isArray(event['xaxis.range']))update={'xaxis.range':event['xaxis.range']};else return;
    syncZoom=true;try{await Promise.all(['force-chart','torque-chart','motion-chart'].filter(other=>other!==id).map(other=>Plotly.relayout($(other),update)));}finally{syncZoom=false;}
   });
  }
 }
}
function safeCost(t,k){return !D.phase2&&!t.eligible&&(k in metrics||k==='time_to_clear_s')?null:t.summary[k];}
function drawTable(){
 const list=[...D.trials].sort((a,b)=>{const aa=safeCost(a,sortKey),bb=safeCost(b,sortKey);if(aa==null)return bb==null?0:1;if(bb==null)return -1;return (finite(aa)&&finite(bb)?aa-bb:String(aa).localeCompare(String(bb)))*(sortAscending?1:-1);});
 $('summary-table').innerHTML='<caption style="text-align:left;padding:10px 0">All trial outcomes · click a column heading to sort</caption><thead><tr>'+columns.map(([k,n])=>`<th><button data-sort="${k}">${esc(n)} ${sortKey===k?(sortAscending?'↑':'↓'):''}</button></th>`).join('')+'</tr></thead><tbody>'+list.map(t=>'<tr>'+columns.map(([k])=>{let v=safeCost(t,k);if(k==='penetration_screen_passed')v=v===true?'Passed':v===false?'Review required':'—';return `<td class="${!t.eligible||k==='penetration_screen_passed'&&v==='Review required'?'bad-text':''}">${esc(fmt(v))}</td>`;}).join('')+'</tr>').join('')+'</tbody>';
 $('summary-table').querySelectorAll('[data-sort]').forEach(b=>b.onclick=()=>{sortAscending=sortKey===b.dataset.sort?!sortAscending:true;sortKey=b.dataset.sort;drawTable();});
}
async function compare(){
 const key=$('metric').value, ts=chosen().filter(t=>t.eligible&&(!$('screen-only').checked||t.summary.penetration_screen_passed===true));
 const available=ts.filter(t=>finite(t.summary[key]));
 const traces=available.length?[{type:'bar',x:available.map(t=>esc(label(t))),y:available.map(t=>t.summary[key]),marker:{color:available.map(color)},customdata:available.map(t=>[esc(t.status),t.summary.penetration_screen_passed===false?'Overlap review required':'']),hovertemplate:'%{x}<br>%{y:.6g}<br>%{customdata[0]}<br>%{customdata[1]}<extra></extra>'}]:[];
 const layout=baseLayout(metrics[key],'Scenario / recovery');layout.margin.b=95;if(!available.length)layout.annotations=[{text:'No eligible recorded costs for this selection.',xref:'paper',yref:'paper',x:.5,y:.5,showarrow:false}];
 await Plotly.react($('cost-chart'),traces,layout,config);drawTable();
}
async function recoverability(){
 const rows=(D.phase2||[]).filter(c=>$('checkpoint-family').value==='all'||c.family===$('checkpoint-family').value);
 const traces=[[1,'#13866b','Safe policy observed'],[0,'#b23f4c','Tested policies failed'],[null,'#8b98a1','Unknown / numerical review']].map(([value,c,name])=>{
  const points=rows.filter(r=>r.reached&&r.Y_R_tested===value);
  return {type:'scatter',mode:'markers',name,x:points.map(p=>p.depth_mm),y:points.map(p=>p.force_n),marker:{color:c,size:10,symbol:value===null?'x':'circle'},customdata:points.map(p=>[esc(p.trajectory_id),esc(p.family),p.time_s,esc(p.label_reason)]),hovertemplate:'%{customdata[0]} · %{customdata[1]}<br>Depth %{x:.4f} mm · Force %{y:.4f} N<br>Time %{customdata[2]:.4f} s<br>%{customdata[3]}<extra></extra>'};
 });
 await Plotly.react($('checkpoint-chart'),traces,baseLayout('Insertion-state net force [N]','Actual checkpoint depth [mm]'),config);
 $('checkpoint-table').innerHTML='<thead><tr><th>Trajectory</th><th>Family</th><th>Requested depth [mm]</th><th>Actual depth [mm]</th><th>Force [N]</th><th>Y_R_tested</th><th>Reason</th><th>Policy outcomes</th></tr></thead><tbody>'+rows.map(r=>'<tr>'+[r.trajectory_id,r.family,r.requested_depth_mm,r.depth_mm,r.force_n,r.Y_R_tested===null?'Unknown':r.Y_R_tested,!r.parent_valid?'Parent invalid/incomplete':r.label_reason,r.probes.map(p=>p.policy+': '+p.status).join('; ')].map(v=>'<td>'+esc(fmt(v))+'</td>').join('')+'</tr>').join('')+'</tbody>';
}
function refresh(){drawing=drawing.catch(()=>{}).then(async()=>{if(activeTab==='profiles')await profiles();if(activeTab==='compare')await compare();if(activeTab==='recoverability')await recoverability();});drawing.catch(error=>{$('plot-warning').textContent='Viewer error: '+error.message;console.error(error);});return drawing;}
function tab(name){activeTab=name;for(const key of ['profiles','compare','source','setup','recoverability'])$(key).hidden=key!==name;$('selection').hidden=!['profiles','compare'].includes(name);document.querySelectorAll('[data-tab]').forEach(b=>b.classList.toggle('active',b.dataset.tab===name));return refresh();}
document.querySelectorAll('[data-tab]').forEach(b=>b.onclick=()=>tab(b.dataset.tab));
$('include-invalid').onchange=()=>{if(!$('include-invalid').checked)D.trials.filter(t=>!t.eligible).forEach(t=>selected.delete(t.id));choices();refresh();};
$('select-completed').onclick=()=>{selected.clear();completed.forEach(t=>selected.add(t.id));choices();refresh();};
$('clear-selection').onclick=()=>{selected.clear();choices();refresh();};
for(const id of ['x-mode','phase','signal','metric','screen-only'])$(id).onchange=refresh;
$('reset-zoom').onclick=()=>{for(const id of ['force-chart','torque-chart','motion-chart'])if($(id).data)Plotly.relayout($(id),{'xaxis.autorange':true,'yaxis.autorange':true});};
$('export-selection').onclick=()=>{const quote=v=>'"'+String(v??'').replace(/"/g,'""')+'"';const keys=columns.map(c=>c[0]);const csv=[keys,...chosen().map(t=>keys.map(k=>safeCost(t,k)))].map(row=>row.map(quote).join(',')).join('\r\n');const url=URL.createObjectURL(new Blob([csv],{type:'text/csv;charset=utf-8'}));const a=document.createElement('a');a.href=url;a.download='selected_trial_summary.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
function inlineMarkdown(text){return esc(text).replace(/!\[([^\]]*)\]\(([^)]+)\)/g,(_,alt,path)=>D.assets[path]?`<img alt="${alt}" src="${D.assets[path]}">`:alt).replace(/\[([^\]]*)\]\(([^)]+)\)/g,(_,name,path)=>D.assets[path]?`<a download="${esc(path)}" href="${D.assets[path]}">${name}</a>`:name).replace(/\*\*([^*]+)\*\*/g,'<strong>$1</strong>').replace(/`([^`]+)`/g,'<code>$1</code>');}
function markdown(text){const out=[];let table=false,code=false;for(const line of text.split('\n')){if(line.startsWith('```')){if(table){out.push('</tbody></table>');table=false;}out.push(code?'</pre>':'<pre>');code=!code;continue;}if(code){out.push(esc(line)+'\n');continue;}if(line.startsWith('|')){if(/^\|[\s:|\-]+$/.test(line))continue;const cells=line.split('|').slice(1,-1);if(!table){out.push('<table><tbody>');table=true;}out.push('<tr>'+cells.map(c=>'<td>'+inlineMarkdown(c.trim())+'</td>').join('')+'</tr>');continue;}if(table){out.push('</tbody></table>');table=false;}const h=line.match(/^(#{1,3}) (.*)/);out.push(h?`<h${h[1].length}>${inlineMarkdown(h[2])}</h${h[1].length}>`:line?'<p>'+inlineMarkdown(line)+'</p>':'');}if(table)out.push('</tbody></table>');if(code)out.push('</pre>');return out.join('');}
$('report').innerHTML=markdown(D.report);$('report-raw').textContent=D.report;
for(const [name,url] of Object.entries(D.assets)){const a=document.createElement('a');a.href=url;a.download=name;a.textContent=name;$('downloads').append(a);if(name.endsWith('.png')){const details=document.createElement('details');const heading=document.createElement('summary');heading.textContent=name;const img=document.createElement('img');img.src=url;img.className='original';img.alt=name;details.append(heading,img);$('original-images').append(details);}}
$('manifest').textContent=JSON.stringify(D.manifest,null,2);
const parameters={...(D.manifest.parameters||{}),physics_device:D.manifest.device||'Not recorded',force_frame:D.manifest.force_frame,torque_origin:D.manifest.torque_origin,grasp:D.manifest.grasp,controller:D.manifest.controller,peg_diameter_mm:D.manifest.peg_diameter_mm,peg_length_mm:D.manifest.peg_length_mm,hole_depth_mm:D.manifest.hole_depth_mm};
$('parameters').innerHTML='<tbody>'+Object.entries(parameters).map(([k,v])=>`<tr><th>${esc(k)}</th><td>${esc(fmt(v))}</td></tr>`).join('')+'</tbody>';
if(D.phase2){$('phase2-tab').hidden=false;options($('checkpoint-family'),Object.fromEntries(scenarios.map(s=>[s,s])));$('checkpoint-family').onchange=refresh;document.querySelector('[data-tab=compare]').textContent='Insertion characterization';$('compare').querySelector('h2').textContent='Insertion characterization';$('compare').querySelector('p').textContent='Reference-insertion metrics from the saved dataset. Invalid attempts remain diagnostic and are excluded from comparison bars.';}
choices();tab(D.phase2?'recoverability':D.trials.some(t=>t.series.time_s?.length)?'profiles':'compare').then(()=>{document.body.dataset.viewerReady='true';});
