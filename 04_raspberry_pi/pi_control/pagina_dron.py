"""Pagina web del dron (ESP32_UAV). La sirve seguidor.py en /dron."""

PAGINA_DRON = r"""<!doctype html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Dron octo</title>
<style>
:root{--fondo:#0f1115;--panel:#181b22;--borde:#2a2f3a;--texto:#e6e8ec;--suave:#9aa3b2;--azul:#4c9aff;
--verde:#3ccf7a;--ambar:#f2b640;--rojo:#ff5a5a;--roll:#4c9aff;--pitch:#f2b640}
*{box-sizing:border-box}body{margin:0;background:var(--fondo);color:var(--texto);font:14px/1.4 system-ui,sans-serif}
header{display:flex;flex-wrap:wrap;align-items:center;gap:10px;padding:12px 16px;border-bottom:1px solid var(--borde)}
header h1{font-size:18px;margin:0 10px 0 0}header a{color:var(--azul);margin-left:auto}
.chip{padding:3px 10px;border-radius:999px;font-weight:600;font-size:12px;background:#2a2f3a}
.ok{background:#173d2a;color:var(--verde)}.mal{background:#43201f;color:var(--rojo)}.aviso{background:#45381a;color:var(--ambar)}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(330px,1fr));gap:12px;padding:12px 16px}
section{background:var(--panel);border:1px solid var(--borde);border-radius:10px;padding:12px}
section h2{font-size:13px;text-transform:uppercase;letter-spacing:.05em;color:var(--suave);margin:0 0 10px}
.grande{display:flex;gap:18px;flex-wrap:wrap}.grande div{min-width:90px}.grande b{display:block;font-size:30px;font-variant-numeric:tabular-nums}
.grande span{color:var(--suave);font-size:12px}
canvas{width:100%;display:block;background:#12151b;border-radius:8px}
.botones{display:grid;grid-template-columns:1fr 1fr;gap:8px}
button{font:600 14px system-ui;padding:11px 8px;border-radius:8px;border:1px solid var(--borde);background:#232836;color:var(--texto);cursor:pointer}
button:hover{border-color:var(--azul)}button:disabled{opacity:.4;cursor:not-allowed}
button.peligro{background:#5a1f1f;border-color:#8a2b2b}button.parada{grid-column:1/-1;background:#b3261e;border-color:#e0453d;font-size:18px;padding:16px}
.paso{font-size:12px;color:var(--suave);margin:8px 0 0}
#consola{height:220px;overflow:auto;font:12px/1.45 ui-monospace,monospace;background:#12151b;border-radius:8px;padding:8px;white-space:pre-wrap}
#consola .yo{color:var(--azul)}#consola .malo{color:var(--rojo)}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}td{padding:3px 4px;border-bottom:1px solid var(--borde)}
#logs a{color:var(--azul)}.ancho{grid-column:1/-1}
@media (max-width:700px){main{padding:10px}}
</style></head><body>
<header><h1>Dron octo · ESP32_UAV</h1>
<span id="cx" class="chip">…</span><span id="fase" class="chip">…</span><span id="cal" class="chip">…</span>
<span id="edad" class="chip">…</span><a href="/robot">Robot completo</a> <a href="/">← Ball and plate</a></header>
<main>
<section><h2>Inclinación</h2>
<div class="grande"><div><span>Roll (X)</span><b id="roll">–</b></div><div><span>Pitch (Y)</span><b id="pitch">–</b></div>
<div><span>Inclinación total</span><b id="incl">–</b></div><div><span>Aceleración</span><b id="acc">–</b></div></div>
<canvas id="nivel" height="260"></canvas>
<p class="paso">Punto = hacia dónde se inclina el dron (visto desde arriba, M8 al frente). Círculos: 2°, 5° y 12° (corte).</p></section>
<section><h2>Altura (láser VL53L0X)</h2>
<div class="grande"><div><span>Altura</span><b id="alt">–</b></div><div><span>Lectura del láser</span><b id="lmm">–</b></div>
<div><span>Mínimo</span><b id="lmin">–</b></div><div><span>Máximo</span><b id="lmax">–</b></div></div>
<canvas id="galt" height="150"></canvas>
<div class="botones" style="margin-top:8px"><button onclick="o('cero_altura')">Poner altura en 0 (y borrar mín/máx)</button></div>
<p class="paso" id="lnota">La altura es vertical: se corrige con la inclinación. El láser está en el brazo de M8 y mira hacia abajo.</p></section>
<section><h2>Roll y pitch · últimos 30 s</h2><canvas id="graf" height="300"></canvas>
<p class="paso"><span style="color:var(--roll)">■ roll</span> &nbsp; <span style="color:var(--pitch)">■ pitch</span> &nbsp; líneas rojas: corte a ±12°</p></section>
<section><h2>Motores (µs)</h2><canvas id="mot" height="300"></canvas>
<p class="paso">Vista desde arriba. Verde 1180 → rojo 1500 µs (2000 con el firmware de vuelo). En VISTA son los PWM calculados; las salidas siguen en 1000.</p></section>
<section><h2>Mandos</h2>
<div class="botones">
<button onclick="o('cal')">1 · Calibrar</button><button onclick="o('vista')">Vista (sin potencia)</button>
<button onclick="o('arm')">2 · Armar (10 s)</button>
<button class="peligro" onclick="iniciar()">3 · Armar e iniciar motores</button>
<button onclick="o('parar')">4 · Parar</button><button onclick="o('marca')">Marca</button>
<button onclick="o('log')">Exportar log (8 s)</button><button onclick="cargarLogs()">Actualizar logs</button>
<button class="parada" onclick="o('!')">PARADA INMEDIATA (Espacio o Esc)</button></div>
<p class="paso">Orden de uso: dron nivelado y quieto → 1 Calibrar (5 s) → Vista para revisar signos con las LiPo desconectadas →
Parar → con banco asegurado: 3 Armar e iniciar (arma y arranca solo, sin que se venzan los 10 s). Tras parar, Exportar log y esperar “LOG FIN”.</p>
<div id="logs" class="paso"></div></section>
<section><h2>Vuelo en la guía</h2>
<div class="grande"><div><span>Colectivo actual</span><b id="col">–</b></div><div><span>Objetivo</span><b id="colobj">–</b></div></div>
<div class="botones" style="margin-top:8px">
<button class="peligro" onclick="flotar('flotar',1750)">Flotar (sube a 1750 µs)</button>
<button class="peligro" onclick="flotar('maximo',1950)">Máximo (1950 µs, 20 s)</button>
<button onclick="o('mantener')">Mantener</button>
<button onclick="o('subir')">Subir +25 µs</button><button onclick="o('bajar')">Bajar −25 µs</button>
<button onclick="o('aterrizar')">Aterrizar (baja a 1300)</button></div>
<p class="paso" id="vnota">Solo con el firmware <b>captura_octo_m8_vuelo_guia</b> y con los motores en CONTROL. El colectivo sube 20 µs/s
(sobre 1650 µs va lento, 8 µs/s: de 1650 a 1750 tarda 12,5 s) y baja 60 µs/s. A máxima potencia (sobre 1850) aguanta 20 s y baja sola a 1750. Pulsa <b>Mantener</b> apenas veas que despega y <b>Marca</b> para guardar el momento.
Si la Pi deja de responder, el dron baja solo a 1300. Espacio = parada inmediata (corta en el aire).</p></section>
<section class="ancho"><h2>Mensajes del ESP32</h2><div id="consola"></div></section>
</main>
<script>
const $=id=>document.getElementById(id);
let hist=[],desde=0,ultimoMsg=0,est=null;
function flotar(orden,us){if(!est||!est.firmware_vuelo){alert('El firmware grabado no tiene modo vuelo.');return}
if(confirm('El dron va a subir el colectivo hasta '+us+' µs.\n\n¿Guía libre con tope arriba, nadie cerca de las hélices, baterías cargadas y el corte físico a mano?'))o(orden)}
function o(n){fetch('/dron/orden?o='+encodeURIComponent(n)).then(r=>r.text()).then(t=>{if(t!=='OK')agregar([['',t]],true)})}
async function iniciar(){if(!confirm('Se encienden los 8 motores.\n\n¿Banco asegurado, hélices despejadas y nadie cerca?'))return;
 // arma e inicia seguido: la confirmacion va ANTES, asi no se vencen los 10 s del armado
 await fetch('/dron/orden?o=arm');await new Promise(r=>setTimeout(r,500));o('iniciar')}
addEventListener('keydown',e=>{if(e.code==='Space'||e.key==='Escape'){e.preventDefault();o('!')}});
function agregar(m,error){const c=$('consola');for(const [h,t] of m){const d=document.createElement('div');
 d.textContent=(h?h+'  ':'')+t;if(t.startsWith('> '))d.className='yo';if(error||/CORTE|RECHAZ|FALLO|INVALIDA|ERROR/.test(t))d.className='malo';c.appendChild(d)}
 while(c.children.length>300)c.firstChild.remove();c.scrollTop=c.scrollHeight}
function chip(el,txt,cls){el.textContent=txt;el.className='chip '+cls}
function ajustar(c){const r=c.getBoundingClientRect(),k=devicePixelRatio||1;if(c.width!==Math.round(r.width*k)){c.width=Math.round(r.width*k);c.height=Math.round(c.getAttribute('height')*k)}
 const x=c.getContext('2d');x.setTransform(1,0,0,1,0,0);x.scale(k,k);return [x,r.width,c.height/k]}
function dibujarNivel(){const [x,w,h]=ajustar($('nivel'));x.clearRect(0,0,w,h);const cx=w/2,cy=h/2,R=Math.min(w,h)/2-14,esc=R/14;
 for(const [g,col] of [[2,'#3ccf7a'],[5,'#f2b640'],[12,'#ff5a5a']]){x.strokeStyle=col;x.lineWidth=1.5;x.beginPath();x.arc(cx,cy,g*esc,0,7);x.stroke()}
 x.strokeStyle='#2a2f3a';x.beginPath();x.moveTo(cx-R,cy);x.lineTo(cx+R,cy);x.moveTo(cx,cy-R);x.lineTo(cx,cy+R);x.stroke();
 x.fillStyle='#9aa3b2';x.font='12px system-ui';x.fillText('M8 (frente, +X)',cx+4,14);x.fillText('M7 (+Y)',6,cy-6);
 if(!est)return;
 // pitch>0 = baja M8 (frente, arriba en la vista); roll<0 = baja M7 (izquierda)
 const dx=est.roll*esc, dy=-est.pitch*esc;
 x.strokeStyle='#4c9aff';x.lineWidth=2;x.beginPath();x.moveTo(cx,cy);x.lineTo(cx+dx,cy+dy);x.stroke();
 x.fillStyle='#e6e8ec';x.beginPath();x.arc(cx+dx,cy+dy,7,0,7);x.fill()}
function dibujarGraf(){const [x,w,h]=ajustar($('graf'));x.clearRect(0,0,w,h);const iz=34,ab=18,ah=h-ab-6,lim=15,ahora=Date.now()/1000;
 const Y=v=>6+ah/2-v/lim*ah/2,X=t=>iz+(w-iz-6)*(1-(ahora-t)/30);
 x.font='11px system-ui';x.fillStyle='#9aa3b2';x.strokeStyle='#2a2f3a';x.lineWidth=1;
 for(const v of [-10,-5,0,5,10]){x.beginPath();x.moveTo(iz,Y(v));x.lineTo(w-6,Y(v));x.stroke();x.fillText((v>0?'+':'')+v+'°',2,Y(v)+4)}
 for(let s=0;s<=30;s+=10){x.fillText('-'+s+' s',X(ahora-s)-14,h-4)}
 x.strokeStyle='#ff5a5a';x.setLineDash([4,4]);for(const v of [12,-12]){x.beginPath();x.moveTo(iz,Y(v));x.lineTo(w-6,Y(v));x.stroke()}x.setLineDash([]);
 for(const [i,col] of [[1,'#4c9aff'],[2,'#f2b640']]){x.strokeStyle=col;x.lineWidth=1.8;x.beginPath();let pen=false,prev=0;
  for(const p of hist){if(ahora-p[0]>30)continue;const xx=X(p[0]),yy=Y(Math.max(-lim,Math.min(lim,p[i])));
   if(pen&&p[0]-prev<0.5)x.lineTo(xx,yy);else x.moveTo(xx,yy);pen=true;prev=p[0]}x.stroke()}}
const POS=[[.7071,-.7071],[-.7071,-.7071],[-.7071,.7071],[.7071,.7071],[0,-1],[-1,0],[0,1],[1,0]];// M1..M8 (x frente, y izquierda)
function dibujarMot(){const [x,w,h]=ajustar($('mot'));x.clearRect(0,0,w,h);const cx=w/2,cy=h/2,R=Math.min(w,h)/2-40;
 x.strokeStyle='#8a2b2b';x.lineWidth=3;
 for(let i=0;i<8;i++){const [mx,my]=POS[i];x.beginPath();x.moveTo(cx,cy);x.lineTo(cx-my*R,cy-mx*R);x.stroke()}
 for(let i=0;i<8;i++){const [mx,my]=POS[i],px=cx-my*R,py=cy-mx*R,us=est?est.motores[i]:1000,tope=est&&est.firmware_vuelo?2000:1500,f=Math.max(0,Math.min(1,(us-1180)/(tope-1180)));
  x.fillStyle=us<=1000?'#2a2f3a':`rgb(${Math.round(60+195*f)},${Math.round(207-150*f)},90)`;x.beginPath();x.arc(px,py,26,0,7);x.fill();
  x.fillStyle='#0f1115';x.font='600 12px system-ui';x.textAlign='center';x.fillText('M'+(i+1),px,py-3);x.fillText(us,px,py+12)}
 x.textAlign='left';x.fillStyle='#9aa3b2';x.font='12px system-ui';x.fillText('frente',cx-16,12)}
function dibujarAlt(){const [x,w,h]=ajustar($('galt'));x.clearRect(0,0,w,h);const iz=38,ahora=Date.now()/1000;
 const v=hist.filter(p=>ahora-p[0]<=30&&p[4]!==null&&p[4]!==undefined).map(p=>p[4]);
 let lo=Math.min(-1,...v),hi=Math.max(1,...v);const m=(hi-lo)*.15;lo-=m;hi+=m;
 const Y=a=>6+(h-24)*(1-(a-lo)/(hi-lo)),X=t=>iz+(w-iz-6)*(1-(ahora-t)/30);
 x.font='11px system-ui';x.fillStyle='#9aa3b2';x.strokeStyle='#2a2f3a';
 for(const a of [lo+m,(lo+hi)/2,hi-m]){x.beginPath();x.moveTo(iz,Y(a));x.lineTo(w-6,Y(a));x.stroke();x.fillText(a.toFixed(1)+' cm',0,Y(a)+4)}
 x.strokeStyle='#3ccf7a';x.lineWidth=1.8;x.beginPath();let pen=false,prev=0;
 for(const p of hist){if(ahora-p[0]>30||p[4]===null||p[4]===undefined){pen=false;continue}
  const xx=X(p[0]),yy=Y(p[4]);if(pen&&p[0]-prev<0.5)x.lineTo(xx,yy);else x.moveTo(xx,yy);pen=true;prev=p[0]}x.stroke()}
function cargarLogs(){fetch('/dron/logs').then(r=>r.json()).then(l=>{$('logs').innerHTML=l.length?'Logs guardados: '+l.map(n=>`<a href="/dron/log?f=${n}">${n}</a>`).join(' · '):'Aún no hay logs exportados.'})}
async function ciclo(){try{const d=await (await fetch('/dron/estado?desde='+desde+'&msg='+ultimoMsg)).json();est=d.estado;$('col').textContent=est.colectivo!=null?est.colectivo+' µs':'–';$('colobj').textContent=est.colectivo_obj!=null?est.colectivo_obj+' µs':'–';$('vnota').style.opacity=est.firmware_vuelo?1:.5;
 for(const p of d.historial)hist.push(p);if(d.historial.length)desde=d.historial[d.historial.length-1][0];
 const lim=Date.now()/1000-35;while(hist.length&&hist[0][0]<lim)hist.shift();
 if(d.mensajes.length){agregar(d.mensajes.map(m=>[m[1],m[2]]));ultimoMsg=d.mensajes[d.mensajes.length-1][0]}
 const e=est,viejo=e.edad_s===null||e.edad_s>1;
 chip($('cx'),e.conectado?'conectado':'sin conexión',e.conectado?'ok':'mal');
 chip($('fase'),e.fase,e.fase==='CONTROL'||e.fase==='RAMPA'?'mal':e.fase==='ARMADO'?'aviso':'ok');
 chip($('cal'),e.calibrado?'calibrado':'sin calibrar',e.calibrado?'ok':'aviso');
 chip($('edad'),viejo?'sin lecturas (con el firmware original solo llegan en VISTA, ARMADO o CONTROL)':'lecturas al día',viejo?'aviso':'ok');
 $('roll').textContent=(e.roll>=0?'+':'')+e.roll.toFixed(2)+'°';$('pitch').textContent=(e.pitch>=0?'+':'')+e.pitch.toFixed(2)+'°';
 $('alt').textContent=e.altura_cm===null?'–':(e.altura_cm>=0?'+':'')+e.altura_cm.toFixed(1)+' cm';
 $('lmm').textContent=e.laser_mm===null?(e.laser_existe?(e.laser_estado||'sin lectura'):'–'):e.laser_mm+' mm';
 if(e.laser_existe&&!e.laser_ok)$('lnota').textContent='Láser: '+e.laser_estado+' (última medida '+e.laser_crudo_mm+' mm). Señal débil o fuera de rango = no ve una superficie cerca.';
 $('lmin').textContent=e.laser_min_mm===null?'–':e.laser_min_mm+' mm';$('lmax').textContent=e.laser_max_mm===null?'–':e.laser_max_mm+' mm';
 if(!e.laser_existe)$('lnota').textContent='Este firmware no manda el láser, o el láser no responde en el bus I2C (ver mensajes).';
 $('incl').textContent=Math.hypot(e.roll,e.pitch).toFixed(2)+'°';$('acc').textContent=e.accel_g?e.accel_g.toFixed(3)+' g':'–';
 if(e.ultimo_log&&!$('logs').textContent.includes(e.ultimo_log))cargarLogs()}catch(err){chip($('cx'),'sin conexión con la Pi','mal')}
 dibujarNivel();dibujarGraf();dibujarMot();dibujarAlt();setTimeout(ciclo,100)}
cargarLogs();ciclo();
</script></body></html>"""
