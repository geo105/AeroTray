"""Pagina web del robot completo (ball and plate + dron; el carrito se mueve aparte).
La sirve seguidor.py en /robot. Solo usa los endpoints que ya existen:
/estado, /comando?n=, /dron/estado, /dron/orden, /video."""

PAGINA_ROBOT = r"""<!doctype html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Robot DSM</title>
<style>
:root{--fondo:#0f1115;--panel:#181b22;--borde:#2a2f3a;--texto:#e6e8ec;--suave:#9aa3b2;--azul:#4c9aff;
--verde:#3ccf7a;--ambar:#f2b640;--rojo:#ff5a5a}
*{box-sizing:border-box}body{margin:0;background:var(--fondo);color:var(--texto);font:14px/1.4 system-ui,sans-serif}
header{display:flex;flex-wrap:wrap;align-items:center;gap:8px;padding:10px 16px;border-bottom:1px solid var(--borde);position:sticky;top:0;background:var(--fondo);z-index:5}
header h1{font-size:18px;margin:0 8px 0 0}header nav{margin-left:auto;display:flex;gap:12px}header a{color:var(--azul)}
.chip{padding:3px 10px;border-radius:999px;font-weight:600;font-size:12px;background:#2a2f3a}
.ok{background:#173d2a;color:var(--verde)}.mal{background:#43201f;color:var(--rojo)}.aviso{background:#45381a;color:var(--ambar)}
#paradaTodo{display:block;width:calc(100% - 32px);margin:12px 16px 0;background:#b3261e;border:1px solid #e0453d;color:#fff;
font:700 20px system-ui;padding:14px;border-radius:10px;cursor:pointer}
#paradaTodo small{display:block;font:500 12px system-ui;opacity:.85}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(340px,1fr));gap:12px;padding:12px 16px}
section{background:var(--panel);border:1px solid var(--borde);border-radius:10px;padding:12px;min-width:0}
section h2{font-size:13px;text-transform:uppercase;letter-spacing:.05em;color:var(--suave);margin:0 0 10px}
.grande{display:flex;gap:16px;flex-wrap:wrap}.grande div{min-width:80px}.grande b{display:block;font-size:26px;font-variant-numeric:tabular-nums}
.grande span{color:var(--suave);font-size:12px}
canvas{width:100%;display:block;background:#12151b;border-radius:8px}
img#video{width:100%;display:block;border-radius:8px;background:#12151b;min-height:120px}
.fila{display:flex;gap:8px;flex-wrap:wrap;margin-top:8px}
button{font:600 14px system-ui;padding:10px 12px;border-radius:8px;border:1px solid var(--borde);background:#232836;color:var(--texto);cursor:pointer}
button:hover{border-color:var(--azul)}button:disabled{opacity:.35;cursor:not-allowed}
button.peligro{background:#5a1f1f;border-color:#8a2b2b}
ol#pasos{list-style:none;margin:0;padding:0;display:grid;gap:8px}
ol#pasos li{display:grid;grid-template-columns:28px 1fr auto;gap:10px;align-items:center;padding:10px;border:1px solid var(--borde);border-radius:8px;background:#141821}
ol#pasos li .n{width:28px;height:28px;border-radius:50%;display:grid;place-items:center;font-weight:700;background:#2a2f3a}
ol#pasos li.hecho .n{background:#173d2a;color:var(--verde)}ol#pasos li.actual{border-color:var(--azul)}
ol#pasos li p{margin:2px 0 0;color:var(--suave);font-size:12px}
ol#pasos li.hecho button{opacity:.6}
.aviso-caja{margin-top:10px;padding:10px;border-radius:8px;background:#45381a;color:var(--ambar);font-size:13px;display:none}
#consola{height:170px;overflow:auto;font:12px/1.45 ui-monospace,monospace;background:#12151b;border-radius:8px;padding:8px;white-space:pre-wrap}
#consola .yo{color:var(--azul)}#consola .malo{color:var(--rojo)}
.ancho{grid-column:1/-1}.leyenda{font-size:12px;color:var(--suave);margin:6px 0 0}
.leyenda i{display:inline-block;width:10px;height:3px;vertical-align:middle;margin:0 4px 0 10px}
@media (max-width:700px){main{padding:10px}#paradaTodo{width:calc(100% - 20px);margin:10px}}
</style></head><body>
<header><h1>Robot DSM</h1>
<span id="cPlaca" class="chip">placa …</span><span id="cBola" class="chip">bola …</span>
<span id="cDron" class="chip">dron …</span><span id="cCal" class="chip">…</span><span id="cFps" class="chip">…</span>
<nav><a href="/">Ball and plate</a><a href="/dron">Dron</a></nav></header>
<button id="paradaTodo" onclick="pararTodo()">PARAR TODO (motores + placa)<small>tecla Espacio · antes, si la rótula está suelta, sostén el dron</small></button>
<main>
<section><h2>Secuencia</h2>
<ol id="pasos">
<li id="p1"><span class="n">1</span><div><b>Placa plana</b><p>Placa en neutro fijo, quieta.</p></div><button onclick="paso1()">Neutro</button></li>
<li id="p2"><span class="n">2</span><div><b>Calibrar dron</b><p>Dron nivelado en el apoyo y sin tocarlo 6 s.</p></div><button onclick="paso2()">Calibrar</button></li>
<li id="p3"><span class="n">3</span><div><b>PID de la placa</b><p>Activa el PID y pon la bola en el centro.</p></div><button onclick="paso3()">Iniciar PID</button></li>
<li id="p4"><span class="n">4</span><div><b>Encender dron</b><p>Inicio 1 + Inicio 2. Cuando los motores suenen parejo, retira el apoyo despacio.</p></div><button class="peligro" onclick="paso4()">Encender</button></li>
<li id="p5"><span class="n">5</span><div><b>Funcionando</b><p>El carrito ya se puede mover. Vigila la bola y la inclinación.</p></div><span id="tiempo" class="chip">–</span></li>
</ol>
<div class="fila"><button onclick="pararDron()">Parar dron (4)</button><button onclick="cmd(6)">Placa neutro (6)</button>
<button onclick="cmd(5)">PID placa (5)</button><button onclick="orden('marca')">Marca en el log</button></div>
<div id="aviso" class="aviso-caja"></div>
<h2 style="margin-top:14px">Vuelo en la guía (después del paso 4)</h2>
<p class="leyenda">Colectivo: <b id="vcol">–</b> · objetivo <b id="vobj">–</b>. «Flotar» sube solo y despacio: 25 µs/s hasta 1650 y 8 µs/s de 1650 a 1750.
Para ir paso a paso usa «Subir +25» y espera a que el colectivo llegue al objetivo antes de volver a pulsar.</p>
<div class="fila"><button class="peligro" onclick="vuelo('flotar')">Flotar (sube gradual a 1750 µs)</button>
<button onclick="vuelo('subir')">Subir +25 µs</button><button onclick="vuelo('bajar')">Bajar −25 µs</button>
<button onclick="vuelo('mantener')">Mantener altura</button><button onclick="vuelo('aterrizar')">Aterrizar (baja a 1300)</button></div>
</section>
<section><h2>Cámara de la placa</h2>
<img id="video" alt="cámara de la placa">
<div class="fila"><button onclick="vista('camara')">Cámara</button><button onclick="vista('placa')">Placa recta</button><button onclick="vista('')">Ocultar (más fps)</button></div></section>
<section><h2>Bola</h2>
<div class="grande"><div><span>X</span><b id="bx">–</b></div><div><span>Y</span><b id="by">–</b></div>
<div><span>Al centro</span><b id="bd">–</b></div><div><span>Servos X / Y</span><b id="bs" style="font-size:18px">–</b></div></div>
<canvas id="placa" height="230"></canvas>
<p class="leyenda">Vista de la placa (250 × 220 mm) con la estela de los últimos 3 s. Estado: <b id="bEst">–</b></p></section>
<section><h2>Dron</h2>
<div class="grande"><div><span>Roll</span><b id="dr">–</b></div><div><span>Pitch</span><b id="dp">–</b></div>
<div><span>Inclinación</span><b id="di">–</b></div><div><span>Motores</span><b id="dm" style="font-size:18px">–</b></div></div>
<canvas id="nivel" height="230"></canvas>
<p class="leyenda">Visto desde arriba (M8 al frente, M7 a la izquierda). Círculos: 2°, 5° y 12° (corte de motores).</p></section>
<section class="ancho"><h2>Últimos 30 s</h2><canvas id="graf" height="200"></canvas>
<p class="leyenda"><i style="background:#3ccf7a"></i>bola al centro (mm, eje izq.)<i style="background:#4c9aff"></i>roll<i style="background:#f2b640"></i>pitch (°, eje der.)</p></section>
<section class="ancho"><h2>Mensajes</h2><div id="consola"></div></section>
</main>
<script>
const $=id=>document.getElementById(id);let S=null,U=null,hist=[],ultimoMsg=0,tInicio=null,vistaAct='camara';
function log(t,cls){const c=$('consola'),d=document.createElement('div');d.textContent=new Date().toLocaleTimeString()+'  '+t;if(cls)d.className=cls;c.appendChild(d);c.scrollTop=c.scrollHeight}
async function cmd(n){const r=await (await fetch('/comando?n='+n)).text();log('> comando '+n+': '+r,r.startsWith('OK')?'yo':'malo');return r.startsWith('OK')}
async function orden(o){const r=await (await fetch('/dron/orden?o='+o)).text();log('> '+o+': '+r,r==='OK'?'yo':'malo');return r==='OK'}
function aviso(t){const a=$('aviso');a.textContent=t||'';a.style.display=t?'block':'none'}
function marcar(n){for(let i=1;i<=5;i++){const li=$('p'+i);li.classList.toggle('hecho',i<n);li.classList.toggle('actual',i===n)}}
const dormir=ms=>new Promise(r=>setTimeout(r,ms));
async function paso1(){aviso('');if(await cmd(6))marcar(2)}
async function paso2(){if(!U||!U.conectado){aviso('El dron no está conectado.');return}
 if(U.activo){aviso('El dron tiene los motores encendidos: páralo antes de calibrar.');return}
 aviso('Calibrando: no toques el dron 6 s…');if(!await cmd(1)){aviso('La Pi no pudo mandar cal al dron.');return}
 await dormir(1000);
 for(let i=0;i<60;i++){await dormir(200);if(U&&U.calibrado){aviso('');marcar(3);return}
  if(U&&U.mensaje&&(U.mensaje.startsWith('ORDEN RECHAZADA')||U.mensaje.startsWith('CAL RECHAZADA')))break}
 aviso('No llegó CAL OK. Revisa los mensajes (si dice ORDEN RECHAZADA, el IMU falló: pulsa EN en el NodeMCU).')}
async function paso3(){aviso('');if(await cmd(5)){marcar(4);aviso('PID activo: pon la bola en el centro de la placa.')}}
async function paso4(){if(!U||!U.calibrado){aviso('Primero calibra el dron (paso 2).');return}
 const inc=Math.hypot(U.roll,U.pitch);if(inc>3.8){aviso('El dron está a '+inc.toFixed(1)+'°: nivélalo (menos de 4°) o vuelve a calibrar.');return}
 if(!(S&&S.pi.detectada)&&!confirm('La cámara no ve la bola. ¿Encender el dron igual?'))return;
 if(!confirm('Se encienden los 8 motores.\n\n¿Baterías conectadas, apoyo puesto, hélices despejadas y nadie cerca?'))return;
 aviso('');await cmd(2);
 for(let i=0;i<15&&!(U&&U.fase==='ARMADO');i++)await dormir(100);
 if(!(U&&U.fase==='ARMADO')){aviso('El dron no se armó: '+(U?U.mensaje:''));return}
 await cmd(3);tInicio=Date.now();marcar(5);aviso('Motores encendiendo: cuando suenen parejo, retira el apoyo despacio.');setTimeout(()=>aviso(''),8000)}
async function pararDron(){if(U&&U.activo&&!confirm('¿Ya pusiste el apoyo? Con la rótula suelta el dron cae al apagarse.'))return;await cmd(4);tInicio=null;marcar(3)}
async function vuelo(o){if(!U||U.fase!=='CONTROL'){aviso('Primero enciende el dron (paso 4): el vuelo solo funciona con los motores en CONTROL.');return}
 if(o==='flotar'&&!confirm('El colectivo va a subir despacio hasta 1750 µs y el dron se levantará en la guía.\n\n¿Guía libre con tope arriba, nadie cerca de las hélices y alguien listo con PARAR TODO?'))return;
 aviso('');await orden(o)}
async function pararTodo(){fetch('/dron/orden?o=!');fetch('/comando?n=4');fetch('/comando?n=6');tInicio=null;log('PARAR TODO','malo');marcar(1)}
document.addEventListener('keydown',e=>{if(e.code==='Space'&&e.target.tagName!=='INPUT'){e.preventDefault();pararTodo()}});
function vista(v){vistaAct=v;$('video').src=v?'/video?v='+v+'&t='+Date.now():'';$('video').style.display=v?'block':'none'}
function chip(el,t,c){el.textContent=t;el.className='chip '+c}
function ajustar(c){const r=c.getBoundingClientRect(),k=devicePixelRatio||1;const W=Math.round(r.width*k),H=Math.round(c.getAttribute('height')*k);
 if(c.width!==W||c.height!==H){c.width=W;c.height=H}const x=c.getContext('2d');x.setTransform(k,0,0,k,0,0);return [x,r.width,+c.getAttribute('height')]}
function dibujarPlaca(){const [x,w,h]=ajustar($('placa'));x.clearRect(0,0,w,h);const pw=250,ph=220,e=Math.min((w-20)/pw,(h-20)/ph),cx=w/2,cy=h/2;
 x.strokeStyle='#2a2f3a';x.lineWidth=1;x.strokeRect(cx-pw/2*e,cy-ph/2*e,pw*e,ph*e);
 x.beginPath();x.moveTo(cx-6,cy);x.lineTo(cx+6,cy);x.moveTo(cx,cy-6);x.lineTo(cx,cy+6);x.stroke();
 x.strokeStyle='#2f4a3a';x.beginPath();x.arc(cx,cy,20*e,0,7);x.stroke();
 const ahora=Date.now()/1000,tr=hist.filter(p=>p[0]>ahora-3&&p[1]!=null);
 x.strokeStyle='rgba(60,207,122,.45)';x.lineWidth=2;x.beginPath();tr.forEach((p,i)=>{const X=cx+p[1]*e,Y=cy-p[2]*e;i?x.lineTo(X,Y):x.moveTo(X,Y)});x.stroke();
 if(S&&S.pi.detectada&&S.pi.x_mm!=null){x.fillStyle='#3ccf7a';x.beginPath();x.arc(cx+S.pi.x_mm*e,cy-S.pi.y_mm*e,8,0,7);x.fill()}
 x.fillStyle='#9aa3b2';x.font='11px system-ui';x.fillText('+X',w-24,cy-6);x.fillText('+Y',cx+6,14)}
function dibujarNivel(){const [x,w,h]=ajustar($('nivel'));x.clearRect(0,0,w,h);const cx=w/2,cy=h/2,R=Math.min(w,h)/2-12,esc=R/14;
 x.lineWidth=1;for(const [g,c] of [[2,'#2f4a3a'],[5,'#4a4430'],[12,'#5a2a2a']]){x.strokeStyle=c;x.beginPath();x.arc(cx,cy,g*esc,0,7);x.stroke()}
 x.strokeStyle='#2a2f3a';x.beginPath();x.moveTo(cx-R,cy);x.lineTo(cx+R,cy);x.moveTo(cx,cy-R);x.lineTo(cx,cy+R);x.stroke();
 x.fillStyle='#9aa3b2';x.font='11px system-ui';x.fillText('M8',cx-8,cy-R+10);x.fillText('M7',cx-R+2,cy+4);
 if(U&&U.calibrado){const dx=Math.max(-14,Math.min(14,U.roll))*esc,dy=-Math.max(-14,Math.min(14,U.pitch))*esc,inc=Math.hypot(U.roll,U.pitch);
  x.fillStyle=inc<5?'#3ccf7a':inc<10?'#f2b640':'#ff5a5a';x.beginPath();x.arc(cx+dx,cy+dy,9,0,7);x.fill()}}
function dibujarGraf(){const [x,w,h]=ajustar($('graf'));x.clearRect(0,0,w,h);const iz=36,de=34,ab=16,ah=h-ab-6,ahora=Date.now()/1000,a=ahora-30;
 const X=t=>iz+(t-a)/30*(w-iz-de),Yb=v=>6+ah*(1-Math.min(v,150)/150),Yd=v=>6+ah/2-Math.max(-12,Math.min(12,v))/12*ah/2;
 x.strokeStyle='#2a2f3a';x.fillStyle='#9aa3b2';x.font='10px system-ui';
 for(const v of [0,50,100,150]){const y=Yb(v);x.beginPath();x.moveTo(iz,y);x.lineTo(w-de,y);x.stroke();x.fillText(v,4,y+3)}
 for(const v of [-12,0,12]){x.fillText(v+'°',w-de+4,Yd(v)+3)}
 const serie=(f,col)=>{x.strokeStyle=col;x.lineWidth=1.5;x.beginPath();let p=false;for(const q of hist){if(q[0]<a)continue;const v=f(q);if(v==null){p=false;continue}const px=X(q[0]);p?x.lineTo(px,v):x.moveTo(px,v);p=true}x.stroke()};
 serie(q=>q[1]==null?null:Yb(Math.hypot(q[1],q[2])),'#3ccf7a');serie(q=>q[3]==null?null:Yd(q[3]),'#4c9aff');serie(q=>q[4]==null?null:Yd(q[4]),'#f2b640')}
async function ciclo(){try{
 S=await (await fetch('/estado')).json();U=S.uav;const p=S.pi,e=S.esp32,ahora=Date.now()/1000;
 const bola=p.detectada&&p.x_mm!=null;
 hist.push([ahora,bola?p.x_mm:null,bola?p.y_mm:null,U.calibrado?U.roll:null,U.calibrado?U.pitch:null]);hist=hist.filter(q=>q[0]>ahora-31);
 chip($('cPlaca'),'placa: '+(e.conectado?e.estado:'desconectada'),!e.conectado?'mal':e.estado==='PID'?'ok':'aviso');
 chip($('cBola'),bola?'bola vista':'sin bola',bola?'ok':'aviso');
 chip($('cDron'),'dron: '+(U.conectado?U.fase:'desconectado'),!U.conectado?'mal':U.activo?'ok':'');
 chip($('cCal'),U.calibrado?'calibrado':'sin calibrar',U.calibrado?'ok':'aviso');
 chip($('cFps'),p.fps.toFixed(0)+' fps',p.fps>120?'':'aviso');
 $('bx').textContent=bola?p.x_mm.toFixed(0):'–';$('by').textContent=bola?p.y_mm.toFixed(0):'–';
 $('bd').textContent=bola?Math.hypot(p.x_mm,p.y_mm).toFixed(0)+' mm':'–';$('bs').textContent=e.pulso_x+' / '+e.pulso_y;$('bEst').textContent=e.estado;
 const inc=Math.hypot(U.roll,U.pitch);
 $('dr').textContent=U.calibrado?U.roll.toFixed(1)+'°':'–';$('dp').textContent=U.calibrado?U.pitch.toFixed(1)+'°':'–';
 $('di').textContent=U.calibrado?inc.toFixed(1)+'°':'–';$('di').style.color=inc<5?'':inc<10?'var(--ambar)':'var(--rojo)';
 $('vcol').textContent=U.colectivo!=null?U.colectivo+' µs':'–';$('vobj').textContent=U.colectivo_obj!=null?U.colectivo_obj+' µs':'–';
 const m=U.motores||[];$('dm').textContent=m.length?Math.min(...m)+'–'+Math.max(...m)+' µs':'–';
 $('tiempo').textContent=tInicio&&U.activo?Math.round((Date.now()-tInicio)/1000)+' s':U.activo?'motores ON':'–';
 if(!U.activo&&tInicio&&Date.now()-tInicio>3000){tInicio=null;log('El dron se detuvo: '+U.mensaje,'malo');marcar(3)}
 dibujarPlaca();dibujarNivel();dibujarGraf()}catch(err){chip($('cDron'),'sin conexión con la Pi','mal')}}
async function mensajes(){try{const d=await (await fetch('/dron/estado?desde='+(Date.now()/1000)+'&msg='+ultimoMsg)).json();
 for(const [t,h,txt] of d.mensajes){ultimoMsg=Math.max(ultimoMsg,t);log('dron: '+txt,/RECHAZ|CORTE|FALLO|INVALIDA/.test(txt)?'malo':'')}}catch(e){}}
setInterval(ciclo,150);setInterval(mensajes,700);vista('camara');marcar(1);
fetch('/dron/estado?desde=0&msg=0').then(r=>r.json()).then(d=>{for(const m of d.mensajes)ultimoMsg=Math.max(ultimoMsg,m[0])});
</script></body></html>
"""
