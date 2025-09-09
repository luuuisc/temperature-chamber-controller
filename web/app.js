const btnConnect = document.getElementById('btnConnect');
const btnDisconnect = document.getElementById('btnDisconnect');
const btnSetSP = document.getElementById('btnSetSP');
const btnSetKpF = document.getElementById('btnSetKpF');
const btnSetKpV = document.getElementById('btnSetKpV');

const statusEl = document.getElementById('status');
const tRealEl = document.getElementById('tReal');
const tSPEl = document.getElementById('tSP');
const modeEl = document.getElementById('mode');
const pwmHEl = document.getElementById('pwmH');
const pwmFEl = document.getElementById('pwmF');

const inpSP = document.getElementById('inpSP');
const inpKpF = document.getElementById('inpKpF');
const inpKpV = document.getElementById('inpKpV');

const logEl = document.getElementById('log');

let port = null, reader = null, writer = null, keepReading = false;

function log(msg){
  const t = new Date().toLocaleTimeString();
  logEl.textContent += `[${t}] ${msg}\n`; logEl.scrollTop = logEl.scrollHeight;
}
function setStatus(text, ok=false){
  statusEl.textContent = text;
  statusEl.classList.remove('ok','warn');
  statusEl.classList.add(ok?'ok':'warn');
}
function setModeBadge(mode){
  modeEl.textContent = mode || '—';
  modeEl.className = 'badge';
  if(mode==='HEAT') modeEl.classList.add('mode-HEAT');
  else if(mode==='FAN') modeEl.classList.add('mode-FAN');
  else if(mode==='IDLE') modeEl.classList.add('mode-IDLE');
}

async function connect(){
  try{
    if(!('serial' in navigator)){ alert('Tu navegador no soporta Web Serial. Usa Chrome/Edge.'); return; }
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200 });

    const encoder = new TextEncoderStream();
    encoder.readable.pipeTo(port.writable);
    writer = encoder.writable.getWriter();

    const decoder = new TextDecoderStream();
    port.readable.pipeTo(decoder.writable);
    reader = decoder.readable.getReader();

    btnConnect.disabled = true;
    btnDisconnect.disabled = false;
    btnSetSP.disabled = false;
    btnSetKpF.disabled = false;
    btnSetKpV.disabled = false;

    setStatus('Conectado', true);
    log('Conectado al puerto serie.');

    keepReading = true;
    readLoop();
  }catch(err){
    console.error(err); log('Error al conectar: '+err.message); setStatus('Error de conexión');
  }
}

async function disconnect(){
  try{
    keepReading = false;
    if(reader){ await reader.cancel().catch(()=>{}); reader.releaseLock(); reader=null; }
    if(writer){ writer.releaseLock(); writer=null; }
    if(port){ await port.close(); port=null; }
  }catch(err){ console.error(err); log('Error al desconectar: '+err.message); }
  btnConnect.disabled=false; btnDisconnect.disabled=true;
  btnSetSP.disabled=true; btnSetKpF.disabled=true; btnSetKpV.disabled=true;
  setStatus('Desconectado'); log('Puerto cerrado.');
}

async function readLoop(){
  let buf = '';
  while(keepReading && reader){
    const {value, done} = await reader.read();
    if(done) break;
    if(value){
      buf += value;
      let i;
      while((i = buf.indexOf('\n')) >= 0){
        const line = buf.slice(0,i).trim();
        buf = buf.slice(i+1);
        if(line) handleLine(line);
      }
    }
  }
}

function handleLine(line){
  try{
    const obj = JSON.parse(line);

    if(typeof obj.t === 'number') tRealEl.textContent = obj.t.toFixed(2)+' °C';
    if(typeof obj.sp === 'number'){
      tSPEl.textContent = obj.sp.toFixed(2)+' °C';
      if(!inpSP.value) inpSP.value = obj.sp.toFixed(1);
    }
    if(typeof obj.pwmH === 'number') pwmHEl.textContent = obj.pwmH.toFixed(1)+' %';
    if(typeof obj.pwmF === 'number') pwmFEl.textContent = obj.pwmF.toFixed(1)+' %';
    if(typeof obj.kpF === 'number' && !inpKpF.value) inpKpF.value = obj.kpF.toFixed(1);
    if(typeof obj.kpV === 'number' && !inpKpV.value) inpKpV.value = obj.kpV.toFixed(1);

    if(obj.mode) setModeBadge(obj.mode);

    log('RX: '+line);
  }catch{
    log('RX(text): '+line);
  }
}

async function sendSetpoint(){
  if(!writer) return;
  const sp = parseFloat((inpSP.value||'').replace(',','.'));
  if(isNaN(sp) || sp<30 || sp>50){ alert('Setpoint fuera de rango [30.0–50.0] °C'); return; }
  const cmd = `S=${sp.toFixed(1)}\n`;
  await writer.write(cmd);
  log('TX: '+cmd.trim());
}

async function sendKpF(){
  if(!writer) return;
  const v = parseFloat((inpKpF.value||'').replace(',','.'));
  if(isNaN(v) || v<1 || v>10){ alert('KpF fuera de rango [1–10]'); return; }
  const cmd = `KPF=${v.toFixed(1)}\n`;
  await writer.write(cmd);
  log('TX: '+cmd.trim());
}

async function sendKpV(){
  if(!writer) return;
  const v = parseFloat((inpKpV.value||'').replace(',','.'));
  if(isNaN(v) || v<1 || v>10){ alert('KpV fuera de rango [1–10]'); return; }
  const cmd = `KPV=${v.toFixed(1)}\n`;
  await writer.write(cmd);
  log('TX: '+cmd.trim());
}

btnConnect.addEventListener('click', connect);
btnDisconnect.addEventListener('click', disconnect);
btnSetSP.addEventListener('click', sendSetpoint);
btnSetKpF.addEventListener('click', sendKpF);
btnSetKpV.addEventListener('click', sendKpV);

window.addEventListener('beforeunload', ()=>{ if(port) disconnect(); });
