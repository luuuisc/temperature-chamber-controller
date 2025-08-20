const btnConnect = document.getElementById('btnConnect');
const btnDisconnect = document.getElementById('btnDisconnect');
const btnSetSP = document.getElementById('btnSetSP');
const statusEl = document.getElementById('status');
const tRealEl = document.getElementById('tReal');
const tSPEl = document.getElementById('tSP');
const modeEl = document.getElementById('mode');
const inpSP = document.getElementById('inpSP');
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
  btnConnect.disabled=false; btnDisconnect.disabled=true; btnSetSP.disabled=true;
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
    if(typeof obj.sp === 'number'){ tSPEl.textContent = obj.sp.toFixed(2)+' °C'; if(!inpSP.value) inpSP.value = obj.sp.toFixed(1); }
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

btnConnect.addEventListener('click', connect);
btnDisconnect.addEventListener('click', disconnect);
btnSetSP.addEventListener('click', sendSetpoint);
window.addEventListener('beforeunload', ()=>{ if(port) disconnect(); });
