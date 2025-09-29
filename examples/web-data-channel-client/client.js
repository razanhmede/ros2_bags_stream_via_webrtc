(function () {
  'use strict';

  // ---- DOM ----
  const nameInput = document.getElementById('name_input');
  const passwordInput = document.getElementById('password_input');
  const connectButton = document.getElementById('connect_button');
  const closeButton = document.getElementById('close_button');
  const clientList = document.getElementById('client_list');
  const callAllButton = document.getElementById('call_all_button');
  const hangUpAllButton = document.getElementById('hang_up_all_button');
  const closeAllRoomPeerConnectionsButton = document.getElementById('close_all_room_peer_connections');
  const idInput = document.getElementById('id_input');
  const callOneButton = document.getElementById('call_one_button');
  const textInput = document.getElementById('text_input');
  const sendButton = document.getElementById('send_button');
  const chatTextArea = document.getElementById('chat_text_area');

  // ---- Initial UI state ----
  closeButton.disabled = true;
  callAllButton.disabled = true;
  hangUpAllButton.disabled = true;
  closeAllRoomPeerConnectionsButton.disabled = true;
  callOneButton.disabled = true;
  sendButton.disabled = true;
  chatTextArea.value = '';

  // ---- Bridge hint labels ----
  document.getElementById('bridge_hint_cmdvel').textContent  = window.BRIDGE_CMDVEL;
  document.getElementById('bridge_hint_active').textContent  = window.BRIDGE_ACTIVE;
  document.getElementById('bridge_hint_connect').textContent = window.BRIDGE_CONNECT;
  const bridgeStatusEl = document.getElementById('bridge_status');

  // ---- HTTP→ROS helpers ----
  async function bridgePost(url, payload){
    try {
      const res = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });
      if (!res.ok) throw new Error('HTTP ' + res.status);
      bridgeStatusEl.textContent = 'ok';
    } catch (e) {
      bridgeStatusEl.textContent = 'error';
      console.warn('Bridge POST failed:', url, e);
    }
  }
  function bridgeSendCmdVel(vx, vy, wz){ return bridgePost(window.BRIDGE_CMDVEL, { vx, vy, wz }); }
  function bridgeSendActive(active, sender){ return bridgePost(window.BRIDGE_ACTIVE, { active: !!active, sender: sender || 'web_wasd' }); }
  function bridgeSendConnect(connected){ return bridgePost(window.BRIDGE_CONNECT, { data: !!connected }); }

  // ---- WASD loop ----
  const TOPIC_CMD     = "/cockpit/cmd_vel";
  const TOPIC_ACTIVE  = "/navigation/active_sender";
  const LIN_SPEED = 0.5, ANG_SPEED = 1.0, RATE_HZ = 20;
  const keys = { w:0, a:0, s:0, d:0 };
  let timer = null;
  const periodMs = Math.max(10, 1000 / RATE_HZ);

  function anyKeyDown(){ return keys.w || keys.a || keys.s || keys.d; }
  function computeCmd(){
    const vx = (keys.w - keys.s) * LIN_SPEED;
    const wz = (keys.a - keys.d) * ANG_SPEED;
    return { vx, vy: 0, wz };
  }
  function dc() {
    return (window.__dc && window.__dc.readyState === 'open') ? window.__dc : null;
  }
  function dcSend(obj){
    const ch = dc(); if (!ch) { return; }
    const str = typeof obj === 'string' ? obj : JSON.stringify(obj);
    ch.send(str);
    const out = document.getElementById('last_sent'); if (out) out.textContent = str;
  }
  function twist(linearX=0, linearY=0, angularZ=0){
    return {
      op: "publish",
      topic: TOPIC_CMD,
      type: "geometry_msgs/msg/Twist",
      msg: { linear:  { x: linearX, y: linearY, z: 0 }, angular: { x: 0, y: 0, z: angularZ } }
    };
  }
  function setActiveSender(active){
    bridgeSendActive(active, 'web_wasd');
    dcSend({ op: "publish", topic: TOPIC_ACTIVE, msg: { active: !!active, sender: 'web_wasd' } });
  }
  function tick(){
    const { vx, vy, wz } = computeCmd();
    bridgeSendCmdVel(vx, vy, wz);
    dcSend(twist(vx, vy, wz));
  }
  function startLoop(){ if (!timer) timer = setInterval(tick, periodMs); }
  function stopLoop(){
    if (!timer) return;
    clearInterval(timer); timer = null;
    bridgeSendCmdVel(0,0,0);
  }

  const enableEl = document.getElementById('wasd_enable');
  enableEl.addEventListener('change', () => {
    const on = enableEl.checked;
    if (on) {
      bridgeSendConnect(true);
      setActiveSender(true);
    } else {
      setActiveSender(false);
      bridgeSendConnect(false);
      stopLoop();
    }
  });
  window.addEventListener('keydown', (e) => {
    if (!enableEl.checked) return;
    const tag = (e.target && e.target.tagName) ? e.target.tagName.toLowerCase() : "";
    if (tag === 'input' || tag === 'textarea' || e.target?.isContentEditable) return;
    const k = e.key.toLowerCase();
    if (!(k in keys) || (e.repeat)) return;
    keys[k] = 1; e.preventDefault();
    if (anyKeyDown()) startLoop();
  }, true);
  window.addEventListener('keyup', (e) => {
    if (!enableEl.checked) return;
    const k = e.key.toLowerCase();
    if (!(k in keys)) return;
    keys[k] = 0; e.preventDefault();
    if (!anyKeyDown()) stopLoop();
  }, true);
  window.addEventListener('blur', () => { if (anyKeyDown()) stopLoop(); });
  window.addEventListener('beforeunload', () => {
    try { bridgeSendCmdVel(0,0,0); setActiveSender(false); bridgeSendConnect(false); } catch {}
  });

  // ---- OpenTera client (loads ICE from local file) ----
  const ICE_SERVERS_URL = './iceServers.json'; // same origin
  let dataChannelClient = null;

  async function loadIceServers() {
    try {
      const res = await fetch(ICE_SERVERS_URL, { cache: 'no-store' });
      if (!res.ok) throw new Error(`HTTP ${res.status} ${res.statusText}`);
      const iceServers = await res.json();
      if (!Array.isArray(iceServers)) throw new Error('iceServers.json must be a JSON array.');
      return iceServers;
    } catch (err) {
      console.error('Failed to load ICE servers:', err);
      alert('Failed to load iceServers.json. Check that it is being served and contains valid JSON.\n' + err.message);
      return []; // fallback
    }
  }

  function connectDataChannelClientEvents() {
    dataChannelClient.onSignalingConnectionOpen = () => {
      connectButton.disabled = true;
      closeButton.disabled = false;
    };
    dataChannelClient.onSignalingConnectionClose = async () => {
      connectButton.disabled = false;
      closeButton.disabled = true;
      callAllButton.disabled = true;
      hangUpAllButton.disabled = true;
      closeAllRoomPeerConnectionsButton.disabled = true;
      callOneButton.disabled = true;
    };
    dataChannelClient.onSignalingConnectionError = (message) => { alert(message); };
    dataChannelClient.onRoomClientsChange = (clients) => {
      callAllButton.disabled = !(clients.length > 1 && hangUpAllButton.disabled);
      callOneButton.disabled = callAllButton.disabled;

      clientList.innerHTML = '';
      clients.forEach((client) => {
        const li = document.createElement('li');
        li.textContent = client.id + ' - ' + client.name;
        li.style.color = client.isConnected ? 'green' : 'red';
        clientList.appendChild(li);
      });
    };
    dataChannelClient.callAcceptor = async (id, name) => confirm('Do you accept the call from ' + name + '?');
    dataChannelClient.onCallReject = (id, name) => { alert('The call is rejected (' + name + ')'); };
    dataChannelClient.onClientConnectionFail = (id, name) => { console.log('Connect to ' + name + ' (' + id + ') failed.'); };
    dataChannelClient.onDataChannelOpen = () => {
      sendButton.disabled = false;
      callAllButton.disabled = true;
      hangUpAllButton.disabled = false;
      closeAllRoomPeerConnectionsButton.disabled = false;
      callOneButton.disabled = true;
    };
    dataChannelClient.onDataChannelClose = () => {
      sendButton.disabled = !dataChannelClient.isRtcConnected;
      callAllButton.disabled = dataChannelClient.isRtcConnected;
      hangUpAllButton.disabled = !dataChannelClient.isRtcConnected;
      closeAllRoomPeerConnectionsButton.disabled = !dataChannelClient.isRtcConnected;
      callOneButton.disabled = dataChannelClient.isRtcConnected;
    };
    dataChannelClient.onDataChannelMessage = (id, name, _data, message) => {
      chatTextArea.value += `${id} - ${name}: ${message}\n`;
      chatTextArea.scrollTop = chatTextArea.scrollHeight;
    };
  }

  connectButton.onclick = async () => {
    if (!window.openteraWebrtcWebClient) {
      alert('OpenTera library not loaded. Check <script src="openteraWebrtcWebClient.js"> path.');
      return;
    }

    const iceServers = await loadIceServers();

    const SignalingServerConfiguration = {
      url: window.SIGNALING_URL, // e.g. ws://HOST:8081/signaling
      name: nameInput.value || 'web',
      data: {},          // Client custom data
      room: 'chat',
      password: passwordInput.value || ''
    };
    const DataChannelConfiguration = {}; // RTCDataChannelInit
    const RtcConfiguration = { iceServers }; // RTCPeerConnection config
    const logger = (...args) => console.log('[webrtc]', ...args);

    dataChannelClient = new window.openteraWebrtcWebClient.DataChannelClient(
      SignalingServerConfiguration,
      DataChannelConfiguration,
      RtcConfiguration,
      logger
    );

    connectDataChannelClientEvents();

    try { await dataChannelClient.connect(); }
    catch (err) {
      console.error('Failed to connect:', err);
      alert('Failed to connect to signaling server.\n' + err.message);
    }
  };

  closeButton.onclick = () => {
    try { dataChannelClient?.close(); } finally { clientList.innerHTML = ''; }
  };
  callAllButton.onclick = () => dataChannelClient?.callAll && dataChannelClient.callAll();
  hangUpAllButton.onclick = () => dataChannelClient?.hangUpAll && dataChannelClient.hangUpAll();
  closeAllRoomPeerConnectionsButton.onclick = () => dataChannelClient?.closeAllRoomPeerConnections && dataChannelClient.closeAllRoomPeerConnections();
  callOneButton.onclick = () => { const id = (idInput.value || '').trim(); if (id && dataChannelClient?.callIds) dataChannelClient.callIds([id]); };
  sendButton.onclick = () => {
    chatTextArea.value += 'Me: ' + textInput.value + '\n';
    chatTextArea.scrollTop = chatTextArea.scrollHeight;
    if (dataChannelClient?.sendToAll) dataChannelClient.sendToAll(textInput.value);
    else if (dc()) dc().send(textInput.value);
  };

  // expose a couple helpers for inline onclicks in HTML
  window.anyKeyDown = anyKeyDown;
  window.tick = tick;
})();
