const TOPIC_CMD = "/dev_web_ui/cmd_vel";
      const TOPIC_CONNECT = "/dev_web_ui/connect";
      const SENDER_NAME = "dev_web_ui";
      const LIN_SPEED = 2, ANG_SPEED = 1.0, RATE_HZ = 20;
      const periodMs = Math.max(10, 1000 / RATE_HZ);
      const enableEl = document.getElementById('wasd_enable');
      const dcStatus = document.getElementById('dc_status');

      const keys = { w:0, a:0, s:0, d:0 };

      let wasdTimer = null;
      let lastKeepalive = 0;
window.keys = keys;
       function anyKeyDown(){ return keys.w || keys.a || keys.s || keys.d; }
      function computeCmd(){
        const vx = (keys.w - keys.s) * LIN_SPEED;
        const wz = (keys.a - keys.d) * ANG_SPEED;
        return { vx, vy: 0, wz };
      }
      function dc(){
        return (window.__dc && window.__dc.readyState === 'open') ? window.__dc : null;
      }
      function dcSend(obj){
        const ch = dc(); if (!ch) return;
        const str = (typeof obj === 'string') ? obj : JSON.stringify(obj);
        ch.send(str);
        const out = document.getElementById('last_sent'); if (out) out.textContent = str;
      }
      function twist(x=0, y=0, z=0){
        return { op:"publish", topic:TOPIC_CMD, type:"geometry_msgs/msg/Twist", msg:{ linear:{x,y,z:0}, angular:{x:0,y:0,z:z} } };
      }
      function msgConnect(connected){
        return { op:"publish", topic:TOPIC_CONNECT, type:"std_msgs/msg/Bool", msg:!!connected };
      }
      function tick(){
        const now = Date.now();
        if (now - lastKeepalive > 1000) {
          dcSend(msgConnect(true));
          lastKeepalive = now;
        }
        const { vx, vy, wz } = computeCmd();
        dcSend(twist(vx, vy, wz));
      }
      function startLoop(){ if (!wasdTimer) wasdTimer = setInterval(tick, periodMs); }
      function stopLoop(){
        if (!wasdTimer) return;
        clearInterval(wasdTimer); wasdTimer = null;
        dcSend(twist(0,0,0));
        dcSend(msgConnect(false));
      }
(function() {
 

  let clientList = document.getElementById('client_list');
  let callAllButton = document.getElementById('call_all_button');
  let hangUpAllButton = document.getElementById('hang_up_all_button');
  let closeAllRoomPeerConnectionsButton = document.getElementById('close_all_room_peer_connections');
  let idInput = document.getElementById('id_input');
  let callOneButton = document.getElementById('call_one_button');
  let remoteVideos = document.getElementById('remote_videos');

 
  callAllButton.disabled = true;
  hangUpAllButton.disabled = true;
  closeAllRoomPeerConnectionsButton.disabled = true;
  callOneButton.disabled = true;


  let streamDataChannelClient = null;

  function updateDcStatus(){
          dcStatus.textContent = (window.__dc ? window.__dc.readyState : 'none');
        }
   
const ROOM_PASSWORD = 'abc'; // used only internally

function initIdentityViewer() {
  const existing = sessionStorage.getItem('viewer_name');
  if (existing) {
    paintIdentity(existing);
    return { name: existing };
  }

  const key = 'viewer_counter';
  const current = parseInt(localStorage.getItem(key) || '0', 10);
  const next = current + 1;
  localStorage.setItem(key, String(next));

  const name = `viewer ${next}`;
  sessionStorage.setItem('viewer_name', name);
  paintIdentity(name);
  return { name };
}

function paintIdentity(name){
  const nameBadge = document.getElementById('name_badge');
  if (nameBadge) nameBadge.textContent = name;
}


  function connectStreamClientEvents() {
    streamDataChannelClient.onSignalingConnectionOpen = () => {
      connectButton.disabled = true;
      closeButton.disabled = false;
    };
    streamDataChannelClient.onSignalingConnectionClose = async () => {
      connectButton.disabled = false;
      closeButton.disabled = true;
      callAllButton.disabled = true;
      hangUpAllButton.disabled = true;
      closeAllRoomPeerConnectionsButton.disabled = true;
      callOneButton.disabled = true;
        stopLoop();
  enableEl.checked = false;
  updateDcStatus();
    };
    streamDataChannelClient.onSignalingConnectionError = message => {
      alert(message);
    }
    streamDataChannelClient.onRoomClientsChange = clients => {
      callAllButton.disabled = !(clients.length > 1 && hangUpAllButton.disabled);
      callOneButton.disabled = callAllButton.disabled;

      clientList.innerHTML = '';
      clients.forEach(client => {
        let li = document.createElement('li');
        li.textContent = client.name;
        li.style.color = client.isConnected ? 'green' : 'red';
        clientList.appendChild(li);
      });
    };

    streamDataChannelClient.onClientConnectionFail = (id, name, clientData) => {
      console.log('The connect with the client ' + name + '(' + id + ') failed.');
    }

    streamDataChannelClient.onAddRemoteStream = (id, name, clientData, stream) => {
      callAllButton.disabled = true;
      hangUpAllButton.disabled = false;
      closeAllRoomPeerConnectionsButton.disabled = false;
      callOneButton.disabled = true;

      let h5 = document.createElement("h5");;
      h5.innerHTML = name;
      

      let video = document.createElement("video");
      video.srcObject = stream;
      video.id = 'video' + id;
      video.autoplay = true;

      remoteVideos.appendChild(h5);
      remoteVideos.appendChild(video);
    }
    let onClientDisconnect = (id, name, clientData) => {
      callAllButton.disabled = streamDataChannelClient.isRtcConnected;
      hangUpAllButton.disabled = !streamDataChannelClient.isRtcConnected;
      closeAllRoomPeerConnectionsButton.disabled = !streamDataChannelClient.isRtcConnected;
      callOneButton.disabled = streamDataChannelClient.isRtcConnected;

      let h5 = document.getElementById('h5' + id);
      let video = document.getElementById('video' + id);
      if (h5 !== null) {
        remoteVideos.removeChild(document.getElementById('h5' + id));
          updateDcStatus();
          stopLoop();
      }
      if (video !== null) {
        remoteVideos.removeChild(document.getElementById('video' + id));
      }

    };
    streamDataChannelClient.onClientDisconnect = onClientDisconnect;

    streamDataChannelClient.onDataChannelOpen = (id, name, clientData) => {
      sendButton.disabled = false;
      callAllButton.disabled = true;
      hangUpAllButton.disabled = false;
      closeAllRoomPeerConnectionsButton.disabled = false;
      callOneButton.disabled = true;
      updateDcStatus();
      if (enableEl.checked) { lastKeepalive = 0; startLoop(); }
    }
    streamDataChannelClient.onDataChannelClose = onClientDisconnect;
   
  }


          (function hookAllDataChannels() {
          function attach(dc){
            window.__dc = dc;
            const _send = dc.send.bind(dc);
            dc.send = (data)=>{ try { JSON.parse(data); } catch {} _send(data); };
            dc.addEventListener('message', ev => {});
            dc.onopen  = updateDcStatus;
            dc.onclose = updateDcStatus;
            updateDcStatus();
          }
          const origCreate = RTCPeerConnection.prototype.createDataChannel;
          RTCPeerConnection.prototype.createDataChannel = function(label, opts){
            const dc = origCreate.call(this, label, opts);
            attach(dc);
            return dc;
          };
          const desc = Object.getOwnPropertyDescriptor(RTCPeerConnection.prototype, 'ondatachannel');
          Object.defineProperty(RTCPeerConnection.prototype, 'ondatachannel', {
            set(fn){ return desc.set.call(this, ev => { attach(ev.channel); fn && fn(ev); }); }
          });
        })();
   const { name: AUTO_NAME } = initIdentityViewer();
  async function connectNow() {
  const SignalingServerConfiguration = {
    url: 'ws://192.168.5.206:3001/signaling',
    name: AUTO_NAME,
    data: {},
    room: 'chat',
    password: ROOM_PASSWORD
  };
  const StreamConfiguration = { isSendOnly: false };
  const DataChannelConfiguration = {};
  const RtcConfiguration = {
    iceServers: [{ urls: "turn:192.168.5.206:3478?transport=udp", username: "razan", credential: "testrazan" }],
    iceTransportPolicy: "relay"
  };
  const logger = (...args) => console.log(...args);

  streamDataChannelClient = new window.openteraWebrtcWebClient.StreamDataChannelClient(
    SignalingServerConfiguration, StreamConfiguration, DataChannelConfiguration, RtcConfiguration, logger
  );
  connectStreamClientEvents();
  await streamDataChannelClient.connect();
}
if (document.readyState === 'complete' || document.readyState === 'interactive') {
  connectNow();
} else {
  window.addEventListener('DOMContentLoaded', connectNow);
}

  callAllButton.onclick = () => streamDataChannelClient.callAll();
 hangUpAllButton.onclick = () => {
  streamDataChannelClient.hangUpAll();
  stopLoop();
  enableEl.checked = false;
  updateDcStatus();
};

closeAllRoomPeerConnectionsButton.onclick = () => {
  streamDataChannelClient.closeAllRoomPeerConnections();
  stopLoop();
  enableEl.checked = false;
  updateDcStatus();
};
  callOneButton.onclick = () => streamDataChannelClient.callIds([idInput.value]);
  
          
        enableEl.addEventListener('change', () => {
          if (enableEl.checked) { lastKeepalive = 0; startLoop(); }
          else { stopLoop(); }
        });
        window.addEventListener('keydown', (e) => {
          if (!enableEl.checked) return;
          const tag = (e.target && e.target.tagName) ? e.target.tagName.toLowerCase() : "";
          if (tag === 'input' || tag === 'textarea' || e.target?.isContentEditable) return;
          const k = e.key.toLowerCase();
          if (!(k in keys) || e.repeat) return;
          keys[k] = 1; e.preventDefault();
          if (wasdTimer === null) startLoop();
        }, true);
        window.addEventListener('keyup', (e) => {
          if (!enableEl.checked) return;
          const k = e.key.toLowerCase();
          if (!(k in keys)) return;
          keys[k] = 0; e.preventDefault();
        }, true);
        window.addEventListener('beforeunload', () => {
          try { dcSend(twist(0,0,0)); dcSend(msgConnect(false)); } catch {}
        });
        window.pressKey = function pressKey(k){
          if (!document.getElementById('wasd_enable').checked) return;
          keys[k] = 1; tick();
          setTimeout(() => { keys[k] = 0; }, 120);
        };

})();
