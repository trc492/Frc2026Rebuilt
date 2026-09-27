(() => {
  const controls = { a:0, b:1, x:2, y:3, lb:4, rb:5, lt:6, rt:7, back:8, start:9, dpadUp:12, dpadDown:13, dpadLeft:14, dpadRight:15 };
  const slots = {
    driver:{ select:document.querySelector('#driverSelect'), state:document.querySelector('#driverState'), fallback:0 },
    operator:{ select:document.querySelector('#operatorSelect'), state:document.querySelector('#operatorState'), fallback:1 }
  };
  let deviceSignature = '';

  function buttonDown(pad, index) {
    const button = pad.buttons[index];
    if (!button) return false;
    if (index === 6 && pad.axes.length >= 6) return button.pressed || button.value > .18 || pad.axes[4] > .18;
    if (index === 7 && pad.axes.length >= 6) return button.pressed || button.value > .18 || pad.axes[5] > .18;
    return button.pressed || button.value > .5;
  }
  function updateMenus(pads) {
    const signature = pads.filter(Boolean).map(p => `${p.index}:${p.id}`).join('|');
    if (signature === deviceSignature) return;
    deviceSignature = signature;
    for (const [name, slot] of Object.entries(slots)) {
      const old = slot.select.value;
      slot.select.replaceChildren(new Option(`Auto - ${name === 'driver' ? 'first' : 'second'} controller`, ''));
      pads.filter(Boolean).forEach(p => slot.select.add(new Option(`${p.index}: ${p.id || 'Gamepad'}`, String(p.index))));
      if ([...slot.select.options].some(o => o.value === old)) slot.select.value = old;
    }
  }
  function selectPad(pads, name, slot) {
    if (slot.select.value !== '') return pads[Number(slot.select.value)] || null;
    const connected = pads.filter(Boolean);
    return connected.find(p => p.index === slot.fallback) || connected[name === 'driver' ? 0 : 1] || null;
  }
  function render() {
    const pads = Array.from(navigator.getGamepads ? navigator.getGamepads() : []);
    updateMenus(pads);
    let connected = false;
    for (const [name, slot] of Object.entries(slots)) {
      const pad = selectPad(pads, name, slot);
      const card = document.querySelector(`[data-slot="${name}"]`);
      card.querySelectorAll('[data-input]').forEach(el => el.classList.remove('active'));
      card.classList.remove('alt-active');
      if (!pad) { slot.state.textContent = 'Waiting for controller'; continue; }
      connected = true;
      for (const [key, index] of Object.entries(controls)) {
        if (buttonDown(pad, index)) card.querySelectorAll(`[data-input="${key}"]`).forEach(el => el.classList.add('active'));
      }
      const alt = buttonDown(pad, controls.lb);
      card.classList.toggle('alt-active', alt);
      for (const type of ['left', 'right']) {
        const axes = type === 'left' ? [0,1] : [2,3];
        const x = Math.max(-1, Math.min(1, pad.axes[axes[0]] || 0));
        const y = Math.max(-1, Math.min(1, pad.axes[axes[1]] || 0));
        const stick = card.querySelector(`[data-stick="${type}"]`);
        stick.classList.toggle('active', Math.hypot(x,y) > .18);
        stick.querySelector('.cap').style.transform = `translate(calc(-50% + ${x*23}px), calc(-50% + ${y*23}px))`;
      }
      slot.state.textContent = `Connected - ${pad.id || `Gamepad ${pad.index}`}`;
    }
    document.querySelector('#connection').classList.toggle('connected', connected);
    document.querySelector('#connection span').textContent = connected ? 'Live input connected' : 'Connect controllers to show live input';
    requestAnimationFrame(render);
  }
  window.addEventListener('gamepadconnected', render);
  window.addEventListener('gamepaddisconnected', render);
  render();
})();
