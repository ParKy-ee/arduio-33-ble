import re
import gzip
import os

with open('index2.html', 'r', encoding='utf-8') as f:
    html = f.read()

# ซ่อนส่วน Toolbar และ Timeline ด้วย CSS
css_hide = '''
  <style>
    .toolbar, .transport { display: none !important; }
    .canvas-wrap { margin-bottom: 20px; }
  </style>
</head>
'''
html = html.replace('</head>', css_hide)

live_script = '''
    // --- LIVE ESP32 INJECTION ---
    let liveInterval = 0;
    let liveQ = [1,0,0,0];
    let lastTime = 0;
    
    function startLive() {
        if(liveInterval) return;
        sourceLabel.textContent = "Live ESP32";
        $("sensorBadge").textContent = "Live Data";
        setMessage("กำลังเชื่อมต่อ...");
        
        liveInterval = setInterval(async () => {
            try {
                // เพิ่ม ?_t=... ป้องกัน Browser Cache ค่าเดิม
                let res = await fetch('/data?_t=' + Date.now());
                let data = await res.json();
                
                let accel = [data.ax, data.ay, data.az];
                let gyro = [data.gx, data.gy, data.gz];
                let t = data.t;
                
                // ป้องกันโค้ดเดิมพัง (Cannot read properties of undefined)
                if (typeof samples !== 'undefined') {
                    samples = [{t: t}];
                }
                
                if (lastTime === 0) {
                    liveQ = getInitialOrientation(accel, null);
                    lastTime = t;
                } else {
                    let dt = t - lastTime;
                    lastTime = t;
                    if(dt > 0 && dt < 1.0) {
                        let omega = gyro;
                        let error = [0, 0, 0];
                        let gain = 0;
                        const accelMagnitude = Math.hypot(accel[0], accel[1], accel[2]);
                        
                        if (accelMagnitude > 1e-8) {
                            const measuredGravity = vecNorm(accel);
                            const expectedGravity = [
                                2 * (liveQ[1] * liveQ[3] - liveQ[0] * liveQ[2]),
                                2 * (liveQ[0] * liveQ[1] + liveQ[2] * liveQ[3]),
                                liveQ[0] * liveQ[0] - liveQ[1] * liveQ[1] - liveQ[2] * liveQ[2] + liveQ[3] * liveQ[3]
                            ];
                            const accelError = cross(measuredGravity, expectedGravity);
                            const expectedMagnitude = accelMagnitude > 3 ? 9.80665 : 1;
                            const magnitudeTrust = clamp(1 - Math.abs(accelMagnitude - expectedMagnitude) / (expectedMagnitude * 0.65), 0.15, 1);
                            error = error.map((v, j) => v + accelError[j] * magnitudeTrust);
                            gain = 1.8; // AHRS gain
                        }
                        
                        omega = omega.map((v, j) => v + error[j] * gain);
                        let derivative = qMultiply(liveQ, [0, omega[0], omega[1], omega[2]]);
                        liveQ = qNormalize(liveQ.map((v, j) => v + derivative[j] * 0.5 * dt));
                    }
                }
                
                let frame = {
                    t: t,
                    q: liveQ,
                    accel: accel,
                    gyro: gyro,
                    mag: null
                };
                updateReadouts(frame);
                setMessage("เชื่อมต่อสำเร็จ รับข้อมูลแบบเรียลไทม์");
                liveStatus.textContent = "Live";
            } catch(e) {
                setMessage("ขาดการเชื่อมต่อ: " + e.message, true);
            }
        }, 80);
    }
    
    window.addEventListener('load', startLive);
    // --- END LIVE ESP32 INJECTION ---
'''

html = html.replace('</script>', live_script + '\n  </script>')
compressed = gzip.compress(html.encode('utf-8'))
array_str = ', '.join([f'0x{b:02x}' for b in compressed])

out = f'''#pragma once
#include <Arduino.h>
const uint8_t webpage_html_gz[] PROGMEM = {{ {array_str} }};
const size_t webpage_html_gz_len = {len(compressed)};
'''

with open('src/webpage.h', 'w', encoding='utf-8') as f:
    f.write(out)
