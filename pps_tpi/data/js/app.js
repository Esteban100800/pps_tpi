/**********************
 *  MENÚ / UI
 **********************/
const robotImg = new Image();
robotImg.src = 'images/robot.png'; // Intenta primero en la raíz relativa

// Esperar a que la imagen cargue
let robotImageLoaded = false;
robotImg.onload = function() {
  robotImageLoaded = true;
  console.log("Imagen del robot cargada correctamente desde:", robotImg.src);
};

robotImg.onerror = function() {
  console.error("Error al cargar robot.png desde:", robotImg.src);
  console.log("Intenta estas rutas:");
  console.log("1. /robot.png");
  console.log("2. ./images/robot.png");
  console.log("3. Verifica que el archivo exista en el servidor");
};

function toggleMenu() {
  document.getElementById("menu").classList.toggle("show");
  document.body.classList.toggle("menu-open");
}

function toggleSubmenu(id) {
  document.getElementById(id).classList.toggle("show");
}

function showTab(id) {
  document.querySelectorAll('.tab').forEach(tab => tab.classList.remove('active'));
  document.getElementById(id).classList.add('active');
  toggleMenu();

  const body = document.body;
  body.className = '';

  switch (id) {
    case 'dashboard':
    case 'yaw':
      body.classList.add('body-dark-blue');
      break;
    case 'motor':
      body.classList.add('body-light');
      break;
    case 'motor2':
    case 'config':
      body.classList.add('body-neon');
      break;
    default:
      body.classList.add('body');
  }
}

// Plugin personalizado para mostrar valores actuales en el título
const valueDisplayPlugin = {
  id: 'valueDisplay',
  beforeUpdate: function(chart) {
    // Ejecutar solo si está habilitado y si es un gráfico de línea
    const enabled = chart?.options?.plugins?.valueDisplay === true;
    if (!enabled || chart.config.type !== 'line') return;

    const datasets = chart.data.datasets;
    if (!datasets || datasets.length < 2) return;

    const ds0 = datasets[0];
    const ds1 = datasets[1];
    if (!ds0?.data?.length || !ds1?.data?.length) return;

    const currentValue = Number(ds0.data[ds0.data.length - 1]);
    const setpointValue = Number(ds1.data[ds1.data.length - 1]);

    const currentTxt = Number.isFinite(currentValue) ? currentValue.toFixed(2) : '--';
    const setpointTxt = Number.isFinite(setpointValue) ? setpointValue.toFixed(2) : '--';

    chart.options.plugins.title.text = [
      `${ds0.label}`,
      `Actual: ${currentTxt} | Setpoint: ${setpointTxt}`
    ];
  }
};

// Registrar el plugin
Chart.register(valueDisplayPlugin);

/**********************
 *  CHART LINEALES
 **********************/
function createChart(canvasId, label, color, spColor, yMin, yMax) {
  const ctx = document.getElementById(canvasId).getContext('2d');
  return new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        {
          label,
          data: [],
          borderColor: 'red',
          pointRadius: 0,
          borderWidth: 2,
          tension: 0.2
        },
        {
          label: 'Setpoint',
          data: [],
          borderColor: 'spanishorange',
          borderDash: [5, 5],
          pointRadius: 0
        },
        {
          label: 'Error',
          data: [],
          borderColor: '#ff9800',
          borderDash: [],
          pointRadius: 0,
          hidden: true
        },
        {
          label: 'Integral',
          data: [],
          borderColor: '#4caf50',
          borderDash: [],
          pointRadius: 0,
          hidden: true
        },
        {
          label: 'Derivativa',
          data: [],
          borderColor: '#2196f3',
          borderDash: [],
          pointRadius: 0,
          hidden: true
        }
      ]
    },
    options: {
      animation: false,
      scales: {
        x: { display: false },
        y: { min: yMin, max: yMax }
      },
      plugins: {
        legend: { display: true },
        title: {
          display: true,
          text: label,
          color: '#333',
          font: { size: 16, weight: 'bold' }
        },
        valueDisplay: true // activar nuestro plugin solo en charts de línea
      }
    }
  });
}

/**********************
 *  CHART POSICIÓN
 **********************/
let positionChart;
let dashPositionChart;
let lastRealPos = { x: 0, y: 0 };

// Plugin para dibujar la imagen del robot rotada
const robotImagePlugin = {
  id: 'robotImage',
  afterDatasetsDraw(chart) {
    if (!robotImageLoaded) return; // No dibujar hasta que la imagen esté cargada
    
    const ctx = chart.ctx;
    const dataset = chart.data.datasets[1]; // Dataset del robot
    
    if (!dataset || !dataset.data || dataset.data.length === 0) return;
    
    const point = dataset.data[0];
    const xScale = chart.scales.x;
    const yScale = chart.scales.y;
    
    if (!point || point.x === undefined || point.y === undefined) return;
    
    const pixelX = xScale.getPixelForValue(point.x);
    const pixelY = yScale.getPixelForValue(point.y);
    const rotation = (point.r || 0) * Math.PI / 180; // Convertir a radianes
    const size = 40; // Tamaño de la imagen
    
    ctx.save();
    ctx.translate(pixelX, pixelY);
    ctx.rotate(rotation);
    ctx.drawImage(robotImg, -size / 2, -size / 2, size, size);
    ctx.restore();
  }
};

Chart.register(robotImagePlugin);

positionChart = new Chart(
  document.getElementById("positionChart").getContext("2d"),
  {
    type: 'scatter',
    data: {
      datasets: [
        {
          label: 'Trayectoria',
          data: [],
          showLine: true,
          borderColor: 'red',
          pointRadius: 0,
          borderWidth: 2
        },
        {
          label: 'Robot',
          data: [{ x: 0, y: 0, r: 0 }],
          pointRadius: 0 // No dibujar punto, usamos la imagen
        }
      ]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      plugins: {
        legend: { display: true },
        robotImage: true
      },
      scales: {
        x: {
          type: 'linear',
          min: -3,
          max: 3
        },
        y: {
          type: 'linear',
          min: -3,
          max: 3
        }
      }
    }
  }
);

dashPositionChart = new Chart(
  document.getElementById("dashPositionChart").getContext("2d"),
  {
    type: 'scatter',
    data: {
      datasets: [
        {
          label: 'Trayectoria',
          data: [],
          showLine: true,
          borderColor: 'red',
          pointRadius: 0,
          borderWidth: 2
        },
        {
          label: 'Robot',
          data: [{ x: 0, y: 0, r: 0 }],
          pointRadius: 0
        }
      ]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      spanGaps: false,
      plugins: {
        legend: { display: true },
        robotImage: true
      },
      scales: {
        x: { min: -3, max: 3, ticks: { stepSize: 1 } },
        y: { min: -3, max: 3, ticks: { stepSize: 1 } }
      }
    }
  }
);

/**********************
 *  INSTANCIAS
 **********************/
const yawChart = createChart("yawChart", "Yaw", "blue", "orange", -180, 180);
const motorChart = createChart("motorChart", "RPM Motor", "green", "red", -90, 90);
const motor2Chart = createChart("motor2Chart", "RPM Motor 2", "purple", "gray", -90, 90);

const dashYawChart = createChart("dashYawChart", "Yaw", "blue", "orange", -180, 180);
const dashMotorChart = createChart("dashMotorChart", "RPM Motor 1", "green", "red", -90, 90);
const dashMotor2Chart = createChart("dashMotor2Chart", "RPM Motor 2", "purple", "gray", -90, 90);

// Estado de pausa por gráfico
const pausedCharts = {
  yawChart: false,
  motorChart: false,
  motor2Chart: false,
  dashYawChart: false,
  dashMotorChart: false,
  dashMotor2Chart: false
};

function togglePauseChart(chartId, event) {
  pausedCharts[chartId] = !pausedCharts[chartId];
  const button = event.currentTarget;
  button.textContent = pausedCharts[chartId] ? '▶️' : '⏸️';
  button.style.backgroundColor = pausedCharts[chartId] ? 'rgba(255, 100, 100, 0.8)' : 'rgba(0, 0, 0, 0.6)';
}

/**********************
 *  DATA LOOP
 **********************/
async function getData() {
  try {
    const res = await fetch("/data");
    const data = await res.json();

    const {
      yaw, motor_RPM, motor2_RPM,
      setpoint_servo, setpoint_motor, setpoint_motor2,
      x, y,
      err_servo, err_motor, err_motor2,
      deriv_servo, deriv_motor, deriv_motor2,
      integ_servo, integ_motor, integ_motor2,
      rs_path
    } = data;

    // ---- INFO DEL PATH REEDS-SHEPP ----
    const rsPathEl = document.getElementById("rs_path_info");
    if (rsPathEl) {
      if (rs_path) {
        const lines = rs_path.split(";").map(seg => {
          const parts = seg.split(":");
          if (parts.length === 4) {
            const [i, type, gear, len] = parts;
            return `[${i}] ${type.padEnd(8)} ${gear}  len=${len}`;
          }
          return seg; // linea "total:x.xxx"
        });
        rsPathEl.textContent = lines.join("\n");
        rsPathEl.style.display = "block";
      } else {
        rsPathEl.style.display = "none";
      }
    }

    [
      [yawChart, yaw, setpoint_servo, err_servo, integ_servo, deriv_servo, 'yawChart'],
      [motorChart, motor_RPM, setpoint_motor, err_motor, integ_motor, deriv_motor, 'motorChart'],
      [motor2Chart, motor2_RPM, setpoint_motor2, err_motor2, integ_motor2, deriv_motor2, 'motor2Chart'],
      [dashYawChart, yaw, setpoint_servo, err_servo, integ_servo, deriv_servo, 'dashYawChart'],
      [dashMotorChart, motor_RPM, setpoint_motor, err_motor, integ_motor, deriv_motor, 'dashMotorChart'],
      [dashMotor2Chart, motor2_RPM, setpoint_motor2, err_motor2, integ_motor2, deriv_motor2, 'dashMotor2Chart']
    ].forEach(([chart, val, sp, err, integ, deriv, chartId]) => {
      if (pausedCharts[chartId]) return; // Saltar si está pausado

      chart.data.labels.push('');
      chart.data.datasets[0].data.push(Number(val));
      chart.data.datasets[1].data.push(Number(sp));
      const errVal = Number.isFinite(Number(err)) ? Number(err) : (Number(sp) - Number(val));
      chart.data.datasets[2].data.push(errVal);
      chart.data.datasets[3].data.push(Number.isFinite(Number(integ)) ? Number(integ) : 0);
      chart.data.datasets[4].data.push(Number.isFinite(Number(deriv)) ? Number(deriv) : 0);

      if (chart.data.labels.length > 100) {
        chart.data.labels.shift();
        chart.data.datasets.forEach(d => d.data.shift());
      }
      chart.update();
    });

    // ---- POSICIÓN - Actualizar AMBOS gráficos
    const xPos = parseFloat(x);
    const yPos = parseFloat(y);

    if (Number.isFinite(xPos) && Number.isFinite(yPos)) {
      const isReset = Math.abs(xPos) < 0.05 && Math.abs(yPos) < 0.05;

      if (!isReset) {
        lastRealPos = { x: xPos, y: yPos };
        positionChart.data.datasets[0].data.push({ x: xPos, y: yPos });
        dashPositionChart.data.datasets[0].data.push({ x: xPos, y: yPos });
      }

      const rotation = -yaw; // Sin restar nada - la imagen ya está orientada correctamente
      
      positionChart.data.datasets[1].data = [{ x: lastRealPos.x, y: lastRealPos.y, r: rotation }];
      dashPositionChart.data.datasets[1].data = [{ x: lastRealPos.x, y: lastRealPos.y, r: rotation }];

      if (positionChart.data.datasets[0].data.length > 300) {
        positionChart.data.datasets[0].data.shift();
      }
      if (dashPositionChart.data.datasets[0].data.length > 300) {
        dashPositionChart.data.datasets[0].data.shift();
      }

      positionChart.update('none');
      dashPositionChart.update('none');
    }

    // Actualizar coordenadas en pantalla (mostrar última posición real)
    if (document.getElementById("currentX")) {
      document.getElementById("currentX").innerText = lastRealPos.x.toFixed(2);
    }
    if (document.getElementById("currentY")) {
      document.getElementById("currentY").innerText = lastRealPos.y.toFixed(2);
    }

  } catch (e) {
    console.error("Error /data:", e);
  }
}

/**********************
 *  GOAL
 **********************/
function sendGoal() {
  const x = document.getElementById("goal_x").value;
  const y = document.getElementById("goal_y").value;
  const theta = document.getElementById("goal_theta").value;

  fetch(`/set_goal?x=${x}&y=${y}&theta=${theta}`)
    .then(res => res.text())
    .then(txt => console.log("Respuesta ESP32:", txt))
    .catch(err => console.error(err));
}

function sendGoalDash() {
  const x = document.getElementById("dash_goal_x").value;
  const y = document.getElementById("dash_goal_y").value;
  const theta = document.getElementById("dash_goal_theta").value;

  fetch(`/set_goal?x=${x}&y=${y}&theta=${theta}`)
    .then(res => res.text())
    .then(txt => console.log("Respuesta ESP32:", txt))
    .catch(err => console.error(err));
}

// "Enviar ahora": interrumpe el trayecto en curso y descarta la cola pendiente
function sendGoalNow() {
  const x = document.getElementById("goal_x").value;
  const y = document.getElementById("goal_y").value;
  const theta = document.getElementById("goal_theta").value;

  fetch(`/set_goal_now?x=${x}&y=${y}&theta=${theta}`)
    .then(res => res.text())
    .then(txt => console.log("Respuesta ESP32:", txt))
    .catch(err => console.error(err));
}

function sendGoalNowDash() {
  const x = document.getElementById("dash_goal_x").value;
  const y = document.getElementById("dash_goal_y").value;
  const theta = document.getElementById("dash_goal_theta").value;

  fetch(`/set_goal_now?x=${x}&y=${y}&theta=${theta}`)
    .then(res => res.text())
    .then(txt => console.log("Respuesta ESP32:", txt))
    .catch(err => console.error(err));
}



// Nuevas funciones para los controles específicos de cada gráfico
function updatePIDServoYaw() {
  const kp = document.getElementById("kp_servo_yaw").value;
  const ki = document.getElementById("ki_servo_yaw").value;
  const kd = document.getElementById("kd_servo_yaw").value;

  fetch(`/update_pid_servo?kp=${kp}&ki=${ki}&kd=${kd}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar PID Servo");
      return response.text();
    })
    .then(text => alert(text))
    .catch(err => console.error("Error:", err));
}

function updatePIDMotor() {
  const kp = document.getElementById("kp_motor").value;
  const ki = document.getElementById("ki_motor").value;
  const kd = document.getElementById("kd_motor").value;

  fetch(`/update_pid?kp=${kp}&ki=${ki}&kd=${kd}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar PID Motor");
      return response.text();
    })
    .then(text => alert(text))
    .catch(err => console.error("Error:", err));
}

function updateSetpointServo() {
  const setpoint = document.getElementById("setpoint_servo").value;
  
  fetch(`/update_setpoint_servo?setpoint=${setpoint}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar Setpoint Servo");
      return response.text();
    })
    .then(text => {
      alert(text);
      // Actualizar también el campo en la sección de configuración si existe
      const configField = document.getElementById("setpoint_servo_config");
      if (configField) configField.value = setpoint;
    })
    .catch(err => console.error("Error:", err));
}

function updateSetpointMotor() {
  const setpoint = document.getElementById("setpoint_motor").value;
  
  fetch(`/update_setpoint_motor?setpoint=${setpoint}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar Setpoint Motor");
      return response.text();
    })
    .then(text => {
      alert(text);
      // Actualizar también el campo en la sección de configuración si existe
      const configField = document.getElementById("setpoint_motor_config");
      if (configField) configField.value = setpoint;
    })
    .catch(err => console.error("Error:", err));
}

function updatePIDMotor2() {
  const kp = document.getElementById("kp_motor2").value;
  const ki = document.getElementById("ki_motor2").value;
  const kd = document.getElementById("kd_motor2").value;


  fetch(`/update_pid_motor2?kp=${kp}&ki=${ki}&kd=${kd}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar PID Motor2");
      return response.text();
    })
    .then(text => alert(text))
    .catch(err => console.error("Error:", err));
}

function updateSetpointMotor2() {
  const setpoint = document.getElementById("setpoint_motor2").value;
  
  fetch(`/update_setpoint_motor2?setpoint=${setpoint}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar Setpoint Motor2");
      return response.text();
    })
    .then(text => {
      alert(text);
      // Actualizar también el campo en la sección de configuración si existe
      const configField = document.getElementById("setpoint_motor2_config");
      if (configField) configField.value = setpoint;
    })
    .catch(err => console.error("Error:", err));
}

// Funciones para el dashboard
function updateSetpointServoDash() {
  const setpoint = document.getElementById("dash_setpoint_servo").value;
  
  fetch(`/update_setpoint_servo?setpoint=${setpoint}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar Setpoint Servo");
      return response.text();
    })
    .then(text => {
      alert("Setpoint Servo actualizado");
      // Sincronizar con otros campos
      const mainField = document.getElementById("setpoint_servo");
      if (mainField) mainField.value = setpoint;
    })
    .catch(err => console.error("Error:", err));
}

function updatePIDServoDash() {
  const kp = document.getElementById("dash_kp_servo").value;
  const ki = document.getElementById("dash_ki_servo").value;
  const kd = document.getElementById("dash_kd_servo").value;

  fetch(`/update_pid_servo?kp=${kp}&ki=${ki}&kd=${kd}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar PID Servo");
      return response.text();
    })
    .then(text => alert("PID Servo actualizado"))
    .catch(err => console.error("Error:", err));
}

function updateSetpointMotorDash() {
  const setpoint = document.getElementById("dash_setpoint_motor").value;
  
  fetch(`/update_setpoint_motor?setpoint=${setpoint}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar Setpoint Motor");
      return response.text();
    })
    .then(text => {
      alert("Setpoint Motor actualizado");
      // Sincronizar con otros campos
      const mainField = document.getElementById("setpoint_motor");
      if (mainField) mainField.value = setpoint;
    })
    .catch(err => console.error("Error:", err));
}

function updatePIDMotorDash() {
  const kp = document.getElementById("dash_kp_motor").value;
  const ki = document.getElementById("dash_ki_motor").value;
  const kd = document.getElementById("dash_kd_motor").value;

  fetch(`/update_pid?kp=${kp}&ki=${ki}&kd=${kd}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar PID Motor");
      return response.text();
    })
    .then(text => alert("PID Motor actualizado"))
    .catch(err => console.error("Error:", err));
}

function updateSetpointMotor2Dash() {
  const setpoint = document.getElementById("dash_setpoint_motor2").value;
  
  fetch(`/update_setpoint_motor2?setpoint=${setpoint}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar Setpoint Motor2");
      return response.text();
    })
    .then(text => {
      alert("Setpoint Motor2 actualizado");
      // Sincronizar con otros campos
      const mainField = document.getElementById("setpoint_motor2");
      if (mainField) mainField.value = setpoint;
    })
    .catch(err => console.error("Error:", err));
}

function updatePIDMotor2Dash() {
  const kp = document.getElementById("dash_kp_motor2").value;
  const ki = document.getElementById("dash_ki_motor2").value;
  const kd = document.getElementById("dash_kd_motor2").value;

  fetch(`/update_pid_motor2?kp=${kp}&ki=${ki}&kd=${kd}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar PID Motor2");
      return response.text();
    })
    .then(text => alert("PID Motor2 actualizado"))
    .catch(err => console.error("Error:", err));
}


function sendGoal() {
  const x = document.getElementById("goal_x").value;
  const y = document.getElementById("goal_y").value;
  const theta = document.getElementById("goal_theta").value;

  fetch(`/set_goal?x=${x}&y=${y}&theta=${theta}`)
    .then(res => res.text())
    .then(txt => console.log("Respuesta ESP32:", txt))
    .catch(err => console.error(err));
}


setInterval(getData, 120);

/**********************
 *  TOGGLE PID
 **********************/
function togglePID(event) {
  const button = event.currentTarget;
  const layout = button.closest('.layout');
  
  if (layout) {
    const wasHidden = layout.classList.contains('pid-hidden');
    layout.classList.toggle('pid-hidden');
    
    // Cambiar texto del botón
    button.textContent = layout.classList.contains('pid-hidden') ? '⚙️ Mostrar' : '⚙️ PID';
    
    // Forzar recálculo del tamaño sin setTimeout
    const canvas = layout.querySelector('canvas');
    if (canvas) {
      const chart = Chart.getChart(canvas);
      if (chart) {
        // Forzar actualización inmediata
        chart.resize();
        // Segunda actualización para asegurar
        requestAnimationFrame(() => {
          chart.resize();
        });
      }
    }
  }
}

/**********************
 *  NUEVAS FUNCIONES PID POSICIÓN
 **********************/
function updatePosPID() {
  const kp = document.getElementById("pos_kp").value;
  const ki = document.getElementById("pos_ki").value;
  const kd = document.getElementById("pos_kd").value;
  fetch(`/update_pid_pos?kp=${kp}&ki=${ki}&kd=${kd}`)
    .then(res => res.text())
    .then(txt => alert("PID Posición actualizado"))
    .catch(err => alert("Error al actualizar PID Posición"));
}

function updatePosPIDDash() {
  const kp = document.getElementById("dash_pos_kp").value;
  const ki = document.getElementById("dash_pos_ki").value;
  const kd = document.getElementById("dash_pos_kd").value;
  fetch(`/update_pid_pos?kp=${kp}&ki=${ki}&kd=${kd}`)
    .then(res => res.text())
    .then(txt => alert("PID Posición actualizado"))
    .catch(err => alert("Error al actualizar PID Posición"));
}

function sendGoalRow() {
  const x = document.getElementById("goal_x_row").value;
  const y = document.getElementById("goal_y_row").value;
  const theta = document.getElementById("goal_theta_row").value;
  fetch(`/set_goal?x=${x}&y=${y}&theta=${theta}`)
    .then(res => res.text())
    .then(txt => alert("Trayectoria enviada"))
    .catch(err => alert("Error al enviar trayectoria"));
}

function sendGoalDashRow() {
  const x = document.getElementById("dash_goal_x_row").value;
  const y = document.getElementById("dash_goal_y_row").value;
  const theta = document.getElementById("dash_goal_theta_row").value;
  fetch(`/set_goal?x=${x}&y=${y}&theta=${theta}`)
    .then(res => res.text())
    .then(txt => alert("Trayectoria enviada"))
    .catch(err => alert("Error al enviar trayectoria"));
}


/**********************
 *  TOGGLE ERROR
 **********************/
function toggleError(chartId, visible) {
  const canvas = document.getElementById(chartId);
  if (!canvas) return;
  const chart = Chart.getChart(canvas);
  if (!chart || !chart.data.datasets[2]) return;
  chart.data.datasets[2].hidden = !visible;
  chart.update('none');
}

function toggleIntegral(chartId, visible) {
  const canvas = document.getElementById(chartId);
  if (!canvas) return;
  const chart = Chart.getChart(canvas);
  if (!chart || !chart.data.datasets[3]) return;
  chart.data.datasets[3].hidden = !visible;
  chart.update('none');
}

function toggleDerivative(chartId, visible) {
  const canvas = document.getElementById(chartId);
  if (!canvas) return;
  const chart = Chart.getChart(canvas);
  if (!chart || !chart.data.datasets[4]) return;
  chart.data.datasets[4].hidden = !visible;
  chart.update('none');
}

/**********************
 *  ACTUALIZAR VELOCIDAD
 **********************/
function updateSpeed() {
  const speed = document.getElementById("speed_input").value;
  
  fetch(`/update_speed?speed=${speed}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar velocidad");
      return response.text();
    })
    .then(text => {
      alert("Velocidad actualizada correctamente");
      document.getElementById("current_speed").innerText = parseFloat(speed).toFixed(2);
    })
    .catch(err => {
      console.error("Error:", err);
      alert("Error al actualizar la velocidad");
    });
}