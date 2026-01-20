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
  toggleMenu(); // cerrar menú al seleccionar

  // Cambiar fondo según la sección activa
  const body = document.body;
  body.className = ''; // resetear clases

  switch (id) {
    case 'dashboard':
      body.classList.add('body-dark-blue');
      break;
    case 'yaw':
      body.classList.add('body-dark-blue');
      break;
    case 'motor':
      body.classList.add('body-light');
      break;
    case 'motor2':
      body.classList.add('body-neon');
      break;
    case 'config':
      body.classList.add('body-neon');
      break;
    default:
      body.classList.add('body'); // fondo por defecto
  }
}


// Plugin personalizado para mostrar valores actuales en el título
const valueDisplayPlugin = {
  id: 'valueDisplay',
  beforeUpdate: function(chart) {
    const datasets = chart.data.datasets;
    
    if (datasets[0].data.length > 0) {
      const currentValue = datasets[0].data[datasets[0].data.length - 1];
      const setpointValue = datasets[1].data[datasets[1].data.length - 1];
      const label = datasets[0].label;
      
      // Actualizar el título con los valores actuales
      chart.options.plugins.title.text = [
        `${label}`,
        `Actual: ${currentValue?.toFixed(2) || '--'} | Setpoint: ${setpointValue?.toFixed(2) || '--'}`
      ];
    }
  }
};

// Registrar el plugin
Chart.register(valueDisplayPlugin);
// Crear los gráficos con datasets para valores y setpoints
const createChart = (canvasId, label, color, setpointColor, yMin, yMax) => {
  const ctx = document.getElementById(canvasId).getContext('2d');
  return new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [{
        label: label,
        data: [],
        borderColor: color,
        fill: false,
        pointRadius: 0,
        borderWidth: 2,
        tension: 0.2 
      }, {
        label: 'Setpoint',
        data: [],
        borderColor: setpointColor,
        borderDash: [5, 5], // Línea punteada
        fill: false,
        pointRadius: 0,
        borderWidth: 2,
        tension: 0
      }]
    },
    options: {
      animation: false,
      scales: {
        x: { display: false },
        y: { min: yMin, max: yMax }
      },
      plugins: {
        legend: {
          display: true,
          position: 'top'
        },
        title: {
          display: true,
          text: label,
          font: {
            size: 16,
            weight: 'bold'
          },
          color: '#ffffff'
        },
        valueDisplay: true // Activar nuestro plugin personalizado
      }
    },
    plugins: [valueDisplayPlugin] // Incluir el plugin
  });
};

const yawChart = createChart("yawChart", "Yaw", "blue", "orange", -180, 180);
const motorChart = createChart("motorChart", "RPM Motor", "green", "red", -90, 90);
const motor2Chart = createChart("motor2Chart", "RPM Motor 2", "purple", "gray", -90, 90);

// Gráficos para el dashboard
const dashYawChart = createChart("dashYawChart", "Yaw", "blue", "orange", -180, 180);
const dashMotorChart = createChart("dashMotorChart", "RPM Motor 1", "green", "red", -90, 90);
const dashMotor2Chart = createChart("dashMotor2Chart", "RPM Motor 2", "purple", "gray", -90, 90);

async function getData() {
  try {
    const res = await fetch("/data");
    const json = await res.json();
    const { yaw, motor_RPM, motor2_RPM, setpoint_servo, setpoint_motor, setpoint_motor2 } = json;

    document.getElementById("yawValue").innerText = yaw.toFixed(2);

    // Actualizar gráficos con valores y setpoints
    const chartsData = [
      [yawChart, yaw, setpoint_servo],
      [motorChart, motor_RPM, setpoint_motor],
      [motor2Chart, motor2_RPM, setpoint_motor2] // Motor2 con su setpoint
    ];

    // Actualizar también los gráficos del dashboard
    const dashboardChartsData = [
      [dashYawChart, yaw, setpoint_servo],
      [dashMotorChart, motor_RPM, setpoint_motor],
      [dashMotor2Chart, motor2_RPM, setpoint_motor2]
    ];

    // Actualizar todos los gráficos (individuales y dashboard)
    [...chartsData, ...dashboardChartsData].forEach(([chart, value, setpoint]) => {
      // Agregar datos al primer dataset (valor actual)
      chart.data.labels.push('');
      chart.data.datasets[0].data.push(value);
      
      // Agregar datos al segundo dataset (setpoint)
      chart.data.datasets[1].data.push(setpoint);
      
      // Mantener solo los últimos 100 puntos
      if (chart.data.labels.length > 100) {
        chart.data.labels.shift();
        chart.data.datasets[0].data.shift();
        chart.data.datasets[1].data.shift();
      }
      
      chart.update();
    });

    // Actualizar valores en los controles también
    if (document.getElementById("motor2Value")) {
      document.getElementById("motor2Value").innerText = motor2_RPM.toFixed(2);
    }

    // Actualizar valores del dashboard
    if (document.getElementById("dashYawValue")) {
      document.getElementById("dashYawValue").innerText = yaw.toFixed(2);
    }
    if (document.getElementById("dashMotorValue")) {
      document.getElementById("dashMotorValue").innerText = motor_RPM.toFixed(2);
    }
    if (document.getElementById("dashMotor2Value")) {
      document.getElementById("dashMotor2Value").innerText = motor2_RPM.toFixed(2);
    }

  } catch (e) {
    console.error("Error al obtener datos:", e);
  }
}
function onLED() {
  fetch('/led/on')
    .then(response => {
      if (!response.ok) throw new Error("No se pudo encender el LED");
      return response.text();
    })
    .then(text => console.log("Respuesta:", text))
    .catch(err => console.error("Error:", err));
}
function offLED() {
  fetch('/led/off')
    .then(response => {
      if (!response.ok) throw new Error("No se pudo apagar el LED");
      return response.text();
    })
    .then(text => console.log("Respuesta:", text))
    .catch(err => console.error("Error:", err));
}

function updatePID() {
  const kp = document.getElementById("kp").value;
  const ki = document.getElementById("ki").value;
  const kd = document.getElementById("kd").value;

  fetch(`/update_pid?kp=${kp}&ki=${ki}&kd=${kd}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar PID");
      return response.text();
    })
    .then(text => alert(text))
    .catch(err => console.error("Error:", err));
}

function updatePIDServo() {
  const kp = document.getElementById("kp_servo").value;
  const ki = document.getElementById("ki_servo").value;
  const kd = document.getElementById("kd_servo").value;

  fetch(`/update_pid_servo?kp=${kp}&ki=${ki}&kd=${kd}`)
    .then(response => {
      if (!response.ok) throw new Error("Error al actualizar PID Servo");
      return response.text();
    })
    .then(text => alert(text))
    .catch(err => console.error("Error:", err));
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





setInterval(getData, 50); // Reducido de 100ms a 50ms para actualización más rápida