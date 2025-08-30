

## MODELO DE ACKERMANN

## Esquema del vehículo Ackermann

![Modelo Ackermann](/images/ackermann_model.png)



## Fórmulas de dirección del vehículo

Parámetros:

- **L**: distancia entre ejes  
- **w**: ancho de vía 
- **R**: radio de giro  
- **δᵢ**: ángulo de dirección de la rueda interior  
- **δₒ**: ángulo de dirección de la rueda exterior  
- **δ**: ángulo de dirección promedio  
- **Vg**: velocidad global


---

### Ángulo de la rueda interior

$$
\tan(\delta_i) = \frac{L}{R - \frac{w}{2}}
$$

Este angulo se obtiene al dividir el cateto opuesto sobre el cateto adyacente del diagrama de triangulos del vehiculo, su resultado es el largo entre los ejes de chasis sobre el radio formado por el centro de las ruedas traseras menos el ancho del chasis dividido entre dos.

### Ángulo de la rueda exterior

$$
\tan(\delta_o) = \frac{L}{R + \frac{w}{2}}
$$

Similar al angulo de rueda interior, la rueda exterior tiene una tangente de su angulo como la relacion del largo entre ejes sobre el radio formado por el centro del vehiculo menos el ancho mas dos (parte mas externa).



### Ángulo promedio de dirección

$$
\tan(\delta) = \frac{L}{R}
$$

Esta es la relacion entre el largo del vehiculo y el radio formado al mover el servo (δ) grados




## Dinámica de ruedas (cinemática de Ackermann)

### Velocidades angulares de las ruedas en función del radio de giro

Sea $\dot{\theta}$  la velocidad angular del vehículo, $r_t$  el radio de las ruedas, y $ V_G $ la velocidad del centro del vehículo:

$$
\begin{cases}
(R + \frac{w}{2}) \dot{\theta}  = r_t \omega_o \\
(R - \frac{w}{2}) \dot{\theta} = r_t \omega_i
\end{cases}
$$

Esta igualdad se puede aplicar solamente cuando el coeficiente de deslizamiento del vehiculo es infima o inexistente.


---

### Velocidad del vehículo

$$
2R \dot{\theta} = 2V_G = r_t \omega_o + r_t \omega_i \Rightarrow V_G = \frac{r_t \omega_o + r_t \omega_i}{2}
$$

Al tener un cuerpo rigido, la velocidad general del vehiculo (global) es el promedio de las velocidades tangenciales de las ruedas anexadas a la parte trasera del chasis.

---

### Relación entre velocidad lateral y velocidades angulares

$$
\begin{cases}
\frac{w V_G}{R} = r_t (\omega_o - \omega_i) \\
2V_G = r_t \omega_o + r_t \omega_i
\end{cases}
$$

---

### Cálculo explícito de velocidades angulares
Resolviendo las ecuaciones anteriores para $w_o$ y $w_i$, se obtiene las distintas velocidades angulares para cada una de las ruedas

$$
\begin{aligned}
\omega_o &= \frac{V_G \left(2 + \frac{w}{R} \right)}{2r_t} \\
\omega_i &= \frac{V_G \left(2 - \frac{w}{R} \right)}{2r_t}
\end{aligned}
$$

Esto es de suma importancia, ya que nos permite calcular la velocidad angular de cada una de las ruedas, medida con un encoder y controlada por un controlador PID.


## DFK Equations – Robot Kinetics

### Cinemática del centro del vehículo

$$
\begin{aligned}
\dot{X} &= V_G \cos(\theta) \\
\dot{Y} &= V_G \sin(\theta) \\
\dot{\theta} &= \frac{V_G}{R} = \frac{V_G}{\frac{L}{\tan(\delta)}} = \frac{V_G \tan(\delta)}{L}
\end{aligned}
$$


---




### Supuesto: dirección constante

Si

$$
\tan(\delta) = \frac{L}{R} = \text{constante}
$$

entonces

$$
\dot{\theta} = \frac{V_G}{R} = \text{constante} \Rightarrow \theta = \frac{V_G}{R} t + \theta_0
$$

---



### Tiempo para una vuelta completa

$$
T = \frac{2\pi}{\omega} = \frac{2\pi}{\frac{V_G}{R}} = \frac{2\pi R}{V_G}
$$

Este calculo puede ser util para corroborar el tiempo utilizado para realizar un giro o una fraccion de este.


### Relacion entre el angulo de giro del servo delantero y la velocidad angular de las ruedas traseras:

$$
\tan(\delta) = \frac{L}{R}  (1) \\
$$

$$
\begin{aligned}
\omega_o &= \frac{V_G \left(2 + \frac{w}{R} \right)}{2r_t} (2)\\
\omega_i &= \frac{V_G \left(2 - \frac{w}{R} \right)}{2r_t} (3)
\end{aligned}
$$


Despejando de la ecuacion (1) en radio de giro del vehiculo(con respecto al centro del mismo) nos permite establecer una relacion entre el angulo de giro y la velocidad angular de cada una de las ruedas anexadas al sector trasero del chasis del automovil.


$$
{R}  \tan(\delta) = {L}   \\
$$


$$
{R} = \frac{L} {\tan(\delta) } (4)  \\
$$


Reemplazando (4) en (2) y (3), obtenemos la relacion entre el angulo de giro y la velocidad angular de cada rueda.


$$
\begin{aligned}
\omega_o &= \frac{V_G \left(2 + \frac{w}{\frac{L} {\tan(\delta) }} \right)}{2r_t} \\ \\
\omega_i &= \frac{V_G \left(2 - \frac{w}{\frac{L} {\tan(\delta) }} \right)}{2r_t}
\end{aligned}
$$

Reacomodando:


$$
\begin{aligned}
\omega_o &= \frac{V_G \left(2 + \frac{w {(\tan(\delta)) }}{{L} } \right)}{2r_t} \\ \\
\omega_i &= \frac{V_G \left(2 - \frac{w {(\tan(\delta)) }}{{L} } \right)}{2r_t} 
\end{aligned}
$$


### Simulacion:

con el software de octave vamos a simular el comportamiento del vehiculo y estas tres variables.

Vale destacar que el modelo es no lineal, ya que posee componentes no lineales como la tangente del "steering angle" del automovil. quizas puede ser costoso a simple vista determinar el comportamiento de las ruedas.

El resultado que se espera es el siguiente:

- Al aumentar el angulo de giro del vehiculo, hacia la izquierda el radio de giro deberia disminuir.

- La rueda exterior trasera deberia tener un $wo$ mayor.


Codigo en Octave:

``` matlab
% Ackermann simulacion 
% En esta simulacion se pretende visualizar el comportamiento de las velocidades angulares
% de las ruedas traseras (interna y externa).
%
clear; clc; close all;

%% Parámetros del vehículo
%% Se declaran los valores constantes.
v   = 2;      % velocidad longitudinal [m/s]
L   = 2.5;    % distancia entre ejes [m]
Tsim= 30;     % tiempo de simulación [s]
dt  = 0.01;   % paso de integración [s]
w   = 1;      % distancia entre ruedas traseras [m] (track width)
rt  = 0.75;   % radio de ruedas traseras [m]


%% Discretización (exacta con paso dt)
N = floor(Tsim/dt) + 1;
t = (0:N-1)*dt;

%% Inicialización de las variables a graficar
Xpos  = zeros(1,N);
Ypos  = zeros(1,N);
theta = zeros(1,N);
delta = zeros(1,N);   % ángulo de giro
w_o   = zeros(1,N);   % vel. ang. rueda externa [rad/s]
w_i   = zeros(1,N);   % vel. ang. rueda interna [rad/s]

% Condiciones iniciales
Xpos(1) = 0;  Ypos(1) = 0;  theta(1) = 0;

%% Simulación (Euler explícito)
%% Para poder visualizar la posicion del vehiculo se debe integrar por el metodo de euler
%% Este metodo es util con pasos (dt) pequeños.
for k = 1:N-1
    % steering angle: de 0 a pi/4 en 30 s
    delta(k) = (pi/4)*(t(k)/Tsim);

    % velocidades angulares ruedas traseras (cinemática diferencial)
    w_o(k) = (v/rt) * ((2*L + w*tan(delta(k)))/(2*L));
    w_i(k) = (v/rt) * ((2*L - w*tan(delta(k)))/(2*L));

    % modelo cinemático
    dx     = v*cos(theta(k));
    dy     = v*sin(theta(k));
    dtheta = (v/L)*tan(delta(k));

    % integración Euler
    Xpos(k+1)  = Xpos(k)  + dx*dt;
    Ypos(k+1)  = Ypos(k)  + dy*dt;
    theta(k+1) = theta(k) + dtheta*dt;
end
delta(end) = pi/4;               % último valor de delta
w_o(end)   = w_o(end-1);
w_i(end)   = w_i(end-1);

%% Graficar trayectoria
figure;
plot(Xpos,Ypos,'LineWidth',2);
xlabel('X [m]'); ylabel('Y [m]');
title('Trayectoria vehículo (Ackermann, Euler)');
grid on; axis equal;

%% Graficar ángulo de giro
figure;
plot(t, rad2deg(delta), 'LineWidth', 2);
xlabel('Tiempo [s]');
ylabel('Ángulo de giro \delta [°]');
title('Ángulo de giro en función del tiempo');
grid on;

%% Graficar delta y velocidades juntas (doble eje Y)
if exist('yyaxis','file')   % MATLAB
    figure;
    yyaxis left
    plot(t, rad2deg(delta), 'LineWidth', 2);
    ylabel('\delta [°]');

    yyaxis right
    hold on;
    plot(t, w_o, 'LineWidth', 2);
    plot(t, w_i, 'LineWidth', 2);
    ylabel('\omega ruedas [rad/s]');

    xlabel('Tiempo [s]');
    title('\delta y velocidades angulares de ruedas');
    legend('\delta', '\omega_{ext}', '\omega_{int}');
    grid on;

else                        % Octave: usar plotyy
    figure;
    [ax, h1, h2] = plotyy(t, rad2deg(delta), t, w_o);
    hold(ax(2), 'on');
    h3 = plot(ax(2), t, w_i, 'LineWidth', 2);

    set(h1, 'LineWidth', 2);
    set(h2, 'LineWidth', 2);
    set(h3, 'LineWidth', 2);

    xlabel('Tiempo [s]');
    ylabel(ax(1), '\delta [°]');
    ylabel(ax(2), '\omega ruedas [rad/s]');
    title('\delta y velocidades angulares de ruedas');
    legend([h1; h2; h3], '\delta', '\omega_{ext}', '\omega_{int}');
    grid(ax(1), 'on'); grid(ax(2), 'on');
end

```


### Resultados (graficas):

#### Posicion:

![Posicion del vehiculo](/images/pos.png)

Se visualiza en cambio de posicion a medida que el angulo de giro de la direccion delantera cambia entre 0º y 45º.

#### Steering angle:

![Angulo de la direccion delantera](/images/ang.png)

Se observa como el angulo es incrementado de manera lineal entre [0 -  $\frac{pi}{4}$] o 0º y 90º.


#### Velocidad de cada rueda:

![Velocidad angular de cada rueda](/images/vel.png)

Se observa como la velocidad angular de la rueda exterior tiene un comportamiento creciente, y que la rueda interior tiende a cero.

Esto tiene sentido. Al analizarlo en el caso limite de estudio (maximo giro del vehiculo), la rueda interior funcionaria como un  punto de apoyo para que la rueda exterior gire al maximo establecido por la mecanica y la electronica disponible.

