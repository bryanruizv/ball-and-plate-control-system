#include <Servo.h>
// --------- Pines de la pantalla t´actil (TU CONFIGURACI´ON) ----------
#define YP A1 // Y+ = P2
#define XM A2 // X- = P3
#define YM 7 // Y- = P4
#define XP 6 // X+ = P1
TouchScreen ts(XP, YP, XM, YM, 400);
#define TS_MINPRESSURE 5
#define TS_MAXPRESSURE 1000
// --------- Servos ----------
Servo servoX;
Servo servoY;
const int SERVO_X_PIN = 9;
const int SERVO_Y_PIN = 10;
// ´Angulos "planos" iniciales (aj´ustalos cuando montes la placa)
int servoX_center = 86;
int servoY_center = 86;
// M´aximo ´angulo de desviaci´on entorno al centro
const int SERVO_MAX_DELTA = 20; // ±20° de inclinaci´on m´axima
// ´Angulos actuales objetivo
double servoX_angle;
double servoY_angle;
// Posici´on ACTUAL del servo X (para barrido suave)
int servoX_actual = 90;
// --------- Calibraci´on t´actil ----------
int X_CENTER_RAW = 476; // p.x cuando la bola est´a en el centro
int Y_CENTER_RAW = 597; // p.y cuando la bola est´a en el centro
// ===================== PID EN X =====================
double KpX = 0.0095;
double KiX = 0.0001;
double KdX = 0.0009;
double errorX = 0;
double lastErrorX = 0;
double integralX = 0;
double derivativeX = 0;
double PID_X = 0;
// ===================== PID EN Y =====================
double KpY = 0.0095;
double KiY = 0.0001;
double KdY = 0.0009;
double errorY = 0;
double lastErrorY = 0;
double integralY = 0;
double derivativeY = 0;
double PID_Y = 0;
// Tiempo de muestreo (ms)
unsigned long lastTime = 0;
const unsigned long sampleTime = 20; // ~50 Hz
// Anti-windup simple (limitar integral)
const double I_MAX = 50.0;
const double I_MIN = -50.0;
void setup() {
Serial.begin(9600);
Serial.println("Ball & Plate con PID en X y Y");
servoX.attach(SERVO_X_PIN);
servoY.attach(SERVO_Y_PIN);
servoX_angle = servoX_center;
servoY_angle = servoY_center;
servoX.write(servoX_center);
servoY.write(servoY_center);
servoX_actual = servoX_center;
lastTime = millis();
delay(1000);
}
void loop() {
TSPoint p = ts.getPoint();
// Restaurar modos de los pines compartidos
pinMode(XM, OUTPUT);
pinMode(YP, OUTPUT);
// ¿Hay toque?
if (p.z > TS_MINPRESSURE && p.z < TS_MAXPRESSURE) {
// Debug
Serial.print("p.x = "); Serial.print(p.x);
Serial.print(" p.y = "); Serial.print(p.y);
Serial.print(" p.z = "); Serial.println(p.z);
unsigned long now = millis();
unsigned long dt_ms = now - lastTime;
if (dt_ms >= sampleTime) {
lastTime = now;
double dt = dt_ms / 1000.0; // en segundos
// ===================== ERRORES =====================
errorX = p.x - X_CENTER_RAW;
errorY = p.y - Y_CENTER_RAW;
// ===================== PID EJE X =====================
double P_X = KpX * errorX;
integralX += KiX * errorX * dt;
if (integralX > I_MAX) integralX = I_MAX;
if (integralX < I_MIN) integralX = I_MIN;
if (dt > 0) {
derivativeX = (errorX - lastErrorX) / dt;
} else {
derivativeX = 0;
}
double D_X = KdX * derivativeX;
PID_X = P_X + integralX + D_X;
lastErrorX = errorX;
// saturaci´on de la salida PID en ±SERVO_MAX_DELTA
if (PID_X > SERVO_MAX_DELTA) PID_X = SERVO_MAX_DELTA;
if (PID_X < -SERVO_MAX_DELTA) PID_X = -SERVO_MAX_DELTA;
servoX_angle = servoX_center + PID_X;
servoX_angle = constrain(servoX_angle, 0, 180);
// ===================== PID EJE Y =====================
double P_Y = KpY * errorY;
integralY += KiY * errorY * dt;
if (integralY > I_MAX) integralY = I_MAX;
if (integralY < I_MIN) integralY = I_MIN;
if (dt > 0) {
derivativeY = (errorY - lastErrorY) / dt;
} else {
derivativeY = 0;
}
double D_Y = KdY * derivativeY;
PID_Y = P_Y + integralY + D_Y;
lastErrorY = errorY;
if (PID_Y > SERVO_MAX_DELTA) PID_Y = SERVO_MAX_DELTA;
if (PID_Y < -SERVO_MAX_DELTA) PID_Y = -SERVO_MAX_DELTA;
servoY_angle = servoY_center + PID_Y;
servoY_angle = constrain(servoY_angle, 0, 180);
// ===================== APLICAR A LOS SERVOS =====================
// ---- Barrido suave para X ----
int targetX = (int)servoX_angle;
if (targetX > servoX_actual) {
for (int pos = servoX_actual; pos <= targetX; pos++) {
servoX.write(pos);
delay(5);
}
} else if (targetX < servoX_actual) {
for (int pos = servoX_actual; pos >= targetX; pos--) {
servoX.write(pos);
delay(5);
}
}
servoX_actual = targetX;
// ---- Servo Y normal ----
servoY.write((int)servoY_angle);
}
} else {
// ===================== SIN TOQUE: VOLVER A POSICI´ON INICIAL =====================
// Barrido suave para X de vuelta al centro
int targetX = servoX_center;
if (targetX > servoX_actual) {
for (int pos = servoX_actual; pos <= targetX; pos++) {
servoX.write(pos);
delay(5);
}
} else if (targetX < servoX_actual) {
for (int pos = servoX_actual; pos >= targetX; pos--) {
servoX.write(pos);
delay(5);
}
}
servoX_actual = targetX;
// Y vuelve directo al centro
servoY_angle = servoY_center;
servoY.write(servoY_center);
// Reset PID
errorX = lastErrorX = 0;
errorY = lastErrorY = 0;
integralX = integralY = 0;
derivativeX = derivativeY = 0;
}
delay(2);
}
